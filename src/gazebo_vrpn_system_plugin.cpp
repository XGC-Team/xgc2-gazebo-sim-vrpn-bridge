#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <xgc2_gazebo_scene/native_component_binding.hpp>
#include <xgc2_gazebo_scene/world_startup_binding.hpp>
#include "gazebo_sim_vrpn_bridge/native_vrpn_extension.h"
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include "gazebo_sim_vrpn_bridge/server_config.h"
#include "gazebo_sim_vrpn_bridge/vrpn_tracker_server.h"

namespace gazebo_sim_vrpn_bridge {
namespace {

geometry_msgs::Pose poseMessage(const ignition::math::Pose3d& value) {
    geometry_msgs::Pose message;
    message.position.x = value.Pos().X();
    message.position.y = value.Pos().Y();
    message.position.z = value.Pos().Z();
    message.orientation.x = value.Rot().X();
    message.orientation.y = value.Rot().Y();
    message.orientation.z = value.Rot().Z();
    message.orientation.w = value.Rot().W();
    return message;
}

} // namespace

// Server-level plugin loaded with gzserver -s. Gazebo model access stays on the
// simulation thread; all filtering, delay simulation, and VRPN network work is
// performed by one dedicated worker.
class GazeboVrpnSystemPlugin final : public gazebo::SystemPlugin {
  public:
    GazeboVrpnSystemPlugin() = default;

    ~GazeboVrpnSystemPlugin() override { stop(); }

    void Load(int /*argc*/, char** /*argv*/) override {
        world_created_connection_ = gazebo::event::Events::ConnectWorldCreated(
            std::bind(&GazeboVrpnSystemPlugin::onWorldCreated, this, std::placeholders::_1));
    }

  private:
    struct TrackedModelHandle {
        std::string gazebo_model_name;
        gazebo::physics::ModelPtr model;
    };

    void onWorldCreated(const std::string& world_name) {
        if (world_) {
            gzerr << "Gazebo VRPN SystemPlugin supports one world per gzserver process\n";
            return;
        }

        world_ = gazebo::physics::get_world(world_name);
        if (!world_) {
            gzerr << "Gazebo VRPN SystemPlugin could not resolve world " << world_name << "\n";
            return;
        }
        startup_binding_ = std::make_unique<xgc2_gazebo_scene::WorldStartupBinding>(world_,
            [this](gazebo::physics::WorldPtr) { attachAuthority(); });
    }

    void attachAuthority() {
        component_binding_ = std::make_unique<xgc2_gazebo_scene::NativeComponentBinding>(world_, "vrpn");
        try {
            // ROS console throttling needs its local time utility initialized.
            // This does not initialize a ROS node, transport or parameter bus.
            if (!ros::isInitialized()) ros::Time::init();
            const auto path = std::getenv("XGC_SIM_VRPN_CONFIG");
            if (!path || !*path) throw std::runtime_error("XGC_SIM_VRPN_CONFIG must name the explicitly authored absolute YAML file");
            auto document = readServerConfigFile(path);
            config_ = loadServerConfig(document);
            native_state_ = std::make_shared<NativeVrpnState>(config_, document);
            native_adapter_ = std::make_shared<NativeVrpnExtension>(native_state_);
            extension_binding_ = std::make_unique<xgc2_gazebo_scene::WorldExtensionBinding>(world_, "vrpn", native_adapter_);
        } catch (const std::exception& error) {
            gzerr << "Gazebo VRPN source configuration failed: " << error.what() << "\n";
            component_binding_->Failed();
            return;
        }

        stopping_.store(false, std::memory_order_release);
        worker_ = std::thread(&GazeboVrpnSystemPlugin::workerLoop, this);
        update_connection_ =
            gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboVrpnSystemPlugin::onWorldUpdateEnd, this));
        time_reset_connection_ = gazebo::event::Events::ConnectTimeReset([this] { publishNativeTime(); });
        ROS_INFO("[GazeboVrpnSystemPlugin] Attached directly to Gazebo world '%s'; "
                 "ROS model-state transport is disabled for this backend",
                 world_->Name().c_str());
    }

    void onWorldUpdateEnd() {
        if (!world_ || stopping_.load(std::memory_order_acquire)) {
            return;
        }

        try {
            publishNativeTime();
            const auto config = std::atomic_load(&native_state_->live);
            if (sampling_config_ != config) {
                sampling_config_ = config;
                last_scan_wall_time_s_ = 0.;
                next_capture_time_ = {};
            }
            // Gazebo may update substantially faster than the requested VRPN
            // rate. Capture at the report cadence so large fleets are not
            // copied on simulation steps that the worker cannot consume.
            const auto capture_time = std::chrono::steady_clock::now();
            if (next_capture_time_ != std::chrono::steady_clock::time_point{} && capture_time < next_capture_time_) {
                return;
            }
            const auto capture_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / config->publish_rate_hz));
            if (next_capture_time_ == std::chrono::steady_clock::time_point{}) {
                next_capture_time_ = capture_time;
            }
            do {
                next_capture_time_ += capture_period;
            } while (next_capture_time_ <= capture_time);

            const double capture_wall_time_s = ros::WallTime::now().toSec();
            refreshTrackedModels(capture_wall_time_s, *config);

            ModelStateSnapshot snapshot;
            const auto simulation_ns = native_simulation_ns_.load(std::memory_order_acquire);
            snapshot.sample_time_s = gazebo::common::Time(static_cast<int>(simulation_ns / 1000000000),
                static_cast<int>(simulation_ns % 1000000000)).Double();
            snapshot.capture_wall_time_s = capture_wall_time_s;
            snapshot.wire_timestamp_source = WireTimestampSource::SimulationTime;
            snapshot.models.reserve(tracked_models_.size());
            for (const TrackedModelHandle& tracked : tracked_models_) {
                if (!tracked.model) {
                    continue;
                }
                snapshot.models.push_back(
                    ModelPoseSample{tracked.gazebo_model_name, poseMessage(tracked.model->WorldPose())});
            }

            {
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                latest_snapshot_ = std::move(snapshot);
                ++latest_snapshot_sequence_;
                native_state_->sample_sequence.store(latest_snapshot_sequence_);
            }
        } catch (const std::exception& error) {
            ROS_ERROR_THROTTLE(2.0, "[GazeboVrpnSystemPlugin] Gazebo snapshot failed: %s", error.what());
        }
    }

    // Called only by native UpdateEnd/TimeReset owners. The science worker
    // consumes the immutable pose+timestamp snapshot and never reads World time.
    void publishNativeTime() {
        if (!world_) return;
        const auto time = world_->SimTime();
        native_simulation_ns_.store(static_cast<std::int64_t>(time.sec) * 1000000000 + time.nsec,
                                    std::memory_order_release);
    }

    void refreshTrackedModels(double wall_time_s, const ServerConfig& config) {
        if (last_scan_wall_time_s_ != 0.0 && wall_time_s - last_scan_wall_time_s_ < config.scan_interval_s) {
            return;
        }

        std::vector<TrackedModelHandle> discovered;
        for (const gazebo::physics::ModelPtr& model : world_->Models()) {
            if (!model) {
                continue;
            }
            std::string model_name = model->GetName();
            // RPC entity identity and ROS tracker names are distinct. Explicit
            // model mappings retain precedence over the authored ROS namespace.
            const auto artifact = model->GetSDF();
            if (artifact && artifact->HasElement("plugin"))
                for (auto plugin = artifact->GetElement("plugin"); plugin; plugin = plugin->GetNextElement("plugin"))
                    if (plugin->HasElement("public_entity_id")) {
                        model_name = plugin->Get<std::string>("public_entity_id");
                        if (plugin->HasElement("ros_namespace") &&
                            config.configured_model_to_tracker.find(model_name) == config.configured_model_to_tracker.end())
                            model_name = plugin->Get<std::string>("ros_namespace");
                        break;
                    }
            if (config.trackerNameForGazeboModel(model_name).empty()) {
                continue;
            }
            discovered.push_back(TrackedModelHandle{model_name, model});
        }
        tracked_models_.swap(discovered);
        last_scan_wall_time_s_ = wall_time_s;
    }

    void workerLoop() noexcept {
        try {
            VrpnTrackerServer server(config_);
            {
                std::lock_guard<std::mutex> lock(native_state_->mutex);
                native_state_->ready.store(true);
                native_state_->applied = native_state_->bootstrap;
                native_state_->applied_revision = 1;
            }
            component_binding_->Ready();
            native_state_->changed.notify_all();
            auto next_cycle = std::chrono::steady_clock::now();
            std::uint64_t processed_sequence = 0;

            while (!stopping_.load(std::memory_order_acquire) && !native_state_->stopped.load()) {
                std::shared_ptr<const NativeVrpnCommand> pending;
                bool changed_config = false;
                {
                    std::lock_guard<std::mutex> lock(native_state_->mutex);
                    pending = native_state_->pending;
                    changed_config = pending && pending->document != native_state_->applied;
                }
                if (pending) {
                    if (changed_config) server.applyConfig(pending->values);
                    auto live = std::make_shared<const ServerConfig>(pending->values);
                    {
                        std::lock_guard<std::mutex> lock(native_state_->mutex);
                        native_state_->applied = pending->document;
                        native_state_->applied_revision = pending->expected_revision + 1;
                        native_state_->pending.reset();
                        if (changed_config) std::atomic_store(&native_state_->live, live);
                    }
                    native_state_->changed.notify_all();
                }
                const auto live = std::atomic_load(&native_state_->live);
                const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(1.0 / live->publish_rate_hz));
                ModelStateSnapshot snapshot;
                bool have_snapshot = false;
                {
                    std::lock_guard<std::mutex> lock(snapshot_mutex_);
                    if (latest_snapshot_sequence_ != processed_sequence) {
                        snapshot = std::move(latest_snapshot_);
                        processed_sequence = latest_snapshot_sequence_;
                        have_snapshot = true;
                    }
                }

                if (have_snapshot) {
                    server.processSnapshot(snapshot);
                }
                server.publish(ros::WallTime::now().toSec());
                server.mainloop();
                native_state_->tracked_models.store(server.trackedModelCount());
                if (!server.healthy()) throw std::runtime_error("VRPN native listener failed");

                next_cycle += period;
                const auto now = std::chrono::steady_clock::now();
                if (next_cycle + period < now) {
                    next_cycle = now + period;
                }
                std::unique_lock<std::mutex> lock(native_state_->mutex);
                native_state_->changed.wait_until(lock, next_cycle, [this] {
                    return stopping_.load(std::memory_order_acquire) || native_state_->stopped.load() || bool(native_state_->pending);
                });
            }
        } catch (const std::exception& error) {
            {std::lock_guard<std::mutex> lock(native_state_->mutex);native_state_->error=error.what();native_state_->ready.store(false);}
            component_binding_->Failed();native_state_->changed.notify_all();
            gzerr << "Gazebo VRPN native worker stopped: " << error.what() << "\n";
        } catch (...) {
            {std::lock_guard<std::mutex> lock(native_state_->mutex);native_state_->error="native VRPN worker failed";native_state_->ready.store(false);}
            component_binding_->Failed();native_state_->changed.notify_all();
        }
    }

    void stop() {
        startup_binding_.reset();
        update_connection_.reset();
        time_reset_connection_.reset();
        world_created_connection_.reset();
        stopping_.store(true, std::memory_order_release);
        if(native_state_)native_state_->Stop();
        if (worker_.joinable()) {
            worker_.join();
        }
        extension_binding_.reset();
        native_adapter_.reset();
        component_binding_.reset();
        tracked_models_.clear();
        world_.reset();
    }

    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr world_created_connection_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::event::ConnectionPtr time_reset_connection_;
    std::atomic<std::int64_t> native_simulation_ns_{0};
    ServerConfig config_;
    std::shared_ptr<NativeVrpnState> native_state_;
    std::shared_ptr<const ServerConfig> sampling_config_;
    std::shared_ptr<NativeVrpnExtension> native_adapter_;
    std::unique_ptr<xgc2_gazebo_scene::WorldStartupBinding> startup_binding_;
    std::unique_ptr<xgc2_gazebo_scene::NativeComponentBinding> component_binding_;
    std::unique_ptr<xgc2_gazebo_scene::WorldExtensionBinding> extension_binding_;
    std::vector<TrackedModelHandle> tracked_models_;
    double last_scan_wall_time_s_{0.0};
    std::chrono::steady_clock::time_point next_capture_time_;

    std::mutex snapshot_mutex_;
    ModelStateSnapshot latest_snapshot_;
    std::uint64_t latest_snapshot_sequence_{0};

    std::atomic_bool stopping_{false};
    std::thread worker_;
};

GZ_REGISTER_SYSTEM_PLUGIN(GazeboVrpnSystemPlugin)

} // namespace gazebo_sim_vrpn_bridge
