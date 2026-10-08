#pragma once
#include "gazebo_sim_vrpn_bridge/server_config.h"
#include <xgc2_gazebo_scene/native_world_extension.hpp>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>

namespace gazebo_sim_vrpn_bridge {
struct NativeVrpnCommand;
// Shared only between the world's management owner, engine sampling callback
// and the existing VRPN data worker. No additional listener or thread exists.
struct NativeVrpnState {
    NativeVrpnState(ServerConfig values,Json::Value document);
    std::mutex mutex;
    std::condition_variable changed;
    std::shared_ptr<const ServerConfig> live;
    Json::Value desired,applied,bootstrap;
    std::uint64_t desired_revision=1,applied_revision=0;
    std::shared_ptr<const NativeVrpnCommand> pending;
    std::atomic<bool> stopped{false},ready{false};
    std::atomic<std::size_t> tracked_models{0};
    std::atomic<std::uint64_t> sample_sequence{0};
    std::string error;
    void Stop() noexcept;
};
class NativeVrpnExtension final:public xgc2_gazebo_scene::NativeWorldExtension {
 public:
    explicit NativeVrpnExtension(std::shared_ptr<NativeVrpnState> state):state_(std::move(state)){}
    Json::Value Describe() const override;
    std::shared_ptr<const xgc2_gazebo_scene::PreparedWorldExtension> Prepare(const std::string& method,const std::string& route,const Json::Value& body) override;
    Json::Value Execute(const std::shared_ptr<const xgc2_gazebo_scene::PreparedWorldExtension>& command,bool* effects_started) override;
    void Stop() noexcept override {state_->Stop();}
 private:
    std::shared_ptr<NativeVrpnState> state_;
};
struct NativeVrpnCommand final:xgc2_gazebo_scene::PreparedWorldExtension {
    ServerConfig values;
    Json::Value document;
    std::uint64_t expected_revision=0;
    bool mutation=false;
};
}
