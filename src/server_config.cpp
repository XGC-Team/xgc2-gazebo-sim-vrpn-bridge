#include "gazebo_sim_vrpn_bridge/server_config.h"

#include <algorithm>
#include <array>
#include <iterator>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <limits>

#include "gazebo_sim_vrpn_bridge/model_identity.h"

namespace gazebo_sim_vrpn_bridge {
namespace {

bool startsWith(const std::string& value, const std::string& prefix) {
    return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

std::string trimSlashes(std::string value) {
    while (!value.empty() && value.front() == '/') {
        value.erase(value.begin());
    }
    while (!value.empty() && value.back() == '/') {
        value.pop_back();
    }
    return value;
}

double xmlRpcToDouble(XmlRpc::XmlRpcValue& value, const std::string& param_name) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return static_cast<int>(value);
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        return static_cast<double>(value);
    }
    throw std::runtime_error(param_name + " entries must be numeric");
}

std::vector<double> parseDoubleVector(XmlRpc::XmlRpcValue& value, const std::string& param_name, int expected_size) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != expected_size) {
        throw std::runtime_error(param_name + " must be a YAML list of " + std::to_string(expected_size) + " numbers");
    }

    std::vector<double> result;
    result.reserve(static_cast<std::size_t>(expected_size));
    for (int index = 0; index < value.size(); ++index) {
        result.push_back(xmlRpcToDouble(value[index], param_name));
    }
    return result;
}

std::array<double, 3> toArray3(const std::vector<double>& values) {
    return {{values[0], values[1], values[2]}};
}

void ConfigFields(XmlRpc::XmlRpcValue& value,const std::string& name,std::initializer_list<const char*> fields) {
    if(value.getType()!=XmlRpc::XmlRpcValue::TypeStruct)throw std::runtime_error(name+" must be a mapping");
    for(auto& item:value){bool known=false;for(auto field:fields)known|=item.first==field;if(!known)throw std::runtime_error("unknown "+name+" field: "+item.first);}
}

tf2::Transform parseTransform(XmlRpc::XmlRpcValue& value, const std::string& param_name) {
    ConfigFields(value,param_name,{"xyz","translation","rpy","rotation_rpy","quaternion"});
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error(param_name + " must be a YAML mapping");
    }

    std::vector<double> xyz{0.0, 0.0, 0.0};
    if (value.hasMember("xyz")) {
        xyz = parseDoubleVector(value["xyz"], param_name + ".xyz", 3);
    } else if (value.hasMember("translation")) {
        xyz = parseDoubleVector(value["translation"], param_name + ".translation", 3);
    }

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);
    if (value.hasMember("rpy")) {
        const std::vector<double> rpy = parseDoubleVector(value["rpy"], param_name + ".rpy", 3);
        quaternion.setRPY(rpy[0], rpy[1], rpy[2]);
    } else if (value.hasMember("rotation_rpy")) {
        const std::vector<double> rpy = parseDoubleVector(value["rotation_rpy"], param_name + ".rotation_rpy", 3);
        quaternion.setRPY(rpy[0], rpy[1], rpy[2]);
    }
    if (value.hasMember("quaternion")) {
        const std::vector<double> values = parseDoubleVector(value["quaternion"], param_name + ".quaternion", 4);
        quaternion = tf2::Quaternion(values[0], values[1], values[2], values[3]);
    }
    quaternion.normalize();

    return tf2::Transform(quaternion, tf2::Vector3(xyz[0], xyz[1], xyz[2]));
}

std::vector<std::string> splitTrackerString(const std::string& value) {
    std::vector<std::string> result;
    std::set<std::string> seen;
    std::string normalized;
    normalized.reserve(value.size());
    std::transform(value.begin(), value.end(), std::back_inserter(normalized), [](char character) {
        return (character == ',' || character == ';') ? ' ' : character;
    });

    std::istringstream stream(normalized);
    std::string item;
    while (stream >> item) {
        item = trimSlashes(item);
        if (!item.empty() && seen.insert(item).second) {
            result.push_back(item);
        }
    }
    return result;
}

std::vector<std::string> parseStringList(XmlRpc::XmlRpcValue& value, const std::string& param_name) {
    std::vector<std::string> result;
    std::set<std::string> seen;
    if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
        return splitTrackerString(static_cast<std::string>(value));
    }
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::runtime_error(param_name + " must be a YAML list");
    }
    for (int index = 0; index < value.size(); ++index) {
        if (value[index].getType() != XmlRpc::XmlRpcValue::TypeString) {
            throw std::runtime_error(param_name + " entries must be strings");
        }
        const std::string item = trimSlashes(static_cast<std::string>(value[index]));
        if (!item.empty() && seen.insert(item).second) {
            result.push_back(item);
        }
    }
    return result;
}

DelayTimestampPolicy parseDelayTimestampPolicy(const std::string& value) {
    if (value == "send_time") {
        return DelayTimestampPolicy::SendTime;
    }
    if (value == "sample_time") {
        return DelayTimestampPolicy::SampleTime;
    }
    throw std::runtime_error("delay.timestamp_policy must be one of: send_time, sample_time");
}

// XmlRpcValue is only an in-memory legacy configuration value here. There
// is no XMLRPC transport, NodeHandle or ROS parameter access in this domain.
XmlRpc::XmlRpcValue configValue(const Json::Value& value, unsigned depth = 0) {
    if (depth > 12) throw std::runtime_error("VRPN config nesting exceeds 12");
    if (value.isBool()) return XmlRpc::XmlRpcValue(value.asBool());
    if (value.type() == Json::intValue || value.type() == Json::uintValue) {
        if (!value.isInt()) throw std::runtime_error("VRPN integer exceeds authored 32-bit configuration");
        return XmlRpc::XmlRpcValue(value.asInt());
    }
    if (value.isDouble()) {
        if (!std::isfinite(value.asDouble())) throw std::runtime_error("VRPN numbers must be finite");
        return XmlRpc::XmlRpcValue(value.asDouble());
    }
    if (value.isString()) { if(value.asString().size()>256)throw std::runtime_error("VRPN configuration string exceeds 256 bytes"); return XmlRpc::XmlRpcValue(value.asString()); }
    XmlRpc::XmlRpcValue result;
    if (value.isArray()) {
        if (value.size() > 256) throw std::runtime_error("VRPN config list exceeds 256 entries");
        result.setSize(value.size());
        for (Json::ArrayIndex i=0;i<value.size();++i) result[int(i)]=configValue(value[i],depth+1);
    } else if (value.isObject()) {
        (void)result.begin(); // establish an empty struct for authored {} mappings
        if (value.size()>128) throw std::runtime_error("VRPN config mapping exceeds 128 entries");
        for (const auto& name:value.getMemberNames()) result[name]=configValue(value[name],depth+1);
    } else throw std::runtime_error("VRPN config cannot contain null values");
    return result;
}
class ConfigValues {
 public:
    explicit ConfigValues(Json::Value value):value_(std::move(value)) {
        if (!value_.isObject()) throw std::runtime_error("VRPN configuration must be a mapping");
        const std::set<std::string> allowed={"bind_address","port","publish_rate","stale_timeout","scan_interval",
            "velocity_filter_cutoff","acceleration_filter_cutoff","derivative_reset_timeout","mocap_noise_enabled",
            "mocap_noise_seed","match_mode","trackers","enabled_trackers","default_body_to_tracker","mocap_noise",
            "delay","auto_mapping","robots","manual_mapping","extrinsics","auto_track_known_models","delay_enabled",
            "delay_timestamp_policy","delay_seed"};
        for(const auto& name:value_.getMemberNames()) if(!allowed.count(name)) throw std::runtime_error("unknown VRPN configuration field: "+name);
    }
    bool getParam(const std::string& name,XmlRpc::XmlRpcValue& output) const {
        if(!value_.isMember(name)) return false;
        output=configValue(value_[name]); return true;
    }
    bool getParam(const std::string& name,bool& output) const {
        if(!value_.isMember(name)) return false;
        if(!value_[name].isBool()) throw std::runtime_error(name+" must be boolean");
        output=value_[name].asBool();return true;
    }
    bool getParam(const std::string& name,int& output) const {
        if(!value_.isMember(name))return false;
        if((value_[name].type()!=Json::intValue && value_[name].type()!=Json::uintValue)||!value_[name].isInt())throw std::runtime_error(name+" must be a 32-bit integer");
        output=value_[name].asInt();return true;
    }
    bool getParam(const std::string& name,double& output) const {
        if(!value_.isMember(name))return false;
        if(!value_[name].isNumeric()||!std::isfinite(value_[name].asDouble()))throw std::runtime_error(name+" must be finite numeric");
        output=value_[name].asDouble();return true;
    }
    bool getParam(const std::string& name,std::string& output) const {
        if(!value_.isMember(name))return false;
        if(!value_[name].isString())throw std::runtime_error(name+" must be string");
        output=value_[name].asString();return true;
    }
    template<class T>void param(const std::string& name,T& output,const T& fallback)const {if(!getParam(name,output))output=fallback;}
 private:
    Json::Value value_;
};

class ServerConfigLoader {
  public:
    explicit ServerConfigLoader(const Json::Value& private_node) : private_node_(private_node) {}

    ServerConfig load() {
        private_node_.param<std::string>("bind_address", config_.bind_address, config_.bind_address);
        private_node_.param<int>("port", config_.port, config_.port);
        private_node_.param<double>("publish_rate", config_.publish_rate_hz, config_.publish_rate_hz);
        private_node_.param<double>("stale_timeout", config_.stale_timeout_s, config_.stale_timeout_s);
        private_node_.param<double>("scan_interval", config_.scan_interval_s, config_.scan_interval_s);
        private_node_.param<double>("velocity_filter_cutoff", config_.velocity_filter_cutoff_hz,
                                    config_.velocity_filter_cutoff_hz);
        private_node_.param<double>("acceleration_filter_cutoff", config_.acceleration_filter_cutoff_hz,
                                    config_.acceleration_filter_cutoff_hz);
        private_node_.param<double>("derivative_reset_timeout", config_.derivative_reset_timeout_s,
                                    config_.derivative_reset_timeout_s);
        private_node_.param<bool>("mocap_noise_enabled", config_.mocap_noise.enabled, config_.mocap_noise.enabled);
        private_node_.param<int>("mocap_noise_seed", mocap_noise_seed_param_,
                                 static_cast<int>(config_.mocap_noise.seed));
        private_node_.param<std::string>("match_mode", config_.match_mode, config_.match_mode);

        loadStructuredConfig();
        // Explicit scalar operator overrides are applied after the authored
        // structured document, preserving the launch precedence without ROS.
        private_node_.getParam("mocap_noise_enabled", config_.mocap_noise.enabled);
        private_node_.getParam("mocap_noise_seed", mocap_noise_seed_param_);
        applyAutoMappingOverrides();
        applyDelayOverrides();
        config_.mocap_noise.seed =
            mocap_noise_seed_param_ <= 0 ? 1U : static_cast<unsigned int>(mocap_noise_seed_param_);
        config_.validate();
        return config_;
    }

  private:
    void loadStructuredConfig() {
        XmlRpc::XmlRpcValue trackers;
        if (private_node_.getParam("trackers", trackers)) {
            config_.tracker_patterns = parseStringList(trackers, "trackers");
        }

        XmlRpc::XmlRpcValue enabled_trackers;
        if (config_.tracker_patterns.empty() && private_node_.getParam("enabled_trackers", enabled_trackers)) {
            config_.tracker_patterns = parseStringList(enabled_trackers, "enabled_trackers");
        }

        XmlRpc::XmlRpcValue default_transform;
        if (private_node_.getParam("default_body_to_tracker", default_transform)) {
            config_.default_body_to_tracker = parseTransform(default_transform, "default_body_to_tracker");
        }

        XmlRpc::XmlRpcValue noise;
        if (private_node_.getParam("mocap_noise", noise)) {
            parseMocapNoise(noise);
        }

        XmlRpc::XmlRpcValue delay;
        if (private_node_.getParam("delay", delay)) {
            parseDelay(delay);
        }

        XmlRpc::XmlRpcValue auto_mapping;
        if (private_node_.getParam("auto_mapping", auto_mapping)) {
            parseAutoMapping(auto_mapping);
        }

        XmlRpc::XmlRpcValue robots;
        if (private_node_.getParam("robots", robots)) {
            parseRobots(robots);
        }

        XmlRpc::XmlRpcValue manual_mapping;
        if (private_node_.getParam("manual_mapping", manual_mapping)) {
            parseManualMapping(manual_mapping);
        }

        XmlRpc::XmlRpcValue extrinsics;
        if (private_node_.getParam("extrinsics", extrinsics)) {
            parseExtrinsics(extrinsics);
        }
    }

    void parseMocapNoise(XmlRpc::XmlRpcValue& noise) {
        ConfigFields(noise,"mocap_noise",{"enabled","position_stddev_xyz","rotation_stddev_rpy","seed"});
        if (noise.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("mocap_noise must be a YAML mapping");
        }
        if (noise.hasMember("enabled")) {
            config_.mocap_noise.enabled = static_cast<bool>(noise["enabled"]);
        }
        if (noise.hasMember("position_stddev_xyz")) {
            config_.mocap_noise.position_stddev_m =
                toArray3(parseDoubleVector(noise["position_stddev_xyz"], "mocap_noise.position_stddev_xyz", 3));
        }
        if (noise.hasMember("rotation_stddev_rpy")) {
            config_.mocap_noise.rotation_stddev_rad =
                toArray3(parseDoubleVector(noise["rotation_stddev_rpy"], "mocap_noise.rotation_stddev_rpy", 3));
        }
        if (noise.hasMember("seed")) {
            mocap_noise_seed_param_ = static_cast<int>(xmlRpcToDouble(noise["seed"], "mocap_noise.seed"));
        }
    }

    void parseDelay(XmlRpc::XmlRpcValue& delay) {
        ConfigFields(delay,"delay",{"enabled","timestamp_policy","seed","max_delay_ms","history_margin_ms","common","trackers"});
        if (delay.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("delay must be a YAML mapping");
        }
        if (delay.hasMember("enabled")) {
            config_.delay.enabled = static_cast<bool>(delay["enabled"]);
        }
        if (delay.hasMember("timestamp_policy")) {
            config_.delay.timestamp_policy =
                parseDelayTimestampPolicy(static_cast<std::string>(delay["timestamp_policy"]));
        }
        if (delay.hasMember("seed")) {
            config_.delay.seed = static_cast<unsigned int>(std::max(0.0, xmlRpcToDouble(delay["seed"], "delay.seed")));
        }
        if (delay.hasMember("max_delay_ms")) {
            config_.delay.max_delay_ms = xmlRpcToDouble(delay["max_delay_ms"], "delay.max_delay_ms");
        }
        if (delay.hasMember("history_margin_ms")) {
            config_.delay.history_margin_ms = xmlRpcToDouble(delay["history_margin_ms"], "delay.history_margin_ms");
        }
        if (delay.hasMember("common")) {
            parseDelayComponent(delay["common"], "delay.common", config_.delay.common);
        }
        if (delay.hasMember("trackers")) {
            parseDelayTrackers(delay["trackers"]);
        }
    }

    void parseDelayTrackers(XmlRpc::XmlRpcValue& trackers) {
        if (trackers.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("delay.trackers must be a YAML mapping");
        }
        for (auto& tracker : trackers) {
            const std::string tracker_name = trimSlashes(tracker.first);
            if (tracker_name.empty()) {
                continue;
            }
            if (tracker.second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                throw std::runtime_error("delay.trackers." + tracker_name + " must be a YAML mapping");
            }
            TrackerDelayConfig config;
            parseDelayComponent(tracker.second, "delay.trackers." + tracker_name, config);
            if (tracker.second.hasMember("max_delay_ms")) {
                config.max_delay_ms =
                    xmlRpcToDouble(tracker.second["max_delay_ms"], "delay.trackers." + tracker_name + ".max_delay_ms");
            }
            config_.delay.trackers[tracker_name] = config;
        }
    }

    static void parseDelayComponent(XmlRpc::XmlRpcValue& value, const std::string& param_name,
                                    DelayComponentConfig& config) {
        if(param_name=="delay.common")ConfigFields(value,param_name,{"base_ms","slow_stddev_ms","slow_tau_s","jitter_stddev_ms","burst_probability","burst_extra_ms"});
        else ConfigFields(value,param_name,{"base_ms","slow_stddev_ms","slow_tau_s","jitter_stddev_ms","burst_probability","burst_extra_ms","max_delay_ms"});
        if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error(param_name + " must be a YAML mapping");
        }
        if (value.hasMember("base_ms")) {
            config.base_ms = xmlRpcToDouble(value["base_ms"], param_name + ".base_ms");
        }
        if (value.hasMember("slow_stddev_ms")) {
            config.slow_stddev_ms = xmlRpcToDouble(value["slow_stddev_ms"], param_name + ".slow_stddev_ms");
        }
        if (value.hasMember("slow_tau_s")) {
            config.slow_tau_s = xmlRpcToDouble(value["slow_tau_s"], param_name + ".slow_tau_s");
        }
        if (value.hasMember("jitter_stddev_ms")) {
            config.jitter_stddev_ms = xmlRpcToDouble(value["jitter_stddev_ms"], param_name + ".jitter_stddev_ms");
        }
        if (value.hasMember("burst_probability")) {
            config.burst_probability = xmlRpcToDouble(value["burst_probability"], param_name + ".burst_probability");
        }
        if (value.hasMember("burst_extra_ms")) {
            const std::vector<double> range =
                parseDoubleVector(value["burst_extra_ms"], param_name + ".burst_extra_ms", 2);
            config.burst_extra_min_ms = range[0];
            config.burst_extra_max_ms = range[1];
        }
    }

    void parseAutoMapping(XmlRpc::XmlRpcValue& auto_mapping) {
        ConfigFields(auto_mapping,"auto_mapping",{"enabled","include_patterns"});
        if (auto_mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("auto_mapping must be a YAML mapping");
        }
        if (auto_mapping.hasMember("enabled")) {
            config_.auto_track_known_models = static_cast<bool>(auto_mapping["enabled"]);
        }
        if (auto_mapping.hasMember("include_patterns")) {
            config_.auto_include_patterns =
                parseStringList(auto_mapping["include_patterns"], "auto_mapping.include_patterns");
        }
    }

    void applyAutoMappingOverrides() {
        // Trusted launchers load YAML first and then set this explicit private
        // parameter. Preserve that precedence for both server backends.
        bool auto_track_known_models = false;
        if (private_node_.getParam("auto_track_known_models", auto_track_known_models)) {
            config_.auto_track_known_models = auto_track_known_models;
        }
    }

    void applyDelayOverrides() {
        bool delay_enabled = false;
        if (private_node_.getParam("delay_enabled", delay_enabled)) {
            config_.delay.enabled = delay_enabled;
        }

        std::string timestamp_policy;
        if (private_node_.getParam("delay_timestamp_policy", timestamp_policy)) {
            config_.delay.timestamp_policy = parseDelayTimestampPolicy(timestamp_policy);
        }

        int delay_seed = 0;
        if (private_node_.getParam("delay_seed", delay_seed)) {
            config_.delay.seed = static_cast<unsigned int>(std::max(delay_seed, 0));
        }
    }

    void parseRobots(XmlRpc::XmlRpcValue& robots) {
        if (robots.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("robots must be a YAML mapping");
        }

        for (auto& robot : robots) {
            const std::string tracker_name = robot.first;
            XmlRpc::XmlRpcValue& value = robot.second;
            if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                throw std::runtime_error("robots." + tracker_name + " must be a YAML mapping");
            }

            ConfigFields(value,"robots."+tracker_name,{"enabled","gazebo_model_name","gazebo_model","body_to_tracker"});
            RobotConfig config;
            config.body_to_tracker = config_.default_body_to_tracker;
            if (value.hasMember("enabled")) {
                config.enabled = static_cast<bool>(value["enabled"]);
            }
            if (!config.enabled) {
                config_.robot_configs[tracker_name] = config;
                continue;
            }
            if (value.hasMember("gazebo_model_name")) {
                config.gazebo_model_name = static_cast<std::string>(value["gazebo_model_name"]);
                config_.configured_model_to_tracker[trimSlashes(config.gazebo_model_name)] = tracker_name;
            } else if (value.hasMember("gazebo_model")) {
                config.gazebo_model_name = static_cast<std::string>(value["gazebo_model"]);
                config_.configured_model_to_tracker[trimSlashes(config.gazebo_model_name)] = tracker_name;
            }
            if (value.hasMember("body_to_tracker")) {
                config.body_to_tracker =
                    parseTransform(value["body_to_tracker"], "robots." + tracker_name + ".body_to_tracker");
            }
            config_.robot_configs[tracker_name] = config;
        }
    }

    void parseManualMapping(XmlRpc::XmlRpcValue& manual_mapping) {
        if (manual_mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("manual_mapping must be a YAML mapping");
        }
        for (auto& mapping : manual_mapping) {
            XmlRpc::XmlRpcValue& value = mapping.second;
            if (value.getType() != XmlRpc::XmlRpcValue::TypeString) {
                throw std::runtime_error("manual_mapping." + mapping.first + " must be a tracker name string");
            }
            const std::string gazebo_model_name = trimSlashes(mapping.first);
            const std::string tracker_name = trimSlashes(static_cast<std::string>(value));
            if (!gazebo_model_name.empty() && !tracker_name.empty()) {
                config_.configured_model_to_tracker[gazebo_model_name] = tracker_name;
            }
        }
    }

    void parseExtrinsics(XmlRpc::XmlRpcValue& extrinsics) {
        if (extrinsics.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("extrinsics must be a YAML mapping");
        }
        for (auto& extrinsic : extrinsics) {
            const std::string tracker_name = trimSlashes(extrinsic.first);
            if (tracker_name.empty()) {
                continue;
            }
            config_.robot_configs[tracker_name].body_to_tracker =
                parseTransform(extrinsic.second, "extrinsics." + tracker_name);
        }
    }

    ConfigValues private_node_;
    ServerConfig config_;
    int mocap_noise_seed_param_{1};
};

bool matchesPattern(const ServerConfig& config, const std::string& model_name, const std::string& pattern) {
    if (pattern.empty()) {
        return false;
    }
    if (config.match_mode == "exact") {
        return model_name == pattern;
    }
    if (config.match_mode == "prefix") {
        return startsWith(model_name, pattern);
    }
    return model_name.find(pattern) != std::string::npos;
}

bool matchesAutoPattern(const ServerConfig& config, const std::string& model_name) {
    // Model discovery is intentionally low-rate, but large worlds can still
    // contain thousands of models. Compile each configured expression once
    // per calling thread instead of once per model and scan.
    static thread_local std::map<std::string, std::regex> compiled_patterns;
    for (const std::string& pattern : config.auto_include_patterns) {
        try {
            auto compiled = compiled_patterns.find(pattern);
            if (compiled == compiled_patterns.end()) {
                compiled = compiled_patterns.emplace(pattern, std::regex(pattern)).first;
            }
            if (std::regex_match(model_name, compiled->second)) {
                return true;
            }
        } catch (const std::regex_error& error) {
            throw std::runtime_error("invalid auto_mapping.include_patterns regex '" + pattern + "': " + error.what());
        }
    }
    return false;
}

} // namespace

std::string ServerConfig::trackerNameForGazeboModel(const std::string& gazebo_model_name) const {
    const std::string name = trimSlashes(gazebo_model_name);
    if (name.empty()) {
        return {};
    }

    const auto configured = configured_model_to_tracker.find(name);
    if (configured != configured_model_to_tracker.end()) {
        return configured->second;
    }

    // Automatic canonical discovery and legacy pattern matching are two
    // mutually exclusive operator modes. The panel retains hidden defaults
    // for the inactive mode, so auto mode must never consult tracker_patterns.
    if (auto_track_known_models) {
        if (matchesAutoPattern(*this, name)) {
            return name;
        }
        if (auto_include_patterns.empty() && isCanonicalAutoTrackedModelName(name)) {
            return name;
        }
        return {};
    }

    const auto matched_pattern =
        std::find_if(tracker_patterns.begin(), tracker_patterns.end(), [this, &name](const std::string& pattern) {
            return matchesPattern(*this, name, pattern);
        });
    if (matched_pattern != tracker_patterns.end()) {
        return *matched_pattern;
    }
    return {};
}

tf2::Transform ServerConfig::bodyToTrackerFor(const std::string& tracker_name) const {
    const auto config = robot_configs.find(tracker_name);
    if (config != robot_configs.end()) {
        return config->second.body_to_tracker;
    }
    return default_body_to_tracker;
}

void ServerConfig::validate() const {
    if (match_mode != "contains" && match_mode != "exact" && match_mode != "prefix") {
        throw std::runtime_error("match_mode must be one of: contains, exact, prefix");
    }
    if (!std::isfinite(publish_rate_hz) || publish_rate_hz < 0.1 || publish_rate_hz > 1000.) {
        throw std::runtime_error("publish_rate must be 0.1 through 1000 Hz");
    }
    if (!std::isfinite(scan_interval_s) || scan_interval_s < 0.001 || scan_interval_s > 60.) {
        throw std::runtime_error("scan_interval must be 0.001 through 60 seconds");
    }
    if (port <= 0 || port > 65535) {
        throw std::runtime_error("port must be in range 1..65535");
    }
    for (const std::string& pattern : auto_include_patterns) {
        try {
            static_cast<void>(std::regex(pattern));
        } catch (const std::regex_error& error) {
            throw std::runtime_error("invalid auto_mapping.include_patterns regex '" + pattern + "': " + error.what());
        }
    }
    MeasurementDelay configured_delay(delay);
    configured_delay.validate(publish_rate_hz);
}

ServerConfig loadServerConfig(const Json::Value& private_node) {
    try { return ServerConfigLoader(private_node).load(); }
    catch(const XmlRpc::XmlRpcException& error) { throw std::runtime_error("invalid VRPN configuration type: "+error.getMessage()); }
}

namespace {
Json::Value yamlConfigNode(const YAML::Node& node,unsigned depth,std::size_t& nodes) {
    if(++nodes>4096)throw std::runtime_error("VRPN YAML expansion exceeds 4096 values");
    if(depth>12)throw std::runtime_error("VRPN YAML nesting exceeds 12");
    if(node.IsMap()) {
        Json::Value result(Json::objectValue);
        if(node.size()>128)throw std::runtime_error("VRPN YAML mapping exceeds 128 entries");
        for(auto item:node){const auto key=item.first.as<std::string>();if(result.isMember(key))throw std::runtime_error("duplicate VRPN YAML field: "+key);result[key]=yamlConfigNode(item.second,depth+1,nodes);}return result;
    }
    if(node.IsSequence()) {
        Json::Value result(Json::arrayValue);if(node.size()>256)throw std::runtime_error("VRPN YAML list exceeds 256 entries");
        for(auto item:node)result.append(yamlConfigNode(item,depth+1,nodes));return result;
    }
    if(!node.IsScalar())throw std::runtime_error("VRPN YAML null value is unsupported");
    const auto value=node.Scalar();
    if(node.Tag()=="!" || node.Tag()=="tag:yaml.org,2002:str")return value;
    if(value=="true")return true;if(value=="false")return false;
    static const std::regex integer("^[+-]?[0-9]+$");
    static const std::regex number(R"(^[+-]?([0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?$)");
    if(std::regex_match(value,integer))return node.as<int>();
    if(std::regex_match(value,number)){auto v=node.as<double>();if(!std::isfinite(v))throw std::runtime_error("VRPN YAML number must be finite");return v;}
    return value;
}
}
Json::Value readServerConfigFile(const std::string& absolute_path) {
    if(!std::filesystem::path(absolute_path).is_absolute())throw std::runtime_error("VRPN configuration path must be explicitly absolute");
    if(!std::filesystem::is_regular_file(absolute_path))throw std::runtime_error("VRPN configuration must be a regular authored file");
    std::ifstream file(absolute_path,std::ios::binary);if(!file)throw std::runtime_error("cannot open authored VRPN configuration");
    std::string document(65537,'\0');file.read(document.data(),document.size());document.resize(file.gcount());
    if(document.size()>65536)throw std::runtime_error("VRPN configuration exceeds 64 KiB");
    std::size_t nodes=0;auto result=yamlConfigNode(YAML::Load(document),0,nodes);loadServerConfig(result);return result;
}

} // namespace gazebo_sim_vrpn_bridge
