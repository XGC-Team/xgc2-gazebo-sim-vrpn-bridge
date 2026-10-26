#include "gazebo_sim_vrpn_bridge/model_identity.h"
#include "gazebo_sim_vrpn_bridge/server_config.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "gazebo_sim_vrpn_bridge/vrpn_tracker_server.h"

using gazebo_sim_vrpn_bridge::isCanonicalAutoTrackedModelName;
using gazebo_sim_vrpn_bridge::ServerConfig;

TEST(ModelIdentity, AcceptsCanonicalNumberedRobotModels) {
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("uav1"));
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("uav12"));
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("ugv1"));
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("ugv12"));
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("mecanum1"));
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("mecanum12"));
}

TEST(ModelIdentity, RejectsUnnumberedAndNoncanonicalNames) {
    EXPECT_FALSE(isCanonicalAutoTrackedModelName(""));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("uav"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("ugv"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("mecanum"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("mecanum_1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("mecanum-1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("Mecanum1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("omni1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("scout_mecanum1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("scoutugv1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("scout_mini_ugv1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("scout_mini_ros_control"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("scout_description"));
}

TEST(TrackerSelectionMode, AutoModeIgnoresRetainedLegacyPatternDefaults) {
    ServerConfig config;
    config.auto_track_known_models = true;
    config.auto_include_patterns = {"^uav[0-9]+$", "^ugv[0-9]+$", "^mecanum[0-9]+$"};
    config.tracker_patterns = {"uav", "ugv", "mecanum"};
    config.match_mode = "contains";

    EXPECT_EQ("ugv1", config.trackerNameForGazeboModel("ugv1"));
    EXPECT_EQ("ugv2", config.trackerNameForGazeboModel("ugv2"));
    EXPECT_TRUE(config.trackerNameForGazeboModel("scout_ugv_alpha").empty());
}

TEST(TrackerSelectionMode, LegacyModeUsesConfiguredPatternAndMatchMode) {
    ServerConfig config;
    config.auto_track_known_models = false;
    config.tracker_patterns = {"ugv"};
    config.match_mode = "contains";

    EXPECT_EQ("ugv", config.trackerNameForGazeboModel("ugv1"));
    EXPECT_EQ("ugv", config.trackerNameForGazeboModel("scout_ugv_alpha"));
}

TEST(TrackerSelectionMode, ExplicitModelMappingAppliesInEitherMode) {
    ServerConfig config;
    config.auto_track_known_models = true;
    config.auto_include_patterns = {"^ugv[0-9]+$"};
    config.tracker_patterns = {"ugv"};
    config.configured_model_to_tracker["scout_blue"] = "rigid_body_7";

    EXPECT_EQ("rigid_body_7", config.trackerNameForGazeboModel("scout_blue"));
}

TEST(NativeServerLogging, EmptyWorldDoesNotRequireRosStartOrClock) {
    ASSERT_FALSE(ros::isStarted());
    ServerConfig config;
    const int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_GE(socket_fd, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(bind(socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0);
    socklen_t address_size = sizeof(address);
    ASSERT_EQ(getsockname(socket_fd, reinterpret_cast<sockaddr*>(&address), &address_size), 0);
    config.port = ntohs(address.sin_port);
    close(socket_fd);
    config.bind_address = "127.0.0.1";
    gazebo_sim_vrpn_bridge::VrpnTrackerServer server(config);
    gazebo_sim_vrpn_bridge::ModelStateSnapshot snapshot;
    snapshot.capture_wall_time_s = ros::WallTime::now().toSec();
    EXPECT_NO_THROW(server.processSnapshot(snapshot));
    EXPECT_NO_THROW(server.publish(snapshot.capture_wall_time_s));
}

TEST(NativeServerLogging, DestructionReleasesEveryListenerWithoutProcessExit) {
    std::array<int, 3> reservations{};
    std::array<int, 3> ports{};
    for (std::size_t i = 0; i < ports.size(); ++i) {
        reservations[i] = socket(AF_INET, SOCK_STREAM, 0);
        ASSERT_GE(reservations[i], 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        ASSERT_EQ(bind(reservations[i], reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0);
        socklen_t size = sizeof(address);
        ASSERT_EQ(getsockname(reservations[i], reinterpret_cast<sockaddr*>(&address), &size), 0);
        ports[i] = ntohs(address.sin_port);
    }
    for (int fd : reservations) close(fd);
    const auto can_bind = [](int port, int kind) {
        const int fd = socket(AF_INET, kind, 0);
        if (fd < 0) return false;
        sockaddr_in address{}; address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(port);
        const bool success = bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0;
        close(fd);
        return success;
    };
    ServerConfig config; config.bind_address = "127.0.0.1"; config.port = ports[0];
    config.tracker_ports = {{"uav1", ports[1]}, {"uav2", ports[2]}};
    {
        gazebo_sim_vrpn_bridge::VrpnTrackerServer server(config);
        ASSERT_TRUE(server.healthy());
        for (int port : ports) {
            EXPECT_FALSE(can_bind(port, SOCK_STREAM));
            EXPECT_FALSE(can_bind(port, SOCK_DGRAM));
        }
    }
    for (int port : ports) {
        EXPECT_TRUE(can_bind(port, SOCK_STREAM));
        EXPECT_TRUE(can_bind(port, SOCK_DGRAM));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
