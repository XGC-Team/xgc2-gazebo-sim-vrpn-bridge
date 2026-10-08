# XGC2 Gazebo Sim VRPN Bridge

Gazebo Classic 11/ROS Noetic positioning data source. The native SystemPlugin
reads selected model poses in Gazebo's UpdateEnd owner and the existing science
worker performs filtering, noise, measurement delay and VRPN wire publication.
Adapter exclusively owns canonical positioning and PX4 vision output. This
source does not introduce a second positioning publisher or relay.

The process/workflow owner explicitly starts and stops gzserver. A prepared
world contains the simulation-v1 WorldPlugin and declares required_component
vrpn. Pass an absolute authored YAML file through native_world.launch's
vrpn_config argument. native_world_start supplies XGC_SIM_VRPN_CONFIG and loads
libgazebo_sim_vrpn_system_plugin.so. The configuration is a startup grant; no
ROS NodeHandle, parameter-server lookup, legacy server node or RPC process
launcher participates. The former vrpn_server.launch is retired.

WorldStartupBinding delays attachment until the already authored world authority
exists. WorldExtensionBinding registers the domain adapter on the world's one
SDK endpoint and management worker. NativeComponentBinding becomes ready only
when VrpnTrackerServer has created its actual listening connection; exceptions
mark it failed. The native world owner grants this required component explicitly.

The world SDK ServiceRef supplies instance and target fencing. Domain routes:

| Method | Route | Result |
| --- | --- | --- |
| GET | /v1/extensions/vrpn | Desired/applied configuration, revisions and native health |
| GET | /v1/extensions/vrpn/config | Same cached domain state |
| GET | /v1/extensions/vrpn/health | Same state including listener_ready, sample_sequence and tracked_models |
| PATCH | /v1/extensions/vrpn/config | Revision-checked actual application by the existing science worker |

Mutation body is `{expected_revision:1,persist:false,operation_timeout_ms:5000,
config:{publish_rate:120}}`. The world host owns the 202 operation receipt,
request identity, replay and terminal `/v1/operations/{id}/wait`. Native application
updates desired/applied revisions; persisted is null. Unknown fields, wrong
JSON types, stale CAS and persistence attempts fail. Port and bind_address are
startup-only and cannot rebind a live listener. Structured top-level config
objects are replaced atomically; scalar fields merge with the authored document.
Reapplying the same document acknowledges a new revision without resetting
measurement state. Changed configuration explicitly reports
measurement_state_reset, using the existing noise/delay/filter implementations.

Live fields retain their domain meanings: publish_rate is report Hz (0.1–1000),
scan_interval is model-discovery wall seconds (0.001–60), stale_timeout and
derivative_reset_timeout are seconds; velocity_filter_cutoff and
acceleration_filter_cutoff are Hz. Model tracker mappings, body-to-tracker
extrinsics, mocap_noise and delay keep their original science definitions.
Mocap noise uses configured deterministic seed, position metre standard
deviations and rotation radian standard deviations. Delay uses milliseconds,
seeded common/per-tracker slow, jitter and burst history selection; its existing
bounded max_delay_ms and history_margin_ms validation remains in the domain.
YAML input is a regular absolute file <=64 KiB, closed fields, bounded nesting,
containers and expanded nodes. SDK JSON never enters the sampling callback.

VRPN reports preserve simulation source timestamps. UpdateEnd and TimeReset
owners publish native simulation nanoseconds into an atomic cache. Captured
pose and time are copied into one immutable snapshot; the science worker never
reads World::SimTime. Capture-rate limiting, stale diagnostics and history
selection retain wall time. send_time uses the latest captured simulation time;
sample_time uses the selected historical sample's simulation time. Delay/noise,
seed/filter mathematics and timestamp units have not been rewritten.

Native factory completion metadata public_entity_id supplies stable raw tracker
identity for simulation-v1-created models; opaque native names are diagnostic.
Authored world model names retain their existing raw names. Automatic mapping
recognizes the existing numbered uavN, ugvN and mecanumN identities.

vrpn_client.launch is only an optional user data consumer, for example the six
calibration markers; it is not a canonical vehicle localization path. Consumers
of this native source should explicitly select use_server_time true. Hybrid
config retains delay.timestamp_policy sample_time, including when delay is off.

The build exports the core and SystemPlugin libraries and depends on the shared
xgc2_gazebo_scene world authority, JsonCpp, yaml-cpp, VRPN server and xgc2_math.
C++20/GCC >=11 is required by the world SDK ABI. A matched published ROS1 image
must supply that toolchain; native-host compilation does not certify Focal ABI.

The existing pure noise, delay, wire-clock and identity tests remain.
test/server_config_native_test.cpp verifies real authored YAML and strict
configuration parsing. Earlier isolated native evidence observed one world
endpoint, actual VRPN listening, public entity position and simulation timestamps,
CAS and real configuration acknowledgement. No new native execution follows the
user's hard stop. Updated native-time cache, cleanup and deployment packages
remain unverified at runtime; no live station is operated.
