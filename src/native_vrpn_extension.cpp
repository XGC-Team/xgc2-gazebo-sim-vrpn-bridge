#include "gazebo_sim_vrpn_bridge/native_vrpn_extension.h"
#include <set>
namespace gazebo_sim_vrpn_bridge {
namespace {
using Error=xgc2_gazebo_scene::NativeExtensionError;
void Fields(const Json::Value& body,std::initializer_list<const char*> fields){
 if(!body.isObject())throw Error(400,"invalid_argument","VRPN body must be an object");
 for(const auto& name:body.getMemberNames()){bool known=false;for(auto field:fields)known|=name==field;if(!known)throw Error(400,"invalid_argument","unknown VRPN field: "+name);}
}
std::uint64_t UInt(const Json::Value& value){if((value.type()!=Json::intValue&&value.type()!=Json::uintValue)||!value.isUInt64()||value.asUInt64()>9007199254740991ULL)throw Error(400,"invalid_argument","VRPN revision must be an unsigned integer");return value.asUInt64();}
}
NativeVrpnState::NativeVrpnState(ServerConfig values,Json::Value document):live(std::make_shared<const ServerConfig>(std::move(values))),desired(document),bootstrap(std::move(document)){}
void NativeVrpnState::Stop()noexcept{{std::lock_guard<std::mutex> lock(mutex);stopped.store(true);}changed.notify_all();}
Json::Value NativeVrpnExtension::Describe()const{
 std::lock_guard<std::mutex> lock(state_->mutex);Json::Value result;
 result["id"]="vrpn";result["kind"]="native-vrpn-source";
 result["desired"]["revision"]=Json::UInt64(state_->desired_revision);result["desired"]["config"]=state_->desired;
 result["applied"]["revision"]=Json::UInt64(state_->applied_revision);result["applied"]["config"]=state_->applied;
 result["persisted"]=Json::nullValue;result["pending"]=bool(state_->pending);
 result["health"]["listener_ready"]=state_->ready.load();result["health"]["stopped"]=state_->stopped.load();
 result["health"]["tracked_models"]=Json::UInt64(state_->tracked_models.load());result["health"]["sample_sequence"]=Json::UInt64(state_->sample_sequence.load());
 if(!state_->error.empty())result["health"]["error"]=state_->error;
 result["configuration_schema"]["persist_supported"]=false;
 for(auto field:{"port","bind_address"})result["configuration_schema"]["startup_only"].append(field);
 result["configuration_schema"]["live_fields"]=Json::arrayValue;
 for(auto field:{"publish_rate","stale_timeout","scan_interval","velocity_filter_cutoff","acceleration_filter_cutoff","derivative_reset_timeout","mocap_noise","delay","auto_mapping","manual_mapping","robots","extrinsics","default_body_to_tracker","trackers","match_mode","auto_track_known_models","mocap_noise_enabled","mocap_noise_seed","delay_enabled","delay_timestamp_policy","delay_seed","enabled_trackers"})result["configuration_schema"]["live_fields"].append(field);
 result["limits"]["configuration_bytes"]=65536;result["limits"]["pending_commands"]=1;
 return result;
}
std::shared_ptr<const xgc2_gazebo_scene::PreparedWorldExtension> NativeVrpnExtension::Prepare(const std::string& method,const std::string& route,const Json::Value& body){
 auto command=std::make_shared<NativeVrpnCommand>();
 if(method=="GET"&&(route=="/v1/extensions/vrpn"||route=="/v1/extensions/vrpn/config"||route=="/v1/extensions/vrpn/health")){Fields(body,{});return command;}
 if(method!="PATCH"||route!="/v1/extensions/vrpn/config")throw Error(404,"not_found","unknown native VRPN route");
 Fields(body,{"expected_revision","persist","config","operation_timeout_ms"});
 command->expected_revision=UInt(body["expected_revision"]);
 if(body.isMember("persist")&&(!body["persist"].isBool()||body["persist"].asBool()))throw Error(400,"invalid_argument","native VRPN configuration supports persist:false only");
 if(!body["config"].isObject()||body["config"].empty())throw Error(400,"invalid_argument","VRPN config must be a nonempty object");
 for(auto field:{"port","bind_address"})if(body["config"].isMember(field))throw Error(422,"unsupported",std::string(field)+" is startup-only");
 {std::lock_guard<std::mutex> lock(state_->mutex);command->document=state_->desired;}
 for(const auto& name:body["config"].getMemberNames())command->document[name]=body["config"][name];
 Json::StreamWriterBuilder writer;writer["indentation"]="";if(Json::writeString(writer,command->document).size()>65536)throw Error(429,"resource_exhausted","VRPN configuration exceeds 64 KiB");
 try{command->values=loadServerConfig(command->document);}catch(const std::exception& error){throw Error(400,"invalid_argument",error.what());}
 command->mutation=true;return command;
}
Json::Value NativeVrpnExtension::Execute(const std::shared_ptr<const xgc2_gazebo_scene::PreparedWorldExtension>& prepared,bool* effects_started){
 auto command=std::dynamic_pointer_cast<const NativeVrpnCommand>(prepared);if(!command)throw Error(400,"invalid_argument","missing typed native VRPN command");
 if(!command->mutation)return Describe();
 bool reset=false;
 {
  std::unique_lock<std::mutex> lock(state_->mutex);
  if(state_->stopped.load()||!state_->error.empty())throw Error(503,"unavailable","native VRPN data worker is unavailable");
  if(command->expected_revision!=state_->desired_revision)throw Error(409,"conflict","native VRPN revision differs");
  if(state_->pending)throw Error(429,"resource_exhausted","native VRPN config slot is busy");
  reset=state_->desired!=command->document;
  if(effects_started)*effects_started=true;
  state_->desired=command->document;++state_->desired_revision;state_->pending=command;
  state_->changed.notify_all();
  state_->changed.wait(lock,[&]{return !state_->pending||state_->stopped.load()||!state_->error.empty();});
  if(state_->stopped.load()||!state_->error.empty())throw Error(503,"unavailable","native VRPN config application was interrupted");
 }
 auto result=Describe();result["effects"]["applied"]=true;result["effects"]["measurement_state_reset"]=reset;return result;
}
}
