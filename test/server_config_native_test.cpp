#include "gazebo_sim_vrpn_bridge/server_config.h"
#include <cassert>
#include <iostream>
#include <stdexcept>
using namespace gazebo_sim_vrpn_bridge;
int main(int argc,char** argv){
 if(argc!=2)throw std::runtime_error("configuration directory required");
 const std::string root=argv[1];
 auto document=readServerConfigFile(root+"/vrpn_server.yaml");auto config=loadServerConfig(document);
 assert(config.publish_rate_hz==120. && config.port==3883 && config.mocap_noise.seed==1);
 assert(config.mocap_noise.position_stddev_m[0]==1e-7);
 auto simple=loadServerConfig(readServerConfigFile(root+"/vrpn_server_delay_simple.yaml"));
 auto complex=loadServerConfig(readServerConfigFile(root+"/vrpn_server_delay_complex.yaml"));
 assert(simple.delay.enabled && simple.delay.timestamp_policy==DelayTimestampPolicy::SendTime && simple.delay.max_delay_ms==30.);
 assert(complex.delay.enabled && complex.delay.timestamp_policy==DelayTimestampPolicy::SampleTime && complex.delay.max_delay_ms==30.);
 auto reject=[](const Json::Value& value){bool rejected=false;try{loadServerConfig(value);}catch(const std::exception&){rejected=true;}assert(rejected);};
 auto unknown=document;unknown["misspelled"]=1;reject(unknown);
 auto port=document;port["port"]=3883.0;reject(port);
 auto rate=document;rate["publish_rate"]=0;reject(rate);
 auto mapped=document;mapped["tracker_ports"]["uav1"]=23001;mapped["tracker_ports"]["uav2"]=23002;
 auto selected=loadServerConfig(mapped);assert(selected.tracker_ports.size()==2 && selected.tracker_ports.at("uav1")==23001);
 auto duplicate=mapped;duplicate["tracker_ports"]["uav2"]=23001;reject(duplicate);
 auto shared=mapped;shared["tracker_ports"]["uav2"]=3883;reject(shared);
 auto invalid=mapped;invalid["tracker_ports"]["uav1"]=0;reject(invalid);
 invalid=mapped;invalid["tracker_ports"]["uav1"]=true;reject(invalid);
 invalid=mapped;invalid["tracker_ports"]["uav1"]=23001.0;reject(invalid);
 invalid=document;invalid["tracker_ports"]=Json::arrayValue;reject(invalid);
 invalid=mapped;invalid["tracker_ports"]["/bad"]=23003;reject(invalid);
 auto mapping=document;mapping["auto_track_known_models"]=true;
 assert(loadServerConfig(mapping).trackerNameForGazeboModel("uav1")=="uav1");
 assert(loadServerConfig(mapping).trackerNameForGazeboModel("not_a_robot").empty());
 std::cout<<"PASS native VRPN authored config, scientific delay fixtures, strict fields/types, canonical identity\n";
}
