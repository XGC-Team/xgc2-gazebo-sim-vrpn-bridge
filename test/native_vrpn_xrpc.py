#!/usr/bin/env python3
"""Isolated world SDK host + native VRPN extension + actual VRPN user data."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
from xgc2_xrpc import Client,Fault,Runtime,TransportError


def free_port():
    with socket.socket() as port:
        port.bind(("127.0.0.1",0))
        return port.getsockname()[1]


def main(args):
    with tempfile.TemporaryDirectory(prefix="sol6-native-vrpn-") as directory:
        root=Path(directory);endpoint=root/"world.sock";vrpn_port=free_port();master_port=free_port()
        configuration={"port":vrpn_port,"bind_address":"127.0.0.1","publish_rate":100.,"scan_interval":.01,
            "auto_track_known_models":True,"mocap_noise":{"enabled":False,"seed":1,"position_stddev_xyz":[0.,0.,0.],"rotation_stddev_rpy":[0.,0.,0.]}}
        config=root/"vrpn.yaml";config.write_text(json.dumps(configuration))
        world=root/"fixture.world";world.write_text(f'''<sdf version="1.6"><world name="vrpn_fixture">
        <physics type="ode"><max_step_size>0.01</max_step_size><real_time_update_rate>100</real_time_update_rate></physics><gravity>0 0 0</gravity>
        <plugin name="authority" filename="{args.world_plugin}"><socket_path>{endpoint}</socket_path><target_id>native-vrpn-fixture</target_id><resource_root>{root}</resource_root><required_component>vrpn</required_component></plugin>
        </world></sdf>''')
        environment=dict(os.environ)
        environment.pop("DISPLAY",None);environment.pop("WAYLAND_DISPLAY",None)
        environment.update(GAZEBO_MASTER_URI=f"http://127.0.0.1:{master_port}",GAZEBO_MODEL_DATABASE_URI="",
            GAZEBO_PLUGIN_PATH=str(Path(args.world_plugin).parent),GAZEBO_LOG_PATH=str(root/"logs"),XGC_SIM_VRPN_CONFIG=str(config))
        process=None;runtime=Runtime(blocking_workers=1,max_calls=8,max_connections=8);discovery=Client(str(endpoint),runtime=runtime);client=None
        try:
            with (root/"gazebo.log").open("wb") as log:
                process=subprocess.Popen([args.gzserver,"--verbose","-s",args.system_plugin,str(world)],env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+20
            while True:
                try:
                    description=discovery.json("/v1/describe",method="GET",timeout=1)
                    break
                except (TransportError,OSError):
                    pass
                if process.poll() is not None or time.monotonic()>=deadline:
                    raise RuntimeError("native world failed: "+(root/"gazebo.log").read_text()[-5000:])
                time.sleep(.025)
            client=Client(str(endpoint),runtime=runtime,instance_id=description["service_ref"]["instance_id"])
            route="/v1/extensions/vrpn"
            deadline=time.monotonic()+5
            while True:
                try:
                    status=client.json(route,method="GET",timeout=1)
                    if status["health"]["listener_ready"]:break
                except Fault as error:
                    if error.status!=404:raise
                if time.monotonic()>=deadline:raise RuntimeError("VRPN native initialization not acknowledged: "+json.dumps(status)+"\n"+(root/"gazebo.log").read_text()[-5000:])
                time.sleep(.025)
            assert status["desired"]["revision"]==status["applied"]["revision"]==1 and status["persisted"] is None,status
            sequence=0
            def mutate(path,body,method="POST"):
                nonlocal sequence
                sequence+=1
                operation=client.json(path,{**body,"operation_timeout_ms":5000},method=method,request_id=f"vrpn-mutation-{sequence}")
                return client.json("/v1/operations/"+operation["id"]+"/wait",{},timeout=8)
            artifact='<sdf version="1.6"><model name="authored"><static>true</static><link name="body"/></model></sdf>'
            created=mutate("/v1/entities",{"entity":{"id":"uav1","role":"robot","pose":{"position":[1,2,3],"orientation":[0,0,0,1]},"asset":{"id":"native-vrpn-body","realization":{"media_type":"application/sdf+xml","content":artifact}}}})
            assert created["state"]=="succeeded",created
            reader=subprocess.run([args.reader,f"uav1@127.0.0.1:{vrpn_port}"],capture_output=True,text=True,timeout=7)
            assert reader.returncode==0,(reader.stdout,reader.stderr,(root/"gazebo.log").read_text()[-2000:])
            reports=[json.loads(line) for line in reader.stdout.splitlines() if line.startswith("{")]
            assert len(reports)>=3 and all(report["position"]==[1,2,3] for report in reports),reports
            assert all(0<=report["timestamp"]<10000 for report in reports),reports
            changed=mutate(route+"/config",{"expected_revision":1,"persist":False,"config":{"publish_rate":120.}},"PATCH")
            assert changed["state"]=="succeeded",changed
            result=changed["result"]
            assert result["applied"]["revision"]==2 and result["applied"]["config"]["publish_rate"]==120.,changed
            assert result["effects"]["measurement_state_reset"],changed
            same=mutate(route+"/config",{"expected_revision":2,"persist":False,"config":{"publish_rate":120.}},"PATCH")
            assert same["state"]=="succeeded" and not same["result"]["effects"]["measurement_state_reset"],same
            stale=mutate(route+"/config",{"expected_revision":1,"persist":False,"config":{"publish_rate":80.}},"PATCH")
            assert stale["state"]=="failed" and stale["error"]["code"]=="conflict",stale
            for body,code in [({"expected_revision":3,"config":{"misspelled":1}},400),({"expected_revision":3,"persist":True,"config":{"publish_rate":80}},400),({"expected_revision":3,"config":{"port":3884}},422)]:
                try:client.json(route+"/config",{**body,"operation_timeout_ms":5000},method="PATCH")
                except Fault as error:assert error.status==code,(code,error.status)
                else:raise AssertionError("VRPN malformed mutation accepted")
            evidence={"world_service_ref":description["service_ref"],"management_endpoint_count":1,
                "process":{"pid":process.pid,"executable":str(Path(args.gzserver).resolve()),"command":[args.gzserver,"--verbose","-s",args.system_plugin,str(world)],"rootfs":"native host","display":None,"wayland_display":None,"gzclient":False},
                "actual_VRPN_reports":reports,"applied_revision":3,"gates":["native-world-startup-binding","actual-listener-ready","public-entity-identity","raw-simulation-wire-time","native-config-ack","same-config-preserves-state","CAS","strict-fields","persist-rejection","startup-binding-rejection"]}
            if args.evidence:Path(args.evidence).write_text(json.dumps(evidence,indent=2)+"\n")
            print(json.dumps(evidence),flush=True)
        finally:
            if client:client.close()
            discovery.close();runtime.close()
            if process and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(3)
                    raise RuntimeError("owned native VRPN world failed to drain after SIGINT")


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--world-plugin",required=True);parser.add_argument("--system-plugin",required=True)
    parser.add_argument("--reader",required=True);parser.add_argument("--gzserver",default="/opt/ros/noetic/opt/gazebo/bin/gzserver");parser.add_argument("--evidence")
    main(parser.parse_args())
