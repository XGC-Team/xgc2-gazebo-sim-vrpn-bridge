#!/usr/bin/env python3
"""Actual Gazebo plugin wire integration; run only in an isolated network namespace."""
import argparse
import collections
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
from xgc2_xrpc import Client, Fault, Runtime, TransportError


def ports(count):
    held=[]
    try:
        for _ in range(count):
            s=socket.socket(); s.bind(('127.0.0.1',0)); held.append(s)
        return [s.getsockname()[1] for s in held]
    finally:
        for s in held:s.close()


def check_released(port):
    for kind in (socket.SOCK_STREAM,socket.SOCK_DGRAM):
        with socket.socket(socket.AF_INET,kind) as s:
            s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            s.bind(('127.0.0.1',port))


def main(args):
    with tempfile.TemporaryDirectory(prefix='xgc-vrpn-wire-') as tmp:
        root=Path(tmp); shared,p1,p2,master=ports(4); endpoint=root/'world.sock'
        cfg={'port':shared,'bind_address':'127.0.0.1','publish_rate':80.,'scan_interval':.01,
             'auto_track_known_models':True,'tracker_ports':{'uav1':p1,'uav2':p2},
             'mocap_noise':{'enabled':True,'seed':9,'position_stddev_xyz':[.001,.002,.003],'rotation_stddev_rpy':[0.,0.,0.]},
             'delay':{'enabled':True,'timestamp_policy':'sample_time','max_delay_ms':30.,'common':{'base_ms':5.}}}
        if args.legacy: cfg.pop('tracker_ports')
        (root/'vrpn.yaml').write_text(json.dumps(cfg))
        world=root/'fixture.world'
        world.write_text(f'''<sdf version="1.6"><world name="wire_fixture"><physics type="ode"><max_step_size>0.01</max_step_size><real_time_update_rate>100</real_time_update_rate></physics><gravity>0 0 0</gravity><plugin name="authority" filename="{args.world_plugin}"><socket_path>{endpoint}</socket_path><target_id>vrpn-wire-fixture</target_id><resource_root>{root}</resource_root><required_component>vrpn</required_component></plugin></world></sdf>''')
        env=dict(os.environ);env.pop('DISPLAY',None);env.pop('WAYLAND_DISPLAY',None)
        env.update(GAZEBO_MASTER_URI=f'http://127.0.0.1:{master}',GAZEBO_MODEL_DATABASE_URI='',
                   GAZEBO_PLUGIN_PATH=str(Path(args.world_plugin).parent),GAZEBO_LOG_PATH=str(root/'logs'),XGC_SIM_VRPN_CONFIG=str(root/'vrpn.yaml'))
        process=None; runtime=Runtime(blocking_workers=1,max_calls=8,max_connections=8); discovery=Client(str(endpoint),runtime=runtime);client=None
        try:
            with (root/'gazebo.log').open('wb') as log:
                process=subprocess.Popen([args.gzserver,'--verbose','-s',args.system_plugin,str(world)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+20
            while True:
                try: desc=discovery.json('/v1/describe',method='GET',timeout=1);break
                except (TransportError,OSError): pass
                if process.poll() is not None or time.monotonic()>deadline: raise RuntimeError((root/'gazebo.log').read_text()[-5000:])
                time.sleep(.05)
            client=Client(str(endpoint),runtime=runtime,instance_id=(desc.get('service_ref') or desc['facts']['service_ref'])['instance_id'])
            deadline=time.monotonic()+10
            while True:
                try:
                    status=client.json('/v1/extensions/vrpn',method='GET',timeout=1)
                    if status['health']['listener_ready']:break
                except Fault:pass
                if time.monotonic()>deadline: raise RuntimeError((root/'gazebo.log').read_text()[-5000:])
                time.sleep(.05)
            artifact='<sdf version="1.6"><model name="authored"><static>true</static><link name="body"/></model></sdf>'
            for i in (1,2):
                body={'entity':{'id':f'uav{i}','role':'robot','pose':{'position':[i*3-2,i*3-1,i*3],'orientation':[0,0,0,1]},'asset':{'id':f'body{i}','realization':{'media_type':'application/sdf+xml','content':artifact}}},'operation_timeout_ms':5000}
                op=client.json('/v1/entities',body,method='POST',request_id=f'entity-{i}')
                result=client.json('/v1/operations/'+op['id']+'/wait',{},timeout=8)
                assert result['state']=='succeeded',result
            time.sleep(.5)
            try:
                client.json('/v1/extensions/vrpn/config',{'expected_revision':1,'config':{'tracker_ports':{'uav1':p2}},'operation_timeout_ms':5000},method='PATCH')
            except Fault as error: assert error.status==422,error.status
            else: raise AssertionError('tracker_ports must be startup-only')
            if args.legacy:
                for mode in ('udp','tcp'):
                    reader=subprocess.run([args.reader,('tcp://' if mode=='tcp' else '')+f'127.0.0.1:{shared}','1400'],capture_output=True,text=True,timeout=5)
                    assert reader.returncode==0,(reader.stdout,reader.stderr)
                    for name in ('uav1','uav2'):
                        for kind in ('pose','velocity','acceleration'):
                            assert f'WIRE {name} {kind} ' in reader.stdout,(mode,name,kind)
                    check_released(p1);check_released(p2)
                print('PASS default shared endpoint carries both trackers over UDP and TCP; startup-only')
                return
            evidence={}
            for mode in ('udp','tcp'):
                readers={name:subprocess.Popen([args.reader,('tcp://' if mode=='tcp' else '')+f'127.0.0.1:{port}','2800'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
                         for name,port in [('shared',shared),('uav1',p1),('uav2',p2)]}
                data={};stats={}
                for name,reader in readers.items():
                    out,err=reader.communicate(timeout=8);assert reader.returncode==0,(name,out,err)
                    records=collections.defaultdict(set)
                    for line in out.splitlines():
                        parts=line.split()
                        if parts and parts[0]=='WIRE':records[(parts[1],parts[2])].add((parts[3],parts[4],parts[5]))
                        if parts and parts[0]=='SOCKET':stats[name]=parts[1:]
                    data[name]=records
                matches={}
                for own in ('uav1','uav2'):
                    other='uav2' if own=='uav1' else 'uav1'
                    for kind in ('pose','velocity','acceleration'):
                        assert len(data[own][(own,kind)])>=5,(mode,own,kind,data[own],stats)
                        assert not data[own][(other,kind)],(mode,own,other,kind)
                        matches[own+'/'+kind]=len(data[own][(own,kind)] & data['shared'][(own,kind)])
                        assert matches[own+'/'+kind]>=5,(mode,own,kind,'fan-out differs')
                        if kind=='pose':
                            xyz=[struct.unpack('!ddd',bytes.fromhex(record[2])[8:32]) for record in data[own][(own,kind)]]
                            expected=(1.,2.,3.) if own=='uav1' else (4.,5.,6.)
                            assert all(all(abs(a-b)<.03 for a,b in zip(actual,expected)) for actual in xyz),(own,xyz[:2])
                            assert len(set(xyz))>1,'noise fixture must vary'
                    udp_reads=int(stats[own][0].split('=')[1])
                    assert (udp_reads>0 if mode=='udp' else udp_reads==0),(mode,own,stats)
                    assert int(stats[own][1].split('=')[1])>0,(mode,own,stats)
                evidence[mode]={'counts':{name:{'/'.join(key):len(value) for key,value in records.items()} for name,records in data.items()},'socket_reads':stats,'matching_payload_and_timestamp':matches}
            # Keep uav2 connected throughout two independent uav1 sessions.
            survivor=subprocess.Popen([args.reader,f'tcp://127.0.0.1:{p2}','3200'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            for _ in range(2):
                reader=subprocess.run([args.reader,f'tcp://127.0.0.1:{p1}','1000'],capture_output=True,text=True,timeout=5)
                assert reader.returncode==0 and 'WIRE uav1 pose' in reader.stdout,(reader.stdout,reader.stderr)
            out,err=survivor.communicate(timeout=5)
            samples=[line.split() for line in out.splitlines() if line.startswith('WIRE uav2 pose ')]
            assert survivor.returncode==0 and len(samples)>100,(len(samples),err)
            assert (int(samples[-1][3])-int(samples[0][3]) + (int(samples[-1][4])-int(samples[0][4]))/1e6)>2.,samples[-1]
            print(json.dumps(evidence,sort_keys=True),flush=True)
        finally:
            if client:client.close()
            discovery.close();runtime.close()
            if process and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(3);raise RuntimeError('Gazebo did not drain')
            if process and process.poll() is not None:
                for port in (shared,p1,p2):check_released(port)
        for port in (shared,p1,p2):check_released(port)
        # Conflict on either transport must be explicit and must unwind every
        # listener already opened by the partially constructed worker.
        for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
            with socket.socket(socket.AF_INET,kind) as blocker:
                blocker.bind(('127.0.0.1',p1))
                if kind==socket.SOCK_STREAM:blocker.listen(1)
                conflict_endpoint=root/('conflict-'+str(kind)+'.sock')
                conflict_world=root/('conflict-'+str(kind)+'.world')
                conflict_world.write_text(world.read_text().replace(str(endpoint),str(conflict_endpoint)))
                env['GAZEBO_MASTER_URI']=f"http://127.0.0.1:{ports(1)[0]}"
                conflict_log=root/('conflict-'+str(kind)+'.log')
                with conflict_log.open('wb') as log:
                    conflict=subprocess.Popen([args.gzserver,'--verbose','-s',args.system_plugin,str(conflict_world)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    deadline=time.monotonic()+12
                    while time.monotonic()<deadline:
                        output=conflict_log.read_text(errors='replace')
                        if f'failed to create VRPN listener on port {p1}' in output:break
                        if conflict.poll() is not None:break
                        time.sleep(.05)
                    assert f'failed to create VRPN listener on port {p1}' in output,(kind,output[-5000:])
                    check_released(shared);check_released(p2)
                finally:
                    if conflict.poll() is None:
                        os.killpg(conflict.pid,signal.SIGINT)
                        try:conflict.wait(8)
                        except subprocess.TimeoutExpired:
                            os.killpg(conflict.pid,signal.SIGKILL);conflict.wait(3);raise RuntimeError('conflicted Gazebo did not drain')
            for port in (shared,p1,p2):check_released(port)
        print('PASS shared/dedicated UDP and TCP wire, sample identity, independent reconnect, release, TCP/UDP conflicts')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--legacy',action='store_true');parser.add_argument('--world-plugin',required=True);parser.add_argument('--system-plugin',required=True);parser.add_argument('--reader',required=True);parser.add_argument('--gzserver',default='/usr/bin/gzserver')
    main(parser.parse_args())
