#!/usr/bin/env python3
"""Observe headers/tracks; optional verified camera-stop control, no publishers or robot motion."""
import argparse,json,time,os,signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

def main():
    a=argparse.ArgumentParser();a.add_argument('output');a.add_argument('--seconds',type=float,default=40);a.add_argument('--camera-stop-identity');v=a.parse_args()
    out=Path(v.output);out.mkdir(parents=True,exist_ok=True);assert not(out/'events.jsonl').exists()
    rclpy.init();n=Node('cam6dof_freshness_watch');f=(out/'events.jsonl').open('w');counts={};start=time.time()
    def cb(ns,kind,m):
        d={'receipt':time.time(),'camera':ns,'kind':kind};counts[ns+'/'+kind]=counts.get(ns+'/'+kind,0)+1
        if kind=='tracks':
            d['payload']=json.loads(m.data)
            if ns=='rgbd2' and d['payload'] and not(out/'confirmed_ready.json').exists():
                (out/'confirmed_ready.json').write_text(json.dumps(d,indent=2)+'\n')
                if v.camera_stop_identity:
                    ident=json.loads(Path(v.camera_stop_identity).read_text());pid=ident['pid'];proc=Path('/proc')/str(pid)
                    cmd=proc.joinpath('cmdline').read_bytes().replace(b'\0',b' ').decode().strip()
                    ticks=proc.joinpath('stat').read_text().rsplit(')',1)[1].split()[19]
                    assert cmd==ident['cmd'] and ticks==ident['ticks'] and proc.stat().st_uid==ident['uid']==os.getuid()
                    assert '/realsense2_camera_node ' in cmd and '__node:=rgbd2 ' in cmd
                    os.kill(pid,signal.SIGINT)
                    (out/'camera_stop.json').write_text(json.dumps({'time':time.time(),'pid':pid,'signal':'SIGINT','identity_verified':True},indent=2)+'\n')

        else:d.update(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,encoding=m.encoding)
        f.write(json.dumps(d)+'\n');f.flush()
    for ns in ('rgbd','rgbd2'):
        for kind,topic in [('depth',f'/{ns}/depth'),('mask',f'/{ns}/seg/instance_segmentation')]:
            n.create_subscription(Image,topic,lambda m,s=ns,k=kind:cb(s,k,m),qos_profile_sensor_data)
        n.create_subscription(String,f'/loc_{ns}/tracks',lambda m,s=ns:cb(s,'tracks',m),qos_profile_sensor_data)
    deadline=time.monotonic()+v.seconds
    while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
    f.close();(out/'watch.json').write_text(json.dumps({'start':start,'end':time.time(),'counts':counts,'observer_only':not bool(v.camera_stop_identity),'camera_stop_identity':v.camera_stop_identity},indent=2)+'\n');print(counts)
    n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
