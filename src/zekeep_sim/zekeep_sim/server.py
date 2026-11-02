"""Loopback HTTP simulation and optional read-only ROS publishers."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading


def serve(simulation,port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def reply(self,code,value):
            data=json.dumps(value,ensure_ascii=False).encode()
            self.send_response(code);self.send_header('Content-Type','application/json')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)))
            self.end_headers();self.wfile.write(data)
        def do_GET(self):
            if self.path=='/state':self.reply(200,simulation.state())
            elif self.path=='/camera':
                with simulation.lock:self.reply(200,simulation.image or {'ok':False,'error':simulation.camera_error or 'camera pending'})
            else:self.reply(404,{'ok':False,'error':'unknown endpoint'})
        def do_POST(self):
            try:
                if self.headers.get('Origin') or self.headers.get('Content-Type','').split(';')[0]!='application/json':
                    raise ValueError('local JSON proxy required')
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=4096:raise ValueError('body size')
                if self.path!='/command':raise ValueError('unknown endpoint')
                self.reply(200,simulation.command(json.loads(self.rfile.read(size))))
            except Exception as exc:self.reply(400,{'ok':False,'error':str(exc)})
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    server.daemon_threads=True
    return server


def publish_ros(sim):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState,Image
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    rclpy.init()
    node=Node('zekeep_sim_feedback')
    joints=node.create_publisher(JointState,'/zekeep_sim/joint_states',10)
    objects=node.create_publisher(String,'/zekeep_sim/mujoco/object_states',10)
    detections=node.create_publisher(String,'/zekeep_sim/vision/detections',10)
    image=node.create_publisher(Image,'/zekeep_sim/camera/image_raw',10)
    tcp=node.create_publisher(PoseStamped,'/zekeep_sim/tcp_pose',10)
    def tick():
        # Read-only: this publisher never refreshes the browser task heartbeat.
        state=sim.state(refresh=False)
        stamp=node.get_clock().now().to_msg()
        message=JointState();message.header.stamp=stamp
        message.name=list(state['joints']);message.position=list(state['joints'].values());joints.publish(message)
        objects.publish(String(data=json.dumps(state['objects'])))
        detections.publish(String(data=json.dumps(state['detections'])))
        pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id='base_link'
        for key,value in state['tcp_ros'].items():setattr(pose.pose.position,key,value)
        with sim.lock:
            import numpy as np,mujoco
            q=np.zeros(4);mujoco.mju_mat2Quat(q,sim.data.site_xmat[sim.tcp])
            pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z=map(float,q)
        tcp.publish(pose)
        with sim.lock:
            if hasattr(sim,'last_rgb'):
                rgb=sim.last_rgb
                frame=Image();frame.header.stamp=stamp;frame.header.frame_id='overhead'
                frame.height,frame.width=rgb.shape[:2];frame.encoding='rgb8';frame.step=frame.width*3;frame.data=rgb.tobytes();image.publish(frame)
    node.create_timer(.1,tick)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bringup')
    parser.add_argument('--port',type=int,default=8083)
    parser.add_argument('--friction',type=float,default=1.2)
    parser.add_argument('--object-mass',type=float,default=.04)
    parser.add_argument('--table-z',type=float,default=0)
    parser.add_argument('--no-camera',action='store_true')
    parser.add_argument('--ros',action='store_true')
    args=parser.parse_args()
    if not args.bringup:
        from ament_index_python.packages import get_package_share_directory
        args.bringup=get_package_share_directory('zekeep_bringup')
    os.environ.setdefault('MUJOCO_GL','egl')
    from .engine import Simulation
    sim=Simulation(args.bringup,friction=args.friction,object_mass=args.object_mass,table_z=args.table_z,camera=not args.no_camera)
    sim.start()
    if args.ros:threading.Thread(target=publish_ros,args=(sim,),daemon=True).start()
    server=serve(sim,args.port)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        sim.cancel.set();sim.closed.set();sim.thread.join(timeout=5);server.server_close()

if __name__=='__main__':main()
