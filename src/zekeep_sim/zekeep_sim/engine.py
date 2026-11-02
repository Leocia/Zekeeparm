"""Isolated MuJoCo physics. No serial, SDK, or hardware command imports."""
from collections import deque
import base64
import math
import struct
import threading
import time
import uuid
import zlib
import numpy as np
import mujoco
from .model import build_model, quaternion

ARM = [f'joint{i}' for i in range(1,7)]
READY = [0.0, 0.698131701, 0.698131701, -0.610865238, 0.0, 0.0]


def png(rgb):
    height, width, _ = rgb.shape
    def chunk(name, data):
        return struct.pack('>I',len(data))+name+data+struct.pack('>I',zlib.crc32(name+data)&0xffffffff)
    rows = b''.join(b'\0'+row.tobytes() for row in rgb)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(rows))+chunk(b'IEND',b'')


class Simulation:
    def __init__(self, bringup, *, friction=1.2, object_mass=.04, table_z=0, camera=True):
        self.model = build_model(bringup,friction=friction,object_mass=object_mass,table_z=table_z)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.closed = threading.Event()
        self.busy = False
        self.mode = 'physics'
        self.last_mirror = 0.0
        self.phase = 'idle'
        self.result = None
        self.task_id = ''
        self.heartbeat = time.monotonic()
        self.camera_enabled = camera
        self.image = None
        self.detections = []
        self.detection_history = deque(maxlen=8)
        self.camera_error = ''
        self.capture_at = 0
        self.table_z = table_z
        self.arm_q = [self.model.joint(name).qposadr[0] for name in ARM]
        self.arm_v = [self.model.joint(name).dofadr[0] for name in ARM]
        self.act = [self.model.actuator(name).id for name in ARM]
        self.fingers = [self.model.actuator(name).id for name in ('gripper_joint','right_joint')]
        self.tcp = self.model.site('tcp').id
        self.object = self.model.body('red').id
        self.obj_q = self.model.joint('red_free').qposadr[0]
        self.reset()

    def reset(self):
        with self.lock:
            mujoco.mj_resetData(self.model,self.data)
            self.data.qpos[self.arm_q]=READY
            self.data.ctrl[self.act]=READY
            for name in ('gripper_joint','right_joint'):
                self.data.qpos[self.model.joint(name).qposadr[0]]=.035
            self.data.ctrl[self.fingers]=.035
            mujoco.mj_forward(self.model,self.data)
            self.mode='physics'
            self.phase,self.result='idle',None
            self.detections=[]
            self.detection_history.clear()

    def contact(self):
        touched=set()
        obj=self.model.geom('red_object').id
        pads={self.model.geom('left_link_contact').id,self.model.geom('right_link_contact').id}
        for index,c in enumerate(self.data.contact[:self.data.ncon]):
            if obj in (c.geom1,c.geom2):
                other=c.geom2 if c.geom1==obj else c.geom1
                if other in pads:
                    force=np.zeros(6)
                    mujoco.mj_contactForce(self.model,self.data,index,force)
                    if force[0]>.02:
                        touched.add(other)
        return len(touched)==2

    def state(self, refresh=True):
        if refresh:self.heartbeat=time.monotonic()
        with self.lock:
            if self.mode=='mirror':
                self.phase='mirror' if time.monotonic()-self.last_mirror<=.5 else 'mirror_stale'
            angles={name:float(self.data.qpos[index]) for name,index in zip(ARM,self.arm_q)}
            angles['gripper']=float(sum(self.data.qpos[self.model.joint(name).qposadr[0]] for name in ('gripper_joint','right_joint')))
            return {'ok':True,'source':'mujoco','namespace':'/zekeep_sim','busy':self.busy,'phase':self.phase,
                    'mode':self.mode,'task_id':self.task_id,'result':self.result,'joints':angles,
                    'tcp_ros':{'x':float(self.data.site_xpos[self.tcp,0]),'y':float(self.data.site_xpos[self.tcp,1]),'z':float(self.data.site_xpos[self.tcp,2])},
                    'objects':[{'color':'red','position':self.data.xpos[self.object].tolist(), 'quaternion':self.data.xquat[self.object].tolist()}],
                    'contact':self.contact(),'detections':list(self.detections) if time.monotonic()-self.capture_at<1 else [],
                    'camera_error':self.camera_error,'simulation_time':float(self.data.time)}

    def hold(self):
        with self.lock:
            self.data.ctrl[self.act]=self.data.qpos[self.arm_q]
            for act,name in zip(self.fingers,('gripper_joint','right_joint')):
                self.data.ctrl[act]=self.data.qpos[self.model.joint(name).qposadr[0]]

    def start(self):
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.thread.start()

    def run(self):
        renderer=None
        if self.camera_enabled:
            try:
                renderer=mujoco.Renderer(self.model,height=240,width=320)
            except Exception as exc:
                self.camera_error=str(exc)
        last_camera=0
        try:
            while not self.closed.is_set():
                begin=time.monotonic()
                if self.busy and begin-self.heartbeat>3:
                    self.cancel.set()
                with self.lock:
                    if self.cancel.is_set():
                        self.hold()
                    if self.mode == 'physics':
                        for _ in range(5):
                            mujoco.mj_step(self.model,self.data)
                    if renderer is not None and begin-last_camera>.2:
                        try:
                            self.render(renderer)
                        except Exception as exc:
                            self.camera_error=str(exc)
                        last_camera=begin
                self.closed.wait(max(0,.01-(time.monotonic()-begin)))
        finally:
            if renderer is not None:
                renderer.close()

    def render(self,renderer):
        renderer.update_scene(self.data,camera='overhead')
        rgb=renderer.render().copy()
        renderer.enable_depth_rendering()
        depth=renderer.render().copy()
        renderer.disable_depth_rendering()
        red=(rgb[:,:,0]>100)&(rgb[:,:,0]>rgb[:,:,1]*1.7)&(rgb[:,:,0]>rgb[:,:,2]*1.7)
        rows,cols=np.where(red)
        self.detection_history.extend((self.capture_at,item) for item in self.detections)
        self.capture_at=time.monotonic()
        self.detections=[]
        if len(rows)>8:
            u,v=float(np.median(cols)),float(np.median(rows))
            z=float(np.median(depth[red]))
            f=240/(2*math.tan(math.radians(55)/2))
            camera=self.model.camera('overhead').id
            point=np.array([(u-159.5)*z/f,-(v-119.5)*z/f,-z])
            world=self.data.cam_xpos[camera]+self.data.cam_xmat[camera].reshape(3,3)@point
            if np.isfinite(world).all():
                self.detections=[{'id':uuid.uuid4().hex,'color':'red','position':world.tolist(),'source':'virtual-rgbd','captured_at':time.time()}]
        self.last_rgb=rgb
        self.image={'ok':True,'source':'mujoco-overhead','captured_at':time.time(),'width':320,'height':240,
                    'image':'data:image/png;base64,'+base64.b64encode(png(rgb)).decode()}

    def check(self):
        if self.cancel.is_set() or self.closed.is_set():
            raise RuntimeError('simulation task cancelled; held')

    def wait(self,predicate,timeout=6):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.check()
            with self.lock:
                if predicate():return
            time.sleep(.02)
        raise RuntimeError('simulation action timed out')

    def move_joints(self,q,speed=.5):
        q=np.asarray(q,dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('six finite joints required')
        limits=np.array([self.model.joint(name).range for name in ARM])
        if np.any(q<limits[:,0]) or np.any(q>limits[:,1]):raise ValueError('joint limit')
        with self.lock:start=self.data.qpos[self.arm_q].copy()
        duration=max(.2,float(np.max(abs(q-start)))/speed)
        begin=time.monotonic()
        while True:
            self.check()
            u=min(1,(time.monotonic()-begin)/duration)
            with self.lock:self.data.ctrl[self.act]=start+(q-start)*u
            if u>=1:break
            time.sleep(.01)
        self.wait(lambda:np.max(abs(self.data.qpos[self.arm_q]-q))<.025,timeout=4)

    def solve(self,position,rotation=None):
        d=mujoco.MjData(self.model)
        with self.lock:d.qpos[:]=self.data.qpos
        target=np.asarray(position,dtype=float)
        for _ in range(400):
            mujoco.mj_forward(self.model,d)
            error=target-d.site_xpos[self.tcp]
            orientation=np.zeros(3)
            if rotation is not None:
                current=d.site_xmat[self.tcp].reshape(3,3)
                orientation=.5*sum((np.cross(current[:,i],rotation[:,i]) for i in range(3)),np.zeros(3))
            if np.linalg.norm(error)<.0015 and np.linalg.norm(orientation)<.01:return d.qpos[self.arm_q].copy()
            jp,jr=np.zeros((3,self.model.nv)),np.zeros((3,self.model.nv))
            mujoco.mj_jacSite(self.model,d,jp,jr,self.tcp)
            jac=np.vstack((jp[:,self.arm_v],jr[:,self.arm_v])) if rotation is not None else jp[:,self.arm_v]
            e=np.r_[error,orientation] if rotation is not None else error
            delta=jac.T@np.linalg.solve(jac@jac.T+.001*np.eye(len(e)),e)
            limits=np.array([self.model.joint(name).range for name in ARM])
            d.qpos[self.arm_q]=np.clip(d.qpos[self.arm_q]+np.clip(delta,-.08,.08),limits[:,0],limits[:,1])
        raise RuntimeError('simulation IK unreachable')

    def move_tcp(self,point,rotation):
        with self.lock:start=self.data.site_xpos[self.tcp].copy()
        # Cartesian segments preserve pad orientation and avoid a sweeping joint-space descent.
        count=max(1,int(np.linalg.norm(np.asarray(point)-start)/.015))
        for index in range(1,count+1):
            q=self.solve(start+(np.asarray(point)-start)*index/count,rotation)
            self.move_joints(q,.6)

    def gripper(self,width,contact=False):
        if not isinstance(width,(int,float)) or not math.isfinite(width) or not 0<=width<=.07:raise ValueError('gripper width must be 0–0.07m')
        with self.lock:self.data.ctrl[self.fingers]=width/2
        if contact:
            self.wait(self.contact,3)
        else:
            self.wait(lambda:all(abs(self.data.qpos[self.model.joint(n).qposadr[0]]-width/2)<.002 for n in ('gripper_joint','right_joint')),3)

    def pick_place(self,detection_id):
        with self.lock:
            candidates=[(self.capture_at,d) for d in self.detections]+list(self.detection_history)
            targets=[d for stamp,d in candidates if d['id']==detection_id and time.monotonic()-stamp<=.8]
            if not targets or not self.detections or time.monotonic()-self.capture_at>.8:raise ValueError('virtual detection stale or unknown')
            if np.linalg.norm(np.array(targets[0]['position'])-self.detections[0]['position'])>.01:raise ValueError('virtual target moved')
            # Surface depth centroid; cube height is a measured scene parameter (50mm).
            position=np.array(targets[0]['position']);position[2]-=.025
        fixed=np.zeros(9)
        mujoco.mju_quat2Mat(fixed,np.fromstring(quaternion('0 -1.5708 3.1415'),sep=' '))
        rotation=np.diag([1,-1,-1])@fixed.reshape(3,3)
        above=position+np.array([0,0,.12])
        self.phase='pregrasp';self.gripper(.07);self.move_joints(self.solve(above,rotation),.4)
        self.phase='descend';self.move_tcp(position,rotation)
        self.phase='contact';self.gripper(0,contact=True)
        with self.lock:initial_z=float(self.data.xpos[self.object,2])
        self.phase='lift';self.move_tcp(above,rotation)
        with self.lock:
            if not self.contact() or self.data.xpos[self.object,2]<initial_z+.07:raise RuntimeError('object not held after lift')
        destination=position+np.array([.06,0,0])
        self.phase='carry';self.move_tcp(destination+np.array([0,0,.12]),rotation)
        self.phase='place';self.move_tcp(destination+np.array([0,0,.005]),rotation);self.gripper(.07)
        self.phase='retreat';self.move_tcp(destination+np.array([0,0,.12]),rotation)
        time.sleep(.25);self.check()
        with self.lock:
            actual=self.data.xpos[self.object].copy()
            if np.linalg.norm(actual[:2]-destination[:2])>.04 or abs(actual[2]-position[2])>.02:
                raise RuntimeError('object placement unconfirmed')
        return {'success':True,'message':'physical contact, lift and placement verified','position':actual.tolist()}

    def command(self,body):
        if not isinstance(body,dict) or 'command' not in body:raise ValueError('command required')
        name=body['command']
        fields={'stop':{'command'},'reset':{'command'},'joints':{'command','positions'},'gripper':{'command','width_m'},'pick_place':{'command','detection_id'},'mirror':{'command','positions','width_m'}}
        if name not in fields or set(body)!=fields[name]:raise ValueError('command or fields not allowed')
        if name=='stop':
            self.cancel.set();self.hold()
            return {'ok':True,'text':'simulation held; task cancellation requested'}
        if name=='mirror':
            q=np.asarray(body['positions'],dtype=float)
            limits=np.array([self.model.joint(n).range for n in ARM])
            width=body['width_m']
            if q.shape!=(6,) or not np.isfinite(q).all() or np.any(q<limits[:,0]) or np.any(q>limits[:,1]):raise ValueError('mirror joint bounds')
            if not isinstance(width,(int,float)) or not math.isfinite(width) or not 0<=width<=.07:raise ValueError('mirror gripper bounds')
            with self.lock:
                if self.busy:raise RuntimeError('simulation busy')
                self.mode='mirror';self.last_mirror=time.monotonic();self.data.qpos[self.arm_q]=q;self.data.ctrl[self.act]=q;self.data.qvel[:]=0
                for act,n in zip(self.fingers,('gripper_joint','right_joint')):
                    self.data.qpos[self.model.joint(n).qposadr[0]]=width/2;self.data.ctrl[act]=width/2
                mujoco.mj_forward(self.model,self.data)
            return {'ok':True,'mode':'mirror'}
        if self.mode=='mirror' and name!='reset':raise RuntimeError('reset scene to leave read-only mirror mode')
        with self.lock:
            if self.busy:raise RuntimeError('simulation task busy')
            self.cancel.clear();self.busy=True;self.heartbeat=time.monotonic()
            self.task_id=uuid.uuid4().hex;self.result=None;self.phase='accepted'
        def worker():
            try:
                if name=='reset':self.reset()
                elif name=='joints':self.move_joints(body['positions'])
                elif name=='gripper':self.gripper(body['width_m'])
                else:
                    self.result=self.pick_place(body['detection_id'])
                if self.result is None:self.result={'success':True,'message':'simulation command completed'}
                self.phase='completed'
            except Exception as exc:
                self.hold();self.phase='failed';self.result={'success':False,'message':str(exc)}
            finally:self.busy=False
        threading.Thread(target=worker,daemon=True).start()
        return {'ok':True,'accepted':True,'mode':self.mode,'task_id':self.task_id}
