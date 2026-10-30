#!/usr/bin/env python3
"""Read-only recorder. IMU-time rows retain command ages and ESC availability."""
import collections
import csv
import json
import math
from pathlib import Path
import threading
import subprocess
import time
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, State, ESCStatus
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import String


class Recorder:
    def __init__(self):
        self.output=Path(rospy.get_param('~output')).expanduser()
        self.output.mkdir(parents=True,exist_ok=False)
        self.lock=threading.RLock(); self.latest={}; self.commands=collections.deque(maxlen=2000)
        self.previous=-1.; self.rows=0
        self.fields=['t','cmd_t','cmd_wx','cmd_wy','cmd_wz','cmd_thrust','wx','wy','wz',
                     'fx','fy','fz','px','py','pz','vx','vy','vz','qx','qy','qz','qw',
                     'rpm0','rpm1','rpm2','rpm3','voltage','armed','mode','segment','channel',
                     'phase','valid','flight_valid','rpm_valid']
        self.stream=(self.output/'samples.csv').open('x',newline='')
        self.writer=csv.DictWriter(self.stream,fieldnames=self.fields); self.writer.writeheader()
        self.metadata=dict(environment=rospy.get_param('~environment','real'),
                           use_sim_time=rospy.get_param('/use_sim_time',False),
                           started_wall=time.time(), imu_convention='FLU specific force, stationary level fz=+9.81',
                           rpm_convention='mechanical revolutions/minute; ESC ordering must be verified',
                           parameters=rospy.get_param('~'), samples=0)
        self.metadata['excitation_parameters']=rospy.get_param('/identification_exciter',{})
        try:
            self.metadata['holydrone_revision']=subprocess.check_output(
                ['git','-C',str(Path(__file__).resolve().parents[2]),'rev-parse','HEAD'],
                stderr=subprocess.DEVNULL,text=True,timeout=3).strip()
        except (OSError,subprocess.SubprocessError):
            self.metadata['holydrone_revision']=None
        self.save_metadata()
        topics={'pose':('/mavros/local_position/pose',PoseStamped),
                'velocity':('/mavros/local_position/velocity_local',TwistStamped),
                'state':('/mavros/state',State),'esc':('/mavros/esc_status',ESCStatus),
                'battery':('/mavros/battery',BatteryState),
                'phase':('/identification/status',String)}
        self.subscribers=[]
        for key,(default,kind) in topics.items():
            topic=rospy.get_param('~topics/'+key,default)
            self.subscribers.append(rospy.Subscriber(topic,kind,lambda m,k=key:self.cache(k,m),queue_size=100))
        self.subscribers.append(rospy.Subscriber(rospy.get_param('~topics/command','/mavros/setpoint_raw/attitude'),AttitudeTarget,self.command,queue_size=200))
        self.subscribers.append(rospy.Subscriber(rospy.get_param('~topics/imu','/mavros/imu/data'),Imu,self.imu,queue_size=200))
        rospy.on_shutdown(self.close)

    def save_metadata(self):
        (self.output/'metadata.json').write_text(json.dumps(self.metadata,indent=2,default=str)+'\n')

    def cache(self,key,message):
        with self.lock:self.latest[key]=(message,time.monotonic())

    def command(self,message):
        with self.lock:self.commands.append((message,time.monotonic()))

    def get(self,key,default,timeout=.3):
        message,receipt=self.latest.get(key,(default,0.))
        return message,time.monotonic()-receipt<timeout

    def imu(self,message):
        with self.lock:
            t=message.header.stamp.to_sec()
            if t<=self.previous or t<=0:return
            self.previous=t
            row={k:float('nan') for k in self.fields}; row['t']=t
            w,f=message.angular_velocity,message.linear_acceleration
            row.update(wx=w.x,wy=w.y,wz=w.z,fx=f.x,fy=f.y,fz=f.z)
            pose,pok=self.get('pose',PoseStamped()); velocity,vok=self.get('velocity',TwistStamped())
            state,sok=self.get('state',State(),2.)
            for key,value in zip(('px','py','pz'),(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z)):row[key]=value
            for key,value in zip(('qx','qy','qz','qw'),(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w)):row[key]=value
            for key,value in zip(('vx','vy','vz'),(velocity.twist.linear.x,velocity.twist.linear.y,velocity.twist.linear.z)):row[key]=value
            row.update(armed=int(state.armed),mode=state.mode)
            flight_ok=(pok and vok and sok and state.connected and state.armed and
                       abs(t-pose.header.stamp.to_sec())<.3 and abs(t-velocity.header.stamp.to_sec())<.3)
            row['flight_valid']=int(flight_ok)
            command=next(((m,receipt) for m,receipt in reversed(self.commands) if m.header.stamp.to_sec()<=t),None)
            command_ok=False
            if command:
                m,receipt=command; row.update(cmd_t=m.header.stamp.to_sec(),cmd_wx=m.body_rate.x,
                    cmd_wy=m.body_rate.y,cmd_wz=m.body_rate.z,cmd_thrust=m.thrust)
                command_ok=(0<=t-row['cmd_t']<.1 and time.monotonic()-receipt<.3 and m.type_mask==128)
            phase,fresh=self.get('phase',String(data='{}'),.3)
            try:status=json.loads(phase.data) if fresh else {}
            except ValueError:status={}
            row.update(segment=status.get('segment',-1),channel=status.get('channel','none'),phase=status.get('phase','passive'))
            row['valid']=int(flight_ok and command_ok and state.mode=='OFFBOARD' and
                             row['phase']=='pulse' and all(math.isfinite(row[k]) for k in ('wx','wy','wz','fz')))
            esc,eok=self.get('esc',ESCStatus())
            rpm_ok=eok and len(esc.esc_status)>=4
            if rpm_ok:
                rpm_ok=all(abs(t-e.header.stamp.to_sec())<.3 for e in esc.esc_status[:4])
                for i,item in enumerate(esc.esc_status[:4]):row['rpm%d'%i]=item.rpm
            row['rpm_valid']=int(rpm_ok)
            battery,bok=self.get('battery',BatteryState(),2.)
            if bok:row['voltage']=battery.voltage
            self.writer.writerow(row); self.rows+=1
            if self.rows%100==0:self.stream.flush()

    def close(self):
        with self.lock:
            if self.stream.closed:return
            self.stream.close(); self.metadata['samples']=self.rows; self.metadata['ended_wall']=time.time()
            self.save_metadata()


if __name__=='__main__':
    rospy.init_node('identification_recorder')
    node=Recorder()
    rospy.spin()
