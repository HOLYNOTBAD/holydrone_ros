#!/usr/bin/env python3
"""Bounded pulse experiments. No automatic arming or mode entry in real flight."""
import copy
import collections
import json
import math
import random
import threading
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class IdentificationFSM:
    STATES = {'idle':'IDLE', 'hold':'HOLD', 'takeoff':'TAKEOFF',
              'recover':'RECOVER', 'pulse':'EXCITE', 'complete':'COMPLETE',
              'landing':'LANDING', 'done':'FINISHED', 'fault':'ABORTED'}

    TRANSITIONS = {
        'idle': {'hold','fault'}, 'hold': {'takeoff','recover','landing','fault'},
        'takeoff': {'hold','landing','fault'}, 'recover': {'pulse','complete','landing','fault'},
        'pulse': {'recover','landing','fault'}, 'complete': {'hold','landing','done','fault'},
        'landing': {'done','fault'}, 'done': {'hold','fault'}, 'fault': {'hold','landing'},
    }

    def __init__(self):
        self.environment=rospy.get_param('~environment','sim')
        sim=rospy.get_param('/use_sim_time',False)
        if self.environment not in ('sim','real') or sim!=(self.environment=='sim'):
            raise ValueError('environment and /use_sim_time disagree')
        self.auto=bool(rospy.get_param('~auto_takeoff',False))
        if self.environment=='real' and (self.auto or not rospy.get_param('~enable_real_excitation',False)):
            raise ValueError('Real excitation requires enable_real_excitation; automatic takeoff is forbidden')
        self.p={}
        for key,default in [('frequency',50.),('hover_thrust',.24),('thrust_amplitude',.015),
                ('pulse_hold',.35),('baseline_seconds',.3),('recovery_seconds',2.),('settle_seconds',1.),
                ('position_tolerance',.15),('speed_tolerance',.15),('rate_tolerance',.15),
                ('tilt_tolerance',.12),('max_radius',1.),('max_tilt',.35),('min_z',.5),('max_z',3.5),
                ('feedback_timeout',.3),('state_timeout',2.),('max_session_seconds',180.),('takeoff_height',2.)]:
            value=float(rospy.get_param('~'+key,default))
            if not math.isfinite(value) or value<=0:raise ValueError('Invalid '+key)
            self.p[key]=value
        self.amplitudes=np.asarray(rospy.get_param('~rate_amplitudes',[.2,.2,.25]),float)
        if self.amplitudes.shape!=(3,) or not np.isfinite(self.amplitudes).all() or np.any(self.amplitudes<=0) or np.any(self.amplitudes>1.):
            raise ValueError('rate_amplitudes must be three positive values <=1 rad/s')
        if not 0<self.p['hover_thrust']-self.p['thrust_amplitude']<self.p['hover_thrust']+self.p['thrust_amplitude']<1:
            raise ValueError('Thrust pulse exceeds [0,1]')
        if not self.p['min_z']<self.p['takeoff_height']<self.p['max_z'] or self.p['max_tilt']>1.:
            raise ValueError('Invalid envelope')
        repetitions=int(rospy.get_param('~repetitions',3))
        if not 1<=repetitions<=20:raise ValueError('repetitions must be 1..20')
        rng=random.Random(int(rospy.get_param('~seed',100)))
        self.plan=[(axis,rng.choice([-1,1]),rng.uniform(.7,1.)) for _ in range(repetitions) for axis in range(4)]
        self.hover_source=rospy.get_param('~hover_source','feedback')
        if self.hover_source not in ('feedback','fixed'):raise ValueError('hover_source must be feedback or fixed')
        self.hover_samples=collections.deque(maxlen=100); self.pulse_hover=self.p['hover_thrust']
        self.data={}; self.lock=threading.RLock(); self.phase='idle'; self.index=0
        self.reason='initialized'; self.segment_base=-(len(self.plan)+1); self.sequence_complete=False
        self.reference=None; self.begin=0.; self.phase_time=0.; self.settled=None; self.last_service=0.
        self.last_clock=0.; self.next_tick=0.; self.auto_used=False; self.seen_offboard=False; self.seen_armed=False; self.last_authority_check=0.
        self.position=rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=1)
        self.command=rospy.Publisher('/mavros/setpoint_raw/attitude',AttitudeTarget,queue_size=1)
        self.state_pub=rospy.Publisher('/identification/state',String,queue_size=1,latch=True)
        self.events=rospy.Publisher('/identification/events',String,queue_size=100,latch=True)
        self.status=rospy.Publisher('/identification/status',String,queue_size=1,latch=True)
        self.subscribers=[]
        for key,topic,kind in [('state','/mavros/state',State),('pose','/mavros/local_position/pose',PoseStamped),
                              ('velocity','/mavros/local_position/velocity_local',TwistStamped),('imu','/mavros/imu/data',Imu)]:
            self.subscribers.append(rospy.Subscriber(topic,kind,lambda m,k=key:self.cache(k,m),queue_size=1))
        self.subscribers.append(rospy.Subscriber('/mavros/setpoint_raw/target_attitude',AttitudeTarget,lambda m:self.cache('target',m),queue_size=1))
        self.mode=rospy.ServiceProxy('/mavros/set_mode',SetMode)
        self.arm=rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        for name,callback in [('prepare',self.prepare),('start',self.start),('stop',self.stop),('land',self.land)]:
            rospy.Service('/identification/'+name,Trigger,callback)

    def transition(self, phase, reason):
        if phase not in self.STATES:raise ValueError('Unknown identification state: '+phase)
        if phase==self.phase:return
        if phase not in self.TRANSITIONS[self.phase]:
            raise ValueError('Invalid transition: '+self.phase+' -> '+phase)
        old=self.STATES[self.phase]
        self.phase=phase; self.reason=reason; self.phase_time=rospy.Time.now().to_sec()
        self.settled=None
        event=dict(time=self.phase_time, old=old, state=self.STATES[phase], reason=reason,
                   segment=self.segment_base+self.index)
        rospy.loginfo('Identification FSM: %s -> %s (%s)',old,self.STATES[phase],reason)
        self.events.publish(String(data=json.dumps(event)))

    def publish_status(self):
        channel=('roll','pitch','yaw','thrust')[self.plan[min(self.index,len(self.plan)-1)][0]]
        self.state_pub.publish(String(data=self.STATES[self.phase]))
        self.status.publish(String(data=json.dumps(dict(
            state=self.STATES[self.phase], phase=self.phase, reason=self.reason,
            segment=self.segment_base+self.index, channel=channel,
            sequence_complete=self.sequence_complete))))

    def cache(self,key,message):
        with self.lock:self.data[key]=(message,time.monotonic())

    def ready(self):
        wall=time.monotonic(); now=rospy.Time.now().to_sec()
        for key in ('state','pose','velocity','imu'):
            if key not in self.data:return False
            msg,receipt=self.data[key]
            limit=self.p['state_timeout'] if key=='state' else self.p['feedback_timeout']
            if wall-receipt>limit:return False
            if key!='state' and not 0<=now-msg.header.stamp.to_sec()<limit:return False
        q=self.data['pose'][0].pose.orientation
        p=self.xyz(self.data['pose'][0].pose.position)
        return (self.data['state'][0].connected and .5<q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<1.5 and
                np.isfinite(np.r_[p,self.xyz(self.data['velocity'][0].twist.linear),self.xyz(self.data['imu'][0].angular_velocity)]).all())

    @staticmethod
    def xyz(v):return np.array([v.x,v.y,v.z])

    def tilt(self):
        q=self.data['pose'][0].pose.orientation
        norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        return math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y)/norm)))

    def stable(self):
        return (np.linalg.norm(self.xyz(self.data['pose'][0].pose.position)-self.xyz(self.reference.pose.position))<self.p['position_tolerance'] and
                np.linalg.norm(self.xyz(self.data['velocity'][0].twist.linear))<self.p['speed_tolerance'] and
                np.linalg.norm(self.xyz(self.data['imu'][0].angular_velocity))<self.p['rate_tolerance'] and self.tilt()<self.p['tilt_tolerance'])

    def exclusive(self):
        code,_,system=rospy.get_master().getSystemState()
        if code!=1:return False
        controlled={'/mavros/setpoint_raw/attitude','/mavros/setpoint_position/local',
                    '/mavros/setpoint_velocity/cmd_vel','/mavros/setpoint_raw/local'}
        return not any(topic in controlled and any(node!=rospy.get_name() for node in nodes)
                       for topic,nodes in system[0])

    def prepare(self,_):
        with self.lock:
            if not self.exclusive():return TriggerResponse(False,'Another setpoint publisher is active')
            if not self.ready() or self.phase not in ('idle','done','fault','complete'):
                return TriggerResponse(False,'Need fresh feedback and an idle experiment')
            if self.environment=='real' and not self.data['state'][0].armed:
                return TriggerResponse(False,'Real vehicle must already be airborne and armed')
            if self.environment=='real' and not self.p['min_z']<self.data['pose'][0].pose.position.z<self.p['max_z']:
                return TriggerResponse(False,'Outside altitude envelope')
            self.reference=copy.deepcopy(self.data['pose'][0])
            self.index=0;self.segment_base+=len(self.plan)+1;self.sequence_complete=False
            self.seen_offboard=False;self.seen_armed=False;self.hover_samples.clear()
            self.begin=rospy.Time.now().to_sec()
            self.transition('hold','operator prepared experiment')
            return TriggerResponse(True,'Streaming position hold; operator may select OFFBOARD')

    def start(self,_):
        with self.lock:
            if not self.ready() or self.phase!='hold' or not self.stable():
                return TriggerResponse(False,'Prepare and settle at the reference first')
            state=self.data['state'][0]
            if not state.armed or state.mode!='OFFBOARD':return TriggerResponse(False,'Requires armed OFFBOARD')
            if not self.p['min_z']<self.data['pose'][0].pose.position.z<self.p['max_z']:
                return TriggerResponse(False,'Outside altitude envelope')
            self.index=0;self.begin=rospy.Time.now().to_sec()
            self.transition('recover','experiment started')
            return TriggerResponse(True,'Bounded pulse sequence started')

    def stop(self,_):
        with self.lock:
            if self.phase in ('idle','done'):return TriggerResponse(True,'No active experiment')
            self.abort('operator stop')
            return TriggerResponse(True,'Excitation cancelled; holding if feedback is valid')

    def land(self,_):
        with self.lock:
            state=self.data.get('state',(State(),0))[0]
            if self.reference is None or not self.ready() or not state.armed or state.mode!='OFFBOARD':
                return TriggerResponse(False,'Landing requires an active, armed OFFBOARD experiment')
            self.transition('landing','operator requested landing')
            return TriggerResponse(True,'Landing requested; waiting for disarm feedback')

    def abort(self,reason):
        if self.phase=='fault':return
        self.transition('fault',reason)
        if self.ready():self.reference=copy.deepcopy(self.data['pose'][0])

    def set_mode(self,name):
        if time.monotonic()-self.last_service<1.:return
        self.last_service=time.monotonic()
        try:self.mode(base_mode=0,custom_mode=name)
        except rospy.ServiceException as error:rospy.logwarn('%s',error)

    def tick(self,now):
        ready=self.ready()
        state=self.data.get('state',(State(),0))[0]
        if self.auto and not self.auto_used and ready and self.phase=='idle':
            self.auto_used=True
            response=self.prepare(None)
            if not response.success:rospy.logerr('%s',response.message);return
            self.reference.pose.position.z=self.p['takeoff_height']; self.transition('takeoff','automatic simulation takeoff')
        if self.reference is not None and self.phase not in ('idle','done') and time.monotonic()-self.last_authority_check>1.:
            self.last_authority_check=time.monotonic()
            if not self.exclusive():
                self.abort('another controller appeared');self.reference=None
                rospy.logerr('Another controller appeared; withdrawing all identification setpoints')
        if self.phase=='complete':
            _,receipt=self.data.get('state',(State(),0.))
            if state.connected and time.monotonic()-receipt<self.p['state_timeout'] and (not state.armed or state.mode!='OFFBOARD'):
                self.transition('done','operator took over completed experiment')
        if self.phase not in ('idle','done','fault','landing'):
            if not ready:self.abort('feedback timeout or invalid data')
            elif now-self.begin>self.p['max_session_seconds']:self.abort('session timeout')
            elif self.phase not in ('hold','takeoff') and (not state.armed or state.mode!='OFFBOARD'):
                self.abort('operator mode exit/disarm')
        if ready and self.phase in ('pulse','recover'):
            p=self.xyz(self.data['pose'][0].pose.position)
            origin=self.xyz(self.reference.pose.position)
            if not self.p['min_z']<p[2]<self.p['max_z'] or np.linalg.norm(p-origin)>self.p['max_radius'] or self.tilt()>self.p['max_tilt']:
                self.abort('flight envelope')
        if self.phase=='takeoff' and ready:
            if ((self.seen_offboard and state.mode!='OFFBOARD') or
                    (self.seen_armed and not state.armed)):
                self.abort('mode exit/disarm during simulated takeoff')
            self.seen_offboard=self.seen_offboard or state.mode=='OFFBOARD'
            self.seen_armed=self.seen_armed or state.armed
            p=self.xyz(self.data['pose'][0].pose.position)
            origin=self.xyz(self.reference.pose.position)
            if p[2]>self.p['max_z'] or np.linalg.norm((p-origin)[:2])>self.p['max_radius'] or self.tilt()>self.p['max_tilt']:
                self.abort('takeoff envelope')
        if self.phase=='takeoff' and ready:
            if now-self.begin>2 and state.mode!='OFFBOARD':self.set_mode('OFFBOARD')
            elif state.mode=='OFFBOARD' and not state.armed and time.monotonic()-self.last_service>1:
                self.last_service=time.monotonic()
                try:self.arm(value=True)
                except rospy.ServiceException:pass
            elif state.armed and self.stable():
                if self.settled is None:self.settled=now
                if now-self.settled>self.p['settle_seconds']:
                    self.transition('hold','takeoff stabilized'); self.start(None)
            else:self.settled=None
        if ready and self.phase in ('hold','takeoff','recover') and state.armed and state.mode=='OFFBOARD' and self.stable():
            target,receipt=self.data.get('target',(AttitudeTarget(),0.))
            if time.monotonic()-receipt<.3 and 0<target.thrust<1:
                self.hover_samples.append((now,float(target.thrust)))
        if self.phase=='recover' and ready:
            if self.stable():
                if self.settled is None:self.settled=now
                if now-self.settled>=self.p['settle_seconds'] and now-self.phase_time>=self.p['recovery_seconds']:
                    if self.index>=len(self.plan):
                        self.sequence_complete=True
                        self.transition('complete','all pulses complete and hover recovered')
                    else:
                        recent=[value for stamp,value in self.hover_samples if now-stamp<2.]
                        if self.hover_source!='feedback' or len(recent)>=20:
                            self.pulse_hover=float(np.median(recent)) if self.hover_source=='feedback' else self.p['hover_thrust']
                            if not self.p['thrust_amplitude']<self.pulse_hover<1-self.p['thrust_amplitude']:
                                self.abort('measured hover thrust outside pulse range')
                            else:
                                rospy.loginfo('Pulse %d: measured hover thrust %.4f',self.index,self.pulse_hover)
                                self.transition('pulse','stable baseline available')
                # Continue streaming hold while waiting for fresh hover-thrust feedback.
            else:self.settled=None
        if self.phase=='pulse':
            baseline=self.p['baseline_seconds']; hold=self.p['pulse_hold']; elapsed=now-self.phase_time
            if elapsed>=2*baseline+2*hold:
                self.index+=1
                self.transition('recover','pulse complete; recover before next action')
            else:
                axis,sign,scale=self.plan[self.index]
                value=0. if elapsed<baseline or elapsed>=baseline+2*hold else (sign if elapsed<baseline+hold else -sign)
                action=[0.,0.,0.,self.pulse_hover]
                action[axis]+=value*scale*(self.amplitudes[axis] if axis<3 else self.p['thrust_amplitude'])
                msg=AttitudeTarget();msg.header.stamp=rospy.Time.from_sec(now);msg.type_mask=128
                msg.body_rate.x,msg.body_rate.y,msg.body_rate.z=action[:3];msg.thrust=action[3]
                self.command.publish(msg)
        if self.phase in ('hold','takeoff','recover','complete','landing','fault') and ready and self.reference is not None:
            # After mode exit, stop publishing; never reclaim OFFBOARD outside simulated takeoff.
            if state.mode=='OFFBOARD' or self.phase in ('hold','takeoff'):
                msg=copy.deepcopy(self.reference);msg.header.stamp=rospy.Time.from_sec(now);self.position.publish(msg)
        if self.phase=='fault' and not ready and state.armed and state.mode=='OFFBOARD':self.set_mode('AUTO.LAND')
        if self.phase=='complete' and self.auto:
            self.transition('landing','simulation experiment complete')
        if self.phase=='landing':
            state_message,receipt=self.data.get('state',(State(),0.))
            connected=state_message.connected and time.monotonic()-receipt<self.p['state_timeout']
            if connected and not state_message.armed:
                self.transition('done','PX4 confirmed disarm')
            elif connected and state_message.mode=='OFFBOARD':self.set_mode('AUTO.LAND')
            elif connected and state_message.mode!='AUTO.LAND':self.abort('operator mode exit during landing')
        self.publish_status()

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                now=rospy.Time.now().to_sec()
                if now<self.last_clock:
                    self.abort('clock moved backwards');self.next_tick=now
                self.last_clock=now
                # A wall-time watchdog remains alive when Gazebo /clock stops.
                if self.phase not in ('idle','done','fault','landing') and not self.ready():self.abort('watchdog')
                if now>0 and now>=self.next_tick:
                    self.tick(now);self.next_tick=now+1./self.p['frequency']
            time.sleep(.002)


def main():
    rospy.init_node('identification_fsm')
    try:
        IdentificationFSM().run()
    except (ValueError, KeyError) as error:
        rospy.logfatal('%s', error)
        raise SystemExit(1)
