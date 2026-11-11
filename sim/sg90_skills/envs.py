"""New locomotion/edge tasks sharing the measured-shape SG90 dynamics.

No serial access. Pose, joint encoders and COM truth are scoring/reset data only.
Actors see the existing noisy firmware-filter observations plus an internal
clock and gyro-integrated direction for locomotion.
"""
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
BASE=HERE.parent/'sg90_1000mah'
sys.path.insert(0,str(BASE))
from env import SG90Env,CFG
from evaluate_filter import FilteredEnv
from model_geometry import place_neutral_robot
import mujoco,numpy as np
from gymnasium import spaces

class SkillEnv(SG90Env):
    def __init__(self,task,randomize=True,start='near',difficulty=0.):
        self.task=task;self.start_kind=start;self.difficulty=difficulty
        self.heading=np.array([-1.,0.,0.]);self.heading_enabled=False
        super().__init__(randomize=randomize,retries=False)
        update=self.runtime.update
        def observed(accel,gyro,dt,*args,**kwargs):
            valid=update(accel,gyro,dt,*args,**kwargs)
            if self.heading_enabled and 0<dt<.05:
                self.heading-=np.cross(self.runtime.gyro,self.heading)*dt
                self.heading/=max(np.linalg.norm(self.heading),1e-9)
            return valid
        self.runtime.update=observed
        self.observation_space=spaces.Box(-1.,1.,(16 if task=='move' else 10,),dtype=np.float32)
        self.horizon=600 if task=='move' else 400
        self.support_weight=9.81*float(self.model.body_mass.sum())

    def set_difficulty(self,value):self.difficulty=float(value)
    def _obs(self):
        obs=FilteredEnv._obs(self)
        if self.task=='edge':return obs
        phase=2*np.pi*.65*self.steps*.02
        return np.r_[obs,self.heading,np.sin(phase),np.cos(phase),1-2*min(1,self.steps/self.horizon)].astype(np.float32)

    def balance_angle(self):
        # Maximum COM potential height over the convex support envelope is the
        # unstable edge equilibrium. Derive anew after each mass randomization.
        self.data.qpos[:]=self.model.qpos0;self.data.qpos[-2:]=0
        mujoco.mj_forward(self.model,self.data)
        com=self.data.subtree_com[self.base_id]-self.data.xpos[self.base_id]
        theta=np.linspace(-1.15,-.4,1501)
        normals=np.c_[-np.sin(theta),np.zeros_like(theta),np.cos(theta)]
        support=self.neutral_support
        support=support[(support[:,0]<-.025)&(support[:,2]<-.005)]
        heights=normals@com-np.min(normals@support.T,axis=1)
        return float(theta[np.argmax(heights)])

    def reset(self,seed=None,options=None):
        self.heading_enabled=False;self.heading=np.array([-1.,0.,0.])
        super().reset(seed=seed,options={'tilt':0.,'azimuth':0.})
        options=options or {};kind=options.get('start',self.start_kind)
        self.target_pitch=self.balance_angle()
        if self.task=='edge':
            if kind=='mixed':kind='flat' if self.np_random.random()<.75*self.difficulty else 'near'
            theta=0. if kind=='flat' else self.target_pitch+self.np_random.uniform(-.015-.05*self.difficulty,.015+.05*self.difficulty)
            if kind=='approach':theta=self.target_pitch*.6
        else:
            kind='side' if kind=='side' else 'flat'
            theta=-np.pi/2 if kind=='side' else 0.
        self.actual_start=kind
        place_neutral_robot(self.model,self.data,abs(theta),-np.pi/2,clearance=.0001,support_points=self.neutral_support)
        self.data.qvel[:]=0;self.data.ctrl[:]=0
        self.command[:]=0;self.servo_target[:]=0;self.previous[:]=0;self.steps=0
        self.queue=[self.command.copy() for _ in range(self.delay)]
        mujoco.mj_forward(self.model,self.data)
        self.runtime.reset();self.control_active=False;self.stop_requested=False
        self.next_sample=self.data.time;self.sample_time=None
        # Assisted near-edge starts are explicitly reported. No force/support
        # constraints are applied once the policy begins.
        FilteredEnv.before_settle(self)
        self.control_active=True;self.heading_enabled=self.task=='move'
        self.origin=self.data.qpos[:2].copy();self.goal=np.array([-1.,0.]);self.side=np.array([0.,1.])
        self.best_edge_seconds=0.;self.edge_ticks=0;self.edge_arrived=False
        self.halfway_progress=None;self.gyro_smooth=np.zeros(3)
        self.support_weight=9.81*float(self.model.body_mass.sum())
        self.prior_error=abs(theta-self.target_pitch)
        self.initial_pitch=theta
        return self._obs(),{}

    def support(self):
        points=[];loads=[];other=0.
        force=np.zeros(6)
        R=self.data.xmat[self.base_id].reshape(3,3)
        for i,c in enumerate(self.data.contact):
            if c.geom1==self.floor_id:g=c.geom2
            elif c.geom2==self.floor_id:g=c.geom1
            else:continue
            mujoco.mj_contactForce(self.model,self.data,i,force)
            normal=max(0.,float(force[0]))
            if normal<.001:continue
            if self.model.geom_bodyid[g]==self.base_id:
                points.append(R.T@(c.pos-self.data.xpos[self.base_id]));loads.append(normal)
            else:other+=normal
        if not points:return False,0.,other,0.,0.
        points=np.asarray(points);loads=np.asarray(loads)
        on_edge=(points[:,0]<-.0305)&(points[:,2]<-.0100)
        edge_force=float(loads[on_edge].sum());wrong=float(loads[~on_edge].sum())+other
        span=float(np.ptp(points[on_edge,1])) if np.sum(on_edge)>1 else 0.
        valid=edge_force>.55*self.support_weight and wrong<.035*self.support_weight and span>.02
        contact=np.average(points,axis=0,weights=loads)
        contact_world=R@contact+self.data.xpos[self.base_id]
        edge_direction=R[:,1].copy();edge_direction[2]=0;edge_direction/=max(np.linalg.norm(edge_direction),1e-9)
        perpendicular=np.array([-edge_direction[1],edge_direction[0],0.])
        com_error=float((self.data.subtree_com[self.base_id]-contact_world)@perpendicular)
        return bool(valid),edge_force,wrong,com_error,span

    def step(self,action):
        action=np.clip(np.asarray(action,dtype=float),-1,1);old=self.previous.copy()
        before=self.data.qpos[:2].copy()
        self.command=np.clip(self.command+.10*action,-self.limit,self.limit)
        self.queue.append(self.command.copy());delayed=self.queue.pop(0)
        self._physics(delayed);self.previous=action.copy();self.steps+=1
        R=self.data.xmat[self.base_id].reshape(3,3);up=float(R[2,2])
        gyro=self.data.sensor('gyro').data;self.gyro_smooth=.75*self.gyro_smooth+.25*gyro
        fault=bool(self.stop_requested or not np.all(np.isfinite(self.data.qpos)) or any(w.number for w in self.data.warning) or np.linalg.norm(self.data.qpos[:2]-self.origin)>1.5)
        pitch=float(np.arctan2(-R[2,0],R[2,2]))
        progress=float((self.data.qpos[:2]-self.origin)@self.goal)
        lateral=float((self.data.qpos[:2]-self.origin)@self.side)
        info={'control_fault':fault,'progress_m':progress,'lateral_m':lateral,'base_up':up,'base_pitch_rad':pitch,
              'edge_seconds':self.best_edge_seconds,'is_success':False,'start':self.actual_start}
        terminated=fault
        if self.task=='move':
            delta=self.data.qpos[:2]-before
            reward=500.*float(delta@self.goal)-35.*abs(float(delta@self.side))
            reward+=.03-.002*float(action@action)-.001*float((action-old)@(action-old))
            if self.actual_start=='flat':reward+=.04*max(up,0)
            reward-=.0003*float(gyro@gyro)+.05*abs(lateral)
            if self.steps==self.horizon//2:self.halfway_progress=progress
            late_progress=progress-(self.halfway_progress if self.halfway_progress is not None else progress)
            info['second_half_progress_m']=late_progress
            info['is_success']=bool(progress>=.06 and late_progress>=.02 and abs(lateral)<.04 and not fault)
            if self.steps>=self.horizon and self.actual_start=='flat':
                reward+=3.*max(up,0)
        else:
            edge,load,bad,com_error,span=self.support()
            angle_error=abs(pitch-self.target_pitch)
            self.edge_arrived|=bool(edge and angle_error<.15)
            slow=float(np.linalg.norm(self.gyro_smooth))<.45
            good=edge and slow and abs(float(R[2,1]))<.10 and -.125>pitch>-1.3
            self.edge_ticks=self.edge_ticks+1 if good else 0
            self.best_edge_seconds=max(self.best_edge_seconds,self.edge_ticks*.02)
            reward=1.7*np.exp(-(angle_error/.22)**2)+.5*np.exp(-(com_error/.010)**2)
            reward+=2.5*edge+.5*good+5.*(self.prior_error-angle_error)
            reward-=.004*float(action@action)+.002*float((action-old)@(action-old))+.002*float(gyro@gyro)
            reward-=2.*min(2.,bad/max(self.support_weight,1e-8))
            self.prior_error=angle_error
            if self.actual_start=='near' and (pitch>-.3 or pitch<-1.35):terminated=True;reward-=5
            if self.edge_arrived and (pitch>-.25 or pitch<-1.4):terminated=True;reward-=5
            info.update(edge_contact=edge,edge_load_N=load,other_load_N=bad,com_error_m=com_error,edge_span_m=span,
                edge_seconds=self.best_edge_seconds,is_success=bool(self.best_edge_seconds>=3 and not fault))
        if fault:reward-=30
        return self._obs(),float(reward),bool(terminated),self.steps>=self.horizon,info

class MoveEnv(SkillEnv):
    def __init__(self,**kw):super().__init__('move',**kw)
class EdgeEnv(SkillEnv):
    def __init__(self,**kw):super().__init__('edge',**kw)
