"""SG90 adaptation of upstream C3 upright environment, using real firmware IMU/retries."""
from pathlib import Path
import json,os,sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
UPSTREAM=ROOT/'upstream/2dof_standing'
os.environ.setdefault('MPLCONFIGDIR',str(HERE/'results/mpl-cache'))
sys.path.insert(0,str(UPSTREAM/'sim/recovery'))
sys.path.insert(0,str(UPSTREAM/'sim/enclosure_c3/upright'))
import mujoco,numpy as np
from evaluate_filter import FilteredEnv
from strategy import Strategy
from model_geometry import geom_vertices_world
from filter_runtime_local import FilterRuntime
from build_model import aggregate

CFG=json.loads((HERE/'configuration.json').read_text())
REPORT=json.loads((HERE/'model_report.json').read_text())

class SG90Env(FilteredEnv):
    """Observation/action, filter, retry strategy and recovery reward match upstream.

    The physical model, component mass randomization, SG90 torque-speed range,
    shared supply saturation, and base support height are changed here.
    """
    def __init__(self,randomize=True,retries=True):
        self.action_increment=.10;self.use_retries=retries;self.strategy=Strategy()
        local=ROOT/'firmware/standing_xiao/RecoveryStrategy.h'
        if local.read_bytes()!=(UPSTREAM/'firmware/policy_control/RecoveryStrategy.h').read_bytes():
            raise ValueError('Local/upstream recovery supervisors differ')
        super().__init__(runtime=FilterRuntime(),randomize=randomize,model_path=HERE/'robot.xml')
        self.owns_runtime=True
        self.mount=np.eye(3);self.bias=np.zeros(3) # our verified chip-to-base axes
        self.ipos0=self.model.body_ipos.copy()
        self.groups0=REPORT['groups'];self.components=REPORT['component_masses']
        self.origins={k:np.array(v)*.001 for k,v in CFG['joint_origins_cad_mm'].items()}
        update=self.runtime.update
        def observed(accel,gyro,dt,*args,**kwargs):
            self.strategy.observe(accel,gyro,dt)
            return update(accel,gyro,dt,*args,**kwargs)
        self.runtime.update=observed

    def _randomize_components(self):
        parts=[]
        for p in self.components:
            p=dict(p);alias=p['alias'];factor=1.
            if self.randomize:
                if alias in ('battery','powerboost'):
                    limits=CFG[alias]['training_mass_range_g'];factor=self.np_random.uniform(*limits)/(p['mass_kg']*1000)
                else:
                    kind='servo_scale_range' if 'servo' in alias else 'printed_scale_range' if alias in ('base_lid','middle_arm','upper_arm','electronics_cup','lower_horn','upper_horn') else 'other_scale_range'
                    factor=self.np_random.uniform(*CFG['mass_randomization'][kind])
            p['mass_kg']*=factor;p['inertia_com_cad_kg_m2']=(np.array(p['inertia_com_cad_kg_m2'])*factor).tolist();parts.append(p)
        for group in ('base','middle','upper'):
            p=aggregate([p for p in parts if p['group']==group]);i=self.model.body(group).id
            values,rotation=np.linalg.eigh(np.array(p['inertia_com_cad_kg_m2']))
            if np.linalg.det(rotation)<0:rotation[:,0]*=-1
            quat=np.empty(4);mujoco.mju_mat2Quat(quat,rotation.ravel())
            self.model.body_mass[i]=p['mass_kg'];self.model.body_inertia[i]=values;self.model.body_iquat[i]=quat
            self.model.body_ipos[i]=np.array(p['com_cad_m'])-self.origins[group]
            if self.randomize:self.model.body_ipos[i]+=self.np_random.uniform(-.002,.002,3)

    def before_settle(self):
        self._randomize_components()
        servo=CFG['servo']
        if self.randomize:
            self.friction=float(self.np_random.uniform(*CFG['friction_range']))
            self.torque=float(self.np_random.uniform(*servo['training_stall_torque_range_Nm']))
            self.speed=float(self.np_random.uniform(*servo['training_no_load_speed_range_rad_s']))
            self.kp=float(self.np_random.uniform(*servo['kp_range_Nm_rad']))
            self.shared_budget=float(self.np_random.uniform(*servo['shared_torque_budget_Nm_range']))
            self.delay=int(self.np_random.integers(0,4))
            if self.np_random.random()<.4:self.data.qpos[-2:]=self.np_random.uniform(-1.15,1.15,2)
        else:
            self.friction=.7;self.torque=servo['nominal_effective_stall_torque_Nm'];self.speed=8.;self.kp=.6;self.shared_budget=.18;self.delay=1
        self.queue=[self.command.copy() for _ in range(self.delay)]
        self.model.geom_friction[:,0]=self.friction
        qpos=self.data.qpos.copy();qvel=self.data.qvel.copy()
        mujoco.mj_setConst(self.model,self.data)
        self.data.qpos[:]=qpos;self.data.qvel[:]=qvel;mujoco.mj_forward(self.model,self.data)
        low=min(float(geom_vertices_world(self.model,self.data,i)[:,2].min())
                for i in range(self.model.ngeom) if self.model.geom_bodyid[i]!=0 and self.model.geom_contype[i]!=0)
        if low<.001:self.data.qpos[2]+=.001-low;mujoco.mj_forward(self.model,self.data)
        self.pre_release_qpos=self.data.qpos.copy()
        self.pre_release_up=float(self.data.xmat[self.base_id].reshape(3,3)[2,2])
        super().before_settle()

    def _physics(self,target):
        # Preserve upstream motoring torque-speed envelope. A shared torque
        # budget approximates voltage sag/current sharing; electrical failure
        # and resets cannot be reproduced by this mechanical proxy.
        for _ in range(self.frame_skip):
            self.servo_target+=np.clip(target-self.servo_target,-self.speed*self.physics_dt,self.speed*self.physics_dt)
            q=self.data.qpos[-2:];v=self.data.qvel[-2:]
            requested=self.kp*(self.servo_target-q)-.012*v
            ceiling=self.torque*np.clip(1-np.sign(requested)*v/self.speed,0,1)
            torque=np.clip(requested,-ceiling,ceiling)
            torque*=min(1.,self.shared_budget/(np.abs(torque).sum()+1e-12))
            self.data.ctrl[:]=torque;mujoco.mj_step(self.model,self.data)
            if self.physics_observer is not None:
                self.physics_observer(self)
                if getattr(self,'stop_requested',False):self.data.ctrl[:]=0;break

    def reset(self,seed=None,options=None):
        self.strategy.reset(seed)
        obs,info=super().reset(seed=seed,options=options)
        self.gyro_smooth=np.zeros(3);self.base_dwell=0;self.episode_retries=0;self.recovery_rewarded=False
        self.max_target=0.;self.action_overridden=False
        return obs,info

    def step(self,action):
        before=float(self.data.xmat[self.base_id].reshape(3,3)[2,2]);old=self.previous.copy()
        self.raw_action=np.clip(np.asarray(action,dtype=np.float32),-1,1)
        applied,self.action_overridden=self.strategy.apply(self.runtime.down,self.command,self.raw_action,self.runtime.gyro) if self.use_retries else (self.raw_action,False)
        obs,_,terminated,_,info=super().step(applied)
        upright=float(info['upright']);gyro=self.data.sensor('gyro').data
        contact=any((c.geom1==self.floor_id and self.model.geom_bodyid[c.geom2]==self.base_id) or
                    (c.geom2==self.floor_id and self.model.geom_bodyid[c.geom1]==self.base_id) for c in self.data.contact)
        self.gyro_smooth=.8*self.gyro_smooth+.2*gyro
        resting=contact or self.data.qpos[2]<.0141 # supplied cup bottom CAD Z=-13.6mm
        base_good=upright>np.cos(np.deg2rad(8)) and np.linalg.norm(self.gyro_smooth)<.4 and resting
        straight=np.max(np.abs(self.data.qpos[-2:]))<.12 and np.max(np.abs(self.command))<.05
        good=base_good and straight
        self.base_dwell=self.base_dwell+1 if good else 0;success=self.base_dwell>=50
        self.recovery_rewarded|=success
        reward=-1.+.35*upright+1.*base_good+1.*good+8*(.995*upright-before)
        reward+=np.exp(-3*float(self.data.qpos[-2:]@self.data.qpos[-2:]))*max(upright,0)
        reward-=.002*float(applied@applied)+.001*float((applied-old)@(applied-old))+.0001*float(gyro@gyro)
        if success:reward+=150
        invalid=info.get('filter_fault',False) or (terminated and not info['is_success']) or not np.all(np.isfinite(self.data.qpos)) or any(w.number for w in self.data.warning)
        if invalid:reward-=100
        self.max_target=max(self.max_target,float(np.max(np.abs(self.command))));self.episode_retries=self.strategy.retries
        return obs,float(reward),bool(success or invalid),self.steps>=1500,{**info,'is_success':bool(success),'strict_success':bool(success),
            'base_good':bool(base_good),'straight':bool(straight),'episode_recovered':bool(self.recovery_rewarded),
            'control_fault':bool(invalid),'retry_count':self.episode_retries,'max_target_deg':float(np.rad2deg(self.max_target))}

    def close(self):self.strategy.close();super().close()
