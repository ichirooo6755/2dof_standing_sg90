"""Residual PPO for reward-searched moving/balancing seeds. Simulation only."""
import os
os.environ.setdefault('OMP_NUM_THREADS','1');os.environ.setdefault('OPENBLAS_NUM_THREADS','1');os.environ.setdefault('VECLIB_MAXIMUM_THREADS','1')
from pathlib import Path
import argparse,hashlib,json,time
from envs import MoveEnv,EdgeEnv,HERE
from policies import SeedPolicy,ResidualPolicy
import gymnasium as gym
import numpy as np,torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv,VecNormalize
from stable_baselines3.common.callbacks import BaseCallback

def save(p,data):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(p)

class ResidualEnv(gym.Wrapper):
    def __init__(self,task,parameters,scale=.3,start='flat'):
        env=(MoveEnv if task=='move' else EdgeEnv)(randomize=True,start='mixed' if task=='edge' else start)
        super().__init__(env);self.seed_policy=SeedPolicy(task,parameters);self.scale=scale
    def set_difficulty(self,x):self.env.set_difficulty(x)
    def reset(self,**kw):
        self.obs,info=self.env.reset(**kw);return self.obs,info
    def step(self,action):
        combined=self.seed_policy.residual_action(self.obs,np.clip(action,-1,1),self.scale)
        self.obs,r,t,tr,info=self.env.step(combined);return self.obs,r,t,tr,info

def evaluate(task,policy,seeds,start='flat',seconds=None,randomize=True):
    env=(MoveEnv if task=='move' else EdgeEnv)(randomize=randomize,start=start,difficulty=1.);rows=[]
    env.horizon=round((seconds or (12 if task=='move' else 8))/.02)
    try:
        for seed in seeds:
            obs,_=env.reset(seed=seed,options={'start':start});total=0.
            for step in range(env.horizon):
                obs,r,t,tr,info=env.step(policy.predict(obs,deterministic=True)[0]);total+=r
                if t or tr:break
            rows.append({'seed':seed,'start':start,'initial_pitch_deg':float(np.rad2deg(env.initial_pitch)),
                'target_pitch_deg':float(np.rad2deg(env.target_pitch)),'seconds':env.steps*.02,'return':float(total),
                'mass_g':float(env.model.body_mass.sum()*1000),'friction':env.friction,'stall_torque_Nm':env.torque,'speed_rad_s':env.speed,
                'final_joints_deg':np.rad2deg(env.data.qpos[-2:]).tolist(),**info})
    finally:env.close()
    return {'episodes':len(rows),'successes':sum(t['is_success'] for t in rows),'faults':sum(t['control_fault'] for t in rows),
        'mean_progress_m':float(np.mean([t['progress_m'] for t in rows])),'mean_lateral_abs_m':float(np.mean([abs(t['lateral_m']) for t in rows])),
        'mean_edge_seconds':float(np.mean([t['edge_seconds'] for t in rows])),'max_edge_seconds':max(t['edge_seconds'] for t in rows),
        'trials':rows}

class Progress(BaseCallback):
    def __init__(self,out):super().__init__();self.out=out;self.last=0;self.start=time.monotonic()
    def _on_step(self):
        if self.num_timesteps-self.last>=8192:
            self.last=self.num_timesteps;p={'status':'learning','steps':self.num_timesteps,'wall_seconds':time.monotonic()-self.start}
            save(self.out/'progress.json',p);print(json.dumps(p),flush=True)
        return True

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('task',choices=['move','edge'])
    p.add_argument('--steps',type=int,default=262144);p.add_argument('--envs',type=int,default=2);p.add_argument('--seed',type=int,default=20261112)
    p.add_argument('--seed-policy',type=Path);p.add_argument('--out',type=Path);p.add_argument('--scale',type=float,default=.3)
    p.add_argument('--start',choices=['flat','side'],default='flat')
    p.add_argument('--resume',type=Path);p.add_argument('--normalization',type=Path);a=p.parse_args();torch.set_num_threads(1);torch.set_num_interop_threads(1)
    if a.task=='edge' and a.start=='side':p.error('side start requires move')
    a.out=a.out or HERE/'results'/a.task/'ppo';a.seed_policy=a.seed_policy or HERE/'results'/a.task/'search/best_policy.json'
    seed_data=json.loads(a.seed_policy.read_text());seed=SeedPolicy(a.task,seed_data['parameters']);a.out.mkdir(parents=True,exist_ok=True)
    save(a.out/'seed_policy.json',seed_data)
    env=make_vec_env(ResidualEnv,n_envs=a.envs,seed=a.seed,env_kwargs={'task':a.task,'parameters':seed.p.tolist(),'scale':a.scale,'start':a.start},vec_env_cls=SubprocVecEnv,
        vec_env_kwargs={'start_method':'spawn'},monitor_dir=str(a.out/'episodes'),monitor_kwargs={'info_keywords':('is_success','control_fault','progress_m','lateral_m','edge_seconds')})
    env=VecNormalize.load(str(a.normalization),env) if a.normalization else VecNormalize(env,norm_obs=False,norm_reward=True,clip_reward=10,gamma=.995)
    if a.resume:actor=PPO.load(a.resume,env=env,device='cpu')
    else:
        actor=PPO('MlpPolicy',env,n_steps=512,batch_size=128,device='cpu',seed=a.seed,gamma=.995,gae_lambda=.95,learning_rate=1.5e-4,
            clip_range=.15,ent_coef=.005,policy_kwargs={'net_arch':{'pi':[64,64],'vf':[64,64]},'activation_fn':torch.nn.Tanh,'log_std_init':-1.3},verbose=0)
        with torch.no_grad():actor.policy.action_net.weight.zero_();actor.policy.action_net.bias.zero_()
    base_steps=actor.num_timesteps;start_time=time.monotonic();best=None;last=0;callback=Progress(a.out)
    report={'status':'running','task':a.task,'algorithm':'CEM seed + residual PPO','stages':[],'initial_ppo_steps':base_steps,'hardware_commands_sent':False,
        'configuration':{k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},
        'hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),HERE/'envs.py',HERE/'policies.py',HERE.parent/'sg90_1000mah/robot.xml',a.seed_policy]}}
    try:
        for step in [0]+list(range(32768,a.steps+1,32768)):
            difficulty=min(1.,max(0.,(step-65536)/131072)) if a.task=='edge' else 1.
            env.env_method('set_difficulty',difficulty)
            if step:actor.learn(total_timesteps=step-last,reset_num_timesteps=False,callback=callback);last=step
            actor.save(a.out/f'checkpoint_{actor.num_timesteps:07d}');env.save(str(a.out/f'normalization_{actor.num_timesteps:07d}.pkl'))
            policy=ResidualPolicy(seed,actor,a.scale)
            primary=evaluate(a.task,policy,range(94000,94006),start=a.start)
            near=evaluate(a.task,policy,range(94100,94106),start='near') if a.task=='edge' else None
            if a.task=='edge':key=(primary['successes'],primary['mean_edge_seconds'],near['successes'],near['mean_edge_seconds'])
            else:key=(primary['successes'],primary['mean_progress_m']-primary['mean_lateral_abs_m'])
            if best is None or key>best:
                best=key;actor.save(a.out/'best_residual');env.save(str(a.out/'normalization_best.pkl'));report['selected_ppo_steps']=actor.num_timesteps
                save(a.out/'policy_bundle.json',{'task':a.task,'start':a.start,'seed_parameters':seed.p.tolist(),'residual_model':'best_residual.zip','residual_scale':a.scale,
                    'selected_ppo_steps':actor.num_timesteps,'observations':16 if a.task=='move' else 10,'hardware_commands_sent':False})
            report['stages'].append({'steps':actor.num_timesteps,'difficulty':difficulty,a.start:primary,'near':near})
            report.update(actual_ppo_steps=actor.num_timesteps,wall_seconds=time.monotonic()-start_time)
            save(a.out/'training_report.json',report)
            print(json.dumps({'stage_steps':actor.num_timesteps,'difficulty':difficulty,'selected':report['selected_ppo_steps'],
                a.start:{k:v for k,v in primary.items() if k!='trials'},'near':{k:v for k,v in near.items() if k!='trials'} if near else None,'wall_seconds':report['wall_seconds']}),flush=True)
        best_actor=PPO.load(a.out/'best_residual.zip',device='cpu');policy=ResidualPolicy(seed,best_actor,a.scale)
        primary=evaluate(a.task,policy,range(95000,95024),start=a.start)
        near=evaluate(a.task,policy,range(95100,95124),start='near') if a.task=='edge' else None
        baseline=evaluate(a.task,seed,range(95000,95024),start=a.start)
        report.update(status='complete',held_out_near=near,wall_seconds=time.monotonic()-start_time)
        report[f'held_out_{a.start}']=primary
        report[f'seed_baseline_{a.start}']=baseline
        save(a.out/'training_report.json',report);save(a.out/f'held_out_{a.start}.json',primary)
        if near:save(a.out/'held_out_near.json',near)
        save(a.out/'progress.json',{'status':'complete','steps':actor.num_timesteps,'wall_seconds':report['wall_seconds']})
        print(json.dumps({'status':'complete','task':a.task,'steps':actor.num_timesteps,a.start:{k:v for k,v in primary.items() if k!='trials'},'near':{k:v for k,v in near.items() if k!='trials'} if near else None}),flush=True)
    finally:env.close()

if __name__=='__main__':main()
