"""Reward-based cross-entropy policy search. No hardware commands or uploads."""
import os
os.environ.setdefault('OMP_NUM_THREADS','1');os.environ.setdefault('OPENBLAS_NUM_THREADS','1');os.environ.setdefault('VECLIB_MAXIMUM_THREADS','1')
from pathlib import Path
import argparse,hashlib,json,time,multiprocessing
from concurrent.futures import ProcessPoolExecutor
from envs import MoveEnv,EdgeEnv,HERE
from policies import SeedPolicy
import numpy as np
ENV=None
def save(p,data):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(p)

def initialize(task):
    global ENV
    ENV=(MoveEnv if task=='move' else EdgeEnv)(randomize=False)

def rollout(item):
    parameters,seed,randomize,start=item
    ENV.randomize=randomize;ENV.horizon=600 if ENV.task=='move' else 250
    obs,_=ENV.reset(seed=seed,options={'start':start});policy=SeedPolicy(ENV.task,parameters);total=0
    for step in range(ENV.horizon):
        obs,reward,terminated,truncated,info=ENV.step(policy.predict(obs)[0]);total+=reward
        if terminated or truncated:break
    if ENV.task=='move':
        score=total/100+20*info['progress_m']-8*abs(info['lateral_m'])+10*info['second_half_progress_m']-10*info['control_fault']
    else:score=total/ENV.horizon+6*info['edge_seconds']+.5*(step+1)/ENV.horizon-5*info['control_fault']
    return {'score':float(score),'steps':step+1,'return':float(total),'seed':seed,'randomized':randomize,
            'target_pitch':ENV.target_pitch,'mass_g':float(ENV.model.body_mass.sum()*1000),**info}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('task',choices=['move','edge']);p.add_argument('--iterations',type=int,default=20)
    p.add_argument('--population',type=int,default=16);p.add_argument('--workers',type=int,default=2);p.add_argument('--seed',type=int,default=1111)
    p.add_argument('--out',type=Path);p.add_argument('--family',choices=['smooth','kick','ramp'],default='smooth');p.add_argument('--negative-edge',action='store_true')
    p.add_argument('--side-start',action='store_true',help='Start locomotion lying on the USB side')
    a=p.parse_args();a.out=a.out or HERE/'results'/a.task/'search';a.out.mkdir(parents=True,exist_ok=True)
    if a.side_start and a.task!='move':p.error('--side-start requires move')
    initial_pose='near' if a.task=='edge' else 'side' if a.side_start else 'flat'
    rng=np.random.default_rng(a.seed)
    if a.task=='move' and a.family=='ramp':
        mean=np.r_[[0,-1.5,0,1.5,0,1.5,0,-1.5],[-1.3,-1.7,.3,.3],1.]
        std=np.r_[np.full(8,.65),np.full(4,.5),0.]
        low=np.r_[np.full(8,-3.),np.full(4,-2.5),1.];high=np.r_[np.full(8,3.),np.full(4,.5),1.]
    elif a.task=='move' and a.family=='kick':
        mean=np.r_[[-1,-1,1,1,1,-1,-1,1],[-1,-.2,-1,-.2]];std=np.r_[np.ones(8),np.full(4,.55)]
        low=np.r_[np.full(8,-3.),np.full(4,-2.5)];high=np.r_[np.full(8,3.),np.full(4,.5)]
    elif a.task=='move':mean=np.zeros(10);std=np.full(10,.8);low=np.full(10,-2.5);high=-low
    elif a.negative_edge:
        mean=np.array([-3.,-.35,-1.5,-.826]);std=np.array([4.,.8,1.,.015]);low=np.array([-25,-5,-10,-.90]);high=np.array([10,3,0,-.75])
    else:mean=np.array([12.,1.,-1.,-.83]);std=np.array([16.,3.,6.,.045]);low=np.array([-70,-15,-35,-1.05]);high=np.array([70,15,35,-.6])
    report={'algorithm':'Cross-entropy method, episodic reward-driven policy search','task':a.task,'history':[],'actual_environment_steps':0,'hardware_commands_sent':False,
        'arguments':{k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},
        'model_sha256':hashlib.sha256((HERE.parent/'sg90_1000mah/robot.xml').read_bytes()).hexdigest()}
    best=-1e9;best_p=mean.copy();start=time.monotonic()
    with ProcessPoolExecutor(a.workers,mp_context=multiprocessing.get_context('spawn'),initializer=initialize,initargs=(a.task,)) as pool:
        for iteration in range(a.iterations):
            candidates=np.clip(rng.normal(mean,std,(a.population,len(mean))),low,high);candidates[0]=best_p
            if a.task=='move' and a.family=='ramp':
                candidates[1::4,[0,2,4,6]]=0 # retain explicit single-axis stick/slip candidates
            # Common random numbers within each generation. Randomized dynamics
            # enter after nominal exploration, and are held fixed for ranking.
            randomize=iteration>=max(5,a.iterations//2)
            trials=list(pool.map(rollout,[(p.tolist(),91000+iteration,randomize,initial_pose) for p in candidates]))
            order=np.argsort([t['score'] for t in trials])[::-1];elite=candidates[order[:max(3,a.population//4)]]
            mean=.25*mean+.75*elite.mean(axis=0)
            floor=np.full(len(mean),.07) if a.task=='move' else np.array([.7,.12,.3,.003])
            if a.task=='move' and a.family=='ramp':floor[-1]=0.
            std=np.maximum(floor,.25*std+.75*elite.std(axis=0))
            chosen=candidates[order[0]]
            dev=list(pool.map(rollout,[(chosen.tolist(),92000+i,True,initial_pose) for i in range(3)]))
            score=float(np.mean([v['score'] for v in dev]));report['actual_environment_steps']+=sum(t['steps'] for t in trials+dev)
            if score>best:
                best=score;best_p=chosen.copy();save(a.out/'best_policy.json',{'task':a.task,'start':initial_pose,'parameters':best_p.tolist(),'algorithm':'CEM','development_score':best,
                    'input':'IMU firmware filter, commands and previous actions; move also internal clock and gyro-integrated heading','hardware_commands_sent':False})
            row={'iteration':iteration,'randomized_training':randomize,'generation_best':trials[order[0]],'development':dev,'development_score':score,'best_development_score':best,
                 'generation_progress_range_m':[min(t['progress_m'] for t in trials),max(t['progress_m'] for t in trials)],
                 'best_parameters':best_p.tolist(),'environment_steps':report['actual_environment_steps'],'wall_seconds':time.monotonic()-start}
            report['history'].append(row);report['status']='running';save(a.out/'search_report.json',report)
            print(json.dumps({k:v for k,v in row.items() if k not in ('development','best_parameters')}),flush=True)
        report['status']='complete';report['wall_seconds']=time.monotonic()-start
        held=list(pool.map(rollout,[(best_p.tolist(),93000+i,True,initial_pose) for i in range(24)]))
        report['held_out']=held;report['actual_environment_steps']+=sum(t['steps'] for t in held);save(a.out/'search_report.json',report)
        print(json.dumps({'status':'complete','task':a.task,'environment_steps':report['actual_environment_steps'],'successes':sum(t['is_success'] for t in held),
            'episodes':len(held),'best_edge_seconds':max(t['edge_seconds'] for t in held),'mean_progress_m':float(np.mean([t['progress_m'] for t in held]))}),flush=True)

if __name__=='__main__':main()
