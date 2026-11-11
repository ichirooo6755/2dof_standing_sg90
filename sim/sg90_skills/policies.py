"""Small sensor-based seed policies for reward-driven CEM and residual PPO."""
import numpy as np

class SeedPolicy:
    def __init__(self,task,parameters):self.task=task;self.p=np.array(parameters,dtype=float)
    def desired_targets(self,obs):
        if len(self.p)>=12:
            knots=np.tanh(self.p[:8].reshape(4,2))*(np.pi/2)
            durations=np.exp(self.p[8:12]);elapsed=(1-float(obs[15]))*6.
            phase=elapsed%float(durations.sum())
            stage=min(3,int(np.searchsorted(np.cumsum(durations),phase)))
            if len(self.p)==13:
                prior=0. if stage==0 else np.cumsum(durations)[stage-1]
                blend=(phase-prior)/durations[stage]
                return knots[(stage-1)%4]+blend*(knots[stage]-knots[(stage-1)%4])
            return knots[stage]
        s,c=obs[13:15];features=np.array([1,s,c,2*s*c,c*c-s*s])
        return np.tanh(self.p.reshape(2,5)@features)*(np.pi/2)
    def predict(self,obs,deterministic=True):
        obs=np.asarray(obs);q=obs[6:8]*(np.pi/2)
        if self.task=='move':
            desired=self.desired_targets(obs)
            action=np.clip((desired-q)/.10,-1,1)
        else:
            pitch=np.arctan2(obs[0],-obs[2]);gyro=obs[3:6]*10
            action=np.array([np.clip(-q[0]/.1,-.4,.4),self.p[0]*(pitch-self.p[3])+self.p[1]*gyro[1]+self.p[2]*q[1]])
        return np.clip(action,-1,1).astype(np.float32),None
    def residual_action(self,obs,delta,scale):
        if self.task=='move':return np.clip((self.desired_targets(obs)+scale*delta-obs[6:8]*(np.pi/2))/.10,-1,1).astype(np.float32)
        return np.clip(self.predict(obs)[0]+scale*delta,-1,1).astype(np.float32)

class ResidualPolicy:
    def __init__(self,seed,model,scale=.3):self.seed=seed;self.model=model;self.scale=scale
    def predict(self,obs,deterministic=True):
        delta,_=self.model.predict(obs,deterministic=deterministic)
        return self.seed.residual_action(obs,delta,self.scale),None
