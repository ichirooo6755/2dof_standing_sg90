"""Render four held-out MuJoCo rollouts and plot recorded training metrics."""
from pathlib import Path
import argparse,json
from env import SG90Env,HERE
import mujoco,numpy as np,torch
from stable_baselines3 import PPO
from PIL import Image,ImageDraw,ImageFont
import imageio.v2 as imageio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,default=HERE/'results/run_20261111')
    p.add_argument('--seconds',type=float,default=10);a=p.parse_args();torch.set_num_threads(1)
    report=json.loads((a.run/'training_report.json').read_text())
    steps=[s['actual_steps'] for s in report['stages']]
    rates=[100*s['evaluation']['successes']/s['evaluation']['episodes'] for s in report['stages']]
    times=[s['evaluation']['mean_capped_recovery_s'] for s in report['stages']]
    fig,axes=plt.subplots(1,2,figsize=(10,3.5),layout='constrained')
    axes[0].plot(steps,rates,'o-');axes[0].set(title='Development: randomized 12 cases',xlabel='PPO steps',ylabel='Recovery success (%)',ylim=(0,105))
    axes[1].plot(steps,times,'o-');axes[1].set(title='Lower is better (20s cap)',xlabel='PPO steps',ylabel='Mean recovery time (s)')
    for ax in axes:ax.grid(alpha=.25);ax.ticklabel_format(axis='x',style='sci',scilimits=(0,0))
    fig.savefig(a.run/'learning_curve.png',dpi=160);plt.close(fig)
    actor=PPO.load(a.run/'best_trained.zip',device='cpu');envs=[];renderers=[];observations=[];cameras=[];first=[None]*4;fault=[False]*4
    font=ImageFont.truetype(str(Path(matplotlib.get_data_path())/'fonts/ttf/DejaVuSans.ttf'),14)
    try:
        for j,index in enumerate((0,2,4,6)):
            env=SG90Env(randomize=True);obs,_=env.reset(seed=88000+index,options={'tilt':np.pi/2,'azimuth':index*np.pi/4})
            cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.07];cam.distance=.41;cam.azimuth=135;cam.elevation=-22
            envs.append(env);observations.append(obs);cameras.append(cam);renderers.append(mujoco.Renderer(env.model,height=360,width=480))
        writer=imageio.get_writer(str(a.run/'recovery_four_directions.mp4'),fps=25,codec='libx264',quality=7,macro_block_size=1)
        try:
            for frame in range(round(a.seconds*25)):
                panels=[]
                for j,(env,renderer,cam) in enumerate(zip(envs,renderers,cameras)):
                    for sub in range(2):
                        if fault[j]:break
                        observations[j],_,_,_,info=env.step(actor.predict(observations[j],deterministic=True)[0])
                        if info['is_success'] and first[j] is None:first[j]=env.steps*.02
                        fault[j]|=info['control_fault']
                    cam.lookat[:2]=env.data.qpos[:2];renderer.update_scene(env.data,camera=cam)
                    panel=Image.fromarray(renderer.render());draw=ImageDraw.Draw(panel)
                    draw.rectangle((0,0,480,66),fill=(20,25,33))
                    name=('rotation axis +X','rotation axis +Y','rotation axis -X','rotation axis -Y')[j]
                    tilt=np.rad2deg(np.arccos(np.clip(env.data.xmat[env.base_id].reshape(3,3)[2,2],-1,1)))
                    state='FAULT (simulation stopped)' if fault[j] else f'recovered at {first[j]:.2f}s' if first[j] is not None else 'recovering'
                    draw.text((10,6),f'{name} | seed {88000+2*j} | {env.steps*.02:.2f}s',font=font,fill='white')
                    draw.text((10,26),f'{state}; base tilt {tilt:.1f} deg',font=font,fill=(130,235,170) if first[j] is not None else 'white')
                    draw.text((10,46),'MuJoCo simulation / SG90 / assumed electronics placement',font=font,fill=(200,200,210))
                    panels.append(np.asarray(panel))
                writer.append_data(np.concatenate([np.concatenate(panels[:2],axis=1),np.concatenate(panels[2:],axis=1)],axis=0))
        finally:writer.close()
        (a.run/'video_rollouts.json').write_text(json.dumps({'model':'best_trained.zip','first_stable_seconds':first,'control_faults':fault,'seeds':[88000,88002,88004,88006],
            'simulation_seconds':a.seconds,'post_success':'Policy continues running after the first success; no state is frozen except on control fault','hardware_commands_sent':False},indent=2)+'\n')
        print(json.dumps({'video':str(a.run/'recovery_four_directions.mp4'),'first_stable_seconds':first,'faults':fault}),flush=True)
    finally:
        for renderer in renderers:renderer.close()
        for env in envs:env.close()

if __name__=='__main__':main()
