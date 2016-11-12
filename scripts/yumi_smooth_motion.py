#!/usr/bin/env python
import numpy as np
from time import sleep
from alan.control import YuMiRobot, YuMiSubscriber
import matplotlib.pyplot as plt
import pickle

import IPython

def collect_circle_data(sub, y, f, d, z, v, p):
    y.set_v(v)
    y.set_z(z)

    r = 0.05
    dT = np.arcsin(d/2./r)
    y.left.goto_pose(p)
    T = 0

    period = 1./f

    data = []
    tp = p.copy()
    sub.reset_time()

    while T < 4 * np.pi:
        dx = r * np.cos(T)
        dy = r * np.sin(T)
        
        tp.translation[0] = p.translation[0] + dx
        tp.translation[1] = p.translation[1] + dy
        y.left.goto_pose(tp)

        data.append(sub.left.get_pose())
        sleep(period)
        T += dT

    return data

def save_data(output_path, title, data):
    # saving x-direction as plot of time
    times = []
    xs = []
    for t, p in data:
        times.append(t)
        xs.append(p.translation[0])

    fig = plt.figure(figsize=(12,8))
    ax = fig.gca()
    ax.scatter(times, xs, label='x vs time')
    ax.set_title("X vs. Time: {0}".format(title))
    ax.set_xlabel("Time (seconds)")
    ax.set_ylabel("X in left arm (meters)")
    fig.savefig(output_path, format='png')

if __name__ == '__main__':
    y = YuMiRobot()
    sub = YuMiSubscriber()
    sub.start()

    y.reset_home()
    p = y.left.get_pose()
    p.translation[0] += 0.15
    p.translation[1] += 0.15    
    
    fs = [f for f in range(5,60,10)]
    ds = [0.001] + [d*0.005 for d in range(1,5)]
    zs = ['fine', 'z1']
    vs = [100, 1000, 1500]

    for i, f in enumerate(fs):
        for j, d in enumerate(ds):
            for _, z in enumerate(zs):
                for _, v in enumerate(vs):
                    if i < 2 and j < 2:
                        continue
                    title = "{0}hz_{1}mm_v{2}_{3}".format(f, d*1000, v, z)
                    print title
                    data = collect_circle_data(sub, y, f, d, z, v, p)
                    
                    output_path = "results/smooth_motions/{0}".format(title)
                    save_data(output_path+".png", title, data)
                    with open(output_path+".pkl", 'w') as file:
                        pickle.dump(data, file)

    sub.stop()
    y.stop()