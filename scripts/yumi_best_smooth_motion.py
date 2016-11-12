#!/usr/bin/env python
import numpy as np
import pickle
import yaml

import IPython

def get_cost(p1, p2):
    p1_t = np.array([p[1].translation for p in p1])
    p2_t = np.array([p[1].translation for p in p2])

    return float(np.linalg.norm(p1_t - p2_t))

if __name__ == '__main__':

    output_path = 'results/smooth_motions/'

    costs = []

    fs = [f for f in range(5,60,10)]
    ds = [0.001] + [d*0.005 for d in range(1,5)]
    zs = ['fine', 'z1']
    vs = [100, 1000, 1500]

    for i, f in enumerate(fs):
        for j, d in enumerate(ds):
            for _, v in enumerate(vs):
                if i < 2 and j < 2:
                    continue

                title =  "{0}hz_{1}mm_v{2}".format(f, d*1000, v)
                fine_title = "{0}_fine".format(title)
                z1_title = "{0}_z1".format(title)
                
                with open(output_path+fine_title+".pkl", 'r') as file:
                    fine_data = pickle.load(file)
                with open(output_path+z1_title+".pkl", 'r') as file:
                    z1_data = pickle.load(file)
                
                costs.append([title, get_cost(fine_data, z1_data)])

    costs.sort(key=lambda x:x[1])
    IPython.embed()
    with open('results/smooth_motions/analysis.yml', 'w') as file:
        yaml.dump(costs, file)

