#!/usr/bin/env python
import argparse, copy, numpy, random, yaml
import matplotlib.pyplot as plt
from util import *

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Plot the primitives in a primitives file")
    parser.add_argument('--filename', type=str, default='primitives.yaml')

    args = parser.parse_args()

    with open(args.filename, 'r') as f:
        doc = yaml.load(f.read())

    actions = doc['actions']
    for angle in actions:
        ang = angle['angle']
        poses = angle['poses']
        xvals = []
        yvals = []
        tvals = []
        for action in poses:
            for coord in action:
                xvals.append(coord[0])
                yvals.append(coord[1])
                tvals.append(coord[2])
                
        plt.plot(xvals, yvals, '.b')
        title_str = 'Angle %d' % ang
        plt.title(title_str)
        plt.show()
