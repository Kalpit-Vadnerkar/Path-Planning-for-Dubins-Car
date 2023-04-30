# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 15:18:02 2023

@author: kvadner
"""

import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
import matplotlib.animation as animation

from dpp.env.car import SimpleCar
from dpp.env.environment import Environment
from dpp.test_cases.cases import TestCase
from dpp.methods.rrt_exploration import RRTExploration

from time import time


def main():

    tc = TestCase()

    env = Environment(tc.obs, lx=15, ly=15)

    car = SimpleCar(env, tc.start_pos, tc.end_pos)

    rrt_exploration = RRTExploration(car, max_steps=50, pick_target=10, check_dubins=1, max_iterations=500)

    t = time()

    nodes = rrt_exploration.search_path()

    print('Total time: {}s'.format(round(time()-t, 3)))

    nodes = nodes[1:]
    branches = []
    nodex, nodey = [], []
    
    for node in nodes:
        branches.append(node.branch)
        nodex.append(node.pos[0])
        nodey.append(node.pos[1])
   
    # plot and annimation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])

    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))

    _branches = LineCollection([], color='b', alpha=0.8, linewidth=1)
    ax.add_collection(_branches)
    _nodes, = ax.plot([], [], 'ro', markersize=4)

    _path, = ax.plot([], [], color='lime', linewidth=2)
    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _path1, = ax.plot([], [], color='whitesmoke', linewidth=2)
    _car = PatchCollection([])
    ax.add_collection(_car)
    
    frames = len(branches) + 1

    def init():
        _branches.set_paths([])
        _nodes.set_data([], [])
        _path.set_data([], [])
        _carl.set_paths([])
        _path1.set_data([], [])
        _car.set_paths([])

        return _branches, _nodes, _path, _carl, _path1, _car

    def animate(i):

        if i < len(branches):
            _branches.set_paths(branches[:i+1])
            _nodes.set_data(nodex[:i+1], nodey[:i+1])
        
        else:
            _branches.set_paths(branches)
            _nodes.set_data(nodex, nodey)

        return _branches, _nodes, _path, _carl, _path1, _car

    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, 
                                  interval=1, repeat=True, blit=True)

    plt.show()


if __name__ == '__main__':
    main()