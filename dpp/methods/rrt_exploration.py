# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 15:15:44 2023

@author: kvadner
"""

import numpy as np
from math import sin, atan2, atan

from dpp.methods.rrt import RRT
from dpp.methods.rrt import Node
from dpp.methods.dubins_path import DubinsPath
from dpp.utils.utils import distance

class RRTExploration(RRT):
    """ RRT algorithm for complete map exploration. """

    def __init__(self, car, max_steps=50, pick_target=10, check_dubins=1, max_iterations=100):
        super().__init__(car, max_steps, pick_target, check_dubins)
        self.max_iterations = max_iterations

    def search_path(self):
        """ Search and explore the map. """

        nodes = [self.start]
        
        count = 0
        for _ in range(self.max_iterations):
            count += 1

            if count % self.pick_target == 0:
                pick = self.goal.pos[:2]
            else:
                pick = self.car.random_pos()[:2]
            
            nearest = self.get_nearest_node(nodes, pick)

            phi = self.get_steering_angle(nearest.pos, pick)
            pos = nearest.pos
            branch = [pos[:2]]
            
            for i in range(self.max_steps):
                pos = self.car.step(pos, phi)
                branch.append(pos[:2])

            # check safety of route-----------------------
            if phi == 0:
                safe = self.dubins.is_straight_route_safe(nearest.pos, pos)
            else:
                d, c, r = self.car.get_params(nearest.pos, phi)
                safe = self.dubins.is_turning_route_safe(nearest.pos, pos, d, c, r)
            # --------------------------------------------
            
            if not safe:
                continue
            
            new_node = Node(pos, phi, i+1)
            
            if new_node in nodes:
                continue
            
            new_node.branch = branch
            new_node.parent = nearest
            nodes.append(new_node)

        print('Total iteration:', count)
        
        return nodes
