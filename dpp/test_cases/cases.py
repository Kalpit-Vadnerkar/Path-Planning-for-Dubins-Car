"""
Created on Mon Apr 27 22:51:01 2023

@author: vasants
"""

from math import pi


class TestCase:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self):

        self.start_pos = [4.6, 2.4, 0]
        self.end_pos = [1.6, 8, -pi/2]

        self.start_pos2 = [4, 4, 0]
        self.end_pos2 = [4, 8, 1.2*pi]

        self.obs = [
            [6, 0, 0.3, 4],
            [0, 4, 3.5, 0.3],
            [0, 7, 3.5, 0.1],
            [5.5, 6, 5, 0.1],
            [2, 9, 0.3, 1.8],
            [6, 9, 0.3, 1.8]
        ]
