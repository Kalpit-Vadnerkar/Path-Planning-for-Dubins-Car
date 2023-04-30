"""
Created on Mon Apr 27 22:51:01 2023

@author: vasants
"""

from math import pi

class TestCase:
    """ Provide some test cases for a given map. """

    def __init__(self):

        self.start_pos = [1, 1, 0]
        self.end_pos = [14, 14, pi/2]

        self.start_pos2 = [4, 4, 0]
        self.end_pos2 = [4, 8, 1.2*pi]

        self.obs = [
            [5.5, 5.5, 4, 4]
        ]
