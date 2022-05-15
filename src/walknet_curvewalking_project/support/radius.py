#!/usr/bin/env python3
import argparse
import math
import os
import re
import sys

import PyKDL
import matplotlib.pyplot as plt
import numpy
import numpy as np
from matplotlib import ticker


# https://stackoverflow.com/questions/55816902/finding-the-intersection-of-two-circles
def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    # non intersecting
    if d > r0 + r1:
        return None
    # One circle within other
    if d < abs(r0 - r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(r0 ** 2 - a ** 2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return x3, y3, x4, y4


def calculate_radii(angle, distance):
    relative_angle_z = angle
    radius = distance / math.sqrt(2 * (1 - math.cos(math.radians(relative_angle_z))))
    (x1, y1, x2, y2) = get_intersections(0, 0, radius, distance, 0, radius)
    print("centroid = x1 {}, y1 {}, x2 {}, y2 {}".format(x1, y1, x2, y2))
    print("radius = {}".format(radius))
    # return x2, y2

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='calculate radius from distance between to points and relative angle between position orientations.')
    parser.add_argument("-d", "--distance", help='distance between two positions.', type=float)
    parser.add_argument("-a", "--alpha", help='angle at start position.', type=float)
    parser.add_argument("-b", "--beta", help='angle at stop position.', type=float)
    parser.add_argument("-r", "--angle", help='relative angle between two positions.', type=float)

    args = parser.parse_args()

    relative_angle = None
    if args.alpha and args.beta:
        relative_angle = args.alpha - args.beta
    if args.angle:
        relative_angle = args.angle

    print("angle = " + str(relative_angle))
    calculate_radii(relative_angle, args.distance)
