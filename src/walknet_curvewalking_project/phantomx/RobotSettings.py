#!/usr/bin/env python3
import numpy

# Settings for the Phantom robot objects: defining leg namings, joint_limits.

# ====== simulation parameters ========
controller_frequency = 50
DEBUG = False

# ========== naming objects ========
leg_names = ('lf', 'rf', 'lm', 'rm', 'lr', 'rr')
wx_topics = {"lf": "left_front", "rf": "right_front", "lm": "left_middle", "rm": "right_middle", "lr": "left_back",
             "rr": "right_back"}

# alpha_rotation_dir true for all legs for which a positive rotation of alpha (c1 joint) will move the leg forward
alpha_rotation_dir = (False, True, False, True, False, True)  # ('lf', 'rf', 'lm', 'rm', 'lr', 'rr')

segment_length = [0.053, 0.06, 0.1308]  # (c1_to_thigh, thigh, tibia)
segment_masses = [0.02471, 0.110612, 0.196603]
segment_coms = numpy.array([[0, -0.02633, 0], [0, 0, 0], [0, 0, 0]])

# adjusted joint ranges in robot description
# joint_angle_limits = [[-0.5, 0.5], [-1.5, 1.2], [-1.5, 0.6]]
joint_angle_limits = [[-1.0, 1.0], [-2.25, 2.25], [-2.75, 1.0]]
# ((c1 - alpha: (lower, upper), thigh - beta: (lower, upper), tibia - gamme: (lower, upper))

lm = numpy.array([[0, -1, 0, 0],
                  [1, 0, 0, 0.1],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

rm = numpy.array([[0, 1, 0, 0],
                  [-1, 0, 0, 0.1],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

lf = numpy.array([[numpy.cos(numpy.radians(45)), -numpy.sin(numpy.radians(45)), 0, 0.125],
                  [numpy.sin(numpy.radians(45)), numpy.cos(numpy.radians(45)), 0, 0.065],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

rf = numpy.array([[numpy.cos(numpy.radians(315)), -numpy.sin(numpy.radians(315)), 0, 0.125],
                  [numpy.sin(numpy.radians(315)), numpy.cos(numpy.radians(315)), 0, -0.065],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

lr = numpy.array([[numpy.cos(numpy.radians(135)), -numpy.sin(numpy.radians(135)), 0, -0.125],
                  [numpy.sin(numpy.radians(135)), numpy.cos(numpy.radians(135)), 0, 0.065],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

rr = numpy.array([[numpy.cos(numpy.radians(225)), -numpy.sin(numpy.radians(225)), 0, -0.125],
                  [numpy.sin(numpy.radians(225)), numpy.cos(numpy.radians(225)), 0, -0.065],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

body_c1_tf = (lf, rf, lm, rm, lr, rr)
# ========== walknet settings ==========
default_stance_distance = 0.08  # 0.09  # 0.10 pep_shifted to aep_shifted
stance_height = -0.09
default_stance_width = 0.20 # 0.365/2
middle_leg_offset = 0.035

_front_initial_aep = numpy.array([0.25, default_stance_width, stance_height])
_front_initial_pep = numpy.array(
        [_front_initial_aep[0] - default_stance_distance, default_stance_width, stance_height])
_middle_initial_aep = numpy.array(
        [0.05, (default_stance_width + middle_leg_offset), stance_height])
_middle_initial_pep = numpy.array(
        [_middle_initial_aep[0] - default_stance_distance, (default_stance_width + middle_leg_offset),
         stance_height])  # -0.07# for forward walking
_hind_initial_aep = numpy.array([-0.17, default_stance_width, stance_height])
_hind_initial_pep = numpy.array([_hind_initial_aep[0] - default_stance_distance, default_stance_width, stance_height])

initial_pep = (_front_initial_pep, _middle_initial_pep, _hind_initial_pep)  # for forward walking
initial_aep = (_front_initial_aep, _middle_initial_aep, _hind_initial_aep)  # for forward walking

# initial_aep = (_front_initial_pep, _middle_initial_pep, _hind_initial_pep)  # for backwards walking
# initial_pep = (_front_initial_aep, _middle_initial_aep, _hind_initial_aep)  # for backwards walking

# == Ground Contact Parameters ========
# =====================================
# Parameter for prediction based method
# parameter describes predicted height value from which on
# a leg is predicted as in gc - is given as a percentage
# (good value: 0.8, means when leg is having a height of 0.8 * the
# intended height control value it is already assumed as having ground contact
predicted_ground_contact_height_factor = 0.9
