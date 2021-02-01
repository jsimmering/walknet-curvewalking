import numpy

# Settings for the Phantom robot objects: defining leg namings, joint_limits.

# ====== simulation parameters ========
controller_frequency = 100

# ========== naming objects ========
leg_names = ('lf', 'rf', 'lm', 'rm', 'lr', 'rr')

# alpha_rotation_dir true for all legs for which a positive rotation of alpha (c1 joint) will move the leg forward
alpha_rotation_dir = (False, True, False, True, False, True)  # ('lf', 'rf', 'lm', 'rm', 'lr', 'rr')

segment_length = (0.054, 0.066, 0.16)  # (c1_to_thigh, thigh, tibia)

joint_angle_limits = [[-0.5, 0.5], [-1.5, 1.2], [-1.5, 0.6]]
# ((c1 - alpha: (lower, upper), thigh - beta: (lower, upper), tibia - gamme: (lower, upper))

lm = numpy.array([[0, 0, 1, 0],
    [0, -1, 0, 0.1034],
    [1, 0, 0, 0.001116],
    [0, 0, 0, 1.0]])

rm = numpy.array([[0, 0, -1, 0],
    [0, 1, 0, -0.1034],
    [1, 0, 0, 0.001116],
    [0, 0, 0, 1]])

lf = numpy.array([[0, -0.707070461, 0.707143097, 0.1248],
    [0, -0.707143100, -0.707070458, 0.06164],
    [1, 0, 0, 0.001116],
    [0, 0, 0, 1]])

rf = numpy.array([[0, -0.706835167, -0.707378288, 0.1248],
    [0, 0.707378291, -0.706835165, -0.06164],
    [1, 0, 0, 0.001116],
    [0, 0, 0, 1]])

lr = numpy.array([[0, 0.706859116, 0.707354357, -0.1248],
    [0, -0.70735436, 0.706859113, 0.06164],
    [1, 0, 0, 0.001116],
    [0, 0, 0, 1]])

rr = numpy.array([[0, 0.707032355, -0.707181196, -0.1248],
    [0, 0.707181199, 0.707032353, -0.06164],
    [1, 0, 0, 0.001116],
    [0, 0, 0, 1]])

body_c1_tf = (lf, rf, lm, rm, lr, rr)
# ========== walknet settings ==========
default_stance_distance = 0.07
stance_height = -0.09
default_stance_width = 0.27

front_initial_aep = numpy.array([0.25, default_stance_width, stance_height])  # for forward walking
front_initial_pep = numpy.array(
    [front_initial_aep[0] - default_stance_distance, default_stance_width, stance_height])  # for forward walking
middle_initial_aep = numpy.array([0.04, 0.327, stance_height])  # for forward walking
middle_initial_pep = numpy.array(
    [middle_initial_aep[0] - default_stance_distance, 0.327, stance_height])  # -0.07# for forward walking
hind_initial_aep = numpy.array([-0.18, default_stance_width, stance_height])
hind_initial_pep = numpy.array([hind_initial_aep[0] - default_stance_distance, default_stance_width, stance_height])

# == Ground Contact Parameters ========
# =====================================
# Parameter for prediction based method
# parameter describes predicted height value from which on
# a leg is predicted as in gc - is given as a percentage
# (good value: 0.8, means when leg is having a height of 0.8 * the
# intended height control value it is already assumed as having ground contact
predicted_ground_contact_height_factor = 0.9
