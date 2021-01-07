import numpy

# Settings for the Phantom robot objects: defining leg namings, joint_limits.

# ====== simulation parameters ========
controller_frequency = 100

# ========== naming objects ========
leg_names = ('lf', 'rf', 'lm', 'rm', 'lr', 'rr')

# alpha_rotation_dir true for all legs for which a positive rotation of alpha (c1 joint) will move the leg forward
alpha_rotation_dir = (False, True, False, True, False, True)  # ('lf', 'rf', 'lm', 'rm', 'lr', 'rr')

segment_length = (0.054, 0.066, 0.16)  # (c1_to_thigh, thigh, tibia)

joint_angle_limits = ((-0.6, 0.6), (-1.0, 0.3), (-1.0, 1.0))
# ((c1 - alpha: (lower, upper), thigh - beta: (lower, upper), tibia - gamme: (lower, upper))

# ========== walknet settings ==========
default_stance_distance = 0.09
stance_height = -0.15
default_stance_width = 0.25

front_initial_aep = numpy.array([0.05, 0.2, stance_height])  # for forward walking
front_initial_pep = numpy.array(
    [front_initial_aep[0] - default_stance_distance, default_stance_width, stance_height])  # for forward walking
middle_initial_aep = numpy.array([0.05, 0.28, stance_height])  # for forward walking
middle_initial_pep = numpy.array(
    [middle_initial_aep[0] - default_stance_distance, 0.28, stance_height])  # -0.07# for forward walking
hind_initial_aep = numpy.array([0.05, default_stance_width, stance_height])
hind_initial_pep = numpy.array([hind_initial_aep[0] - default_stance_distance, 0.2, stance_height])

#== Ground Contact Parameters ========
#=====================================
# Parameter for prediction based method
# parameter describes predicted height value from which on
# a leg is predicted as in gc - is given as a percentage
# (good value: 0.8, means when leg is having a height of 0.8 * the
# intended height control value it is already assumed as having ground contact
predicted_ground_contact_height_factor = 0.9