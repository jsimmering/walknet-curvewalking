# Settings for the Phantom robot objects: defining leg namings, joint_limits.

# ====== simulation parameters ========
controller_frequency = 100

# ========== naming objects ========
leg_names = ('lf', 'rf', 'lm', 'rm', 'lr', 'rr')

# alpha_rotation_dir true for all legs for which a positive rotation of alpha (c1 joint) will move the leg forward
alpha_rotation_dir = (False, True, False, True, False, True)  # ('lf', 'rf', 'lm', 'rm', 'lr', 'rr')

segment_length = (0.054, 0.066, 0.16) # (c1_to_thigh, thigh, tibia)

joint_angle_limits = ((-0.6, 0.6), (-1.0, 0.3), (-1.0, 1.0))
# ((c1 - alpha: (lower, upper), thigh - beta: (lower, upper), tibia - gamme: (lower, upper))
