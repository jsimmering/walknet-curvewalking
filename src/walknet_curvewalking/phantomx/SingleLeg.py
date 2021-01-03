import copy

import numpy
import rospy
import tf.transformations as transformations
from std_msgs.msg import Float64
from math import sin, cos, atan2, pow, pi, acos, radians


class SingleLeg:

    def __init__(self, name, segment_lengths, tf_listener):
        self.name = name
        self.tf_listener = tf_listener
        self.alpha_pub = rospy.Publisher('/phantomx/j_c1_' + self.name + '_position_controller/command', Float64,
            queue_size=1)
        self.beta_pub = rospy.Publisher('/phantomx/j_thigh_' + self.name + '_position_controller/command', Float64,
            queue_size=1)
        self.gamma_pub = rospy.Publisher('/phantomx/j_tibia_' + self.name + '_position_controller/command', Float64,
            queue_size=1)

        self.alpha = None
        self.beta = None
        self.gamma = None

        self.alpha_target = None
        self.beta_target = None
        self.gamma_target = None

        self.alpha_reached = True
        self.beta_reached = True
        self.gamma_reached = True

        self.segment_lengths = segment_lengths
        # self.rotation_dir = rotation_dir

        self.ee_pos = None

    def is_ready(self):
        if self.alpha is None:
            rospy.loginfo("alpha: " + str(self.alpha))
        if self.beta is None:
            rospy.loginfo("beta: " + str(self.beta))
        if self.gamma is None:
            rospy.loginfo(" gamma: " + str(self.gamma))
        if self.alpha is not None and self.beta is not None and self.gamma is not None:
            # rospy.loginfo("ready")
            return True
        else:
            return False

    def c1_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard that %s", data)
        self.alpha = data.process_value
        self.alpha_target = data.set_point
        # rospy.loginfo('set current alpha_target as: ' + str(self.alpha_target))
        if data.error < 0.01:
            self.alpha_reached = True
        else:
            self.alpha_reached = False

    def thigh_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard that %s", data)
        self.beta = data.process_value
        self.beta_target = data.set_point
        # rospy.loginfo('set current beta_target as: ' + str(self.beta_target))
        if data.error < 0.01:
            self.beta_reached = True
        else:
            self.beta_reached = False

    def tibia_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard that %s", data)
        self.gamma = data.process_value
        self.gamma_target = data.set_point
        # rospy.loginfo('set current gamma_target as: ' + str(self.gamma_target))
        if data.error < 0.01:
            self.gamma_reached = True
        else:
            self.gamma_reached = False

    def ee_position(self):
        self.update_ee_position()
        return self.ee_pos

    def update_ee_position(self):
        self.ee_pos = self.compute_forward_kinematics()

    def target_reached(self):
        return self.alpha_reached and self.beta_reached and self.gamma_reached

    # compute ee_position based on current joint values in c1 coordinate frame (= leg coordinate frame)
    # code from https://www.programcreek.com/python/example/96799/tf.transformations
    def compute_forward_kinematics_tf(self):
        if not self.is_ready():
            rospy.loginfo("haven't received Joint values yet! skipp")
            return
        (trans, rot) = self.tf_listener.lookupTransform('MP_BODY', 'tibia_' + self.name, rospy.Time(0))
        # (trans, rot) = self.tf_listener.lookupTransform('MP_BODY', 'tibia_' + self.name, rospy.Time(0))
        pos = numpy.array(transformations.quaternion_matrix(rot))
        pos[0, 3] = trans[0]
        pos[1, 3] = trans[1]
        pos[2, 3] = trans[2]
        return numpy.array(numpy.dot(pos, [0, 0, 0.13, 1]))
        # return numpy.array(numpy.dot(pos, [0, 0, 0, 1]))

    def check_joint_ranges(self, angles):
        return angles[0] >= -0.6 or angles[0] <= 0.6 or angles[1] >= -1.0 or angles[1] <= 0.3 or angles[2] >= -1.0 or \
               angles[2] <= 1.0

    # ee position in body frame
    def compute_forward_kinematics(self, angles=None):
        if angles is None:
            alpha = self.alpha
            beta = self.beta
            gamma = self.gamma
        elif self.check_joint_ranges(angles):
            alpha = angles[0]
            beta = angles[1]
            gamma = angles[2]
        else:
            raise Exception(
                'The provided angles for ' + self.name + '(' + str(angles[0]) + ', ' + str(angles[1]) + ', ' + str(
                    angles[2]) + ') are not valid for the forward/inverse kinematics.')

        c1_pos = self.body_c1_transformation(alpha)
        temp_tarsus_position = self.body_c1_transformation(alpha, self.c1_thigh_transformation(beta,
            self.thigh_tibia_transformation(gamma, self.tibia_ee_transformation())))
        # calculate shoulder angle as angle of vector from c1 pos to ee pos in body frame
        # rospy.loginfo('temp_tarsus_position_y = ' + str(temp_tarsus_position[1]) + ' temp_tarsus_position_x =
        #               ' + str(temp_tarsus_position[0]))
        # rospy.loginfo(
        #    'c1_pos_y = ' + str(c1_pos[1]) + ' c1_pos_x = ' + str(c1_pos[0]))
        x_pos = temp_tarsus_position[0] - c1_pos[0]
        y_pos = temp_tarsus_position[1] - c1_pos[1]
        # TODO probably only works for lm leg find general solution!
        alpha_check = -atan2(x_pos,
            y_pos)  # switched x and y coordination because the leg points in the direction of the y axis of the
        #             MP_BODY frame
        # rospy.loginfo('ee_y = ' + str(y_pos) + ' ee_x = ' + str(x_pos) + ' alpha_check= ' + str(alpha_check) +
        #               ' alpha = ' + str(alpha))
        if abs(alpha_check - alpha) >= 0.01:
            raise Exception('The provided angles for ' + self.name + '(' + str(alpha) + ', ' + str(beta) + ', ' + str(
                gamma) + ') are not valid for the forward/inverse kinematics.')
        return temp_tarsus_position

    def body_c1_transformation(self, alpha, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        # TODO ist alpha reversed?
        # alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        cos_alpha = cos(alpha + radians(180))
        sin_alpha = sin(alpha + radians(180))
        cos90 = cos(radians(-90))
        sin90 = sin(radians(-90))
        trans = numpy.array([(cos90, sin_alpha * sin90, cos_alpha * sin90, 0),
            (0, cos_alpha, 0 - sin_alpha, 0.1034),
            (0 - sin90, sin_alpha * cos90, cos_alpha * cos90, 0.001116),
            (0, 0, 0, 1)])
        # print('trans: ', trans)
        return trans.dot(point)

    def body_leg_transformation(self, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        # TODO ist alpha reversed?
        # alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        trans = numpy.array([(1, 0, 0, 0),
            (0, 1, 0, 0.1034),
            (0, 0, 1, 0.001116),
            (0, 0, 0, 1)])
        # print('trans: ', trans)
        return trans.dot(point)

    def leg_c1_transformation(self, alpha, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        # TODO ist alpha reversed?
        # alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        cos_alpha = cos(alpha + radians(180))
        sin_alpha = sin(alpha + radians(180))
        cos90 = cos(radians(-90))
        sin90 = sin(radians(-90))
        trans = numpy.array([(cos90, sin_alpha * sin90, cos_alpha * sin90, 0),
            (0, cos_alpha, 0 - sin_alpha, 0),
            (0 - sin90, sin_alpha * cos90, cos_alpha * cos90, 0),
            (0, 0, 0, 1)])
        # print('trans: ', trans)
        return trans.dot(point)

    def c1_thigh_transformation(self, alpha, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        # TODO ist alpha reversed?
        # alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        cos_alpha = cos(alpha)
        sin_alpha = sin(alpha)
        cos90 = cos(radians(90))
        sin90 = sin(radians(90))
        trans = numpy.array([(cos90, sin_alpha * sin90, cos_alpha * sin90, 0),
            (0, cos_alpha, 0 - sin_alpha, -0.054),
            (0 - sin90, sin_alpha * cos90, cos_alpha * cos90, 0),
            (0, 0, 0, 1)])
        # print('trans: ', trans)
        return trans.dot(point)

    def thigh_tibia_transformation(self, alpha, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        # TODO ist alpha reversed?
        # alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        cos_alpha = cos(alpha)
        sin_alpha = sin(alpha)
        cos90 = cos(radians(180))
        sin90 = sin(radians(180))
        trans = numpy.array([(cos90, sin_alpha * sin90, cos_alpha * sin90, 0),
            (0, cos_alpha, 0 - sin_alpha, -0.0645),
            (0 - sin90, sin_alpha * cos90, cos_alpha * cos90, -0.0145),
            (0, 0, 0, 1)])
        # print('trans: ', trans)
        return trans.dot(point)

    def tibia_ee_transformation(self, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        # TODO ist alpha reversed?
        # alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        trans = numpy.array([(1, 0, 0, 0),
            (0, 1, 0, -0.16),
            (0, 0, 1, 0.02),
            (0, 0, 0, 1)])
        # print('trans: ', trans)
        return trans.dot(point)

    def body_c1_transform(self, point=[0, 0, 0, 1]):
        # (trans, rot) = self.tf_listener.lookupTransform('MP_BODY', 'thigh_' + self.name, rospy.Time(0))
        (trans, rot) = self.tf_listener.lookupTransform('MP_BODY', 'c1_' + self.name, rospy.Time(0))
        pos = numpy.array(transformations.quaternion_matrix(rot))
        pos[0, 3] = trans[0]
        pos[1, 3] = trans[1]
        pos[2, 3] = trans[2]
        return numpy.array(numpy.dot(pos, point))

    # code from https://www.programcreek.com/python/example/96799/tf.transformations
    def alpha_forward_kinematics(self, point=[0, 0, 0, 1]):
        # (trans, rot) = self.tf_listener.lookupTransform('MP_BODY', 'thigh_' + self.name, rospy.Time(0))
        (trans, rot) = self.tf_listener.lookupTransform('c1_' + self.name, 'thigh_' + self.name, rospy.Time(0))
        pos = numpy.array(transformations.quaternion_matrix(rot))
        pos[0, 3] = trans[0]
        pos[1, 3] = trans[1]
        pos[2, 3] = trans[2]
        return numpy.array(numpy.dot(pos, point))

    # code from https://www.programcreek.com/python/example/96799/tf.transformations
    def beta_forward_kinematics(self, point=[0, 0, 0, 1]):
        (trans, rot) = self.tf_listener.lookupTransform('thigh_' + self.name, 'tibia_' + self.name, rospy.Time(0))
        pos = numpy.array(transformations.quaternion_matrix(rot))
        pos[0, 3] = trans[0]
        pos[1, 3] = trans[1]
        pos[2, 3] = trans[2]
        return numpy.array(numpy.dot(pos, point))

    #  Calculation of inverse kinematics for a leg:
    #   Given a position in 3D space a joint configuration is calculated.
    #   When no position is provided the current position of the current leg is used
    #   to calculate the complementing angles.
    #   @param p point in body coordinate system
    def compute_inverse_kinematics(self, p=None):
        rospy.loginfo("in compute inverse kinematics")
        if isinstance(p, (type(None))):
            p = self.ee_position()
        if len(p) == 3:
            p = numpy.append(p, [1])
        p_temp = copy.copy(p)
        # rospy.loginfo('ee_pos target = ' + str(p))

        c1_pos = self.body_c1_transformation(self.alpha)
        # alpha_angle: float = -atan2(p[1], (p[0]))
        # TODO probably only works for lm leg find general solution!
        # rospy.loginfo('c1_pos = ' + str(c1_pos))
        # switched x and y coordination because the leg points in the direction of the y axis of the MP_BODY frame:
        alpha_angle = -atan2(p[0] - c1_pos[0], p[1] - c1_pos[1])
        beta_pos = self.body_c1_transformation(alpha_angle, self.c1_thigh_transformation(0))
        lct = numpy.linalg.norm(p[0:3] - beta_pos[0:3])
        # rospy.loginfo('beta_pos = ' + str(beta_pos) + ' h = ' + str(lct))

        default_gamma_pos = self.body_c1_transformation(alpha_angle,
            self.c1_thigh_transformation(0, self.thigh_tibia_transformation(0)))
        thigh_tibia_angle = -atan2(default_gamma_pos[2] - beta_pos[2], default_gamma_pos[1] - beta_pos[1])
        # rospy.loginfo('thigh_tibia_angle = ' + str(thigh_tibia_angle))
        tibia_z_angle = pi - atan2(0.02, -0.16)
        # rospy.loginfo('tibia_z_angle = ' + str(tibia_z_angle))
        # if (self.name=='rm'):
        #    print(p)
        try:
            gamma_inner = (acos((pow(self.segment_lengths[2], 2) + pow(self.segment_lengths[1], 2) - pow(lct, 2)) / (
                    2 * self.segment_lengths[1] * self.segment_lengths[2])))  # - pi / 2
            # gamma_angle = pi - gamma_inner
            gamma_angle = gamma_inner - pi - tibia_z_angle
            # rospy.loginfo(
            #    'gamma_angle = ' + str(gamma_angle) + ' gamma_inner = ' + str(gamma_inner) + ' pi = ' + str(pi))
            # TODO gamma not exactly correct but not exactly wrong
            h1 = (acos((pow(self.segment_lengths[1], 2) + pow(lct, 2) - pow(self.segment_lengths[2], 2)) / (
                    2 * self.segment_lengths[1] * lct)))
            # rospy.loginfo('beta_inner = ' + str(h1))
            ee_angle = -atan2(p[2] - beta_pos[2], p[1] - beta_pos[1])
            # rospy.loginfo('ee_angle = ' + str(ee_angle))
            # rospy.loginfo('beta atan = ' + str(ee_angle - h1))
            # Avoid running in numerical rounding error
            # rospy.loginfo('c1 to ee vector = ' + str(numpy.linalg.norm(p[0:3] - c1_pos[0:3])))
            vector_c1_ee = numpy.linalg.norm(p[0:3] - c1_pos[0:3])
            # rospy.loginfo(
            #    'lct =  ' + str(lct) + ' c1_link length = ' + str(self.segment_lengths[0]) + ' c1->ee_vector = ' + str(
            #        vector_c1_ee))
            cos_beta = (pow(lct, 2) + pow(self.segment_lengths[0], 2) - pow(vector_c1_ee, 2)) / (
                    2 * lct * self.segment_lengths[0])
            # rospy.loginfo('cos_beta = ' + str(cos_beta))
            if (cos_beta < -1.):
                rospy.loginfo('----------------PI----------')
                h2 = pi
            else:
                h2 = (acos(cos_beta))
            # rospy.loginfo('beta_outer = ' + str(h2))
        except ValueError:
            raise ValueError('The provided position (' + str(p_temp[0]) + ', ' + str(p_temp[1]) + ', ' + str(
                p_temp[2]) + ') is not valid for the given geometry for leg ' + self.name)
        if p[2] < 0:
            # beta_angle = (h1 + h2 + thigh_tibia_angle) - pi  # with rotation_dir
            beta_angle = pi - (h1 + h2 + thigh_tibia_angle)
            # rospy.loginfo('ee lower than body')
        else:
            # rospy.loginfo('ee higher than body')
            # beta_angle = h1 - h2 + pi + thigh_tibia_angle  # with rotation_dir
            beta_angle = h1 - h2 - pi - thigh_tibia_angle

        # if self.rotation_dir is True:
        #    gamma_angle *= -1
        # else:
        #    beta_angle *= -1
        rospy.loginfo(
            "return angles alpha = " + str(alpha_angle) + " beta = " + str(beta_angle) + " gamme = " + str(gamma_angle))
        return numpy.array([alpha_angle, beta_angle, gamma_angle])

    def get_current_angles(self):
        if self.alpha is None or self.beta is None or self.gamma is None:
            return None
        return [self.alpha, self.beta, self.gamma]

    def get_current_targets(self):
        rospy.loginfo('in get_current_targets; targets are: ' + str(self.alpha_target) + ', ' + str(
            self.beta_target) + ', ' + str(self.gamma_target))
        if self.alpha_target is None or self.beta_target is None or self.gamma_target is None:
            return None
        return [self.alpha_target, self.beta_target, self.gamma_target]

    def set_command(self, next_angles):
        # TODO check joint ranges
        rospy.loginfo(
            "set command. angles = " + str(next_angles) + " current angles = " + str(self.get_current_angles()))
        self.alpha_pub.publish(next_angles[0])
        self.beta_pub.publish(next_angles[1])
        self.gamma_pub.publish(next_angles[2])
