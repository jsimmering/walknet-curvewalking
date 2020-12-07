import copy

import numpy
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from math import sin, cos, atan2, pow, pi, acos


# code based on https://github.com/malteschilling/cognitiveWalker/blob/master/Hector/LegF.py
class SingleLeg:

    def __init__(self, name, segment_lengths, rotation_dir):
        self.name = name

        self.alpha_pub = rospy.Publisher('/phantomx/j_c1_' + self.name + '_position_controller/command', Float64,
            queue_size=1)
        self.beta_pub = rospy.Publisher('/phantomx/j_thigh_' + self.name + '_position_controller/command', Float64,
            queue_size=1)
        self.gamma_pub = rospy.Publisher('/phantomx/j_tibia_' + self.name + '_position_controller/command', Float64,
            queue_size=1)

        self.alpha = None
        self.beta = None
        self.gamma = None

        self.alpha_reached = True
        self.beta_reached = True
        self.gamma_reached = True

        self.joints = (self.alpha, self.beta, self.gamma)

        self.segment_lengths = segment_lengths
        self.rotation_dir = rotation_dir

        self.ee_pos = None

    def is_ready(self):
        if self.alpha is None:
            rospy.loginfo("alpha: " + str(self.alpha))
        if self.beta is None:
            rospy.loginfo("beta: " + str(self.beta))
        if self.gamma is None:
            rospy.loginfo(" gamma: " + str(self.gamma))
        if self.alpha is not None and self.beta is not None and self.gamma is not None:
            rospy.loginfo("ready")
            return True
        else:
            return False

    def set_c1(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard that %s", data)
        self.alpha = data
        if data.error < 0.05:
            self.alpha_reached = True
        else:
            self.alpha_reached = False

    def set_thigh(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard that %s", data)
        self.beta = data
        if data.error < 0.05:
            self.beta_reached = True
        else:
            self.beta_reached = False

    def set_tibia(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard that %s", data)
        self.gamma = data
        if data.error < 0.05:
            self.gamma_reached = True
        else:
            self.gamma_reached = False

    def ee_position(self):
        self.update_ee_position()
        return self.ee_pos

    def update_ee_position(self):
        self.ee_pos = self.compute_forward_kinematics()

    def compute_forward_kinematics(self):
        if None in self.joints:
            rospy.loginfo("have not recieved joint values yet. skipp")
            return None
        alpha = JointControllerState(self.alpha).process_value
        beta = JointControllerState(self.beta).process_value
        gamma = JointControllerState(self.gamma).process_value

        tmp_ee_pos = self.alpha_forward_kinematics(alpha,
            self.beta_forward_kinematics(beta, self.gamma_forward_kinematics(gamma, numpy.array([0, 0, 0, 1]))))
        alpha_check = -atan2(tmp_ee_pos[1], (tmp_ee_pos[0]))
        if abs(alpha_check - alpha) >= 0.01:
            raise Exception('The provided angles for ' + self.name + '(' + str(alpha) + ', ' + str(beta) + ', ' + str(
                gamma) + ') are not valid for the forward/inverse kinematics.')
        return tmp_ee_pos

    # code from https://github.com/malteschilling/cognitiveWalker/blob/master/Hector/LegF.py
    def alpha_forward_kinematics(self, alpha, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        cos_alpha = cos(alpha)
        sin_alpha = sin(alpha)
        alpha_trans = numpy.array([(cos_alpha, 0, sin_alpha, (self.segment_lengths[0] * cos_alpha)),
                                   (sin_alpha, 0, -cos_alpha, self.segment_lengths[0] * sin_alpha), (0, 1, 0, 0,),
                                   (0, 0, 0, 1)])
        # print('alpha_trans: ', alpha_trans)
        return alpha_trans.dot(point)  # [0:3]

    # code from https://github.com/malteschilling/cognitiveWalker/blob/master/Hector/LegF.py
    def beta_forward_kinematics(self, beta, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        if self.rotation_dir is False:
            beta *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        cos_beta = cos(beta)
        sin_beta = sin(beta)
        beta_trans = numpy.array([(cos_beta, -sin_beta, 0, self.segment_lengths[1] * cos_beta),
                                  (sin_beta, cos_beta, 0, self.segment_lengths[1] * sin_beta), (0, 0, 1, 0,),
                                  (0, 0, 0, 1)])
        return beta_trans.dot(point)  # [0:3]

    # code from https://github.com/malteschilling/cognitiveWalker/blob/master/Hector/LegF.py
    def gamma_forward_kinematics(self, gamma, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        if self.rotation_dir is True:
            gamma *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        gamma = gamma - pi / 2
        cos_gamma = cos(gamma)
        sin_gamma = sin(gamma)
        gamma_trans = numpy.array([(cos_gamma, -sin_gamma, 0, self.segment_lengths[2] * cos_gamma),
                                   (sin_gamma, cos_gamma, 0, self.segment_lengths[2] * sin_gamma), (0, 0, 1, 0),
                                   (0, 0, 0, 1)])
        return gamma_trans.dot(point)  # [0:3]  # return self._phi_psi_trans.dot(temp_tarsus_position)[0:3]

    #  Calculation of inverse kinematics for a leg:
    #   Given a position in 3D space a joint configuration is calculated.
    #   When no position is provided the current position of the current leg is used
    #   to calculate the complementing angles.
    #   PhiPsi Inverse Transformation is applied (meaning the position is with respect
    #   to the leg coordinate system aligned with the body coordinate system)
    #   @param p point in leg coordinate system
    def compute_inverse_kinematics(self, p=None):
        if isinstance(p, (type(None))):
            p = self.ee_pos
        if len(p) == 3:
            p = numpy.append(p, [1])
        p_temp = copy.copy(p)
        # p = self._phi_psi_trans_inv.dot(p)
        alpha_angle = -atan2(p[1], (p[0]))
        beta_pos = self.alpha_forward_kinematics(alpha_angle)
        lct = numpy.linalg.norm(p[0:3] - beta_pos[0:3])
        # if (self.name=='rm'):
        #    print(p)
        try:
            gamma_angle = (acos((pow(self.segment_lengths[2], 2) + pow(self.segment_lengths[1], 2) - pow(lct, 2)) / (
                    2 * self.segment_lengths[1] * self.segment_lengths[2]))) - pi / 2
            h1 = (acos((pow(self.segment_lengths[1], 2) + pow(lct, 2) - pow(self.segment_lengths[2], 2)) / (
                    2 * self.segment_lengths[1] * lct)))
            # Avoid running in numerical rounding error
            if ((pow(lct, 2) + pow(self.segment_lengths[0], 2) - pow(numpy.linalg.norm(p[0:3]), 2)) / (
                    2 * self.segment_lengths[0] * lct) < -1.):
                h2 = pi
            else:
                h2 = (acos((pow(lct, 2) + pow(self.segment_lengths[0], 2) - pow(numpy.linalg.norm(p[0:3]), 2)) / (
                        2 * self.segment_lengths[
                    0] * lct)))  # h2 = (acos((pow(lct, 2) + pow(self.segment_lengths[0], 2) - pow(  #   #   #   #  #
                # numpy.linalg.norm(p[0:3]), 2)) /  #       (2 * self.segment_lengths[0] * lct)))
        except ValueError:
            raise ValueError('The provided position (' + str(p_temp[0]) + ', ' + str(p_temp[1]) + ', ' + str(
                p_temp[2]) + ') is not valid for the given geometry for leg ' + self.name)
        if p[2] < 0:
            beta_angle = (h1 + h2) - pi
        else:
            beta_angle = (pi - h2) + h1

        if self.rotation_dir is True:
            gamma_angle *= -1
        else:
            beta_angle *= -1

        return numpy.array([alpha_angle, beta_angle, gamma_angle])

    def get_current_angles(self):
        if None in self.joints:
            return None
        return [JointControllerState(joint).process_value for joint in self.joints]

    def get_current_targets(self):
        if None in self.joints:
            return None
        return [JointControllerState(joint).set_point for joint in self.joints]

    def set_command(self, next_angles):
        # TODO check joint ranges
        self.alpha_pub.publish(next_angles[0])
        self.beta_pub.publish(next_angles[1])
        self.gamma_pub.publish(next_angles[2])
