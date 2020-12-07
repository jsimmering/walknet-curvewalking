import rospy
import copy
import numpy
from math import sin, cos, atan2, pow, pi, acos
from control_msgs.msg import JointControllerState

"""Took code from: https://github.com/malteschilling/cognitiveWalker/blob/master/Hector/LegF.py 
modified for PhantomX Robot"""


class Leg:

    def c1_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard from %s that %s", args, data.error)
        self.alpha = data

    def thigh_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard from %s that %s", args, data.error)
        self.beta = data

    def tibia_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard from %s that %s", args, data.error)
        self.gamma = data

    def __init__(self, name, segment_lengths, segment_masses, segment_centers_of_mass, phi, psi, chi, beta_direction):
        self.name = name

        self.alpha_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
                                          JointControllerState, self.thigh_callback)
        self.beta_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
                                         JointControllerState, self.thigh_callback)
        self.gamma_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
                                          JointControllerState, self.thigh_callback)

        self.alpha = None
        self.beta = None
        self.gamma = None

        self.joints = (self.alpha, self.beta, self.gamma)
        self.last_pos = 0

        self.segment_lengths = segment_lengths
        self.segment_masses = segment_masses
        self.mass = sum(self.segment_masses)
        self.segment_centers_of_mass = segment_centers_of_mass
        self.phi = phi
        self.psi = psi
        self.chi = chi
        # print("INIT " , name, " phi: ", phi, " - psi: ", psi)
        if 'r' in name:
            self.left_leg = False
        else:
            self.left_leg = True

        self.beta_direction = beta_direction
        self._phi_psi_trans = self.__phi_psi_transform()
        self._phi_psi_trans_inv = numpy.linalg.inv(self._phi_psi_trans)
        self._input_foot_position = None
        self._output_foot_position = None

        self._center_of_mass = None

        # Variables for detection of ground contact
        self.contact = 0
        self.contTime = 0
        self.contactOld = 0
        self.lift_counter = 0

        self.input_velocities = [0, 0, 0]

        # This is the end of the working range
        self.min_x = -0.3

    @property
    def input_foot_position(self):
        if self._input_foot_position is None:
            self.update_input_foot_position()
        return self._input_foot_position

    def update_input_foot_position(self):
        self._input_foot_position = self.compute_forward_kinematic()

    def compute_forward_kinematic(self, angles=None):
        if angles is None:
            alpha = self.alpha.inputPosition
            beta = self.beta.inputPosition
            gamma = self.gamma.inputPosition
        else:
            alpha = angles[0]
            beta = angles[1]
            gamma = angles[2]

        temp_tarsus_position = self._alpha_forward_kinematics(alpha, self._beta_forward_kinematics(beta,
                                                                                                   self._gamma_forward_kinematics(
                                                                                                       gamma,
                                                                                                       numpy.array(
                                                                                                           [0, 0, 0,
                                                                                                            1]))))
        # inverse kinematics alpha angle check
        alpha_check = -atan2(temp_tarsus_position[1], (temp_tarsus_position[0]))
        if abs(alpha_check - alpha) >= 0.01:
            raise Exception('The provided angles for ' + self.name + '(' + str(alpha) + ', ' + str(beta) + ', ' + str(
                gamma) + ') are not valid for the forward/inverse kinematics.')
        return self._phi_psi_trans.dot(temp_tarsus_position)[0:3]

    def compute_center_of_mass(self, angles=None):
        if angles is None:
            alpha = self.alpha.inputPosition
            beta = self.beta.inputPosition
            gamma = self.gamma.inputPosition
        else:
            alpha = angles[0]
            beta = angles[1]
            gamma = angles[2]

        center_of_mass_of_coxa = self._alpha_forward_kinematics(alpha, numpy.append(self.segment_centers_of_mass[0], 1))
        center_of_mass_of_femur = self._alpha_forward_kinematics(alpha, self._beta_forward_kinematics(beta,
                                                                                                      numpy.append(
                                                                                                          self.segment_centers_of_mass[
                                                                                                              1], 1)))
        center_of_mass_of_tibia = self._alpha_forward_kinematics(alpha, self._beta_forward_kinematics(beta,
                                                                                                      self._gamma_forward_kinematics(
                                                                                                          gamma,
                                                                                                          numpy.append(
                                                                                                              self.segment_centers_of_mass[
                                                                                                                  2],
                                                                                                              1))))
        center_of_mass = (center_of_mass_of_coxa * self.segment_masses[0] + center_of_mass_of_femur *
                          self.segment_masses[1] + center_of_mass_of_tibia * self.segment_masses[2]) / sum(
            self.segment_masses)
        return (self._phi_psi_trans.dot(center_of_mass))[0:3]

    def _alpha_forward_kinematics(self, alpha, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        cos_alpha = cos(alpha)
        sin_alpha = sin(alpha)
        alpha_trans = numpy.array([(cos_alpha, 0, sin_alpha, (self.segment_lengths[0] * cos_alpha)),
                                   (sin_alpha, 0, -cos_alpha, self.segment_lengths[0] * sin_alpha), (0, 1, 0, 0,),
                                   (0, 0, 0, 1)])
        # print('alpha_trans: ', alpha_trans)
        return alpha_trans.dot(point)  # [0:3]

    def _beta_forward_kinematics(self, beta, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        if self.beta_direction is False:
            beta *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        cos_beta = cos(beta)
        sin_beta = sin(beta)
        beta_trans = numpy.array([(cos_beta, -sin_beta, 0, self.segment_lengths[1] * cos_beta),
                                  (sin_beta, cos_beta, 0, self.segment_lengths[1] * sin_beta), (0, 0, 1, 0,),
                                  (0, 0, 0, 1)])
        return beta_trans.dot(point)  # [0:3]

    def _gamma_forward_kinematics(self, gamma, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        if self.beta_direction is True:
            gamma *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        gamma = gamma - pi / 2
        cos_gamma = cos(gamma)
        sin_gamma = sin(gamma)
        gamma_trans = numpy.array([(cos_gamma, -sin_gamma, 0, self.segment_lengths[2] * cos_gamma),
                                   (sin_gamma, cos_gamma, 0, self.segment_lengths[2] * sin_gamma), (0, 0, 1, 0),
                                   (0, 0, 0, 1)])
        return gamma_trans.dot(point)  # [0:3]

    def compute_inverse_kinematic(self, p=None):
        if isinstance(p, (type(None))):
            p = self.input_foot_position
        if len(p) == 3:
            p = numpy.append(p, [1])
        p_temp = copy.copy(p)
        p = self._phi_psi_trans_inv.dot(p)
        alpha_angle = -atan2(p[1], (p[0]))
        beta_pos = self._alpha_forward_kinematics(alpha_angle)
        lct = numpy.linalg.norm(p[0:3] - beta_pos[0:3])
        # if (self.name=='middle_right_leg'):
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
                    0] * lct)))  # h2 = (acos((pow(lct, 2) + pow(self.segment_lengths[0], 2) - pow(  #   #   #  #  #
                # numpy.linalg.norm(p[0:3]), 2)) /  #       (2 * self.segment_lengths[0] * lct)))
        except ValueError:
            raise ValueError('The provided position (' + str(p_temp[0]) + ', ' + str(p_temp[1]) + ', ' + str(
                p_temp[2]) + ') is not valid for the given geometry for leg ' + self.name + '.' + str(self.last_pos))
        if p[2] < 0:
            beta_angle = (h1 + h2) - pi
        else:
            beta_angle = (pi - h2) + h1

        if self.beta_direction is True:
            gamma_angle *= -1
        else:
            beta_angle *= -1

        return numpy.array([alpha_angle, beta_angle, gamma_angle])

    # Pre calculate phiPsi transformation once.
    def __phi_psi_transform(self):
        phi_trans = numpy.array(
            [(cos(self.phi), 0, sin(self.phi), 0), (sin(self.phi), 0, -cos(self.phi), 0), (0, 1, 0, 0), (0, 0, 0, 1)])
        psi_trans = numpy.array(
            [(cos(self.psi), 0, -sin(self.psi), 0), (sin(self.psi), 0, cos(self.psi), 0), (0, -1, 0, 0), (0, 0, 0, 1)])
        chi_trans = numpy.array(
            [(cos(self.chi), -sin(self.chi), 0, 0), (sin(self.chi), cos(self.chi), 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)])
        return phi_trans.dot(psi_trans).dot(chi_trans)
