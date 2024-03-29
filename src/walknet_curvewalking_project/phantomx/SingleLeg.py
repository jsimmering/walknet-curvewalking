#!/usr/bin/env python3
from math import sin, cos, atan2, pow, pi, acos, radians

import numpy
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC


class SingleLeg:

    def __init__(self, name, movement_dir, use_step_length):
        self.name = name
        self._alpha_pub = rospy.Publisher('/phantomx/j_c1_' + self.name + '_position_controller/command', Float64,
                queue_size=1)
        self._beta_pub = rospy.Publisher('/phantomx/j_thigh_' + self.name + '_position_controller/command', Float64,
                queue_size=1)
        self._gamma_pub = rospy.Publisher('/phantomx/j_tibia_' + self.name + '_position_controller/command', Float64,
                queue_size=1)

        self.viz_pub_rate = rospy.Rate(RSTATIC.controller_frequency)

        self.use_step_length = use_step_length

        self.alpha = None
        self.beta = None
        self.gamma = None
        self._c1_static_transform = RSTATIC.body_c1_tf[RSTATIC.leg_names.index(self.name)].copy()

        self.alpha_set_point = None
        self.beta_set_point = None
        self.gamma_set_point = None

        self.alpha_command = None
        self.beta_command = None
        self.gamma_command = None

        self.alpha_target = None
        self.beta_target = None
        self.gamma_target = None

        self.alpha_reached = True
        self.beta_reached = True
        self.gamma_reached = True

        self._segment_lengths = RSTATIC.segment_length.copy()
        self._segment_masses = RSTATIC.segment_masses.copy()
        self.mass = sum(self._segment_masses)
        self._segment_centers_of_mass = RSTATIC.segment_coms
        self._center_of_mass = None

        self.movement_dir = movement_dir

        self.ee_pos = None

        self.pep_thresh = RSTATIC.initial_pep[RSTATIC.leg_names.index(self.name) // 2][0].copy()
        self.aep_thresh = RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name) // 2][0].copy()

        if self.use_step_length:
            self.default_step_length = RSTATIC.default_stance_distance
            self.step_length = RSTATIC.default_stance_distance
        default_aep = RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name) // 2].copy()
        self.default_aep = numpy.array([default_aep[0], self.movement_dir * default_aep[1], default_aep[2]])

        self.current_stance_start = self.default_aep.copy()

        self.pep_shift_ipsilateral = 0
        self.pep_shift_ipsilateral_front = 0
        self.pep_shift_contralateral = 0

        default_gamma_pos = self.c1_rotation(0, self.c1_thigh_transformation(0, self.thigh_tibia_transformation(0)))
        default_beta_pos = self.c1_rotation(0, self.c1_thigh_transformation(0))
        self.thigh_tibia_angle = -atan2(default_gamma_pos[0] - default_beta_pos[0],
                                        -default_gamma_pos[1] + default_beta_pos[1])  # 0.2211...
        self.tibia_z_angle = pi - atan2(0.02, -0.16)  # 0.12435499454676124
        # rospy.loginfo(self.name + " thigh_tibia_angle = {} tibia_z_angle = {}".format(self.thigh_tibia_angle, self.tibia_z_angle))

        # publish pep visualization
        self.viz = False
        if self.viz:
            self.visualization_pub = rospy.Publisher('/kinematics', Marker, queue_size=1)
            self.pep_thresh_line = Marker()
            self.pep_init_thresh_line = Marker()
            self.aep_line = Marker()
            self.pep_thresh_line.header.frame_id = self.aep_line.header.frame_id = self.pep_init_thresh_line.header.frame_id = "MP_BODY"
            self.pep_thresh_line.header.stamp = self.aep_line.header.stamp = self.pep_init_thresh_line.header.stamp = rospy.Time.now()
            self.pep_thresh_line.ns = self.aep_line.ns = self.pep_init_thresh_line.ns = self.name + "_points_and_lines"
            self.pep_thresh_line.action = self.aep_line.action = self.pep_init_thresh_line.action = Marker.ADD
            self.pep_thresh_line.pose.orientation.w = self.aep_line.pose.orientation.w = self.pep_init_thresh_line.pose.orientation.w = 1.0
            self.pep_init_thresh_line.id = 4
            self.aep_line.id = 5 + RSTATIC.leg_names.index(self.name)
            self.pep_thresh_line.id = 11 + RSTATIC.leg_names.index(self.name)
            if self.use_step_length:
                self.pep_thresh_line.type = Marker.CYLINDER
                self.pep_thresh_line.scale.z = 0.005
                self.pep_thresh_line.color.a = 0.5
            else:
                self.pep_thresh_line.type = Marker.LINE_LIST
                self.pep_thresh_line.scale.x = 0.0025
                self.pep_thresh_line.color.a = 1.0
            self.aep_line.type = self.pep_init_thresh_line.type = Marker.LINE_LIST
            self.aep_line.scale.x = self.pep_init_thresh_line.scale.x = 0.0025
            self.pep_init_thresh_line.color.g = 1.0
            self.aep_line.color.b = 1.0
            self.pep_thresh_line.color.r = 1.0
            self.aep_line.color.a = self.pep_init_thresh_line.color.a = 1.0
            self.pep_init_thresh_line.points.append(Point(self.pep_thresh, self.movement_dir * 0.20, -0.1))
            self.pep_init_thresh_line.points.append(Point(self.pep_thresh, self.movement_dir * 0.35, -0.1))
            point1 = Point(self.aep_thresh, self.movement_dir * 0.20, -0.1)
            point2 = Point(self.aep_thresh, self.movement_dir * 0.35, -0.1)
            self.aep_line.points.append(point1)
            self.aep_line.points.append(point2)
            # self.pub_pep_threshold()

    def pub_default_pep_threshold(self):
        # while not rospy.is_shutdown():
        # self.pep_thresh_line.points.clear()
        self.pep_init_thresh_line.points.append(
                Point(self.default_aep[0] - self.default_step_length, self.movement_dir * 0.20, -0.1))
        self.pep_init_thresh_line.points.append(
                Point(self.default_aep[0] - self.default_step_length, self.movement_dir * 0.35, -0.1))
        # point1 = Point(self.pep_thresh, self.movement_dir * 0.20, -0.1)
        # point2 = Point(self.pep_thresh, self.movement_dir * 0.35, -0.1)
        # point1 = Point(self.default_aep[0] - self.step_length, self.movement_dir * 0.20, -0.1)
        # point2 = Point(self.default_aep[0] - self.step_length, self.movement_dir * 0.35, -0.1)
        # self.pep_thresh_line.points.append(point1)
        # self.pep_thresh_line.points.append(point2)

        # for i in range(0, 3):
        # if rospy.is_shutdown():
        # break
        # self.visualization_pub.publish(self.aep_line)
        self.visualization_pub.publish(self.pep_init_thresh_line)
        # self.visualization_pub.publish(self.pep_thresh_line)
        # self.viz_pub_rate.sleep()

    def pub_pep_threshold(self):
        if self.use_step_length:
            self.pep_thresh_line.scale.x = self.pep_thresh_line.scale.y = 2 * self.step_length
            self.pep_thresh_line.pose.position.x = self.default_aep[0]
            self.pep_thresh_line.pose.position.y = self.default_aep[1]
            self.pep_thresh_line.pose.position.z = self.default_aep[2]
        else:
            self.pep_thresh_line.points.clear()
            point1 = Point(self.pep_thresh, self.movement_dir * 0.20, -0.1)
            point2 = Point(self.pep_thresh, self.movement_dir * 0.35, -0.1)
            # point1 = Point(self.default_aep[0] - self.step_length, self.movement_dir * 0.20, -0.1)
            # point2 = Point(self.default_aep[0] - self.step_length, self.movement_dir * 0.35, -0.1)
            self.pep_thresh_line.points.append(point1)
            self.pep_thresh_line.points.append(point2)

        self.visualization_pub.publish(self.aep_line)
        self.visualization_pub.publish(self.pep_init_thresh_line)
        self.visualization_pub.publish(self.pep_thresh_line)

    def is_ready(self):
        return self.alpha is not None and self.beta is not None and self.gamma is not None

    def c1_callback(self, data):
        # rospy.loginfo(self.name + ": got c1 data = " + str(data))
        self.alpha = data.process_value
        self.alpha_target = data.set_point
        self.alpha_command = data.command
        self.alpha_reached = -0.005 < data.error < 0.005

    def thigh_callback(self, data):
        # rospy.loginfo(self.name + ": got thigh data = " + str(data))
        self.beta = data.process_value
        self.beta_target = data.set_point
        self.beta_command = data.command
        self.beta_reached = -0.05 < data.error < 0.05

    def tibia_callback(self, data):
        # rospy.loginfo(self.name + ": got tibia data = " + str(data))
        self.gamma = data.process_value
        self.gamma_target = data.set_point
        self.gamma_command = data.command
        self.gamma_reached = -0.05 < data.error < 0.05

    def ee_position(self):
        self.update_ee_position()
        return self.ee_pos

    def update_ee_position(self):
        self.ee_pos = self.compute_forward_kinematics()

    def leg_center_of_mass(self):
        self.update_com_position()
        return self._center_of_mass

    def update_com_position(self):
        self._center_of_mass = self.compute_com()

    def leg_current_power(self):
        return abs(self.alpha_command) + abs(self.beta_command) + abs(self.gamma_command)

    ##
    #   Estimate ground ground_contact:
    #   Predict current leg position (using fw kinematics) and
    #   simply decide if the leg should touch ground
    #   (very stable, but works only on flat terrain).
    def predicted_ground_contact(self):
        if self.ee_position()[2] < (RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name) // 2][
                                        2] * RSTATIC.predicted_ground_contact_height_factor):
            self.current_stance_start = self.ee_position()
            return True
        else:
            return False

    def shift_pep_ipsilateral(self, distance):
        self.pep_shift_ipsilateral = distance
        if self.use_step_length:
            self.shift_step_length()
        else:
            self.shift_pep()

    def shift_pep_ipsilateral_from_front(self, distance):
        self.pep_shift_ipsilateral_front = distance
        if self.use_step_length:
            self.shift_step_length()
        else:
            self.shift_pep()

    def shift_pep_contralateral(self, distance):
        self.pep_shift_contralateral = distance
        if self.use_step_length:
            self.shift_step_length()
        else:
            self.shift_pep()

    def shift_pep(self):
        if self.use_step_length:
            rospy.logerr("calling x-pep-thresh function, but shift_step_length is used! Return without action")
            return
        # for backwards walking change minus to plus everywhere
        pep_thresh = RSTATIC.initial_pep[RSTATIC.leg_names.index(self.name) // 2][0].copy() + \
                     self.pep_shift_ipsilateral + self.pep_shift_ipsilateral_front + self.pep_shift_contralateral
        if pep_thresh > self.aep_thresh - (RSTATIC.default_stance_distance / 6):
            self.pep_thresh = self.aep_thresh - (RSTATIC.default_stance_distance / 6)
        else:
            self.pep_thresh = pep_thresh

    def shift_step_length(self):
        if not self.use_step_length:
            rospy.logerr("calling shift_step_length function, but x-pep-thresh is used! Return without action")
            return
        step_length = self.default_step_length - self.pep_shift_ipsilateral - self.pep_shift_ipsilateral_front - self.pep_shift_contralateral
        if step_length < (self.default_step_length / 6):
            self.step_length = self.default_step_length / 6  # 0.01
        else:
            self.step_length = step_length

    def set_default_step_length(self, length):
        if not self.use_step_length:
            rospy.logerr("calling shift_step_length function, but x-pep-thresh is used! Return without action")
            return
        self.default_step_length = length
        self.shift_step_length()

    def shift_default_aep(self, aep):
        self.default_aep = aep

    ##
    #   Estimate if end of stance is reached:
    #   check current end effector position against current pep threshold and decide if the stance can end.
    def reached_pep(self):
        if self.use_step_length:
            rospy.logerr("calling x-pep-thresh function, but shift_step_length is used! Return without action")
            return
        # TODO find min pep_thresh for warning in case the leg has to move to far back.
        # if self.pep_thresh == self.min_pep:
        #     rospy.logerr("go to swing because end of motion range is reached. This should usually not happen!")
        return self.ee_position()[0] < self.pep_thresh
        # for backwards walking:
        # return self.ee_position()[0] > self.pep_thresh

    ##
    #   Estimate if end of stance is reached:
    #   calculate current step length and decide based on threshold if the stance can end.
    def reached_step_length(self):
        if not self.use_step_length:
            rospy.logerr("calling shift_step_length function, but x-pep-thresh is used! Return without action")
            return
        # TODO find min pep_thresh for warning in case the leg has to move to far away.
        # if self.pep_thresh == self.min_pep:
        #     rospy.logerr("go to swing because end of motion range is reached. This should usually not happen!")
        # step_length = numpy.linalg.norm(self.default_aep - self.ee_position())
        step_length = numpy.linalg.norm(self.current_stance_start - self.ee_position())

        # if step_length > self.step_length:
        #     if self.name == "lr": # or self.name == "lm" or self.name == "lr":
        #         rospy.logwarn(self.name + ": current step_length {} > pep_step_length {}? then switch to swing".format(
        #                 step_length, self.step_length))
        #     return True
        # else:
        #     return False
        return step_length > self.step_length

    def check_joint_ranges(self, angles):
        return angles[0] >= RSTATIC.joint_angle_limits[0][0] or angles[0] <= RSTATIC.joint_angle_limits[0][
            1] or angles[1] >= RSTATIC.joint_angle_limits[1][0] or angles[1] <= \
               RSTATIC.joint_angle_limits[1][1] or angles[2] >= RSTATIC.joint_angle_limits[2][0] or angles[
                   2] <= RSTATIC.joint_angle_limits[2][1]

    def compute_com(self, angles=None):
        if angles is None:
            alpha = self.alpha
            beta = self.beta
            gamma = self.gamma
        else:
            alpha = angles[0]
            beta = angles[1]
            gamma = angles[2]

        center_of_mass_of_coxa = self.c1_rotation(alpha, numpy.append(self._segment_centers_of_mass[0], 1))
        center_of_mass_of_femur = self.c1_rotation(alpha,
                self.c1_thigh_transformation(beta, numpy.append(self._segment_centers_of_mass[1], 1)))
        center_of_mass_of_tibia = self.c1_rotation(alpha, self.c1_thigh_transformation(beta,
                self.thigh_tibia_transformation(gamma, numpy.append(self._segment_centers_of_mass[2], 1))))
        center_of_mass = (center_of_mass_of_coxa * self._segment_masses[0] + center_of_mass_of_femur *
                          self._segment_masses[1] + center_of_mass_of_tibia * self._segment_masses[2]) / \
                         sum(self._segment_masses)
        return (self.apply_c1_static_transform(center_of_mass))[0:3]

    # ee position in body frame
    def compute_forward_kinematics(self, angles=None):
        return self.apply_c1_static_transform(self.compute_forward_kinematics_c1(angles))[0:3]

    def compute_forward_kinematics_c1(self, angles=None):
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

        temp_tarsus_position = self.c1_rotation(alpha, self.c1_thigh_transformation(beta,
                self.thigh_tibia_transformation(gamma, self.tibia_ee_transformation())))

        # calculate shoulder angle as angle of vector from c1 pos to ee pos in body frame
        x_pos = -temp_tarsus_position[1]
        y_pos = temp_tarsus_position[2]
        alpha_check = -atan2(y_pos, x_pos)
        if abs(alpha_check - alpha) >= 0.01:
            raise Exception(
                    'The provided angles for ' + self.name + '(' + str(angles[0]) + ', ' + str(angles[1]) + ', ' + str(
                            angles[2]) + ') are not valid for the forward/inverse kinematics.')
        return temp_tarsus_position[0:3]

    def c1_rotation(self, alpha, point=numpy.array([0, 0, 0, 1])):
        cos_alpha = cos(alpha)
        sin_alpha = sin(alpha)
        cos90 = cos(radians(0))
        sin90 = sin(radians(0))
        trans = numpy.array([(cos90, sin_alpha * sin90, cos_alpha * sin90, 0),
                             (0, cos_alpha, 0 - sin_alpha, 0),
                             (0 - sin90, sin_alpha * cos90, cos_alpha * cos90, 0),
                             (0, 0, 0, 1)])
        return trans.dot(point)

    def leg_c1_transformation(self, alpha, point=numpy.array([0, 0, 0, 1])):
        cos_alpha = cos(alpha + radians(180))
        sin_alpha = sin(alpha + radians(180))
        cos90 = cos(radians(-90))
        sin90 = sin(radians(-90))
        trans = numpy.array([(cos90, sin_alpha * sin90, cos_alpha * sin90, 0),
                             (0, cos_alpha, 0 - sin_alpha, 0),
                             (0 - sin90, sin_alpha * cos90, cos_alpha * cos90, 0),
                             (0, 0, 0, 1)])
        return trans.dot(point)

    def c1_thigh_transformation(self, beta, point=numpy.array([0, 0, 0, 1])):
        cos_alpha = cos(beta)
        sin_alpha = sin(beta)
        cos90 = cos(radians(90))
        sin90 = sin(radians(90))
        trans = numpy.array([(cos90, sin_alpha * sin90, cos_alpha * sin90, 0),
                             (0, cos_alpha, 0 - sin_alpha, -0.054),
                             (0 - sin90, sin_alpha * cos90, cos_alpha * cos90, 0),
                             (0, 0, 0, 1)])
        return trans.dot(point)

    def thigh_tibia_transformation(self, gamma, point=numpy.array([0, 0, 0, 1])):
        cos_alpha = cos(gamma)
        sin_alpha = sin(gamma)
        cos90 = cos(radians(180))
        sin90 = sin(radians(180))
        trans = numpy.array([(cos90, sin_alpha * sin90, cos_alpha * sin90, 0),
                             (0, cos_alpha, 0 - sin_alpha, -0.0645),
                             (0 - sin90, sin_alpha * cos90, cos_alpha * cos90, -0.0145),
                             (0, 0, 0, 1)])
        return trans.dot(point)

    def tibia_ee_transformation(self, point=numpy.array([0, 0, 0, 1])):
        trans = numpy.array([(1, 0, 0, 0),
                             (0, 1, 0, -0.16),
                             (0, 0, 1, 0.02),
                             (0, 0, 0, 1)])
        return trans.dot(point)

    def apply_c1_static_transform(self, point=None):
        if point is None:
            point = [0, 0, 0, 1]
        if len(point) == 3:
            point = numpy.append(point, [1])
        return numpy.array(numpy.dot(self._c1_static_transform, point))[0:3]

    #  Calculation of inverse kinematics for a leg:
    #   Given a position in 3D space a joint configuration is calculated.
    #   When no position is provided the current position of the current leg is used
    #   to calculate the complementing angles.
    #   @param p point in body coordinate system
    def compute_inverse_kinematics(self, p=None):
        if isinstance(p, (type(None))):
            p = self.ee_position()
        if len(p) == 3:
            p = numpy.append(p, [1])
        # p_temp = copy.copy(p)
        # c1_pos = self.apply_c1_static_transform()

        # alpha_angle: float = -atan2(p[1], (p[0]))
        # switched x and y coordination because the leg points in the direction of the y axis of the MP_BODY frame:
        c1_static_rotation_inverse = numpy.array([
            [self._c1_static_transform[0][0], self._c1_static_transform[1][0], self._c1_static_transform[2][0]],
            [self._c1_static_transform[0][1], self._c1_static_transform[1][1], self._c1_static_transform[2][1]],
            [self._c1_static_transform[0][2], self._c1_static_transform[1][2], self._c1_static_transform[2][2]]])
        translation_inverse = numpy.array(numpy.dot(-c1_static_rotation_inverse,
                [self._c1_static_transform[0][3], self._c1_static_transform[1][3], self._c1_static_transform[2][3]]))
        c1_static_inverse = numpy.array([
            [c1_static_rotation_inverse[0][0], c1_static_rotation_inverse[0][1], c1_static_rotation_inverse[0][2],
             translation_inverse[0]],
            [c1_static_rotation_inverse[1][0], c1_static_rotation_inverse[1][1], c1_static_rotation_inverse[1][2],
             translation_inverse[1]],
            [c1_static_rotation_inverse[2][0], c1_static_rotation_inverse[2][1], c1_static_rotation_inverse[2][2],
             translation_inverse[2]],
            [0, 0, 0, 1]])
        p_c1 = numpy.array(numpy.dot(c1_static_inverse, p))
        alpha_angle = -atan2(p_c1[2], -p_c1[1])

        beta_pos = self.c1_rotation(alpha_angle, self.c1_thigh_transformation(0))
        beta_to_ee = numpy.linalg.norm(p[0:3] - self.apply_c1_static_transform(beta_pos))

        gamma_flipped = False

        try:
            cos_gamma = (pow(self._segment_lengths[2], 2) + pow(self._segment_lengths[1], 2) - pow(beta_to_ee, 2)) / (
                    2 * self._segment_lengths[1] * self._segment_lengths[2])
            # Avoid running in numerical rounding error
            if cos_gamma < -1:
                gamma_inner = pi
            else:
                gamma_inner = (acos(cos_gamma))
            gamma_angle = gamma_inner - pi - self.tibia_z_angle
            if RSTATIC.joint_angle_limits[2][0] >= gamma_angle and p[2] > 0:  # if gamma angle not in range
                rospy.logerr(self.name + " gamma limit = " + str(
                        RSTATIC.joint_angle_limits[2][0]) + " > gamma = " + str(gamma_angle))
                gamma_angle = pi - gamma_inner - self.tibia_z_angle
                gamma_flipped = True
                rospy.logerr(self.name + " flip gamma_angle. gamma new = " + str(gamma_angle))
        except ValueError:
            raise ValueError(self.name + ': gamma: The provided position (' + str(p[0]) + ', ' + str(p[1]) + ', ' +
                             str(p[2]) + ') is not valid for the given geometry')

        try:
            cos_beta_inner = (pow(self._segment_lengths[1], 2) + pow(beta_to_ee, 2) -
                              pow(self._segment_lengths[2], 2)) / (2 * self._segment_lengths[1] * beta_to_ee)
            # Avoid running in numerical rounding error
            if cos_beta_inner > 1:
                h1 = 0
            else:
                h1 = (acos(cos_beta_inner))

            vector_c1_ee = numpy.linalg.norm(p[0:3] - self.apply_c1_static_transform())
            cos_beta = (pow(beta_to_ee, 2) + pow(self._segment_lengths[0], 2) - pow(vector_c1_ee, 2)) / (
                    2 * beta_to_ee * self._segment_lengths[0])
            # Avoid running in numerical rounding error
            if cos_beta < -1.:
                h2 = pi
            else:
                h2 = (acos(cos_beta))
        except ValueError:
            raise ValueError(self.name + ': beta: The provided position (' + str(p[0]) + ', ' + str(p[1]) + ', ' +
                             str(p[2]) + ') is not valid for the given geometry.')

        beta_angle = pi - (h1 + h2 + self.thigh_tibia_angle)
        if RSTATIC.joint_angle_limits[1][0] >= beta_angle:
            rospy.logerr(self.name + " beta limit = " + str(
                    RSTATIC.joint_angle_limits[1][0]) + " > beta = " + str(beta_angle))
            if not gamma_flipped:
                rospy.logerr(self.name + ": gamma not flipped ")
            # if p[2] >= 0:
            beta_angle = h1 + h2 - pi - self.thigh_tibia_angle
            rospy.logerr(self.name + " flip beta_angle. beta new = " + str(beta_angle))

        return numpy.array([alpha_angle, beta_angle, gamma_angle])

    def get_current_angles(self):
        if self.alpha is None or self.beta is None or self.gamma is None:
            return None
        return [self.alpha, self.beta, self.gamma]

    def get_current_targets(self):
        if self.alpha_target is None or self.beta_target is None or self.gamma_target is None:
            return None
        return [self.alpha_target, self.beta_target, self.gamma_target]

    def is_target_reached(self):
        if self.alpha_target is None or self.beta_target is None or self.gamma_target is None:
            return None
        return self.alpha_reached and self.beta_reached and self.gamma_reached

    def is_target_set(self):
        return self.alpha_target == self.alpha_set_point and self.beta_target == self.beta_set_point and self.gamma_target == self.gamma_set_point

    def set_joint_point(self, next_angles):
        # rospy.loginfo("set command " + self.name + ". angles = " + str(next_angles) + " current angles = " +
        #               str(self.get_current_angles()))
        if not self.check_joint_ranges(next_angles):
            rospy.logerr("provided angles " + str(next_angles) + " are not valid for the joint ranges. COMMAND NOT SET")
        else:
            # rospy.loginfo(self.name + ": set angles " + str(next_angles))
            self._alpha_pub.publish(next_angles[0])
            self._gamma_pub.publish(next_angles[2])
            self._beta_pub.publish(next_angles[1])

    def set_joint_point_and_target(self, next_angles):
        # rospy.loginfo("set command " + self.name + ". angles = " + str(next_angles) + " current angles = " + str(
        #                self.get_current_angles()))
        if not self.check_joint_ranges(next_angles):
            rospy.logerr("provided angles " + str(next_angles) + " are not valid for the joint ranges. COMMAND NOT SET")
        else:
            # rospy.loginfo(self.name + ": set angles " + str(next_angles))
            self._alpha_pub.publish(next_angles[0])
            self._beta_pub.publish(next_angles[1])
            self._gamma_pub.publish(next_angles[2])
            self.alpha_set_point = next_angles[0]
            self.beta_set_point = next_angles[1]
            self.gamma_set_point = next_angles[2]
