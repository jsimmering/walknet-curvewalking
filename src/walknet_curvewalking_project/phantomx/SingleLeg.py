import copy
from math import sin, cos, atan2, pow, pi, acos, radians

import numpy
import rospy
import tf.transformations as transformations
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC


class SingleLeg:

    def __init__(self, name, segment_lengths, tf_listener, movement_dir):
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
        self.c1_static_transform = RSTATIC.body_c1_tf[RSTATIC.leg_names.index(self.name)].copy()

        self.alpha_target = None
        self.beta_target = None
        self.gamma_target = None

        self.alpha_reached = True
        self.beta_reached = True
        self.gamma_reached = True

        self.segment_lengths = segment_lengths
        self.movement_dir = movement_dir
        # self.rotation_dir = rotation_dir

        self.ee_pos = None

        self.visualization_pub = rospy.Publisher('/kinematics', Marker, queue_size=1)
        self.c1_ee_points = Marker()
        self.global_ee_points = Marker()
        self.c1_leg_vec_lines = Marker()
        self.global_leg_vec_lines = Marker()
        self.set_up_visualization()

    def set_up_visualization(self):
        self.global_ee_points.header.frame_id = self.global_leg_vec_lines.header.frame_id = "MP_BODY"
        self.c1_ee_points.header.frame_id = self.c1_leg_vec_lines.header.frame_id = "c1_" + self.name
        # self.c1_ee_points.header.frame_id = self.c1_leg_vec_lines.header.frame_id = "MP_BODY"
        self.c1_ee_points.header.stamp = self.global_ee_points.header.stamp = self.global_leg_vec_lines.header.stamp = \
            self.c1_leg_vec_lines.header.stamp = rospy.Time.now()
        self.c1_ee_points.ns = self.global_ee_points.ns = self.global_leg_vec_lines.ns = self.c1_leg_vec_lines.ns = \
            "points_and_lines"
        self.c1_ee_points.action = self.global_ee_points.action = self.global_leg_vec_lines.action = \
            self.c1_leg_vec_lines.action = Marker.ADD
        self.c1_ee_points.pose.orientation.w = self.global_ee_points.pose.orientation.w = self.global_leg_vec_lines.pose.orientation.w = \
            self.c1_leg_vec_lines.pose.orientation.w = 1.0

        self.global_ee_points.id = 0
        self.c1_ee_points.id = 1
        self.global_leg_vec_lines.id = 2
        self.c1_leg_vec_lines.id = 3

        self.global_ee_points.type = Marker.POINTS
        self.c1_ee_points.type = Marker.POINTS
        self.global_leg_vec_lines.type = Marker.LINE_LIST
        self.c1_leg_vec_lines.type = Marker.LINE_LIST

        self.global_ee_points.scale.x = 0.005
        self.global_ee_points.scale.y = 0.005
        self.c1_ee_points.scale.x = 0.005
        self.c1_ee_points.scale.y = 0.005

        self.global_leg_vec_lines.scale.x = 0.0025
        self.c1_leg_vec_lines.scale.x = 0.0025

        self.global_ee_points.color.r = 1.0
        self.global_ee_points.color.a = 1.0
        self.c1_ee_points.color.b = 1.0
        self.c1_ee_points.color.a = 1.0
        self.global_leg_vec_lines.color.a = 1.0
        self.c1_leg_vec_lines.color.a = 1.0
        self.global_leg_vec_lines.color.r = 1.0
        self.c1_leg_vec_lines.color.b = 1.0

    def pub_local(self):
        start_point = Point()
        start = [0, 0, 0]
        start_point.x = start[0]
        start_point.y = start[1]
        start_point.z = start[2]
        vecs = self.c1_rotation(-self.alpha, self.compute_forward_kinematics_c1())
        pos = Point()
        pos.x = start_point.x + vecs[0]
        pos.y = start_point.y + vecs[1]
        pos.z = start_point.z + vecs[2]
        self.c1_ee_points.points.append(start_point)
        self.c1_ee_points.points.append(pos)
        self.c1_leg_vec_lines.points.append(start_point)
        self.c1_leg_vec_lines.points.append(pos)

        rate = rospy.Rate(RSTATIC.controller_frequency)
        for i in range(0, 5):
            self.visualization_pub.publish(self.c1_ee_points)
            self.visualization_pub.publish(self.c1_leg_vec_lines)
            rate.sleep()

    def pub_global(self):
        start_point = Point()
        start = [0, 0, 0]
        start_point.x = start[0]
        start_point.y = start[1]
        start_point.z = start[2]
        self.global_ee_points.points.append(start_point)
        vecs = self.compute_forward_kinematics()
        pos = Point()
        pos.x = start_point.x + vecs[0]
        pos.y = start_point.y + vecs[1]
        pos.z = start_point.z + vecs[2]
        self.global_ee_points.points.append(pos)
        self.global_leg_vec_lines.points.append(start_point)
        self.global_leg_vec_lines.points.append(pos)

        rate = rospy.Rate(RSTATIC.controller_frequency)
        for i in range(0, 5):
            self.visualization_pub.publish(self.global_ee_points)
            self.visualization_pub.publish(self.global_leg_vec_lines)
            rate.sleep()

    def is_ready(self):
        if self.alpha is not None and self.beta is not None and self.gamma is not None:
            return True
        else:
            return False

    def c1_callback(self, data):
        self.alpha = data.process_value
        self.alpha_target = data.set_point
        if data.error < 0.1:
            # rospy.loginfo(self.name + ": alpha = " + str(self.alpha) + " alpha_target = " + str(
            #    self.alpha_target) + " alpha_reached = True")
            self.alpha_reached = True
        else:
            # rospy.loginfo(self.name + ": alpha = " + str(self.alpha) + " alpha_target = " + str(
            #    self.alpha_target) + " alpha_reached = False")
            self.alpha_reached = False

    def thigh_callback(self, data):
        self.beta = data.process_value
        self.beta_target = data.set_point
        if data.error < 0.1:
            # rospy.loginfo(self.name + ": beta = " + str(self.beta) + " beta_target = " + str(
            #    self.beta_target) + " beta_reached = True")
            self.beta_reached = True
        else:
            # rospy.loginfo(self.name + ": beta = " + str(self.beta) + " beta_target = " + str(
            #    self.beta_target) + " beta_reached = False")
            self.beta_reached = False

    def tibia_callback(self, data):
        self.gamma = data.process_value
        self.gamma_target = data.set_point
        if data.error < 0.1:
            # rospy.loginfo(self.name + ": gamma = " + str(self.gamma) + " gamma_target = " + str(
            #    self.gamma_target) + " gamma_reached = True")
            self.gamma_reached = True
        else:
            # rospy.loginfo(self.name + ": gamma = " + str(self.gamma) + " gamma_target = " + str(
            #    self.gamma_target) + " gamma_reached = False")
            self.gamma_reached = False

    def ee_position(self):
        self.update_ee_position()
        return self.ee_pos[0:3]

    def update_ee_position(self):
        self.ee_pos = self.compute_forward_kinematics()

    ##
    #   Estimate ground ground_contact:
    #   Predict current leg position (using fw kinematics) and
    #   simply decide if the leg should touch ground
    #   (very stable, but works only on flat terrain).
    def predictedGroundContact(self):
        if self.name == "lf" or self.name == "rf":
            if (self.ee_position()[2] < (RSTATIC.front_initial_aep[2] * RSTATIC.predicted_ground_contact_height_factor)) \
                    and abs(self.ee_position()[0] - RSTATIC.front_initial_aep[0]) < 0.025:
                rospy.loginfo("predict ground contact for front leg")
                return 1
        if self.name == "lm" or self.name == "rm":
            if (self.ee_position()[2] < (
                    RSTATIC.middle_initial_aep[2] * RSTATIC.predicted_ground_contact_height_factor)) \
                    and abs(self.ee_position()[0] - RSTATIC.middle_initial_aep[0]) < 0.025:
                rospy.loginfo("predict ground contact for middle leg")
                return 1
        if self.name == "lr" or self.name == "rr":
            if (self.ee_position()[2] < (RSTATIC.hind_initial_aep[2] * RSTATIC.predicted_ground_contact_height_factor)) \
                    and abs(self.ee_position()[0] - RSTATIC.hind_initial_aep[0]) < 0.025:
                rospy.loginfo("predict ground contact for rear leg")
                return 1

        return 0

    ##
    #   Estimate ground ground_contact:
    #   Predict current leg position (using fw kinematics) and
    #   simply decide if the leg should touch ground
    #   (very stable, but works only on flat terrain).
    def reached_pep(self):
        if self.name == "lf" or self.name == "rf":
            # if abs(self.ee_position()[0] - self.movement_dir * RSTATIC.front_initial_pep[0]) < 0.025:
            if abs(self.ee_position()[0] - RSTATIC.front_initial_pep[0]) < 0.025:
                rospy.loginfo("stop stance for front leg")
                return 1
        if self.name == "lm" or self.name == "rm":
            rospy.loginfo("abs(self.ee_position()[0] (" + str(self.ee_position()[0]) + ") - self.movement_dir (" + str(
                    self.movement_dir) + ") * RSTATIC.middle_initial_pep[0] (" + str(
                    RSTATIC.middle_initial_pep[0]) + ")) < 0.025")
            # if abs(self.ee_position()[0] - self.movement_dir * RSTATIC.middle_initial_pep[0]) < 0.025:
            if abs(self.ee_position()[0] - RSTATIC.middle_initial_pep[0]) < 0.025:
                rospy.loginfo("stop stance for middle leg")
                return 1
        if self.name == "lr" or self.name == "rr":
            # if abs(self.ee_position()[0] - self.movement_dir * RSTATIC.hind_initial_pep[0]) < 0.025:
            if abs(self.ee_position()[0] - RSTATIC.hind_initial_pep[0]) < 0.025:
                rospy.loginfo("stop stance for rear leg")
                return 1

        return 0

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
        return angles[0] >= RSTATIC.joint_angle_limits[0][0] or angles[0] <= RSTATIC.joint_angle_limits[0][
            1] or angles[1] >= RSTATIC.joint_angle_limits[1][0] or angles[1] <= \
               RSTATIC.joint_angle_limits[1][1] or angles[2] >= RSTATIC.joint_angle_limits[2][0] or angles[
                   2] <= RSTATIC.joint_angle_limits[2][1]

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
        return temp_tarsus_position

    def c1_rotation(self, alpha, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        # TODO ist alpha reversed?
        # alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        # left legs:
        cos_alpha = cos(alpha)
        sin_alpha = sin(alpha)
        # if self.movement_dir == -1:
        #    cos_alpha = cos(alpha + radians(180))
        #    sin_alpha = sin(alpha + radians(180))
        cos90 = cos(radians(0))
        sin90 = sin(radians(0))
        trans = numpy.array([(cos90, sin_alpha * sin90, cos_alpha * sin90, 0),
                             (0, cos_alpha, 0 - sin_alpha, 0),
                             (0 - sin90, sin_alpha * cos90, cos_alpha * cos90, 0),
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

    def c1_thigh_transformation(self, beta, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        # TODO ist alpha reversed?
        # alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        cos_alpha = cos(beta)
        sin_alpha = sin(beta)
        # TODO rotate by 90 degrees only for left legs?
        cos90 = cos(radians(90))
        sin90 = sin(radians(90))
        trans = numpy.array([(cos90, sin_alpha * sin90, cos_alpha * sin90, 0),
                             (0, cos_alpha, 0 - sin_alpha, -0.054),  # TODO - only for left legs?
                             (0 - sin90, sin_alpha * cos90, cos_alpha * cos90, 0),
                             (0, 0, 0, 1)])
        # print('trans: ', trans)
        return trans.dot(point)

    def thigh_tibia_transformation(self, gamma, point=numpy.array([0, 0, 0, 1])):
        # point=numpy.append(point,1)
        # TODO ist alpha reversed?
        # alpha *= -1  # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
        cos_alpha = cos(gamma)
        sin_alpha = sin(gamma)
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
        return numpy.array(numpy.dot(pos, self.c1_rotation(-self.alpha, point)))
        # return numpy.array(numpy.dot(pos, point))

    def apply_c1_static_transform(self, point=[0, 0, 0, 1]):
        return numpy.array(numpy.dot(self.c1_static_transform, point))

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
        if isinstance(p, (type(None))):
            p = self.ee_position()
        if len(p) == 3:
            p = numpy.append(p, [1])
        p_temp = copy.copy(p)

        # c1_pos = self.body_c1_transformation(self.alpha)
        c1_pos = self.apply_c1_static_transform()
        # alpha_angle: float = -atan2(p[1], (p[0]))
        # switched x and y coordination because the leg points in the direction of the y axis of the MP_BODY frame:
        c1_static_rotation_inverse = numpy.array([
            [self.c1_static_transform[0][0], self.c1_static_transform[1][0], self.c1_static_transform[2][0]],
            [self.c1_static_transform[0][1], self.c1_static_transform[1][1], self.c1_static_transform[2][1]],
            [self.c1_static_transform[0][2], self.c1_static_transform[1][2], self.c1_static_transform[2][2]]])
        translation_inverse = numpy.array(numpy.dot(-c1_static_rotation_inverse,
                [self.c1_static_transform[0][3], self.c1_static_transform[1][3], self.c1_static_transform[2][3]]))
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
        lct = numpy.linalg.norm(p[0:3] - self.apply_c1_static_transform(beta_pos)[0:3])

        default_gamma_pos = self.c1_rotation(alpha_angle,
                self.c1_thigh_transformation(0, self.thigh_tibia_transformation(0)))
        thigh_tibia_angle = -atan2(default_gamma_pos[0] - beta_pos[0], -default_gamma_pos[1] + beta_pos[1])  # 0.2211...
        tibia_z_angle = pi - atan2(0.02, -0.16)  # 0.12435499454676124
        try:
            cos_gamma = (pow(self.segment_lengths[2], 2) + pow(self.segment_lengths[1], 2) - pow(lct, 2)) / (
                    2 * self.segment_lengths[1] * self.segment_lengths[2])
            #rospy.loginfo("cos_gamma = " + str(cos_gamma))
            # Avoid running in numerical rounding error
            if cos_gamma < -1:
                gamma_inner = pi
                #rospy.loginfo("cos_gamma < -1 => gamma_inner = pi " + str(gamma_inner))
            else:
                gamma_inner = (acos(cos_gamma))
                #rospy.loginfo("cos_gamma >= -1 => gamma_inner = " + str(gamma_inner))
            gamma_angle = gamma_inner - pi - tibia_z_angle
            #rospy.loginfo("gamma_angle = gamma_inner (" + str(gamma_inner) + ") - pi (" + str(
            #        pi) + ") - tibia_z_angle (" + str(tibia_z_angle) + ") = " + str(gamma_angle))
            # if gamma angle not in range
            # if RSTATIC.joint_angle_limits[2][0] > gamma_angle:
            if p[2] > 0:
                gamma_angle = pi - gamma_inner - tibia_z_angle
                #rospy.loginfo("p[2] > 0")
                #rospy.loginfo("gamma_angle = pi (" + str(pi) + ") - gamma_inner (" + str(
                #        gamma_inner) + ") - tibia_z_angle (" + str(tibia_z_angle) + ") = " + str(gamma_angle))

            cos_beta_inner = (pow(self.segment_lengths[1], 2) + pow(lct, 2) - pow(self.segment_lengths[2], 2)) / (
                    2 * self.segment_lengths[1] * lct)
            #rospy.loginfo("cos_beta_inner = " + str(cos_beta_inner))
            # Avoid running in numerical rounding error
            if cos_beta_inner > 1:
                h1 = 0
                #rospy.loginfo("cos_beta_inner > 1 => h1 = 0 " + str(h1))
            else:
                h1 = (acos(cos_beta_inner))
                #rospy.loginfo("cos_beta_inner <= 1 => h1 = " + str(h1))

            # ee_angle = -atan2(p[2] - beta_pos[2], p[1] - beta_pos[1])
            vector_c1_ee = numpy.linalg.norm(p[0:3] - c1_pos[0:3])
            cos_beta = (pow(lct, 2) + pow(self.segment_lengths[0], 2) - pow(vector_c1_ee, 2)) / (
                    2 * lct * self.segment_lengths[0])
            #rospy.loginfo("cos_beta = " + str(cos_beta))
            # Avoid running in numerical rounding error
            if cos_beta < -1.:
                h2 = pi
                #rospy.loginfo("cos_beta_inner < -1 => h2 = pi " + str(h2))
            else:
                h2 = (acos(cos_beta))
                #rospy.loginfo("cos_beta_inner >= -1 => h2 = " + str(h2))
        except ValueError:
            raise ValueError('The provided position (' + str(p_temp[0]) + ', ' + str(p_temp[1]) + ', ' + str(
                    p_temp[2]) + ') is not valid for the given geometry for leg ' + self.name)
        # if RSTATIC.joint_angle_limits[1][0] > gamma_angle:
        if p[2] >= 0:
            beta_angle = h1 + h2 - pi - thigh_tibia_angle
            #rospy.loginfo("p[2] >= 0 => beta_angle = " + str(beta_angle))
        else:
            beta_angle = pi - (h1 + h2 + thigh_tibia_angle)
            #rospy.loginfo("p[2] < 0 => beta_angle = " + str(beta_angle))


        # if self.rotation_dir is True:
        #    gamma_angle *= -1
        # else:
        #    beta_angle *= -1
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
        # rospy.loginfo("alpha is reached = " + str(self.alpha_reached))
        # rospy.loginfo("beta is reached = " + str(self.beta_reached))
        # rospy.loginfo("gamma is reached = " + str(self.gamma_reached))
        if self.alpha_target is None or self.beta_target is None or self.gamma_target is None:
            return None
        return self.alpha_reached and self.beta_reached and self.gamma_reached

    def set_command(self, next_angles):
        # TODO check joint ranges
        rospy.loginfo(
                "set command " + self.name + ". angles = " + str(next_angles) + " current angles = " + str(
                        self.get_current_angles()))
        self.alpha_pub.publish(next_angles[0])
        self.beta_pub.publish(next_angles[1])
        self.gamma_pub.publish(next_angles[2])
