# Took code from:
# https://github.com/malteschilling/cognitiveWalker/blob/master/controller/reaCog/Movements/SwingMovementBezier.py
# modified for PhantomX Robot
import copy

import numpy
import rospy

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
import walknet_curvewalking_project.support.constants as CONST


# Function that computes a point that lies on a bezier curve
# The curve is defined by the parameters in points (knots and control points) and the order.
# The "parameter" defines the position of the point on the curve
# Also for multiple pieces, the curve starts at parameter=0 and ends at parameter=1!
# For parameters outside the interval [0,1], the curve will be extrapolated.
# points: knots and control points that define the curve
# parameter: position of the point on the curve
# order: order of the curve
def bezier(points, parameter, order=2):
    num_of_segments = (points.shape[0] - 1) / order  # number of pieces, the piecewise bezier curve consists of
    # rospy.loginfo("points.shape[0] = " + str(points.shape[0]) + " order = " + str(order) + " num_of_segments = " + str(num_of_segments))
    assert num_of_segments % 1 == 0

    segment_number = None  # the number of the currently relevant segment
    if 0 <= parameter <= 1:
        segment_number = numpy.floor(parameter * num_of_segments)
    elif parameter < 0:
        segment_number = 0
    elif 1 < parameter:
        segment_number = num_of_segments - 1
    # rospy.loginfo("number of currently relevant segment = " + str(segment_number))
    segment_number = int(segment_number)
    # rospy.loginfo("number of currently relevant segment = " + str(segment_number))

    relative_parameter = (parameter * num_of_segments) - segment_number  # this is the parameter for the
    # current segment. It is mapped to the interval [0,1]
    relevant_points = copy.copy(points[segment_number * order:(segment_number + 1) * order + 1, :])
    # #the bezier points of the current segment
    # print("bezier points of the current segment = " + str(relevant_points))
    # compute the point
    while relevant_points.shape[0] > 1:
        deltas = numpy.diff(relevant_points, n=1, axis=0)
        relevant_points = relevant_points[0:-1] + deltas * relative_parameter  # [0:-1] = [:-1] all elements except last
    # rospy.loginfo("return bezier point: " + str(relevant_points[0]))
    return relevant_points[0]  # [0, :]  # everything in row 0 (first row)


class TrajectoryGenerator:
    def __init__(self):
        self.last_distance = None
        self.last_target_parameter = None  # The bezier parameter of the last iteration
        self.last_target_position = None  # The position on the piecewise bezier curve corresponding to the
        # last_target_parameter
        self.accuracy = 1 / 100  # the accuracy of the target point with respect to the desired_distance
        self.norm_delta_parameter = None
        self.bezier_points = None  # The points that are used for the piecewise bezier
        self.order = 2  # the order to the bezier curve

        # self.frozen = True

    def reset(self):
        self.last_target_position = None
        self.last_target_parameter = None

    # Method to compute the next goal position along the curve that has a certain distance from the current position.
    # In a first attempt, the function tries to find a point on the curve by iteratively increasing/decreasing the
    # value of delta_parameter.
    # If this fails, binary search is used to find the next target point. In order to use this algorithm, first of all,
    # a point is needed that is located at less than desired_distance from the current_position. Additionally,
    # a point is needed that is further than desired_distance away from the current_position.
    def compute_next_target(self, desired_distance=None, current_position=None):
        # if desired_distance is not None and desired_distance < 0.15:
        # rospy.logerr("desired_distance = " + str(desired_distance)) # + ". To small, set to 0.15")
        # desired_distance = 0.15

        if self.last_target_parameter is None:
            # if the last target parameter is unknown, probably, this is the first iteration for a new trajectory.
            self.last_target_parameter = 0
            # the last target position is set to the first point of the bezier points
            # print("last_target_position = " + str(self.last_target_position))
            # print("bezier_points = " + str(self.bezier_points))
            self.last_target_position = self.bezier_points[0]  # [0, :]  # everything in row 0
            # The number of segments the piecewise bezier curve consists of:
            num_of_segments = (self.bezier_points.shape[0] - 1) / self.order
            # A good estimation of the delta_paramet is necessary in order to reduce the number of iterations
            self.norm_delta_parameter = ((1 / num_of_segments) * desired_distance) / numpy.linalg.norm(
                    numpy.diff(self.bezier_points[0:2, :]))  # [0:2, :]  array containing row 0 and 1 of array
            # rospy.loginfo("num_of_segments = " + str(num_of_segments))
            # rospy.loginfo("desired_distance = " + str(desired_distance))
            # rospy.loginfo("numpy.linalg.norm(numpy.diff(self.bezier_points[0:2, :])) = " + str(numpy.linalg.norm(
            #        numpy.diff(self.bezier_points[0:2, :]))))
            # rospy.loginfo("norm_delta_parameter = " + str(self.norm_delta_parameter))

        # if no current_position is given by parameter, the last_target_position is assumed
        if current_position is None:
            current_position = self.last_target_position
        else:
            delta_position = self.last_target_position - current_position
            # rospy.loginfo("delta_position (" + str(delta_position) + ") = self.last_target_position (" + str(
            #        self.last_target_position) + ") - current_position (" + str(current_position) + ")")
            delta_distance = numpy.linalg.norm(delta_position)
            # rospy.loginfo("current distance " + str(delta_distance) + " desired distance = " + str(desired_distance))
            if delta_distance > desired_distance:
                # delta_distance needs to be smaller than desired distance otherwise swing movement gets stuck sometimes.
                # if the current_position is more than desired_distance away from the last_target_position,
                # a velocity vector will be returned that points to the last target (its norm is the desired_distance).
                rospy.loginfo(
                        "---ELSE: current distance (" + str(delta_distance) + ") is more than desired_distance (" +
                        str(desired_distance) + ") away from the last_target_position,\nlast_target_pos = " +
                        str(self.last_target_position) + " current_pos = " + str(current_position))
                if RSTATIC.DEBUG:
                    rospy.loginfo("+++RETURN velocity vector pointing to the last_target_position vec = " +
                                  str(current_position + delta_position /
                                      numpy.linalg.norm(delta_position) * desired_distance))

                next_target = current_position + delta_position / numpy.linalg.norm(delta_position) * desired_distance
                return (next_target, self.last_target_parameter)
            del delta_position
        # Try to adapt the delta_parameter to the desired_distance

        delta_parameter = self.norm_delta_parameter * desired_distance
        parameter_position_list = [(self.last_target_parameter, self.last_target_position)]
        # slightly_further_parameter = None
        # slightly_closer_parameter = None
        # slightly_further_position = None
        for _ in range(6):
            # do this six times - this should be enough, usually, it takes two or three iterations to reach
            # an accuracy of 0.01.
            # This is just a temporary target position that is used to estimate the amount by which the
            # delta_parameter should be changed.
            test_parameter = self.last_target_parameter + delta_parameter
            test_target_position = bezier(self.bezier_points, test_parameter)
            parameter_position_list.append((test_parameter, test_target_position))
            # rospy.loginfo(
            #        "parameter list append temp_parameter = " + str(test_parameter) + " temp_target_position = " + str(
            #                test_target_position) + " delta_parameter = " + str(delta_parameter))

            # If a delta_parameter was found that produces a point which has a distance to the current_position that
            # is close enough to the desired_distance, stop this algorithm.
            current_distance = numpy.linalg.norm(test_target_position - current_position)
            if abs(current_distance - desired_distance) / desired_distance < self.accuracy:
                self.last_target_parameter += delta_parameter
                self.last_target_position = current_position + (
                        test_target_position - current_position) / numpy.linalg.norm(
                        test_target_position - current_position) * desired_distance
                if RSTATIC.DEBUG:
                    rospy.loginfo(
                            "+++RETURN delta_parameter found that produces a point = " + str(self.last_target_position))

                return (self.last_target_position, test_parameter)

            # adapt the delta_parameter for the next iteration
            delta_parameter = delta_parameter / (
                    numpy.linalg.norm(test_target_position - current_position) / desired_distance)
            if (delta_parameter / (
                    numpy.linalg.norm(test_target_position - current_position) / desired_distance)) < 0.0001:
                rospy.logerr("delta_parameter = " + str(delta_parameter) + " to small! set to 0.0001")
                delta_parameter = 0.0001
                if RSTATIC.DEBUG:
                    rospy.loginfo("delta_parameter (" + str(delta_parameter) + ") = \ndelta_parameter (" +
                                  str(delta_parameter) + ") / \n(numpy.linalg.norm(test_target_position (" +
                                  str(test_target_position) + ") - current_position (" + str(current_position) +
                                  ")) (" + str(numpy.linalg.norm(test_target_position - current_position)) +
                                  ") \n/ desired_distance (" + str(desired_distance) + "))")

            # TODO can be done only once after for??
            # variables containing a parameter, position and the corresponding distance that is slightly smaller than
            # the desired_distance
            slightly_closer_parameter = self.last_target_parameter

            # variables containing a parameter, position and the corresponding distance that is slightly longer than
            # the desired_distance
            slightly_further_parameter = float('inf')

            max_parameter = float('-inf')

            # search in the parameter_position_list for a parameter that leads to a distance slightly less than
            # desired_distance and a parameter that leads to a slightly longer distance
            for temp_parameter, temp_target_position in parameter_position_list:
                # rospy.loginfo("parameter list temp_parameter = " + str(temp_parameter) + " temp_target_position = " + str(temp_target_position))
                temp_distance = numpy.linalg.norm(temp_target_position - current_position)

                if temp_distance <= desired_distance and slightly_closer_parameter < temp_parameter < \
                        slightly_further_parameter:  # The equal case will never happen since this would have led the
                    # previous attempt to delete the parameter_position_list and thus would have prohibited the
                    # execution of this part.
                    slightly_closer_parameter = temp_parameter
                elif temp_distance > desired_distance and slightly_closer_parameter < temp_parameter < \
                        slightly_further_parameter:
                    slightly_further_parameter = temp_parameter

                if temp_parameter > max_parameter:
                    max_parameter = temp_parameter

        # if there was no parameter in the list that corresponds to a point that is further than desired_distance
        # away from the current_position,
        # such a point will be produced by doubling the delta_parameter iteratively until it is big enough (or the
        # maximum number of iterations is reached.)
        if slightly_further_parameter == float('inf'):
            # rospy.loginfo("last_target_parameter = " + str(self.last_target_parameter))
            slightly_further_parameter = slightly_closer_parameter
            for _ in range(20):
                slightly_further_parameter = slightly_further_parameter + (
                        slightly_further_parameter - self.last_target_parameter) * 2
                slightly_further_position = bezier(self.bezier_points, slightly_further_parameter)
                slightly_further_distance = numpy.linalg.norm(slightly_further_position - self.last_target_position)
                if slightly_further_distance > desired_distance:
                    break
            if slightly_further_position is None:
                raise Exception(
                        'The bezier curve behaves extremely strange! No appropriate point could be reached. Just for '
                        'information some data: bezier points: ' + str(self.bezier_points) + ', parameter: ' + str(
                                self.last_target_parameter))

        # use binary search to find a point with sufficient accuracy
        # initialize the test_parameter to be between the slightly closer and the slightly further parameters
        test_parameter = (slightly_closer_parameter + slightly_further_parameter) / 2
        # rospy.loginfo("test_param (" + str(test_parameter) + ") = (slightly_closer_parameter (" + str(
        #        slightly_closer_parameter) + ") + slightly_further_parameter (" + str(
        #        slightly_further_parameter) + ")) / 2")
        # test_target_position = None
        for _ in range(20):
            test_target_position = bezier(self.bezier_points, test_parameter)
            test_distance = numpy.linalg.norm(test_target_position - current_position)
            if abs(test_distance - desired_distance) / desired_distance < self.accuracy:
                break

            if test_distance < desired_distance:
                slightly_closer_parameter = test_parameter
            elif test_distance > desired_distance:
                slightly_further_parameter = test_parameter

            test_parameter = (slightly_further_parameter + slightly_closer_parameter) / 2

        # rospy.loginfo("abs(test_distance - desired_distance) / desired_distance (" +
        #   str(abs(test_distance - desired_distance) / desired_distance) + ") < self.accuracy (" +
        #   str(self.accuracy) + ")")
        self.norm_delta_parameter = (test_parameter - self.last_target_parameter) / desired_distance
        self.last_target_parameter = test_parameter
        self.last_target_position = current_position + (test_target_position - current_position) / numpy.linalg.norm(
                test_target_position - current_position) * desired_distance
        if RSTATIC.DEBUG:
            rospy.loginfo("+++RETURN binary search = " + str(self.last_target_position))
        return (self.last_target_position, self.last_target_parameter)


class SwingMovementBezier:
    def __init__(self, leg=None):
        self.leg = leg
        self.trajectory_generator = TrajectoryGenerator()
        self.last_activation = 0
        self.swing_velocity = CONST.DEFAULT_SWING_VELOCITY

        self.swing_start_point = None  # the point where the swing phase starts
        self.swing_target_point = None  # the point where the swing phase should end
        self.apex_point_ratio = 0.5  # at which position of the interval between the start and the end point the
        # middle point should be placed
        self.apex_point_offset = CONST.DEFAULT_APEX_POINT_OFFSET  # the offset that is added to the middle point that was
        # computed on the connecting line between start and end point using the apex_point_ratio concept.

        self.collision_point = None

        self.evasion_distance = 0.06
        self.retraction_distance = 0.10
        self.max_evasion_distance = 0.25

        self.reacht_peak = False
        # self.frozen = True

    def notify_of_collision(self, weight=1):
        if weight >= 1:
            self.collision_point = self.leg.ee_position()
            rospy.loginfo("in notify_of_collision: calculate bezier points based on positions!")
            bezier_points = self.compute_bezier_points()
            self.trajectory_generator.reset()
            self.trajectory_generator.bezier_points = bezier_points

    def compute_bezier_points(self):  # ForNormalSwingMovement(self):
        # rospy.loginfo("target = " + str(self.swing_target_point) + " start = " + str(self.swing_start_point))
        start_to_end_vector = self.swing_target_point - self.swing_start_point
        start_to_end_distance = numpy.linalg.norm(start_to_end_vector)
        start_to_end_direction = start_to_end_vector / start_to_end_distance
        apex_point_distance = numpy.linalg.norm(self.apex_point_offset)
        apex_point = self.swing_start_point + self.apex_point_ratio * start_to_end_vector + self.apex_point_offset
        apex_point_direction = self.apex_point_offset / apex_point_distance
        if self.collision_point is None:  # if no collision happened
            control_point_1 = apex_point - 0.5 * self.apex_point_ratio * start_to_end_vector
            control_point_2 = apex_point + 0.5 * (1 - self.apex_point_ratio) * start_to_end_vector
            return numpy.array(
                    [self.swing_start_point, control_point_1, apex_point, control_point_2, self.swing_target_point])
        else:  # in case of a collision
            collision_ratio = numpy.dot((self.collision_point - self.swing_start_point),
                    start_to_end_direction) / start_to_end_distance  # where during the swing
            # phase the collision happened
            start_to_end_vector_to_collision_point_distance = numpy.linalg.norm(
                    (self.collision_point - self.swing_start_point) - collision_ratio * start_to_end_vector)
            control_point_1 = self.collision_point - self.retraction_distance * start_to_end_direction
            evasion_point = self.collision_point + self.evasion_distance * apex_point_direction
            if collision_ratio < self.apex_point_ratio:  # the collision happened before the leg reached the apex
                if start_to_end_vector_to_collision_point_distance + self.evasion_distance < apex_point_distance:  #
                    # the evasion movement will be below the apex point
                    evasion_to_apex_vector = apex_point - evasion_point
                    control_point_3 = apex_point - numpy.dot(0.5 * evasion_to_apex_vector,
                            start_to_end_direction) * start_to_end_direction
                    evasion_to_control_point_3_vector = control_point_3 - evasion_point
                    evasion_to_control_point_3_distance = numpy.linalg.norm(evasion_to_control_point_3_vector)
                    evasion_to_control_point_3_direction = evasion_to_control_point_3_vector / \
                                                           evasion_to_control_point_3_distance
                    evasion_to_control_point_3_distance = self.retraction_distance / numpy.dot(
                            evasion_to_control_point_3_direction, start_to_end_direction)
                    control_point_2 = evasion_point - evasion_to_control_point_3_distance * \
                                      evasion_to_control_point_3_direction
                    retraction_point = 0.5 * (control_point_2 - control_point_1) + control_point_1
                    control_point_4 = apex_point + 0.5 * self.apex_point_ratio * start_to_end_vector
                    return numpy.array(
                            [self.collision_point, control_point_1, retraction_point, control_point_2, evasion_point,
                             control_point_3, apex_point, control_point_4, self.swing_target_point])
                else:  # the evasion movement will be above the standard apex point
                    if start_to_end_vector_to_collision_point_distance + self.evasion_distance > \
                            self.max_evasion_distance:
                        evasion_point = self.swing_start_point + collision_ratio * start_to_end_vector + \
                                        self.max_evasion_distance * apex_point_direction
                        apex_point = self.swing_start_point + self.apex_point_ratio * start_to_end_vector + \
                                     self.max_evasion_distance * apex_point_direction

                    else:
                        apex_point = evasion_point + (self.apex_point_ratio - collision_ratio) * start_to_end_vector
                    control_point_3 = 0.5 * (apex_point - evasion_point) + evasion_point
                    control_point_2 = evasion_point - self.retraction_distance * start_to_end_direction
                    retraction_point = 0.5 * (control_point_2 - control_point_1) + control_point_1
                    control_point_4 = apex_point + 0.5 * (1 - self.apex_point_ratio) * start_to_end_vector
                    return numpy.array(
                            [self.collision_point, control_point_1, retraction_point, control_point_2, evasion_point,
                             control_point_3, apex_point, control_point_4, self.swing_target_point])
            else:  # the collision happened after the leg reached the apex
                if collision_ratio > 0.9:
                    return numpy.array([self.collision_point, self.collision_point - apex_point_direction / 2,
                                        self.collision_point - apex_point_direction])
                if start_to_end_vector_to_collision_point_distance + self.evasion_distance > self.max_evasion_distance:
                    evasion_point = self.swing_start_point + collision_ratio * start_to_end_vector + \
                                    self.max_evasion_distance * apex_point_direction
                control_point_2 = evasion_point - self.retraction_distance * start_to_end_direction
                retraction_point = 0.5 * (control_point_2 - control_point_1) + control_point_1
                control_point_3 = evasion_point + (1 - collision_ratio) * start_to_end_vector
                return numpy.array(
                        [self.collision_point, control_point_1, retraction_point, control_point_2, evasion_point,
                         control_point_3, self.swing_target_point])

    def compute_bezier_points_with_joint_angles(self):  # ForNormalSwingMovement(self):
        # rospy.loginfo("target = " + str(self.swing_target_point) + " start = " + str(self.swing_start_point))
        start_to_end_vector = (self.swing_target_point - self.swing_start_point)
        start_angles = self.leg.compute_inverse_kinematics(self.swing_start_point)
        target_angles = self.leg.compute_inverse_kinematics(self.swing_target_point)
        angles = [(start_angles[0] + target_angles[0]) / 2,
                  (start_angles[1] + target_angles[1]) / 2 - CONST.DEFAULT_APEX_THIGH_OFFSET,
                  (start_angles[2] + target_angles[2]) / 2]
        if not self.leg.check_joint_ranges(angles):
            rospy.logerr('The provided angles for ' + self.leg.name + '(' + str(angles[0]) + ', ' + str(angles[1]) +
                         ', ' + str(angles[2]) + ') are not valid for the forward/inverse kinematics.')
        apex_point = self.leg.compute_forward_kinematics([angles[0], angles[1], angles[2]])
        if self.collision_point is None:  # if no collision happened
            control_point_1 = apex_point - 0.5 * self.apex_point_ratio * start_to_end_vector
            control_point_2 = apex_point + 0.5 * (1 - self.apex_point_ratio) * start_to_end_vector
            return numpy.array(
                    [self.swing_start_point, control_point_1, apex_point, control_point_2, self.swing_target_point])
        else:  # in case of a collision
            rospy.logerr("a collision occured! calculate bezier points based on positions instead of joint angles!")
            return self.compute_bezier_points()

    def move_to_next_point(self, activation):
        # if not self.mleg.leg_enabled:
        #    return

        if activation >= 0.5:
            if self.last_activation < 0.5:
                self.trajectory_generator.reset()
                self.swing_start_point = self.leg.ee_position()
                self.swing_target_point = None
                self.reacht_peak = False
                self.collision_point = None

            # if numpy.any(self.swing_target_point != self.mleg.aep_shifted):
            #    self.swing_target_point = self.mleg.aep_shifted
            #    bezier_points = self.compute_bezier_points()
            #    self.trajectory_generator.bezier_points = bezier_points
            # target_position = self.trajectory_generator.compute_next_target(
            #   desired_distance=self.swing_velocity / RSTATIC.controller_frequency)
            # rospy.loginfo("ee pos  = " + str(self.leg.ee_position()))
            # rospy.loginfo("angles  = " + str(self.leg.get_current_angles()))
            # rospy.loginfo("targets = " + str(self.leg.get_current_targets()))
            target_position, target_parameter = self.trajectory_generator.compute_next_target(
                    desired_distance=self.swing_velocity / RSTATIC.controller_frequency,
                    current_position=self.leg.ee_position())
            if target_parameter >= 0.5:
                self.reacht_peak = True
            # now it's just a matter of moving the leg to the next position
            # current_input_angles = self.leg.get_current_angles()
            # compute the values should for the next iteration
            next_angles = None
            try:
                next_angles = self.leg.compute_inverse_kinematics(target_position)
                if RSTATIC.DEBUG:
                    rospy.loginfo("current targets: " + str(self.leg.get_current_targets()))
                    rospy.loginfo("target position is: " + str(target_position))
                    rospy.loginfo("computed next angles as: " + str(next_angles))
                    rospy.loginfo("would reach pos: " + str(self.leg.compute_forward_kinematics(next_angles)))
                self.leg.set_command(next_angles)
            except ValueError:
                rospy.logerr("ValueError in " + str(
                        self.leg.name) + " during inverse kinematics computation.\n Tried to reach position " + str(
                        target_position) + "\ncurrent angles are: " + str(self.leg.get_current_angles()) +
                             "\nMaintaining current angles.")
                self.leg.set_command(self.leg.get_current_angles())
            # compute the difference
            # delta_angles = next_angles - numpy.array(current_input_angles)
            # compute the required speed in order to reach those angles
            # angle_vel = delta_angles * RSTATIC.controller_frequency

            # if self.mleg.wleg.leg.leg_enabled:
            #    self.mleg.wleg.addControlVelocities(angle_vel)
        self.last_activation = activation

    def end_swing_phase(self):
        pass


if __name__ == '__main__':
    # print("here0")
    # rospy.init_node('bezier_swing_controller', anonymous=True)
    temp = SwingMovementBezier()
    temp.swing_start_point = numpy.array([0., 0., 0.])  # the point where the swing phase starts
    temp.swing_target_point = numpy.array([1., 0., 0.])  # the point where the swing phase should end
    # at which position of the interval between the start and the end point the middle point should be placed
    temp.apex_point_ratio = 0.5
    # the offset that is added to the middle point that was computed on the connecting line between start and
    # end point using the apex_point_ratio concept.
    temp.apex_point_offset = numpy.array([0, 0, 0.4])
    temp.collision_point = numpy.array([0.8, 0, 0.256])
    # bezier_points = temp.compute_bezier_points()
    bezier_points = temp.compute_bezier_points_with_joint_angles()
    print(bezier_points)
    # temp.move_to_next_point(1)
