#!/usr/bin/env python

# Took code from:
# https://github.com/malteschilling/cognitiveWalker/blob/master/controller/reaCog/Movements/BodymodelStance/mmcBodyModel3D.py
# modified for PhantomX Robot

import math
import numpy
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC


##
#	A Body Model for a hexapod walker, based on MMC computation.
#	Now extended to 3 dimensions, (body segments are 3D and have now an orientation).
#
#	Higher level of the hierarchical body model for a six legged robot:
#	On this level
#		(- there are three segments described through each six vectors) - not relevant for phantomX
#		- each leg is represented only as a vector to the tip of the leg
#			for each leg there are two vectors - one starting at the front of the segment
#			(the leg vector) and one starting at the back of the segment (diag vector)
#			both ending at the tip of the leg
#	The body model is constructed as a Mean of Multiple Computation network:
#		- each variable vector is described by multiple kinematic relations
#			= computations
#		- these different computations are integrated using a mean calculation
#	The resulting set of equations directly defines a neural network connection matrix.
#	This can be computed in an iterative fashion - introduction of a recurrent connection
#	damps the overall relaxation procedure and prevents oscillations.
#	The network is able to compute forward, inverse and any mixed kinematic problem and
#	is quite fast (it only needs a few iteration steps).
#	It acts as an autoassociator - so a problem is given to the network as an incomplete
#	set of variables and the network fills in complementing missing values.
#	The network used here is special as it uses no specific reference coordinate system.
#	Described in the network are only relative vectors between certain body parts.
#	The footdiag vectors connecting the feet of the standing legs are actually
#	constituting a fixed reference frame in which coordinate systems and
#	representation of the environment can be grounded.
#	To move the body actually, there are special vectors in the body model which
#	explicitly represent a disturbance of the dynamics of the network (the pull vector
#	which is driving the delta vectors of the segments). One can envision this as
#	a passive motion paradigm: the body model is pulled into direction of this vector
#	and the model is following this movement (one can think of this as a stick model which
#	legs are attached to the ground).
#
#	The body model is used in connection to leg models. The leg vectors are enforced onto
#	the lower leg networks which compute corresponding joint angles for the legs from
#	this. Overall this is embedded in a processing loop:
#		- the leg network is updated by sensory data
#			(iterates once to integrate the values)
#		- the leg network updates the body model
#		- the body model gets a movement command (usually constant)
#			and is iterated (one step is sufficient)
#		- the body model pushes down the new leg vectors for the standing legs into
#			the leg networks (these are iterated for a few iteration steps and provide
#			joint angles)
#		- the joint motors are controlled by the new motor commands
##


class mmcBodyModelStance:

    # Initialisation of the body model.
    # The segment vectors are encoded here, the leg vectors are initialised here
    # but are actually set when a leg is really put on the ground (initially all are
    # assumed in the air, so an update of the legs is forced in the first iteration)
    def __init__(self, robot):  # , motiv_net, stab_thr):
        # set up marker publisher for rviz visualization
        self.visualization_pub = rospy.Publisher('/mmcBodyModel', Marker,
                queue_size=1)
        self.points = Marker()
        self.leg_lines = Marker()
        self.front_lines = Marker()
        self.segm_leg_ant_lines = Marker()
        self.segm_leg_post_lines = Marker()
        self.segm_line = Marker()
        self.segm_diag_to_right_lines = Marker()
        self.set_up_visualization()

        # self.motivationNetRobot = motiv_net -- not used for phantomX
        self.robot = robot
        # The stability is determined in the following way:
        # The connection between the two most hind legs touching the ground
        # (one leg on each side) is constructing a vector to the back.
        # The segment_factor is the fraction of the last body segment meeting this
        # vector (counted from in between middle and hind segment):
        # 	0 = between the two segments (0 part of the hind segment)
        #	-1 = at the end of the hind segment_factor
        # The robot is determined instable when the segment_factor is greater
        # than this threshold.
        # self.stability_threshold = stab_thr -- not yet used for phantomX

        # Used for storing stability calculation in visualization
        # self.temp_stability_fact = 0.

        # The height is currently set to a fixed value
        # Here a height net might be introduced?
        # The body model is now always enforcing this specific height.
        self.height = RSTATIC.stance_height
        self.width = RSTATIC.default_stance_width + 0.04

        self.ee_positions = [numpy.array([0.26, (self.width - 0.1), self.height]),
                             numpy.array([0.26, -(self.width - 0.1), self.height]),
                             numpy.array([0.0, (self.width), self.height]),
                             numpy.array([0.0, -(self.width), self.height]),
                             numpy.array([-0.26, (self.width - 0.1), self.height]),
                             numpy.array([-0.26, -(self.width - 0.1), self.height])]

        front_segment_start = [0.12, 0.0, 0.0]

        # Additional Vectors (from front of a segment to footpoint)
        # estimated based on estimated end effector positions.
        self.front_vect = [numpy.array([pos[0] - front_segment_start[0], pos[1] - front_segment_start[1],
                                        pos[2] - front_segment_start[2]]) for pos in self.ee_positions]

        # Segment defining vectors: is constructed as a diamond for the mounting points of
        # the legs: segm_leg_ant = from coxa to front (anterior)
        # 			segm_leg_post = from coxa to back (posterior)
        # vector defining the segments from posterior (end) point to anterior (start) point?
        # self.segm_post_ant = [numpy.array([0.22,0.0,0.0]),numpy.array([0.35, 0.0, 0.0]),numpy.array([0.36,0.0,0.0])]
        # self.segm_post_ant = [numpy.array([0.6, 0.0, 0.0]), numpy.array([0.12, 0.0, 0.0]),
        #    numpy.array([0.06, 0.0, 0.0])]
        self.segm_post_ant = numpy.array([0.24, 0.0, 0.0])
        self.segm_post_ant_norm = numpy.linalg.norm(self.segm_post_ant)

        # positions of the shoulder c1 joints for calculating real leg vectors
        self.c1_positions = [numpy.array([0.12, 0.06, 0.0]), numpy.array([0.12, -0.06, 0.0]),
                             numpy.array([0.0, 0.10, 0.0]),
                             numpy.array([0.0, -0.10, 0.0]), numpy.array([-0.12, 0.06, 0.0]),
                             numpy.array([-0.12, -0.06, 0.0])]
        # Real Leg vectors
        # self.leg_vect = [numpy.array([0., self.width, -self.height]), numpy.array([0., -self.width, -self.height]),
        #     numpy.array([0., self.width, -self.height]), numpy.array([0., -self.width, -self.height]),
        #     numpy.array([0., self.width, -self.height]), numpy.array([0., -self.width, -self.height])]
        self.leg_vect = [numpy.array([target[0] - start[0], target[1] - start[1],
                                      target[2] - start[2]]) for target, start in
                         zip(self.ee_positions, self.c1_positions)]

        # segm_leg_ant = from coxa to front (anterior) = real leg vector (coax to ee) - front vector (front of a segment to footpoint)
        # self.segm_leg_ant = [(self.leg_vect[0] - self.front_vect[0]), (self.leg_vect[1] - self.front_vect[1])]
        self.segm_leg_ant = [(self.leg_vect[i] - self.front_vect[i]) for i in
                             range(0, 6)]
        self.segm_leg_ant_norm = [numpy.linalg.norm(segm_vect) for segm_vect in self.segm_leg_ant]

        # segm_leg_post = from coxa to back (posterior)
        # self.segm_leg_post = [(self.segm_leg_ant[0] - self.segm_post_ant[0]), (self.segm_leg_ant[1] - self.segm_post_ant[0])]
        # self.segm_leg_post = [(self.segm_leg_ant[i] - self.segm_post_ant[i // 2]) for i in range(0, 6)] -- for 3 segments
        self.segm_leg_post = [(self.segm_leg_ant[i] - self.segm_post_ant) for i in range(0, 6)]
        self.segm_leg_post_norm = [numpy.linalg.norm(segm_vect) for segm_vect in self.segm_leg_post]

        # lines connecting shoulders for normalization of segment length after iteration step
        self.segm_diag_to_right = [(self.segm_leg_ant[i * 2] - self.segm_leg_ant[i * 2 + 1]) for i in
                                   range(0, len(self.segm_leg_ant) // 2)]
        self.segm_diag_norm = [numpy.linalg.norm(segm_vect) for segm_vect in self.segm_diag_to_right]

        # Ground contact - which feet are on the ground
        self.gc = [False, False, False, False, False, False]
        self.old_stance_motivation = [False, False, False, False, False, False]

        # Vectors between footpoints - these are a fixed coordinate system which
        # shall not be altered. The standing feet are connected through the ground
        # and their relation shall be constant.
        # The footdiags table has to be adapted whenever the configuration of the walker
        # changes - when a leg is lifted from the ground this leg vector is not forming
        # closed kinematic chains with the other legs anymore and the footdiags can
        # not be exploited anymore.
        # When a leg is touching the ground, new footdiags to the other standing
        # legs have to be established.
        self.footdiag = [[], [0], [0, 0], [0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0, 0]]
        for i in range(0, len(self.footdiag)):
            for j in range(0, i):
                self.footdiag[i][j] = self.set_up_foot_diag(i, j)

        # The explicit disturbance vectors of the network
        # self.delta_front = [numpy.array([0, 0, 0]), numpy.array([0, 0, 0]), numpy.array([0, 0, 0])]
        # self.delta_back = [[numpy.array([0, 0, 0])], [numpy.array([0, 0, 0])], [numpy.array([0, 0, 0])]]
        self.delta_front = numpy.array([0, 0, 0])
        self.delta_back = numpy.array([0, 0, 0])
        # These are operated through a pull at the front
        self.pull_front = numpy.array([0.0, 0.0, 0.0])
        self.pull_back = numpy.array([0.0, 0.0, 0.0])
        self.step = 0
        self.damping = 5

    """ **** Graphic methods: For Visualization of the body model in RVIZ **************************
    """

    def set_up_visualization(self):
        self.points.header.frame_id = self.leg_lines.header.frame_id = self.front_lines.header.frame_id = \
            self.segm_leg_ant_lines.header.frame_id = self.segm_leg_post_lines.header.frame_id = \
            self.segm_line.header.frame_id = self.segm_diag_to_right_lines.header.frame_id = "MP_BODY"
        self.points.header.stamp = self.leg_lines.header.stamp = self.front_lines.header.stamp = \
            self.segm_leg_ant_lines.header.stamp = self.segm_leg_post_lines.header.stamp = \
            self.segm_line.header.stamp = self.segm_diag_to_right_lines.header.stamp = rospy.Time.now()
        self.points.ns = self.leg_lines.ns = self.front_lines.ns = self.segm_leg_ant_lines.ns = \
            self.segm_leg_post_lines.ns = self.segm_line.ns = self.segm_diag_to_right_lines.ns = "points_and_lines"
        self.points.action = self.leg_lines.action = self.front_lines.action = self.segm_leg_ant_lines.action = \
            self.segm_leg_post_lines.action = self.segm_line.action = self.segm_diag_to_right_lines.action = Marker.ADD
        self.points.pose.orientation.w = self.leg_lines.pose.orientation.w = self.front_lines.pose.orientation.w = \
            self.segm_leg_ant_lines.pose.orientation.w = self.segm_leg_post_lines.pose.orientation.w = \
            self.segm_line.pose.orientation.w = self.segm_diag_to_right_lines.pose.orientation.w = 1.0

        self.points.id = 0
        self.leg_lines.id = 1
        self.front_lines.id = 2
        self.segm_leg_ant_lines.id = 3
        self.segm_leg_post_lines.id = 4
        self.segm_line.id = 5
        self.segm_diag_to_right_lines.id = 6

        self.points.type = Marker.POINTS
        self.leg_lines.type = Marker.LINE_LIST
        self.front_lines.type = Marker.LINE_LIST
        self.segm_leg_ant_lines.type = Marker.LINE_LIST
        self.segm_leg_post_lines.type = Marker.LINE_LIST
        self.segm_line.type = Marker.LINE_LIST
        self.segm_diag_to_right_lines.type = Marker.LINE_LIST

        self.points.scale.x = 0.005
        self.points.scale.y = 0.005

        self.leg_lines.scale.x = 0.0025
        self.front_lines.scale.x = 0.0025
        self.segm_leg_ant_lines.scale.x = 0.0025
        self.segm_leg_post_lines.scale.x = 0.0025
        self.segm_line.scale.x = 0.0025
        self.segm_diag_to_right_lines.scale.x = 0.0025

        self.points.color.g = 1.0
        self.points.color.a = 1.0
        self.leg_lines.color.a = 1.0
        self.front_lines.color.a = 1.0
        self.segm_leg_ant_lines.color.a = 1.0
        self.segm_leg_post_lines.color.a = 1.0
        self.segm_line.color.a = 1.0
        self.segm_diag_to_right_lines.color.a = 1.0

        self.leg_lines.color.b = 1.0
        self.front_lines.color.r = 1.0
        self.segm_leg_ant_lines.color.g = 1.0
        self.segm_leg_post_lines.color.g = 1.0
        self.segm_leg_post_lines.color.b = 1.0
        self.segm_line.color.r = 1.0
        self.segm_line.color.b = 1.0
        self.segm_diag_to_right_lines.color.r = 1.0
        self.segm_diag_to_right_lines.color.g = 1.0

    def set_line_color(self, list, r, g, b):
        # self.line_list.points.clear()
        list.color.r = r
        list.color.g = g
        list.color.b = b
        list.color.a = 1.0

    def pub_vecs(self, start, vecs, markers):
        # rospy.loginfo("#############################################in pub vecs")
        start_point = Point()
        start_point.x = start[0]
        start_point.y = start[1]
        start_point.z = start[2]
        self.points.points.append(start_point)
        for position in vecs:
            pos = Point()
            pos.x = start_point.x + position[0]
            pos.y = start_point.y + position[1]
            pos.z = start_point.z + position[2]
            self.points.points.append(pos)
            markers.points.append(start_point)
            markers.points.append(pos)

        rate = rospy.Rate(RSTATIC.controller_frequency)
        for i in range(0, 3):
            self.visualization_pub.publish(self.points)
            self.visualization_pub.publish(markers)
            rate.sleep()

    def pub_relative_vecs(self, start_points, vecs, markers):
        # rospy.loginfo("#############################################in pub relative vecs")
        # rospy.loginfo("start_points = " + str(start_points))
        # rospy.loginfo("vectors      = " + str(vecs))
        for idx in range(0, len(vecs)):
            start_point = Point()
            start = start_points[idx]
            start_point.x = start[0]
            start_point.y = start[1]
            start_point.z = start[2]
            # rospy.loginfo("append start point: " + str(start_point))
            self.points.points.append(start_point)
            pos = Point()
            pos.x = start_point.x + vecs[idx][0]
            pos.y = start_point.y + vecs[idx][1]
            pos.z = start_point.z + vecs[idx][2]
            # rospy.loginfo("type of vecs is: " + str(type(vecs)))
            # rospy.loginfo("target point x: " + str(pos.x) + " = start.point.x (" + str(
            #    start_point.x) + ") + vecs[idx][0] (" + str(vecs[idx][0]) + ")")
            # rospy.loginfo("append target point: " + str(pos))
            self.points.points.append(pos)
            markers.points.append(start_point)
            markers.points.append(pos)

        rate = rospy.Rate(RSTATIC.controller_frequency)
        for i in range(0, 3):
            self.visualization_pub.publish(self.points)
            self.visualization_pub.publish(markers)
            rate.sleep()

    """ **** Set up methods and calculation of vector methods ***************************
    """

    ##	Calculation of the vector connection the feet of two standing legs.
    #	When a leg is put on the ground a new connecting vector has to be established.
    #	This is calculated as the difference between the leg vectors.
    def set_up_foot_diag(self, start, target):
        diag_vec = self.leg_vect[target] - self.leg_vect[start] + self.get_segm_vectors_between_legs(start, target)
        return diag_vec

    ##	Providing the segment vectors connecting two legs.
    #	An equation is described by a series of vectors forming a closed kinematic
    #	chain (two leg vectors, the footdiag and -possibly- segment vectors in
    #	between). This function is calculating the segment vectors between the
    #	given legs.
    def get_segm_vectors_between_legs(self, start_leg, end_leg):
        # rospy.loginfo("in get_segm_vectors_between_legs start_leg = " + str(start_leg) + " end_leg = " + str(end_leg))
        leg_diff = (end_leg // 2 - start_leg // 2)
        if leg_diff == 0:
            return (self.segm_leg_post[start_leg] - self.segm_leg_post[end_leg])
        elif leg_diff == 1 or leg_diff == 2:
            # return (self.segm_leg_post[start_leg] - self.segm_leg_ant[end_leg])
            return (self.segm_leg_ant[start_leg] - self.segm_leg_ant[end_leg])
        elif leg_diff == -1 or leg_diff == -2:
            # return (self.segm_leg_ant[start_leg] - self.segm_leg_post[end_leg])
            return (self.segm_leg_ant[start_leg] - self.segm_leg_ant[end_leg])
        # elif leg_diff == 2:
        #     return (self.segm_leg_post[start_leg] - self.segm_post_ant[1] - self.segm_leg_ant[end_leg])
        # elif leg_diff == -2:
        #     return (self.segm_leg_ant[start_leg] + self.segm_post_ant[1] - self.segm_leg_post[end_leg])

    ##	Sets ground contact to false and removes this leg from the body
    #	model computations. As the leg is not part of the closed kinematic chains
    #	after being lifted from the ground it shall not participate.
    def lift_leg_from_ground(self, leg_nr):
        rospy.loginfo("lift leg from ground: " + RSTATIC.leg_names[leg_nr])
        if self.gc[leg_nr]:
            self.gc[leg_nr] = False

    ##	Sets the ground contact of the leg and initiates the calculation of connection
    #	vectors to all other standing legs (footdiag) which are used by the
    #	network.
    def put_leg_on_ground(self, leg_name, leg_vec):
        rospy.loginfo("put leg on ground: " + leg_name)
        leg_nr = RSTATIC.leg_names.index(leg_name)
        if not self.gc[leg_nr]:
            # Set leg and diag vector
            self.leg_vect[leg_nr] = numpy.array(leg_vec)
            self.front_vect[leg_nr] = self.leg_vect[leg_nr] - self.segm_leg_ant[leg_nr]
            # Construction of all foot vectors - the ones to legs in the air are not used!
            for i in range(0, leg_nr):
                self.footdiag[leg_nr][i] = self.set_up_foot_diag(leg_nr, i)
            for i in range(leg_nr + 1, 6):
                self.footdiag[i][leg_nr] = self.set_up_foot_diag(i, leg_nr)
            self.gc[leg_nr] = True

    ##	Update the current state of the legs.
    #	Only legs in stance mode are part of the body model (which is used for computation
    #	of the stance movement). When a leg switches between states it has to be added
    #	or removed from the body model.
    def updateLegStates(self):
        # rospy.loginfo("updateLegStates")
        # motiv leg needs to be singleLegController which has attribute swing that replaces inSwingPhase()
        # for motiv_leg, leg_nr in zip(self.motivationNetRobot.motivationNetLegs,
        #        range(len(self.motivationNetRobot.motivationNetLegs))):
        for motiv_leg, leg_nr in zip(self.robot.legs,
                range(len(self.robot.legs))):
            # if (motiv_leg.inSwingPhase()):
            if (motiv_leg.swing):
                if (self.old_stance_motivation[leg_nr]):
                    # TODO check motivationNetRobot
                    # who has the bodyModelStance in this Modell? RobotController?
                    # self.motivationNetRobot.bodyModelStance.lift_leg_from_ground(leg_nr)
                    self.lift_leg_from_ground(leg_nr)
                # print(motiv_leg.wleg.leg.name, " starts swing")
                self.old_stance_motivation[leg_nr] = False
            else:
                self.old_stance_motivation[leg_nr] = True

    """ **** Computation of the MMC equations *******************************************
    """

    ##	Compute the leg vectors: For all standing legs
    #	the new leg vectors are computed, summed and the mean is calculated
    #	(the old value is also integrated, weighted by the damping value)
    def compute_leg_computations_and_integrate(self, leg_nr):
        equation_counter = 1
        # segm_leg_vect = -self.delta_back[leg_nr // 2] + self.segm_leg_post[leg_nr] + self.segm_post_ant[leg_nr // 2] + \
        #                self.front_vect[leg_nr]  # 3 segments
        segm_leg_vect = -self.delta_back + self.segm_leg_post[leg_nr] + self.segm_post_ant + \
                        self.front_vect[leg_nr]
        segm_leg_vect += self.damping * self.leg_vect[leg_nr]
        equation_counter += self.damping
        for target_leg in range(0, len(self.gc)):
            if self.gc[target_leg] and target_leg != leg_nr:
                part_vec = numpy.array(self.leg_vect[target_leg]) + self.get_segm_vectors_between_legs(leg_nr,
                        target_leg)
                if target_leg < leg_nr:
                    part_vec -= self.footdiag[leg_nr][target_leg]
                elif target_leg > leg_nr:
                    part_vec += self.footdiag[target_leg][leg_nr]
                segm_leg_vect += part_vec
                equation_counter += 1
        return (segm_leg_vect / equation_counter)

    ##	Compute the front vectors: For all standing legs
    #	the new front vectors are computed, summed and the mean is calculated
    #	(the old value is also integrated, weighted by the damping value)
    def compute_front_computations_and_integrate(self, leg_nr):
        # rospy.loginfo("compute_front_computations_and_integrate: " + RSTATIC.leg_names[leg_nr])
        equation_counter = 1
        # new_front_vect = -self.delta_front[leg_nr // 2] - self.segm_post_ant[leg_nr // 2] + self.leg_vect[leg_nr] - \
        #                 self.segm_leg_post[leg_nr]  # 3 segments
        new_front_vect = -self.delta_front - self.segm_post_ant + self.leg_vect[leg_nr] - \
                         self.segm_leg_post[leg_nr]
        new_front_vect += self.damping * self.front_vect[leg_nr]
        equation_counter += self.damping
        # Computations of connections between legs in ground contacts
        # Compute equations to all other standing legs using the footdiag
        for target_leg in range(0, len(self.gc)):
            if self.gc[target_leg] and target_leg != leg_nr:
                part_vec = numpy.array(
                        self.front_vect[target_leg])  # + self.get_segm_vectors_between_front(leg_nr, target_leg)
                if target_leg < leg_nr:
                    part_vec -= self.footdiag[leg_nr][target_leg]
                elif target_leg > leg_nr:
                    part_vec += self.footdiag[target_leg][leg_nr]
                new_front_vect += part_vec
                equation_counter += 1
        return (new_front_vect / equation_counter)

    ##	Compute the segment vectors:
    #	Using equations including the two legs connected to the segment,
    #	integrating the explicit displacement given as delta
    #	and the recurrent old value of the vector.
    def compute_segment_leg_ant_computations_and_integrate(self, leg_nr):
        # rospy.loginfo("compute_segment_leg_ant_computations_and_integrate: " + RSTATIC.leg_names[leg_nr])
        equation_counter = 1
        # new_segm_leg_ant = self.segm_leg_post[leg_nr] + self.segm_post_ant[leg_nr // 2]
        new_segm_leg_ant = self.segm_leg_post[leg_nr] + self.segm_post_ant
        # Neighboring leg with respect to leg_nr
        new_segm_leg_ant += self.segm_leg_ant[leg_nr] * self.damping
        equation_counter += self.damping
        new_segm_leg_ant = new_segm_leg_ant / equation_counter
        return ((self.segm_leg_ant_norm[leg_nr] / numpy.linalg.norm(new_segm_leg_ant)) * new_segm_leg_ant)

    ##	Compute the segment vectors:
    #	Using equations including the two legs connected to the segment,
    #	integrating the explicit displacement given as delta
    #	and the recurrent old value of the vector.
    def compute_segment_leg_post_computations_and_integrate(self, leg_nr):
        # rospy.loginfo("compute_segment_leg_post_computations_and_integrate: " + RSTATIC.leg_names[leg_nr])
        equation_counter = 1
        # new_segm_leg_post = self.segm_leg_ant[leg_nr] - self.segm_post_ant[leg_nr // 2]
        new_segm_leg_post = self.segm_leg_ant[leg_nr] - self.segm_post_ant
        new_segm_leg_post += self.segm_leg_post[leg_nr] * self.damping
        equation_counter += self.damping
        new_segm_leg_post = new_segm_leg_post / equation_counter
        return ((self.segm_leg_post_norm[leg_nr] / numpy.linalg.norm(new_segm_leg_post)) * new_segm_leg_post)

    ##	Compute the segment vectors:
    #	Using equations including the two legs connected to the segment,
    #	integrating the explicit displacement given as delta
    #	and the recurrent old value of the vector.
    def compute_segm_post_ant_computations_and_integrate(self, seg_nr):
        # rospy.loginfo("compute_segm_post_ant_computations_and_integrate: segment nr = " + str(seg_nr))
        if seg_nr != 0:
            rospy.logerr("segment nr is " + str(seg_nr) + " but only 1 segment exists")
        equation_counter = 7
        new_segm_post_ant = self.segm_post_ant + self.delta_front - self.delta_back
        new_segm_post_ant += -self.segm_leg_post[0] + self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[1]
        new_segm_post_ant += -self.segm_leg_post[1] - self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[0]
        new_segm_post_ant += -self.segm_leg_post[2] + self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[3]
        new_segm_post_ant += -self.segm_leg_post[3] - self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[2]
        new_segm_post_ant += -self.segm_leg_post[4] + self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[5]
        new_segm_post_ant += -self.segm_leg_post[5] - self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[4]

        new_segm_post_ant += self.damping * self.segm_post_ant
        equation_counter += self.damping

        new_segm_div = new_segm_post_ant / equation_counter
        return ((self.segm_post_ant_norm / numpy.linalg.norm(new_segm_div)) * new_segm_div)

    ##	Compute the segment vectors:
    #	Using equations including the two legs connected to the segment,
    #	integrating the explicit displacement given as delta
    #	and the recurrent old value of the vector.
    def compute_segm_diag_computations_and_integrate(self, joint_nr):
        # rospy.loginfo("compute_segm_diag_computations_and_integrate: joint_nr = " + str(joint_nr))
        equation_counter = 2

        new_segm_diag = self.segm_leg_post[2 * joint_nr] + self.segm_post_ant - self.segm_leg_ant[1 + joint_nr * 2]
        new_segm_diag -= self.segm_leg_post[1 + 2 * joint_nr] + self.segm_post_ant - self.segm_leg_ant[joint_nr * 2]

        new_segm_diag += self.damping * self.segm_diag_to_right[joint_nr]
        equation_counter += self.damping
        new_segm_diag = new_segm_diag / equation_counter
        return ((self.segm_diag_norm[joint_nr] / numpy.linalg.norm(new_segm_diag)) * new_segm_diag)

    ##	The MMC Method:
    #	- the multiple computations are computed for each variable
    #	- the mean for each variable is calculated
    #	The new values are appended to the list of element values.
    #	For each variable new values are calculated through
    #	different equations.
    ##
    def mmc_iteration_step(self):
        rospy.loginfo(
                "mmc_iteration_step: pull_front = " + str(self.pull_front) + " pull_back = " + str(self.pull_back))
        self.delta_front = self.pull_front
        self.delta_back = self.pull_back

        front_vect = [self.compute_front_computations_and_integrate(i) for i in range(0, 6)]
        leg_vect = [self.compute_leg_computations_and_integrate(i) for i in range(0, 6)]
        segm_leg_ant = [self.compute_segment_leg_ant_computations_and_integrate(i) for i in range(0, 6)]
        segm_leg_post = [self.compute_segment_leg_post_computations_and_integrate(i) for i in range(0, 6)]
        segm_post_ant = self.compute_segm_post_ant_computations_and_integrate(0)
        segm_diag_to_right = [self.compute_segm_diag_computations_and_integrate(i) for i in range(0, 3)]
        for i in range(0, 6):
            self.segm_leg_ant[i] = segm_leg_ant[i]
            self.segm_leg_post[i] = segm_leg_post[i]
            self.front_vect[i] = front_vect[i]
            self.leg_vect[i] = leg_vect[i]
        # self.pub_relative_vecs(self.c1_positions, self.segm_leg_ant, self.segm_leg_ant_lines)
        # self.pub_relative_vecs(self.c1_positions, self.segm_leg_post, self.segm_leg_post_lines)
        # self.pub_vecs([0.12, 0.0, 0.0], self.front_vect, self.front_lines)
        # self.pub_relative_vecs(self.c1_positions, self.leg_vect, self.leg_lines)
        self.segm_diag_to_right[0] = segm_diag_to_right[0]
        # self.pub_relative_vecs([self.c1_positions[i * 2] for i in range(0, len(self.c1_positions) // 2)],
        #    self.segm_diag_to_right, self.segm_diag_to_right_lines)
        self.segm_post_ant = segm_post_ant
        # self.pub_vecs([-0.12, 0.0, 0.0], [self.segm_post_ant], self.segm_line)

        self.step += 1

    """ **** Get, set methods - connection to the robot simulator ***********************
    """

    ##	Pull the body model into a direction relative to the current first
    #	segment. Takes an angle (0 os straight ahead) and a velocity factor
    #	(around 0.1-0.2 should be fine) to come up with a corresponding pull vector.
    def pullBodyModelAtFrontIntoRelativeDirection(self, pull_angle, speed_fact):
        rospy.loginfo(
                "pullBodyModelAtFrontIntoRelativeDirection: angle = " + str(pull_angle) + " speed = " + str(speed_fact))
        pull_angle_BM = pull_angle + math.atan2(self.segm_post_ant[1], self.segm_post_ant[0])
        self.pull_front[0] = speed_fact * math.cos(pull_angle_BM)  # pull x
        self.pull_front[1] = speed_fact * math.sin(pull_angle_BM)  # pull y

    ##	Pull the body model into a direction relative to the last body
    #	segment. Takes an angle (0 means straight backwards) and a velocity factor
    #	(around 0.1 - positive means backwards walking!)
    #	to come up with a corresponding pull vector.
    def pullBodyModelAtBackIntoRelativeDirection(self, pull_angle, speed_fact):
        rospy.loginfo(
                "pullBodyModelAtBackIntoRelativeDirection: angle = " + str(pull_angle) + " speed = " + str(speed_fact))
        # pull_angle_BM = pull_angle + math.atan2(-self.segm_post_ant[2][1], -self.segm_post_ant[2][0]) -- 3 segments
        pull_angle_BM = pull_angle + math.atan2(-self.segm_post_ant[1], -self.segm_post_ant[0])
        self.pull_back[0] = speed_fact * math.cos(pull_angle_BM)  # pull x
        self.pull_back[1] = speed_fact * math.sin(pull_angle_BM)  # pull y

    def get_leg_vector(self, leg_name):
        # rospy.loginfo("get_leg_vector: " + leg_name)
        leg_nr = RSTATIC.leg_names.index(leg_name)
        target_vec_wn = [self.leg_vect[leg_nr][0], self.leg_vect[leg_nr][1], self.leg_vect[leg_nr][2], 0]
        return target_vec_wn

    def get_ground_contact(self, leg_nr):
        # rospy.loginfo("get_ground_contact: " + RSTATIC.leg_names[leg_nr])
        return self.gc[leg_nr]
