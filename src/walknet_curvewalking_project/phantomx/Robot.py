#!/usr/bin/env python3
import datetime

import numpy
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
from walknet_curvewalking_project.controller.single_leg_controller import SingleLegController
from walknet_curvewalking_project.phantomx.mmcBodyModel3D import mmcBodyModelStance
from walknet_curvewalking_project.phantomx.mmcBodyModel3D_matrix import mmcBodyModelStance_matrix
from walknet_curvewalking_project.support import stability


class Robot:
    def __init__(self, name, nh):
        self.name = name
        self.running = True
        self.log_data = True
        self.write_at_end = False
        self.str_list = []

        self.center_of_mass_of_body_segments = numpy.array([0, 0, 0])
        self.mass_of_body_segments = 1.4
        self.last_state_stable = True
        self.unstable_count = 0

        self.body_model = mmcBodyModelStance(self)
        # self.body_model = mmcBodyModelStance_matrix(self)
        self.stance_speed = 0.0
        self.direction = 0.0
        self.file_name = "logs/walknet_stability_"
        self.file_suffix = ""

        self.legs = []
        for name in RSTATIC.leg_names:
            swing = False
            if name == 'rm' or name == 'lf' or name == 'lr':
                # swing = True
                self.legs.append(SingleLegController(name, nh, swing, self))
            if name == 'lm' or name == 'rf' or name == 'rr':
                # swing = True
                self.legs.append(SingleLegController(name, nh, swing, self))

        self.viz = False
        if self.viz:
            self.viz_pub_rate = rospy.Rate(RSTATIC.controller_frequency)
            self.visualization_pub = rospy.Publisher('/com', Marker, queue_size=1)
            self.com_point = Marker()
            self.shortest_vectors = Marker()
            self.com_point.header.frame_id = self.shortest_vectors.header.frame_id = "MP_BODY"
            self.com_point.header.stamp = self.shortest_vectors.header.stamp = rospy.Time.now()
            self.com_point.ns = self.shortest_vectors.ns = self.name + "_points_and_lines"
            self.com_point.action = self.shortest_vectors.action = Marker.ADD
            self.com_point.pose.orientation.w = self.shortest_vectors.pose.orientation.w = 1.0
            self.com_point.id = 20
            self.shortest_vectors.id = 21
            self.com_point.type = Marker.POINTS
            self.shortest_vectors.type = Marker.LINE_LIST
            self.com_point.scale.x = 0.005
            self.com_point.scale.y = 0.005
            self.shortest_vectors.scale.x = 0.001
            self.com_point.color.r = 1.0
            self.com_point.color.b = 0.0
            self.com_point.color.a = 1.0
            self.shortest_vectors.color.g = 0.0
            self.shortest_vectors.color.a = 1.0

    def initialize_body_model(self):
        ee_positions = [leg.leg.ee_position() for leg in self.legs]
        self.body_model.initialize_body_model(ee_positions)

    def get_com_of_leg(self, leg_num):
        return self.legs[leg_num].leg.leg_center_of_mass()

    def center_of_mass(self):
        return self._get_center_of_mass()

    def _get_center_of_mass(self):
        sum_of_centers_of_mass = numpy.zeros(3)
        for leg_num in range(6):
            sum_of_centers_of_mass += self.get_com_of_leg(leg_num) * self.legs[leg_num].leg.mass
        sum_of_centers_of_mass += self.center_of_mass_of_body_segments * self.mass_of_body_segments
        center_of_mass = sum_of_centers_of_mass / (
                numpy.sum([leg.leg.mass for leg in self.legs]) + self.mass_of_body_segments)

        return center_of_mass

    def pub_com_vectors(self, com):
        if not self.viz:
            return

        self.com_point.points.clear()
        self.com_point.color.b += 0.05
        self.com_point.points.append(Point(com[0], com[1], com[2]))

        self.visualization_pub.publish(self.com_point)

    def check_stability(self):
        com = self._get_center_of_mass()
        # rospy.loginfo("Center of Mass = " + str(com))
        temp_foot_positions = []
        str_list = ["{}".format(rospy.Time.now().to_sec())]
        # self.str_list.extend(str(rospy.Time.now().to_sec()))
        leg_list = [RSTATIC.leg_names.index('lf'), RSTATIC.leg_names.index('lm'), RSTATIC.leg_names.index('lr'),
                    RSTATIC.leg_names.index('rr'), RSTATIC.leg_names.index('rm'), RSTATIC.leg_names.index('rf')]
        for i in leg_list:
            if self.body_model.gc[i]:
                # temp_foot_position = self.legs[i].leg.apply_c1_static_transform() + self.body_model.get_leg_vector(
                #         self.legs[i].leg.name)
                temp_foot_position = self.legs[i].leg.ee_position()
                temp_foot_positions.append(temp_foot_position)
                str_list.extend(";{x};{y};{z}".format(x=temp_foot_position[0], y=temp_foot_position[1],
                        z=temp_foot_position[2]))
            else:
                str_list.extend(";{x};{y};{z}".format(x=0.0, y=0.0, z=0.0))

        str_list.extend(";{x};{y};{z}".format(x=com[0], y=com[1], z=com[2]))
        if len(temp_foot_positions) > 0:
            convex_hull_points = stability.convex_hull(list(temp_foot_positions))

            # removed com projection, pcom usually less than 0.004meters = 4mm away from com and
            # can be calculated afterwards from com and ee data ⇒ remove pcom computation and logging from walknet
            projected_com = stability.project_com_onto_ground_plane(temp_foot_positions, com)
            if projected_com is None:
                self.last_state_stable = False
                rospy.logwarn("Unstable! Not enough legs on ground temp_foot_positions = " + str(temp_foot_positions))
                str_list.append("\n")
                if self.write_at_end:
                    self.str_list.extend(str_list)
                else:
                    self.write_stability_data_to_file(''.join(str_list))
                self.unstable_count += 1
                return False

            # pcom_error = numpy.linalg.norm(projected_com[:-1] - com[:-1])
            # rospy.loginfo("pcom xy error: pcom = {} pcom xy = {}, com = {} com xy = {} pcom xy error = {}".format(projected_com, projected_com[:-1], com, com[:-1], pcom_error))
            str_list.append(";{x};{y};{z}".format(x=projected_com[0], y=projected_com[1], z=projected_com[2]))
            if not stability.is_point_inside_convex_hull(convex_hull_points, projected_com):
                self.last_state_stable = False
                rospy.logwarn("Unstable!")
                # self.pub_com_vectors(projected_com)
                str_list.extend("\n")
                if self.write_at_end:
                    self.str_list.extend(str_list)
                else:
                    self.write_stability_data_to_file(''.join(str_list))
                self.unstable_count += 1
                return False
            # If the center of mass lies inside the support polygon
            elif not self.last_state_stable:
                rospy.loginfo("back to stable state\n\n")
                self.last_state_stable = True
        str_list.extend("\n")
        if self.write_at_end:
            self.str_list.extend(str_list)
        else:
            self.write_stability_data_to_file(''.join(str_list))
        return True

    def write_stability_data_to_file(self, data):
        # rospy.loginfo("write to file: " + data)
        if self.log_data:
            with open(self.file_name + self.file_suffix, 'a') as f_handle:
                f_handle.write(data)

    def write_all_stability_data_to_file(self):
        # rospy.loginfo("write to file: " + data)
        if self.log_data:
            with open(self.file_name + self.file_suffix, 'a') as f_handle:
                f_handle.write(''.join(self.str_list))

    def initialize_stability_data_file(self):
        if self.log_data:
            time = datetime.datetime.now()
            self.file_suffix = str(RSTATIC.controller_frequency) + "hz_" + str(round(self.stance_speed, 4)) + "s_" + \
                               str(round(self.direction, 3)) + "dir_on_" + str(time.month) + "-" + str(time.day) + \
                               "_at_" + str(time.hour) + "-" + str(time.minute) + "-" + str(time.second)
            print("DATA COLLECTOR MODEL POSITION NAME: ", self.file_name + self.file_suffix)
            with open(self.file_name + self.file_suffix, "a") as f_handle:
                # leg_list = 'lf', 'lm', 'lr', 'rr', 'rm', 'rf'
                f_handle.write(
                        "time;lf x;lf y;lf z;lm x;lm y;lm z;lr x;lr y;lr z;rr x;rr y;rr z;rm x;rm y;rm z;rf x;rf y;rf z;com x;com y;com z\n")
