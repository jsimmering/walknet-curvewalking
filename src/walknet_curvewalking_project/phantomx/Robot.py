import numpy
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
from walknet_curvewalking_project.controller.single_leg_controller import SingleLegController
from walknet_curvewalking_project.phantomx.mmcBodyModel3D import mmcBodyModelStance
from walknet_curvewalking_project.support import stability


class Robot:
    def __init__(self, name, nh):
        self.name = name

        self.center_of_mass_of_body_segments = numpy.array([0, 0, 0])
        self.mass_of_body_segments = 1.4

        self.body_model = mmcBodyModelStance(self)

        self.legs = []
        for name in RSTATIC.leg_names:
            swing = False
            if name == 'rm' or name == 'lf' or name == 'lr':
                # swing = True
                self.legs.append(SingleLegController(name, nh, swing, self))
            if name == 'lm' or name == 'rf' or name == 'rr':
                swing = True
                self.legs.append(SingleLegController(name, nh, swing, self))

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

    def pub_shortest_vectors(self, vecs, com):
        self.shortest_vectors.points.clear()
        point1 = Point(com[0], com[1], com[2])
        for vec in vecs:
            point2 = Point(com[0] + vec[0], com[1] + vec[1], com[2] + vec[2])
            self.shortest_vectors.points.append(point1)
            self.shortest_vectors.points.append(point2)

        self.com_point.points.clear()
        self.com_point.color.b += 0.05
        self.com_point.points.append(point1)

        self.visualization_pub.publish(self.com_point)
        self.visualization_pub.publish(self.shortest_vectors)

    def check_stability(self):
        com = self._get_center_of_mass()
        rospy.loginfo("Center of Mass = " + str(com))
        temp_foot_positions = []
        for i in range(0, len(RSTATIC.leg_names)):
            if self.body_model.gc[i]:
                temp_foot_positions.append(self.legs[i].leg.apply_c1_static_transform() +
                                           self.body_model.get_leg_vector(self.legs[i].leg.name))

        if len(temp_foot_positions) > 0:
            convex_hull_points = stability.convex_hull(list(temp_foot_positions))

            projected_com = stability.project_com_onto_ground_plane(temp_foot_positions, com)
            # If the center of mass lies inside the support polygon
            if stability.is_point_inside_convex_hull(convex_hull_points, projected_com):
                rospy.loginfo("stable")
                shortest_vectors = stability.shortest_vectors_to_convex_hull(convex_hull_points, projected_com)
                self.pub_shortest_vectors(shortest_vectors, projected_com)
            else:
                rospy.logwarn("Unstable!")
                self.pub_shortest_vectors([], projected_com)
