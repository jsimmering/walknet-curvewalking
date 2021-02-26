import numpy
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
from walknet_curvewalking_project.controller.single_leg_controller import SingleLegController
from walknet_curvewalking_project.phantomx.mmcBodyModel3D import mmcBodyModelStance


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
        self.com_point.header.frame_id = "MP_BODY"
        self.com_point.header.stamp = rospy.Time.now()
        self.com_point.ns = self.name + "_points_and_lines"
        self.com_point.action = Marker.ADD
        self.com_point.pose.orientation.w = 1.0
        self.com_point.id = 20
        self.com_point.type = Marker.POINTS
        self.com_point.scale.x = 0.005
        self.com_point.scale.y = 0.005
        self.com_point.color.r = 1.0
        self.com_point.color.b = 0.0
        self.com_point.color.a = 1.0

    def get_com_of_leg(self, leg_num):
        return self.legs[leg_num].leg.leg_center_of_mass()

    def center_of_mass(self):
        return self.get_center_of_mass()

    def get_center_of_mass(self):
        sum_of_centers_of_mass = numpy.zeros(3)
        for leg_num in range(6):
            sum_of_centers_of_mass += self.get_com_of_leg(leg_num) * self.legs[leg_num].leg.mass
        sum_of_centers_of_mass += self.center_of_mass_of_body_segments * self.mass_of_body_segments
        center_of_mass = sum_of_centers_of_mass / (
                numpy.sum([leg.leg.mass for leg in self.legs]) + self.mass_of_body_segments)

        self.com_point.points.clear()
        self.com_point.color.b += 0.05
        point1 = Point(center_of_mass[0], center_of_mass[1], center_of_mass[2])
        self.com_point.points.append(point1)
        self.visualization_pub.publish(self.com_point)

        return center_of_mass
