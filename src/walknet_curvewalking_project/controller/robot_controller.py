#!/usr/bin/env python3

import rospy
import threading

from walknet_curvewalking.msg import robot_control
from walknet_curvewalking_project.support.constants import CONTROLLER_FREQUENCY

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
from walknet_curvewalking_project.phantomx.mmcBodyModel3D import mmcBodyModelStance
from walknet_curvewalking_project.controller.single_leg_controller import SingleLegController


class RobotController:
    def __init__(self, name, note_handle):
        self.debug = True

        self.nh = note_handle
        self.name = name
        self.walk_motivation = False
        self.legs = []
        self.body_model = mmcBodyModelStance(self)
        for name in RSTATIC.leg_names:
            swing = False
            if name == 'rm' or name == 'lf' or name == 'lr':
                leg_c = SingleLegController(name, self.nh, swing, self)
                self.legs.append(leg_c)
            if name == 'lm' or name == 'rf' or name == 'rr':
                # swing = True
                leg_c = SingleLegController(name, self.nh, swing, self)
                #leg_c.manage_stance()
                self.legs.append(leg_c)
        # for leg in self.legs:
        #    rospy.loginfo("############################### leg: " + leg.name + " ee_pos: " + str(leg.leg.ee_position()))
        self.control_robot_sub = rospy.Subscriber('/control_robot', robot_control, self.control_robot_callback)

    def control_robot_callback(self, data):
        if data.speed_fact > 0:
            self.body_model.pullBodyModelAtFrontIntoRelativeDirection(data.pull_angle, data.speed_fact)
            self.body_model.pullBodyModelAtBackIntoRelativeDirection(0, 0)
            self.walk_motivation = True

    def init_body_model(self):
        for leg in self.legs:
            self.body_model.put_leg_on_ground(leg.name, leg.leg.ee_position()[0:3])
            rospy.loginfo("BODY MODEL LEG INIT: " + str(leg.name) + " ee:pos: " + str(leg.leg.ee_position()))
        self.body_model.updateLegStates()

    # Update all the leg networks.
    # Main Processing Step?
    def updateStanceBodyModel(self):
        if self.debug:
            mleg = self.legs[4]
            print("GC: ", mleg.leg.predictedGroundContact(), " - ", mleg.leg.ee_position()[2])
            print("SWING: ", mleg.swing)
            # input()

        self.body_model.updateLegStates()

        for i in range(0, 6):
            # if (self.motivationNetLegs[i].swing_motivation.output_value > 0.5):
            if self.legs[i].swing:
                self.body_model.lift_leg_from_ground(i)

        self.body_model.mmc_iteration_step()

    def walk(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY)
        for leg in self.legs:
            while not leg.leg.is_ready():
                rospy.loginfo("leg not connected yet! wait...")
                rate.sleep()
        rospy.loginfo("leg connected start walking")
        self.init_body_model()
        rospy.loginfo("------------------------------------------after init_body_model()")

        for leg in self.legs:
            thread = threading.Thread(target=leg.manage_walk_bezier_body_model)
            thread.daemon = True
            thread.start()

    def start_walk(self):
        for leg in self.legs:
            thread = threading.Thread(target=leg.manage_walk_bezier)
            thread.daemon = True
            thread.start()

    def move_body(self):
        for leg in self.legs:
            thread = threading.Thread(target=leg.manage_stance)
            thread.daemon = True
            thread.start()

    def move_body_cohesive(self):
        while not rospy.is_shutdown():
            self.updateStanceBodyModel()
            for leg in self.legs:
                # input("press any key to performe the next step.")
                leg.manage_stance()


if __name__ == '__main__':
    nh = rospy.init_node('robot_controller', anonymous=True)
    robot_controller = RobotController('robot', nh)
    # rospy.spin()
    try:
        # robot_controller.walk()
        # robot_controller.start_walk()
        robot_controller.move_body_cohesive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
