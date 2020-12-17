#!/usr/bin/env python3

import rospy
import threading
import walknet_curvewalking.phantomx.RobotSettings as RSTATIC
from walknet_curvewalking.controller.single_leg_controller import SingleLegController


class RobotController:
    def __init__(self, name, note_handle):
        self.nh = note_handle
        self.name = name
        self.movement_dir = 1
        self.legs = []
        for name in RSTATIC.leg_names:
            swing = False
            if name == 'lm':
                swing = True
                leg_c = SingleLegController(name, self.nh, swing)
                self.legs.append(leg_c)
            if name == 'rm':
                leg_c = SingleLegController(name, self.nh, swing)
                self.legs.append(leg_c)

    def start_walk(self):
        for leg in self.legs:
            thread = threading.Thread(target=leg.manage_walk)
            thread.daemon = True
            thread.start()


if __name__ == '__main__':
    nh = rospy.init_node('robot_controller', anonymous=True)
    robot_controller = RobotController('robot', nh)
    # rospy.spin()
    try:
        robot_controller.start_walk()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
