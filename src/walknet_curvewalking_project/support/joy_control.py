#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from walknet_curvewalking.msg import robot_control
from enum import Enum

class PS4ButtonMapping(Enum):
    CROSS = 0
    SQUARE = 3
    TRIANGLE = 2
    CIRCLE = 1
    R1 = 5
    R2 = 7
    L1 = 4
    L2 = 6
    SHARE = 8
    OPTIONS = 9
    PS = 10
    JOYLEFT = 11
    JOYRIGHT = 12

class PS4AxesMapping(Enum):
    DPADY = 7
    DPADX = 6
    JOYLEFTX = 0
    JOYLEFTY = 1
    JOYRIGHTX = 3
    JOYRIGHTY = 4
    R2 = 2
    L2 = 5



class JoyController:
    def __init__(self):
        self.current_speed = 0.0
        self.current_dir = 0.0

    def control_callback(self, data):
        if data.buttons[PS4ButtonMapping.CROSS.value] == 1:
            self.current_speed = 0.0
            self.current_dir = 0.0
        elif data.axes[PS4AxesMapping.DPADX.value] < -0.5:
            if self.current_dir-0.2 < -1.57:
                self.current_dir = -1.57
            elif self.current_dir > 1.5:
                self.current_dir = 1.4
            else:
                self.current_dir -= 0.2
        elif data.axes[PS4AxesMapping.DPADX.value] > 0.5:
            if self.current_dir + 0.2 > 1.57:
                self.current_dir = 1.57
            elif self.current_dir < -1.5:
                self.current_dir = -1.4
            else:
                self.current_dir += 0.2
        elif data.axes[PS4AxesMapping.DPADY.value] < -0.5:
            if self.current_speed - 0.01 < 0:
                self.current_speed = 0.0
            else:
                self.current_speed -= 0.01
        elif data.axes[PS4AxesMapping.DPADY.value] > 0.5:
            if self.current_speed + 0.01 > 0.05:
                self.current_speed = 0.05
            else:
                self.current_speed += 0.01
        else:
            return
        self.brodcast_robot_control()

    def brodcast_robot_control(self):
        if not rospy.is_shutdown():
            msg = robot_control()
            msg.speed_fact = self.current_speed
            msg.pull_angle = self.current_dir
            rospy.loginfo("JoyController publish msg: " + str(msg))
            pub.publish(msg)


if __name__ == '__main__':
    rospy.loginfo("IN JOY CONTROL")

    pub = rospy.Publisher('/control_robot', robot_control, queue_size=1)
    rospy.init_node('joy_controller')

    joy_controller = JoyController()

    rospy.Subscriber('/wxmark4/joy', Joy, joy_controller.control_callback)

    rospy.spin()
