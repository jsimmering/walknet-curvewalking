#!/usr/bin/env python3

import rospy
from walknet_curvewalking.msg import robot_control


def talker(speed_fact, pull_angle):
    if not rospy.is_shutdown():
        msg = robot_control()
        msg.speed_fact = speed_fact
        msg.pull_angle = pull_angle
        rospy.loginfo("publish msg: " + str(msg))
        pub.publish(msg)


if __name__ == '__main__':
    pub = rospy.Publisher('/control_robot', robot_control, queue_size=5)
    rospy.init_node('talker', anonymous=True)
    # rospy.Rate(1).sleep()
    speed = rospy.get_param('~speed', 0.075)
    direction = rospy.get_param('~direction', 0.0)
    try:
        talker(speed, direction)
    except rospy.ROSInterruptException:
        pass

# example for usage:
# rosrun walknet_curvewalking robot_control_pub.py
# rosrun walknet_curvewalking robot_control_pub.py _speed:=<value> _direction:=<value>
# rosrun walknet_curvewalking robot_control_pub.py _speed:=0.2 _direction:=0.0
