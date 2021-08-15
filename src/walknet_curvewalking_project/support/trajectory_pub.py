#!/usr/bin/env python3

import rospy
from walknet_curvewalking.msg import robot_control
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def talker():
    if not rospy.is_shutdown():
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = ['c1_lf_joint', 'thigh_lf_joint', 'tibia_lf_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0]
        point.time_from_start = rospy.Duration(1, 0)
        msg.points = [point]
        rospy.loginfo("publish msg: " + str(msg))
        pub.publish(msg)


if __name__ == '__main__':
    pub = rospy.Publisher('/lf_controller/command', JointTrajectory, queue_size=5)
    rospy.init_node('talker', anonymous=True)
    # rospy.Rate(1).sleep()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

# example for usage:
# rosrun walknet_curvewalking robot_control_pub.py
# rosrun walknet_curvewalking robot_control_pub.py _speed:=<value> _direction:=<value>
# rosrun walknet_curvewalking robot_control_pub.py _speed:=0.2 _direction:=0.0
