#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import *


def talker():
    if not rospy.is_shutdown():
        msg = ModelState('phantomx', Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 0.0)),
                Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), '')
        # msg.model_name = 'phantomx'
        # msg.pose.position.x = 0.0
        # msg.pose.position.y = 0.0
        # msg.pose.position.z = 0.0
        # msg.pose.orientation.x = 0.0
        # msg.pose.orientation.y = 0.0
        # msg.pose.orientation.z = 0.0
        # msg.pose.orientation.w = 0.0
        # msg.twist.linear.x = 0.0
        # msg.twist.linear.y = 0.0
        # msg.twist.linear.z = 0.0
        # msg.twist.angular.x = 0.0
        # msg.twist.angular.y = 0.0
        # msg.twist.angular.z = 0.0
        # msg.reference_frame = ''
        rospy.loginfo("publish msg: " + str(msg))
        pub.publish(msg)


if __name__ == '__main__':
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    rospy.init_node('talker5', anonymous=True)
    try:
        rospy.sleep(1)
        talker()
    except rospy.ROSInterruptException:
        pass
