#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import *


def talker():
    if not rospy.is_shutdown():
        # msg = ModelState('phantomx', Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 0.0)),
        msg = ModelState('wxmark4', Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 0.0)),
                Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), '')
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
