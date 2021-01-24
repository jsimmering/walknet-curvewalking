import rospy
from walknet_curvewalking.msg import robot_control


def talker():
    pub = rospy.Publisher('/control_robot', robot_control, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    if not rospy.is_shutdown():
        msg = robot_control()
        msg.speed_fact = 0.1
        msg.pull_angle = 0.0
        pub.publish(msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
