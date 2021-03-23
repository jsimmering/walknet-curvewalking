#!/usr/bin/env python3
import datetime
import sys

import rospy
from gazebo_msgs.msg import ModelStates
from walknet_curvewalking.msg import robot_control


class DataCollector:
    def __init__(self, circles):
        self.running = True
        self.walking = False
        self.init_stability = False
        self.file_name = ""
        self.circles_to_walk = circles
        self.circle_count = 0
        self.last_in_origin_area = None
        self.current_position = [0, 0, 0]
        self.rate = rospy.Rate(2)

    def control_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " control_callback heard %s", data)
        if data.speed_fact > 0:
            time = datetime.datetime.now()
            self.file_name = "logs/walknet_position_" + str(round(data.speed_fact, 4)) + "s_" + \
                             str(round(data.pull_angle, 3)) + "dir_on_" + str(time.month) + "-" + str(time.day) + \
                             "_at_" + str(time.hour) + "-" + str(time.minute) + "-" + str(time.second)
            print("DATA COLLECTOR MODEL POSITION NAME: ", self.file_name)
            with open(self.file_name, "a") as f_handle:
                f_handle.write(
                        "time;point x;point y;point z;quaternion x;quaternion y;quaternion z;quaternion w;linear x;linear y;linear z;angular x;angular y;angular z\n")

            self.walking = True
            self.last_in_origin_area = rospy.Time.now()
        else:
            self.walking = False
            self.running = False

    def position_callback(self, data):
        if self.walking and self.running:
            robot_position = data.pose[data.name.index('phantomx')]
            self.current_position = [robot_position.position.x,
                                     robot_position.position.y]
            self.write_positiion_to_file(robot_position, data.twist[data.name.index('phantomx')])
        elif not self.running:
            rospy.signal_shutdown("Run finished done collecting Data! Shutting down")

    def write_positiion_to_file(self, pose, twist):
        with open(self.file_name, 'a') as f_handle:
            f_handle.write(
                    "{time};{point_x};{point_y};{point_z};{quaternion_x};{quaternion_y};{quaternion_z};{quaternion_w};{linear_x};{linear_y};{linear_z};{angular_x};{angular_y};{angular_z}\n".format(
                            time=rospy.Time.now().to_sec(),
                            point_x=pose.position.x,
                            point_y=pose.position.y,
                            point_z=pose.position.z,
                            quaternion_x=pose.orientation.x,
                            quaternion_y=pose.orientation.y,
                            quaternion_z=pose.orientation.z,
                            quaternion_w=pose.orientation.w,
                            linear_x=twist.linear.x,
                            linear_y=twist.linear.y,
                            linear_z=twist.linear.z,
                            angular_x=twist.angular.x,
                            angular_y=twist.angular.y,
                            angular_z=twist.angular.z))

    def check_if_circle_finished(self):
        while not rospy.is_shutdown():
            if self.walking and self.running:
                if self.current_position[0] > 0 and self.current_position[1] > 0:
                    now = rospy.Time.now()
                    if now - self.last_in_origin_area > rospy.Duration(60):
                        self.circle_count += 1
                        if self.circle_count >= self.circles_to_walk:
                            self.brodcast_stop_walking()
                    else:
                        self.last_in_origin_area = now
            self.rate.sleep()

    def brodcast_stop_walking(self):
        if not rospy.is_shutdown():
            msg = robot_control()
            msg.speed_fact = 0.0
            msg.pull_angle = 0.0
            rospy.loginfo("DataCollector publish msg: " + str(msg))
            pub.publish(msg)


if __name__ == '__main__':
    circles = 0
    if len(sys.argv) > 1:
        rospy.loginfo("walk " + sys.argv[1] + "circles")
        try:
            circles = int(sys.argv[1])
        except ValueError:
            rospy.loginfo("wrong usage. Use: rosrun walknet_curvewalking DataCollector.py <number_of_circles>")
            rospy.loginfo("walking until stop command")

    if circles == 0:
        rospy.loginfo("walking until stop command")

    pub = rospy.Publisher('/control_robot', robot_control, queue_size=1)
    rospy.init_node('data_collector')

    data_collector = DataCollector(circles)

    rospy.Subscriber('/control_robot', robot_control, data_collector.control_callback)
    rospy.Subscriber('/gazebo/model_states', ModelStates, data_collector.position_callback)

    if circles != 0:
        try:
            data_collector.check_if_circle_finished()
        except rospy.ROSInterruptException:
            pass
    else:
        rospy.spin()
