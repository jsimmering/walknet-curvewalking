#!/usr/bin/env python3
import datetime

import rospy
from gazebo_msgs.msg import ModelStates
from walknet_curvewalking.msg import robot_control


class DataCollector:
    def __init__(self):
        self.walking = False
        self.init_stability = False
        #self.init_position = False

        time = datetime.datetime.now()
        self.file_name = "logs/walknet_position_" + str(time.month) + "-" + str(time.day) + "_" + str(time.hour) + \
                         "-" + str(time.minute) + "-" + str(time.second)
        print("DATA COLLECTOR MODEL POSITION NAME: ", self.file_name)
        with open(self.file_name, "a") as f_handle:
            f_handle.write(
                    "time;point x;point y;point z;quaternion x;quaternion y;quaternion z;quaternion w;linear x;linear y;linear z;angular x;angular y;angular z\n")

    def control_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " control_callback heard %s", data)
        if data.speed_fact > 0:
            self.walking = True
        else:
            self.walking = False

    def stability_callback(self, data):
        if not self.init_stability:
            rospy.loginfo(rospy.get_caller_id() + " stability_callback heard %s", data)
            self.init_stability = True
        if self.walking:
            rospy.loginfo(rospy.get_caller_id() + " stability_callback heard %s", data)
            # with open(self.file_name, 'ba') as f_handle:
            # Save an array to a text file.
            # np.savetxt(f_handle, np.atleast_2d(mu_activation_data), fmt='%1.3f')

    def position_callback(self, data):
        # if not self.init_position:
        #     # rospy.loginfo(rospy.get_caller_id() + " position_callback heard pose: %s and twist: %s",
        #     #        data.pose[data.name.index('phantomx')], data.twist[data.name.index('phantomx')])
        #     self.write_positiion_to_file#(data)
        #     self.init_position = True
        if self.walking:
            # rospy.loginfo(rospy.get_caller_id() + " position_callback heard pose: %s and twist: %s",
            #        data.pose[data.name.index('phantomx')], data.twist[data.name.index('phantomx')])
            self.write_positiion_to_file(data)

    def write_positiion_to_file(self, data):
        with open(self.file_name, 'a') as f_handle:
            f_handle.write(
                    "{time};{point_x};{point_y};{point_z};{quaternion_x};{quaternion_y};{quaternion_z};{quaternion_w};{linear_x};{linear_y};{linear_z};{angular_x};{angular_y};{angular_z}\n".format(
                            time=rospy.Time.now().to_sec(),
                            point_x=data.pose[data.name.index('phantomx')].position.x,
                            point_y=data.pose[data.name.index('phantomx')].position.y,
                            point_z=data.pose[data.name.index('phantomx')].position.z,
                            quaternion_x=data.pose[data.name.index('phantomx')].orientation.x,
                            quaternion_y=data.pose[data.name.index('phantomx')].orientation.y,
                            quaternion_z=data.pose[data.name.index('phantomx')].orientation.z,
                            quaternion_w=data.pose[data.name.index('phantomx')].orientation.w,
                            linear_x=data.twist[data.name.index('phantomx')].linear.x,
                            linear_y=data.twist[data.name.index('phantomx')].linear.y,
                            linear_z=data.twist[data.name.index('phantomx')].linear.z,
                            angular_x=data.twist[data.name.index('phantomx')].angular.x,
                            angular_y=data.twist[data.name.index('phantomx')].angular.y,
                            angular_z=data.twist[data.name.index('phantomx')].angular.z))


if __name__ == '__main__':
    rospy.init_node('data_collector')

    data_collector = DataCollector()

    rospy.Subscriber('/control_robot', robot_control, data_collector.control_callback)
    # rospy.Subscriber('/com', Marker, data_collector.stability_callback)
    rospy.Subscriber('/gazebo/model_states', ModelStates, data_collector.position_callback)

    rospy.spin()
