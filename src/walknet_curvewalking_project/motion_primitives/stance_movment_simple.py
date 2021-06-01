#!/usr/bin/env python3
import rospy


class StanceMovementSimple:

    def __init__(self, leg):
        self.leg = leg
        self.start_point = None
        self.target_point = None
        self.finished_Stance = False

    def set_start_point(self, start_point):
        self.start_point = start_point
        rospy.loginfo('in set stance start_point: ' + str(self.start_point))

    def set_target_point(self, target_point):
        self.target_point = target_point
        rospy.loginfo('in set stance target_point: ' + str(self.target_point))

    def is_finished(self):
        return self.finished_Stance

    def stance(self):
        if not self.leg.is_ready():
            rospy.loginfo("haven't received Joint values yet! skipp")
            return
        if self.check_if_current_target_is_reached():
            self.finished_Stance = True
            rospy.loginfo("finished stance trajectory")
            return
        else:
            self.finished_Stance = False
        next_angles = self.leg.compute_inverse_kinematics(self.target_point)
        rospy.loginfo('angles to reach current target: ' + str(next_angles))
        self.leg.set_joint_point(next_angles)
        rospy.loginfo('set_angles: ' + str(next_angles))

    def check_if_current_target_is_reached(self):
        cur_target = self.target_point
        cur_ee = self.leg.ee_position()
        rospy.loginfo('cur_target is ' + str(cur_target) + ' cur_ee is ' + str(cur_ee))
        if abs(cur_target[0] - cur_ee[0]) < 0.02 and abs(cur_target[1] - cur_ee[1]) < 0.02 and abs(
                cur_target[2] - cur_ee[2]) < 0.02:
            return True
