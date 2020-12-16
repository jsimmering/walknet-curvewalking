import rospy


class SimpleSwingTrajectoryGen:
    def __init__(self, leg):
        self.leg = leg
        self.start_point = None
        self.target_point = None
        self.mid_point = None
        self.swing_height = -0.05
        self.trajectory = []
        self.point = 0
        self.inSwing = False
        self.finished_Swing = False
        self.set_angles = None

    def set_start_point(self, start_point):
        self.start_point = start_point
        rospy.loginfo('in set start_point: ' + str(self.start_point))

    def set_target_point(self, target_point):
        self.target_point = target_point
        rospy.loginfo('in set target_point: ' + str(self.target_point))

    def set_mid_point(self, mid_point):
        self.mid_point = mid_point
        rospy.loginfo('in set mid_point: ' + str(self.mid_point))

    def is_finished(self):
        return self.finished_Swing

    def compute_trajectory_points(self):
        if self.start_point is None or self.target_point is None:
            rospy.loginfo('start_point or target_point not set. Can not compute trajectory')
            return False
        self.trajectory.append(self.start_point)
        #middle_point = [0, 0.386, -0.055]
        # middle_point[2] += self.swing_height
        #control_point1 = [(self.start_point[0] + middle_point[0]) / 2, (self.start_point[1] + middle_point[1]) / 2,
        #    (self.start_point[2] + middle_point[2]) / 2]
        #control_point2 = [(self.target_point[0] + middle_point[0]) / 2, (self.target_point[1] + middle_point[1]) / 2,
        #    (self.target_point[2] + middle_point[2]) / 2]
        #self.trajectory.append(control_point1)
        self.trajectory.append(self.mid_point)
        #self.trajectory.append(control_point2)
        self.trajectory.append(self.target_point)
        return True

    def move_to_next_point(self):
        if not self.leg.is_ready():
            rospy.loginfo("haven't received Joint values yet! skipp")
            return
        if self.point >= len(self.trajectory):
            self.point = 0
            self.trajectory = []
            self.inSwing = False
            self.finished_Swing = True
            rospy.loginfo("finished trajectory")
            return
        else:
            self.inSwing = True
            self.finished_Swing = False
        if self.check_if_current_target_is_reached():
            rospy.loginfo("target is reached")
            # if not self.check_if_current_point_reached():
            # TODO targets set but not reached
            rospy.loginfo("set next point as target")
            self.point += 1
            if self.point >= len(self.trajectory):
                self.point = 0
                self.trajectory = []
                self.inSwing = False
                self.finished_Swing = True
                rospy.loginfo("finished trajectory")
                return
        else:
            rospy.loginfo(
                "target is not reached got target: " + str(self.set_angles) + " and current angles are: " + str(
                    self.leg.get_current_targets()) + ". skipp")
        target = self.trajectory[self.point]
        # get current angles
        rospy.loginfo('in move to next point; current target: ' + str(target))
        current_angles = self.leg.get_current_angles()
        next_angles = self.leg.compute_inverse_kinematics(target)
        rospy.loginfo('angles to reach current target: ' + str(next_angles))
        self.leg.set_command(next_angles)
        self.set_angles = next_angles
        rospy.loginfo('set_angles: ' + str(self.set_angles))

    def check_if_current_target_is_set(self):
        set_targets = self.leg.get_current_targets()
        if self.set_angles is None or set_targets is None:
            return False
        rospy.loginfo("current leg targets: " + str(set_targets))
        rospy.loginfo('current set angles = ' + str(self.set_angles))
        return self.set_angles.item(0) == set_targets[0] and self.set_angles.item(0) == set_targets[
            1] and self.set_angles.item(0) == set_targets[2]

    def check_if_current_target_is_reached(self):
        cur_target = self.trajectory[self.point]
        cur_ee = self.leg.ee_position()
        if abs(cur_target[0] - cur_ee[0]) < 0.02 and abs(cur_target[1] - cur_ee[1]) < 0.02 and abs(
                cur_target[2] - cur_ee[2]) < 0.02:
            return True
        else:
            rospy.loginfo('cur_target is ' + str(cur_target) + ' cur_ee is ' + str(cur_ee))
