import rospy


class SimpleSwingTrajectoryGen:
    def __init__(self, leg):
        self.leg = leg
        self.start_point = [0, 0, 0]
        self.target_point = [0, 0, 0]
        self.swing_height = 0.5
        self.trajectory = []
        self.point = 0
        self.inSwing = False
        self.set_angles = None

    def set_start_point(self, start_point):
        self.start_point = start_point

    def set_target_point(self, target_point):
        self.target_point = target_point

    def compute_trajectory_points(self):
        self.trajectory.append(self.start_point)
        middle_point = [(self.start_point[0] + self.target_point[0]) / 2,
                        (self.start_point[1] + self.target_point[1]) / 2,
                        (self.start_point[2] + self.target_point[2]) / 2]
        middle_point[2] += self.swing_height
        control_point1 = [(self.start_point[0] + middle_point[0]) / 2, (self.start_point[1] + middle_point[1]) / 2,
                          (self.start_point[2] + middle_point[2]) / 2]
        control_point2 = [(self.target_point[0] + middle_point[0]) / 2, (self.target_point[1] + middle_point[1]) / 2,
                          (self.target_point[2] + middle_point[2]) / 2]
        self.trajectory.append(control_point1)
        self.trajectory.append(middle_point)
        self.trajectory.append(control_point2)
        self.trajectory.append(self.target_point)

    def move_to_next_point(self):
        if not self.leg.is_ready():
            rospy.loginfo("haven't received Joint values yet! skipp")
            return
        if self.leg.get_current_targets() is None:
            # TODO always true?
            rospy.loginfo("haven't started Swing")
            self.inSwing = True
        if self.check_if_current_target_is_set():
            rospy.loginfo("target is set")
            if not self.check_if_current_point_reached():
                rospy.loginfo("target is set but not reached got target: " + str(
                    self.set_angles) + " and current angles are: " + str(self.leg.get_current_targets) + ". skipp")
                return
            rospy.loginfo("set next point as target")
            self.point += 1
        if self.point >= len(self.trajectory):
            self.point = 0
            self.trajectory = []
            self.inSwing = False
            rospy.loginfo("finished trajectory")
            return
        else:
            self.inSwing = True
        target = self.trajectory[self.point]
        # get current angles
        current_angles = self.leg.get_current_angles()
        next_angles = self.leg.compute_inverse_kinematics(target)
        self.leg.set_command(next_angles)
        self.set_angles = next_angles

    def check_if_current_point_reached(self):
        current_pos = self.leg.ee_position
        current_target = self.trajectory[self.point]
        return abs(current_pos[0] - current_target[0]) < 0.05 and abs(
            current_pos[1] - current_target[1]) < 0.05 and abs(current_pos[2] - current_target[2]) < 0.05

    def check_if_current_target_is_set(self):
        set_targets = self.leg.get_current_targets()
        if self.set_angles is None or set_targets is None:
            return False
        rospy.loginfo("targets: " + str(type(set_targets)))
        return self.set_angles.item(0) == set_targets[0] and self.set_angles.item(0) == set_targets[
            1] and self.set_angles.item(0) == set_targets[2]
