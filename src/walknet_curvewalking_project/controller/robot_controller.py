#!/usr/bin/env python3
import datetime
import sys

import rospy
from walknet_curvewalking.msg import robot_control

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
import walknet_curvewalking_project.support.constants as CONST
from walknet_curvewalking_project.phantomx.Robot import Robot


class RobotController:
    def __init__(self, name, note_handle, duration=None):
        self.debug = False
        self.rate = rospy.Rate(RSTATIC.controller_frequency)
        self.rate_leg = rospy.Rate(RSTATIC.controller_frequency)
        self.rate_body = rospy.Rate(RSTATIC.controller_frequency)
        self.nh = note_handle

        self.name = name
        self.robot = Robot(self.name, self.nh)
        self.walk_motivation = False
        self.walk_start_time = None
        self.stance_speed = 0
        self.velocity = 0
        self.pull_angle = 0
        self.controller_steps = 0

        self.started = False
        self.walk_duration = duration

        self.control_robot_sub = rospy.Subscriber('/control_robot', robot_control, self.control_robot_callback)

    def move_legs_into_init_pos(self):
        for leg in self.robot.legs:
            while not leg.leg.is_ready() and not rospy.is_shutdown():
                rospy.loginfo("leg not connected yet! wait...")
                self.rate.sleep()
        rospy.loginfo("legs connected move to init pos")
        for leg in self.robot.legs:
            init_pos = RSTATIC.initial_pep[RSTATIC.leg_names.index(leg.name) // 2].copy()
            if leg.name == "lf" or leg.name == "rm" or leg.name == "lr":
                # init_pos = RSTATIC.front_initial_pep.copy()
                # init_pos[0] += (RSTATIC.default_stance_distance * 1)  # (1.0 / 4.0))
                init_pos[0] += (RSTATIC.default_stance_distance * (3.0 / 4.0))
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)
            elif leg.name == "rf" or leg.name == "lm" or leg.name == "rr":
                # init_pos = RSTATIC.front_initial_pep.copy()
                # init_pos[0] += (RSTATIC.default_stance_distance * 1)  # (1.0 / 4.0))
                init_pos[0] += (RSTATIC.default_stance_distance * (1.0 / 4.0))
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)

            if leg.init_pos is not None:
                leg.move_leg_to(leg.init_pos)
                self.rate.sleep()
            else:
                rospy.logerr("leg init pose not set leg = {} init pose = {}".format(leg.name, leg.init_pos))

        finished = False
        while not rospy.is_shutdown() and not finished:
            finished = True
            for leg in self.robot.legs:
                if not leg.leg.is_target_set() or not leg.leg.is_target_reached():
                    # rospy.logerr(leg.name + " : target set = " + str(leg.leg.is_target_set()) + " target reached = " +
                    #             str(leg.leg.is_target_reached()))
                    if not leg.leg.is_target_set():
                        rospy.logerr(leg.name + ": set targets: a={} b={} c={}; real targets: {}".format(
                                leg.leg.alpha_command, leg.leg.beta_command, leg.leg.gamma_command,
                                leg.leg.get_current_targets()))
                    finished = False
                    leg.move_leg_to()
                    self.rate.sleep()
        rospy.loginfo("reached init positions")

    def control_robot_callback(self, data):
        if data.speed_fact > 0:
            self.velocity = data.speed_fact
            self.pull_angle = data.pull_angle
            counter_damping_fact = (-109.5 * data.speed_fact + 0.0145 / data.speed_fact + 31.53)
            # counter_damping_fact = 27
            rospy.loginfo("COUNTER_DAMPING_FACTOR = " + str(counter_damping_fact))
            self.stance_speed = (data.speed_fact * counter_damping_fact) / RSTATIC.controller_frequency
            self.robot.body_model.pullBodyModelAtFrontIntoRelativeDirection(data.pull_angle, self.stance_speed)
            self.robot.body_model.pullBodyModelAtBackIntoRelativeDirection(0, 0)
            for leg in self.robot.legs:
                leg.set_pull_dependent_parameter(self.stance_speed, data.pull_angle)
            # if data.speed_fact * 10 > 0.70:
            # if data.speed_fact * 10 >= 0.60:
            #     CONST.DEFAULT_SWING_VELOCITY += 0.3
            rospy.loginfo("DEFAULT_SWING_VELOCITY = " + str(CONST.DEFAULT_SWING_VELOCITY))
            rospy.loginfo("STANCE SPEED = " + str(self.stance_speed))
            self.robot.stance_speed = data.speed_fact
            self.robot.direction = data.pull_angle
            self.robot.initialize_stability_data_file()
            self.walk_motivation = True
            self.walk_start_time = rospy.Time.now()
        else:
            self.walk_motivation = False
            self.robot.stance_speed = 0.0
            self.robot.direction = 0.0
            self.robot.body_model.pullBodyModelAtFrontIntoRelativeDirection(0, 0)
            self.robot.body_model.pullBodyModelAtBackIntoRelativeDirection(0, 0)
            self.robot.running = False
            # self.robot.write_all_stability_data_to_file()
        # self.update_stance_body_model(True)

    def init_body_model(self):
        for leg in self.robot.legs:
            self.robot.body_model.put_leg_on_ground(leg.name,
                    leg.leg.ee_position() - leg.leg.apply_c1_static_transform())
            rospy.loginfo("BODY MODEL LEG INIT: " + str(leg.name) + " ee:pos: " + str(leg.leg.ee_position()))
        self.robot.body_model.updateLegStates()

    # Update all the leg networks.
    # Main Processing Step
    def update_stance_body_model(self, reset_segments):
        if self.debug:
            mleg = self.robot.legs[4]
            print("GC: ", mleg.leg.predicted_ground_contact(), " - ", mleg.leg.ee_position()[2])
            print("SWING: ", mleg.swing)
            # input()

        self.robot.body_model.updateLegStates()

        # for i in range(0, 6):
        #     #if (self.motivationNetLegs[i].swing_motivation.output_value > 0.5):
        #     if self.robot.legs[i].swing:
        #         self.robot.body_model.lift_leg_from_ground(i)
        #         rospy.loginfo("lift leg " + str(i))

        self.robot.body_model.mmc_iteration_step(reset_segments)
        # self.robot.body_model.mmc_iteration_step_matrix(reset_segments)

    def walk_body_model(self):
        while not rospy.is_shutdown() and self.walk_motivation and self.robot.running:
            if self.walk_duration is not None and rospy.Time.now() - self.walk_start_time > self.walk_duration:
                self.robot.running = False
                # self.robot.write_all_stability_data_to_file()
            self.controller_steps += 1
            self.update_stance_body_model(False)
            legs_in_swing = self.robot.body_model.gc.count(False)
            for leg in reversed(self.robot.legs):
                if rospy.is_shutdown():
                    break
                # input("press any key to performe the next step.")
                legs_in_swing = leg.manage_walk(legs_in_swing)
            if not self.robot.check_stability():
                rospy.loginfo("gc ('lf', 'rf', 'lm', 'rm', 'lr', 'rr') = " + str(self.robot.body_model.gc))
            # self.robot.body_model.pullBodyModelAtFrontIntoRelativeDirection(self.pull_angle, self.stance_speed)
            self.rate.sleep()
        self.rate.sleep()

    def record_additional_data(self):
        file_name = "logs/walknet_"
        file_suffix = self.robot.file_suffix
        if file_suffix == "":
            time = datetime.datetime.now()
            file_suffix = str(RSTATIC.controller_frequency) + "hz_" + str(round(self.velocity, 4)) + "s_" + \
                          str(round(self.pull_angle, 3)) + "dir_on_" + str(time.month) + "-" + str(time.day) + \
                          "_till_" + str(time.hour) + "-" + str(time.minute) + "-" + str(time.second)
        print("DATA COLLECTOR MODEL ROBOT NAME: ", file_name + file_suffix)
        actual_duration = 0
        if self.walk_start_time:
            actual_duration = (rospy.Time.now() - self.walk_start_time).to_sec()
        valueError_count = "valueError_count: \n"
        for leg in self.robot.legs:
            valueError_count += leg.name + " " + str(leg.stance_net.valueError_count) + "\n"
        with open(file_name + file_suffix, "a") as f_handle:
            # leg_list = 'lf', 'lm', 'lr', 'rr', 'rm', 'rf'
            f_handle.write(
                    "controller frequency = {hz}\ndefault stance distance (length) = {step_length}\ndefault stance height = {height}\nstance width = {width}\npredicted ground contact = {gc_height}\nswing velocity = {swing}\nbm stance speed factor = {stance}\nset average velocity = {velocity}\npull angle = {angle}\nduration = {duration}\ncontroller steps = {cs}\nunstable_count = {unstable}\n".format(
                            hz=RSTATIC.controller_frequency, step_length=RSTATIC.default_stance_distance,
                            height=RSTATIC.stance_height, width=RSTATIC.default_stance_width,
                            gc_height=RSTATIC.predicted_ground_contact_height_factor,
                            swing=CONST.DEFAULT_SWING_VELOCITY, stance=self.stance_speed, velocity=self.velocity,
                            angle=self.pull_angle, duration=actual_duration, cs=self.controller_steps,
                            unstable=self.robot.unstable_count) + valueError_count, )


def talker():
    if not rospy.is_shutdown():
        msg = robot_control()
        msg.speed_fact = 0.0
        msg.pull_angle = 0.0
        rospy.loginfo("robot_controller publish msg: " + str(msg))
        pub.publish(msg)


if __name__ == '__main__':
    pub = rospy.Publisher('/control_robot', robot_control, queue_size=1)
    nh = rospy.init_node('robot_controller', anonymous=True)

    walk = rospy.get_param('~walk', True)

    duration = 0
    if rospy.has_param('~duration'):
        duration = rospy.get_param('~duration')

    if duration != 0:
        robot_controller = RobotController('robot', nh, rospy.Duration.from_sec(duration * 60))
        rospy.loginfo("Robot Controller: Walk for {} seconds.".format(duration * 60))
    else:
        robot_controller = RobotController('robot', nh)  # executing until stop command
        rospy.loginfo("Robot Controller: Walk until stop command")

    try:
        robot_controller.move_legs_into_init_pos()
        # robot_controller.move_body_cohesive()
        ready_status = [leg.leg.is_ready() for leg in robot_controller.robot.legs]
        rospy.loginfo("ready status = " + str(ready_status))
        while not rospy.is_shutdown() and ready_status.__contains__(False):
            rospy.loginfo("leg not connected yet! wait...")
            robot_controller.rate.sleep()
            ready_status = [leg.leg.is_ready() for leg in robot_controller.robot.legs]
            rospy.loginfo("ready status = " + str(ready_status))

        while not rospy.is_shutdown() and robot_controller.robot.running and walk:
            robot_controller.walk_body_model()

        talker()
        if robot_controller.walk_start_time:
            robot_controller.record_additional_data()
            rospy.loginfo("DURATION = " + str((rospy.Time.now() - robot_controller.walk_start_time).to_sec()))
        else:
            rospy.loginfo("DURATION = 0. No walk command received")
        rospy.loginfo("CONTROLLER STEPS = " + str(robot_controller.controller_steps))
    except rospy.ROSInterruptException:
        pass
