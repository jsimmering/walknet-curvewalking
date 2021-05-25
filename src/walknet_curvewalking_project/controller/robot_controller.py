#!/usr/bin/env python3
import datetime

import rospy
from walknet_curvewalking.msg import robot_control

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
import walknet_curvewalking_project.support.constants as CONST
from walknet_curvewalking_project.phantomx.Robot import Robot
from math import sin, cos, pi, pow


class RobotController:
    def __init__(self, name, note_handle, swing_legs, step_length, shift_aep, shift_aep_x, decrease_inner_stance,
                 back_pull, trial_name, walk_duration=None):
        self.rate = rospy.Rate(RSTATIC.controller_frequency)
        self.nh = note_handle
        self.name = name
        self.step_length = step_length
        self.shift_aep = shift_aep
        self.shift_aep_x = shift_aep_x
        self.decrease_inner_stance = decrease_inner_stance
        self.pull_at_back = back_pull
        self.robot = Robot(self.name, self.nh, step_length, shift_aep, shift_aep_x, decrease_inner_stance, trial_name)

        # only move body cohesively or actually walk (swig legs)
        self.walk = swing_legs
        # whether robot currently wants to move forward
        self.walk_motivation = False
        self.walk_start_time = None
        self.walk_duration = walk_duration

        self.control_robot_sub = rospy.Subscriber('/control_robot', robot_control, self.control_robot_callback)

        # variables saving additional information
        self.stance_speed = 0
        self.velocity = 0
        self.pull_angle = 0
        self.back_pull_length_factor = None
        self.controller_steps = 0
        self.counter_damping_fact = 0
        self.trial_name = trial_name

    def move_legs_into_init_pos(self):
        for leg in self.robot.legs:
            while not leg.leg.is_ready() and not rospy.is_shutdown():
                rospy.loginfo("leg not connected yet! wait...")
                self.rate.sleep()
        rospy.loginfo("legs connected move to init pos")
        for leg in self.robot.legs:
            init_pos = RSTATIC.initial_pep[RSTATIC.leg_names.index(leg.name) // 2].copy()
            # for backwards walking: switch plus to minus in all statements in if
            if not self.walk:
                init_pos[0] += (RSTATIC.default_stance_distance * 1)
            elif leg.name == "lf" or leg.name == "rm" or leg.name == "lr":
                init_pos[0] += (RSTATIC.default_stance_distance * (3.0 / 4.0))
            elif leg.name == "rf" or leg.name == "lm" or leg.name == "rr":
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
                    if not leg.leg.is_target_set():
                        rospy.logwarn(leg.name + ": set targets: a={} b={} c={}; real targets: {}".format(
                                leg.leg.alpha_command, leg.leg.beta_command, leg.leg.gamma_command,
                                leg.leg.get_current_targets()))
                    finished = False
                    leg.move_leg_to()
                    self.rate.sleep()
        rospy.loginfo("reached init positions")

    def initialize_body_model(self):
        self.robot.initialize_body_model()

        for leg in self.robot.legs:
            self.robot.body_model.put_leg_on_ground(leg.name, leg.leg.ee_position())
            #       # leg.leg.ee_position() - leg.leg.apply_c1_static_transform())
            rospy.loginfo("BODY MODEL LEG INIT: " + str(leg.name) + " ee:pos: " + str(leg.leg.ee_position()))
        self.robot.body_model.updateLegStates()

    def control_robot_callback(self, data):
        if data.speed_fact > 0:
            self.velocity = data.speed_fact
            self.pull_angle = data.pull_angle
            # self.counter_damping_fact = (-109.5 * self.velocity + 0.0145 / self.velocity + 31.53)
            # new linear:
            self.counter_damping_fact = (-141.5 * self.velocity) + 35.5

            self.stance_speed = (self.velocity * self.counter_damping_fact) / RSTATIC.controller_frequency
            # TODO figure out how if pulling at back can allow for rotation on the spot
            if self.pull_at_back:
                if 0.0 <= abs(self.pull_angle) < 1.0:
                    self.back_pull_length_factor = ((self.stance_speed / 2) * abs(self.pull_angle))
                else:
                    self.back_pull_length_factor = self.stance_speed / 2
                self.robot.body_model.pullBodyModelAtFrontIntoRelativeDirection(self.pull_angle,
                        self.stance_speed - self.back_pull_length_factor)
                # ((-pow(1 / 30, self.pull_angle) + 1) * (self.stance_speed / 2)))
                # (((2 * self.pull_angle) / pi) * (self.stance_speed / 2)))
                # ((self.stance_speed / 2) * sin(self.pull_angle)))  # / 2)
                self.robot.body_model.pullBodyModelAtBackIntoRelativeDirection(-self.pull_angle,
                        self.back_pull_length_factor)
                # (-pow(1 / 30, self.pull_angle) + 1) * (self.stance_speed / 2))
                # (((2 * self.pull_angle) / pi) * (self.stance_speed / 2)))
                # ((self.stance_speed / 2) * sin(self.pull_angle)))  # /2)
            else:
                self.robot.body_model.pullBodyModelAtFrontIntoRelativeDirection(self.pull_angle, self.stance_speed)
                self.robot.body_model.pullBodyModelAtBackIntoRelativeDirection(0, 0)
            for leg in self.robot.legs:
                leg.set_pull_dependent_parameter(self.stance_speed, self.pull_angle)
            self.robot.stance_speed = self.velocity
            self.robot.direction = self.pull_angle
            self.robot.initialize_stability_data_file()
            self.walk_motivation = True
            self.walk_start_time = rospy.Time.now()
            rospy.loginfo("COUNTER_DAMPING_FACTOR = " + str(self.counter_damping_fact))
            rospy.loginfo("DEFAULT_SWING_VELOCITY = " + str(CONST.DEFAULT_SWING_VELOCITY))
            rospy.loginfo("STANCE SPEED = " + str(self.stance_speed))
        # elif data.speed_fact < 0:
        #     self.velocity = data.speed_fact
        #     self.pull_angle = data.pull_angle
        #     # self.counter_damping_fact = (-109.5 * self.velocity + 0.0145 / self.velocity + 31.53)
        #     # new linear:
        #     self.counter_damping_fact = (-141.5 * self.velocity) + 35.5
        #
        #     self.stance_speed = (self.velocity * self.counter_damping_fact) / RSTATIC.controller_frequency
        #     self.robot.body_model.pullBodyModelAtFrontIntoRelativeDirection(0, 0)
        #     # in this case negative angle leads to backwards left circle (left legs inside legs)
        #     self.robot.body_model.pullBodyModelAtBackIntoRelativeDirection(self.pull_angle, -self.stance_speed)
        #     for leg in self.robot.legs:
        #         leg.set_pull_dependent_parameter(self.stance_speed, self.pull_angle)
        #     self.robot.stance_speed = self.velocity
        #     self.robot.direction = self.pull_angle
        #     self.robot.initialize_stability_data_file()
        #     self.walk_motivation = True
        #     self.walk_start_time = rospy.Time.now()
        #     rospy.loginfo("COUNTER_DAMPING_FACTOR = " + str(self.counter_damping_fact))
        #     rospy.loginfo("DEFAULT_SWING_VELOCITY = " + str(CONST.DEFAULT_SWING_VELOCITY))
        #     rospy.loginfo("STANCE SPEED = " + str(self.stance_speed))
        else:
            self.walk_motivation = False
            self.robot.stance_speed = 0.0
            self.robot.direction = 0.0
            self.robot.body_model.pullBodyModelAtFrontIntoRelativeDirection(0, 0)
            self.robot.body_model.pullBodyModelAtBackIntoRelativeDirection(0, 0)
            self.robot.running = False
            # self.robot.write_all_stability_data_to_file()

    # Update the body model state and perform next iteration step
    # Main Processing Step
    def update_stance_body_model(self):
        self.robot.body_model.updateLegStates()
        # for i in range(0, 6):
        #     #if (self.motivationNetLegs[i].swing_motivation.output_value > 0.5):
        #     if self.robot.legs[i].swing:
        #         self.robot.body_model.lift_leg_from_ground(i)
        #         rospy.loginfo("lift leg " + str(i))

        self.robot.body_model.mmc_iteration_step()

    def walk_body_model(self):
        while not rospy.is_shutdown() and self.walk_motivation and self.robot.running:
            if self.walk_duration is not None and rospy.Time.now() - self.walk_start_time > self.walk_duration:
                self.robot.running = False
                # self.robot.write_all_stability_data_to_file()
            self.controller_steps += 1
            self.update_stance_body_model()
            legs_in_swing = self.robot.body_model.gc.count(False)
            for leg in reversed(self.robot.legs):
                if rospy.is_shutdown():
                    break
                legs_in_swing = leg.manage_walk(legs_in_swing, swing)
            if not self.robot.check_stability():
                rospy.loginfo("gc ('lf', 'rf', 'lm', 'rm', 'lr', 'rr') = " + str(self.robot.body_model.gc))
            # if self.velocity > 0:
            if self.pull_at_back:
                self.robot.body_model.pullBodyModelAtFrontIntoRelativeDirection(self.pull_angle,
                        self.stance_speed / 2)
                self.robot.body_model.pullBodyModelAtBackIntoRelativeDirection(-self.pull_angle,
                        self.stance_speed / 2)
            else:
                self.robot.body_model.pullBodyModelAtFrontIntoRelativeDirection(self.pull_angle, self.stance_speed)
                self.robot.body_model.pullBodyModelAtBackIntoRelativeDirection(0, 0)
            # else:
            #     # for backwards walking
            #     self.robot.body_model.pullBodyModelAtBackIntoRelativeDirection(self.pull_angle, -self.stance_speed)
            # if self.controller_steps > 50:
            #    self.robot.running = False
            self.rate.sleep()
        self.rate.sleep()

    def record_additional_data(self):
        file_name = self.trial_name + "walknet"  # "logs/walknet_"
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
        value_error_count = "value_error_count: \n"
        swing_delay_count = "swing_delay_count: \n"
        for leg in self.robot.legs:
            value_error_count += leg.name + " " + str(leg.stance_net.valueError_count) + " = " + str(
                    (leg.stance_net.valueError_count * 100) / self.controller_steps) + "%\n"
            swing_delay_count += leg.name + " " + str(leg.swing_delays) + " = " + str(
                    (leg.swing_delays * 100) / self.controller_steps) + "%\n"
        with open(file_name + file_suffix, "a") as f_handle:
            # leg_list = 'lf', 'lm', 'lr', 'rr', 'rm', 'rf'
            f_handle.write(
                    ("controller frequency = {hz}\ndefault stance distance (length) = {step_length}\ndefault stance " +
                     "height = {height}\nstance width = {width}\npredicted ground contact = {gc_height}\nswing " +
                     "velocity = {swing}\ncounter_damping_fact = {factor}\nbm stance speed factor = {stance}\n" +
                     "set average velocity = {velocity}\npull angle = {angle}\nstep_length used as pep = " +
                     "{step_length_on}\naep shifted for curve = {shift_aep}\naep shifted in x dir for curve = " +
                     "{shift_aep_x}\ninner stance step decreased for curve by = {decrease_inner_stance}\nduration = " +
                     "{duration}\ncontroller steps = {cs}\nunstable_count = {unstable}\nunstable_percent = {percent}\n"
                     ).format(
                            hz=RSTATIC.controller_frequency, step_length=RSTATIC.default_stance_distance,
                            height=RSTATIC.stance_height, width=RSTATIC.default_stance_width,
                            gc_height=RSTATIC.predicted_ground_contact_height_factor,
                            swing=CONST.DEFAULT_SWING_VELOCITY,
                            factor=self.counter_damping_fact, stance=self.stance_speed, velocity=self.velocity,
                            angle=self.pull_angle, step_length_on=self.step_length, shift_aep=self.shift_aep,
                            shift_aep_x=self.shift_aep_x, decrease_inner_stance=self.decrease_inner_stance,
                            duration=actual_duration, cs=self.controller_steps, unstable=self.robot.unstable_count,
                            percent=(self.robot.unstable_count * 100) / self.controller_steps) +
                    value_error_count + swing_delay_count)


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

    # if the robot should wait for a controll command
    walk = rospy.get_param('~walk', True)
    # if the robot shoul swing it's legs or only move the body with the body model
    swing = rospy.get_param('~swing', True)
    # if the same vector should be applied to the back of the body model to turn on the spot
    pull_at_back_param = rospy.get_param('~back', True)

    duration = 0
    if rospy.has_param('~duration'):
        duration = rospy.get_param('~duration')

    step_length_param = rospy.get_param('~stepLength', 0.0)
    shift_aep_param = rospy.get_param('~aepShift', 0.0)
    shift_aep_x_param = rospy.get_param('~aepShiftX', 0.0)
    decrease_inner_stance_param = rospy.get_param('~innerStep', True)

    trial_name_param = rospy.get_param('~name', "logs/")

    if duration != 0:
        robot_controller = RobotController('robot', nh, swing, step_length_param, shift_aep_param, shift_aep_x_param,
                decrease_inner_stance_param, pull_at_back_param, trial_name_param,
                rospy.Duration.from_sec(duration * 60))
        rospy.loginfo("Robot Controller: Walk for {} seconds.".format(duration * 60))
    else:
        robot_controller = RobotController('robot', nh, swing, step_length_param, shift_aep_param, shift_aep_x_param,
                decrease_inner_stance_param, pull_at_back_param, trial_name_param)  # executing until stop command
        rospy.loginfo("Robot Controller: Walk until stop command")

    try:
        robot_controller.move_legs_into_init_pos()
        ready_status = [leg.leg.is_ready() for leg in robot_controller.robot.legs]
        rospy.loginfo("ready status = " + str(ready_status))
        while not rospy.is_shutdown() and ready_status.__contains__(False):
            robot_controller.rate.sleep()
            ready_status = [leg.leg.is_ready() for leg in robot_controller.robot.legs]
        rospy.loginfo("all legs ready! status = " + str(ready_status))
        robot_controller.initialize_body_model()
        while not rospy.is_shutdown() and robot_controller.robot.running and walk:
            robot_controller.walk_body_model()
        if robot_controller.robot.write_at_end:
            robot_controller.robot.write_all_stability_data_to_file()
        talker()
        if robot_controller.walk_start_time:
            robot_controller.record_additional_data()
            rospy.loginfo("DURATION = " + str((rospy.Time.now() - robot_controller.walk_start_time).to_sec()))
        else:
            rospy.loginfo("DURATION = 0. No walk command received")
        rospy.loginfo("CONTROLLER STEPS = " + str(robot_controller.controller_steps))
        # input()
    except rospy.ROSInterruptException:
        pass
