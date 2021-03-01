#!/usr/bin/env python3
import threading

import rospy
from walknet_curvewalking.msg import robot_control

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
from walknet_curvewalking_project.phantomx.Robot import Robot
from walknet_curvewalking_project.phantomx.mmcBodyModel3D import mmcBodyModelStance
from walknet_curvewalking_project.support import stability


class RobotController:
    def __init__(self, name, note_handle):
        self.debug = False
        self.rate = rospy.Rate(RSTATIC.controller_frequency)
        self.rate_leg = rospy.Rate(RSTATIC.controller_frequency)
        self.rate_body = rospy.Rate(RSTATIC.controller_frequency)
        self.nh = note_handle

        self.name = name
        self.robot = Robot(self.name, self.nh)
        self.walk_motivation = False
        # self.leg_threads_list = []
        self.started = False

        # for leg in self.legs:
        #     self.leg_threads_list.append(threading.Thread(target=leg.walking_thread, daemon=True))
        # self.th = threading.Thread(target=self.leg_thread, daemon=True)
        # self.th_body = threading.Thread(target=self.body_thread, daemon=True)
        self.control_robot_sub = rospy.Subscriber('/control_robot', robot_control, self.control_robot_callback)

    def move_legs_into_init_pos(self):
        for leg in self.robot.legs:
            while not leg.leg.is_ready() and not rospy.is_shutdown():
                rospy.loginfo("leg not connected yet! wait...")
                self.rate.sleep()
        rospy.loginfo("legs connected move to init pos")
        for leg in self.robot.legs:
            if leg.swing and (leg.name == "lf" or leg.name == "rf"):
                init_pos = RSTATIC.front_initial_pep.copy()
                # init_pos = (RSTATIC.front_initial_aep.copy() + RSTATIC.front_initial_pep.copy()) / 2.0
                init_pos[1] = init_pos[1] * leg.movement_dir
                # leg.swing = False
                # leg.last_stance_activation = rospy.Time.now()
                leg.set_init_pos(init_pos)
            elif not leg.swing and (leg.name == "lf" or leg.name == "rf"):
                init_pos = RSTATIC.front_initial_aep.copy()
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)
            elif leg.swing and (leg.name == "lm" or leg.name == "rm"):
                init_pos = RSTATIC.middle_initial_pep.copy()
                # init_pos = (RSTATIC.middle_initial_aep.copy() + RSTATIC.middle_initial_pep.copy()) / 2.0
                init_pos[1] = init_pos[1] * leg.movement_dir
                # leg.swing = False
                # leg.last_stance_activation = rospy.Time.now()
                leg.set_init_pos(init_pos)
            elif not leg.swing and (leg.name == "lm" or leg.name == "rm"):
                init_pos = RSTATIC.middle_initial_aep.copy()
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)
            elif leg.swing and (leg.name == "lr" or leg.name == "rr"):
                init_pos = RSTATIC.hind_initial_pep.copy()
                # init_pos = (RSTATIC.hind_initial_aep.copy() + RSTATIC.hind_initial_pep.copy()) / 2.0
                init_pos[1] = init_pos[1] * leg.movement_dir
                # leg.swing = False
                # leg.last_stance_activation = rospy.Time.now()
                leg.set_init_pos(init_pos)
            elif not leg.swing and (leg.name == "lr" or leg.name == "rr"):
                init_pos = RSTATIC.hind_initial_aep.copy()
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)

            if leg.init_pos is not None:
                leg.move_leg_to(leg.init_pos)
                self.rate.sleep()

        finished = False
        while not rospy.is_shutdown() and not finished:
            finished = True
            for leg in self.robot.legs:
                if not leg.leg.is_target_reached():
                    finished = False
                    leg.move_leg_to()
                    self.rate.sleep()
        rospy.loginfo("reached init positions")

    def leg_thread(self):
        # self.started = True
        while not rospy.is_shutdown() and self.walk_motivation:
            # self.updateStanceBodyModel()
            for leg in self.robot.legs:
                if rospy.is_shutdown():
                    break
                # input("press any key to performe the next step.")
                leg.manage_walk()
            self.rate_leg.sleep()

    def body_thread(self):
        # self.started = True
        while not rospy.is_shutdown() and self.walk_motivation:
            self.update_stance_body_model()
            self.rate_body.sleep()

    def leg_threads(self):
        # self.started = True
        # while not rospy.is_shutdown() and self.walk_motivation:
        # self.updateStanceBodyModel()
        for leg in self.leg_threads_list:
            if rospy.is_shutdown():
                break
            # input("press any key to performe the next step.")
            leg.start()
        # self.rate.sleep()

    def control_robot_callback(self, data):
        if data.speed_fact > 0:
            self.robot.body_model.pullBodyModelAtFrontIntoRelativeDirection(data.pull_angle, data.speed_fact)
            self.robot.body_model.pullBodyModelAtBackIntoRelativeDirection(0, 0)
            for leg in self.robot.legs:
                leg.set_delay_1b(data.speed_fact)
            self.walk_motivation = True
        else:
            self.walk_motivation = False

    def init_body_model(self):
        for leg in self.robot.legs:
            self.robot.body_model.put_leg_on_ground(leg.name,
                    leg.leg.ee_position() - leg.leg.apply_c1_static_transform())
            rospy.loginfo("BODY MODEL LEG INIT: " + str(leg.name) + " ee:pos: " + str(leg.leg.ee_position()))
        self.robot.body_model.updateLegStates()

    # Update all the leg networks.
    # Main Processing Step?
    def update_stance_body_model(self):
        if self.debug:
            mleg = self.robot.legs[4]
            print("GC: ", mleg.leg.predicted_ground_contact(), " - ", mleg.leg.ee_position()[2])
            print("SWING: ", mleg.swing)
            # input()

        self.robot.body_model.updateLegStates()

        for i in range(0, 6):
            # if (self.motivationNetLegs[i].swing_motivation.output_value > 0.5):
            if self.robot.legs[i].swing:
                self.robot.body_model.lift_leg_from_ground(i)
                #rospy.loginfo("lift leg " + str(i))

        self.robot.body_model.mmc_iteration_step()

    def walk_body_model(self):
        rate = rospy.Rate(RSTATIC.controller_frequency)
        ready_status = [leg.leg.is_ready() for leg in self.robot.legs]
        rospy.loginfo("ready status = " + str(ready_status))
        while not rospy.is_shutdown() and ready_status.__contains__(False):
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
            ready_status = [leg.leg.is_ready() for leg in self.robot.legs]
            rospy.loginfo("ready status = " + str(ready_status))
        while not rospy.is_shutdown() and self.walk_motivation:
            self.update_stance_body_model()
            self.robot.check_stability()
            for leg in reversed(self.robot.legs):
                if rospy.is_shutdown():
                    break
                # input("press any key to performe the next step.")
                leg.manage_walk()
            rate.sleep()

    def walk_body_model_two_threads(self):
        ready_status = [leg.leg.is_ready() for leg in self.robot.legs]
        rospy.loginfo("ready status = " + str(ready_status))
        while ready_status.__contains__(False):
            rospy.loginfo("leg not connected yet! wait...")
            self.rate.sleep()
            ready_status = [leg.leg.is_ready() for leg in self.robot.legs]
            rospy.loginfo("ready status = " + str(ready_status))

        while not rospy.is_shutdown():
            if not self.started and self.walk_motivation:
                self.started = True
                self.th.start()
                self.th_body.start()
            if self.started and not self.walk_motivation:
                # TODO check in thread if finished.
                pass
            # rospy.logwarn("robot controller updated body model. step = " + str(self.body_model.step))
            self.rate.sleep()

    def walk_body_model_7_threads(self):
        rate = rospy.Rate(RSTATIC.controller_frequency)
        ready_status = [leg.leg.is_ready() for leg in self.robot.legs]
        rospy.loginfo("ready status = " + str(ready_status))
        while ready_status.__contains__(False):
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
            ready_status = [leg.leg.is_ready() for leg in self.robot.legs]
            rospy.loginfo("ready status = " + str(ready_status))
        while not rospy.is_shutdown() and self.walk_motivation:
            self.update_stance_body_model()
            if not self.started:
                self.started = True
                self.leg_threads()
            # rospy.logwarn("robot controller updated body model. step = " + str(self.body_model.step))
            rate.sleep()

    def move_body_cohesive(self):
        rate = rospy.Rate(RSTATIC.controller_frequency)
        ready_status = [leg.leg.is_ready() for leg in self.robot.legs]
        rospy.loginfo("ready status = " + str(ready_status))
        while ready_status.__contains__(False):
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
            ready_status = [leg.leg.is_ready() for leg in self.robot.legs]
            rospy.loginfo("ready status = " + str(ready_status))
        leg_status = [not leg.swing for leg in self.robot.legs]
        rospy.loginfo("leg_status = " + str(leg_status))
        while not rospy.is_shutdown() and not leg_status.__contains__(False):
            self.update_stance_body_model()
            for leg in self.robot.legs.reverse():
                # input("press any key to performe the next step.")
                leg.manage_stance()
            leg_status = [not leg.swing for leg in self.robot.legs]
            rospy.loginfo("leg_status = " + str(leg_status))


if __name__ == '__main__':
    nh = rospy.init_node('robot_controller', anonymous=True)
    robot_controller = RobotController('robot', nh)
    try:
        robot_controller.move_legs_into_init_pos()
        # robot_controller.move_body_cohesive()
        while not rospy.is_shutdown():
            robot_controller.walk_body_model()
            # robot_controller.walk_body_model_two_threads()
            # robot_controller.walk_body_model_7_threads()
    except rospy.ROSInterruptException:
        pass
