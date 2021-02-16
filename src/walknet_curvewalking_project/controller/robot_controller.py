#!/usr/bin/env python3

import rospy
from walknet_curvewalking.msg import robot_control

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
from walknet_curvewalking_project.controller.single_leg_controller import SingleLegController
from walknet_curvewalking_project.phantomx.mmcBodyModel3D import mmcBodyModelStance


class RobotController:
    def __init__(self, name, note_handle):
        self.debug = True

        self.nh = note_handle
        self.name = name
        self.walk_motivation = False
        self.legs = []
        self.body_model = mmcBodyModelStance(self)
        for name in RSTATIC.leg_names:
            swing = False
            if name == 'rm' or name == 'lf' or name == 'lr':
                #swing = True
                self.legs.append(SingleLegController(name, self.nh, swing, self))
            if name == 'lm' or name == 'rf' or name == 'rr':
                swing = True
                self.legs.append(SingleLegController(name, self.nh, swing, self))
        self.control_robot_sub = rospy.Subscriber('/control_robot', robot_control, self.control_robot_callback)

    def move_legs_into_init_pos(self):
        rate = rospy.Rate(RSTATIC.controller_frequency)
        for leg in self.legs:
            while not leg.leg.is_ready() and not rospy.is_shutdown():
                rospy.loginfo("leg not connected yet! wait...")
                rate.sleep()
        rospy.loginfo("legs connected move to init pos")
        for leg in self.legs:
            if leg.swing and (leg.name == "lf" or leg.name == "rf"):
                init_pos = RSTATIC.front_initial_pep.copy()
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)
            elif not leg.swing and (leg.name == "lf" or leg.name == "rf"):
                init_pos = RSTATIC.front_initial_aep.copy()
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)
            elif leg.swing and (leg.name == "lm" or leg.name == "rm"):
                init_pos = RSTATIC.middle_initial_pep.copy()
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)
            elif not leg.swing and (leg.name == "lm" or leg.name == "rm"):
                init_pos = RSTATIC.middle_initial_aep.copy()
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)
            elif leg.swing and (leg.name == "lr" or leg.name == "rr"):
                init_pos = RSTATIC.hind_initial_pep.copy()
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)
            elif not leg.swing and (leg.name == "lr" or leg.name == "rr"):
                init_pos = RSTATIC.hind_initial_aep.copy()
                init_pos[1] = init_pos[1] * leg.movement_dir
                leg.set_init_pos(init_pos)

            if not leg.init_pos is None:
                leg.move_leg_to(leg.init_pos)
                rate.sleep()

        finished = False
        while not rospy.is_shutdown() and not finished:
            finished = True
            for leg in self.legs:
                if not leg.leg.is_target_reached():
                    finished = False
                    leg.move_leg_to()
                    rate.sleep()
        rospy.loginfo("reached init positions")

    def control_robot_callback(self, data):
        if data.speed_fact > 0:
            self.body_model.pullBodyModelAtFrontIntoRelativeDirection(data.pull_angle, data.speed_fact)
            self.body_model.pullBodyModelAtBackIntoRelativeDirection(0, 0)
            for leg in self.legs:
                leg.set_delay_1b(data.speed_fact)
            self.walk_motivation = True

    def init_body_model(self):
        for leg in self.legs:
            self.body_model.put_leg_on_ground(leg.name,
                leg.leg.ee_position() - leg.leg.apply_c1_static_transform())
            rospy.loginfo("BODY MODEL LEG INIT: " + str(leg.name) + " ee:pos: " + str(leg.leg.ee_position()))
        self.body_model.updateLegStates()

    # Update all the leg networks.
    # Main Processing Step?
    def updateStanceBodyModel(self):
        if self.debug:
            mleg = self.legs[4]
            print("GC: ", mleg.leg.predicted_ground_contact(), " - ", mleg.leg.ee_position()[2])
            print("SWING: ", mleg.swing)
            # input()

        self.body_model.updateLegStates()

        for i in range(0, 6):
            # if (self.motivationNetLegs[i].swing_motivation.output_value > 0.5):
            if self.legs[i].swing:
                self.body_model.lift_leg_from_ground(i)

        self.body_model.mmc_iteration_step()

    def walk_body_model(self):
        rate = rospy.Rate(RSTATIC.controller_frequency)
        ready_status = [leg.leg.is_ready() for leg in self.legs]
        rospy.loginfo("ready status = " + str(ready_status))
        while ready_status.__contains__(False):
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
            ready_status = [leg.leg.is_ready() for leg in self.legs]
            rospy.loginfo("ready status = " + str(ready_status))
        while not rospy.is_shutdown():
            self.updateStanceBodyModel()
            for leg in self.legs:
                if rospy.is_shutdown():
                    break
                # input("press any key to performe the next step.")
                leg.manage_walk()
            rate.sleep()

    def move_body_cohesive(self):
        rate = rospy.Rate(RSTATIC.controller_frequency)
        ready_status = [leg.leg.is_ready() for leg in self.legs]
        rospy.loginfo("ready status = " + str(ready_status))
        while ready_status.__contains__(False):
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
            ready_status = [leg.leg.is_ready() for leg in self.legs]
            rospy.loginfo("ready status = " + str(ready_status))
        leg_status = [not leg.swing for leg in self.legs]
        rospy.loginfo("leg_status = " + str(leg_status))
        while not rospy.is_shutdown() and not leg_status.__contains__(False):
            self.updateStanceBodyModel()
            for leg in self.legs.reverse():
                # input("press any key to performe the next step.")
                leg.manage_stance()
            leg_status = [not leg.swing for leg in self.legs]
            rospy.loginfo("leg_status = " + str(leg_status))


if __name__ == '__main__':
    nh = rospy.init_node('robot_controller', anonymous=True)
    robot_controller = RobotController('robot', nh)
    # rospy.spin()
    try:
        robot_controller.move_legs_into_init_pos()
        #robot_controller.move_body_cohesive()
        robot_controller.walk_body_model()
    except rospy.ROSInterruptException:
        pass
