#!/usr/bin/env python3
import subprocess
import sys
import time
import datetime
import numpy
from pathlib import Path


def run(args):
    p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=sys.stdout, stderr=subprocess.STDOUT,
            universal_newlines=True, bufsize=-1)
    return p


def record(speed, trials, direction, repetitions_per_speed, duration, pull_at_back, first_straight, circles, distance):
    start_time = datetime.datetime.now()

    for s in speed:
        for j in range(0, len(trials)):
            for d in direction:
                for i in range(0, repetitions_per_speed):
                    # s = trials[j]["speed"]
                    # d = trials[j]["dir"]
                    trial_name = trials[j]["name"] + str(s) + "s/" + str(d) + "dir/"
                    # trial_name = trials[j]["name"] + str(d) + "dir/"
                    # trials[j]["name"] + "/" + str(trials[j]["speed"]) + "s/" + str(trials[j]["dir"]) + "dir" + "/"  # trials[j]["name"] + str(s) + "s/" + str(d) + "dir/"  # trial_root_dir[j] + str(d) + "dir/"
                    print("TRIAL NAME: " + str(trial_name))

                    print("#############################")
                    print("stand up")
                    stand_up_exit_code = None
                    for k in range(0, 3):
                        stand_up_p = run(["rosrun", "walknet_curvewalking", "robot_controller.py", "_walk:=False"])
                        try:
                            stand_up_exit_code = stand_up_p.wait(10)
                        except subprocess.TimeoutExpired:
                            print("reset sim")
                            # sim_p = run(["rosservice", "call", "/gazebo/reset_simulation", "{}"])
                            sim_p = run(["rosrun", "walknet_curvewalking", "reset_model_pub.py"])
                            sim_exit_code = sim_p.wait(10)
                            if sim_exit_code != 0:
                                print("resetting simulation failed! This should not happen")
                                exit(1)
                            continue
                        if stand_up_exit_code == 0:
                            break
                    if stand_up_exit_code is None or stand_up_exit_code != 0:
                        print("standing up failed!")
                        exit(1)

                    print("reset sim")
                    # sim_p = run(["rosservice", "call", "/gazebo/reset_simulation", "{}"])
                    sim_p = run(["rosrun", "walknet_curvewalking", "reset_model_pub.py"])
                    sim_exit_code = sim_p.wait(10)
                    if sim_exit_code != 0:
                        print("resetting simulation failed! This should not happen")
                        exit(1)

                    print("initialize Robot")
                    if duration:
                        print("recorder: Robot Collector walk for {} minutes".format(duration))
                        print("aep_y = " + str(trials[j]["aep_y"]))
                        print("aep_x = " + str(trials[j]["aep_x"]))
                        robot_p = run(["rosrun", "walknet_curvewalking", "robot_controller.py", "_walk:=True",
                                       "_stepLength:=" + str(trials[j]["length"]),
                                       "_aepShift:=" + str(trials[j]["aep_y"]),
                                       "_aepShiftX:=" + str(trials[j]["aep_x"]),
                                       "_innerStep:=" + str(trials[j]["decrease"]), "_back:=" + str(pull_at_back),
                                       "_name:=" + trial_name, "_duration:=" + str(duration)])
                    else:
                        print("recorder: Robot Controller walk walk until stop")
                        robot_p = run(["rosrun", "walknet_curvewalking", "robot_controller.py", "_walk:=True",
                                       "_stepLength:=" + str(trials[j]["length"]),
                                       "_aepShift:=" + str(trials[j]["aep_y"]),
                                       "_aepShiftX:=" + str(trials[j]["aep_x"]),
                                       "_innerStep:=" + str(trials[j]["decrease"]), "_back:=" + str(pull_at_back),
                                       "_name:=" + trial_name, "_duration:=0"])

                    print("initialize Data Collector")
                    if circles:
                        print("recorder: Data Collector walk {} circles".format(circles))
                        data_collector_p = run(
                                ["rosrun", "walknet_curvewalking", "DataCollector.py", "_circles:=" + str(circles),
                                 "_distance:=0", "_name:=" + trial_name])
                    elif distance:
                        print("recorder: Data Collector walk {} meter".format(distance))
                        data_collector_p = run(
                                ["rosrun", "walknet_curvewalking", "DataCollector.py", "_circles:=0",
                                 "_distance:=" + str(distance), "_name:=" + trial_name])
                    else:
                        print("recorder: Data Collector walk walk until stop")
                        data_collector_p = run(
                                ["rosrun", "walknet_curvewalking", "DataCollector.py", "_circles:=0", "_distance:=0.0",
                                 "_name:=" + trial_name])

                    time.sleep(1)
                    print("start walking")
                    if first_straight:
                        run(["rosrun", "walknet_curvewalking", "robot_control_pub.py", "_speed:=" + str(s),
                             "_direction:=0.0"])
                        time.sleep(30)
                    run(["rosrun", "walknet_curvewalking", "robot_control_pub.py", "_speed:=" + str(s),
                         "_direction:=" + str(d)])

                    robot_p.wait()
                    data_collector_p.wait()

    end_time = datetime.datetime.now()
    recording_duration = end_time - start_time
    seconds_in_day = 24 * 60 * 60
    (minutes, seconds) = divmod(recording_duration.days * seconds_in_day + recording_duration.seconds, 60)
    print("recording took {} minutes and {} seconds".format(minutes, seconds))


def main_1():
    # test suit 1
    repetitions_per_speed = 3
    duration = 2  # 0.83
    circles = None
    distance = None

    pull_at_back = True
    aep_param = 0.025  # float('nan')  # 0.025
    root_dir = "logs/swing_stance_duration/"  # "logs/test_new_aep_shift/quadratic_x_shift/"  # "logs/curvewalk_improvements_with_increasing_mirrored_pull_at_back/all_velocities/"
    trials = [
        {"name": root_dir + "all_aep_xy_average_decreased_1cm/", "length": True,
         "aep_y": aep_param, "aep_x": aep_param, "decrease": 0.01},  # , "speed": 0.01, "dir": 0.8},
        # {"name": root_dir + "original/", "length": False, "aep_y": float('nan'), "aep_x": float('nan'), "decrease": 0.0}
        # {"name": root_dir + "all_aep_xy_average_decreased_0cm_changed_rules_more_with_angle/", "length": True, "aep_y": aep_param,
        #  "aep_x": aep_param, "decrease": 0.0}  # , "speed": 0.01, "dir": 0.8},
        # all aep_x
        # 0.01s 0.8dir
        # {"name": root_dir + "all_aep_xm2-3_-1.0cm_decreased_0cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": -0.01, "decrease": 0.0, "speed": 0.01, "dir": 0.8},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_0cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.0, "speed": 0.01, "dir": 0.8},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_0cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.0, "speed": 0.01, "dir": 0.8},
        # # 0.01s 1.4dir
        # {"name": root_dir + "all_aep_xm2-3_-1.0cm_decreased_0cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.01, "decrease": 0.0, "speed": 0.01, "dir": 1.4},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_0cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.0, "speed": 0.01, "dir": 1.4},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_0cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.0, "speed": 0.01, "dir": 1.4},
        # # 0.02s 0.6dir
        # {"name": root_dir + "all_aep_xm2-3_-0.5cm_decreased_0.25cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": -0.005, "decrease": 0.0025, "speed": 0.02, "dir": 0.6},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_0.25cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.0025, "speed": 0.02, "dir": 0.6},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_0.25cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.0025, "speed": 0.02, "dir": 0.6},
        # # 0.02s 1.0dir
        # {"name": root_dir + "all_aep_xm2-3_-0.5cm_decreased_0.25cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": -0.005, "decrease": 0.0025, "speed": 0.02, "dir": 1.0},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_0.25cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.0025, "speed": 0.02, "dir": 1.0},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_0.25cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.0025, "speed": 0.02, "dir": 1.0},
        # # 0.02s 1.4dir
        # {"name": root_dir + "all_aep_xm2-3_-0.5cm_decreased_0.25cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": -0.005, "decrease": 0.0025, "speed": 0.02, "dir": 1.4},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_0.25cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.0025, "speed": 0.02, "dir": 1.4},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_0.25cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.0025, "speed": 0.02, "dir": 1.4},
        # # 0.03s 0.8dir
        # {"name": root_dir + "all_aep_xm2-3_-1.0cm_decreased_0.5cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": -0.01, "decrease": 0.005, "speed": 0.03, "dir": 0.8},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_0.5cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.005, "speed": 0.03, "dir": 0.8},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_0.5cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.005, "speed": 0.03, "dir": 0.8},
        # # 0.03s 1.0dir
        # {"name": root_dir + "all_aep_xm2-3_-1.0cm_decreased_0.5cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": -0.01, "decrease": 0.005, "speed": 0.03, "dir": 1.0},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_0.5cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.005, "speed": 0.03, "dir": 1.0},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_0.5cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.005, "speed": 0.03, "dir": 1.0},
        # # 0.04s 0.8dir
        # {"name": root_dir + "all_aep_xm2-3_-0.25cm_decreased_0.75cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": -0.0025, "decrease": 0.0075, "speed": 0.04, "dir": 0.8},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_0.75cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.0075, "speed": 0.04, "dir": 0.8},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_0.75cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.0075, "speed": 0.04, "dir": 0.8},
        # # 0.04s 1.0dir
        # {"name": root_dir + "all_aep_xm2-3_-0.25cm_decreased_0.75cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": -0.0025, "decrease": 0.0075, "speed": 0.04, "dir": 1.0},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_0.75cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.0075, "speed": 0.04, "dir": 1.0},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_0.75cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.0075, "speed": 0.04, "dir": 1.0},
        # # 0.04s 1.2dir
        # {"name": root_dir + "all_aep_xm2-3_-0.25cm_decreased_0.75cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": -0.0025, "decrease": 0.0075, "speed": 0.04, "dir": 1.2},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_0.75cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.0075, "speed": 0.04, "dir": 1.2},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_0.75cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.0075, "speed": 0.04, "dir": 1.2},
        # # 0.06s 0.4dir
        # {"name": root_dir + "all_aep_xm2-3_3.0cm_decreased_1.25cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.03, "decrease": 0.0125, "speed": 0.06, "dir": 0.4},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_1.25cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.0125, "speed": 0.06, "dir": 0.4},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_1.25cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.0125, "speed": 0.06, "dir": 0.4},
        # # 0.06s 1.57dir
        # {"name": root_dir + "all_aep_xm2-3_3.0cm_decreased_1.25cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.03, "decrease": 0.0125, "speed": 0.06, "dir": 1.57},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_1.25cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.0125, "speed": 0.06, "dir": 1.57},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_1.25cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.0125, "speed": 0.06, "dir": 1.57},
        # # 0.07s 0.2dir
        # {"name": root_dir + "all_aep_xm2-3_3.5cm_decreased_1.5cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.035, "decrease": 0.015, "speed": 0.07, "dir": 0.2},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_1.5cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.015, "speed": 0.07, "dir": 0.2},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_1.5cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.015, "speed": 0.07, "dir": 0.2},
        # # 0.07s 0.4dir
        # {"name": root_dir + "all_aep_xm2-3_3.5cm_decreased_1.5cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.035, "decrease": 0.015, "speed": 0.07, "dir": 0.4},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_1.5cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.015, "speed": 0.07, "dir": 0.4},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_1.5cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.015, "speed": 0.07, "dir": 0.4},
        # # 0.07s 1.57dir
        # {"name": root_dir + "all_aep_xm2-3_3.5cm_decreased_1.5cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.035, "decrease": 0.015, "speed": 0.07, "dir": 1.57},
        # {"name": root_dir + "all_aep_xm2-3_0.0cm_decreased_1.5cm/", "length": True, "aep_y": float('nan'),
        #  "aep_x": 0.0, "decrease": 0.015, "speed": 0.07, "dir": 1.57},
        # {"name": root_dir + "all_aep_xy_xm2-3_0.0cm_decreased_1.5cm/", "length": True, "aep_y": 0.0,
        #  "aep_x": 0.0, "decrease": 0.015, "speed": 0.07, "dir": 1.57},
        # all aep_xy
        # {"name": root_dir + "all_aep-xy_2.5cm_xm2-3_decreased_1.0cm/", "length": True, "aep_y": aep_param,
        # "aep_x": aep_param, "decrease": 0.01},
        # all aep_y
        # {"name": root_dir + "all_aep-y_2.5cm_decreased_1.0cm/", "length": True, "aep_y": aep_param, "aep_x": 0.0,
        #  "decrease": 0.01},
        # step length aep_xy
        # {"name": root_dir + "step_length_aep-xy_2.5cm_xm2-3/", "length": True, "aep_y": aep_param, "aep_x": aep_param,
        #  "decrease": 0.0},
        # step length aep_y
        # {"name": root_dir + "step_length_aep-y_2.5cm/", "length": True, "aep_y": aep_param, "aep_x": 0.0,
        # "decrease": 0.0},
        # step length aep_x
        # {"name": root_dir + "step_length_aep-xm2-3_2.5cm/", "length": True, "aep_y": 0.0, "aep_x": aep_param,
        #  "decrease": 0.0},
        # all aep_x decreased 0.5cm
        # {"name": root_dir + "all_aep-xm2-3_2.5cm_decreased_0.5cm/", "length": True, "aep_y": 0.0, "aep_x": aep_param,
        #  "decrease": 0.005},
        # all aep_x decreased 1.5cm
        # {"name": root_dir + "all_aep-xm2-3_2.5cm_decreased_1.5cm/", "length": True, "aep_y": 0.0, "aep_x": aep_param,
        #  "decrease": 0.015},
        # all aep_x decreased 2.0cm
        # {"name": root_dir + "all_aep-xm2-3_2.5cm_decreased_2.0cm/", "length": True, "aep_y": 0.0, "aep_x": aep_param,
        # "decrease": 0.020},
        # all aep_x 5cm decreased 0.5cm
        # {"name": root_dir + "all_aep-xm2-3_5.0cm_decreased_0.5cm/", "length": True, "aep_y": 0.0, "aep_x": 0.05,
        #  "decrease": 0.005},
        # all aep_x 5cm decreased 1.5cm
        # {"name": root_dir + "all_aep-xm2-3_5.0cm_decreased_1.5cm/", "length": True, "aep_y": 0.0, "aep_x": 0.05,
        #  "decrease": 0.015},
        # all aep_x 5cm decreased 2.0cm
        # {"name": root_dir + "all_aep-xm2-3_5.0cm_decreased_2.0cm/", "length": True, "aep_y": 0.0, "aep_x": 0.05,
        #  "decrease": 0.020},
        # step length decreased
        # {"name": root_dir + "step_length_decreased_1cm/", "length": True, "aep_y": 0.0, "aep_x": 0.0,
        #  "decrease": 0.01},
        # step length
        # {"name": root_dir + "step_length/", "length": True, "aep_y": 0.0, "aep_x": 0.0, "decrease": 0.0},
        # aep_xy decreased
        # {"name": root_dir + "aep-xy_xm2-3_decreased_1cm/", "length": False, "aep_y": aep_param, "aep_x": aep_param,
        #  "decrease": 0.01},
        # aep_y decreased
        # {"name": root_dir + "aep-y_decreased_1cm/", "length": False, "aep_y": aep_param, "aep_x": 0.0,
        # "decrease": 0.01},
        # aep_x decreased
        # {"name": root_dir + "aep-xm2-3_decreased_1cm/", "length": False, "aep_y": 0.0, "aep_x": aep_param,
        # "decrease": 0.01},
        # decreased
        # {"name": root_dir + "decreased_1cm/", "length": False, "aep_y": 0.0, "aep_x": 0.0, "decrease": 0.01},
        # aep_xy
        # {"name": root_dir + "aep-xy_xm2-3/", "length": False, "aep_y": aep_param, "aep_x": aep_param, "decrease": 0.0},
        # aep_y
        # {"name": root_dir + "aep-y/", "length": False, "aep_y": aep_param, "aep_x": 0.0, "decrease": 0.0},
        # aep_x
        # {"name": root_dir + "aep-xm2-3/", "length": False, "aep_y": 0.0, "aep_x": aep_param, "decrease": 0.0},
        # original approach
        # {"name": root_dir + "original_approach/", "length": False, "aep_y": 0.0, "aep_x": 0.0, "decrease": 0.0}
    ]

    # direction = [0.6, 0.8]
    # direction = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]
    # direction = [0.4, 0.6, 0.8, 1.0, 1.2]
    # direction.reverse()
    direction = [0.0]
    # direction = [0.6, 0.8]
    # direction = [0.0, 0.5, 1.0, 1.57]
    # speed = numpy.arange(0.02, 0.065, 0.01)
    # speed = [0.01, 0.03, 0.05, 0.07]
    speed = [0.02, 0.03, 0.04, 0.05, 0.06]
    # speed = [0.05]
    for s in speed:
        for trial in [dic["name"] for dic in trials]:
            for d in direction:
                Path(trial + str(s) + "s/" + str(d) + "dir/").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/position").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/stability").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/durations").mkdir(parents=True, exist_ok=True)

    # for trial in trials:
    #     Path(trial["name"] + "/" + str(trial["speed"]) + "s/" + str(trial["dir"]) + "dir" + "/").mkdir(parents=True, exist_ok=True)
    #     Path(trial["name"] + "/" + str(trial["speed"]) + "s/" + str(trial["dir"]) + "dir" + "/position").mkdir(parents=True, exist_ok=True)
    #     Path(trial["name"] + "/" + str(trial["speed"]) + "s/" + str(trial["dir"]) + "dir" + "/stability").mkdir(parents=True, exist_ok=True)
    record(speed, trials, direction, repetitions_per_speed, duration, pull_at_back, circles, distance)

    # ----------------------------------------------------------------------------------------------------------------
    # test suit 2
    repetitions_per_speed = 3
    duration = 2  # 0.83
    circles = None
    distance = None

    pull_at_back = True
    aep_param = 0.025  # float('nan')  # 0.025
    root_dir = "logs/swing_stance_duration/"  # "logs/test_new_aep_shift/quadratic_x_shift/"  # "logs/curvewalk_improvements_with_increasing_mirrored_pull_at_back/all_velocities/"
    trials = [
        {"name": root_dir + "all_aep_xy_average_decreased_1cm/", "length": True,
         "aep_y": aep_param, "aep_x": aep_param, "decrease": 0.01},  # , "speed": 0.01, "dir": 0.8},
    ]
    direction = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]
    speed = [0.03, 0.05]
    for s in speed:
        for trial in [dic["name"] for dic in trials]:
            for d in direction:
                Path(trial + str(s) + "s/" + str(d) + "dir/").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/position").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/stability").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/durations").mkdir(parents=True, exist_ok=True)

    record(speed, trials, direction, repetitions_per_speed, duration, pull_at_back, False, circles, distance)


def main_3():
    repetitions_per_speed = 3
    duration = 2  # 0.83
    circles = None
    distance = None

    pull_at_back = True
    aep_param = 0.025  # float('nan')  # 0.025
    root_dir = "logs/check_rules/tuned_for_0.06s/"  # "logs/check_rules/fixed_rules/"
    trials = [
        {"name": root_dir + "all_aep_xy_average_decre_0.0cm/", "length": True, "aep_y": aep_param, "aep_x": aep_param,
         "decrease": 0.0}
        # {"name": root_dir + "original/", "length": False, "aep_y": float('nan'), "aep_x": float('nan'), "decrease": 0.0}
    ]
    # direction = [0.0, 0.4, 0.8, 1.2, 1.57]
    direction = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]
    # direction = [0.8, 1.0, 1.2, 1.4, 1.57]
    # direction.reverse()
    # direction = [0.0]

    # speed = [0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01]
    # speed = [0.04]
    speed = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]
    # speed.reverse()
    for s in speed:
        for trial in [dic["name"] for dic in trials]:
            for d in direction:
                Path(trial + str(s) + "s/" + str(d) + "dir/").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/position").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/stability").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/durations").mkdir(parents=True, exist_ok=True)
    # for s in speed:
    # for trial in [dic["name"] for dic in trials]:
    #     for d in direction:
    #         Path(trial + str(d) + "dir/").mkdir(parents=True, exist_ok=True)
    #         Path(trial + str(d) + "dir/position").mkdir(parents=True, exist_ok=True)
    #         Path(trial + str(d) + "dir/stability").mkdir(parents=True, exist_ok=True)
    #         Path(trial + str(d) + "dir/durations").mkdir(parents=True, exist_ok=True)

    record(speed, trials, direction, repetitions_per_speed, duration, pull_at_back, False, circles, distance)


def main():
    repetitions_per_speed = 3
    duration = 2  # 0.83
    circles = None
    distance = None

    pull_at_back = True
    first_straight = True
    if first_straight:
        duration += 0.5
    aep_param = 0.025  # float('nan')  # 0.025
    root_dir = "logs/check_rules/first_straight/"  # "logs/check_rules/fixed_rules/"
    trials = [
        {"name": root_dir + "all_aep_xy_average_decre_func_FIXED_stanceDiff/", "length": True, "aep_y": aep_param,
         "aep_x": aep_param,
         "decrease": 0.01},
        # {"name": root_dir + "new_all_aep_xy_average_decre_0.0cm/", "length": True, "aep_y": aep_param, "aep_x": aep_param,
        #  "decrease": 0.0}
        # {"name": root_dir + "original/", "length": False, "aep_y": float('nan'), "aep_x": float('nan'), "decrease": 0.0}
    ]
    # direction = [0.0, 0.4, 0.8, 1.2, 1.57]
    direction = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]
    # direction = [1.0, 1.2, 1.4, 1.57]
    # direction.reverse()
    # direction = [0.4, 0.6, 0.8]

    speed = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
    # speed = [0.06]
    # speed = [0.06]
    # speed.reverse()
    for s in speed:
        for trial in [dic["name"] for dic in trials]:
            for d in direction:
                Path(trial + str(s) + "s/" + str(d) + "dir/").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/position").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/stability").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/durations").mkdir(parents=True, exist_ok=True)

    record(speed, trials, direction, repetitions_per_speed, duration, pull_at_back, first_straight, circles, distance)

    repetitions_per_speed = 3
    duration = 2  # 0.83
    circles = None
    distance = None

    pull_at_back = True
    first_straight = False
    if first_straight:
        duration += 0.5
    aep_param = 0.025  # float('nan')  # 0.025
    root_dir = "logs/check_rules/first_straight/"  # "logs/check_rules/fixed_rules/"
    trials = [
        {"name": root_dir + "all_aep_xy_average_decre_func_FIXED_stanceDiff_not_first_straight/", "length": True,
         "aep_y": aep_param, "aep_x": aep_param, "decrease": 0.01},
        # {"name": root_dir + "new_all_aep_xy_average_decre_0.0cm/", "length": True, "aep_y": aep_param, "aep_x": aep_param,
        #  "decrease": 0.0}
        # {"name": root_dir + "original/", "length": False, "aep_y": float('nan'), "aep_x": float('nan'), "decrease": 0.0}
    ]
    # direction = [0.0, 0.4, 0.8, 1.2, 1.57]
    direction = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]
    # direction = [1.0, 1.2, 1.4, 1.57]
    # direction.reverse()
    # direction = [0.4, 0.6, 0.8]

    speed = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
    # speed = [0.06]
    # speed = [0.06]
    # speed.reverse()
    for s in speed:
        for trial in [dic["name"] for dic in trials]:
            for d in direction:
                Path(trial + str(s) + "s/" + str(d) + "dir/").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/position").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/stability").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/durations").mkdir(parents=True, exist_ok=True)

    record(speed, trials, direction, repetitions_per_speed, duration, pull_at_back, first_straight, circles, distance)


if __name__ == "__main__":
    main()
