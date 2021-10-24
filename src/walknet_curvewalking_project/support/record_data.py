#!/usr/bin/env python3
import subprocess
import sys
import time
import datetime
import numpy
from pathlib import Path
from math import pi


def run(args):
    p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=sys.stdout, stderr=subprocess.STDOUT,
            universal_newlines=True, bufsize=-1)
    return p


def record(speed, trials, direction, repetitions_per_speed, duration, first_straight, circles, distance):
    start_time = datetime.datetime.now()

    for d in direction:
        for s in speed:
            for i in range(0, repetitions_per_speed):
                for j in range(0, len(trials)):
                    # for i in range(0, repetitions_per_speed):
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
                    print("trial = " + str(trials[j]))
                    print("_innerStep:=" + str(trials[j]["decrease"]))
                    if duration:
                        print("recorder: Robot Collector walk for {} minutes".format(duration))
                        print("aep_y = " + str(trials[j]["aep_y"]))
                        print("aep_x = " + str(trials[j]["aep_x"]))
                        robot_p = run(["rosrun", "walknet_curvewalking", "robot_controller.py", "_walk:=True",
                                       "_stepLength:=" + str(trials[j]["length"]),
                                       "_aepShift:=" + str(trials[j]["aep_y"]),
                                       "_aepShiftX:=" + str(trials[j]["aep_x"]),
                                       "_innerStep:=" + str(trials[j]["decrease"]), "_back:=" + str(trials[j]["pull_at_back"]),
                                       "_name:=" + trial_name, "_duration:=" + str(duration)])
                    else:
                        print("recorder: Robot Controller walk walk until stop")
                        robot_p = run(["rosrun", "walknet_curvewalking", "robot_controller.py", "_walk:=True",
                                       "_stepLength:=" + str(trials[j]["length"]),
                                       "_aepShift:=" + str(trials[j]["aep_y"]),
                                       "_aepShiftX:=" + str(trials[j]["aep_x"]),
                                       "_innerStep:=" + str(trials[j]["decrease"]), "_back:=" + str(trials[j]["pull_at_back"]),
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


def main():

    repetitions_per_speed = 6
    duration = 2  # 0.83
    circles = None # 7
    distance = None

    #pull_at_back = True
    first_straight = False
    if first_straight:
        duration += 0.5
    aep_param = True  # float('nan')  # 0.025
    root_dir = "logs/walknet_curve-walking_final_results/"  # "logs/widowX-final_results/"
    # "logs/original_walknet_straightwalking/tetrapod/"
    #"logs/test/" # "logs/new_pull_vector/"  # "logs/test_stopping/"
    # "logs/check_stability_enforcement/0.06s_1.0s/" # "logs/tuning/tuned_for_1.2dir/"  # "logs/check_rules/fixed_rules/"
    trials = [
        # {"name": root_dir + "all_aep_xy_average_tuned_decre/", "length": True, "aep_y": True, "aep_x": True,
        #  "decrease": True},
        # {"name": root_dir + "new_all_aep_xy_average_decre_0.0cm/", "length": True, "aep_y": True, "aep_x": True,
        #  "decrease": False}
        # {"name": root_dir + "original/", "length": False, "aep_y": False, "aep_x": False, "decrease": False}
        # {"name": root_dir + "original_with_pull_at_back/", "length": False, "aep_y": False, "aep_x": False, "decrease": False}
        # {"name": root_dir + "step_length/", "length": True, "aep_y": False, "aep_x": False, "decrease": False}
        # {"name": root_dir + "aep_shift/", "length": False, "aep_y": True, "aep_x": True, "decrease": False}
        {"name": root_dir + "decrease_step_length/", "length": True, "aep_y": False, "aep_x": False, "decrease": True, "pull_at_back": False}
    ]

    #direction = [0.0]
    # direction = [0.6]
    # direction = [1.4]
    direction = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]
    # direction = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]
    # direction = [1.2, 1.4, pi / 2]
    # direction = [1.0]  # , 0.4, 0.8, 1.2, pi/2]
    # direction = [0.0, 0.5, 1.0, pi/2]

    speed = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]  # 0.007
    # speed = [0.06]  # , 0.04, 0.03]
    # speed = [0.03]  # , 0.025, 0.015]

    for s in speed:
        for trial in [dic["name"] for dic in trials]:
            for d in direction:
                Path(trial + str(s) + "s/" + str(d) + "dir/").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/position").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/stability").mkdir(parents=True, exist_ok=True)
                Path(trial + str(s) + "s/" + str(d) + "dir/durations").mkdir(parents=True, exist_ok=True)

    record(speed, trials, direction, repetitions_per_speed, duration, first_straight, circles, distance)


if __name__ == "__main__":
    main()
