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


def main():
    start_time = datetime.datetime.now()

    repetitions_per_speed = 3
    duration = 2
    circles = None
    distance = None

    pull_at_back = True
    root_dir = "logs/curvewalk_improvements_with_changing_pull_at_back/parameter_eval/decrease"
    trials = [{"name": root_dir + "all_aep-xy_xm2-3_decreased_1cm/", "length": True, "aep_y": True, "aep_x": True,
               "decrease": 0.01},
              {"name": root_dir + "all_aep-y_decreased_1cm/", "length": True, "aep_y": True, "aep_x": False,
               "decrease": 0.01},
              {"name": root_dir + "all_aep-xm2-3_decreased_1cm/", "length": True, "aep_y": False, "aep_x": True,
               "decrease": 0.01},
              {"name": root_dir + "step_length_decreased_1cm/", "length": True, "aep_y": False, "aep_x": False,
               "decrease": 0.01},
              {"name": root_dir + "step_length_aep-xy_xm2-3/", "length": True, "aep_y": True, "aep_x": True,
               "decrease": 0.0},
              {"name": root_dir + "step_length_aep-y/", "length": True, "aep_y": True, "aep_x": False, "decrease": 0.0},
              {"name": root_dir + "step_length_aep-xm2-3/", "length": True, "aep_y": False, "aep_x": True,
               "decrease": 0.0},
              {"name": root_dir + "step_length/", "length": True, "aep_y": False, "aep_x": False, "decrease": 0.0},
              # {"name": root_dir + "aep-xy_xm2-3_decreased_1cm/", "length": False, "aep_y": True, "aep_x": True, "decrease": 0.01},
              # {"name": root_dir + "aep-y_decreased_1cm/", "length": False, "aep_y": True, "aep_x": False, "decrease": 0.01},
              # {"name": root_dir + "aep-xm2-3_decreased_1cm/", "length": False, "aep_y": False, "aep_x": True, "decrease": 0.01},
              # {"name": root_dir + "decreased_1cm/", "length": False, "aep_y": False, "aep_x": False, "decrease": 0.01},
              # {"name": root_dir + "aep-xy_xm2-3/", "length": False, "aep_y": True, "aep_x": True, "decrease": 0.0},
              # {"name": root_dir + "aep-y/", "length": False, "aep_y": True, "aep_x": False, "decrease": 0.0},
              # {"name": root_dir + "aep-xm2-3/", "length": False, "aep_y": False, "aep_x": True, "decrease": 0.0},
              # {"name": root_dir + "original_approach/", "length": False, "aep_y": False, "aep_x": False, "decrease": 0.0}
              ]

    # direction = numpy.arange(0.10, 0.24, 0.05)
    # direction = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    # direction = [1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5]
    direction = [0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]
    for trial in [dic["name"] for dic in trials]:
        for d in direction:
            Path(trial + str(d) + "dir/").mkdir(parents=True, exist_ok=True)
            Path(trial + str(d) + "dir/position").mkdir(parents=True, exist_ok=True)
            Path(trial + str(d) + "dir/stability").mkdir(parents=True, exist_ok=True)

    # speed = numpy.arange(0.02, 0.065, 0.01)
    # speed = [0.007, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08]
    speed = [0.05]
    for s in speed:
        for j in range(0, len(trials)):
            for d in direction:
                for i in range(0, repetitions_per_speed):
                    trial_name = trials[j]["name"] + str(d) + "dir/"  # trial_root_dir[j] + str(d) + "dir/"
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
                            sim_p = run(["rosservice", "call", "/gazebo/reset_simulation", "{}"])
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
                    sim_p = run(["rosservice", "call", "/gazebo/reset_simulation", "{}"])
                    sim_exit_code = sim_p.wait(10)
                    if sim_exit_code != 0:
                        print("resetting simulation failed! This should not happen")
                        exit(1)

                    print("initialize Robot")
                    if duration:
                        print("recorder: Robot Collector walk for {} minutes".format(duration))
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
                    run(["rosrun", "walknet_curvewalking", "robot_control_pub.py", "_speed:=" + str(s),
                         "_direction:=" + str(d)])

                    robot_p.wait()
                    data_collector_p.wait()

    end_time = datetime.datetime.now()
    recording_duration = end_time - start_time
    seconds_in_day = 24 * 60 * 60
    (minutes, seconds) = divmod(recording_duration.days * seconds_in_day + recording_duration.seconds, 60)
    print("recording took {} minutes and {} seconds".format(minutes, seconds))


if __name__ == "__main__":
    main()
