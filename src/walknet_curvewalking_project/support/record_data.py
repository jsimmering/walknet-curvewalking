#!/usr/bin/env python3
import subprocess
import sys
import time

import numpy


def run(args):
    p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=sys.stdout, stderr=subprocess.STDOUT,
            universal_newlines=True, bufsize=-1)
    return p


def main():
    repetitions_per_speed = 5
    duration = None
    circles = 1
    distance = None
    speed = numpy.arange(0.02, 0.065, 0.01)
    direction = numpy.arange(0.10, 0.24, 0.05)
    # direction = numpy.arange(1.1, 1.5, 0.1)
    # direction = [0.2]
    #speed = [0.007]
    for i in range(0, repetitions_per_speed):
        for s in speed:
            for d in direction:
                print("#############################")
                print("stand up")
                stand_up_exit_code = None
                for i in range(0, 5):
                    stand_up_p = run(["rosrun", "walknet_curvewalking", "robot_controller.py", "_walk:=False"])
                    try:
                        stand_up_exit_code = stand_up_p.wait(10)
                    except subprocess.TimeoutExpired:
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
                                   "_duration:=" + str(duration)])
                else:
                    print("recorder: Robot Controller walk walk until stop")
                    robot_p = run(["rosrun", "walknet_curvewalking", "robot_controller.py", "_walk:=True",
                                   "_duration:=0"])

                print("initialize Data Collector")
                if circles:
                    print("recorder: Data Collector walk {} circles".format(circles))
                    data_collector_p = run(
                            ["rosrun", "walknet_curvewalking", "DataCollector.py", "_circles:=" + str(circles),
                             "_distance:=0"])
                elif distance:
                    print("recorder: Data Collector walk {} meter".format(distance))
                    data_collector_p = run(
                            ["rosrun", "walknet_curvewalking", "DataCollector.py", "_circles:=0",
                             "_distance:=" + str(distance)])
                else:
                    print("recorder: Data Collector walk walk until stop")
                    data_collector_p = run(
                            ["rosrun", "walknet_curvewalking", "DataCollector.py", "_circles:=0", "_distance:=0.0"])

                time.sleep(1)
                print("start walking")
                run(["rosrun", "walknet_curvewalking", "robot_control_pub.py", "_speed:=" + str(s),
                     "_direction:=" + str(d)])

                robot_p.wait()
                data_collector_p.wait()


if __name__ == "__main__":
    main()
