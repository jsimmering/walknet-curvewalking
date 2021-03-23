#!/usr/bin/env python3
import time
import subprocess
import sys

import numpy


def run(args):
    p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=sys.stdout, stderr=subprocess.STDOUT,
            universal_newlines=True, bufsize=-1)
    return p


def main():
    speed = numpy.arange(0.02, 0.035, 0.005)
    for i in range(0, 10):
        for s in speed:
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
            #robot_p = run(["rosrun", "walknet_curvewalking", "robot_controller.py", "_walk:=True", "_duration:=0.5"])
            robot_p = run(["rosrun", "walknet_curvewalking", "robot_controller.py", "_walk:=True"])

            print("initialize Data Collector")
            data_collector_p = run(["rosrun", "walknet_curvewalking", "DataCollector.py", "1"])

            time.sleep(1)
            print("start walking")
            run(["rosrun", "walknet_curvewalking", "robot_control_pub.py", "_speed:=" + s, "_direction:=0.5"])

            robot_p.wait()
            data_collector_p.wait()

    # print("#############################")
    # print("mit parameter, ohne mp3s")
    # os.chdir("mp3")
    # p = run(["../bin/" + name, "leer"])
    # time.sleep(3)
    # outs, errs = p.communicate()
    # time.sleep(2)
    # print("out:")
    # print(outs)
    # print("err:")
    # print(errs)
    # print("returncode:")
    # print(p.returncode)


if __name__ == "__main__":
    main()
