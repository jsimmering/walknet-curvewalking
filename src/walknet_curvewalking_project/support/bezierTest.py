import time
import os
import subprocess
import io
import sys


def run(args):
    my_env = os.environ.copy()
    p = subprocess.Popen(args, env=my_env, stdin=subprocess.PIPE, stdout=sys.stdout, stderr=subprocess.STDOUT,
            universal_newlines=True, bufsize=-1)
    return p


def main():
    count = 0
    while True:
        print("robot_controller")
        p = run(["rosrun", "walknet_curvewalking", "robot_controller.py"])
        try:
            p.wait(5)
        except subprocess.TimeoutExpired:
            p.terminate()
            exit(9)

        print("single_leg_controller.py count = " + str(count))
        p = run(["rosrun", "walknet_curvewalking", "single_leg_controller.py"])
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            print("count = " + str(count))
            p.kill()
            exit(1)

        count += 1


if __name__ == "__main__":
    main()
