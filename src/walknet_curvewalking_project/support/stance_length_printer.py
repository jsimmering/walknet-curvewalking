#!/usr/bin/env python3
import math
import operator
import os
import re
import sys

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import svgutils.compose as sc
from IPython.display import SVG


def plot_workspace_data():
    files = None
    if sys.argv[1] == "-dir":
        files = os.listdir(sys.argv[2])
        files.sort(key=lambda x: os.path.getmtime(sys.argv[2] + "/" + x), reverse=False)
        # print(files)
    elif len(sys.argv) == 2:
        files = [sys.argv[1]]
    else:
        print("wrong parameters provide data-filename or -dir directory with files")
        exit()

    average_step_length_overall = [0, 0, 0, 0, 0, 0]
    average_step_length_overall_counter = [0, 0, 0, 0, 0, 0]
    leg_names = ['lf', 'lm', 'lr', 'rr', 'rm', 'rf']
    leg_step_length = [[], [], [], [], [], []]
    run = 0
    for file in files:
        legs = [[[]], [[]], [[]], [[]], [[]], [[]]]
        steps = [0, 0, 0, 0, 0, 0]
        last_state_swing = [False, False, False, False, False, False]
        first_line = True

        file_name = None
        if len(sys.argv) > 2:
            file_name = sys.argv[2] + "/" + str(file)
        else:
            file_name = str(file)
        for line in open(str(file_name), 'r'):
            if first_line:
                first_line = False
                pass
            else:
                line = line.rstrip("\n")
                values = [float(s) for s in line.split(";")]

                for i in range(1, 19, 3):
                    if values[i] != 0.0 and values[i + 1] != 0.0:
                        # print("i = {}, (i-1)//3 = {}, steps[{}] = {}, leg[{}] len = {}".format(i, (i-1)//3, (i-1)//3, steps[(i-1)//3], (i-1)//3, len(legs[(i - 1) // 3])))
                        legs[(i - 1) // 3][steps[(i - 1) // 3]].append([values[i], values[i + 1]])
                        last_state_swing[(i - 1) // 3] = False
                    else:
                        if not last_state_swing[(i - 1) // 3]:
                            steps[(i - 1) // 3] += 1
                            legs[(i - 1) // 3].append([])
                            last_state_swing[(i - 1) // 3] = True

        # leg value mapping: ([0, 1, 2, 3, 4, 5], ['lf', 'lm', 'lr', 'rr', 'rm', 'rf'])
        counter = 0
        for leg in legs:
            # print("leg = " + str(leg_names[legs.index(leg)]))
            step_length = []
            first_step = True
            for stance in leg:
                X = [point[0] for point in stance]
                Y = [point[1] for point in stance]
                if stance and not first_step:
                    # print("stance = " + str(stance))
                    # print("beginning = stance[0] = " + str(stance[0]))
                    # print("end = stance[len(stance) - 1] = " + str(stance[len(stance) - 1]))
                    length = 0
                    start_pos_x = None
                    start_pos_y = None
                    for i in range(1, len(stance)):
                        length += np.linalg.norm(np.array(stance[i]) - np.array(stance[i-1]))
                    # step_length.append(np.linalg.norm(np.array(stance[len(stance) - 1]) - np.array(stance[0])))
                    step_length.append(length)
                if first_step:
                    first_step = False
            if [] in leg:
                leg.remove([])

            if len(step_length) > 0:
                average_step_length = np.sum(step_length) / len(step_length)
                if average_step_length:
                    #print("no average_step_length")
                    leg_step_length[counter].append(average_step_length)
                    average_step_length_overall[counter] += average_step_length
                    average_step_length_overall_counter[counter] += 1
            else:
                leg_step_length[counter].append(float('nan'))
            counter += 1

        run += 1

    # print("length collected = " + str(leg_step_length))
    sqrt_errors = []
    #print("leg_step_length = " + str(leg_step_length))
    #print("average_step_length_overall = " + str([average_step_length_overall[i]/average_step_length_overall_counter[i] for i in range(0, len(average_step_length_overall))]))
    for i in range(0, len(files)):
        # for each run
        sqrt_error = 0
        error_counter = 0
        for j in range(0, 6):
            # for each leg
            #print("sqrt_error += ({} - {})^2 = {}".format(leg_step_length[j][i], average_step_length_overall[j]/average_step_length_overall_counter[j], pow(leg_step_length[j][i] - average_step_length_overall[j]/average_step_length_overall_counter[j], 2)))
            if average_step_length_overall_counter[j] != 0:
                error = pow(leg_step_length[j][i] - average_step_length_overall[j]/average_step_length_overall_counter[j], 2)
                if not math.isnan(error):
                    sqrt_error += error
                    error_counter += 1
        #print("sqrt_errors.append {}/{} = {}".format(sqrt_error, error_counter, sqrt_error / error_counter))
        sqrt_errors.append(sqrt_error / error_counter)
    min_idx = sqrt_errors.index(min(sqrt_errors))
    #print("errors = " + str(sqrt_errors) + " idx of min (starting at idx 1) = " + str(min_idx + 1))
    #print("\n\n")
    #print("**workspace:** run " + str(min_idx + 1))
    #print("")

    file_name = None
    if len(sys.argv) > 2:
        file_name = sys.argv[2] + "/" + str(files[min_idx])
    else:
        file_name = str(files[min_idx])
    split = re.findall(r"[^/_,]+", file_name, re.ASCII)
    name = "_".join(split[(split.index("stability") + 4):])
    #print("workspace_" + name + ".png")

    #print("**average step length over all runs:**")
    #print("average_step_length_overall = " + str(average_step_length_overall))
    # for i in [0, 5, 1, 4, 2, 3]:
    #     # print("{} leg = {}".format(leg_names[i], round(average_step_length_overall[i] / len(files), 5)))
    #     try:
    #         print(str(round(average_step_length_overall[i] / average_step_length_overall_counter[i], 3)) + " & ", end="")
    #     except ZeroDivisionError:
    #         print(" nan & ", end="")
    # print("")
    # print("")

    for i in [0, 5, 1, 4, 2, 3]:
        # print("{} leg = {}".format(leg_names[i], round(average_step_length_overall[i] / len(files), 5)))
        try:
            print(str(average_step_length_overall[i] / average_step_length_overall_counter[i]) + " ", end="")
        except ZeroDivisionError:
            print(" nan ", end="")
    #print("")
    #print("")

    # print([round(average_step_length_overall[i] / average_step_length_overall_counter[i], 3) for i in [0, 5, 1, 4, 2, 3]])
    # print("")
    # print("")


if __name__ == '__main__':
    plot_workspace_data()
