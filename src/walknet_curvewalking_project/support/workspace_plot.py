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
    plot = True
    safe_plot = True
    print_aep_median = False

    files = None
    if sys.argv[1] == "-dir":
        files = os.listdir(sys.argv[2])
        files.sort(key=lambda x: os.path.getmtime(sys.argv[2] + "/" + x), reverse=False)
        print(files)
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
        if plot:
            fig, axs = plt.subplots()
            plt.xlim(-0.35, 0.3)
            plt.ylim(-0.4, 0.4)

            # img = mpimg.imread('/home/jsimmering/plots_masterthesis/images/forPlots/widowX-body.png')
            img = mpimg.imread('/home/jsimmering/plots_masterthesis/images/forPlots/body.png')
            axs.imshow(img, alpha=1.0, aspect='equal', extent=(-0.1378, 0.1378, -0.1164, 0.1164))

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
                # Clear old plot
                # plt.clf()

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

        if plot:
            lf_shoulder = np.matrix([0.1248, 0.06164]).T
            axs.plot(lf_shoulder.T[:, 0], lf_shoulder.T[:, 1], 'x', color='gray')
            lm_shoulder = np.matrix([0, 0.1034]).T
            axs.plot(lm_shoulder.T[:, 0], lm_shoulder.T[:, 1], 'x', color='gray')
            lr_shoulder = np.matrix([-0.1248, 0.06164]).T
            axs.plot(lr_shoulder.T[:, 0], lr_shoulder.T[:, 1], 'x', color='gray')
            rf_shoulder = np.matrix([0.1248, -0.06164]).T
            axs.plot(rf_shoulder.T[:, 0], rf_shoulder.T[:, 1], 'x', color='gray')
            rm_shoulder = np.matrix([0, -0.1034]).T
            axs.plot(rm_shoulder.T[:, 0], rm_shoulder.T[:, 1], 'x', color='gray')
            rr_shoulder = np.matrix([-0.1248, -0.06164]).T
            axs.plot(rr_shoulder.T[:, 0], rr_shoulder.T[:, 1], 'x', color='gray')

        # leg value mapping: ([0, 1, 2, 3, 4, 5], ['lf', 'lm', 'lr', 'rr', 'rm', 'rf'])
        counter = 0
        for leg in legs:
            #print("leg = " + str(leg_names[legs.index(leg)]))
            step_length = []
            first_step = True
            for stance in leg:
                X = [point[0] for point in stance]
                Y = [point[1] for point in stance]
                if stance and not first_step:
                    # print("stance = " + str(stance))
                    # print("beginning = stance[0] = " + str(stance[0]))
                    # print("end = stance[len(stance) - 1] = " + str(stance[len(stance) - 1]))
                    # step_length.append(np.linalg.norm(np.array(stance[len(stance) - 1]) - np.array(stance[0])))
                    length = 0
                    start_pos_x = None
                    start_pos_y = None
                    for i in range(1, len(stance)):
                        length += np.linalg.norm(np.array(stance[i]) - np.array(stance[i - 1]))
                    # step_length.append(np.linalg.norm(np.array(stance[len(stance) - 1]) - np.array(stance[0])))
                    step_length.append(length)
                    #print("step length = " + str(length))
                if first_step:
                    first_step = False
                if plot:
                    axs.plot(X, Y)
            if [] in leg:
                leg.remove([])

            if print_aep_median:
                #print("{} leg".format(leg_names[counter]))
                aeps = [stance[0] for stance in leg]
                # aeps.sort(key=operator.itemgetter(0, 1))
                # print("aeps = " + str(aeps))
                # print("aeps length = " + str(len(aeps)))
                # print("aep lower middle value {} = {}".format(len(aeps)//2, aeps[len(aeps)//2]))
                # print("aep upper middle value {} = {}".format(len(aeps) // 2 + 1, aeps[len(aeps) // 2 + 1]))
                print("aep length = {}; / 2 = {}".format(len(aeps), len(aeps) / 2))
                if len(aeps) % 2 == 0:
                    aeps.sort(key=operator.itemgetter(0))
                    print("aep x lower middle value {} = {}".format(len(aeps) // 2, aeps[len(aeps) // 2]))
                    aeps.sort(key=operator.itemgetter(1))
                    print("aep y lower middle value {} = {}".format(len(aeps) // 2, aeps[len(aeps) // 2]))
                else:
                    aeps.sort(key=operator.itemgetter(0))
                    print("aep x lower middle value {} = {}".format(len(aeps) // 2, aeps[len(aeps) // 2]))
                    print("aep x upper middle value {} = {}".format(len(aeps) // 2 + 1, aeps[len(aeps) // 2 + 1]))
                    aeps.sort(key=operator.itemgetter(1))
                    print("aep y lower middle value {} = {}".format(len(aeps) // 2, aeps[len(aeps) // 2]))
                    print("aep y upper middle value {} = {}".format(len(aeps) // 2 + 1, aeps[len(aeps) // 2 + 1]))

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

        if plot:
            # A = [point[0] for point in polygon_list[4]]
            # A.append(polygon_list[4][0][0])
            # A2 = [point[1] for point in polygon_list[4]]
            # A2.append(polygon_list[4][0][1])
            # axs.plot(A, A2)

            # plt.xlim(-0.3, 0.3)
            # plt.ylim(-0.4, 0.4)

            # plt.draw()
            # plt.pause(0.0001)
            # axs.axvline(x=0.25, color='b', lw=1)
            # axs.axvline(x=0.05, color='b', lw=1)
            # axs.axvline(x=-0.17, color='b', lw=1)
            axs.plot(0.25, 0.24, 'x', color='b', markersize=16)
            axs.plot(0.05, (0.24 + 0.04176), 'x', color='b', markersize=16)
            axs.plot((-0.17), 0.24, 'x', color='b', markersize=16)
            axs.plot(0.25, -0.24, 'x', color='b', markersize=16)
            axs.plot(0.05, -(0.24 + 0.04176), 'x', color='b', markersize=16)
            axs.plot(-0.17, -0.24, 'x', color='b', markersize=16)

            # circle = plt.Circle((0.25, 0.24), 0.08, color='g', fill=False)
            # axs.add_patch(circle)
            # circle = plt.Circle((0.05, 0.24 + 0.04176), 0.08, color='g', fill=False)
            # axs.add_patch(circle)
            # circle = plt.Circle((-0.17, 0.24), 0.08, color='g', fill=False)
            # axs.add_patch(circle)
            # circle = plt.Circle((0.25, -0.24), 0.08, color='g', fill=False)
            # axs.add_patch(circle)
            # circle = plt.Circle((0.05, -(0.24 + 0.04176)), 0.08, color='g', fill=False)
            # axs.add_patch(circle)
            # circle = plt.Circle((-0.17, -0.24), 0.08, color='g', fill=False)
            # axs.add_patch(circle)

            # WIDOWX
            # axs.plot(0.27, 0.18, 'x', color='b', markersize=16)
            # axs.plot(0.055, (0.18 + 0.035), 'x', color='b', markersize=16)
            # axs.plot((-0.17), 0.18, 'x', color='b', markersize=16)
            # axs.plot(0.27, -0.18, 'x', color='b', markersize=16)
            # axs.plot(0.055, -(0.18 + 0.035), 'x', color='b', markersize=16)
            # axs.plot(-0.17, -0.18, 'x', color='b', markersize=16)
            # circle = plt.Circle((0.27, 0.18), 0.08, color='g', fill=False)
            # axs.add_patch(circle)
            # circle = plt.Circle((0.055, 0.18 + 0.035), 0.08, color='g', fill=False)
            # axs.add_patch(circle)
            # circle = plt.Circle((-0.17, 0.18), 0.08, color='g', fill=False)
            # axs.add_patch(circle)
            # circle = plt.Circle((0.27, -0.18), 0.08, color='g', fill=False)
            # axs.add_patch(circle)
            # circle = plt.Circle((0.055, -(0.18 + 0.035)), 0.08, color='g', fill=False)
            # axs.add_patch(circle)
            # circle = plt.Circle((-0.17, -0.18), 0.08, color='g', fill=False)
            # axs.add_patch(circle)

            # axs.axvline(x=0.25 - 0.08, ymin=0, ymax=0.5, color='g', lw=1)
            # axs.axvline(x=0.05 - 0.08, ymin=0, ymax=0.5, color='g', lw=1)
            # axs.axvline(x=-0.17 - 0.08, ymin=0, ymax=0.5, color='g', lw=1)


            # walk direction
            split = re.findall(r"[^/_,]+", sys.argv[2], re.ASCII)
            print(split)
            # velocity = float(split[-1][:-1])
            # counter_damping_fact = (-141.5 * velocity) + 35.5
            # stance_speed = (velocity * counter_damping_fact)/35
            stance_speed = 0.045
            # print("stance speed = " + str(stance_speed))
            direction = 0.0
            for s in split:
                if 'dir' in s:
                    direction = float(s.replace('dir', ''))
            # WidowX
            # if direction == 0.0:
            #     for s in split:
            #         if 'rad' in s and 'radii' != s:
            #             direction = float(s.replace('rad', ''))
            print("direction = " + str(direction))
            # arrow = plt.arrow(0.111, 0.0, 0.046, 0.0)
            # axs.add_patch(arrow)
            # plot_pull_vector([0.111, 0.0], [stance_speed * math.cos(i), stance_speed * math.sin(i)], axs, colors[directions.index(i)])
            arrow = plt.arrow(0.111, 0.0, stance_speed * math.cos(direction), stance_speed * math.sin(direction), color='r', head_width=0.01, overhang=0.25)
            # print("arrow length = " + str(math.sqrt(pow(stance_speed * math.cos(direction), 2) + pow(stance_speed * math.sin(direction), 2))))
            axs.add_patch(arrow)

            stance_diff = -pow(0.28 * (direction - 0.9), 2) + 0.02
            print("stance diff = " + str(stance_diff))
            if stance_diff < 0:
                stance_diff = 0
            print("stance diff = " + str(stance_diff))

            # axs.axvline(x=(0.25 - 0.08) + stance_diff, ymin=0.5, ymax=1, color='g', lw=1)
            # axs.axvline(x=(0.05 - 0.08) + stance_diff, ymin=0.5, ymax=1, color='g', lw=1)
            # axs.axvline(x=(-0.17 - 0.08) + stance_diff, ymin=0.5, ymax=1, color='g', lw=1)

            circle = plt.Circle((0.25, 0.24), 0.08 - stance_diff, color='g', fill=False)
            axs.add_patch(circle)
            circle = plt.Circle((0.05, 0.24 + 0.04176), 0.08 - stance_diff, color='g', fill=False)
            axs.add_patch(circle)
            circle = plt.Circle((-0.17, 0.24), 0.08 - stance_diff, color='g', fill=False)
            axs.add_patch(circle)
            circle = plt.Circle((0.25, -0.24), 0.08, color='g', fill=False)
            axs.add_patch(circle)
            circle = plt.Circle((0.05, -(0.24 + 0.04176)), 0.08, color='g', fill=False)
            axs.add_patch(circle)
            circle = plt.Circle((-0.17, -0.24), 0.08, color='g', fill=False)
            axs.add_patch(circle)

            plt.tick_params(labelsize=35)
            plt.grid()
            plt.axis('scaled')
            # plt.gca().set_aspect('equal', adjustable='box')
            if safe_plot:
                plt.subplots_adjust(top=2, bottom=0, right=2, left=0, hspace=1, wspace=1)
                plt.margins(1, 1)

                # split = [i.split("_") for i in files]
                split = re.findall(r"[^/_,]+", file_name, re.ASCII)
                # print("split = " + str(split))
                name = "_".join(split[(split.index("stability") + 4):])
                # print("name = " + name)
                print("single file: " + "/home/jsimmering/plots_masterthesis/workspace/workspace_" + name + ".svg")
                plt.savefig("/home/jsimmering/plots_masterthesis/workspace/workspace_" + name + ".svg",
                        bbox_inches='tight', pad_inches=0, format='svg', transparent=True)

                # sc.Figure("13.9cm", "15.75cm",
                #         # plt.rcParams["figure.figsize"][0], plt.rcParams["figure.figsize"][1],
                #         sc.Panel(sc.SVG("/home/jsimmering/plots_masterthesis/body.svg").scale(0.03175).move(4.35, 5.1)),
                #         sc.Panel(sc.SVG("/home/jsimmering/plots_masterthesis/workspace/workspace_" + name + ".svg").scale(0.022))
                # ).save("/home/jsimmering/plots_masterthesis/workspace/robot_workspace_" + name + ".svg")
                # SVG("/home/jsimmering/plots_masterthesis/workspace/robot_workspace_" + name + ".svg")
            else:
                plt.show()
        print("")

        run += 1

    # print("length collected = " + str(leg_step_length))
    sqrt_errors = []
    #print("leg_step_length = " + str(leg_step_length))
    print("average_step_length_overall = " + str([average_step_length_overall[i]/average_step_length_overall_counter[i] for i in range(0, len(average_step_length_overall))]))
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
    print("errors = " + str(sqrt_errors) + " idx of min (starting at idx 1) = " + str(min_idx + 1))
    print("\n\n")
    print("**workspace:** run " + str(min_idx + 1))
    print("")

    file_name = None
    if len(sys.argv) > 2:
        file_name = sys.argv[2] + "/" + str(files[min_idx])
    else:
        file_name = str(files[min_idx])
    split = re.findall(r"[^/_,]+", file_name, re.ASCII)
    name = "_".join(split[(split.index("stability") + 4):])
    print("workspace_" + name + ".png")

    print("**average step length over all runs:**")
    #print("average_step_length_overall = " + str(average_step_length_overall))
    for i in [0, 5, 1, 4, 2, 3]:
        # print("{} leg = {}".format(leg_names[i], round(average_step_length_overall[i] / len(files), 5)))
        try:
            print(str(round(average_step_length_overall[i] / average_step_length_overall_counter[i], 3)) + " & ", end="")
        except ZeroDivisionError:
            print(" nan & ", end="")
    print("")
    print("")

    for i in [0, 5, 1, 4, 2, 3]:
        # print("{} leg = {}".format(leg_names[i], round(average_step_length_overall[i] / len(files), 5)))
        try:
            print(str(average_step_length_overall[i] / average_step_length_overall_counter[i]) + " ", end="")
        except ZeroDivisionError:
            print(" nan ", end="")
    print("")
    print("")

    print([round(average_step_length_overall[i] / average_step_length_overall_counter[i], 3) for i in [0, 5, 1, 4, 2, 3]])
    print("")
    print("")


if __name__ == '__main__':
    plot_workspace_data()
