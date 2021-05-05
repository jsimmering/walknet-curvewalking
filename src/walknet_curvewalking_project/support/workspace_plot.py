#!/usr/bin/env python3
import os
import re
import sys

import matplotlib.pyplot as plt
import numpy as np


def plot_workspace_data():
    plot = True
    safe_plot = True

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
    leg_names = ['lf', 'lm', 'lr', 'rr', 'rm', 'rf']
    for file in files:
        if plot:
            plt.figure()
            plt.xlim(-0.35, 0.3)
            plt.ylim(-0.4, 0.4)

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
            plt.plot(lf_shoulder.T[:, 0], lf_shoulder.T[:, 1], 'xb')
            lm_shoulder = np.matrix([0, 0.1034]).T
            plt.plot(lm_shoulder.T[:, 0], lm_shoulder.T[:, 1], 'xb')
            lr_shoulder = np.matrix([-0.1248, 0.06164]).T
            plt.plot(lr_shoulder.T[:, 0], lr_shoulder.T[:, 1], 'xb')
            rf_shoulder = np.matrix([0.1248, -0.06164]).T
            plt.plot(rf_shoulder.T[:, 0], rf_shoulder.T[:, 1], 'xb')
            rm_shoulder = np.matrix([0, -0.1034]).T
            plt.plot(rm_shoulder.T[:, 0], rm_shoulder.T[:, 1], 'xb')
            rr_shoulder = np.matrix([-0.1248, -0.06164]).T
            plt.plot(rr_shoulder.T[:, 0], rr_shoulder.T[:, 1], 'xb')

        # leg value mapping: ([0, 1, 2, 3, 4, 5], ['lf', 'lm', 'lr', 'rr', 'rm', 'rf'])
        counter = 0
        for leg in legs:
            print("{}: {} leg".format(counter, leg_names[counter]))
            step_length = []
            first_step = True
            for stance in leg:
                X = [point[0] for point in stance]
                Y = [point[1] for point in stance]
                if stance and not first_step:
                    # print("stance = " + str(stance))
                    # print("beginning = stance[0] = " + str(stance[0]))
                    # print("end = stance[len(stance) - 1] = " + str(stance[len(stance) - 1]))
                    step_length.append(np.linalg.norm(np.array(stance[len(stance) - 1]) - np.array(stance[0])))
                if first_step:
                    first_step = False
                if plot:
                    plt.plot(X, Y)
            average_step_length = np.sum(step_length) / len(step_length)
            print("average_step_length = " + str(average_step_length))
            average_step_length_overall[counter] += average_step_length
            counter += 1

        if plot:
            # A = [point[0] for point in polygon_list[4]]
            # A.append(polygon_list[4][0][0])
            # A2 = [point[1] for point in polygon_list[4]]
            # A2.append(polygon_list[4][0][1])
            # plt.plot(A, A2)

            # plt.xlim(-0.3, 0.3)
            # plt.ylim(-0.4, 0.4)

            # plt.draw()
            # plt.pause(0.0001)
            plt.axvline(x=0.25, color='b', lw=1)
            plt.axvline(x=0.05, color='b', lw=1)
            plt.axvline(x=-0.17, color='b', lw=1)

            plt.axvline(x=0.25 - 0.08, color='g', lw=1)
            plt.axvline(x=0.05 - 0.08, color='g', lw=1)
            plt.axvline(x=-0.17 - 0.08, color='g', lw=1)

            plt.tick_params(labelsize=20)
            plt.grid()
            plt.axis('scaled')
            # plt.gca().set_aspect('equal', adjustable='box')
            if safe_plot:
                plt.subplots_adjust(top=2, bottom=0, right=2, left=0, hspace=1, wspace=1)
                plt.margins(1, 1)

                # split = [i.split("_") for i in files]
                split = re.findall(r"[^/_,]+", file_name, re.ASCII)
                print("split = " + str(split))
                name = "_".join(split[(split.index("stability") + 4):])
                print("name = " + name)
                print("single file: " + "/home/jsimmering/plots_masterthesis/workspace/workspace_" + name + ".pdf")
                plt.savefig("/home/jsimmering/plots_masterthesis/workspace/workspace_" + name + ".png",
                        bbox_inches='tight',
                        pad_inches=0)
            else:
                plt.show()
        print("")

    print("average step length over all runs:")
    for i in [0, 1, 2, 5, 4, 3]:
        print("{} leg = {}".format(leg_names[i], round(average_step_length_overall[i] / len(files), 5)))


if __name__ == '__main__':
    plot_workspace_data()
