#!/usr/bin/env python3
import sys

import matplotlib.pyplot as plt
import numpy as np


def plot_workspace_data():
    # X, Y = [], []
    legs = [[[]], [[]], [[]], [[]], [[]], [[]]]
    steps = [0, 0, 0, 0, 0, 0]
    last_state_swing = [False, False, False, False, False, False]
    first_line = True
    plt.figure()
    plt.xlim(-0.35, 0.3)
    plt.ylim(-0.4, 0.4)
    for line in open(str(sys.argv[1]), 'r'):
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
    leg_names = ['lf', 'lm', 'lr', 'rr', 'rm', 'rf']
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
            plt.plot(X, Y)
        average_step_length = np.sum(step_length) / len(step_length)
        print("average_step_length = " + str(average_step_length))
        counter += 1
    # A = [point[0] for point in polygon_list[4]]
    # A.append(polygon_list[4][0][0])
    # A2 = [point[1] for point in polygon_list[4]]
    # A2.append(polygon_list[4][0][1])
    # plt.plot(A, A2)

    # plt.xlim(-0.3, 0.3)
    # plt.ylim(-0.4, 0.4)

    # plt.draw()
    # plt.pause(0.0001)
    plt.grid()
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) == 2:
        plot_workspace_data()
