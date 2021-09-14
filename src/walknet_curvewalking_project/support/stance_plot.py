#!/usr/bin/env python3
import math
import operator
import os
import re
import sys

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy
import numpy as np
import svgutils.compose as sc
from IPython.display import SVG


def plot_step(leg_aeps, leg_peps, axs, lines=None):
    if len(leg_aeps) != len(leg_peps):
        raise IndexError("not the same number of AEP and PEPs in lists")
    directions = ['0.0rad', '0.2rad', '0.4rad', '0.6rad', '0.8rad', '1.0rad', '1.2rad', '1.4rad', '1.57rad']
    for i in range(0, len(leg_aeps)):
        if lines is not None:
            line = axs.plot([leg_aeps[i][0], leg_peps[i][0]], [leg_aeps[i][1], leg_peps[i][1]], label=directions[i])
            lines.append(line)
        else:
            axs.plot([leg_aeps[i][0], leg_peps[i][0]], [leg_aeps[i][1], leg_peps[i][1]])
    print("leng_aeps = " + str(len(leg_aeps)))
    if len(leg_aeps) < 7:
        axs.plot([10, 10], [10, 10])
    return lines

def plot_pull_vector(robot_front, pull_vector, axs, color):
    directions = ['0.0rad', '0.2rad', '0.4rad', '0.6rad', '0.8rad', '1.0rad', '1.2rad', '1.4rad', '1.57rad']
    # for i in range(0, len(pull_vectors)):
    #axs.plot([robot_front[0], robot_front[0] + pull_vectors[i][0]], [robot_front[1], robot_front[1] + pull_vectors[i][1]])
    print("color = " + str(color))
    arrow = plt.arrow(robot_front[0], robot_front[1], pull_vector[0], pull_vector[1], color=color, head_width=0.01, overhang=0.25)
    axs.add_patch(arrow)
    # print("leng_aeps = " + str(len(pull_vectors)))
    # if len(pull_vectors) < 7:
    #     axs.plot([10, 10], [10, 10])


def plot_workspace_data():
    # get average AEP and PEP per dir
    average_AEPs_lf = []
    average_PEPs_lf = []
    average_AEPs_lm = []
    average_PEPs_lm = []
    average_AEPs_lr = []
    average_PEPs_lr = []
    average_AEPs_rf = []
    average_PEPs_rf = []
    average_AEPs_rm = []
    average_PEPs_rm = []
    average_AEPs_rr = []
    average_PEPs_rr = []

    for direction in file_names:
        # print("direction = " + str(file_names.index(direction)))
        average_aeps_for_direction = []
        average_peps_for_direction = []
        for run in direction:
            # print("run = " + str(direction.index(run)))
            legs_aeps = [[], [], [], [], [], []]
            legs_peps = [[], [], [], [], [], []]
            steps = [0, 0, 0, 0, 0, 0]
            last_state_swing = [False, False, False, False, False, False]
            last_values = None
            first_line = True
            for line in open(str(run), 'r'):
                if first_line:
                    first_line = False
                else:
                    line = line.rstrip("\n")
                    values = [float(s) for s in line.split(";")]

                    for i in range(1, 19, 3):
                        if values[i] != 0.0 and values[i + 1] != 0.0:
                            if last_state_swing[(i - 1) // 3]:
                                legs_aeps[(i - 1) // 3].append([values[i], values[i + 1]])
                                last_state_swing[(i - 1) // 3] = False
                        else:
                            if not last_state_swing[(i - 1) // 3]:
                                if last_values:
                                    legs_peps[(i - 1) // 3].append([last_values[i], last_values[i + 1]])
                                steps[(i - 1) // 3] += 1
                                last_state_swing[(i - 1) // 3] = True

                    last_values = values

            # print("steps = " + str(steps))
            average_aeps_for_direction.append([[numpy.average(numpy.array([step[0] for step in leg if len(step) > 0])),
                                                numpy.average(numpy.array([step[1] for step in leg if len(step) > 0]))]
                                               for
                                               leg in legs_aeps])
            average_peps_for_direction.append([[numpy.average(numpy.array([step[0] for step in leg if len(step) > 0])),
                                                numpy.average(numpy.array([step[1] for step in leg if len(step) > 0]))]
                                               for
                                               leg in legs_peps])

        lf_average_aep = []
        lm_average_aep = []
        lr_average_aep = []
        rf_average_aep = []
        rm_average_aep = []
        rr_average_aep = []
        lf_average_pep = []
        lm_average_pep = []
        lr_average_pep = []
        rf_average_pep = []
        rm_average_pep = []
        rr_average_pep = []
        for i in range(0, 6):  # for each run
            if len(average_aeps_for_direction) > i:
                lf_average_aep.append(average_aeps_for_direction[i][0])
                lm_average_aep.append(average_aeps_for_direction[i][1])
                lr_average_aep.append(average_aeps_for_direction[i][2])
                rf_average_aep.append(average_aeps_for_direction[i][3])
                rm_average_aep.append(average_aeps_for_direction[i][4])
            else:
                print("no average aep for run " + str(i))
            if len(average_peps_for_direction) > i:
                rr_average_aep.append(average_aeps_for_direction[i][5])
                lf_average_pep.append(average_peps_for_direction[i][0])
                lm_average_pep.append(average_peps_for_direction[i][1])
                lr_average_pep.append(average_peps_for_direction[i][2])
                rf_average_pep.append(average_peps_for_direction[i][3])
                rm_average_pep.append(average_peps_for_direction[i][4])
                rr_average_pep.append(average_peps_for_direction[i][5])
            else:
                print("no average pep for run " + str(i))

        average_AEPs_lf.append([numpy.average(numpy.array([run[0] for run in lf_average_aep])),
                                numpy.average(numpy.array([run[1] for run in lf_average_aep]))])
        average_AEPs_lm.append([numpy.average(numpy.array([run[0] for run in lm_average_aep])),
                                numpy.average(numpy.array([run[1] for run in lm_average_aep]))])
        average_AEPs_lr.append([numpy.average(numpy.array([run[0] for run in lr_average_aep])),
                                numpy.average(numpy.array([run[1] for run in lr_average_aep]))])
        average_AEPs_rf.append([numpy.average(numpy.array([run[0] for run in rf_average_aep])),
                                numpy.average(numpy.array([run[1] for run in rf_average_aep]))])
        average_AEPs_rm.append([numpy.average(numpy.array([run[0] for run in rm_average_aep])),
                                numpy.average(numpy.array([run[1] for run in rm_average_aep]))])
        average_AEPs_rr.append([numpy.average(numpy.array([run[0] for run in rr_average_aep])),
                                numpy.average(numpy.array([run[1] for run in rr_average_aep]))])

        average_PEPs_lf.append([numpy.average(numpy.array([run[0] for run in lf_average_pep])),
                                numpy.average(numpy.array([run[1] for run in lf_average_pep]))])
        average_PEPs_lm.append([numpy.average(numpy.array([run[0] for run in lm_average_pep])),
                                numpy.average(numpy.array([run[1] for run in lm_average_pep]))])
        average_PEPs_lr.append([numpy.average(numpy.array([run[0] for run in lr_average_pep])),
                                numpy.average(numpy.array([run[1] for run in lr_average_pep]))])
        average_PEPs_rf.append([numpy.average(numpy.array([run[0] for run in rf_average_pep])),
                                numpy.average(numpy.array([run[1] for run in rf_average_pep]))])
        average_PEPs_rm.append([numpy.average(numpy.array([run[0] for run in rm_average_pep])),
                                numpy.average(numpy.array([run[1] for run in rm_average_pep]))])
        average_PEPs_rr.append([numpy.average(numpy.array([run[0] for run in rr_average_pep])),
                                numpy.average(numpy.array([run[1] for run in rr_average_pep]))])

        # print("")

    print("average_aeps_for_direction lf: " + str(average_AEPs_lf))
    print("average_aeps_for_direction lm: " + str(average_AEPs_lm))
    print("average_aeps_for_direction lr: " + str(average_AEPs_lr))
    print("average_aeps_for_direction rf: " + str(average_AEPs_rf))
    print("average_aeps_for_direction rm: " + str(average_AEPs_rm))
    print("average_aeps_for_direction rr: " + str(average_AEPs_rr))
    print("average_peps_for_direction lf: " + str(average_AEPs_lf))
    print("average_peps_for_direction lm: " + str(average_AEPs_lm))
    print("average_peps_for_direction lr: " + str(average_AEPs_lr))
    print("average_peps_for_direction rf: " + str(average_AEPs_rf))
    print("average_peps_for_direction rm: " + str(average_AEPs_rm))
    print("average_peps_for_direction rr: " + str(average_AEPs_rr))

    if plot:
        fig, axs = plt.subplots()
        # axs.set_prop_cycle(color=['#b2babb', '#99a3a4', '#7f8c8d', '#707b7c', '#616a6b', '#515a5a', '#424949'])
        # hell lila '#9400D3' blue '#0000FF' green '#00FF00' yellow '#FFFF00' red '#FF0000'
        if len(file_names) == 7:
            colors = ['m', 'indigo', 'b', 'c', 'g', '#FF7F00', 'r']
        elif len(file_names) == 8:
            colors = ['m', 'indigo', 'b', 'c', 'g', '#FF7F00', 'r', 'firebrick']
        elif len(file_names) == 9:
            colors = ['m', 'indigo', 'b', 'c', 'g', 'y', '#FF7F00', 'r', 'firebrick']
        else:
            raise IndexError('number of directions does not match number of colors')
        axs.set_prop_cycle(color=colors)
        plot_lines = []
        plt.xlim(-0.35, 0.3)
        plt.ylim(-0.4, 0.4)

        img = mpimg.imread('/home/jsimmering/plots_masterthesis/images/forPlots/body.png')
        axs.imshow(img, alpha=1.0, aspect='equal', extent=(-0.1378, 0.1378, -0.1164, 0.1164))

        lf_shoulder = np.matrix([0.1248, 0.06164]).T
        axs.plot(lf_shoulder.T[:, 0], lf_shoulder.T[:, 1], 'xb')
        lm_shoulder = np.matrix([0, 0.1034]).T
        axs.plot(lm_shoulder.T[:, 0], lm_shoulder.T[:, 1], 'xb')
        lr_shoulder = np.matrix([-0.1248, 0.06164]).T
        axs.plot(lr_shoulder.T[:, 0], lr_shoulder.T[:, 1], 'xb')
        rf_shoulder = np.matrix([0.1248, -0.06164]).T
        axs.plot(rf_shoulder.T[:, 0], rf_shoulder.T[:, 1], 'xb')
        rm_shoulder = np.matrix([0, -0.1034]).T
        axs.plot(rm_shoulder.T[:, 0], rm_shoulder.T[:, 1], 'xb')
        rr_shoulder = np.matrix([-0.1248, -0.06164]).T
        axs.plot(rr_shoulder.T[:, 0], rr_shoulder.T[:, 1], 'xb')

        # leg value mapping: ([0, 1, 2, 3, 4, 5], ['lf', 'lm', 'lr', 'rr', 'rm', 'rf'])
        plot_lines = plot_step(average_AEPs_lf, average_PEPs_lf, axs, plot_lines)
        plot_step(average_AEPs_lm, average_PEPs_lm, axs)
        plot_step(average_AEPs_lr, average_PEPs_lr, axs)
        plot_step(average_AEPs_rf, average_PEPs_rf, axs)
        plot_step(average_AEPs_rm, average_PEPs_rm, axs)
        plot_step(average_AEPs_rr, average_PEPs_rr, axs)
        # legend = axs.legend(handles=plot_lines, loc='upper center')
        axs.legend(fontsize=18, bbox_to_anchor=(0.5, -0.1))

        if plot:
            axs.plot((0.25), (0.24), 'x', color='b')
            axs.plot((0.05), (0.24 + 0.04176), 'x', color='b')
            axs.plot((-0.17), (0.24), 'x', color='b')
            axs.plot((0.25), -(0.24), 'x', color='b')
            axs.plot((0.05), -(0.24 + 0.04176), 'x', color='b')
            axs.plot((-0.17), -(0.24), 'x', color='b')

            # axs.axvline(x=0.25 - 0.08, color='g', lw=1)
            # axs.axvline(x=0.05 - 0.08, color='g', lw=1)
            # axs.axvline(x=-0.17 - 0.08, color='g', lw=1)

            split = re.findall(r"[^/_,]+", sys.argv[2], re.ASCII)
            print(split)
            #velocity = float(split[-1][:-1])
            #counter_damping_fact = (-141.5 * velocity) + 35.5
            #stance_speed = (velocity * counter_damping_fact)/35
            stance_speed = 0.045
            print("stance speed = " + str(stance_speed))
            # arrow = plt.arrow(0.111, 0.0, 0.046, 0.0)
            # axs.add_patch(arrow)
            directions = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]
            for i in directions[:len(file_names)]:
                plot_pull_vector([0.111, 0.0], [stance_speed * math.cos(i), stance_speed * math.sin(i)], axs, colors[directions.index(i)])

            plt.tick_params(labelsize=20)
            # plt.grid()
            plt.axis('scaled')
            plt.axis('off')
            # plt.gca().set_aspect('equal', adjustable='box')
            if safe_plot:
                plt.subplots_adjust(top=2, bottom=0, right=2, left=0, hspace=1, wspace=1)
                plt.margins(1, 1)

                # split = re.findall(r"[^/_,]+", sys.argv[2], re.ASCII)
                # print(split)
                name = ""
                name += "_".join(split[split.index("walknet"):])
                plot_path = "/home/jsimmering/plots_masterthesis/workspace/static_speed_workspace" + "_" + name + ".svg"
                print("single file: " + plot_path)
                plt.savefig(plot_path, bbox_inches='tight', pad_inches=0, format='svg', transparent=True)
            else:
                plt.show()
        print("")


if __name__ == '__main__':
    plot = True
    safe_plot = True
    print_aep_median = False

    file_names = None
    if sys.argv[1] == "-dir":
        file_names = [[sys.argv[2] + "/" + x] for x in os.listdir(sys.argv[2])]
        [file_names[i].sort(key=lambda x: os.path.getmtime(x), reverse=False) for i in range(0, len(file_names))]
        print(file_names)
    elif sys.argv[1] == "-c":
        dirs = os.listdir(sys.argv[2] + "/")
        dirs = sorted(dirs)
        # if '0.0dir' in dirs:
        #     dirs.remove('0.0dir')
        # dirs.sort(key=lambda x: os.path.getmtime(sys.argv[2] + "/" + x), reverse=False)
        print(dirs)
        file_names = [
            [sys.argv[2] + "/" + path + "/stability/" + x for x in os.listdir(sys.argv[2] + "/" + path + "/stability/")]
            for path in dirs]
        print(file_names)
        # full_paths = [[re.findall(r"[^/_,]+", file_names[j][i], re.ASCII) for i in range(0, len(file_names[j]))] for
        #               j in range(0, len(file_names))]
        # file_names_only = [[label[label.index("walknet"):] for label in labels] for labels in full_paths]
        # print("file_names = " + str(
        #         [["_".join(run) for run in configurations] for configurations in file_names_only]))
    elif len(sys.argv) == 2:
        file_names = [[sys.argv[1]]]
    else:
        print("wrong parameters provide data-filename, -dir directory or -c with top directory with files")
        exit()

    print("file_names length = " + str(len(file_names)))

    plot_workspace_data()
