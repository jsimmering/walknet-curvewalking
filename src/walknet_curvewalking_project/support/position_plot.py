#!/usr/bin/env python3
import os
import sys
import re

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import argparse
from scipy.constants import g

import svgutils.compose as sc
from IPython.display import SVG

if len(sys.argv) >= 2:

    plot = True
    safe_plot = True
    plot_heigth = True
    print_height = False
    calculate_cot = True
    controll_colors = False
    controll_colors_dir = False
    colors = ['b', 'r', 'g', 'm', 'c', 'y', '#e67e22', '#78281f', '#1abc9c', '#909497', '#34495e', '#17202a']

    files = None
    parser = argparse.ArgumentParser(
            description='plot the path the robot walked by providing a file containing the positions of the robot. ' +
                        'Optionally calculate the cost of transport, by additionally providing a file containing an ' +
                        'approximation of the power consumption and controller steps.')
    parser.add_argument('-dir', '--directory',
            help='the directory containing all walknet_position files that should be considered.')
    parser.add_argument('-cot_dir', '--costOfTransport_directory',
            help='the directory containing the walknet_ files corresponding to the provided position files.')
    parser.add_argument('-f', '--file', help='the walknet_position file that should be considered.')
    parser.add_argument('-cot_f', '--costOfTransport_file',
            help='the walknet_ files corresponding to the provided position file.')
    args = parser.parse_args()
    if args.directory:
        files = os.listdir(args.directory)
        files.sort(key=lambda x: os.path.getmtime(args.directory + "/" + x), reverse=False)
        print(files)
    elif args.file:
        files = [args.file]
    else:
        print("wrong parameters provide -f data-filename or -dir directory with files")
        exit()
    if args.costOfTransport_directory:
        cot_files = [file for file in os.listdir(args.costOfTransport_directory) if file.find('walknet') != -1]
        cot_files.sort(key=lambda x: os.path.getmtime(args.costOfTransport_directory + "/" + x), reverse=False)
        print(cot_files)
    elif args.costOfTransport_file:
        cot_files = [args.costOfTransport_file]
    else:
        cot_files = []
        calculate_cot = False

    if calculate_cot and len(cot_files) != len(files):
        print("same number of position and additional information files need to be provided")
        exit()

    distance = [0 for i in range(0, len(files))]
    X, Y, Z, start_x, start_y, start_z, velocity, x_dim, y_dim = [], [], [], [], [], [], [], [], []
    for i in range(0, len(files)):
        X.append([])
        Y.append([])
        Z.append([])
        start_x.append([])
        start_y.append([])
        start_z.append([])
        velocity.append([])
        x_dim.append([])
        y_dim.append([])

    if plot:
        fig, axs = plt.subplots()
        fig.tight_layout()

        # img = mpimg.imread('/home/jsimmering/plots_masterthesis/phantomXBody_turned.png')
        # img = mpimg.imread('/home/jsimmering/plots_masterthesis/body.svg')
        # axs.imshow(img, alpha=0.5, aspect='equal', extent=(-0.1378, 0.1378, -0.1164, 0.1164))

    current_speed = None
    current_dir = None
    current_color = 0
    for j in range(0, len(files)):
        first_line = True
        last_position = None
        first_position = None
        last_time = None
        x_min = float('inf')
        x_max = float('-inf')
        y_min = float('inf')
        y_max = float('-inf')

        line_number = 0
        file_name = None
        if not args.file:
            file_name = args.directory + "/" + str(files[j])
        else:
            file_name = str(files[j])
        for line in open(file_name, 'r'):
            if line_number % 1000 == 0:
                line = line.rstrip("\n")
                try:
                    values = [float(s) for s in line.split(";")]
                except ValueError as err:
                    if first_line:
                        first_line = False
                        continue
                    else:
                        raise ValueError('First Line already found') from err
                # print("time = " + str(values[0]))
                # if start_x is None:
                if not start_x[j]:
                    start_x[j].append(values[1])
                    # print("startx = " + str(start_x) + " j = " + str(j))
                    start_y[j].append(values[2])
                    start_z[j].append(values[3])
                    # print("start_z = " + str(start_z))

                if first_position is None:
                    first_position = [values[1], values[2]]

                # print("x = " + str(round(values[1],4)) + "y = " + str(round(values[2],4)) + "z = " + str(round(values[3],4)))
                X[j].append(round(values[1] - start_x[j][0], 4))
                Y[j].append(round(values[2] - start_y[j][0], 4))
                # if line_number > 5000:
                Z[j].append(round(values[3], 4))

                if values[1] > x_max:
                    x_max = values[1]
                if values[1] < x_min:
                    x_min = values[1]
                if values[2] > y_max:
                    y_max = values[2]
                if values[2] < y_min:
                    y_min = values[2]

                # print("gazebo velocity = " + str(gazebo_vel))
                if last_position is not None:
                    dis = np.linalg.norm(np.array([values[1], values[2]]) - np.array(last_position))
                    distance[j] += dis
                    vel = dis / (values[0] - last_time)
                    velocity[j].append(vel)
                last_position = [values[1], values[2]]
                last_time = values[0]
            line_number += 1

        x_dim[j].append(x_max - x_min)
        y_dim[j].append(y_max - y_min)
        if plot:
            if controll_colors:
                if not current_speed:
                    split = re.findall(r"[^/_,]+", files[j], re.ASCII)
                    speed = split[split.index("position") + 1]
                    current_speed = float(speed[:-1])

                split = re.findall(r"[^/_,]+", files[j], re.ASCII)
                speed = split[split.index("position") + 1]
                if current_speed != speed:
                    current_color += 1
                    current_speed = speed
                axs.plot(X[j], Y[j], colors[current_color % len(colors)])
            elif controll_colors_dir:
                if not current_dir:
                    split = re.findall(r"[^/_,]+", files[j], re.ASCII)
                    dir = split[split.index("position") + 2]
                    current_dir = float(dir[:-3])

                split = re.findall(r"[^/_,]+", files[j], re.ASCII)
                dir = split[split.index("position") + 2]
                if current_dir != dir:
                    current_color += 1
                    current_dir = dir
                axs.plot(X[j], Y[j], colors[current_color % len(colors)])
            else:
                axs.plot(X[j], Y[j])
            split = [i.split("_") for i in files]
            # axs.legend([i.split("_")[2] + "_" + i.split("_")[3] for i in files], loc='lower right')
            axs.legend([str(n+1) for n in range(0, len(files))],
                    # [split[i][split[i].index("position") + 1] + "_" + split[i][split[i].index("position") + 2] for i in
                    #  range(0, len(split))],
                    loc='lower right')

    if calculate_cot:
        total_power_command = []
        total_power_joint_torque = []
        controller_steps = []
        duration = []
        for k in range(0, len(cot_files)):
            file_name = None
            if args.costOfTransport_directory:
                file_name = args.costOfTransport_directory + "/" + str(cot_files[k])
            elif args.costOfTransport_file:
                file_name = str(cot_files[k])
            for line in open(file_name, 'r'):
                line = line.rstrip("\n")
                split = line.split(" ")
                if split[0] == "total" and split[1] == "power" and split[2] == "command":
                    total_power_command.append(float(split[-1]))
                if split[0] == "total" and split[1] == "power" and split[2] == "joint" and split[3] == "torque":
                    total_power_joint_torque.append(float(split[-1]))
                if split[0] == "controller" and split[1] == "steps":
                    controller_steps.append(float(split[-1]))
                if split[0] == "duration":
                    duration.append(float(split[-1]))

    print("**average velocity** = " + str(
            [round(np.sum(velocity[i]) / len(velocity[i]), 4) for i in range(0, len(files))]))
    print("**average velocity over all runs** = " + str(
            round(np.average([np.sum(velocity[i]) / len(velocity[i]) for i in range(0, len(files))]), 4)))

    print("**circle dimensions x** = " + str([round(i[0], 3) for i in x_dim]))
    print("**circle dimensions y** = " + str([round(i[0], 3) for i in y_dim]))
    print("")

    if calculate_cot:
        print("**duration in seconds** = " + str(duration))
        print("**duration in steps** = " + str(controller_steps))

        print("**total distance traveled** = " + str([round(i, 3) for i in distance]))
        print("**total power** = " + str(total_power_joint_torque))
        print("**velocity [m/step]** = " + str([d / s for d, s in zip(distance, controller_steps)]))

        cot_values_command = []
        cot_values_joint_torque = []
        if len(total_power_command) != len(controller_steps) != len(distance) != len(total_power_joint_torque):
            print("not the same number of values for distance ({}), power ({}) and controller steps ({})".format(
                    len(distance), len(total_power_command), len(controller_steps)))
            print("values for distance ({}), power ({}), controller steps ({})".format(
                    distance, total_power_command, controller_steps))
            print("could not calculate cot.")
        else:
            mass = 2.48  # kg
            for l in range(0, len(controller_steps)):
                com_vel = distance[l] / controller_steps[l]
                cot_values_command.append((total_power_command[l] / controller_steps[l]) / (mass * g * com_vel))
                cot_values_joint_torque.append(
                        (total_power_joint_torque[l] / controller_steps[l]) / (mass * g * com_vel))
            print("**cost of transport (joint torque)** = " + str([round(i, 3) for i in cot_values_joint_torque]))
            print("")

            print("**cost of transport (command)** = " + str([round(i, 3) for i in cot_values_command]))

    # print("first position = " + str(first_position) + " last position = " + str(last_position) + " distance = " + str(
    #        np.linalg.norm(np.array(last_position) - np.array(first_position))))

    if plot:
        ## --- for height plot
        # axs.figure()
        # axs.plot(X, Y)
        #
        ## -----

        plt.tick_params(labelsize=20)
        plt.grid(which='both')
        plt.axis('scaled')

        # axs.set_xlim(-1.5, 3)
        # axs.set_ylim(-0.5, 4.0)

        # lf_shoulder = np.matrix([0.1248, 0.06164]).T
        # axs.plot(lf_shoulder.T[:, 0], lf_shoulder.T[:, 1], 'xb')
        # lm_shoulder = np.matrix([0, 0.1034]).T
        # axs.plot(lm_shoulder.T[:, 0], lm_shoulder.T[:, 1], 'xb')
        # lr_shoulder = np.matrix([-0.1248, 0.06164]).T
        # axs.plot(lr_shoulder.T[:, 0], lr_shoulder.T[:, 1], 'xb')
        # rf_shoulder = np.matrix([0.1248, -0.06164]).T
        # axs.plot(rf_shoulder.T[:, 0], rf_shoulder.T[:, 1], 'xb')
        # rm_shoulder = np.matrix([0, -0.1034]).T
        # axs.plot(rm_shoulder.T[:, 0], rm_shoulder.T[:, 1], 'xb')
        # rr_shoulder = np.matrix([-0.1248, -0.06164]).T
        # axs.plot(rr_shoulder.T[:, 0], rr_shoulder.T[:, 1], 'xb')

        # axs.xaxis.set_minor_locator(ticker.MultipleLocator(base=0.5))
        # axs.yaxis.set_minor_locator(ticker.MultipleLocator(base=0.5))

        name = ""
        if safe_plot:
            plt.subplots_adjust(top=2, bottom=0, right=2, left=0, hspace=1, wspace=1)
            plt.margins(1, 1)
            if len(files) == 1:
                # split = [i.split("_") for i in files]
                split = re.findall(r"[^/_,]+", files[0], re.ASCII)
                print("split = " + str(split))
                name = "_".join(split[split.index("walknet"):])
                print("name = " + name)
                print("single file: " + "/home/jsimmering/plots_masterthesis/path/" + name + ".svg")
                plt.savefig("/home/jsimmering/plots_masterthesis/path/" + name + ".svg", bbox_inches='tight',
                        pad_inches=0, format='svg', transparent=True)
            else:
                split = [i.split("_") for i in files]
                speeds = [float(split[i][split[i].index("position") + 1][:-1]) for i in range(0, len(split))]
                directions = [float(split[i][split[i].index("position") + 2][:-3]) for i in range(0, len(split))]
                name = ""
                if min(speeds) != max(speeds):
                    name += str(min(speeds)) + "-to-" + str(max(speeds)) + "speed"
                else:
                    name += str(min(speeds)) + "speed"
                if min(directions) != max(directions):
                    name += str(min(directions)) + "-to-" + str(max(directions)) + "dir"
                else:
                    name += str(max(directions)) + "dir"
                name += "_"
                name += "_".join(split[0][split[0].index("position") + 3:])
                print("multiple files: " + "/home/jsimmering/plots_masterthesis/path/" + name + ".svg")
                plt.savefig("/home/jsimmering/plots_masterthesis/path/" + name + ".svg", bbox_inches='tight',
                        pad_inches=0, format='svg', transparent=True)

            # sc.Figure("15.1cm", "15.95cm",
            #         # plt.rcParams["figure.figsize"][0], plt.rcParams["figure.figsize"][1],
            #         sc.Panel(sc.SVG("/home/jsimmering/plots_masterthesis/body.svg").scale(0.00475).move(5.815, 13.275)),
            #         sc.Panel(sc.SVG("/home/jsimmering/plots_masterthesis/path/" + name + ".svg").scale(0.022))
            # ).save("/home/jsimmering/plots_masterthesis/path/robot_" + name + ".svg")
            # SVG("/home/jsimmering/plots_masterthesis/path/robot_" + name + ".svg")
            # axs.imshow(img, alpha=0.5, aspect='equal', extent=(-0.1378, 0.1378, -0.1164, 0.1164))
        else:
            plt.show()
            # pass

        if plot_heigth:
            plt.figure()
            plt.tick_params(labelsize=20)
            plt.grid(which='both')
            # plt.axis('scaled')
            plt.ylim(0.0, 0.1)
            plt.ylabel("body height [m]", fontsize=20)
            plt.xlabel("time [s]", fontsize=20)
            # plt.axis('scaled')
            for j in range(0, len(files)):
                # print("z[{}] = {}".format(j, Z[j]))
                plt.plot(Z[j], label=j)
            # print(sorted(Z, key=float, reverse=True))
            # print(type(Z[0]))
            import itertools
            # print("Z = " + str(Z))
            heights = list(itertools.chain.from_iterable(Z[1:len(Z)]))
            if len(heights) == 0:
                heights = Z[0]
            # print("heights = " + str(heights))
            last = heights[0]
            idx = 0
            for i in range(1, len(Z)):
                if heights[i] < last:
                    idx = i
                else:
                    break
            # print("idx = " + str(idx))
            #z_pos = heights[5: len(heights)]

            if print_height:
                print(sorted(heights, key=float, reverse=True))
            print("init height = " + str(Z[0][1]))
            plt.legend([str(n + 1) for n in range(0, len(files))],
                    # [split[i][split[i].index("position") + 1] + "_" + split[i][split[i].index("position") + 2] for i in
                    #  range(0, len(split))],
                    loc='lower right')
            print("**height criterion:** " + str([sum(i < 0.045 for i in files) for files in Z]))
            plt.axhline(y=0.045, color='r', linestyle='-')
            plt.axhline(y=0.088, color='b', linestyle='-')
            if safe_plot:
                plt.savefig("/home/jsimmering/plots_masterthesis/height/" + name + ".svg", bbox_inches='tight',
                        pad_inches=0, format='svg')
                print("height file: " + "/home/jsimmering/plots_masterthesis/height/" + name + ".svg")

        if not safe_plot:
            plt.show()
