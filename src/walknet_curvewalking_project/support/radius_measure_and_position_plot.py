#!/usr/bin/env python3
import os
import sys
import re
import math
import PyKDL

import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import numpy
import numpy as np
import tf.transformations as tf_trans

# https://stackoverflow.com/questions/55816902/finding-the-intersection-of-two-circles
from matplotlib import ticker


def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    # non intersecting
    if d > r0 + r1:
        return None
    # One circle within other
    if d < abs(r0 - r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(r0 ** 2 - a ** 2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return x3, y3, x4, y4


def position_plot(axis_position, axis_radius):
    colors = plt.get_cmap("tab10")
    # colors = ['#0000FF', '#FF0000', '#008000', '#ff00ff', '#00ffff', '#ffff00', '#ff8000', '#78281f', '#1abc9c']
    circle_colors = ['#0080ff', '#ffb763', '#80ff00', '#ff80ff', '#80ffff', '#ffff80', '#ffb05d', '#ac4f35', '#49e0b8']
    circle_count = 0

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

    distance = [0 for i in range(0, len(files))]
    X, Y, Z, start_x, start_y, start_z, velocity, x_dim, y_dim = [], [], [], [], [], [], [], [], []
    positions = []
    quaternions = []
    radii = []
    centroids = []
    plot_lines_position = []
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
        positions.append([])
        quaternions.append([])
        radii.append([])
        centroids.append([])

    legend = None
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
        if len(sys.argv) > 2:
            file_name = sys.argv[2] + "/" + str(files[j])
        else:
            file_name = str(files[j])
        for line in open(file_name, 'r'):
            if first_line:
                first_line = False
                pass
            else:
                if line_number % 1000 == 0:
                    line = line.rstrip("\n")
                    values = [float(s) for s in line.split(";")]
                    # print("time = " + str(values[0]))
                    # if start_x is None:
                    if not start_x[j]:
                        start_x[j].append(values[1])
                        # print("startx = " + str(start_x) + " j = " + str(j))
                        start_y[j].append(values[2])
                        start_z[j].append(values[3])

                    if first_position is None:
                        first_position = [values[1], values[2]]

                    # print("x = " + str(round(values[1],4)) + "y = " + str(round(values[2],4)) + "z = " + str(round(values[3],4)))
                    X[j].append(round(values[1] - start_x[j][0], 4))
                    Y[j].append(round(values[2] - start_y[j][0], 4))
                    Z[j].append(round(values[3] - start_z[j][0], 4))

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

                    if line_number % (1000 * 10) == 0:
                        # print("add quaternion")
                        # idea https://math.stackexchange.com/questions/285866/calculating-circle-radius-from-two-points-on-circumference-for-game-movement
                        # execution https://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
                        positions[j].append(np.array([values[1], values[2], values[3]]))
                        quaternions[j].append(PyKDL.Rotation.Quaternion(values[4], values[5], values[6], values[7]))

                line_number += 1

        radii_label = False
        for i in range(0, len(quaternions[j]) - 1):
            # print("add radius")
            relative_angle_z = (quaternions[j][i + 1] * quaternions[j][i].Inverse()).GetRPY()[2]
            radius = np.linalg.norm(positions[j][i + 1] - positions[j][i]) / math.sqrt(
                    2 * (1 - math.cos(relative_angle_z)))
            radii[j].append(radius)
            (x1, y1, x2, y2) = get_intersections(positions[j][i][0], positions[j][i][1], radius, positions[j][i + 1][0],
                    positions[j][i + 1][1], radius)
            centroids[j].append(np.array([x2, y2]))

            if plot and plot_position:
                # print("i = {}, (len(quaternions[j])/2) = {}, % = {}".format(i, (len(quaternions[j])//2), i % (len(quaternions[j])//2)))
                # if i % (len(quaternions[j])//2) == 0:
                #     circle1 = plt.Circle((x2, y2), radius, color=circle_colors[circle_count], fill=False)
                #     ax.add_artist(circle1)
                #     plt.plot([positions[j][i][0], x2, positions[j][i+1][0]], [positions[j][i][1], y2, positions[j][i+1][1]], color=circle_colors[circle_count])
                #     plt.plot([positions[j][i][0], x2, positions[j][i + 1][0]],
                #             [positions[j][i][1], y2, positions[j][i + 1][1]], 'x', color=circle_colors[circle_count])
                #     circle_count += 1
                # else:
                line, = axis_position.plot([x2], [y2], 'x', color=circle_colors[circle_count],
                        label="radii line " + str(j),
                        zorder=0)
                if not radii_label:
                    plot_lines_position.append(line)
                    radii_label = True

        if plot and plot_position:
            mean_x = float(np.mean([centroid[0] for centroid in centroids[j]]))
            mean_y = float(np.mean([centroid[1] for centroid in centroids[j]]))
            if plot_mean_circles:
                circle1 = plt.Circle((mean_x, mean_y), np.mean(radii[j]), color=circle_colors[circle_count],
                        fill=False, label="circle line " + str(j), zorder=1)
                plot_lines_position.append(circle1)
                axis_position.add_artist(circle1)
            # plt.plot([positions[j][i][0], x2, positions[j][i + 1][0]],
            #        [positions[j][i][1], y2, positions[j][i + 1][1]], color=circle_colors[circle_count])
            # plt.plot([positions[j][i][0], x2, positions[j][i + 1][0]],
            #        [positions[j][i][1], y2, positions[j][i + 1][1]], 'x', color=circle_colors[circle_count])
            line, = axis_position.plot([mean_x], [mean_y], '.', color=colors(current_color),
                    label="mean radius line " + str(j), zorder=2)
            plot_lines_position.append(line)
            circle_count += 1

        x_dim[j].append(x_max - x_min)
        y_dim[j].append(y_max - y_min)
        if plot and plot_position:
            split = [i.split("_") for i in files]
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
                line, = axis_position.plot(X[j], Y[j], colors(current_color), linestyle='-',
                        label=split[j][split[j].index("position") + 1] + "_" + split[j][split[j].index("position") + 2],
                        zorder=2)
                plot_lines_position.append(line)
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
                line, = axis_position.plot(X[j], Y[j], colors(current_color), linestyle='-',
                        label=split[j][split[j].index("position") + 1] + "_" + split[j][split[j].index("position") + 2],
                        zorder=2)
                plot_lines_position.append(line)
            else:
                line, = axis_position.plot(X[j], Y[j], color=colors(current_color), linestyle='-',
                        label=split[j][split[j].index("position") + 1] + "_" + split[j][split[j].index("position") + 2],
                        zorder=2)
                plot_lines_position.append(line)
                current_color += 1
            # split = [i.split("_") for i in files]
            # plt.legend([i.split("_")[2] + "_" + i.split("_")[3] for i in files], loc='lower right')
            legend = axis_position.legend(handles=plot_lines_position, bbox_to_anchor=(0.5,-0.1),
                    loc='upper center', ncol=len(files)) # borderaxespad=0.) #, ncol=1, mode="expand", borderaxespad=0.)
            # plt.legend(handles=[p1, p2], title='title', bbox_to_anchor=(1.05, 1), loc='upper left', prop=fontP)
            # plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',
            #            ncol=2, mode="expand", borderaxespad=0.)
            axis_position.add_artist(legend)
            # plt.legend(
            #        [split[i][split[i].index("position") + 1] + "_" + split[i][split[i].index("position") + 2] for i in
            #         range(0, len(split))], loc='lower right')

    print("**average velocity** = " + str(
            [round(np.sum(velocity[i]) / len(velocity[i]), 4) for i in range(0, len(files))]))

    # print("**radii** = " + str(radii))
    print("**circle dimensions x** = " + str([round(i[0], 3) for i in x_dim]))
    print("**circle dimensions y** = " + str([round(i[0], 3) for i in y_dim]))
    print("**mean of radii** = " + str([round(np.mean(i), 3) for i in radii]))
    print("**std of radii** = " + str([round(np.std(i), 3) for i in radii]))

    print("distance traveled = " + str([round(i, 3) for i in distance]))
    # print("first position = " + str(first_position) + " last position = " + str(last_position) + " distance = " + str(
    #        np.linalg.norm(np.array(last_position) - np.array(first_position))))

    if plot and plot_position:
        # --- for height plot
        # plt.figure()
        # plt.plot(X, Y)

        # plt.figure()
        # plt.plot(Z)
        # -----

        axis_position.set_xlim(-1.5, 2.5)
        axis_position.set_ylim(-0.5, 4.0)
        axis_position.axis('scaled')

    if plot and plot_radius:
        axis_radius.boxplot(radii, labels=['radii line ' + str(j) for j in range(0, len(files))])
        # axis[1].set_xlim(0.5, len(radii) + 0.5)
        # plt.gca().set_aspect('equal', adjustable='box')
    return axis_position, axis_radius, legend, files


if __name__ == '__main__':
    if len(sys.argv) >= 2:
        plot = True
        safe_plot = False
        plot_position = True
        plot_radius = True
        plot_mean_circles = True
        controll_colors = False
        controll_colors_dir = False

        if plot_position:
            fig_pos, axs_pos = plt.subplots()
            axs_pos.grid()
            axs_pos.xaxis.set_major_locator(ticker.MultipleLocator(base=0.5))
            axs_pos.tick_params(labelsize=12)
            plt.subplots_adjust(top=0.9, bottom=0.1, left=-0.15, hspace=1, wspace=1)

        if plot_radius:
            fig_rad, axs_rad = plt.subplots()
            axs_rad.grid()
            axs_rad.xaxis.set_major_locator(ticker.MultipleLocator(base=0.5))
            axs_rad.tick_params(labelsize=20)

        # plt.subplots_adjust(top=0.9, right=0.9, hspace=0, wspace=0.5)
        # fig.tight_layout()
        # for ax in axs:
        #     ax.grid()
        #     # loc = ticker.MultipleLocator(base=0.5)  # this locator puts ticks at regular intervals
        #     ax.xaxis.set_major_locator(ticker.MultipleLocator(base=0.5))

        axs_pos, axs_rad, legend, files = position_plot(axs_pos, axs_rad)

        if plot_position:
            axs_pos.set_xlim(-1.5, 2.5)
            axs_pos.set_ylim(-0.5, 4.0)
            # axs_pos.yaxis.set_major_locator(ticker.MultipleLocator(base=0.5))

        # if plot_position:
        #     axs_rad.yaxis.set_major_locator(ticker.MultipleLocator(base=0.05))

        # asp = numpy.diff(axs[0].get_xlim())[0] / numpy.diff(axs[0].get_ylim())[0]
        # axs[0].set_aspect(asp)

        if safe_plot:
            # plt.subplots_adjust(top=2, bottom=0, right=2, left=0, hspace=1, wspace=1)
            # plt.margins(1, 1)
            if len(files) == 1:
                # split = [i.split("_") for i in files]
                split = re.findall(r"[^/_,]+", files[0], re.ASCII)
                print("split = " + str(split))
                name = "radius_"
                name += "_".join(split[split.index("walknet"):])
                print("name = " + name)
                if plot_position:
                    print("single file: " + "/home/jsimmering/plots_masterthesis/path/" + name + ".pdf")
                    fig_pos.savefig("/home/jsimmering/plots_masterthesis/path/" + name + ".png", bbox_inches='tight',
                            bbox_extra_artists=(legend,), pad_inches=0)
                if plot_radius:
                    print("single file: " + "/home/jsimmering/plots_masterthesis/radius/" + name + ".pdf")
                    fig_rad.savefig("/home/jsimmering/plots_masterthesis/radius/" + name + ".png", bbox_inches='tight',
                            pad_inches=0)
            else:
                split = [i.split("_") for i in files]
                speeds = [float(split[i][split[i].index("position") + 1][:-1]) for i in range(0, len(split))]
                directions = [float(split[i][split[i].index("position") + 2][:-3]) for i in range(0, len(split))]
                name = "radius_"
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

                if plot_position:
                    print("multiple files: " + "/home/jsimmering/plots_masterthesis/path/" + name + ".png")
                    fig_pos.savefig("/home/jsimmering/plots_masterthesis/path/" + name + ".png", bbox_inches='tight',
                            bbox_extra_artists=(legend,), pad_inches=0)
                if plot_radius:
                    print("multiple files: " + "/home/jsimmering/plots_masterthesis/radius/" + name + ".png")
                    fig_rad.savefig("/home/jsimmering/plots_masterthesis/radius/" + name + ".png", bbox_inches='tight',
                            pad_inches=0)
        else:
            plt.show()
