#!/usr/bin/env python3
import os
import sys
import re
import math
import PyKDL

import matplotlib.pyplot as plt
import numpy as np

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


def handle_line(configuration, line, line_number, start_x, start_y, start_z, first_position, X, Y, Z, x_max, x_min, y_max,
                y_min, last_position, distance, velocity, positions, quaternions, last_time):
    line = line.rstrip("\n")
    values = [float(s) for s in line.split(";")]
    if not start_x[configuration]:
        start_x[configuration].append(values[1])
        start_y[configuration].append(values[2])
        start_z[configuration].append(values[3])

    if first_position is None:
        first_position = [values[1], values[2]]

    X[configuration].append(round(values[1] - start_x[configuration][0], 4))
    Y[configuration].append(round(values[2] - start_y[configuration][0], 4))
    Z[configuration].append(round(values[3] - start_z[configuration][0], 4))

    if values[1] > x_max:
        x_max = values[1]
    if values[1] < x_min:
        x_min = values[1]
    if values[2] > y_max:
        y_max = values[2]
    if values[2] < y_min:
        y_min = values[2]

    if last_position is not None:
        dis = np.linalg.norm(np.array([values[1], values[2]]) - np.array(last_position))
        distance[configuration] += dis
        vel = dis / (values[0] - last_time)
        velocity[configuration].append(vel)
    last_position = [values[1], values[2]]
    last_time = values[0]

    #if line_number % (1000 * 10) == 0:
    # idea https://math.stackexchange.com/questions/285866/calculating-circle-radius-from-two-points-on-circumference-for-game-movement
    # execution https://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
    positions[configuration].append(np.array([values[1], values[2], values[3]]))
    quaternions[configuration].append(PyKDL.Rotation.Quaternion(values[4], values[5], values[6], values[7]))

    return first_position, last_position, last_time, x_max, x_min, y_max, y_min


def calculate_radii(radii_number, quaternions, positions, radii, centroids):
    relative_angle_z = (quaternions[radii_number + 1] * quaternions[radii_number].Inverse()).GetRPY()[2]
    radius = np.linalg.norm(positions[radii_number + 1] - positions[radii_number]) / math.sqrt(
            2 * (1 - math.cos(relative_angle_z)))
    radii.append(radius)
    (x1, y1, x2, y2) = get_intersections(positions[radii_number][0], positions[radii_number][1], radius,
            positions[radii_number + 1][0], positions[radii_number + 1][1], radius)
    centroids.append(np.array([x2, y2]))
    return x2, y2


def plot_radius_boxplot(files, mean_radii, radii, axis_radius):
    split_label = [re.findall(r"[^/_,]+", files[j][0], re.ASCII) for j in range(0, len(files))]
    # print("split lable = " + str(split_label))
    boxplot_label = [lable[lable.index("walknet") + 2:] for lable in split_label]
    # print("boxplot_label = " + str(boxplot_label))
    if plot_relative_radius:
        relative_radii = []
        for i in range(0, len(files)):
            # todo test
            relative_radii.append([(radius - mean_radii[i])/mean_radii[i] for radius in radii[i]])
        # axis_radius.boxplot(relative_radii, labels=['radii line ' + str(j) for j in range(0, len(files))])
        axis_radius.boxplot(relative_radii, labels=["_".join(boxplot_label[j]) for j in range(0, len(files))])
    else:
        # axis_radius.boxplot(radii, labels=['radii line ' + str(j) for j in range(0, len(files))])
        axis_radius.boxplot(radii, labels=["_".join(boxplot_label[j]) for j in range(0, len(files))])
        # control y axis limits, not working!
        # axis_radius.set_ylim(-0.4, 0.4)
        # axis_radius.set(ylim=(-0.4, 0.4))
        # axis_radius.xtics(rotation=90)


def position_plot(files, axis_position=None, axis_radius=None):
    colors = plt.get_cmap("tab10")
    # colors = ['#0000FF', '#FF0000', '#008000', '#ff00ff', '#00ffff', '#ffff00', '#ff8000', '#78281f', '#1abc9c']
    circle_colors = ['#0080ff', '#ffb763', '#80ff00', '#ff80ff', '#80ffff', '#ffff80', '#ffb05d', '#ac4f35', '#49e0b8']
    circle_count = 0

    distance = [0 for _ in range(0, len(files))]
    X, Y, Z, start_x, start_y, start_z, velocity, x_dim, y_dim = [], [], [], [], [], [], [], [], []
    positions = []
    quaternions = []
    radii = []
    angles = []
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
        angles.append([])
        centroids.append([])

    legend = None
    current_speed = None
    current_dir = None
    current_color = 0

    for j in range(0, len(files)):
        last_position = None
        first_position = None
        last_time = None
        x_min = float('inf')
        x_max = float('-inf')
        y_min = float('inf')
        y_max = float('-inf')

        for run in files[j]:
            first_line = True
            line_number = 0
            if len(sys.argv) > 2:
                # file_name = sys.argv[2] + "/" + str(run)
                file_name = str(run)
            else:
                file_name = str(run)

            # print("configuration = " + str(j))
            for line in open(file_name, 'r'):
                if first_line:
                    first_line = False
                    pass
                else:
                    if line_number % 1000 == 0:
                        first_position, last_position, last_time, x_max, x_min, y_max, y_min = handle_line(j, line,
                                line_number, start_x, start_y, start_z, first_position, X, Y, Z,
                                x_max, x_min, y_max, y_min, last_position, distance, velocity, positions,
                                quaternions, last_time)

                    line_number += 1

            # print(
            #         "j = {}, \nstart_x = {}, \nstart_y = {}, \nstart_z = {}, \nX = {}, \nY = {}, \nZ = {}, \ndistance = {}, \nvelocity = {}, \npositions = {}, \nquaternions = {}\n".format(
            #                 j, start_x, start_y, start_z, X, Y, Z, distance, velocity, positions, quaternions))

            radii_label = False
            for i in range(0, len(quaternions[j]) - 1):
                centroid_x, centroid_y = calculate_radii(i, quaternions[j], positions[j], radii[j], centroids[j])

                # print("centroid_x = {}, centroid_y = {}".format(centroid_x, centroid_y))
                if plot and plot_position:
                    line, = axis_position.plot([centroid_x], [centroid_y], 'x', color=circle_colors[circle_count % len(circle_colors)],
                            label="radii line " + str(j), zorder=0)
                    if not radii_label:
                        plot_lines_position.append(line)
                        radii_label = True

            if plot and plot_position:
                mean_x = float(np.mean([centroid[0] for centroid in centroids[j]]))
                mean_y = float(np.mean([centroid[1] for centroid in centroids[j]]))
                if plot_mean_circles:
                    circle1 = plt.Circle((mean_x, mean_y), np.mean(radii[j]), color=circle_colors[circle_count % len(circle_colors)],
                            fill=False, label="circle line " + str(j), zorder=1)
                    plot_lines_position.append(circle1)
                    axis_position.add_artist(circle1)
                line, = axis_position.plot([mean_x], [mean_y], '.', color=colors(current_color),
                        label="mean radius line " + str(j), zorder=2)
                plot_lines_position.append(line)
                circle_count += 1

            x_dim[j].append(x_max - x_min)
            y_dim[j].append(y_max - y_min)
            if plot and plot_position:
                # print(files[j])
                split_control = re.findall(r"[^/_,]+", run, re.ASCII)
                if controll_colors:
                    speed = split_control[split_control.index("position") + 1]
                    if not current_speed:
                        current_speed = float(speed[:-1])
                    if current_speed != speed:
                        current_color += 1
                        current_speed = speed
                elif controll_colors_dir:
                    direction = split_control[split_control.index("position") + 2]
                    if not current_dir:
                        current_dir = float(direction[:-3])
                    if current_dir != direction:
                        current_color += 1
                        current_dir = direction

                label = split_control[split_control.index("position") + 3] + "_" + split_control[
                    split_control.index("position") + 4]
                # print("label = " + str(label))
                line, _ = axis_position.plot(X[j], Y[j], colors(current_color), linestyle='-', label=label, zorder=2)
                plot_lines_position.append(line)
                if not controll_colors and not controll_colors_dir:
                    current_color += 1

                legend = axis_position.legend(handles=plot_lines_position, bbox_to_anchor=(0.5, -0.1),
                        loc='upper center', ncol=len(files))  # , ncol=1, mode="expand", borderaxespad=0.)
                axis_position.add_artist(legend)

    print("")
    print("**average velocity** = " + str(
            [round(np.sum(velocity[i]) / len(velocity[i]), 4) for i in range(0, len(files))]))

    # print("**radii** = " + str(radii))
    print("**circle dimensions x** = " + str([round(i[0], 3) for i in x_dim]))
    print("**circle dimensions y** = " + str([round(i[0], 3) for i in y_dim]))
    mean_radii = [np.mean(i) for i in radii]
    print("**mean of radii** = " + str([round(i, 3) for i in mean_radii]))
    print("**std of radii** = " + str([round(np.std(i), 3) for i in radii]))
    print("")

    print("distance traveled = " + str([round(i, 3) for i in distance]))
    print("")

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
        # print("files = {}, \nmean_radii = {}, \nradii = {}, \naxis_radius = {}\n".format(files, mean_radii, radii, axis_radius))
        plot_radius_boxplot(files, mean_radii, radii, axis_radius)

    return axis_position, axis_radius, legend


def save_plot():
    if plot_position:
        pos_plot_path = "/home/jsimmering/plots_masterthesis/path/" + name + ".png"
        print("file path: " + pos_plot_path)
        fig_pos.savefig(pos_plot_path, bbox_inches='tight', bbox_extra_artists=(pos_plot_legend,), pad_inches=0)
    if plot_radius:
        rad_plot_path = "/home/jsimmering/plots_masterthesis/radius/" + name + ".png"
        print("file path: " + rad_plot_path)
        fig_rad.savefig(rad_plot_path, bbox_inches='tight', pad_inches=0)


if __name__ == '__main__':
    if len(sys.argv) >= 2:
        plot = True
        safe_plot = True
        plot_position = True
        plot_radius = True
        plot_relative_radius = True
        plot_mean_circles = True
        radius_measure_time_step = 5
        controll_colors = False
        controll_colors_dir = False

        file_names = None
        if sys.argv[1] == "-dir":
            file_names = [[x] for x in os.listdir(sys.argv[2])]
            [file_names[i].sort(key=lambda x: os.path.getmtime(sys.argv[2] + "/" + x), reverse=False) for i in
             range(0, len(file_names))]
            print(file_names)
        elif sys.argv[1] == "-c":
            dirs = os.listdir(sys.argv[2])
            dirs = sorted(dirs)
            if '0.0dir' in dirs:
                dirs.remove('0.0dir')
            # dirs.sort(key=lambda x: os.path.getmtime(sys.argv[2] + "/" + x), reverse=False)
            print(dirs)
            file_names = [[sys.argv[2] + path + "/position/" + x for x in os.listdir(sys.argv[2] + path + "/position/")]
                          for path in dirs]
            # print(file_names)
            full_paths = [[re.findall(r"[^/_,]+", file_names[j][i], re.ASCII) for i in range(0, len(file_names[j]))] for j in range(0, len(file_names))]
            file_names_only = [[label[label.index("walknet"):] for label in labels] for labels in full_paths]
            print("file_names = " + str([["_".join(label) for label in boxplt_label] for boxplt_label in file_names_only]))
        elif len(sys.argv) == 2:
            file_names = [[sys.argv[1]]]
        else:
            print("wrong parameters provide data-filename, -dir directory or -c with top directory with files")
            exit()

        if plot_position:
            fig_pos, axs_pos = plt.subplots()
            axs_pos.grid()
            axs_pos.xaxis.set_major_locator(ticker.MultipleLocator(base=0.5))
            axs_pos.tick_params(labelsize=12)
            plt.subplots_adjust(top=0.9, bottom=0.1, left=-0.15, hspace=1, wspace=1)
        else:
            axs_pos = None
            fig_pos = None

        if plot_radius:
            #axs_pos.set_xlim(-1.5, 2.5)
            fig_rad, axs_rad = plt.subplots()
            axs_rad.grid()
            axs_rad.xaxis.set_major_locator(ticker.MultipleLocator(base=0.5))
            axs_rad.tick_params(labelsize=12)
        else:
            axs_rad = None
            fig_rad = None

        if axs_rad and axs_pos:
            axs_pos, axs_rad, pos_plot_legend = position_plot(file_names, axs_pos, axs_rad)
        else:
            axs_pos, axs_rad, pos_plot_legend = position_plot(file_names)

        if plot_position:
            axs_pos.set_xlim(-1.5, 2.5)
            axs_pos.set_ylim(-0.5, 4.0)
            # axs_pos.yaxis.set_major_locator(ticker.MultipleLocator(base=0.5))

        if plot_radius:
            fig_rad.autofmt_xdate()

        if safe_plot:
            # plt.subplots_adjust(top=2, bottom=0, right=2, left=0, hspace=1, wspace=1)
            # plt.margins(1, 1)
            if len(file_names) == 1:
                split = re.findall(r"[^/_,]+", file_names[0][0], re.ASCII)
                print("split = " + str(split))
                name = "radius_"
                name += "_".join(split[split.index("walknet"):])
                print("single file: ")
                save_plot()
            else:
                split = [i.split("_") for i in file_names[0]]
                speeds = [float(split[i][split[i].index("position") + 1][:-1]) for i in range(0, len(split))]
                directions = [float(split[i][split[i].index("position") + 2][:-3]) for i in range(0, len(split))]
                name = "radius1_"
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
                print("multiple files:")
                save_plot()
        else:
            plt.show()
