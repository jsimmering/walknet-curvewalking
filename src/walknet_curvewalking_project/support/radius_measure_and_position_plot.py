#!/usr/bin/env python3
import math
import os
import re
import sys

import PyKDL
import matplotlib.pyplot as plt
import numpy
import numpy as np
from matplotlib import ticker


# https://stackoverflow.com/questions/55816902/finding-the-intersection-of-two-circles
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


def handle_line(configuration, run, line, last_position, last_time, distance, velocity, positions, quaternions,
                start=None):  # X, Y, Z,
    line = line.rstrip("\n")
    values = [float(s) for s in line.split(";")]

    if start is None:
        # save start position of this run to be able to use relative positions
        start = [values[1], values[2], values[3]]
        # print("configuration {} run {} start position = {}".format(configuration, run, start))

    # save current relative position for position plot
    # X[configuration][run].append(round(values[1] - start[0], 4))
    # Y[configuration][run].append(round(values[2] - start[1], 4))
    # Z[configuration][run].append(round(values[3] - start[2], 4))

    # save position and quaternion for radius measure calculation (position not relative)
    # if line_number % (1000 * 10) == 0:
    # idea https://math.stackexchange.com/questions/285866/calculating-circle-radius-from-two-points-on-circumference-for-game-movement
    # execution https://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
    position = np.array([values[1], values[2], values[3]])
    positions[configuration][run].append(position)
    quaternions[configuration][run].append(PyKDL.Rotation.Quaternion(values[4], values[5], values[6], values[7]))

    if last_position is not None:
        dis = np.linalg.norm(np.array([values[1], values[2]]) - np.array(last_position))
        distance[configuration] += dis
        vel = dis / (values[0] - last_time)
        velocity[configuration].append(vel)
    last_position = [values[1], values[2]]
    last_time = values[0]

    return last_position, last_time, start


def calculate_radii(radii_number, quaternions, positions, radii, centroids, angles):
    relative_angle_z = (quaternions[radii_number + 1] * quaternions[radii_number].Inverse()).GetRPY()[2]
    angles.append(relative_angle_z)
    if len(quaternions) > radii_number + radius_measure_time_step:
        if radius_measure_time_step != 1:
            relative_angle_z = \
                (quaternions[radii_number + radius_measure_time_step] * quaternions[radii_number].Inverse()).GetRPY()[2]
        radius = np.linalg.norm(
                positions[radii_number + radius_measure_time_step] - positions[radii_number]) / math.sqrt(
                2 * (1 - math.cos(relative_angle_z)))
        radii.append(radius)
        (x1, y1, x2, y2) = get_intersections(positions[radii_number][0], positions[radii_number][1], radius,
                positions[radii_number + radius_measure_time_step][0],
                positions[radii_number + radius_measure_time_step][1], radius)
        centroids.append(np.array([x2, y2]))
        return x2, y2
    else:
        return None, None


def plot_radius_boxplot(files, mean_radii, radii, axis_radius):
    split_label = [re.findall(r"[^/_,]+", files[j][0], re.ASCII) for j in range(0, len(files))]
    # print("split lable = " + str(split_label))
    boxplot_label = [label[-6:-4] for label in split_label]
    # print("boxplot_label = " + str(boxplot_label))
    if plot_relative_radius:
        relative_radii = []
        for i in range(0, len(files)):
            # todo test
            relative_radii.append([(radius - mean_radii[i]) / mean_radii[i] for radius in radii[i]])
        # axis_radius.boxplot(relative_radii, labels=['radii line ' + str(j) for j in range(0, len(files))])
        axis_radius.boxplot(relative_radii, labels=["_".join(boxplot_label[j]) for j in range(0, len(files))])
    else:
        # axis_radius.boxplot(radii, labels=['radii line ' + str(j) for j in range(0, len(files))])
        axis_radius.boxplot(radii, labels=["_".join(boxplot_label[j]).replace('dir', 'rad').replace('s_', 'm/s_') for j in range(0, len(files))])
        # control y axis limits, not working!
        # axis_radius.set_ylim(-0.4, 0.4)
        # axis_radius.set(ylim=(-0.4, 0.4))
        # axis_radius.xtics(rotation=90)


def plot_angle_boxplot(files, mean_angle, angles, axis_angle):
    split_label = [re.findall(r"[^/_,]+", files[j][0], re.ASCII) for j in range(0, len(files))]
    # print("split lable = " + str(split_label))
    boxplot_label = [label[-6:-4] for label in split_label]
    # print("boxplot_label = " + str(boxplot_label))
    relative_angle = []
    for i in range(0, len(files)):
        # todo test
        relative_angle.append([(angle - mean_angle[i]) / mean_angle[i] for angle in angles[i]])
    # axis_radius.boxplot(relative_angle, labels=['radii line ' + str(j) for j in range(0, len(files))])
    axis_angle.boxplot(relative_angle, labels=["_".join(boxplot_label[j]).replace('dir', 'rad').replace('s_', 'm/s_') for j in range(0, len(files))])


def plot_position_and_radius(files, axis_position=None, axis_radius=None, axis_angle=None):
    colors = plt.get_cmap("tab10")
    circle_colors = ['#0080ff', '#ffb763', '#80ff00', '#ff80ff', '#80ffff', '#ffff80', '#ffb05d', '#ac4f35', '#49e0b8']
    circle_count = 0

    # distance = [[0 for _ in configurations] for configurations in files]
    distance = [0 for _ in files]
    # X, Y, Z = [], [], []
    velocity = []
    # x_dim, y_dim = [], []
    positions, quaternions, radii, angles, centroids = [], [], [], [], []
    plot_lines_position = []
    # check if X, Y, Z can be replaced with positions
    for i in range(0, len(files)):
        # X.append([])
        # Y.append([])
        # Z.append([])
        velocity.append([])
        # x_dim.append([])
        # y_dim.append([])
        positions.append([])
        quaternions.append([])
        radii.append([])
        angles.append([])
        centroids.append([])
        for m in range(0, len(files[i])):
            # X[i].append([])
            # Y[i].append([])
            # Z[i].append([])
            # x_dim.append([])
            # y_dim.append([])
            positions[i].append([])
            quaternions[i].append([])
            # radii.append([])
            # angles.append([])
            # centroids.append([])

    legend = None
    current_speed = None
    current_dir = None
    current_color = 0

    for configuration_number in range(0, len(files)):
        # print("configuration = " + str(configuration_number))

        for run_number in range(0, len(files[configuration_number])):
            # print("run " + str(run_number) + " in configuration " + str(configuration_number))
            first_line = True
            start = None
            last_position = None
            last_time = None
            line_number = 0
            used_lines = 0

            # print("file_name = " + str(files[configuration_number][run_number]))
            for line in open(files[configuration_number][run_number], 'r'):
                if first_line:
                    first_line = False
                else:
                    if line_number % 1000 == 0:
                        if start is not None:
                            last_position, last_time, start = handle_line(configuration_number, run_number, line,
                                    last_position, last_time, distance, velocity, positions, quaternions,
                                    start)  # X, Y, Z,
                        else:
                            last_position, last_time, start = handle_line(configuration_number, run_number, line,
                                    last_position, last_time, distance, velocity, positions, quaternions)  # X, Y, Z,
                        used_lines += 1

                    line_number += 1
            # print("effective lines = " + str(used_lines))
            # print("#of quaternions[{}] = {}".format(configuration_number,
            #       len(quaternions[configuration_number][run_number])))

        radii_label = False
        # print("quaternions len = " + str(len(quaternions)))
        for i in range(0, len(quaternions[configuration_number])):
            # print("quaternions[{}] len = {}".format(configuration_number, len(quaternions[configuration_number])))
            # print("i = {}".format(i))
            for run_number in range(0, len(quaternions[configuration_number][i]) - 1):
                centroid_x, centroid_y = calculate_radii(run_number, quaternions[configuration_number][i],
                        positions[configuration_number][i], radii[configuration_number],
                        centroids[configuration_number], angles[configuration_number])

                if plot and plot_position and centroid_x is not None and centroid_y is not None:
                    line, = axis_position.plot([centroid_x], [centroid_y], 'x',
                            color=circle_colors[circle_count % len(circle_colors)],
                            label="centroids for configuration " + str(configuration_number), zorder=0)
                    if not radii_label:
                        plot_lines_position.append(line)
                        radii_label = True
        #print("#of angles[{}] = {}".format(configuration_number, len(angles[configuration_number])))
        #print("#of radii[{}] = {}".format(configuration_number, len(radii[configuration_number])))

        if plot and plot_position:
            mean_x = float(np.mean([centroid[0] for centroid in centroids[configuration_number]]))
            mean_y = float(np.mean([centroid[1] for centroid in centroids[configuration_number]]))
            # std_x = float(round(np.std([centroid[0] for centroid in centroids[configuration_number]]), 3))
            # std_y = float(round(np.std([centroid[1] for centroid in centroids[configuration_number]]), 3))
            dis = sum([numpy.linalg.norm(numpy.array([mean_x - centroid[0], mean_y - centroid[1]])) for centroid in centroids[configuration_number]])
            average_dis = dis/len(centroids[configuration_number])
            print("mean circle center position = x: {}, y : {}, a_dist: {}".format(round(mean_x, 4), round(mean_y, 4), round(average_dis, 4)))
            if plot_mean_circles:
                circle1 = plt.Circle((mean_x, mean_y), np.mean(radii[configuration_number]),
                        color=circle_colors[circle_count % len(circle_colors)],
                        fill=False, label="mean circle for configuration " + str(configuration_number), zorder=1)
                plot_lines_position.append(circle1)
                axis_position.add_artist(circle1)
            line, = axis_position.plot([mean_x], [mean_y], '.', color=colors(current_color),
                    label="mean centroid for configuration " + str(configuration_number), zorder=2)
            plot_lines_position.append(line)
            circle_count += 1

        for run_number in range(0, len(files[configuration_number])):
            if plot and plot_position:
                # split = re.findall(r"[^/_,]+", files[0], re.ASCII)
                split_control = re.findall(r"[^/_,]+", files[configuration_number][0], re.ASCII)
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

                x_positions = [points[0] for points in positions[configuration_number][run_number]]
                y_positions = [points[1] for points in positions[configuration_number][run_number]]
                # print("positions x = " + str(x_positions))
                # print("positions y = " + str(y_positions))

                line, = axis_position.plot(x_positions, y_positions, color=colors(current_color), linestyle='-',
                        label=label, zorder=2)
                plot_lines_position.append(line)
                if not controll_colors and not controll_colors_dir:
                    current_color += 1

                legend = axis_position.legend(handles=plot_lines_position, bbox_to_anchor=(0.5, -0.1),
                        loc='upper center', ncol=len(files))  # , ncol=1, mode="expand", borderaxespad=0.)
                axis_position.add_artist(legend)
                # print("")

    print("")
    print("**average velocity** = " + str(
            [round(np.sum(velocity[i]) / len(velocity[i]), 4) for i in range(0, len(files))]))

    # print("**radii** = " + str(radii))
    # print("**circle dimensions x** = " + str([round(i[0], 3) for i in x_dim]))
    # print("**circle dimensions y** = " + str([round(i[0], 3) for i in y_dim]))

    mean_radii = [np.mean(i) for i in radii]
    mean_angles = [np.mean(i) for i in angles]
    if len(files[0]) > 1:
        print("**mean of radii per configuration** = " + str([round(i, 3) for i in mean_radii]))
        # print("**average mean of radii per configuration** = " + str(np.average(np.array([round(i, 3) for i in mean_radii]))))
        print("**std of radii per configuration** = " + str([round(np.std(i), 3) for i in radii]))
        print("**mean of angles per configuration** = " + str([round(i, 3) for i in mean_angles]))
        # print("**average mean of angles per configuration** = " + str(np.average(np.array([round(i, 3) for i in mean_angles]))))
        print("**std of angles per configuration** = " + str([round(np.std(i), 3) for i in angles]))
        print("")
        print("distance traveled = " + str([round(i, 3) for i in distance]))
        print("")
    else:
        print("**mean of radii** = " + str([round(i, 3) for i in mean_radii]))
        print("**average mean of radii** = " + str(np.average(np.array([round(i, 3) for i in mean_radii]))))
        print("**std of radii** = " + str([round(np.std(i), 3) for i in radii]))
        print("**mean of angles** = " + str([round(i, 3) for i in mean_angles]))
        print("**average mean of angles** = " + str(np.average(np.array([round(i, 3) for i in mean_angles]))))
        print("**std of angles** = " + str([round(np.std(i), 3) for i in angles]))
        print("")
        print("distance traveled (all runs per configuration together) = " + str([round(i, 3) for i in distance]))
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
        plot_radius_boxplot(files, mean_radii, radii, axis_radius)

    if plot and plot_angle:
        plot_angle_boxplot(files, mean_angles, angles, axis_angle)

    return axis_position, axis_radius, axis_angle, legend


def save_plot():
    if plot_position:
        pos_plot_path = "/home/jsimmering/plots_masterthesis/path/path_w_radius_" + name + ".svg"
        print("file path: " + pos_plot_path)
        fig_pos.savefig(pos_plot_path, bbox_inches='tight', bbox_extra_artists=(pos_plot_legend,), pad_inches=0,
                format='svg')
    if plot_radius:
        rad_plot_path = "/home/jsimmering/plots_masterthesis/radius/radius_" + name + ".svg"
        print("file path: " + rad_plot_path)
        fig_rad.savefig(rad_plot_path, bbox_inches='tight', pad_inches=0, format='svg')
    if plot_radius:
        angle_plot_path = "/home/jsimmering/plots_masterthesis/angle/angle_" + name + ".svg"
        print("file path: " + angle_plot_path)
        fig_angle.savefig(angle_plot_path, bbox_inches='tight', pad_inches=0, format='svg')


if __name__ == '__main__':
    if len(sys.argv) >= 2:
        plot = True
        safe_plot = True
        plot_position = True
        plot_radius = True
        plot_angle = True
        plot_relative_radius = True
        plot_mean_circles = True
        radius_measure_time_step = 5
        controll_colors = False
        controll_colors_dir = False

        file_names = None
        if sys.argv[1] == "-dir":
            file_names = [[sys.argv[2] + "/" + x] for x in os.listdir(sys.argv[2])]
            [file_names[i].sort(key=lambda x: os.path.getmtime(x), reverse=False) for i in range(0, len(file_names))]
            print(file_names)
        elif sys.argv[1] == "-c":
            dirs = os.listdir(sys.argv[2] + "/")
            dirs = sorted(dirs)
            if '0.0dir' in dirs:
                dirs.remove('0.0dir')
            # dirs.sort(key=lambda x: os.path.getmtime(sys.argv[2] + "/" + x), reverse=False)
            print(dirs)
            file_names = [[sys.argv[2] + "/" + path + "/position/" + x for x in os.listdir(sys.argv[2] + "/" + path + "/position/")]
                          for path in dirs]
            # print(file_names)
            full_paths = [[re.findall(r"[^/_,]+", file_names[j][i], re.ASCII) for i in range(0, len(file_names[j]))] for
                          j in range(0, len(file_names))]
            file_names_only = [[label[label.index("walknet"):] for label in labels] for labels in full_paths]
            print("file_names = " + str(
                    [["_".join(run) for run in configurations] for configurations in file_names_only]))
        elif len(sys.argv) == 2:
            file_names = [[sys.argv[1]]]
        else:
            print("wrong parameters provide data-filename, -dir directory or -c with top directory with files")
            exit()

        if plot_position:
            fig_pos, axs_pos = plt.subplots()
            axs_pos.set_title(
                    'path of robot including path centroids (points ' + str(radius_measure_time_step) + ' sec apart)')
            axs_pos.grid()
            axs_pos.xaxis.set_major_locator(ticker.MultipleLocator(base=0.5))
            axs_pos.tick_params(labelsize=12)
            # plt.subplots_adjust(top=0.9, bottom=0.1, left=-0.15, hspace=1, wspace=1)
        else:
            axs_pos = None
            fig_pos = None

        if plot_radius:
            fig_rad, axs_rad = plt.subplots()
            axs_rad.set_title(
                    'relative radius of curvature boxplot for positions' + str(radius_measure_time_step) + ' sec apart')
            axs_rad.grid()
            axs_rad.set_ylabel('relative radius', fontsize=14)
            axs_rad.xaxis.set_major_locator(ticker.MultipleLocator(base=0.5))
            axs_rad.tick_params(labelsize=12)
            if sys.argv[1] == "-dir":
                # axs_rad.set_ylim(-1.0, 1.0)
                pass
            else:
                # axs_rad.set_ylim(-35, 15)
                pass
        else:
            axs_rad = None
            fig_rad = None

        if plot_angle:
            fig_angle, axs_angle = plt.subplots()
            # axs_angle.set_title('relative angle boxplot for consecutive points')
            axs_angle.grid()
            axs_angle.set_ylabel('relation of angle difference and mean angle', fontsize=16)
            axs_angle.xaxis.set_major_locator(ticker.MultipleLocator(base=0.5))
            axs_angle.tick_params(labelsize=12)
            if sys.argv[1] == "-dir":
                # axs_angle.set_ylim(-1.25, 1.25)
                pass
            else:
                # axs_angle.set_ylim(-4.5, 2.25)
                # axs_angle.set_ylim(-35, 15)
                pass
        else:
            axs_angle = None
            fig_angle = None

        axs_pos, axs_rad, axs_angle, pos_plot_legend = plot_position_and_radius(file_names, axs_pos, axs_rad, axs_angle)

        if plot_position:
            axs_pos.set_xlim(-1.5, 2.5)
            axs_pos.set_ylim(-0.5, 4.0)

        if plot_radius:
            fig_rad.autofmt_xdate()

        if plot_angle:
            fig_angle.autofmt_xdate()

        if safe_plot:
            if len(file_names) == 1:
                split = re.findall(r"[^/_,]+", file_names[0][0], re.ASCII)
                print("split = " + str(split))
                name = ""
                name += "_".join(split[split.index("walknet"):])
                print("single file: ")
                save_plot()
            else:
                split = [i.split("_") for i in file_names[0]]
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
                    name += str(min(directions)) + "dir"
                name += "_"
                name += "_".join(split[0][split[0].index("position") + 3:])
                print("multiple files:")
                save_plot()
        else:
            plt.show()
