#!/usr/bin/env python3
import re
import sys
from math import pi
import numpy

import matplotlib.pyplot as plt
import tf.transformations as tf_trans


def plot_orientation_data(ax0, ax1, ax2, start_time, stop_time):
    r = re.compile(".*position.*")
    files = list(filter(r.match, sys.argv))
    # files = None
    # if sys.argv[1] == "-dir":
    #     files = os.listdir(sys.argv[2])
    #     print(files)
    # elif len(sys.argv) == 2:
    #     files = [sys.argv[1]]
    # else:
    #     print("wrong parameters provide data-filename or -dir directory with files")
    #     exit()

    orientation_yaw, orientation_roll, orientation_pitch, orientation_diff, time = [], [], [], [], []
    for i in range(0, len(files)):
        orientation_yaw.append([])
        orientation_roll.append([])
        orientation_pitch.append([])
        orientation_diff.append([])
        time.append([])

    # plt.figure()
    # fig, axs = plt.subplots(2)

    for j in range(0, len(files)):
        walk_start_time = None
        first_line = True
        last_time = None
        last_roll_orientation = None
        initial_roll_orientation = None
        last_pitch_orientation = None
        initial_pitch_orientation = None
        last_yaw_orientation = None
        initial_yaw_orientation = None

        max_y_roll = -3.14
        min_y_roll = 3.14
        max_y_pitch = -3.14
        min_y_pitch = 3.14
        max_y_yaw = -3.14
        min_y_yaw = 3.14

        line_number = 0
        used_lines = 0
        file_name = str(files[j])
        # file_name = None
        # if len(sys.argv) > 2:
        #     file_name = sys.argv[2] + "/" + str(files[j])
        # else:
        #     file_name = str(files[j])

        for line in open(file_name, 'r'):
            if first_line:
                first_line = False
                pass
            else:
                # if line_number % 5 == 0:
                line = line.rstrip("\n")
                values = [float(s) for s in line.split(";")]
                # print("time = " + str(values[0]))
                if walk_start_time is None:
                    walk_start_time = values[0]
                values[0] -= walk_start_time
                if stop_time != 0:
                    if values[0] > stop_time:
                        break
                    if values[0] < start_time:
                        continue

                used_lines += 1

                angles = tf_trans.euler_from_quaternion([values[4], values[5], values[6], values[7]])
                # current_max = max(angles)
                # current_min = min(angles)
                # if current_max > max_y:
                #     max_y = current_max
                # if current_min < min_y:
                #     min_y = current_min

                # angle = None
                # if yaw:
                #     # angle = ((2 * pi) + angles[2]) % (2 * pi)
                #     angle = angles[2]
                # if pitch:
                #     # angle = ((2 * pi) + angles[1]) % (2 * pi)
                #     angle = angles[1]
                # if roll:
                #     # angle = ((2 * pi) + angles[0]) % (2 * pi)
                #     angle = angles[0]

                yaw = angles[2]
                if yaw > max_y_yaw:
                    max_y_yaw = yaw
                if yaw < min_y_yaw:
                    min_y_yaw = yaw
                if last_yaw_orientation is not None:
                    if initial_yaw_orientation is None:
                        initial_orientation = yaw
                    # orientation_yaw[j].append(angles[2] - initial_orientation)
                    orientation_yaw[j].append(yaw)
                    if abs(yaw - last_yaw_orientation) <= 0.00002:
                        orientation_diff[j].append(0)
                    else:
                        orientation_diff[j].append(yaw - last_yaw_orientation)
                    time[j].append(values[0])
                last_yaw_orientation = yaw

                pitch = angles[1]
                if pitch > max_y_pitch:
                    max_y_pitch = pitch
                if pitch < min_y_pitch:
                    min_y_pitch = pitch
                if last_pitch_orientation is not None:
                    if initial_pitch_orientation is None:
                        initial_pitch_orientation = pitch
                    # orientation_pitch[j].append(angles[2] - initial_orientation)
                    orientation_pitch[j].append(pitch)
                    if abs(pitch - last_pitch_orientation) <= 0.00002:
                        orientation_diff[j].append(0)
                    else:
                        orientation_diff[j].append(pitch - last_pitch_orientation)
                    # time[j].append(values[0])
                last_pitch_orientation = pitch

                roll = angles[0]
                if roll > max_y_roll:
                    max_y_roll = roll
                if roll < min_y_roll:
                    min_y_roll = roll
                if last_roll_orientation is not None:
                    if initial_roll_orientation is None:
                        initial_roll_orientation = roll
                    # orientation_roll[j].append(angles[2] - initial_orientation)
                    orientation_roll[j].append(roll)
                    if abs(roll - last_roll_orientation) <= 0.00002:
                        orientation_diff[j].append(0)
                    else:
                        orientation_diff[j].append(roll - last_roll_orientation)
                    # time[j].append(values[0])
                last_roll_orientation = roll

                line_number += 1

        print("used lines = " + str(used_lines) + " of total lines = " + str(line_number))
        # print("orientation_z = " + str(orientation_z))
        # print("time = " + str(time))
        # plt.plot(time[j], orientation_z[j])
        ax0.plot(time[j], orientation_roll[j])
        # ax0.title.set_text('roll')
        ax0.set_title('roll', fontsize=20)
        ax1.plot(time[j], orientation_pitch[j])
        # ax1.title.set_text('pitch')
        ax1.set_title('pitch', fontsize=20)
        ax2.plot(time[j], orientation_yaw[j])
        # ax2.title.set_text('yaw')
        ax2.set_title('yaw', fontsize=20)
        # axs[0].axis(ymax=initial_orientation + 0.05, ymin=initial_orientation - 0.15)
        # plt.plot(time[j], orientation_diff[j])
        # axs[1].plot(time[j], orientation_diff[j])
        # axs[1].axis(ymax=0.00007, ymin=-0.0001)
        plt.legend([i.split("_")[2] + "_" + i.split("_")[3] for i in files], loc='upper right')

    # plt.figure()
    # plt.plot(X, Y)

    # plt.figure()
    # plt.plot(Z)

    # plt.show()
    # return axs
    return max([abs(max_y_yaw - min_y_yaw) + 0.01, abs(max_y_pitch - min_y_pitch) + 0.01,
                abs(max_y_roll - min_y_roll) + 0.01]), min_y_yaw - 0.005, max_y_yaw + 0.005
    # return max_y, min_y


# uses walknet_stability_ files
def plot_stability_data_to_footfall_pattern(ax0, ax1, ax2, ax3, start_time, stop_time):
    stance_times = [[], [], [], [], [], []]
    last_state_swing = [True, True, True, True, True, True]
    first_line = True
    line_count = 0
    plot = True
    # plt.figure()
    # plt.xlim(-0.3, 0.3)
    # plt.ylim(-0.4, 0.4)
    r = re.compile(".*stability.*")
    filename = list(filter(r.match, sys.argv))[0]
    walk_start_time = None
    max_time = 0
    for line in open(str(filename), 'r'):
        if first_line:
            first_line = False
        else:
            line = line.rstrip("\n")
            try:
                values = [float(s) for s in line.split(";")]
                # print("time = " + str(values[0))
            except ValueError:
                tmp = line.split(";")
                tmp_time = tmp[0].split(".")
                tmp[0] = tmp_time[0] + '.' + tmp_time[2]
                values = [float(s) for s in tmp]
            if walk_start_time is None:
                walk_start_time = values[0]
            values[0] -= walk_start_time
            if stop_time != 0:
                if values[0] > stop_time:
                    break
                if values[0] < start_time:
                    continue

            column_idx = 1
            while column_idx < 19:
                if values[column_idx] != 0.0 and values[column_idx + 1] != 0.0:
                    if last_state_swing[column_idx // 3]:
                        last_state_swing[column_idx // 3] = False
                        stance_times[column_idx // 3].append([values[0]])
                        stance_times[column_idx // 3][len(stance_times[column_idx // 3]) - 1].append(values[0])
                    else:
                        if len(stance_times[column_idx // 3][len(stance_times[column_idx // 3]) - 1]) == 2:
                            stance_times[column_idx // 3][len(stance_times[column_idx // 3]) - 1][1] = values[0]
                        # elif len(stance_times[column_idx // 3][len(stance_times[column_idx // 3]) - 1]) < 2:
                        #     stance_times[column_idx // 3][len(stance_times[column_idx // 3]) - 1].append(values[0])
                        else:
                            print("something went wrong stance_times = " + str(stance_times))
                else:
                    last_state_swing[column_idx // 3] = True
                column_idx += 3
            max_time = values[0]
            line_count += 1

    if plot:
        leg_order = [5, 4, 3, 0, 1, 2]
        # marked_step = [3, 3, 2, 1, 1, 1]
        marked_step = [2, 1, 1, 0, 0, 0]
        show_steps = True
        leg_color = ['r', 'g', 'b', 'c', 'm', 'y']
        for leg in stance_times:
            for step in leg:
                if show_steps:
                    # if stance_times.index(leg) == 3 and leg.index(step) == 1:
                    if leg.index(step) == marked_step[stance_times.index(leg)]:
                        ax0.axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                        ax1.axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                        ax2.axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                        ax3.axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                    if leg.index(step) == (marked_step[stance_times.index(leg)] + 1):
                        ax0.axvline(x=step[0], color=leg_color[stance_times.index(leg)])
                        ax1.axvline(x=step[0], color=leg_color[stance_times.index(leg)])
                        ax2.axvline(x=step[0], color=leg_color[stance_times.index(leg)])
                        ax3.axvline(x=step[0], color=leg_color[stance_times.index(leg)])
                # print("leg index = {} ['lf', 'lm', 'lr', 'rr', 'rm', 'rf']".format(stance_times.index(leg)))
                # print("step = {}, stance_times.index(leg) = {}, leg_order = {}".format(step, stance_times.index(leg), leg_order))
                # ax3.plot([step[0], step[1]],
                #         [leg_order[stance_times.index(leg)], leg_order[stance_times.index(leg)]],
                #         linestyle='-', linewidth=12.0, color='black', marker='', solid_capstyle="butt")
        marked_step = [3, 2, 2, 4, 4, 4]
        for leg in stance_times:
            for step in leg:
                if show_steps:
                    # if stance_times.index(leg) == 3 and leg.index(step) == 1:
                    if leg.index(step) == marked_step[stance_times.index(leg)]:
                        ax0.axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                        ax1.axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                        ax2.axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                        ax3.axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                    if leg.index(step) == (marked_step[stance_times.index(leg)] + 1):
                        ax0.axvline(x=step[0], color=leg_color[stance_times.index(leg)])
                        ax1.axvline(x=step[0], color=leg_color[stance_times.index(leg)])
                        ax2.axvline(x=step[0], color=leg_color[stance_times.index(leg)])
                        ax3.axvline(x=step[0], color=leg_color[stance_times.index(leg)])
        for leg in stance_times:
            for i in range(0, len(leg) - 1):
                plt.plot([leg[i][1], leg[i + 1][0]],
                        [leg_order[stance_times.index(leg)], leg_order[stance_times.index(leg)]],
                        linestyle='-', linewidth=20.0, color='black', marker='', solid_capstyle="butt")

        # plt.xlim(-0.3, 0.3)
        plt.ylim(-0.5, 5.5)
        # plt.yticks([0, 1, 2, 3, 4, 5], ['lf', 'lm', 'lr', 'rr', 'rm', 'rf'])
        # plt.yticks([0, 1, 2, 3, 4, 5],
        #         ['right rear', 'right middle', 'right front', 'left rear', 'left middle', 'left front'])
        plt.yticks([0, 1, 2, 3, 4, 5], ['RR', 'RM', 'RF', 'LR', 'LM', 'LF'])

        return max_time


if __name__ == '__main__':
    if len(sys.argv) == 3:
        # start_duration = 115
        # start_duration = 70
        start_duration = 45  # 0 # 30  # 15  # 40
        # stop_duration = 60
        # stop_duration = 100
        # stop_duration = 145
        stop_duration = 75  # 120 # 60  # 45  # 70 # 60
        # stop_duration = 45  # 0.05s 0.5dir
        # roll = False
        # pitch = True
        # yaw = False

        # f, (a0, a1, a2) = plt.subplots(3, 1, gridspec_kw={'height_ratios': [1, 1, 3]})
        fig, (ax0, ax1, ax2, ax3) = plt.subplots(4, figsize=(16, 20), sharex=True,
                gridspec_kw={'height_ratios': [1, 1, 2, 1.5]})
        plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=4)
        # fig.tight_layout(rect=[0, 0.03, 1, 0.95])
        # fig.tight_layout()

        # if roll and not pitch and not yaw:
        #     fig.suptitle('roll')
        # elif pitch and not roll and not yaw:
        #     fig.suptitle('pitch')
        # elif yaw and not pitch and not roll:
        #     fig.suptitle('yaw')
        # else:
        #     print("invalid configuration")
        #     sys.exit()

        # plt.setp(axs, xticks=range(0, stop_duration, 2))

        max_range, min_y_yaw, max_y_yaw = plot_orientation_data(ax0, ax1, ax2, start_duration, stop_duration)
        max_time = plot_stability_data_to_footfall_pattern(ax0, ax1, ax2, ax3, start_duration, stop_duration)
        print("max_time = " + str(max_time))
        print("min y yaw = {}, max y yaw = {}".format(min_y_yaw, max_y_yaw))

        zero_diff = max_range / 2
        ax0.set_ylim(-zero_diff / 2, zero_diff / 2)
        # ax0.set_ylim(-(zero_diff *2) / 3, zero_diff / 3)
        ax1.set_ylim(-zero_diff / 2, zero_diff / 2)
        # ax1.set_ylim(-zero_diff / 5, (zero_diff *4) / 5)
        ax2.set_ylim(min_y_yaw, max_y_yaw)
        #ax3.set_xlim(start_duration, stop_duration)

        axs = [ax0, ax1, ax2, ax3]
        for ax in axs:
            ax.grid()
            # ax.tick_params(axis='x', labelsize=18)
            ax.tick_params(axis='both', labelsize=18, labelbottom=True)
            if stop_duration != 0:
                plt.xticks(numpy.arange(start_duration, stop_duration + 1, 2))
            else:
                plt.xticks(numpy.arange(0, round(max_time) + 1, 2))
            # plt.yticks(numpy.arange(0, max(y), 2))

            # plt.tick_params(labelsize=18)
            # plt.grid(which='both')
            # plt.setp(ax.get_xticklabels(), visible=True)
            # plt.setp(ax.get_xticklabels(), fontsize=18)
            # ax.xaxis.set_tick_params(which='both', labelbottom=True)
            # ax.minorticks_on()

        # plt.show()
        safe_plot = True
        if safe_plot:
            r = re.compile(".*stability.*")
            filename = list(filter(r.match, sys.argv))[0]
            # split = [i.split("_") for i in files]
            split = re.findall(r"[^/_,]+", filename, re.ASCII)
            # print("split = " + str(split))
            name = "_".join(split[(split.index("stability") + 4):])
            # print("name = " + name)
            print("single file: " + "/home/jsimmering/plots_masterthesis/orientation/orientation_" + name + ".svg")
            plt.savefig("/home/jsimmering/plots_masterthesis/orientation/orientation_" + name + ".svg",
                    bbox_inches='tight', pad_inches=0, format='svg', transparent=True)
        else:
            plt.show()
