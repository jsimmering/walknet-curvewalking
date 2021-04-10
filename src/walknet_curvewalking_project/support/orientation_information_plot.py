import sys
import re

import matplotlib.pyplot as plt
import tf.transformations as tf_trans


def plot_orientation_data(axs, start_time, stop_time):
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

    orientation_z, orientation_diff, time = [], [], []
    for i in range(0, len(files)):
        orientation_z.append([])
        orientation_diff.append([])
        time.append([])

    # plt.figure()
    # fig, axs = plt.subplots(2)

    for j in range(0, len(files)):
        first_line = True
        last_time = None
        last_orientation = None
        initial_orientation = None

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
                if stop_time != 0:
                    if values[0] > stop_time:
                        break
                    if values[0] < start_time:
                        continue

                used_lines += 1

                angles = tf_trans.euler_from_quaternion([values[4], values[5], values[6], values[7]])
                # print("angles = " + str(angles))
                if last_orientation is not None:
                    if initial_orientation is None:
                        initial_orientation = angles[2]
                    # orientation_z[j].append(angles[2] - initial_orientation)
                    orientation_z[j].append(angles[2])
                    if abs(angles[2] - last_orientation) <= 0.00002:
                        orientation_diff[j].append(0)
                    else:
                        orientation_diff[j].append(angles[2] - last_orientation)
                    time[j].append(values[0])
                last_orientation = angles[2]

                line_number += 1

        print("used lines = " + str(used_lines) + " of total lines = " + str(line_number))
        # print("orientation_z = " + str(orientation_z))
        # print("time = " + str(time))
        # plt.plot(time[j], orientation_z[j])
        axs[0].plot(time[j], orientation_z[j])
        axs[0].axis(ymax=initial_orientation + 0.05, ymin=initial_orientation - 0.15)
        # plt.plot(time[j], orientation_diff[j])
        axs[1].plot(time[j], orientation_diff[j])
        axs[1].axis(ymax=0.00007, ymin=-0.0001)
        plt.legend([i.split("_")[2] + "_" + i.split("_")[3] for i in files], loc='upper right')

    # plt.figure()
    # plt.plot(X, Y)

    # plt.figure()
    # plt.plot(Z)

    # plt.show()
    return axs


# uses walknet_stability_ files
def plot_stability_data_to_footfall_pattern(axs, start_time, stop_time):
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
    for line in open(str(filename), 'r'):
        if first_line:
            first_line = False
            pass
        else:
            line = line.rstrip("\n")
            try:
                values = [float(s) for s in line.split(";")]
            except ValueError:
                tmp = line.split(";")
                tmp_time = tmp[0].split(".")
                tmp[0] = tmp_time[0] + '.' + tmp_time[2]
                values = [float(s) for s in tmp]
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
            line_count += 1

    if plot:
        leg_order = [5, 4, 3, 0, 1, 2]
        # marked_step = [6, 5, 4, 1, 2, 3]
        marked_step = [2, 1, 1, 0, 0, 1]
        # marked_step = [1, 1, 1, 0, 1, 1]  # 0.01s 0.0 dir
        # marked_step = [2, 2, 2, 0, 1, 1]  # 0.01s 0.5 dir
        # marked_step = [2, 2, 2, 1, 1, 1] # 0.02s 0.5 dir
        # marked_step = [3, 2, 2, 0, 1, 1] # 0.05s 0.5dir
        leg_color = ['r', 'g', 'b', 'c', 'm', 'y']
        for leg in stance_times:
            for step in leg:
                # if stance_times.index(leg) == 3 and leg.index(step) == 1:
                if leg.index(step) == marked_step[stance_times.index(leg)]:
                    axs[0].axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                    axs[1].axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                    axs[2].axvline(x=step[1], color=leg_color[stance_times.index(leg)])
                if leg.index(step) == (marked_step[stance_times.index(leg)] + 1):
                    axs[0].axvline(x=step[0], color=leg_color[stance_times.index(leg)])
                    axs[1].axvline(x=step[0], color=leg_color[stance_times.index(leg)])
                    axs[2].axvline(x=step[0], color=leg_color[stance_times.index(leg)])
                # print("leg index = {} ['lf', 'lm', 'lr', 'rr', 'rm', 'rf']".format(stance_times.index(leg)))
                # print("step = {}, stance_times.index(leg) = {}, leg_order = {}".format(step, stance_times.index(leg), leg_order))
                axs[2].plot([step[0], step[1]],
                        [leg_order[stance_times.index(leg)], leg_order[stance_times.index(leg)]],
                        linestyle='-', linewidth=15.0, color='black', marker='', solid_capstyle="butt")

        # plt.xlim(-0.3, 0.3)
        plt.ylim(-0.5, 5.5)
        # plt.yticks([0, 1, 2, 3, 4, 5], ['lf', 'lm', 'lr', 'rr', 'rm', 'rf'])
        plt.yticks([0, 1, 2, 3, 4, 5],
                ['right rear', 'right middle', 'right front', 'left rear', 'left middle', 'left front'])

        return axs


if __name__ == '__main__':
    if len(sys.argv) == 3:
        # start_duration = 45
        start_duration = 30
        # stop_duration = 60
        stop_duration = 0
        # stop_duration = 45  # 0.05s 0.5dir
        fig, axs = plt.subplots(3)
        plt.setp(axs, xticks=range(0, stop_duration, 2))

        axs = plot_orientation_data(axs, start_duration, stop_duration)
        axs = plot_stability_data_to_footfall_pattern(axs, start_duration, stop_duration)

        for ax in axs:
            ax.grid()
        plt.show()
