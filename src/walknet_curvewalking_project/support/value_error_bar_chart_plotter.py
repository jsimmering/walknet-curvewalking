#!/usr/bin/env python3
import datetime
import os
import sys
import operator
import re

import numpy
import matplotlib.pyplot as plt


def plot_bar_chart(title, max_y, data):
    X = numpy.arange(7)
    fig, axs = plt.subplots()
    axs.set_title(title, fontsize=18)
    axs.grid()
    axs.tick_params(labelsize=16)
    axs.set_ylim(0, max_y)
    axs.set_ylabel('unreachable positions [%]', fontsize=18)
    plt.xticks([0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5],
            ['0.0rad', '0.2rad', '0.4rad', '0.6rad', '0.8rad', '1.0rad', '1.2rad'])
    axs.bar(X + 0.00, data[0], color='b', width=0.15)
    axs.bar(X + 0.15, data[1], color='g', width=0.15)
    axs.bar(X + 0.3, data[2], color='r', width=0.15)
    axs.bar(X + 0.45, data[3], color='c', width=0.15)
    axs.bar(X + 0.6, data[4], color='y', width=0.15)
    axs.bar(X + 0.75, data[5], color='m', width=0.15)
    axs.legend(labels=['0.01m/s', '0.02m/s', '0.03m/s', '0.04m/s', '0.05m/s', '0.06m/s'], loc='upper left', fontsize=18)

    fig.autofmt_xdate()

    if save_plot:
        split = re.findall(r"[^/_,]+", sys.argv[2], re.ASCII)
        print(split)
        name = ""
        name += "_".join(split[split.index("walknet"):])
        plot_path = "/home/jsimmering/plots_masterthesis/valueErrorHis/" + title + "_" + name + ".svg"
        print("file path: " + plot_path)
        fig.savefig(plot_path, bbox_inches='tight', pad_inches=0, format='svg')


if __name__ == '__main__':
    if len(sys.argv) >= 2:

        files = None
        if sys.argv[1] == "-dir":
            files = os.listdir(sys.argv[2])
            files.sort(key=lambda x: os.path.getmtime(sys.argv[2] + "/" + x), reverse=False)
            print(files)
        elif sys.argv[1] == "-c":
            dirs = os.listdir(sys.argv[2] + "/")
            dirs = sorted(dirs)
            # if '0.0dir' in dirs:
            #     dirs.remove('0.0dir')
            # dirs.sort(key=lambda x: os.path.getmtime(sys.argv[2] + "/" + x), reverse=False)
            print(dirs)
            speeds = [os.listdir(sys.argv[2] + "/" + s) for s in dirs]
            [s.sort(key=lambda x: x.split()[0]) for s in speeds]
            print(speeds)
            files = []
            for d in dirs:
                files.append([])
                for s in speeds[0]:
                    files[dirs.index(d)].append([sys.argv[2] + "/" + d + "/" + s + "/" + x for x in
                                                 os.listdir(sys.argv[2] + "/" + d + "/" + s)])
            print(files)
            print("")
        elif len(sys.argv) == 2:
            files = [sys.argv[1]]
        else:
            print("wrong parameters provide data-filename or -dir directory with files")
            exit()

        max_unstable_percent = []
        max_delays_percent = []
        max_valueError_percent = []
        average_swing_counts = []
        average_stance_counts = []

        for speed in range(0, len(files)):
            print("speed = " + str(speed))

            max_unstable_percent.append([])
            max_delays_percent.append([])
            max_valueError_percent.append([])
            average_swing_counts.append([])
            average_stance_counts.append([])

            for direction in range(0, len(files[speed])):
                print("direction = " + str(direction))

                unstable_percent = []
                delays_percent = []
                valueError_percent = []
                swing_count = []
                stance_count = []

                for run_number in range(0, len(files[speed][direction])):
                    print("file name = " + str(files[speed][direction]))
                    delays_dict = {}
                    valueError_dict = {}
                    swing_count_dict = {}
                    stance_count_dict = {}

                    line_number = 0

                    is_delay_line = False
                    is_error_line = False
                    is_swing_count_line = False
                    is_stance_count_line = False
                    try:
                        for line in open(files[speed][direction][run_number], 'r'):
                            line = line.rstrip("\n")
                            split = line.split(" ")
                            print("split = " + str(split))
                            if split[0] == "unstable_percent":  # line_number == 16:
                                unstable_percent.append(round(float(split[split.index("=") + 1]), 2))
                            elif split[0] == "value_error_count:":
                                is_error_line = True
                            elif split[0] == "swing_delay_count:":
                                is_delay_line = True
                                is_error_line = False
                            elif split[0] == "swing_count:":
                                is_delay_line = False
                                is_swing_count_line = True
                                print("swing count line = " + str(is_swing_count_line))
                            elif split[0] == 'average_swing_duration:':
                                is_swing_count_line = False
                                print("swing count line = " + str(is_swing_count_line))
                            elif split[0] == "stance_count":
                                is_stance_count_line = True
                            elif split[0] == "average_stance_duration":
                                is_stance_count_line = False
                            elif is_error_line:
                                split[len(split) - 1] = split[len(split) - 1][0:-1]
                                # print(split)
                                v_error = round(float(split[split.index("=") + 1]), 2)
                                if v_error != 0.0:
                                    valueError_dict[split[0]] = v_error
                            elif is_delay_line:
                                split[len(split) - 1] = split[len(split) - 1][0:-1]
                                # print(split)
                                delay = round(float(split[split.index("=") + 1]), 2)
                                if delay != 0.0:
                                    delays_dict[split[0]] = delay
                            elif is_swing_count_line:
                                print(split)
                                count = float(split[-1])
                                if count != 0.0:
                                    swing_count_dict[split[0]] = count
                            elif is_stance_count_line:
                                print(split)
                                count = float(split[-1])
                                if count != 0.0:
                                    stance_count_dict[split[0]] = count
                            line_number += 1
                        valueError_percent.append(valueError_dict)
                        delays_percent.append(delays_dict)
                        swing_count.append(swing_count_dict)
                        stance_count.append(stance_count_dict)
                    except IsADirectoryError:
                        pass
                    print("")

                max_unstable = max(unstable_percent)
                unstable_str = "* "
                for value in unstable_percent:
                    if value >= 3.0:
                        if value == max_unstable:
                            unstable_str += "**_" + str(value) + "_**"
                        else:
                            unstable_str += "**" + str(value) + "**"
                    else:
                        if value == max_unstable:
                            unstable_str += "_" + str(value) + "_"
                        else:
                            unstable_str += str(value)
                    if not unstable_percent.index(value) == len(unstable_percent) - 1:
                        unstable_str += ", "
                print("**unstable:**")
                print(unstable_str)
                print("")
                max_unstable_percent[speed].append(100 - max_unstable)

                v = {k: max([dic[k] if k in dic.keys() else 0 for dic in delays_percent]) for k in
                     ['lf', 'rf', 'lm', 'rm', 'lr', 'rr']}
                print("**delays:**")
                # print([str(val) + ": " + str(max(v[val])) for val in v])
                max_delays_percent[speed].append(v)

                print(valueError_percent)
                v = {k: max([dic[k] if k in dic.keys() else 0 for dic in valueError_percent]) for k in
                     ['lf', 'rf', 'lm', 'rm', 'lr', 'rr']}
                print("**ValueErrors:**")
                # print([str(val) + ": " + str(max(v[val])) for val in v])
                max_valueError_percent[speed].append(v)

                print(swing_count)
                v = {k: numpy.average(numpy.array([dic[k] if k in dic.keys() else 0 for dic in swing_count])) for k in
                     ['lf', 'rf', 'lm', 'rm', 'lr', 'rr']}
                print("**Average Swing Counts:**")
                # print([str(val) + ": " + str(max(v[val])) for val in v])
                average_swing_counts[speed].append(v)

                print(stance_count)
                v = {k: numpy.average(numpy.array([dic[k] if k in dic.keys() else 0 for dic in stance_count])) for k in
                     ['lf', 'rf', 'lm', 'rm', 'lr', 'rr']}
                print("**Average Swing Counts:**")
                # print([str(val) + ": " + str(max(v[val])) for val in v])
                average_stance_counts[speed].append(v)

        print("")
        print("unstable = ")  # + str(max_unstable_percent))
        for speed in max_unstable_percent:
            print(speed)
        print("")
        print("delays = ")  # + str(max_delays_percent))
        for speed in max_delays_percent:
            print(speed)
        print("")
        print("valueError = ")  # + str(max_valueError_percent))
        for speed in max_valueError_percent:
            print(speed)
        print("")
        print("swingCount = ")  # + str(max_valueError_percent))
        for speed in average_swing_counts:
            print(speed)
        print("")
        print("stanceCount = ")  # + str(max_valueError_percent))
        for speed in average_stance_counts:
            print(speed)
        print("")

        lf_data = []
        lm_data = []
        lr_data = []
        rf_data = []
        rm_data = []
        rr_data = []
        for speed in max_valueError_percent:
            lf_data.append([])
            lm_data.append([])
            lr_data.append([])
            rf_data.append([])
            rm_data.append([])
            rr_data.append([])
            for d in speed:
                lf_data[max_valueError_percent.index(speed)].append(d['lf'])
                lm_data[max_valueError_percent.index(speed)].append(d['lm'])
                lr_data[max_valueError_percent.index(speed)].append(d['lr'])
                rf_data[max_valueError_percent.index(speed)].append(d['rf'])
                rm_data[max_valueError_percent.index(speed)].append(d['rm'])
                rr_data[max_valueError_percent.index(speed)].append(d['rr'])

        data = lf_data + lm_data + lr_data + rf_data + rm_data + rr_data
        max_value = max(max(data, key=max)) + 2

        save_plot = True
        plot_bar_chart('left front', max_value, lf_data)
        plot_bar_chart('left middle', max_value, lm_data)
        plot_bar_chart('left rear', max_value, lr_data)
        plot_bar_chart('right front', max_value, rf_data)
        plot_bar_chart('right middle', max_value, rm_data)
        plot_bar_chart('right rear', max_value, rr_data)

        if not save_plot:
            plt.show()
