#!/usr/bin/env python3
import datetime
import os
import sys
import operator
import re

import numpy

if len(sys.argv) >= 2:

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

    durations = []
    unstable_percent = []
    stability_enforcement = []
    delays_percent = []
    valueError_percent = []
    average_swing_duration = []
    average_stance_duration = []
    average_stance_steps = []

    for j in range(0, len(files)):
        delays_dict = {}
        valueError_dict = {}
        swing_duration_dict = {}
        stance_duration_dict = {}
        stance_step_dict = {}

        line_number = 0
        file_name = None
        if len(sys.argv) > 2:
            file_name = sys.argv[2] + "/" + str(files[j])
        else:
            file_name = str(files[j])

        is_delay_line = False
        is_error_line = False
        is_swing_duration_line = False
        is_stance_duration_line = False
        is_stance_step_line = False
        try:
            for line in open(file_name, 'r'):
                line = line.rstrip("\n")
                split = line.split(" ")
                if split[0] == "duration":  # line_number == 16:
                    durations.append(float(split[-1]))
                if split[0] == "unstable_percent":  # line_number == 16:
                    unstable_percent.append(round(float(split[split.index("=") + 1]), 2))
                if split[0] == "cs" and split[1] == "with" and split[2] == "delay" and split[3] == "and" and split[6] == "[%]":  # line_number == 16:
                    stability_enforcement.append(round(float(split[split.index("=") + 1]), 2))
                elif split[0] == "value_error_count:":
                    is_error_line = True
                elif split[0] == "swing_delay_count:":
                    is_delay_line = True
                    is_error_line = False
                elif split[0] == "swing_count:":
                    is_delay_line = False
                elif split[0] == "average_swing_duration:":
                    is_swing_duration_line = True
                elif split[0] == "stance_count:":
                    is_swing_duration_line = False
                    is_stance_step_line = True
                elif split[0] == "average_stance_duration:":
                    is_stance_duration_line = True
                    is_stance_step_line = False
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
                elif is_swing_duration_line:
                    split[len(split) - 1] = split[len(split) - 1][0:-1]
                    try:
                        swing_duration_dict[split[0]] = float(split[len(split) - 1])
                    except ValueError:
                        swing_duration_dict[split[0]] = float('nan')
                elif is_stance_duration_line:
                    split[len(split) - 1] = split[len(split) - 1][0:-1]
                    try:
                        stance_duration_dict[split[0]] = float(split[len(split) - 1])
                    except ValueError:
                        stance_duration_dict[split[0]] = float('nan')
                elif is_stance_step_line:
                    split[len(split) - 1] = split[len(split) - 1][0:-1]
                    try:
                        stance_step_dict[split[0]] = float(split[len(split) - 1])
                    except ValueError:
                        stance_step_dict[split[0]] = float('nan')
                line_number += 1

            # if line_number == 23:
            valueError_percent.append(valueError_dict)
            # valueError_dict = {}
            # if line_number == 30:
            delays_percent.append(delays_dict)
            # delays_dict = {}
            average_swing_duration.append(swing_duration_dict)
            average_stance_duration.append(stance_duration_dict)
            average_stance_steps.append(stance_step_dict)
        except IsADirectoryError:
            pass

    keys = ['lf', 'rf', 'lm', 'rm', 'lr', 'rr']

    # printing
    print("**duration:**")
    durations_str = "* "
    for dur in durations:
        durations_str += str(dur) + " seconds = " + str(datetime.timedelta(seconds=round(dur)))
        if not durations.index(dur) == len(durations) - 1:
            durations_str += ", "
    print(durations_str)
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

    max_enforcement = max(stability_enforcement)
    enforcement_str = "* "
    for value in stability_enforcement:
        if value >= 3.0:
            if value == max_enforcement:
                enforcement_str += "**_" + str(value) + "_**"
            else:
                enforcement_str += "**" + str(value) + "**"
        else:
            if value == max_enforcement:
                enforcement_str += "_" + str(value) + "_"
            else:
                enforcement_str += str(value)
        if not stability_enforcement.index(value) == len(stability_enforcement) - 1:
            enforcement_str += ", "
    print("**stability enforcement:**")
    print(enforcement_str)
    print("")

    # try:
    #     max_delays = [max(dict.items(), key=operator.itemgetter(1)) for dict in delays_percent]
    #
    #     max_delays_value = 0
    #     for key, value in max_delays:
    #         if value > max_delays_value:
    #             max_delays_value = value
    #
    #     print("**max delay:**")
    #     delays_str = "* "
    #     for key, value in max_delays:
    #         # print("value = " + str(value))
    #         if value >= 1.0:
    #             if value == max_delays_value:
    #                 delays_str += "**_(" + key + ", " + str(value) + ")_**"
    #             else:
    #                 delays_str += "**(" + key + ", " + str(value) + ")**"
    #         else:
    #             if value == max_delays_value:
    #                 delays_str += "_(" + key + ", " + str(value) + ")_"
    #             else:
    #                 delays_str += "(" + key + ", " + str(value) + ")"
    #         if not max_delays[len(max_delays) - 1] == (key, value):
    #             delays_str += ", "
    #     print(delays_str)
    #     # print("**max delay:**\n" + str(max_delays))
    #     print("")
    # except ValueError:
    #     # print("**delays:**")
    #     # delays_str = "* "
    #     # for delay in delays_percent:
    #     #     delays_str += "["
    #     #     for key, value in delay:
    #     #         # print("value = " + str(type(value)))
    #     #         if value >= 1.0:
    #     #             delays_str += "**(" + key + ", " + str(value) + ")**"
    #     #         else:
    #     #             delays_str += "(" + key + ", " + str(value) + ")"
    #     #     delays_str += "]"
    #     #     if not unstable_percent.index(value) == len(unstable_percent) - 1:
    #     #         delays_str += ", "
    #     #
    #     # print(delays_str)
    #     print("**delays:**\n" + str(delays_percent))
    #     # print("")

    error_str = "* "
    # print("delays_percent = " + str(delays_percent))
    v = {k: [dic[k] if k in dic.keys() else 0 for dic in delays_percent] for k in keys}
    # print(v)
    # print([str(key) + ": " + str(max(val)) for key, val in v])
    print("**delays:**")
    print([str(val) + ": " + str(max(v[val])) for val in v])

    # for i in range(0, len(delays_percent)):
    #     # for error in delays_percent:
    #     # print("error = " + str(error))
    #     error_str += "["
    #     if delays_percent[i]:
    #         for key, value in delays_percent[i].items():
    #             if value >= 0.0:
    #                 error_str += "**(" + key + ", " + str(value) + ")**, "
    #             else:
    #                 error_str += "(" + key + ", " + str(value) + "), "
    #         # else:
    #         #     error_str += "{}"
    #     error_str += "]"
    #     if not i == len(delays_percent) - 1:
    #         error_str += ", "

    #print("**delays:**")
    #print(error_str)
    print("")
    # print("**ValueErrors:**\n" + str(valueError_percent))

    # try:
    #     max_error = [max(dict.items(), key=operator.itemgetter(1)) for dict in valueError_percent]
    #
    #     max_error_value = 0
    #     for key, value in max_error:
    #         if value > max_error_value:
    #             max_error_value = value
    #
    #     print("**max ValueErrors:**")
    #     error_str = "* "
    #     for key, value in max_error:
    #         # print("value = " + str(type(value)))
    #         if value >= 0.0:
    #             if value == max_error_value:
    #                 error_str += "**_(" + key + ", " + str(value) + ")_**"
    #             else:
    #                 error_str += "**(" + key + ", " + str(value) + ")**"
    #         else:
    #             if value == max_error_value:
    #                 error_str += "_(" + key + ", " + str(value) + ")_"
    #             else:
    #                 error_str += "(" + key + ", " + str(value) + ")"
    #         if not max_error[len(max_error) - 1] == (key, value):
    #             error_str += ", "
    #
    #     print(error_str)
    #     print("")
    # except ValueError:

    error_str = "* "
    # print("delays_percent = " + str(delays_percent))
    v = {k: [dic[k] if k in dic.keys() else 0 for dic in valueError_percent] for k in keys}
    # print(v)
    # print([str(key) + ": " + str(max(val)) for key, val in v])
    print("**ValueErrors:**")
    print([str(val) + ": " + str(max(v[val])) for val in v])

    # print("valueError_percent = " + str(valueError_percent))
    # for i in range(0, len(valueError_percent)):
    #     # for error in valueError_percent:
    #     # print("error = " + str(error))
    #     error_str += "["
    #     if valueError_percent[i]:
    #         for key, value in valueError_percent[i].items():
    #             if value >= 0.0:
    #                 error_str += "**(" + key + ", " + str(value) + ")**, "
    #             else:
    #                 error_str += "(" + key + ", " + str(value) + "), "
    #         # else:
    #         #     error_str += "{}"
    #     error_str += "]"
    #     if not i == len(valueError_percent) - 1:
    #         error_str += ", "

    # print("**ValueErrors:**")
    # print(error_str)
    print("")
    # print("**ValueErrors:**\n" + str(valueError_percent))

    print("**average swing durations (all runs):**")
    swing_str = "* "
    swing_dict = {}
    for k in keys:
        #swing_dict[k] = round(sum(swings[k] for swings in average_swing_duration) / len(average_swing_duration), 2)
        print(str(round(sum(swings[k] for swings in average_swing_duration) / len(average_swing_duration), 2)) + " & ", end="")
    # swing_str += str(swing_dict)

    # for dur in average_swing_duration:
    #     swing_str += str(dur)
    #     if not average_swing_duration.index(dur) == len(average_swing_duration) - 1:
    #         swing_str += "\n* "
    #print(swing_str)
    print("")
    print("")

    print("**average stance durations (all runs):**")
    #stance_str = "* "
    v = {k: [dic[k] for dic in average_stance_duration] for k in keys}
    average = []
    for k in keys:
        v[k] = [x for x in v[k] if numpy.isnan(x) == False]
        if len(v[k]) > 0:
            average.append(numpy.average(numpy.array(v[k])))
        else:
            average.append(float('nan'))
        # print(str(round(sum(stances[k] for stances in average_stance_duration) / len(average_stance_duration), 2)) + " & ", end="")
    #print("v = " + str(v))
    print("average = " + str(average))
    print("")
    for i in range(0, len(average)):
        print(" " + str(average[i]), end="")
    print(" ")
    print("")
    for i in range(0, len(average)):
        print(" & " + str(round(average[i], 2)), end="")
    print(" ")
    #stance_str += str(stance_dict)
    # for dur in average_stance_duration:
    #     stance_str += str(dur)
    #     if not average_stance_duration.index(dur) == len(average_stance_duration) - 1:
    #         stance_str += "\n* "
    #print(stance_str)
    print("")

    print("**average swing stance ratio (all runs):**")
    for k in keys:
        average_swing = sum(swings[k] for swings in average_swing_duration) / len(average_swing_duration)
        average_stance = sum(stances[k] for stances in average_stance_duration) / len(average_stance_duration)
        average_cycle = average_stance + average_swing
        stance_ratio = (average_stance * 100)/average_cycle
        print(" & " + str(round(stance_ratio)) + "%", end="")
    print("")

    print("**average stance steps (all runs):**")
    # stance_str = "* "
    #average = [round(sum(stances[k] for stances in average_stance_steps) / len(average_stance_steps), 2) for k in keys]

    v = {k: [dic[k] for dic in average_stance_steps] for k in keys}
    average = []
    for k in keys:
        v[k] = [x for x in v[k] if numpy.isnan(x) == False]
        # print(type(v[k]))
        if len(v[k]) > 0:
            average.append(numpy.average(numpy.array(v[k])))
        else:
            average.append(float('nan'))
    print("v = " + str(v))
    print("average = " + str(average))



    # stance_str += str(stance_dict)
    # for dur in average_stance_duration:
    #     stance_str += str(dur)
    #     if not average_stance_duration.index(dur) == len(average_stance_duration) - 1:
    #         stance_str += "\n* "
    # print(stance_str)
    print("")
    print("")

    print("")
    print("#############################################################################")
    print("")
