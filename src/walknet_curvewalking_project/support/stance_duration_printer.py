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
        # print(files)
    elif len(sys.argv) == 2:
        files = [sys.argv[1]]
    else:
        print("wrong parameters provide data-filename or -dir directory with files")
        exit()

    durations = []
    unstable_percent = []
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
    #print("**average stance durations (all runs):**")
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
    #print("average = " + str(average))
    #print("")
    for i in range(0, len(average)):
        print(" " + str(average[i]), end="")
    #print(" ")
    #print("")
    # for i in range(0, len(average)):
    #     print(" & " + str(round(average[i], 2)), end="")
    # print(" ")
    #stance_str += str(stance_dict)
    # for dur in average_stance_duration:
    #     stance_str += str(dur)
    #     if not average_stance_duration.index(dur) == len(average_stance_duration) - 1:
    #         stance_str += "\n* "
    #print(stance_str)
