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
        trial = sys.argv[2].split("/")
        if trial[len(trial) - 1] == "0.0dir":
            print("\hline")
        print(trial[len(trial) - 2].replace('ms', ' & ') + trial[len(trial) - 1].replace('rad', ''), end='')
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
        duration = None
        try:
            for line in open(file_name, 'r'):
                line = line.rstrip("\n")
                split = line.split(" ")
                if split[0] == "duration":  # line_number == 16:
                    durations.append(float(split[-1]))
                    duration = float(split[-1])
                if split[0] == "stance_count:":
                    is_stance_step_line = True
                elif split[0] == "average_stance_duration:":
                    is_stance_step_line = False
                elif is_stance_step_line:
                    # split[len(split) - 1] = split[len(split) - 1][0:-1]
                    # print(split)
                    try:
                        stance_step_dict[split[0]] = float(split[len(split) - 1])/duration
                    except ValueError:
                        stance_step_dict[split[0]] = float('nan')
                line_number += 1

            average_stance_steps.append(stance_step_dict)
        except IsADirectoryError:
            pass

    keys = ['lf', 'rf', 'lm', 'rm', 'lr', 'rr']

    # printing
    # print("**duration:**")
    # durations_str = "* "
    # for dur in durations:
    #     durations_str += str(dur) + " seconds = " + str(datetime.timedelta(seconds=round(dur)))
    #     if not durations.index(dur) == len(durations) - 1:
    #         durations_str += ", "
    # print(durations_str)
    # print("")

    # print("**average stance steps (all runs):** " + str(average_stance_steps))
    # stance_str = "* "
    # average = [round(sum(stances[k] for stances in average_stance_steps) / len(average_stance_steps), 2) for k in keys]

    v = {k: [dic[k] for dic in average_stance_steps] for k in keys}
    average = []
    for k in keys:
        v[k] = [x for x in v[k] if numpy.isnan(x) == False]
        # print(type(v[k]))
        if len(v[k]) > 0:
            average.append(numpy.average(numpy.array(v[k])))
        else:
            average.append(float('nan'))
    #print("v = " + str(v))
    for a in average:
        print(" & " + str(round(a, 2)), end='')
    print(" \\\ ")
    #print(average)
