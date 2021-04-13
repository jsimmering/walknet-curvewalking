#!/usr/bin/env python3
import sys

import matplotlib.pyplot as plt


# uses walknet_stability_ files
def plot_stability_data_to_footfall_pattern(duration):
    stance_times = [[], [], [], [], [], []]
    last_state_swing = [True, True, True, True, True, True]
    first_line = True
    line_count = 0
    plot = True
    plt.figure()
    for line in open(str(sys.argv[1]), 'r'):
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
            if duration != 0:
                if values[0] > duration:
                    break

            column_idx = 1
            while column_idx < 19:
                if values[column_idx] != 0.0 and values[column_idx + 1] != 0.0:
                    if last_state_swing[column_idx // 3]:
                        last_state_swing[column_idx // 3] = False
                        stance_times[column_idx // 3].append([values[0]])
                    else:
                        if len(stance_times[column_idx // 3][len(stance_times[column_idx // 3]) - 1]) == 2:
                            stance_times[column_idx // 3][len(stance_times[column_idx // 3]) - 1][1] = values[0]
                        elif len(stance_times[column_idx // 3][len(stance_times[column_idx // 3]) - 1]) < 2:
                            stance_times[column_idx // 3][len(stance_times[column_idx // 3]) - 1].append(values[0])
                        else:
                            print("something went wrong stance_times = " + str(stance_times))
                else:
                    last_state_swing[column_idx // 3] = True
                column_idx += 3
            line_count += 1

    if plot:
        leg_order = [5, 4, 3, 0, 1, 2]
        for leg in stance_times:
            for step in leg:
                print("leg index = {} ['lf', 'lm', 'lr', 'rr', 'rm', 'rf']".format(stance_times.index(leg)))
                plt.plot([step[0], step[1]], [leg_order[stance_times.index(leg)], leg_order[stance_times.index(leg)]],
                        linestyle='-', linewidth=20.0, color='black', marker='', solid_capstyle="butt")

        # plt.xlim(-0.3, 0.3)
        plt.ylim(-0.5, 5.5)
        # plt.yticks([0, 1, 2, 3, 4, 5], ['lf', 'lm', 'lr', 'rr', 'rm', 'rf'])
        plt.yticks([0, 1, 2, 3, 4, 5],
                ['right rear', 'right middle', 'right front', 'left rear', 'left middle', 'left front'])
        # plt.set_yticklabels(['FL', 'ML','HL','FR', 'MR','HR'], size= 18)
        # self.ax_footfall.set_yticklabels(['FL', 'ML','HL','FR', 'MR','HR'], size= 18)
        plt.grid()
        plt.show()


if __name__ == '__main__':
    if len(sys.argv) == 2:
        duration = 60
        plot_stability_data_to_footfall_pattern(duration)
