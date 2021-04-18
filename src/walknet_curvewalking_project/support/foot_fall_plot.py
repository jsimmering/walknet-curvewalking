#!/usr/bin/env python3
import re
import sys

import matplotlib.pyplot as plt


# uses walknet_stability_ files
def plot_stability_data_to_footfall_pattern(start, end):
    stance_times = [[], [], [], [], [], []]
    last_state_swing = [True, True, True, True, True, True]
    first_line = True
    line_count = 0
    plot = True
    plt.figure()
    for line in open(str(sys.argv[1]), 'r'):
        # print("line count = " + str(line_count))
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
            if end != 0:
                if values[0] > end:
                    break
                if values[0] < start:
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
            for i in range(0, len(leg) - 1):
                # for step in leg:
                # print("leg index = {} ['lf', 'lm', 'lr', 'rr', 'rm', 'rf']".format(stance_times.index(leg)))
                # plt.plot([step[0], step[1]], [leg_order[stance_times.index(leg)], leg_order[stance_times.index(leg)]],
                #         linestyle='-', linewidth=20.0, color='black', marker='', solid_capstyle="butt")
                plt.plot([leg[i][1], leg[i + 1][0]],
                        [leg_order[stance_times.index(leg)], leg_order[stance_times.index(leg)]],
                        linestyle='-', linewidth=20.0, color='black', marker='', solid_capstyle="butt")

        if end != 0:
            plt.xlim(start, end)
        plt.ylim(-0.5, 5.5)
        # plt.yticks([0, 1, 2, 3, 4, 5], ['lf', 'lm', 'lr', 'rr', 'rm', 'rf'])
        plt.yticks([0, 1, 2, 3, 4, 5],
                ['right rear', 'right middle', 'right front', 'left rear', 'left middle', 'left front'])
        # plt.set_yticklabels(['FL', 'ML','HL','FR', 'MR','HR'], size= 18)
        # self.ax_footfall.set_yticklabels(['FL', 'ML','HL','FR', 'MR','HR'], size= 18)

        # plt.rc('xtick', labelsize=20)
        # plt.rc('ytick', labelsize=20)
        # plt.rcParams.update({'font.size': 40})
        plt.tick_params(labelsize=20)

        plt.grid()
        plt.show()

        plt.subplots_adjust(top=2, bottom=0, right=2, left=0, hspace=1, wspace=1)
        plt.margins(1, 1)

        split = re.findall(r"[^/_,]+", str(sys.argv[1]), re.ASCII)
        print("split = " + str(split))
        name = "_".join(split[split.index("50hz") + 1:])
        print("name = " + name)
        print("single file: " + "/home/jsimmering/plots_masterthesis/footfall/foot_fall_" + name + ".pdf")

        #plt.savefig("/home/jsimmering/plots_masterthesis/footfall/foot_fall_" + name + ".png", bbox_inches='tight',
        #        pad_inches=0)


if __name__ == '__main__':
    if len(sys.argv) == 2:
        start_time = 90  # 60
        end_time = 120  # 90
        plot_stability_data_to_footfall_pattern(start_time, end_time)
