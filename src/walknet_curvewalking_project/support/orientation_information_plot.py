import sys
import re

import matplotlib.pyplot as plt
import tf.transformations as tf_trans
from matplotlib.widgets import Slider


def plot_orientation_data(axs):
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
                used_lines += 1
                line = line.rstrip("\n")
                values = [float(s) for s in line.split(";")]
                # print("time = " + str(values[0]))

                angles = tf_trans.euler_from_quaternion([values[4], values[5], values[6], values[7]])
                # print("angles = " + str(angles))
                if not last_orientation is None:
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
        print("time = " + str(time))
        # plt.plot(time[j], orientation_z[j])
        axs[0].plot(time[j], orientation_z[j])
        # plt.plot(time[j], orientation_diff[j])
        axs[1].plot(time[j], orientation_diff[j])
        plt.legend([i.split("_")[2] + "_" + i.split("_")[3] for i in files], loc='upper right')

    # plt.figure()
    # plt.plot(X, Y)

    # plt.figure()
    # plt.plot(Z)

    # plt.show()
    return axs


# uses walknet_stability_ files
def plot_stability_data(axs):
    legs = [[], [], [], [], [], []]
    time = []
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
            # Clear old plot
            # plt.clf()

            line = line.rstrip("\n")
            # print(line)
            try:
                values = [float(s) for s in line.split(";")]
            except ValueError:
                tmp = line.split(";")
                # print("time = " + str(tmp[0]))
                tmp_time = tmp[0].split(".")
                tmp[0] = tmp_time[0] + '.' + tmp_time[2]
                # print("time = " + str(tmp[0]))
                values = [float(s) for s in tmp]

            time.append(values[0])
            column_idx = 1
            while column_idx < 19:
                # print("column_idx = {}, len(values) = {}".format(column_idx, len(values)))
                if values[column_idx] != 0.0 and values[column_idx + 1] != 0.0:
                    legs[column_idx // 3].append(column_idx // 3)
                    # marker = np.matrix([values[0], column_idx // 3]).T
                    # plt.plot(marker.T[:, 0], marker.T[:, 1], 'bs')
                else:
                    legs[column_idx // 3].append(-1)
                column_idx += 3
            line_count += 1

    if plot:
        print("time = " + str(time))
        A = [point for point in legs[0]]
        # print(str(A))
        B = [point for point in legs[1]]
        C = [point for point in legs[2]]
        D = [point for point in legs[3]]
        E = [point for point in legs[4]]
        F = [point for point in legs[5]]
        # plt.plot(time, A, 'bs')
        # plt.plot(time, B, 'bs')
        # plt.plot(time, C, 'bs')
        # plt.plot(time, D, 'bs')
        # plt.plot(time, E, 'bs')
        # plt.plot(time, F, 'bs')
        axs[2].plot(time, A, 'bs')
        axs[2].plot(time, B, 'bs')
        axs[2].plot(time, C, 'bs')
        axs[2].plot(time, D, 'bs')
        axs[2].plot(time, E, 'bs')
        axs[2].plot(time, F, 'bs')

        # plt.xlim(-0.3, 0.3)
        plt.ylim(-0.5, 5.5)
        #
        # plt.draw()
        # plt.pause(0.0001)

        # input('Press ENTER to continue...')
        # plt.show()
        return axs


if __name__ == '__main__':
    if len(sys.argv) == 3:
        fig, axs = plt.subplots(3)
        axs = plot_orientation_data(axs)
        axs = plot_stability_data(axs)

        plt.show()
