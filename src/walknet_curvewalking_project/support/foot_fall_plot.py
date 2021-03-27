import sys

import matplotlib.pyplot as plt
import numpy as np

from walknet_curvewalking_project.support import stability


# uses walknet_stability_ files
def plot_stability_data():
    legs = [[], [], [], [], [], []]
    time = []
    first_line = True
    line_count = 0
    plot = True
    plt.figure()
    # plt.xlim(-0.3, 0.3)
    # plt.ylim(-0.4, 0.4)
    for line in open(str(sys.argv[1]), 'r'):
        if first_line:
            first_line = False
            pass
        else:
            # Clear old plot
            # plt.clf()

            line = line.rstrip("\n")
            #print(line)
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
                #print("column_idx = {}, len(values) = {}".format(column_idx, len(values)))
                if values[column_idx] != 0.0 and values[column_idx + 1] != 0.0:
                    legs[column_idx // 3].append(column_idx // 3)
                    #marker = np.matrix([values[0], column_idx // 3]).T
                    #plt.plot(marker.T[:, 0], marker.T[:, 1], 'bs')
                else:
                    legs[column_idx // 3].append(-1)
                column_idx += 3
            line_count += 1

    if plot:
        print("time = " + str(time))
        A = [point for point in legs[0]]
        #print(str(A))
        B = [point for point in legs[1]]
        C = [point for point in legs[2]]
        D = [point for point in legs[3]]
        E = [point for point in legs[4]]
        F = [point for point in legs[5]]
        plt.plot(time, A, 'bs')
        plt.plot(time, B, 'bs')
        plt.plot(time, C, 'bs')
        plt.plot(time, D, 'bs')
        plt.plot(time, E, 'bs')
        plt.plot(time, F, 'bs')

        # plt.xlim(-0.3, 0.3)
        plt.ylim(-0.5, 5.5)
        #
        # plt.draw()
        # plt.pause(0.0001)

        # input('Press ENTER to continue...')
        plt.show()


if __name__ == '__main__':
    if len(sys.argv) == 2:
        plot_stability_data()
