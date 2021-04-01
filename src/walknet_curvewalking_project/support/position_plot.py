import os
import sys

import matplotlib.pyplot as plt
import numpy as np

if len(sys.argv) >= 2:

    plot = True

    files = None
    if sys.argv[1] == "-dir":
        files = os.listdir(sys.argv[2])
        print(files)
    elif len(sys.argv) == 2:
        files = [sys.argv[1]]
    else:
        print("wrong parameters provide data-filename or -dir directory with files")
        exit()

    distance = [0 for i in range(0, len(files))]
    X, Y, Z, start_x, start_y, start_z, velocity, x_dim, y_dim = [], [], [], [], [], [], [], [], []
    for i in range(0, len(files)):
        X.append([])
        Y.append([])
        Z.append([])
        start_x.append([])
        start_y.append([])
        start_z.append([])
        velocity.append([])
        x_dim.append([])
        y_dim.append([])

    if plot:
        plt.figure()

    for j in range(0, len(files)):
        first_line = True
        last_position = None
        first_position = None
        last_time = None
        x_min = float('inf')
        x_max = float('-inf')
        y_min = float('inf')
        y_max = float('-inf')

        line_number = 0
        file_name = None
        if len(sys.argv) > 2:
            file_name = sys.argv[2] + "/" + str(files[j])
        else:
            file_name = str(files[j])
        for line in open(file_name, 'r'):
            if first_line:
                first_line = False
                pass
            else:
                if line_number % 1000 == 0:
                    line = line.rstrip("\n")
                    values = [float(s) for s in line.split(";")]
                    # print("time = " + str(values[0]))
                    # if start_x is None:
                    if not start_x[j]:
                        start_x[j].append(values[1])
                        # print("startx = " + str(start_x) + " j = " + str(j))
                        start_y[j].append(values[2])
                        start_z[j].append(values[3])

                    if first_position is None:
                        first_position = [values[1], values[2]]

                    # print("x = " + str(round(values[1],4)) + "y = " + str(round(values[2],4)) + "z = " + str(round(values[3],4)))
                    X[j].append(round(values[1] - start_x[j][0], 4))
                    Y[j].append(round(values[2] - start_y[j][0], 4))
                    Z[j].append(round(values[3] - start_z[j][0], 4))

                    if values[1] > x_max:
                        x_max = values[1]
                    if values[1] < x_min:
                        x_min = values[1]
                    if values[2] > y_max:
                        y_max = values[2]
                    if values[2] < y_min:
                        y_min = values[2]

                    # print("gazebo velocity = " + str(gazebo_vel))
                    if last_position is not None:
                        dis = np.linalg.norm(np.array([values[1], values[2]]) - np.array(last_position))
                        distance[j] += dis
                        vel = dis / (values[0] - last_time)
                        velocity[j].append(vel)
                    last_position = [values[1], values[2]]
                    last_time = values[0]
                line_number += 1

        x_dim[j].append(x_max - x_min)
        y_dim[j].append(y_max - y_min)
        if plot:
            plt.plot(X[j], Y[j])
            plt.legend([i.split("_")[2] + "_" + i.split("_")[3] for i in files], loc='upper right')

    for i in range(0, len(files)):
        print("steps = " + str(len(velocity[i])))
        print("average velocity = " + str(np.sum(velocity[i]) / len(velocity[i])))

    print("circle dimensions x = " + str(x_dim))
    print("circle dimensions y = " + str(y_dim))

    print("distance traveled = " + str(distance))
    print("first position = " + str(first_position) + " last position = " + str(last_position) + " distance = " + str(
            np.linalg.norm(np.array(last_position) - np.array(first_position))))

    if plot:
        ## --- for height plot
        # plt.figure()
        # plt.plot(X, Y)

        # plt.figure()
        # plt.plot(Z)
        ## -----

        plt.grid()
        plt.show()


