import os
import sys

import matplotlib.pyplot as plt
import tf.transformations as tf_trans

# uses walknet_position_ files
if len(sys.argv) >= 2:

    duration = 60
    files = None
    if sys.argv[1] == "-dir":
        files = os.listdir(sys.argv[2])
        print(files)
    elif len(sys.argv) == 2:
        files = [sys.argv[1]]
    else:
        print("wrong parameters provide data-filename or -dir directory with files")
        exit()

    orientation_z, time = [], []
    for i in range(0, len(files)):
        orientation_z.append([])
        time.append([])

    plt.figure()

    for j in range(0, len(files)):
        first_line = True
        last_time = None
        last_orientation = None

        line_number = 0
        used_lines = 0
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
                #if line_number % 5 == 0:
                used_lines += 1
                line = line.rstrip("\n")
                values = [float(s) for s in line.split(";")]
                #print("time = " + str(values[0]))
                if duration != 0:
                    # print("values[0] {} > duration {}".format(values[0], duration))
                    if values[0] > duration:
                        break

                angles = tf_trans.euler_from_quaternion([values[4], values[5], values[6], values[7]])
                #print("angles = " + str(angles))
                if not last_orientation is None:
                    orientation_z[j].append(angles[2] - last_orientation)
                    time[j].append(values[0])
                last_orientation = angles[2]

                line_number += 1

        print("used lines = " + str(used_lines) + " of total lines = " + str(line_number))
        #print("orientation_z = " + str(orientation_z))
        #print("time = " + str(time))
        plt.plot(time[j], orientation_z[j])
        plt.legend([i.split("_")[2] + "_" + i.split("_")[3] for i in files], loc='upper right')

    # plt.figure()
    # plt.plot(X, Y)

    # plt.figure()
    # plt.plot(Z)

    plt.grid()
    plt.show()
