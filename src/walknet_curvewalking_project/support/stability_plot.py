#!/usr/bin/env python3
import sys

import matplotlib.pyplot as plt
import numpy as np

from walknet_curvewalking_project.support import stability


def check_stability(projected_com, foot_positions):
    if len(foot_positions) > 0:
        convex_hull_points = stability.convex_hull(list(foot_positions))

        if projected_com is None:
            print("Unstable! Not enough legs on ground temp_foot_positions = " + str(foot_positions))
            return False

        # If the center of mass lies inside the support polygon
        if not stability.is_point_inside_convex_hull(convex_hull_points, projected_com):
            # print("Unstable!")
            return False
    return True


def calculate_projected_com(values, com):
    # print("original values = " + str(values))
    # ee_on_ground = values.remove(0.0)
    ee_on_ground = [x for x in values if x != 0.0]
    # print("ee on ground = " + str(ee_on_ground))
    temp_foot_positions = np.reshape(ee_on_ground, (-1, 3))
    # print("temp_foot_positions = " + str(temp_foot_positions))
    projected_com = stability.project_com_onto_ground_plane(temp_foot_positions, com)
    if projected_com is None:
        # print("Unstable! Not enough legs on ground temp_foot_positions = " + str(temp_foot_positions))
        return False

    # pcom_error = numpy.linalg.norm(projected_com[:-1] - com[:-1])
    # rospy.loginfo("pcom xy error: pcom = {} pcom xy = {}, com = {} com xy = {} pcom xy error = {}".format(projected_com, projected_com[:-1], com, com[:-1], pcom_error))
    # str_list.append(";{x};{y};{z}".format(x=projected_com[0], y=projected_com[1], z=projected_com[2]))
    # if not stability.is_point_inside_convex_hull(convex_hull_points, projected_com):
    return projected_com[0], projected_com[1]


def plot_stability_data():
    # X, Y = [], []
    bins = [0, 0, 0, 0, 0, 0]
    legs_with_gc = [0, 0, 0, 0]
    max_pcom_err = 0
    min_pcom_err = float('inf')
    average_pcom_err = 0
    step_count = 0
    last_unstable = -1
    unstable_after_5sec = 0
    unstable_before_5sec = 0
    start_time = None
    first_line = True
    plot = False
    plt.figure()
    plt.xlim(-0.3, 0.3)
    plt.ylim(-0.4, 0.4)
    for line in open(str(sys.argv[1]), 'r'):
        if first_line:
            first_line = False
            pass
        else:
            # Clear old plot
            plt.clf()

            line = line.rstrip("\n")
            try:
                values = [float(s) for s in line.split(";")]
            except ValueError:
                tmp = line.split(";")
                # print("time = " + str(tmp[0]))
                tmp_time = tmp[0].split(".")
                tmp[0] = tmp_time[0] + '.' + tmp_time[2]
                # print("time = " + str(tmp[0]))
                values = [float(s) for s in tmp]
            if start_time is None:
                start_time = values[0]
            foot_polygon = []
            first_leg = -1
            column_idx = 1
            legs_with_gc_count = 0
            while column_idx < 19:
                # print("column_idx = {}, len(values) = {}".format(column_idx, len(values)))
                if values[column_idx] != 0.0 and values[column_idx + 1] != 0.0:
                    legs_with_gc_count += 1
                    if first_leg == -1:
                        first_leg = column_idx
                    # append_value_to_plot_polygons(values[column_idx], values[column_idx + 1], X, Y, A, B, C, D, E, F, G, H)
                    foot_polygon.append([values[column_idx], values[column_idx + 1]])
                    # append_foot_position_to_polygons(values[column_idx], values[column_idx + 1], foot_positions_poly1,
                    #        foot_positions_poly2, foot_positions_poly3, foot_positions_poly4, foot_positions_poly5)
                column_idx += 3
            # append_value_to_plot_polygons(values[first_leg], values[first_leg + 1], X, Y, A, B, C, D, E, F, G, H)
            legs_with_gc[legs_with_gc_count - 3] += 1

            centroid_pt = centroid(foot_polygon)

            if plot:
                marker = np.matrix([round(values[19], 4), round(values[20], 4)]).T
                plt.plot(marker.T[:, 0], marker.T[:, 1], 'xb')
                marker = np.matrix(centroid_pt).T
                plt.plot(marker.T[:, 0], marker.T[:, 1], 'og')

            polygon_list = generate_bin_polygons(centroid_pt, foot_polygon, 5)
            # if len(values) >= 24:
            # print("values = " + str(values))
            p_com_x, p_com_y = calculate_projected_com(values[1:19], [values[19], values[20], values[21]])
            stable = False
            if plot:
                # marker = np.matrix([round(values[22], 4), round(values[23], 4)]).T
                # marker = np.matrix([round(values[19], 4), round(values[20], 4)]).T
                marker = np.matrix([round(p_com_x, 4), round(p_com_y, 4)]).T
                plt.plot(marker.T[:, 0], marker.T[:, 1], 'xr')
            for i in range(0, len(polygon_list)):  # polygon in polygon_list:
                # print("check bin " + str(polygon_list.index(polygon)))
                # if check_stability([values[22], values[23]], polygon_list[i]):
                # if check_stability([values[19], values[20]], polygon_list[i]):
                if check_stability([p_com_x, p_com_y], polygon_list[i]):
                    bins[i] += 1
                    stable = True
                    # print("increase bin " + str(polygon_list.index(polygon)))
                    break
            if not stable:
                bins[5] += 1
                last_unstable = step_count
                if values[0] - start_time > 5:
                    unstable_after_5sec += 1
                elif values[0] - start_time <= 5:
                    unstable_before_5sec += 1

            # pcom_err = np.linalg.norm(np.array([values[19], values[20]]) - np.array([values[22], values[23]]))
            # if pcom_err > max_pcom_err:
            #     max_pcom_err = pcom_err
            # if pcom_err < min_pcom_err:
            #     min_pcom_err = pcom_err
            # average_pcom_err += pcom_err
            step_count += 1
            # print("step_count = " + str(step_count))
            # else:
            #     print("unstable len(values) " + str(len(values)) + " >= 24")
            #     bins[5] += 1
            #     last_unstable = step_count
            #     # print("unstable")

            if plot:
                A = [point[0] for point in polygon_list[4]]
                A.append(polygon_list[4][0][0])
                A2 = [point[1] for point in polygon_list[4]]
                A2.append(polygon_list[4][0][1])
                B = [point[0] for point in polygon_list[3]]
                B.append(polygon_list[3][0][0])
                B2 = [point[1] for point in polygon_list[3]]
                B2.append(polygon_list[3][0][1])
                C = [point[0] for point in polygon_list[2]]
                C.append(polygon_list[2][0][0])
                C2 = [point[1] for point in polygon_list[2]]
                C2.append(polygon_list[2][0][1])
                D = [point[0] for point in polygon_list[1]]
                D.append(polygon_list[1][0][0])
                D2 = [point[1] for point in polygon_list[1]]
                D2.append(polygon_list[1][0][1])
                E = [point[0] for point in polygon_list[0]]
                E.append(polygon_list[0][0][0])
                E2 = [point[1] for point in polygon_list[0]]
                E2.append(polygon_list[0][0][1])
                plt.plot(A, A2)
                plt.plot(B, B2)
                plt.plot(C, C2)
                plt.plot(D, D2)
                plt.plot(E, E2)

                # for i in range(25, len(values), 3):
                #    plt.plot([round(values[22], 4), round(values[i], 4)], [round(values[23], 4), round(values[i + 1], 4)])

                plt.xlim(-0.3, 0.3)
                plt.ylim(-0.4, 0.4)

                # plt.grid()
                plt.draw()
                plt.pause(0.0001)

                # input('Press ENTER to continue...')

    print("bins: middle = {}, bin 2 = {}, bin 3 = {}, bin 4 = {}, closest to border = {}, unstable = {}".format(bins[0],
            bins[1], bins[2], bins[3], bins[4], bins[5]))
    print("legs with ground contact: 3 = {}, 4 = {}, 5 = {}, 6 = {}".format(legs_with_gc[0], legs_with_gc[1],
            legs_with_gc[2], legs_with_gc[3]))
    print("average number of legs with ground contact = " + str(
            (3 * legs_with_gc[0] + 4 * legs_with_gc[1] + 5 * legs_with_gc[2] + 6 * legs_with_gc[3]) / step_count))
    # average_pcom_err = average_pcom_err / step_count
    # print("average pcom error = {}, max pcom error = {} min pcom error = {}".format(average_pcom_err, max_pcom_err,
    #         min_pcom_err))
    print("step_count = " + str(step_count))
    print("")
    print("**last unstable controller step:** " + str(last_unstable) + " / " + str(step_count))
    print("**unstable total:** " + str(bins[5]) + " / " + str(step_count))
    print("**unstable after 5sec:** " + str(unstable_after_5sec) + " / " + str(step_count))
    print("**unstable before 5sec:** " + str(unstable_before_5sec) + " / " + str(step_count))


# https://progr.interplanety.org/en/python-how-to-find-the-polygon-center-coordinates/
def centroid(vertexes):
    _x_list = [vertex[0] for vertex in vertexes]
    _y_list = [vertex[1] for vertex in vertexes]
    _len = len(vertexes)
    _x = sum(_x_list) / _len
    _y = sum(_y_list) / _len
    return _x, _y


def generate_bin_polygons(centroid_pt, polygon, number_of_bins):
    # polygons = [number_of_bins][len(polygon)][2]
    polygons = np.empty((number_of_bins, len(polygon), 2), dtype=object)
    for i in range(0, len(polygon)):
        for j in range(0, number_of_bins):
            # print("polygon x = {} y = {}".format(polygon[i][0], polygon[i][1]))
            polygons[j][i][0] = centroid_pt[0] + ((polygon[i][0] - centroid_pt[0]) * (j + 1) / number_of_bins)
            polygons[j][i][1] = centroid_pt[1] + ((polygon[i][1] - centroid_pt[1]) * (j + 1) / number_of_bins)
            # print("polygon {} x = {} y = {}".format(j, polygons[j][i][0], polygons[j][i][1]))
    return polygons


def append_value_to_plot_polygons(value_x, value_y, X, Y, A, B, C, D, E, F, G, H):
    X.append(round(value_x, 4))
    Y.append(round(value_y, 4))
    A.append(round(value_x, 4) * (4 / 5))
    B.append(round(value_y, 4) * (4 / 5))
    C.append(round(value_x, 4) * (3 / 5))
    D.append(round(value_y, 4) * (3 / 5))
    E.append(round(value_x, 4) * (2 / 5))
    F.append(round(value_y, 4) * (2 / 5))
    G.append(round(value_x, 4) * (1 / 5))
    H.append(round(value_y, 4) * (1 / 5))


if __name__ == '__main__':
    if len(sys.argv) == 2:
        plot_stability_data()
