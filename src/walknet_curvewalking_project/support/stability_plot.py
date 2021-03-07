import numpy as np
import matplotlib.pyplot as plt
import sys

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


def plot_stability_data():
    # X, Y = [], []
    bins = [0, 0, 0, 0, 0, 0]
    first_line = True
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
            values = [float(s) for s in line.split(";")]
            foot_polygon = []
            first_leg = -1
            count = 1
            while count < 19:
                if values[count] != 0.0 and values[count + 1] != 0.0:
                    if first_leg == -1:
                        first_leg = count
                    # append_value_to_plot_polygons(values[count], values[count + 1], X, Y, A, B, C, D, E, F, G, H)
                    foot_polygon.append([values[count], values[count + 1]])
                    # append_foot_position_to_polygons(values[count], values[count + 1], foot_positions_poly1,
                    #        foot_positions_poly2, foot_positions_poly3, foot_positions_poly4, foot_positions_poly5)
                count += 3
            # append_value_to_plot_polygons(values[first_leg], values[first_leg + 1], X, Y, A, B, C, D, E, F, G, H)

            marker = np.matrix([round(values[19], 4), round(values[20], 4)]).T
            plt.plot(marker.T[:, 0], marker.T[:, 1], 'xb')

            centroid_pt = centroid(foot_polygon)
            marker = np.matrix(centroid_pt).T
            plt.plot(marker.T[:, 0], marker.T[:, 1], 'og')

            polygon_list = generate_bin_polygons(centroid_pt, foot_polygon, 5)
            if len(values) >= 24:
                marker = np.matrix([round(values[22], 4), round(values[23], 4)]).T
                for i in range(0, len(polygon_list)):  # polygon in polygon_list:
                    # print("check bin " + str(polygon_list.index(polygon)))
                    if check_stability([values[22], values[23]], polygon_list[i]):
                        bins[i] += 1
                        # print("increase bin " + str(polygon_list.index(polygon)))
                        break
                plt.plot(marker.T[:, 0], marker.T[:, 1], 'xr')
            else:
                bins[5] += 1
                # print("unstable")

            # plt.plot(X, Y)
            # plt.plot(A, B)
            # plt.plot(C, D)
            # plt.plot(E, F)
            # plt.plot(G, H)
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

            plt.draw()
            plt.pause(0.0001)

            # input('Press ENTER to continue...')

    print("bins: middle = {}, bin 2 = {}, bin 3 = {}, bin 4 = {}, closest to border = {}, unstable = {}".format(bins[0],
            bins[1], bins[2], bins[3], bins[4], bins[5]))


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
