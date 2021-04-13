#!/usr/bin/env python3
import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC

""" **** Graphic methods: Simple drawing of the body model **************************
    """


class BodyModelVisualization:
    def __init__(self, body_model):
        self.body_model = body_model
        self.segm1_in_global = numpy.array([-0.24, 0.0, RSTATIC.stance_height])
        self.foot_global = [(self.segm1_in_global + self.body_model.front_vect[0]),
                            (self.segm1_in_global + self.body_model.front_vect[1]),
                            (self.segm1_in_global + self.body_model.front_vect[2]),
                            (self.segm1_in_global + self.body_model.front_vect[3]),
                            (self.segm1_in_global + self.body_model.front_vect[4]),
                            (self.segm1_in_global + self.body_model.front_vect[5])]
        self.leg_lines, self.fig_3d = self.initialise_drawing_window()

    def initialise_drawing_window(self):
        py.ion()
        fig = plt.figure(figsize=(12, 6))
        # py.rcParams['figure.figsize'] = 2, 2
        for i in range(0, 6):
            print("{}: mmc.get_leg_triangle({})[1] = {}".format(i, i, self.get_leg_triangle(i)[1]))
            py.plot(self.get_leg_triangle(i)[0], self.get_leg_triangle(i)[1], linestyle=':',
                    linewidth=1.0, color='gray', marker='')
        leg_lines = [
            py.plot(self.get_leg_triangle(i)[0], self.get_leg_triangle(i)[1], linewidth=1.0,
                    color='gray', marker='o', alpha=0.7, mfc='gray')[0] for i in range(0, 6)]
        plt.axis('scaled')
        py.xlim(-0.5, 7.5)
        py.ylim(-0.5, 5.0)
        py.ioff()
        py.draw()
        plt.pause(0.000001)
        return leg_lines, fig

    def draw_manipulator(self):
        """ The draw method for the manipulator leg.
            It is called from the outside iteration loop.
        """
        # Update two dimensional top view
        # Update current legs
        # segment_vecs = [[self.get_leg_triangle(i)[0][2] - self.get_leg_triangle(i)[0][3], self.get_leg_triangle(i)[1][2] - self.get_leg_triangle(i)[1][3]] for i in range(0,1)]
        # print("segm_post_ant = {}".format(segment_vecs))
        # print("segm_post_ant norm = {}".format([numpy.linalg.norm(i) for i in segment_vecs]))
        # print("segm_post_ant angle = {}".format([atan2(vec[1], vec[0]) for vec in segment_vecs]))
        for i in range(0, 6):
            self.leg_lines[i].set_marker('o')
            if self.body_model.gc[i]:
                py.setp(self.leg_lines[i], linestyle='-', linewidth=2.0, color='green', marker='o', alpha=0.7,
                        mfc='gray')
            else:
                # py.setp(self.leg_lines[i], visible=False)
                py.setp(self.leg_lines[i], linestyle='-', linewidth=2.0, color='gray', marker='o', alpha=0.5,
                        mfc='gray')
            self.leg_lines[i].set_xdata(self.get_leg_triangle(i)[0][0:5])
            self.leg_lines[i].set_ydata(self.get_leg_triangle(i)[1][0:5])
        # plt.axis('scaled')
        py.draw()
        plt.pause(0.000001)

    ##	Extract the global positions of the feet, the segments, diagonals ...
    def get_leg_triangle_g(self, leg):
        if self.body_model.mathplot_viz:
            return ([[self.foot_global[leg][0], (self.foot_global[leg][0] - self.body_model.leg_vect[leg][0]),
                      (self.foot_global[leg][0] - self.body_model.leg_vect[leg][0] +
                       self.body_model.segm_leg_post[leg][0]),
                      (self.foot_global[leg][0] - self.body_model.leg_vect[leg][0] +
                       self.body_model.segm_leg_post[leg][0] + self.body_model.segm_post_ant[0]),
                      # self.segm_post_ant[leg // 2][0]), (self.foot_global[leg][0] - self.leg_vect[leg][0])],
                      (self.foot_global[leg][0] - self.body_model.leg_vect[leg][0])],
                     [self.foot_global[leg][1], (self.foot_global[leg][1] - self.body_model.leg_vect[leg][1]),
                      (self.foot_global[leg][1] - self.body_model.leg_vect[leg][1] +
                       self.body_model.segm_leg_post[leg][1]),
                      (self.foot_global[leg][1] - self.body_model.leg_vect[leg][1] +
                       self.body_model.segm_leg_post[leg][1] + self.body_model.segm_post_ant[1]),
                      # self.segm_post_ant[leg // 2][1]), (self.foot_global[leg][1] - self.leg_vect[leg][1])],
                      (self.foot_global[leg][1] - self.body_model.leg_vect[leg][1])],
                     [self.foot_global[leg][2], (self.foot_global[leg][2] - self.body_model.leg_vect[leg][2]),
                      (self.foot_global[leg][2] - self.body_model.leg_vect[leg][2] +
                       self.body_model.segm_leg_post[leg][2]),
                      (self.foot_global[leg][2] - self.body_model.leg_vect[leg][2] +
                       self.body_model.segm_leg_post[leg][2] + self.body_model.segm_post_ant[2]),
                      # self.segm_post_ant[leg // 2][2]), (self.foot_global[leg][2] - self.leg_vect[leg][2])]])
                      (self.foot_global[leg][2] - self.body_model.leg_vect[leg][2])]
                     ])

    ##	Extract the global positions of the feet, the segments, diagonals ...
    def get_leg_triangle(self, leg):
        if self.body_model.mathplot_viz:
            origin_foot_vec_x = self.body_model.segm_post_ant[0] / 2 + self.body_model.front_vect[leg][0]
            origin_foot_vec_y = self.body_model.segm_post_ant[1] / 2 + self.body_model.front_vect[leg][1]
            origin_foot_vec_z = self.body_model.segm_post_ant[2] / 2 + self.body_model.front_vect[leg][2]
            return ([[origin_foot_vec_x, (origin_foot_vec_x - self.body_model.leg_vect[leg][0]),
                      (origin_foot_vec_x - self.body_model.leg_vect[leg][0] +
                       self.body_model.segm_leg_post[leg][0]),
                      (origin_foot_vec_x - self.body_model.leg_vect[leg][0] +
                       self.body_model.segm_leg_post[leg][0] + self.body_model.segm_post_ant[0]),
                      # self.segm_post_ant[leg // 2][0]), (self.foot_global[leg][0] - self.leg_vect[leg][0])],
                      (origin_foot_vec_x - self.body_model.leg_vect[leg][0])],
                     [origin_foot_vec_y,
                      (origin_foot_vec_y - self.body_model.leg_vect[leg][1]),
                      (origin_foot_vec_y - self.body_model.leg_vect[leg][1] +
                       self.body_model.segm_leg_post[leg][1]),
                      (origin_foot_vec_y - self.body_model.leg_vect[leg][1] +
                       self.body_model.segm_leg_post[leg][1] + self.body_model.segm_post_ant[1]),
                      # self.segm_post_ant[leg // 2][1]), (self.foot_global[leg][1] - self.leg_vect[leg][1])],
                      (origin_foot_vec_y - self.body_model.leg_vect[leg][1])],
                     [origin_foot_vec_z,
                      (origin_foot_vec_z - self.body_model.leg_vect[leg][2]),
                      (origin_foot_vec_z - self.body_model.leg_vect[leg][2] +
                       self.body_model.segm_leg_post[leg][2]),
                      (origin_foot_vec_z - self.body_model.leg_vect[leg][2] +
                       self.body_model.segm_leg_post[leg][2] + self.body_model.segm_post_ant[2]),
                      # self.segm_post_ant[leg // 2][2]), (self.foot_global[leg][2] - self.leg_vect[leg][2])]])
                      (origin_foot_vec_z - self.body_model.leg_vect[leg][2])]
                     ])

    def update_foot_global(self, leg_nr):
        for i in range(0, 6):
            if self.body_model.gc[i]:
                if i > leg_nr:
                    self.foot_global[leg_nr] = self.foot_global[i] + self.body_model.footdiag[i][leg_nr]
                # elif i < leg_nr:
                else:
                    # rospy.loginfo("foot_global = {}; idx = {}, i = {}, footdiag = {} idx1 = {}, idx2 = {}".format(
                    #        self.foot_global, leg_nr, i, self.footdiag, leg_nr, i))
                    self.foot_global[leg_nr] = self.foot_global[i] - self.body_model.footdiag[leg_nr][i]
                break
