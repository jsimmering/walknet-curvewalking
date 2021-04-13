import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy
from mpl_toolkits.mplot3d import Axes3D


class mmcBodyModelStance:
    """	Initialisation of the body model.
        The segment vectors are encoded here, the leg vectors are initialised here
        but are actually set when a leg is really put on the ground (initially all are
        assumed in the air, so an update of the legs is forced in the first iteration)
    """

    def __init__(self, motiv_net):
        self.motivationNetRobot = motiv_net
        self.height = -0.09  # 0.18  # WSTATIC.stanceheight
        width = 0.24

        self.ee_positions = [numpy.array([0.26, (width - 0.1), self.height]),
                             numpy.array([0.26, -(width - 0.1), self.height]),
                             numpy.array([0.0, width, self.height]),
                             numpy.array([0.0, -width, self.height]),
                             numpy.array([-0.26, (width - 0.1), self.height]),
                             numpy.array([-0.26, -(width - 0.1), self.height])]

        front_segment_start = [0.12, 0.0, 0.0]

        # Additional Vectors (from front of a segment to footpoint) estimated based on estimated end effector positions.
        self.front_vect = [numpy.array([pos[0] - front_segment_start[0], pos[1] - front_segment_start[1],
                                        pos[2] - front_segment_start[2]]) for pos in self.ee_positions]

        # Segment defining vectors: is constructed as a diamond for the mounting points of
        # the legs: segm_leg_ant = from coxa to front (anterior)
        # 			segm_leg_post = from coxa to back (posterior)
        # self.segm_post_ant = [numpy.array([0.22, 0.0, 0.0]), numpy.array([0.35, 0.0, 0.0]),
        #                       numpy.array([0.36, 0.0, 0.0])]
        # self.segm_post_ant_norm = [numpy.linalg.norm(segm_vect) for segm_vect in self.segm_post_ant]
        self.segm_post_ant = numpy.array([0.24, 0.0, 0.0])
        self.segm_post_ant_norm = numpy.linalg.norm(self.segm_post_ant)

        # positions of the shoulder c1 joints for calculating real leg vectors
        self.c1_positions = [numpy.array([0.12, 0.06, 0.0]), numpy.array([0.12, -0.06, 0.0]),
                             numpy.array([0.0, 0.10, 0.0]),
                             numpy.array([0.0, -0.10, 0.0]), numpy.array([-0.12, 0.06, 0.0]),
                             numpy.array([-0.12, -0.06, 0.0])]
        # Real Leg vectors
        # self.leg_vect = [numpy.array([0., self.width, -self.height]), numpy.array([0., -self.width, -self.height]),
        #     numpy.array([0., self.width, -self.height]), numpy.array([0., -self.width, -self.height]),
        #     numpy.array([0., self.width, -self.height]), numpy.array([0., -self.width, -self.height])]
        self.leg_vect = [numpy.array([target[0] - start[0], target[1] - start[1], target[2] - start[2]]) for
                         target, start in zip(self.ee_positions, self.c1_positions)]

        # self.segm_leg_ant = [(self.leg_vect[i] - self.front_vect[i]) for i in range(0, 6)]
        # self.segm_leg_ant = [(self.leg_vect[0] - self.front_vect[0]), (self.leg_vect[1] - self.front_vect[1])]
        # self.segm_leg_ant_norm = [numpy.linalg.norm(segm_vect) for segm_vect in self.segm_leg_ant]
        self.segm_leg_ant = [(self.leg_vect[i] - self.front_vect[i]) for i in
                             range(0, 6)]
        self.segm_leg_ant_norm = [numpy.linalg.norm(segm_vect) for segm_vect in self.segm_leg_ant]

        # self.segm_leg_post = [(self.segm_leg_ant[0] - self.segm_post_ant[0]), (self.segm_leg_ant[1] - self.segm_post_ant[0])]
        # self.segm_leg_post = [(self.segm_leg_ant[i] - self.segm_post_ant[i // 2]) for i in range(0, 6)]
        # self.segm_leg_post_norm = [numpy.linalg.norm(segm_vect) for segm_vect in self.segm_leg_post]
        self.segm_leg_post = [(self.segm_leg_ant[i] - self.segm_post_ant) for i in range(0, 6)]
        self.segm_leg_post_norm = [numpy.linalg.norm(segm_vect) for segm_vect in self.segm_leg_post]

        self.segm_diag_to_right = [(self.segm_leg_ant[i * 2] - self.segm_leg_ant[i * 2 + 1]) for i in
                                   range(0, len(self.segm_leg_ant) // 2)]
        self.segm_diag_norm = [numpy.linalg.norm(segm_vect) for segm_vect in self.segm_diag_to_right]

        # Help vectors for the drawing routines: We have to keep track of the feet
        # positions in a global coordinate system (and of one segment, too)
        # self.segm1_in_global = numpy.array([-0.02, 0.0, self.height])
        # self.foot_global = [(self.segm1_in_global + self.segm_post_ant[1] + self.segm_post_ant[0] + self.front_vect[0]),
        #                     (self.segm1_in_global + self.segm_post_ant[1] + self.segm_post_ant[0] + self.front_vect[1]),
        #                     (self.segm1_in_global + self.segm_post_ant[1] + self.front_vect[2]),
        #                     (self.segm1_in_global + self.segm_post_ant[1] + self.front_vect[3]),
        #                     (self.segm1_in_global + self.front_vect[4]),
        #                     (self.segm1_in_global + self.front_vect[5])]
        self.segm1_in_global = numpy.array([-0.02, 0.0, self.height])
        self.foot_global = [(self.segm1_in_global + self.front_vect[0]),
                            (self.segm1_in_global + self.front_vect[1]),
                            (self.segm1_in_global + self.front_vect[2]),
                            (self.segm1_in_global + self.front_vect[3]),
                            (self.segm1_in_global + self.front_vect[4]),
                            (self.segm1_in_global + self.front_vect[5])]

        # Ground contact - which feet are on the ground
        self.gc = [True, True, True, True, True, True]  # , False, False, False, False]
        #		self.old_stance_motivation = [False, False, False, False, False, False]

        # Vectors between footpoints - these are a fixed coordinate system which
        # shall not be altered. The standing feet are connected through the ground
        # and their relation shall be constant.
        # The footdiags table has to be adapted whenever the configuration of the walker
        # changes - when a leg is lifted from the ground this leg vector is not forming
        # closed kinematic chains with the other legs anymore and the footdiags can
        # not be exploited anymore.
        # When a leg is touching the ground, new footdiags to the other standing
        # legs have to be established.
        self.footdiag = [[], [0], [0, 0], [0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0, 0]]
        for i in range(0, len(self.footdiag)):
            for j in range(0, i):
                self.footdiag[i][j] = self.set_up_foot_diag(i, j)
        # The explicit disturbance vectors of the network
        # self.delta_front = [numpy.array([0, 0, 0]), numpy.array([0, 0, 0]), numpy.array([0, 0, 0])]
        self.delta_front = [numpy.array([0, 0, 0])]
        #		self.delta_back = [[numpy.array([0,0,0])], [numpy.array([0,0,0])], [numpy.array([0,0,0])]]
        # These are operated through a pull at the front
        # self.pull_front = numpy.array([0.1, 0.05, 0.0])
        self.pull_front = numpy.array([0.0075, 0.01, 0.0])
        #		self.pull_back = numpy.array([0.0,0.0,0.0])
        self.step = 0
        self.damping = 1

    """ **** Set up methods and calculation of vector methods ***************************
    """

    def set_up_foot_diag(self, start, target):
        """ Calculation of the vector connection the feet of two standing legs.
            When a leg is put on the ground a new connecting vector has to be established.
            This is calculated as the difference between the leg vectors.
        """
        diag_vec = self.leg_vect[target] - self.leg_vect[start] + \
                   self.get_segm_vectors_between_legs(start, target)
        return diag_vec

    def get_segm_vectors_between_legs(self, start_leg, end_leg):
        """ Providing the segment vectors connecting two legs.
            An equation is described by a series of vectors forming a closed kinematic
            chain (two leg vectors, the footdiag and -possibly- segment vectors in
            between). This function is calculating the segment vectors between the
            given legs.
        """
        leg_diff = (end_leg // 2 - start_leg // 2)
        if leg_diff == 0:
            return self.segm_leg_post[start_leg] - self.segm_leg_post[end_leg]
        elif abs(leg_diff) == 1 or abs(leg_diff) == 2:
            return self.segm_leg_ant[start_leg] - self.segm_leg_ant[end_leg]

    # def get_segm_vectors_between_front(self, start_leg, end_leg):
    #     """ Providing the segment vectors connecting two front positions of two segments.
    #         An equation is described by a series of vectors forming a closed kinematic
    #         chain (two leg vectors, the footdiag and -possibly- segment vectors in
    #         between). This function is calculating the segment vectors between the
    #         given legs.
    #     """
    #     leg_diff = (end_leg // 2 - start_leg // 2)
    #     if leg_diff == 0:
    #         return numpy.array([0., 0., 0.])
    #     elif abs(leg_diff) == 1:
    #         return numpy.array(-1 * leg_diff * self.segm_post_ant[min(start_leg, end_leg) // 2])
    #     elif abs(leg_diff) == 2:
    #         return (-0.5 * leg_diff * (self.segm_post_ant[0]
    #                                    + self.segm_post_ant[1]))

    """ **** Computation of the MMC equations *******************************************
    """

    def compute_leg_computations_and_integrate(self, leg_nr):
        """ Compute the leg vectors: For all standing legs
            the new leg vectors are computed, summed and the mean is calculated
            (the old value is also integrated, weighted by the damping value)
        """
        equation_counter = 1
        # segm_leg_vect = -self.delta_back[leg_nr // 2] + self.segm_leg_post[leg_nr] + self.segm_post_ant[leg_nr // 2] + \
        #                self.front_vect[leg_nr]  # 3 segments
        # segm_leg_vect = -self.delta_back + self.segm_leg_post[leg_nr] + self.segm_post_ant + self.front_vect[leg_nr]
        segm_leg_vect = self.segm_leg_post[leg_nr] + self.segm_post_ant + self.front_vect[leg_nr]
        segm_leg_vect += self.damping * self.leg_vect[leg_nr]
        equation_counter += self.damping
        for target_leg in range(0, len(self.gc)):
            if self.gc[target_leg] and target_leg != leg_nr:
                part_vec = numpy.array(self.leg_vect[target_leg]) + \
                           self.get_segm_vectors_between_legs(leg_nr, target_leg)
                if target_leg < leg_nr:
                    part_vec -= self.footdiag[leg_nr][target_leg]
                elif target_leg > leg_nr:
                    part_vec += self.footdiag[target_leg][leg_nr]
                segm_leg_vect += part_vec
                equation_counter += 1
        return segm_leg_vect / equation_counter

    def compute_front_computations_and_integrate(self, leg_nr):
        """ Compute the front vectors: For all standing legs
            the new help vectors from front to footpoint are computed,
            summed and the mean is calculated
            (the old value is also integrated, weighted by the damping value)
        """
        equation_counter = 1
        # new_front_vect = -self.delta_front[leg_nr // 2] - self.segm_post_ant[leg_nr // 2] + self.leg_vect[leg_nr] - \
        #                 self.segm_leg_post[leg_nr]  # 3 segments
        new_front_vect = -self.delta_front[0] - self.segm_post_ant + self.leg_vect[leg_nr] - self.segm_leg_post[leg_nr]
        new_front_vect += self.damping * self.front_vect[leg_nr]
        equation_counter += self.damping
        # Computations of connections between legs in ground contacts
        # Compute equations to all other standing legs using the footdiag
        for target_leg in range(0, len(self.gc)):
            if self.gc[target_leg] and target_leg != leg_nr:
                part_vec = numpy.array(self.front_vect[target_leg])
                # part_vec += self.get_segm_vectors_between_front(leg_nr, target_leg)
                if target_leg < leg_nr:
                    part_vec -= self.footdiag[leg_nr][target_leg]
                elif target_leg > leg_nr:
                    part_vec += self.footdiag[target_leg][leg_nr]
                new_front_vect += part_vec
                equation_counter += 1
        return new_front_vect / equation_counter

    def compute_segment_leg_ant_computations_and_integrate(self, leg_nr):
        """ Compute the segment vectors:
            Using equations including the two legs connected to the segment,
            integrating the explicit displacement given as delta
            and the recurrent old value of the vector.
        """
        equation_counter = 1
        # new_segm_leg_ant = self.segm_leg_post[leg_nr] + self.segm_post_ant[leg_nr // 2]
        new_segm_leg_ant = self.segm_leg_post[leg_nr] + self.segm_post_ant
        # Neighboring leg with respect to leg_nr
        new_segm_leg_ant += self.segm_leg_ant[leg_nr] * self.damping
        equation_counter += self.damping
        new_segm_leg_ant = new_segm_leg_ant / equation_counter
        # print("leg " + str(leg_nr) + " normal = " + str(numpy.linalg.norm(new_segm_leg_ant)))
        return (self.segm_leg_ant_norm[leg_nr] / numpy.linalg.norm(new_segm_leg_ant)) * new_segm_leg_ant

    def compute_segment_leg_post_computations_and_integrate(self, leg_nr):
        """ Compute the segment vectors:
            Using equations including the two legs connected to the segment,
            integrating the explicit displacement given as delta
            and the recurrent old value of the vector.
        """
        equation_counter = 1
        # new_segm_leg_post = self.segm_leg_ant[leg_nr] - self.segm_post_ant[leg_nr // 2]
        new_segm_leg_post = self.segm_leg_ant[leg_nr] - self.segm_post_ant
        new_segm_leg_post += self.segm_leg_post[leg_nr] * self.damping
        equation_counter += self.damping
        new_segm_leg_post = new_segm_leg_post / equation_counter
        # print("leg " + str(leg_nr) + " normal = " + str(numpy.linalg.norm(new_segm_leg_post)))
        return (self.segm_leg_post_norm[leg_nr] / numpy.linalg.norm(new_segm_leg_post)) * new_segm_leg_post

    def compute_segm_post_ant_computations_and_integrate(self, seg_nr):
        equation_counter = 7
        new_segm_post_ant = self.segm_post_ant + self.delta_front[0]  # - self.delta_back
        new_segm_post_ant += -self.segm_leg_post[0] + self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[1]
        new_segm_post_ant += -self.segm_leg_post[1] - self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[0]
        new_segm_post_ant += -self.segm_leg_post[2] + self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[3]
        new_segm_post_ant += -self.segm_leg_post[3] - self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[2]
        new_segm_post_ant += -self.segm_leg_post[4] + self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[5]
        new_segm_post_ant += -self.segm_leg_post[5] - self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[4]

        new_segm_post_ant += self.damping * self.segm_post_ant
        equation_counter += self.damping

        new_segm_div = new_segm_post_ant / equation_counter
        return (self.segm_post_ant_norm / numpy.linalg.norm(new_segm_div)) * new_segm_div

    def compute_segm_diag_computations_and_integrate(self, joint_nr):
        equation_counter = 2

        new_segm_diag = self.segm_leg_post[2 * joint_nr] + self.segm_post_ant - self.segm_leg_ant[1 + joint_nr * 2]
        new_segm_diag -= self.segm_leg_post[1 + 2 * joint_nr] + self.segm_post_ant - self.segm_leg_ant[joint_nr * 2]

        new_segm_diag += self.damping * self.segm_diag_to_right[joint_nr]
        equation_counter += self.damping
        new_segm_diag = new_segm_diag / equation_counter
        return (self.segm_diag_norm[joint_nr] / numpy.linalg.norm(new_segm_diag)) * new_segm_diag

    def mmc_iteration_step(self):
        self.delta_front = self.pull_front
        # self.delta_back = self.pull_back

        front_vect = [self.compute_front_computations_and_integrate(i) for i in range(0, 6)]
        leg_vect = [self.compute_leg_computations_and_integrate(i) for i in range(0, 6)]
        segm_leg_ant = [self.compute_segment_leg_ant_computations_and_integrate(i) for i in range(0, 6)]
        segm_leg_post = [self.compute_segment_leg_post_computations_and_integrate(i) for i in range(0, 6)]
        segm_post_ant = self.compute_segm_post_ant_computations_and_integrate(0)
        segm_diag_to_right = [self.compute_segm_diag_computations_and_integrate(i) for i in range(0, 3)]

        for i in range(0, 6):
            self.segm_leg_ant[i] = segm_leg_ant[i]
            self.segm_leg_post[i] = segm_leg_post[i]
            self.front_vect[i] = front_vect[i]
            self.leg_vect[i] = leg_vect[i]

        self.segm_post_ant = segm_post_ant
        self.segm_diag_to_right[0] = segm_diag_to_right[0]

    ##	Extract the global positions of the feet, the segments, diagonals ...
    def get_leg_triangle(self, leg):
        return ([[self.foot_global[leg][0],
                  (self.foot_global[leg][0] - self.leg_vect[leg][0]),
                  (self.foot_global[leg][0] - self.leg_vect[leg][0] + self.segm_leg_post[leg][0]),
                  (self.foot_global[leg][0] - self.leg_vect[leg][0] + self.segm_leg_post[leg][0] +
                   # self.segm_post_ant[leg // 2][0]), (self.foot_global[leg][0] - self.leg_vect[leg][0])],
                   self.segm_post_ant[0]),
                  (self.foot_global[leg][0] - self.leg_vect[leg][0])],
                 [self.foot_global[leg][1],
                  (self.foot_global[leg][1] - self.leg_vect[leg][1]),
                  (self.foot_global[leg][1] - self.leg_vect[leg][1] + self.segm_leg_post[leg][1]),
                  (self.foot_global[leg][1] - self.leg_vect[leg][1] + self.segm_leg_post[leg][1] +
                   # self.segm_post_ant[leg // 2][1]), (self.foot_global[leg][1] - self.leg_vect[leg][1])],
                   self.segm_post_ant[1]),
                  (self.foot_global[leg][1] - self.leg_vect[leg][1])],
                 [self.foot_global[leg][2],
                  (self.foot_global[leg][2] - self.leg_vect[leg][2]),
                  (self.foot_global[leg][2] - self.leg_vect[leg][2] + self.segm_leg_post[leg][2]),
                  (self.foot_global[leg][2] - self.leg_vect[leg][2] + self.segm_leg_post[leg][2] +
                   # self.segm_post_ant[leg // 2][2]), (self.foot_global[leg][2] - self.leg_vect[leg][2])]])
                   self.segm_post_ant[2]),
                  (self.foot_global[leg][2] - self.leg_vect[leg][2])]])


""" **** Graphic methods: Simple drawing of the body model **************************
"""


def initialise_drawing_window(mmc):
    py.ion()
    fig = plt.figure(figsize=(12, 6))
    # py.rcParams['figure.figsize'] = 2, 2
    for i in range(0, 6):
        print("{}: mmc.get_leg_triangle({})[1] = {}".format(i, i, mmc.get_leg_triangle(i)[1]))
        py.plot(mmc.get_leg_triangle(i)[0], mmc.get_leg_triangle(i)[1], linestyle=':', linewidth=1.0, color='gray',
                marker='')
    leg_lines = [
        py.plot(mmc.get_leg_triangle(i)[0], mmc.get_leg_triangle(i)[1], linewidth=1.0, color='gray', marker='o',
                alpha=0.7, mfc='gray')[0] \
        for i in range(0, 6)]
    py.xlim(-0.5, 1.5)
    py.ylim(-0.5, 0.5)
    py.ioff()
    py.draw()
    return (leg_lines, 0, fig)


def initialise_drawing_window_3d(mmc):
    py.ion()
    fig = plt.figure(figsize=(12, 6))

    ax_fig_3d = Axes3D(fig)
    ax_fig_3d.view_init(30, -75)
    ax_fig_3d.plot([-0.5, 1.5], [-1, 1], [0, 1], linestyle='None')

    # py.rcParams['figure.figsize'] = 2, 2
    leg_lines = [0, 0, 0, 0, 0, 0]
    for i in range(0, 6):
        print("{}: mmc.get_leg_triangle({})[1] = {}".format(i, i, mmc.get_leg_triangle(i)[1]))
        ax_fig_3d.plot(mmc.get_leg_triangle(i)[0], mmc.get_leg_triangle(i)[1], mmc.get_leg_triangle(i)[2],
                linestyle=':', linewidth=1.0, color='gray', marker='')
        leg_lines[i], = ax_fig_3d.plot(mmc.get_leg_triangle(i)[0], mmc.get_leg_triangle(i)[1],
                mmc.get_leg_triangle(i)[2], '--', linewidth=1.0, color='gray', marker='o', alpha=0.7, mfc='black')

    py.xlim(-0.5, 1.5)
    py.ylim(-0.5, 0.5)
    py.ioff()
    py.draw()
    return (leg_lines, ax_fig_3d, fig)


def draw_manipulator(mmc):
    """ The draw method for the manipulator leg.
        It is called from the outside iteration loop.
    """
    # Update two dimensional top view
    # Update current legs
    for i in range(0, 6):
        leg_lines[i].set_marker('o')
        py.setp(leg_lines[i], linestyle='-', linewidth=2.0, color='green', marker='o', alpha=0.7, mfc='gray')
        leg_lines[i].set_xdata(mmc.get_leg_triangle(i)[0][0:5])
        leg_lines[i].set_ydata(mmc.get_leg_triangle(i)[1][0:5])
    py.draw()


# plt.savefig("/Users/mschilling/Desktop/Backward_RightA_"+str(step)+".pdf")

def draw_manipulator_3d(mmc):
    for i in range(0, 6):
        print("{}: mmc.get_leg_triangle({})[1] = {}".format(i, i, mmc.get_leg_triangle(i)[1]))
        leg_lines[i].set_marker('None')
        py.setp(leg_lines[i], linestyle='', linewidth=1.0, color='gray', marker='', alpha=0.7, mfc='gray')
        if (mmc.gc[i]):
            leg_lines[i], = ax_fig_3d.plot(mmc.get_leg_triangle(i)[0][0:5], mmc.get_leg_triangle(i)[1][0:5],
                    mmc.get_leg_triangle(i)[2][0:5], linewidth=2.0, color='black', marker='o', alpha=0.7, mfc='black')
        fig_3d.canvas.draw()


mmcBM = mmcBodyModelStance(0)
print("initialise_drawing_window")
(leg_lines, ax_fig_3d, fig_3d) = initialise_drawing_window(mmcBM)
print("before first fore")
for i in range(0, 35):  # 25
    mmcBM.mmc_iteration_step()
    draw_manipulator(mmcBM)
print("after for, waiting for input")
input()
delta_0 = numpy.array([0., 0., 0.])
mmcBM.pull_front = numpy.array([0., 0., 0.])
print("Q ADAPTED")
for i in range(0, 100):
    mmcBM.mmc_iteration_step()
    draw_manipulator(mmcBM)
print("after 2nd for, waiting for input")
input()

# delta_0 = numpy.array([0.025, 0.0075, 0.0])
# mmcBM.pull_front =numpy.array([0.025, 0.0075, 0.0])
# print("Q ADAPTED")
# for i in range(0, 25):
#     mmcBM.mmc_iteration_step()
#     draw_manipulator(mmcBM)
# print("after 2nd for, waiting for input")
# input()
# delta_0 = numpy.array([0., 0., 0.])
# mmcBM.pull_front = numpy.array([0., 0., 0.])
# print("Q ADAPTED")
# for i in range(0, 100):
#     mmcBM.mmc_iteration_step()
#     draw_manipulator(mmcBM)
# print("after 2nd for, waiting for input")
# input()
