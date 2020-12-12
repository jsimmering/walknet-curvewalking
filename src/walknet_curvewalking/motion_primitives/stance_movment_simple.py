import numpy



if __name__ == '__main__':
    temp = StanceMovementSimple()
    temp.swing_start_point = numpy.array([0., 0., 0.])  # the point where the stance phase starts
    temp.swing_target_point = numpy.array([1., 0., 0.])  # the point where the stance phase should end
    bezier_points = temp.compute_bezier_points()
    print(bezier_points)