import numpy
from numpy.core.umath_tests import inner1d
from numpy.linalg import svd

"""Took code from https://stackoverflow.com/questions/12299540/plane-fitting-to-4-or-more-xyz-points"""


def plane_fit(points):
    """
    p, n = planeFit(points)

    Given an array, points, of shape (d,...)
    representing points in d-dimensional space,
    fit an d-dimensional plane to the points.
    Return a point, p, on the plane (the point-cloud centroid),
    and the normal, n.
    """
    points = numpy.reshape(points, (numpy.shape(points)[0], -1))  # Collapse trialing dimensions
    points = points.T
    if points.shape[0] > points.shape[1]:
        #print("There are only {} points in {} dimensions.".format(points.shape[1], points.shape[0]))
        return None, None
    ctr = points.mean(axis=1)
    x = points - ctr[:, numpy.newaxis]
    M = numpy.dot(x, x.T)  # Could also use np.cov(x) here.
    return ctr, svd(M)[0][:, -1]

def project_com_onto_ground_plane(points, com):
    plane_centroid, normal = plane_fit(list(points))
    if plane_centroid is None or normal is None:
        #print("plane fit failed")
        return None

    v = com - plane_centroid
    dist = numpy.dot(v, normal)

    return com - (dist * normal)


"""Took code from https://github.com/malteschilling/cognitiveWalker/blob/master/stability/stability.py"""


def _is_ccw_turn(o, a, b):
    return ((a.item(0) - o.item(0)) * (b.item(1) - o.item(1)) - (a.item(1) - o.item(1)) * (b.item(0) - o.item(0))) > 0


def convex_hull(points):
    """Returns points on 2D convex hull of an array of points in CCW order. The function uses Andrew's monotone chain
    convex hull algorithm. Only the first two entries of every point list are considered."""
    points = numpy.array(points)
    points = points[points[:, 0].argsort()]  # points are being sorted by the first value in the point arrays.
    lower = numpy.ones(numpy.shape(points)) * numpy.nan  # lower hull
    upper = lower.copy()  # upper
    len_lower = len_upper = 0
    # Build lower hull
    for p in points:
        for len_lower in range(len_lower, 0, -1):
            if _is_ccw_turn(lower[len_lower - 2, :], lower[len_lower - 1, :], p):
                break
        lower[len_lower, :] = p
        len_lower += 1
    # Build upper hull
    for p in points[::-1]:
        for len_upper in range(len_upper, 0, -1):
            if _is_ccw_turn(upper[len_upper - 2, :], upper[len_upper - 1, :], p):
                break
        upper[len_upper, :] = p
        len_upper += 1
    # Concatenation of the lower and upper hulls gives the convex hull.
    # Last point of the first list is omitted because it is repeated at the beginning of the other list.
    return numpy.vstack([lower[:len_lower - 1, :], upper[:len_upper, :]])


def is_point_inside_convex_hull(convex_hull_points, test_point):
    """The function tests whether a given point is inside a convex hull. The hull's points must be given in
    CCW order to get the correct results! If the points are given in CW order, also points that lie on the
    boundary are classified as 'inside'. """
    if numpy.shape(convex_hull_points)[0] <= 3:
        # if the lists consists only of the starting point, another point and once more the starting point,
        # you get a line as convex hull. Therefore, the testPoint won't fit in.
        return False
    rel_ch_points = numpy.subtract(convex_hull_points, test_point)
    a = rel_ch_points[1:, 0] * rel_ch_points[:-1, 1] - rel_ch_points[0:-1, 0] * rel_ch_points[1:, 1]
    return all(a < 0)


def shortest_vectors_to_convex_hull(convex_hull_points, test_point):
    """The function computes the shortest vectors that connect the test point with the lines between the
    points specified in cHPs"""
    rel_ch_points = numpy.subtract(convex_hull_points, test_point)
    p1_p2 = numpy.diff(convex_hull_points, axis=0)
    lambda1 = (inner1d(rel_ch_points[:-1, :], p1_p2)) / numpy.sum(p1_p2 ** 2, axis=1)
    return rel_ch_points[:-1, :] - lambda1[:, numpy.newaxis] * p1_p2


def z_axis_to_neg_g_vector_rot(g_vec):
    """ This function returns a rotation matrix with which points may be transformed into a coordinate system
    whose z-vector is parallel to the gravitation vector."""
    g_vec /= -numpy.linalg.norm(g_vec)
    if g_vec.item(0) == g_vec.item(1) == 0:
        return numpy.eye(3)
    cos_rot_angle = g_vec.item(2)
    sin_rot_angle = (1 - cos_rot_angle ** 2) ** 0.5
    rot_axis = numpy.array([g_vec[1], -g_vec[0], 0])
    rot_axis /= numpy.linalg.norm(rot_axis)
    return numpy.linalg.inv(
            numpy.eye(3) * cos_rot_angle + numpy.cross(rot_axis, numpy.eye(3)) * sin_rot_angle +
            numpy.outer(rot_axis, rot_axis) * (1 - cos_rot_angle))


def g_vector_from_ground_contacts(ground_contacts):
    """ In lack of an acceleration sensor, this function is able to compute the g-vector dependent on the positions
    of the ground contacts. The result, however, is only valid for a plane ground (less plane -> less valid)! """
    a = numpy.column_stack((ground_contacts[:, 0:2], numpy.ones(ground_contacts.shape[0])))
    temp, *_ = (numpy.linalg.lstsq(a, ground_contacts[:, 2]))
    temp[2] = -1
    temp /= -numpy.linalg.norm(temp)
    if temp.item(2) > 0:
        temp *= -1
    return temp


def direction_to_move(ground_contacts, center_of_mass, threshold):
    """ If the center of mass of the robot comes too close to the boundary of the convex hull that is spanned by the
    foot points, a vector is returned that points in the direction where a better stability would could be reached."""
    if numpy.shape(ground_contacts)[0] < 3:
        return None
    g_vec = g_vector_from_ground_contacts(ground_contacts)
    r = z_axis_to_neg_g_vector_rot(g_vec)
    com_grav = numpy.dot(r, center_of_mass)
    ground_contacts_grav = numpy.dot(r, ground_contacts.T).T
    convex_hull_grav = convex_hull(ground_contacts_grav)
    if not is_point_inside_convex_hull(convex_hull_grav, com_grav):
        return None
    shortest_vecs = shortest_vectors_to_convex_hull(convex_hull_grav, com_grav)
    dists = (numpy.sum(shortest_vecs ** 2, axis=1)) ** 0.5
    norm_energies = (dists + shortest_vecs[:, 2])
    temp_ind, *_ = numpy.nonzero(norm_energies < threshold)
    if numpy.shape(temp_ind)[0] == 0:
        return numpy.array([0, 0, 0])
    else:
        direction_to_move_vec = numpy.dot(-shortest_vecs[temp_ind, 0:2].T, (threshold - norm_energies[temp_ind]).T)
        return numpy.append(direction_to_move_vec, 0)


def nesm(convex_hull_points, center_of_mass):
    """ This gives the minimum normalized energy that is needed to push the center of mass over the lines that built
    up the support polygon. The formula for the required normalized energy h_i that is needed to push the center of
    mass over line i is given in Garcia2002:
    h_i=|R_i|*(1-cos(theta))*cos(psi)
    with R_i being the vector between the center of mass and the line, theta being the angle between R_i and the
    vertical axis and psi being the angle between the line and the horizontal plane."""
    hs = []
    for point1, point2 in zip(convex_hull_points[:-1, :], convex_hull_points[1:, :]):
        dp = point2 - point1
        ndpq = numpy.linalg.norm(dp) ** 2
        temp1 = numpy.dot((center_of_mass - point1), dp) / ndpq
        temp2 = numpy.linalg.norm(point1 - center_of_mass + dp * temp1)
        h = numpy.sqrt(1 - dp[2] ** 2 / ndpq) * temp2 * (1 + (point1[2] - center_of_mass[2] + dp[2] * temp1) / temp2)
        hs.append(h)
    return min(hs)
