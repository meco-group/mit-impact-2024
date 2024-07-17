import numpy as np
from numpy import cos, sin

import casadi as c
from multipledispatch import dispatch


@dispatch(float, float, float, float, float, float, float)
def get_vehicle_vertices(x, y, theta, w_left, w_right, l_front, l_back):
    '''Get all vertices of a vehicle, using following convention::


        (4)       y     (3)   ^
        .|--------|------|    |  w_left
        .|        o---x  |    x
        .|               |    |
        .|---------------|    |  w_right
        (1)             (2)   v
        . <-------><---->
        .  l_back   l_front


    :param x, y: center point of vehicle
    :type x, y: float

    :param theta: angle wrt x-axis
    :type theta: float

    :param w_left: width of vehicle, to the left of center
    :type w_left: float

    :param w_right: width of vehicle, to the right of center
    :type w_right: float

    :param l_front: length of vehicle, front to center
    :type l_front: float

    :param l_back: length of vehicle, back to center
    :type l_back: float

    :return: points - matrix with all vertices
    :rtype: np.ndarray
    '''
    # Vertices of vehicle in homogeneous coordinates
    vertices = np.array([[-l_back, l_front, l_front, -l_back],
                         [-w_right, -w_right, w_left, w_left],
                         [0, 0, 0, 0],
                         [1, 1, 1, 1]])

    # Homogeneous transformation matrix
    homog_transf_matrix = np.array([[c.cos(theta), -c.sin(theta), 0, x],
                                    [c.sin(theta),  c.cos(theta), 0, y],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])

    # Transform vertices and extract 2D points
    points = homog_transf_matrix.dot(vertices)[:2].transpose()

    return points


@dispatch(c.MX, c.MX, c.MX, float, float, float, float)
def get_vehicle_vertices(x, y, theta, w_left, w_right, l_front, l_back):
    '''Get all vertices of a vehicle, using following convention::


        (4)       y     (3)   ^
        .|--------|------|    |  w_left
        .|        o---x  |    x
        .|               |    |
        .|---------------|    |  w_right
        (1)             (2)   v
        . <-------><---->
        .  l_back   l_front


    :param x, y: center point of vehicle
    :type x, y: c.MX

    :param theta: angle wrt x-axis
    :type theta: c.MX

    :param w_left: width of vehicle, to the left of center
    :type w_left: float

    :param w_right: width of vehicle, to the right of center
    :type w_right: float

    :param l_front: length of vehicle, front to center
    :type l_front: float

    :param l_back: length of vehicle, back to center
    :type l_back: float

    :return: points - matrix with all vertices
    :rtype: np.ndarray
    '''
    # Vertices of vehicle in 3D homogeneous coordinates
    vertices = c.vertcat(
        c.horzcat(-l_back, l_front, l_front, -l_back),
        c.horzcat(-w_right, -w_right, w_left, w_left),
        c.horzcat(0, 0, 0, 0),
        c.horzcat(1, 1, 1, 1))

    # Homogeneous transformation matrix
    homog_transf_matrix = c.vertcat(
        c.horzcat(c.cos(theta), -c.sin(theta), 0, x),
        c.horzcat(c.sin(theta),  c.cos(theta), 0, y),
        c.horzcat(0, 0, 1, 0),
        c.horzcat(0, 0, 0, 1))

    # import pdb; pdb.set_trace()
    # Transform vertices and extract 2D points
    points = (homog_transf_matrix @ vertices)[0:2, :]

    return points



def vert_vehic(x, y, theta, w_left, w_right, l_front, l_back):

	#   (4)       y     (3)   ^
	#    |--------|------|    |  w_left
	#    |        o---x  |    x
	#    |               |    |
	#    |---------------|    |  w_right
	#   (1)             (2)   v
	#     <-------><---->
	#      l_back   l_front

	# Vertices of vehicle in homogeneous coordinates
	vertices = np.array([[ -l_back,  l_front, l_front, -l_back],
						 [-w_right, -w_right,  w_left,  w_left],
						 [       0,        0,       0,       0],
						 [       1,        1,       1,       1]])

	# Homogeneous transformation matrix
	homog_transf_matrix = np.array([[cos(theta), -sin(theta),      0,      x],
									[sin(theta),  cos(theta),      0,      y],
									[         0,           0,      1,      0],
									[         0,           0,      0,      1]])

	# Transformed vertices
	points = homog_transf_matrix.dot(vertices)[:2].transpose()
	return points

def vert_single(x, y, theta, dx, dy):

	#   (4)       y     (3)   ^
	#    |--------|------|    |  w_left
	#    |        o---x  |    x
	#    |               |    |
	#    |---------------|    |  w_right
	#   (1)             (2)   v
	#     <-------><---->
	#      l_back   l_front

	# Vertices of vehicle in homogeneous coordinates
	vertices = np.array([[dx],
						 [dy],
						 [ 0],
						 [ 1]])

	# Homogeneous transformation matrix
	homog_transf_matrix = np.array([[cos(theta), -sin(theta),      0,      x],
									[sin(theta),  cos(theta),      0,      y],
									[         0,           0,      1,      0],
									[         0,           0,      0,      1]])

	# Transformed vertices
	points = homog_transf_matrix.dot(vertices)[:2].transpose()
	return points

def wheel_to_plot(ax, x, y, theta, dx, dy, delta=0, color='k'):
	steering_wheel_pos = vert_single(x, y, theta, dx, dy)
	steering_wheel = vert_vehic(steering_wheel_pos[0][0], steering_wheel_pos[0][1], theta+delta, .01, .01, .1, .1)
	steering_wheel_ext = np.append(steering_wheel, [[steering_wheel[0][0], steering_wheel[0][1]]], axis=0)
	wheel = ax.plot([x[0] for x in steering_wheel_ext], [y[1] for y in steering_wheel_ext], color=color)
	return wheel

def vehic_to_plot(ax, x, y, theta, w_left, w_right, l_front, l_back, color='b'):
	vertices_veh = vert_vehic(x, y, theta, w_left, w_right, l_front, l_back)
	vertices_veh_ext = np.append(vertices_veh, [[vertices_veh[0][0], vertices_veh[0][1]]], axis=0)
	veh = ax.plot([x[0] for x in vertices_veh_ext], [y[1] for y in vertices_veh_ext], color=color)
	return veh

def line_points(a=0, b=0, c=0, ref=[-1., 1.]):
    """given a, b, c for straight line as ax+by+c=0,
    return a pair of points based on ref values
    e.g linePoints(-1, 1, 2) == [(-1.0, -3.0), (1.0, -1.0)]
    """
    if (a == 0 and b == 0):
        raise Exception("linePoints: a and b cannot both be zero")
    return [(-c/a, p) if b == 0 else (p, (-c-a*p)/b) for p in ref]

def draw_constraint(w, ax, color):
    ax.axline(*line_points(a=w[0], b=w[1], c=w[2]),
              color=color, alpha=0.4, linestyle='dashed')