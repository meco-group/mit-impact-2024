import numpy as np
import scipy.interpolate as scipy_interpolate
from casadi import pi
import contextlib, time


@contextlib.contextmanager
def measure_elapsed_time(identifier):
    start_time = time.time_ns()
    yield
    elapsed = (time.time_ns() - start_time)/1000
    print(f"{identifier}: {elapsed} us")


def interpolate_b_spline_path(x, y, n_path_points, degree=3):
    ipl_t = np.linspace(0.0, len(x) - 1, len(x))
    spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree)
    spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree)
    travel = np.linspace(0.0, len(x) - 1, n_path_points)
    return spl_i_x(travel), spl_i_y(travel)

def interpolate_path(path, sample_rate):
    choices = np.arange(0,len(path),sample_rate)
    if len(path)-1 not in choices:
            choices =  np.append(choices , len(path)-1)
    way_point_x = path[choices,0]
    way_point_y = path[choices,1]
    n_course_point = len(path)*3
    rix, riy = interpolate_b_spline_path(way_point_x, way_point_y, n_course_point)
    new_path = np.vstack([rix,riy]).T
    # new_path[new_path<0] = 0
    return new_path

def interpolate_truck_trailer_path(x1, y1, x0, y0, theta0, theta1, time_grid, dt):
        
    Tf = time_grid[-1]
    dt_timegrid = Tf/time_grid.shape[0]

    new_number_of_samples = int(Tf/dt)
    interpolated_time_grid = np.linspace(time_grid[0], time_grid[-1], new_number_of_samples)

    interpolated_path = np.empty((6,0))
    k = 1 # iterate in original path

    for i in range(new_number_of_samples): # iterate in interpolated path

        # perform linear interpolation
        current_time_diff = (interpolated_time_grid[i]-time_grid[k-1])
        current_sample_time_diff = time_grid[k]-time_grid[k-1]

        current_state = np.array([
            [x1[k-1] + current_time_diff*((x1[k] - x1[k-1])/current_sample_time_diff)], 
            [y1[k-1] + current_time_diff*((y1[k] - y1[k-1])/current_sample_time_diff)],
            [x0[k-1] + current_time_diff*((x0[k] - x0[k-1])/current_sample_time_diff)], 
            [y0[k-1] + current_time_diff*((y0[k] - y0[k-1])/current_sample_time_diff)],
            [theta0[k-1] + current_time_diff*((theta0[k] - theta0[k-1])/current_sample_time_diff)],
            [theta1[k-1] + current_time_diff*((theta1[k] - theta1[k-1])/current_sample_time_diff)],
        ])

        # current_state = np.array([
        #     [x1[k]], 
        #     [y1[k]],
        #     [x0[k]], 
        #     [y0[k]],
        #     [theta0[k]],
        #     [theta1[k]],
        # ])

        interpolated_path = np.append(interpolated_path, current_state, axis=1)

        if interpolated_time_grid[i] >= time_grid[k]:
            k += 1

    return interpolated_path


def resample_path(x1, y1, x0, y0, time_grid, scale):

    dt_timegrid = time_grid[-1]/time_grid.shape[0]

    return interpolate_truck_trailer_path(x1, y1, x0, y0, time_grid, dt_timegrid/scale)

def interpolate_truck_trailer_ctrl(delta0, v0, time_grid, dt):
        
    Tf = time_grid[-1]
    dt_timegrid = Tf/time_grid.shape[0]

    new_number_of_samples = int(Tf/dt)
    interpolated_time_grid = np.linspace(time_grid[0], time_grid[-1], new_number_of_samples)

    interpolated_ctrl = np.empty((2,0))
    k = 1 # iterate in original path

    for i in range(new_number_of_samples): # iterate in interpolated path

        # perform linear interpolation
        current_time_diff = (interpolated_time_grid[i]-time_grid[k-1])
        current_sample_time_diff = time_grid[k]-time_grid[k-1]

        current_ctrl = np.array([
            [delta0[k-1] + current_time_diff*((delta0[k] - delta0[k-1])/current_sample_time_diff)], 
            [v0[k-1] + current_time_diff*((v0[k] - v0[k-1])/current_sample_time_diff)]
        ])

        # current_ctrl = np.array([
        #     [delta0[k-1]], 
        #     [v0[k-1]],
        # ])

        interpolated_ctrl = np.append(interpolated_ctrl, current_ctrl, axis=1)

        if interpolated_time_grid[i] >= time_grid[k]:
            k += 1

    return interpolated_ctrl