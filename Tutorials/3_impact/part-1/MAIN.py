# This script is the main script for the IMPACT tutorial part 1.
from motion_planner import PathPlannerParking, PathPlannerLane, PathPlannerRoundAbout, PathPlannerRoundAboutSouth
from controller import IMPACT_Controller

from utils.simulator import Simulator
from utils.misc import pi, interpolate_truck_trailer_path, interpolate_truck_trailer_ctrl, measure_elapsed_time
import numpy as np

###########################################
# Set environment (lanes with roundabouts)
###########################################
east_lane_params = [0.0, 1.425, 0.5, 2.50] # lane_min_x, lane_max_x, lane_min_y, lane_max_y
west_lane_params = [1.425, 2.85, 0.50, 2.50] # lane_min_x, lane_max_x, lane_min_y, lane_max_y
north_roundabout_params = [1.425, 2.75, 0.25, 1.25] # x_center, y_center, inner_radius, external_radius,  round_y+(round_radius_min+round_radius_max)/2
south_roundabout_params = [1.425, 0.25, 0.25, 1.25] # x_center, y_center, inner_radius, external_radius
parking_params = [0.5, 4.5, 0.0, 1.0] # min_x, max_x, min_y, max_y
north_east_wall_params = [2.85, 1.5, 0.5, 1.5] # x_bottomleft, y_bottomleft, width, height
south_east_wall_params = [2.85, -0.5, 0.5, 1.0] # x_bottomleft, y_bottomleft, width, height


PARKING_PARAMS = [1.0, 4.0, 0.5, 1.5] # min_x, max_x, min_y, max_y
FINAL_PARKING_STATE = [3.5, 1.0, pi, pi]


# To test one action, just uncomment the corresponding line.
# This allows sequences of actions to be executed.
LIST_OF_ACTIONS = [
    {'type': 'lane', 'current_state': [1.76, -0.09, 1.32, 0.94], 'desired_state': [2.13, 1.75, pi/2, pi/2], 'lane_params': west_lane_params},
    {'type': 'parking', 'current_state': [2.2, 0.6, 1.543, 1.543], 'desired_state': FINAL_PARKING_STATE, 'parking_params':PARKING_PARAMS},
    {'type': 'roundabout', 'current_state': [2.1955, 0.85, 1.5957, 1.5957], 'desired_state': [0.75, 2.0, 3*pi/2, 3*pi/2], 'roundabout_params': north_roundabout_params},
    # {'type': 'roundabout_south', 'current_state': [0.6986148, 1.705157, -1.57043, -1.5735031], 'desired_state': [2.2, 0.6, pi/2, pi/2], 'roundabout_params': south_roundabout_params},
    # {'type': 'parking', 'current_state': [2.2, 0.6, 1.543, 1.543], 'desired_state': FINAL_PARKING_STATE, 'parking_params':PARKING_PARAMS},
]


##########################################
# Instantiate vehicle for simulation
##########################################
truck_trailer_simulator = Simulator()

# Add objects to simulation
truck_trailer_simulator.add_object(object_type='lane', params=east_lane_params)
truck_trailer_simulator.add_object(object_type='lane', params=west_lane_params)
truck_trailer_simulator.add_object(object_type='roundabout', params=north_roundabout_params)
truck_trailer_simulator.add_object(object_type='roundabout', params=south_roundabout_params)
truck_trailer_simulator.add_object(object_type='wall', params=north_east_wall_params)
truck_trailer_simulator.add_object(object_type='wall', params=south_east_wall_params)

##########################################
# Instantiate planners and controller(s)
##########################################
lane_planner = PathPlannerLane()
roundabout_planner = PathPlannerRoundAbout()
parking_planner = PathPlannerParking()

roundabout_south_planner = PathPlannerRoundAboutSouth()

controller = IMPACT_Controller(model_file = 'truck_trailer.yaml')


##################################################
# Plan and execute motion based on list of actions
##################################################

for action in LIST_OF_ACTIONS:
    
    if action['type'] == 'lane':
        _, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_res, time_grid_ctrl_res = lane_planner.plan_path(action['current_state'], action['desired_state'], action['lane_params'])
    elif action['type'] == 'parking':
        _, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_res, time_grid_ctrl_res = parking_planner.plan_path(action['current_state'], action['desired_state'], action['parking_params'])
    elif action['type'] == 'roundabout':
        _, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_res, time_grid_ctrl_res = roundabout_planner.plan_path(action['current_state'], action['desired_state'], action['roundabout_params'])
    elif action['type'] == 'roundabout_south':
        _, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_res, time_grid_ctrl_res = roundabout_south_planner.plan_path(action['current_state'], action['desired_state'], action['roundabout_params'])
    # Plot planned path
    truck_trailer_simulator.plot_path(x1_res, y1_res, x0_res, y0_res)

    interpolated_TT_path = interpolate_truck_trailer_path(x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, time_grid_res, dt=controller.DT)
    interpolated_TT_ctrl = interpolate_truck_trailer_ctrl(delta0_res, V0_res, time_grid_ctrl_res, dt=controller.DT)

    current_state = action['current_state']
    
    print('driving to destination ...')
    number_of_points = interpolated_TT_path.shape[1]

    state_logger = np.empty((4,0))
    control_logger = np.empty((2,0))

    i = 0
    while i < number_of_points:
        # for i, point in enumerate(interpolated_TT_path.T):

        if i <= number_of_points - controller.HORIZON:
            considered_points = interpolated_TT_path[:, i:i+controller.HORIZON]
            considered_controls = interpolated_TT_ctrl[:, i:i+controller.HORIZON]
        else:
            points_left = number_of_points - i - 1
            considered_points = np.hstack((interpolated_TT_path[:, i:i+points_left], np.tile(interpolated_TT_path[:,-1],(controller.HORIZON-points_left,1)).T))
            considered_controls = np.hstack((interpolated_TT_ctrl[:, i:i+points_left], np.tile(interpolated_TT_ctrl[:,-1],(controller.HORIZON-points_left,1)).T))

        with measure_elapsed_time('Time required for controller'):
            x_initial_guess = np.hstack((np.array(current_state).reshape((4,1)), considered_points[[0,1,4,5],:]))
            u_initial_guess = considered_controls
            delta0_opt, V0_opt = controller.optimize(current_state, considered_points, considered_controls, x_initial_guess, u_initial_guess) 

        current_state = truck_trailer_simulator.simulate_vehicle(x0=current_state, u=[delta0_opt, V0_opt], dt=controller.DT*1.5)

        i += 1

        # Logging
        state_logger = np.append(state_logger, current_state, axis=1)
        control_logger = np.append(control_logger, np.array([delta0_opt, V0_opt]).reshape((2,1)), axis=1)
        

    #########################################
    # Plot results
    #########################################
    truck_trailer_simulator.plot_trajectory(state_logger, control_logger)
