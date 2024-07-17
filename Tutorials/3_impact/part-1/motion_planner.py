import casadi as cs
from impact import MPC, MultipleShooting, external_method, FreeTime
from casadi import cos, sin, pi


class PathPlannerLane:
    def __init__(self):

        self.N_hor = 50
        self.M = 1

        self.V_max = 0.2
        self.V_min = -0.1
        self.Delta0_max = 50*pi/180
        self.Beta01_max = 50*pi/180

        self._set_path_planner()
        self._set_planner_function()

    def _set_path_planner(self):

        mpc = MPC(T=FreeTime(12.0))

        # Define the model
        self.model = mpc.add_model('truck_trailer','truck_trailer.yaml')

        # Set parameters
        x_current = mpc.parameter('x_current', self.model.nx)
        x_final = mpc.parameter('x_final', self.model.nx)

        lane_params = mpc.parameter('lane_params', 4)
        lane_min_x = lane_params[0]
        lane_max_x = lane_params[1]

        # Compute expression for the position of the truck
        x0 = self.model.x1 + self.model.L1*cos(self.model.theta1) + self.model.M0*cos(self.model.theta0)
        y0 = self.model.y1 + self.model.L1*sin(self.model.theta1) + self.model.M0*sin(self.model.theta0)

        # Compute expression for the angle between the truck and the trailer
        beta01 = self.model.theta0 - self.model.theta1

        # Compute expression for the difference between the current and the next input
        delta_u = mpc.next(self.model.u) - self.model.u

        # Initial constraints
        mpc.subject_to(mpc.at_t0(self.model.x)==x_current)

        # Road constraints
        mpc.subject_to(lane_min_x <= (self.model.x1 <= lane_max_x))

        # Path constraints
        mpc.subject_to(self.V_min <= (self.model.V0 <= self.V_max)) 
        mpc.subject_to(-self.Delta0_max <= (self.model.delta0 <= self.Delta0_max))
        mpc.subject_to(-self.Beta01_max <= (beta01 <= self.Beta01_max))

        # Terminal constraint
        mpc.subject_to(mpc.at_tf(cs.sumsqr(cs.vertcat(self.model.x1,self.model.y1) - x_final[:2])) <= (5e-2)**2)
        mpc.subject_to(mpc.at_tf(cs.sumsqr(cs.vertcat(self.model.theta0,self.model.theta1) - x_final[2:4])) <= (pi/4)**2) #errror in a ball of 30 degrees

        mpc.subject_to(mpc.at_tf(self.model.V0)==0)
        mpc.subject_to(mpc.at_tf(beta01) == 0)
        
        # Set initial guess
        mpc.set_initial(self.model.x1,0)
        mpc.set_initial(self.model.y1,0)
        mpc.set_initial(self.model.theta1, pi/2)
        mpc.set_initial(self.model.V0,0.2)

        # Set initial values of parameters
        mpc.set_value(x_current, [0]*self.model.nx)
        mpc.set_value(x_final, [0]*self.model.nx)
        mpc.set_value(lane_params, [0]*4)

        # Minimal time
        mpc.add_objective(mpc.at_tf(mpc.T))

        # Add regularization terms
        mpc.add_objective(mpc.sum(1e-1*beta01**2))
        mpc.add_objective(mpc.sum((1e-2)*cs.sumsqr(self.model.u)))
        mpc.add_objective(1e-1*mpc.sum(cs.sumsqr(delta_u)))


        # Choose a discretization method and solver
        mpc.method(MultipleShooting(N=self.N_hor, M=self.M, intg='rk'))
        options = {
            "expand": True,
            "verbose": False,
            "print_time": False,
            "error_on_fail": True,
            "ipopt": {
                # "linear_solver": "ma27",
                "print_level": 0,
                # "print_timing_statistics": "yes",
                "tol": 1e-3,
                "sb": "yes"
            }
        }
        mpc.solver('ipopt', options)
       
        self.mpc = mpc

        self.x_current = x_current
        self.x_final = x_final
        self.lane_params = lane_params
        self.x0, self.y0 = x0, y0

    def _set_planner_function(self):
        mpc = self.mpc

        V0, delta0 = self.model.V0, self.model.delta0

        x_current_samp = mpc.value(self.x_current)
        x_final_samp = mpc.value(self.x_final)
        lane_params_samp = mpc.value(self.lane_params)
    
        input_vector = [x_current_samp, x_final_samp, lane_params_samp]
        input_names = ['vehicle_state', 'final_state', 'lane_params']

        time_grid_intg, X_res = mpc.sample(self.model.x, grid='integrator')
        x0_res = mpc.sample(self.x0, grid='integrator')[1]
        y0_res = mpc.sample(self.y0, grid='integrator')[1]
        x1_res = mpc.sample(self.model.x1, grid='integrator')[1]
        y1_res = mpc.sample(self.model.y1, grid='integrator')[1]
        theta0_res = mpc.sample(self.model.theta0, grid='integrator')[1]
        theta1_res = mpc.sample(self.model.theta1, grid='integrator')[1]

        time_grid_ctrl, delta0_res = mpc.sample(self.model.delta0, grid='control')
        V0_res = mpc.sample(self.model.V0, grid='control')[1]

        output_vector = [
            X_res,
            x1_res, y1_res, 
            x0_res, y0_res, 
            theta0_res, theta1_res, 
            delta0_res, V0_res, 
            time_grid_intg, time_grid_ctrl
        ]
        output_names = [
            'state_vector', 
            'x1_res', 'y1_res', 
            'x0_res', 'y0_res', 
            'theta0_res', 'theta1_res', 
            'delta0_res', 'V0_res', 
            'time_grid_intg', 'time_grid_ctrl'
        ]

        self.path_planner_function = mpc.to_function('mpc_fun', input_vector, output_vector, input_names, output_names)
        # self.path_planner_function.save('lane_planner_orin.casadi')

    def plan_path(self, current_state, desired_state, lane_parameters):
        
        # self.mpc.set_value(self.x_current, current_state)
        # self.mpc.set_value(self.x_final, desired_state)
        # self.mpc.set_value(self.lane_params, lane_parameters)
        # sol = self.mpc.solve()
        # time_grid_intg, X_res = sol.sample(self.model.x, grid='integrator')
        # x0_res = sol.sample(self.x0, grid='integrator')[1]
        # y0_res = sol.sample(self.y0, grid='integrator')[1]
        # x1_res = sol.sample(self.model.x1, grid='integrator')[1]
        # y1_res = sol.sample(self.model.y1, grid='integrator')[1]
        # theta0_res = sol.sample(self.model.theta0, grid='integrator')[1]
        # theta1_res = sol.sample(self.model.theta1, grid='integrator')[1]

        # time_grid_ctrl, delta0_res = sol.sample(self.model.delta0, grid='control')
        # V0_res = sol.sample(self.model.V0, grid='control')[1]
        
        # # path_res = self.path_planner_function(current_state, desired_state)

        # return X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl

        X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl = self.path_planner_function(current_state, desired_state, lane_parameters)

        # return X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl
        return X_res, x1_res.full().flatten(), y1_res.full().flatten(), x0_res.full().flatten(), y0_res.full().flatten(), theta0_res.full().flatten(), theta1_res.full().flatten(), delta0_res.full().flatten(), V0_res.full().flatten(), time_grid_intg.full().flatten(), time_grid_ctrl.full().flatten()
    

class PathPlannerParking:
    def __init__(self):

        self._solver = 'ipopt'  # choose between 'ipopt', 'fatropy' and 'fatrop'
        self.N_hor = 50
        self.M = 1

        self.SAFETY_MARGIN_ROUNDABOUT = 0.05

        self.V_max = 0.2
        self.V_min = -0.2
        self.Delta0_max = 60*pi/180 # pi/6
        self.Beta01_max = 70*pi/180

        self._set_path_planner()
        self._set_planner_function()


    def _set_path_planner(self):

        mpc = MPC(T=FreeTime(5.0))
        # mpc = MPC(T=10.0)

        self.model = mpc.add_model('truck_trailer','truck_trailer.yaml')

        x_current = mpc.parameter('x_current', self.model.nx)
        x_final = mpc.parameter('x_final', self.model.nx)

        parking_params = mpc.parameter('parking_params', 4)
        parking_min_x = parking_params[0]
        parking_max_x = parking_params[1]
        parking_min_y = parking_params[2]
        parking_max_y = parking_params[3]

        # round_params = mpc.parameter('round_params', 4)
        # round_x = round_params[0]
        # round_y = round_params[1]
        # round_radius_min = round_params[2]
        # round_radius_max = round_params[3]

        x0 = self.model.x1 + self.model.L1*cos(self.model.theta1) + self.model.M0*cos(self.model.theta0)
        y0 = self.model.y1 + self.model.L1*sin(self.model.theta1) + self.model.M0*sin(self.model.theta0)

        beta01 = self.model.theta0 - self.model.theta1

        delta_u = mpc.next(self.model.u) - self.model.u

        # Initial constraints
        mpc.subject_to(mpc.at_t0(self.model.x)==x_current)

        # Road constraints
        # mpc.subject_to(-0.5 <= (self.model.x1 <= 3.5))
        # mpc.subject_to(-0.5 <= (x0 <= 3.0))
        # mpc.subject_to(-0.8 <= (self.model.y1 <= 0.1))
        # mpc.subject_to(parking_min_x <= (self.model.x1 <= parking_max_x))
        # mpc.subject_to(parking_min_x <= (x0 <= parking_max_x))
        # mpc.subject_to(parking_min_y <= (self.model.y1 <= parking_max_y))
        # mpc.subject_to(parking_min_y <= (y0 <= parking_max_y))

        # mpc.subject_to(x0 <= lane_max_x)
        # mpc.subject_to(y0 <= round_y)
        # mpc.subject_to(self.model.y1 <= round_y)
        # mpc.subject_to(-0.5 <= (self.model.x1 <= 3.0))
        # mpc.subject_to(-0.5 <= (x0 <= 3.0))
        # mpc.subject_to(-2.0 <= (self.model.y1 <= 0.5))
        # mpc.subject_to(-2.0 <= (y0 <= 0.5))
        # mpc.subject_to(-0.5 <= (self.model.x1 <= 3.5))
        # mpc.subject_to(-0.5 <= (x0 <= 3.0))
        # mpc.subject_to(-0.8 <= (self.model.y1 <= 0.1))
        # mpc.subject_to(-1.0 <= (y0 <= 0.0))
        # mpc.subject_to((round_radius_min + self.SAFETY_MARGIN_ROUNDABOUT)**2 <= (self.model.x1-round_x)**2+(self.model.y1-round_y)**2 )
        # mpc.subject_to((round_radius_min + self.SAFETY_MARGIN_ROUNDABOUT)**2 <= (x0-round_x)**2+(y0-round_y)**2 )

        # Final constraint
        mpc.subject_to(mpc.at_tf(self.model.x)==x_final)

        # mpc.subject_to(mpc.at_tf(cs.sumsqr(cs.vertcat(self.model.x1,self.model.y1) - x_final[:2])) <= (5e-3)**2)
        # mpc.subject_to(mpc.at_tf(cs.sumsqr(cs.vertcat(self.model.theta0,self.model.theta1) - x_final[2:4])) <= (pi/12)**2) #errror in a ball of 30 degrees

        # mpc.subject_to(mpc.at_tf(self.model.x1)==x_final[0])
        # mpc.subject_to(mpc.at_tf(self.model.y1)==x_final[1])
        # mpc.subject_to(mpc.at_tf(cs.sin(self.model.theta0))==cs.sin(x_final[2]))
        # mpc.subject_to(mpc.at_tf(cs.cos(self.model.theta0))==cs.cos(x_final[2]))
        # mpc.subject_to(mpc.at_tf(cs.sin(self.model.theta1))==cs.sin(x_final[3]))
        # mpc.subject_to(mpc.at_tf(cs.cos(self.model.theta1))==cs.cos(x_final[3]))

        # mpc.subject_to(mpc.at_tf(cs.sumsqr(cs.vertcat(self.model.x1,self.model.y1) - x_final[:2])) <= (2e-2)**2)
        # mpc.subject_to(mpc.at_tf(cs.sumsqr(cs.vertcat(self.model.theta0,self.model.theta1) - x_final[2:4])) <= (2e-2)**2)


        mpc.set_initial(self.model.x1,0.4)
        mpc.set_initial(self.model.y1,-0.2)
        mpc.set_initial(self.model.theta1, pi/2+pi/6)
        mpc.set_initial(self.model.theta0, pi/2+pi/4)
        mpc.set_initial(self.model.V0,-0.2)
        mpc.set_initial(self.model.delta0,0.0)

        mpc.set_value(x_current, [0]*self.model.nx)
        mpc.set_value(x_final, [0]*self.model.nx)
        mpc.set_value(parking_params, [0]*4)

        mpc.subject_to(self.V_min <= (self.model.V0 <= self.V_max)) 
        mpc.subject_to(-self.Delta0_max <= (self.model.delta0 <= self.Delta0_max))
        mpc.subject_to(-self.Beta01_max <= (beta01 <= self.Beta01_max))
        # mpc.subject_to(mpc.at_tf(self.model.V0)==0)

        # mpc.subject_to(cs.sumsqr(mpc.at_tf(beta01)) <= (5*pi/180)**2)


        # # #######################################################
        # # # parking constraints (hyperplanes)
        # # PA = [-1.0,3.5]
        # # PB = [-1.0,1.9]
        # # PC = [-5.0,1.9]
        # PA = [1.0,1.0]
        # PB = [1.0,0.0]
        # PC = [2.0,0.0]

        # P1 = np.array([PA,
        #     PB,
        #     PC])

        # # Parking forbidden points, for Parking2 PD,PE, and PF
        # # PD = [-5.0,0.9]
        # # PE = [-1.0,0.9]
        # # PF = [-1.0,-3.5]
        # PD = [1.0,-3.0]
        # PE = [1.0,-1.0]
        # PF = [2.0,-1.0]

        # P2 = np.array([PD,
        #     PE,
        #     PF])
        
        
        # ax = mpc.control()
        # ay = mpc.control()
        # b = mpc.control()
        # safe_dist = 0.05
        # mpc.subject_to(ax*self.model.x1+ay*self.model.y1<=b + safe_dist, include_last=False)

        # for i in range(P1.shape[0]):
        #     mpc.subject_to(ax*P1[i][0]+ay*P1[i][1]>=b + safe_dist)
        # for i in range(P2.shape[0]):
        #     mpc.subject_to(ax*P2[i][0]+ay*P2[i][1]>=b + safe_dist)

        # mpc.subject_to(ax**2 + ay**2 == 1)

        # # #######################################################



        # Minimal time
        mpc.add_objective(mpc.at_tf(mpc.T))
        mpc.add_objective(mpc.sum(1e-2*beta01**2))
        mpc.add_objective(mpc.sum((1e-2)*cs.sumsqr(self.model.u)))
        mpc.add_objective(1e-1*mpc.sum(cs.sumsqr(delta_u)))
    

        # Choose a discretization method and solver
        if self._solver == 'ipopt':
            mpc.method(MultipleShooting(N=self.N_hor, intg='rk')) 
            options = {
                "expand": True,
                "verbose": False,
                "print_time": False,
                "error_on_fail": True,
                "ipopt": {
                    # "linear_solver": "ma27",
                    "print_level": 0,
                    "tol": 1e-3,
                    "sb": "yes"
                }
            }
            mpc.solver('ipopt',options)
        elif self._solver == 'fatropy':
            mpc.method(external_method('fatrop',N=self.N_hor,mode='fatropy',fatrop_options={"tol":1e-3}))
        elif self._solver == 'fatrop':
            mpc.method(external_method('fatrop',N=self.N_hor,mode='interface',fatrop_options={"tol":1e-3}))

        self.mpc = mpc

        self.x_current = x_current
        self.x_final = x_final
        self.parking_params = parking_params
        self.x0, self.y0 = x0, y0

    def _set_planner_function(self):
        mpc = self.mpc

        V0, delta0 = self.model.V0, self.model.delta0

        x_current_samp = mpc.value(self.x_current)
        x_final_samp = mpc.value(self.x_final)
        parking_params_samp = mpc.value(self.parking_params)
    
        input_vector = [x_current_samp, x_final_samp, parking_params_samp]
        input_names = ['vehicle_state', 'final_state', 'parking_params']

        # a_samp = mpc.sample(self.a, grid='control-')[1]
        # delta_samp = mpc.sample(self.delta, grid='control-')[1]

        time_grid_intg, X_res = mpc.sample(self.model.x, grid='integrator')
        x0_res = mpc.sample(self.x0, grid='integrator')[1]
        y0_res = mpc.sample(self.y0, grid='integrator')[1]
        x1_res = mpc.sample(self.model.x1, grid='integrator')[1]
        y1_res = mpc.sample(self.model.y1, grid='integrator')[1]
        theta0_res = mpc.sample(self.model.theta0, grid='integrator')[1]
        theta1_res = mpc.sample(self.model.theta1, grid='integrator')[1]

        time_grid_ctrl, delta0_res = mpc.sample(self.model.delta0, grid='control')
        V0_res = mpc.sample(self.model.V0, grid='control')[1]

        output_vector = [
            X_res,
            x1_res, y1_res, 
            x0_res, y0_res, 
            theta0_res, theta1_res, 
            delta0_res, V0_res, 
            time_grid_intg, time_grid_ctrl
        ]
        output_names = [
            'state_vector', 
            'x1_res', 'y1_res', 
            'x0_res', 'y0_res', 
            'theta0_res', 'theta1_res', 
            'delta0_res', 'V0_res', 
            'time_grid_intg', 'time_grid_ctrl'
        ]

        self.path_planner_function = mpc.to_function('mpc_fun', input_vector, output_vector, input_names, output_names)
        # self.path_planner_function.save('parking_planner_orin.casadi')

    def plan_path(self, current_state, desired_state, parking_parameters):
        
        # self.mpc.set_value(self.x_current, current_state)
        # self.mpc.set_value(self.x_final, desired_state)
        # self.mpc.set_value(self.lane_params, lane_parameters)
        # sol = self.mpc.solve()
        # time_grid_intg, X_res = sol.sample(self.model.x, grid='integrator')
        # x0_res = sol.sample(self.x0, grid='integrator')[1]
        # y0_res = sol.sample(self.y0, grid='integrator')[1]
        # x1_res = sol.sample(self.model.x1, grid='integrator')[1]
        # y1_res = sol.sample(self.model.y1, grid='integrator')[1]
        # theta0_res = sol.sample(self.model.theta0, grid='integrator')[1]
        # theta1_res = sol.sample(self.model.theta1, grid='integrator')[1]

        # time_grid_ctrl, delta0_res = sol.sample(self.model.delta0, grid='control')
        # V0_res = sol.sample(self.model.V0, grid='control')[1]
        
        # return X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl

        X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl = self.path_planner_function(current_state, desired_state, parking_parameters)

        # return X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl
        return X_res, x1_res.full().flatten(), y1_res.full().flatten(), x0_res.full().flatten(), y0_res.full().flatten(), theta0_res.full().flatten(), theta1_res.full().flatten(), delta0_res.full().flatten(), V0_res.full().flatten(), time_grid_intg.full().flatten(), time_grid_ctrl.full().flatten()


class PathPlannerRoundAbout:
    def __init__(self):

        self._solver = 'ipopt'  # choose between 'ipopt', 'fatropy' and 'fatrop'
        self.N_hor = 30
        self.M = 1

        self.SAFETY_MARGIN_WALL = 0.1
        self.SAFETY_MARGIN_ROUNDABOUT = 0.1

        self.V_max = 0.2
        self.V_min = 0
        self.Delta0_max = 60*pi/180
        self.Beta01_max = pi/2
        # self.Delta0_max = 50*pi/180
        # self.Beta01_max = 50*pi/180

        self._set_path_planner()
        self._set_planner_function()

    def _set_path_planner(self):

        mpc = MPC(T=FreeTime(10.0))

        self.model = mpc.add_model('truck_trailer','truck_trailer.yaml')

        x_current = mpc.parameter('x_current', self.model.nx)
        x_final = mpc.parameter('x_final', self.model.nx)

        round_params = mpc.parameter('round_params', 4)
        round_x = round_params[0]
        round_y = round_params[1]
        round_radius_min = round_params[2]
        round_radius_max = round_params[3]

        x0 = self.model.x1 + self.model.L1*cos(self.model.theta1) + self.model.M0*cos(self.model.theta0)
        y0 = self.model.y1 + self.model.L1*sin(self.model.theta1) + self.model.M0*sin(self.model.theta0)

        beta01 = self.model.theta0 - self.model.theta1

        delta_u = mpc.next(self.model.u) - self.model.u

        # Initial constraints
        mpc.subject_to(mpc.at_t0(self.model.x)==x_current)

        # Road constraints
        mpc.subject_to((round_radius_min + self.SAFETY_MARGIN_ROUNDABOUT)**2 <= (self.model.x1-round_x)**2+(self.model.y1-round_y)**2 )
        # mpc.subject_to((round_radius_min )**2 <= (self.model.x0-round_x)**2+(self.model.y0-round_y)**2 )
        # mpc.subject_to((self.model.x1-round_x)**2+(self.model.y1-round_y)**2 <= (round_radius_max + self.SAFETY_MARGIN_ROUNDABOUT)**2 )
        # mpc.subject_to( 1 <= ((self.model.x1-0)**2)/0.1**2+((self.model.y1-0)**2)/2**2 )
        # mpc.subject_to(round_radius_min**2<= (x0-round_x)**2+(y0-round_y)**2 )
        # mpc.subject_to(self.model.y1 <= round_y + round_radius_max - self.model.W0/2)
        # mpc.subject_to(y0 <= round_y + round_radius_max - self.model.W0/2)
        
        mpc.subject_to( mpc.at_t0((mpc.offset(self.model.x1,round(self.N_hor/2))))==round_x)
        mpc.subject_to( mpc.at_t0((mpc.offset(self.model.y1,round(self.N_hor/2))))== round_y+(round_radius_min+round_radius_max)/2)
        mpc.subject_to( mpc.at_t0((mpc.offset(self.model.theta1,round(self.N_hor/2))))== (x_current[2]+x_final[2])/2)
        
        mpc.subject_to(0 <= (y0 <= 4.7))
        mpc.subject_to(0 <= (x0 <= 2.85))
        # mpc.subject_to(-1 + self.SAFETY_MARGIN_WALL <= (x0 <= 2))
        # mpc.subject_to(lane_min_y <= (y0 <= lane_max_y))
        # mpc.subject_to(-1 + self.SAFETY_MARGIN_WALL <= (self.model.x1 <= 2 ))



        # Final constraint
        # mpc.subject_to(mpc.at_tf(self.model.x)==x_final)
        mpc.subject_to(mpc.at_tf(cs.sumsqr(cs.vertcat(self.model.x1,self.model.y1) - x_final[:2])) <= (5e-2)**2)
        mpc.subject_to(mpc.at_tf(cs.sumsqr(cs.vertcat(self.model.theta0,self.model.theta1) - x_final[2:4])) <= (pi/6)**2) #errror in a ball of 30 degrees

        mpc.set_initial(self.model.x1,2.2)
        mpc.set_initial(self.model.y1,1)
        mpc.set_initial(self.model.theta1, pi/2)
        mpc.set_initial(self.model.theta0, pi/2)
        mpc.set_initial(self.model.V0,0.2)
        mpc.set_initial(self.model.delta0,0.0)

        mpc.set_value(x_current, [0]*self.model.nx)
        mpc.set_value(x_final, [0]*self.model.nx)
        mpc.set_value(round_params, [0]*4)

        mpc.subject_to(self.V_min <= (self.model.V0 <= self.V_max)) 
        mpc.subject_to(-self.Delta0_max <= (self.model.delta0 <= self.Delta0_max))
        mpc.subject_to(-self.Beta01_max <= (beta01 <= self.Beta01_max))

        # Minimal time
        mpc.add_objective(mpc.at_tf(mpc.T))
        mpc.add_objective(mpc.sum(1e-1*beta01**2))
        mpc.add_objective(mpc.sum((1e-2)*cs.sumsqr(self.model.u)))
        mpc.add_objective(1e-1*mpc.sum(cs.sumsqr(delta_u)))


        # Choose a discretization method and solver
        if self._solver == 'ipopt':
            mpc.method(MultipleShooting(N=self.N_hor, M=self.M, intg='rk')) 
            options = {
                "expand": True,
                "verbose": False,
                "print_time": False,
                "error_on_fail": True,
                "ipopt": {
                    # "linear_solver": "ma27",
                    "print_level": 0,
                    "tol": 1e-3,
                    "sb": "yes"
                }
            }
            mpc.solver('ipopt', options)
        elif self._solver == 'fatropy':
            mpc.method(external_method('fatrop',N=self.N_hor,mode='fatropy',fatrop_options={"tol":1e-3}))
        elif self._solver == 'fatrop':
            mpc.method(external_method('fatrop',N=self.N_hor,mode='interface',fatrop_options={"tol":1e-3}))

        self.mpc = mpc

        self.x_current = x_current
        self.x_final = x_final
        self.round_params = round_params
        self.x0, self.y0 = x0, y0

    def _set_planner_function(self):
        mpc = self.mpc

        V0, delta0 = self.model.V0, self.model.delta0

        x_current_samp = mpc.value(self.x_current)
        x_final_samp = mpc.value(self.x_final)
        round_params_samp = mpc.value(self.round_params)
    
        input_vector = [x_current_samp, x_final_samp, round_params_samp]
        input_names = ['vehicle_state', 'final_state', 'roundabout_params']

        # a_samp = mpc.sample(self.a, grid='control-')[1]
        # delta_samp = mpc.sample(self.delta, grid='control-')[1]

        time_grid_intg, X_res = mpc.sample(self.model.x, grid='integrator')
        x0_res = mpc.sample(self.x0, grid='integrator')[1]
        y0_res = mpc.sample(self.y0, grid='integrator')[1]
        x1_res = mpc.sample(self.model.x1, grid='integrator')[1]
        y1_res = mpc.sample(self.model.y1, grid='integrator')[1]
        theta0_res = mpc.sample(self.model.theta0, grid='integrator')[1]
        theta1_res = mpc.sample(self.model.theta1, grid='integrator')[1]

        time_grid_ctrl, delta0_res = mpc.sample(self.model.delta0, grid='control')
        V0_res = mpc.sample(self.model.V0, grid='control')[1]

        output_vector = [
            X_res,
            x1_res, y1_res, 
            x0_res, y0_res, 
            theta0_res, theta1_res, 
            delta0_res, V0_res, 
            time_grid_intg, time_grid_ctrl
        ]
        output_names = [
            'state_vector', 
            'x1_res', 'y1_res', 
            'x0_res', 'y0_res', 
            'theta0_res', 'theta1_res', 
            'delta0_res', 'V0_res', 
            'time_grid_intg', 'time_grid_ctrl'
        ]

        self.path_planner_function = mpc.to_function('mpc_fun', input_vector, output_vector, input_names, output_names)
        # self.path_planner_function.save('roundabout_planner_orin.casadi')


    def plan_path(self, current_state, desired_state, roundabout_params):
        
        # self.mpc.set_value(self.x_current, current_state)
        # self.mpc.set_value(self.x_final, desired_state)
        # self.mpc.set_value(self.round_params, roundabout_params)
        # sol = self.mpc.solve()
        # time_grid_intg, X_res = sol.sample(self.model.x, grid='integrator')
        # x0_res = sol.sample(self.x0, grid='integrator')[1]
        # y0_res = sol.sample(self.y0, grid='integrator')[1]
        # x1_res = sol.sample(self.model.x1, grid='integrator')[1]
        # y1_res = sol.sample(self.model.y1, grid='integrator')[1]
        # theta0_res = sol.sample(self.model.theta0, grid='integrator')[1]
        # theta1_res = sol.sample(self.model.theta1, grid='integrator')[1]

        # time_grid_ctrl, delta0_res = sol.sample(self.model.delta0, grid='control')
        # V0_res = sol.sample(self.model.V0, grid='control')[1]
        
        # # path_res = self.path_planner_function(current_state, desired_state)

        # return X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl

        X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl = self.path_planner_function(current_state, desired_state, roundabout_params)

        # return X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl
        return X_res, x1_res.full().flatten(), y1_res.full().flatten(), x0_res.full().flatten(), y0_res.full().flatten(), theta0_res.full().flatten(), theta1_res.full().flatten(), delta0_res.full().flatten(), V0_res.full().flatten(), time_grid_intg.full().flatten(), time_grid_ctrl.full().flatten()
    

class PathPlannerRoundAboutSouth:
    def __init__(self):

        self._solver = 'ipopt'  # choose between 'ipopt', 'fatropy' and 'fatrop'
        self.N_hor = 30
        self.M = 1

        self.SAFETY_MARGIN_WALL = 0.1
        self.SAFETY_MARGIN_ROUNDABOUT = 0.1

        self.V_max = 0.2
        self.V_min = 0
        self.Delta0_max = 60*pi/180
        self.Beta01_max = pi/2
        # self.Delta0_max = 50*pi/180
        # self.Beta01_max = 50*pi/180

        self._set_path_planner()
        self._set_planner_function()

    def _set_path_planner(self):

        mpc = MPC(T=FreeTime(10.0))

        self.model = mpc.add_model('truck_trailer','truck_trailer.yaml')

        x_current = mpc.parameter('x_current', self.model.nx)
        x_final = mpc.parameter('x_final', self.model.nx)

        round_params = mpc.parameter('round_params', 4)
        round_x = round_params[0]
        round_y = round_params[1]
        round_radius_min = round_params[2]
        round_radius_max = round_params[3]

        x0 = self.model.x1 + self.model.L1*cos(self.model.theta1) + self.model.M0*cos(self.model.theta0)
        y0 = self.model.y1 + self.model.L1*sin(self.model.theta1) + self.model.M0*sin(self.model.theta0)

        beta01 = self.model.theta0 - self.model.theta1

        delta_u = mpc.next(self.model.u) - self.model.u

        # Initial constraints
        mpc.subject_to(mpc.at_t0(self.model.x)==x_current)

        # Road constraints
        mpc.subject_to((round_radius_min + self.SAFETY_MARGIN_ROUNDABOUT)**2 <= (self.model.x1-round_x)**2+(self.model.y1-round_y)**2 )
        # mpc.subject_to((round_radius_min )**2 <= (self.model.x0-round_x)**2+(self.model.y0-round_y)**2 )
        # mpc.subject_to((self.model.x1-round_x)**2+(self.model.y1-round_y)**2 <= (round_radius_max + self.SAFETY_MARGIN_ROUNDABOUT)**2 )
        # mpc.subject_to( 1 <= ((self.model.x1-0)**2)/0.1**2+((self.model.y1-0)**2)/2**2 )
        # mpc.subject_to(round_radius_min**2<= (x0-round_x)**2+(y0-round_y)**2 )
        # mpc.subject_to(self.model.y1 <= round_y + round_radius_max - self.model.W0/2)
        # mpc.subject_to(y0 <= round_y + round_radius_max - self.model.W0/2)
        
        mpc.subject_to( mpc.at_t0((mpc.offset(self.model.x1,round(self.N_hor/2))))==round_x)
        mpc.subject_to( mpc.at_t0((mpc.offset(self.model.y1,round(self.N_hor/2))))== round_y-(round_radius_min+round_radius_max)/2)
        mpc.subject_to( mpc.at_t0((mpc.offset(self.model.theta1,round(self.N_hor/2))))== (x_current[2]+x_final[2])/2)
        
        mpc.subject_to(-0.8 <= (y0 <= 4))
        mpc.subject_to(-0.5 <= (x0 <= 2.85))
        # mpc.subject_to(-1 + self.SAFETY_MARGIN_WALL <= (x0 <= 2))
        # mpc.subject_to(lane_min_y <= (y0 <= lane_max_y))
        # mpc.subject_to(-1 + self.SAFETY_MARGIN_WALL <= (self.model.x1 <= 2 ))



        # Final constraint
        # mpc.subject_to(mpc.at_tf(self.model.x)==x_final)
        mpc.subject_to(mpc.at_tf(cs.sumsqr(cs.vertcat(self.model.x1,self.model.y1) - x_final[:2])) <= (5e-2)**2)
        mpc.subject_to(mpc.at_tf(cs.sumsqr(cs.vertcat(self.model.theta0,self.model.theta1) - x_final[2:4])) <= (pi/6)**2) #errror in a ball of 30 degrees

        mpc.set_initial(self.model.x1,2.2)
        mpc.set_initial(self.model.y1,1)
        mpc.set_initial(self.model.theta1, pi/2)
        mpc.set_initial(self.model.theta0, pi/2)
        mpc.set_initial(self.model.V0,0.2)
        mpc.set_initial(self.model.delta0,0.0)

        mpc.set_value(x_current, [0]*self.model.nx)
        mpc.set_value(x_final, [0]*self.model.nx)
        mpc.set_value(round_params, [0]*4)

        mpc.subject_to(self.V_min <= (self.model.V0 <= self.V_max)) 
        mpc.subject_to(-self.Delta0_max <= (self.model.delta0 <= self.Delta0_max))
        mpc.subject_to(-self.Beta01_max <= (beta01 <= self.Beta01_max))

        # Minimal time
        mpc.add_objective(mpc.at_tf(mpc.T))
        mpc.add_objective(mpc.sum(1e-1*beta01**2))
        mpc.add_objective(mpc.sum((1e-2)*cs.sumsqr(self.model.u)))
        mpc.add_objective(1e-1*mpc.sum(cs.sumsqr(delta_u)))


        # Choose a discretization method and solver
        if self._solver == 'ipopt':
            mpc.method(MultipleShooting(N=self.N_hor, M=self.M, intg='rk')) 
            options = {
                "expand": True,
                "verbose": False,
                "print_time": False,
                "error_on_fail": True,
                "ipopt": {
                    # "linear_solver": "ma27",
                    "print_level": 0,
                    "tol": 1e-3,
                    "sb": "yes"
                }
            }
            mpc.solver('ipopt', options)
        elif self._solver == 'fatropy':
            mpc.method(external_method('fatrop',N=self.N_hor,mode='fatropy',fatrop_options={"tol":1e-3}))
        elif self._solver == 'fatrop':
            mpc.method(external_method('fatrop',N=self.N_hor,mode='interface',fatrop_options={"tol":1e-3}))

        self.mpc = mpc

        self.x_current = x_current
        self.x_final = x_final
        self.round_params = round_params
        self.x0, self.y0 = x0, y0

    def _set_planner_function(self):
        mpc = self.mpc

        V0, delta0 = self.model.V0, self.model.delta0

        x_current_samp = mpc.value(self.x_current)
        x_final_samp = mpc.value(self.x_final)
        round_params_samp = mpc.value(self.round_params)
    
        input_vector = [x_current_samp, x_final_samp, round_params_samp]
        input_names = ['vehicle_state', 'final_state', 'roundabout_params']

        # a_samp = mpc.sample(self.a, grid='control-')[1]
        # delta_samp = mpc.sample(self.delta, grid='control-')[1]

        time_grid_intg, X_res = mpc.sample(self.model.x, grid='integrator')
        x0_res = mpc.sample(self.x0, grid='integrator')[1]
        y0_res = mpc.sample(self.y0, grid='integrator')[1]
        x1_res = mpc.sample(self.model.x1, grid='integrator')[1]
        y1_res = mpc.sample(self.model.y1, grid='integrator')[1]
        theta0_res = mpc.sample(self.model.theta0, grid='integrator')[1]
        theta1_res = mpc.sample(self.model.theta1, grid='integrator')[1]

        time_grid_ctrl, delta0_res = mpc.sample(self.model.delta0, grid='control')
        V0_res = mpc.sample(self.model.V0, grid='control')[1]

        output_vector = [
            X_res,
            x1_res, y1_res, 
            x0_res, y0_res, 
            theta0_res, theta1_res, 
            delta0_res, V0_res, 
            time_grid_intg, time_grid_ctrl
        ]
        output_names = [
            'state_vector', 
            'x1_res', 'y1_res', 
            'x0_res', 'y0_res', 
            'theta0_res', 'theta1_res', 
            'delta0_res', 'V0_res', 
            'time_grid_intg', 'time_grid_ctrl'
        ]

        self.path_planner_function = mpc.to_function('mpc_fun', input_vector, output_vector, input_names, output_names)
        # self.path_planner_function.save('roundabout_planner_orin_south.casadi')


    def plan_path(self, current_state, desired_state, roundabout_params):
        
        # self.mpc.set_value(self.x_current, current_state)
        # self.mpc.set_value(self.x_final, desired_state)
        # self.mpc.set_value(self.round_params, roundabout_params)
        # sol = self.mpc.solve()
        # time_grid_intg, X_res = sol.sample(self.model.x, grid='integrator')
        # x0_res = sol.sample(self.x0, grid='integrator')[1]
        # y0_res = sol.sample(self.y0, grid='integrator')[1]
        # x1_res = sol.sample(self.model.x1, grid='integrator')[1]
        # y1_res = sol.sample(self.model.y1, grid='integrator')[1]
        # theta0_res = sol.sample(self.model.theta0, grid='integrator')[1]
        # theta1_res = sol.sample(self.model.theta1, grid='integrator')[1]

        # time_grid_ctrl, delta0_res = sol.sample(self.model.delta0, grid='control')
        # V0_res = sol.sample(self.model.V0, grid='control')[1]
        
        # # path_res = self.path_planner_function(current_state, desired_state)

        # return X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl

        X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl = self.path_planner_function(current_state, desired_state, roundabout_params)

        # return X_res, x1_res, y1_res, x0_res, y0_res, theta0_res, theta1_res, delta0_res, V0_res, time_grid_intg, time_grid_ctrl
        return X_res, x1_res.full().flatten(), y1_res.full().flatten(), x0_res.full().flatten(), y0_res.full().flatten(), theta0_res.full().flatten(), theta1_res.full().flatten(), delta0_res.full().flatten(), V0_res.full().flatten(), time_grid_intg.full().flatten(), time_grid_ctrl.full().flatten()