import casadi as cs
from casadi import pi, cos, sin, tan
import numpy as np
from impact import MPC, MultipleShooting, external_method


class IMPACT_Controller:
    def __init__(self, model_file):
        self.model_file = model_file

        # Define weights for the cost function
        self.R = np.diag([1e-1, 1e-1])                 # input cost matrix
        self.Rf = 10*np.diag([1e-4, 1e-4])                 # terminal cost
        self.Q = np.diag([100.0, 100.0, 50.0, 50.0, 1e-1, 100.0]) # state cost matrix # x1,y1,x0,y0,theta0,theta1
        self.Qf = 30*np.diag([30.0, 30.0, 15.0, 15.0, 1e-1, 30.0]) # state final matrix

        self.integrator = 'rk' # 'expl_euler', 'rk'
        self.HORIZON = 20 # 12, 20
        self.TIME_HORIZON = 1.0 # 0.6, 1.
        self.DT = self.TIME_HORIZON/self.HORIZON

        self.V_max = 0.2
        self.V_min = -0.2
        self.Delta0_max = pi/3
        self.Beta01_max = pi/2
        
        self._set_mpc()            
        self._set_MPC_function()  
        self._export_artifacts()         

    def _set_mpc(self):

        # Set a fixed time horizon MPC
        mpc = MPC(T=self.TIME_HORIZON)

        # Define the model
        self.model = mpc.add_model('truck_trailer',self.model_file)

        # Instantiate parameters
        x_current = mpc.parameter('x_current', self.model.nx)
        path = mpc.parameter('path', 6, grid='control')
        control_path = mpc.parameter('control_path', 2, grid='control')

        # Set initial values
        mpc.set_value(x_current, [0]*self.model.nx)
        mpc.set_value(path, cs.DM.zeros(6,self.HORIZON))
        mpc.set_value(control_path, cs.DM.zeros(2,self.HORIZON))

        # Compute additional expressions for the position of the truck
        x0 = self.model.x1 + self.model.L1*cos(self.model.theta1) + self.model.M0*cos(self.model.theta0)
        y0 = self.model.y1 + self.model.L1*sin(self.model.theta1) + self.model.M0*sin(self.model.theta0)

        # Compute additional expressions for the angle between the truck and the trailer
        beta01 = self.model.theta0 - self.model.theta1

        # Compute additional expressions for the difference between the current and the next state
        delta_delta0 = mpc.next(self.model.delta0) - self.model.delta0
        delta_V0 = mpc.next(self.model.V0) - self.model.V0


        # Set initial constraints
        mpc.subject_to(mpc.at_t0(self.model.x)==x_current)

        # Set path constraints
        mpc.subject_to(self.V_min <= (self.model.V0 <= self.V_max)) 
        mpc.subject_to(-self.Delta0_max <= (self.model.delta0 <= self.Delta0_max))
        mpc.subject_to(-self.Beta01_max <= (beta01 <= self.Beta01_max))

        # Set path-following objective
        mpc.add_objective(mpc.sum((path - cs.vertcat(self.model.x1,self.model.y1,x0,y0,self.model.theta0,self.model.theta1)).T @ (self.Q @ (path - cs.vertcat(self.model.x1,self.model.y1,x0,y0,self.model.theta0,self.model.theta1)))))
        # Set terminal objective
        mpc.add_objective(mpc.at_tf((path[:,-1] - cs.vertcat(self.model.x1,self.model.y1,x0,y0,self.model.theta0,self.model.theta1)).T @ (self.Qf @ (path[:,-1] - cs.vertcat(self.model.x1,self.model.y1,x0,y0,self.model.theta0,self.model.theta1)))))

        # Add regularization term for the inputs
        mpc.add_objective(1e-1*mpc.sum(cs.sumsqr(self.model.u)))
        
        # Penalize the rate of change of the inputs
        mpc.add_objective(5*mpc.sum(cs.sumsqr(delta_delta0)))
        mpc.add_objective(5*mpc.sum(cs.sumsqr(delta_V0)))



        # Choose a discretization method and solver
        mpc.method(MultipleShooting(N=self.HORIZON, M=1, intg=self.integrator))
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
        self.path = path
        self.control_path = control_path

        self.x0, self.y0 = x0, y0

    def _set_MPC_function(self):
        mpc = self.mpc

        V0, delta0 = self.model.V0, self.model.delta0

        # Sample the control variables
        V0_samp = mpc.sample(V0, grid='control-')[1]
        delta0_samp = mpc.sample(delta0, grid='control-')[1]

        # Sample the parameters
        x_current_samp = mpc.value(self.x_current)
        path_samp = mpc.sample(self.path, grid='control-')[1]
        control_path_samp = mpc.sample(self.control_path, grid='control-')[1]

        # Sample the inputs and states
        u_initial_guess = mpc.sample(self.model.u, grid='control-')[1]
        x_initial_guess = mpc.sample(self.model.x, grid='control')[1]

        # Define the input vector
        input_vector = [x_current_samp, path_samp, control_path_samp, x_initial_guess, u_initial_guess]
        input_names = ['vehicle_state', 'waypoints', 'control_waypoints', 'x_initial_guess', 'u_initial_guess']

        # Sample the states
        x0_res = mpc.sample(self.x0, grid='integrator')[1]
        y0_res = mpc.sample(self.y0, grid='integrator')[1]
        x1_res = mpc.sample(self.model.x1, grid='integrator')[1]
        y1_res = mpc.sample(self.model.y1, grid='integrator')[1]

        # Define the output vector
        output_vector = [V0_samp[0], delta0_samp[0], x0_res, y0_res, x1_res, y1_res]
        output_names = ['velocity', 'delta', 'x0', 'y0', 'x1', 'y1']

        # Define the MPC function
        self.MPC_function = mpc.to_function('mpc_fun', input_vector, output_vector, input_names, output_names)

        # Serialize the MPC function
        # self.MPC_function.save('path_follower_controller_orin.casadi')

    def _export_artifacts(self):
        # Export the MPC artifacts
        self.mpc.export('truck_trailer_mpc', mode='debug', ros2=True, compile=True)


    def optimize(self, current_state, path, control_path, x_initial_guess, u_initial_guess):

        # Solve the optimal control problem

        # self.mpc.set_value(self.x_current, current_state)
        # self.mpc.set_value(self.path, path)
        # self.mpc.set_value(self.control_path, control_path)

        # sol = self.mpc.solve()      

        # V0_res = sol.sample(self.model.V0, grid='control')[1]
        # delta0_res = sol.sample(self.model.delta0, grid='control')[1]

        # return delta0_res[0], V0_res[0]

        V0_res, delta0_res, _, _ ,_ ,_ = self.MPC_function(current_state, path, control_path, x_initial_guess, u_initial_guess)

        print(f"V0_res: {V0_res}, delta0_res: {delta0_res}")

        return float(delta0_res), float(V0_res)

    