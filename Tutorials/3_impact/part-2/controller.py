import casadi as cs
from casadi import pi, cos, sin, tan
import numpy as np
from impact import MPC, MultipleShooting, external_method


class IMPACT_Controller:
    def __init__(self, model_file):
        self._solver = 'ipopt'

        self.model_file = model_file


        self.integrator = 'rk' # 'expl_euler', 'rk'
        self.HORIZON = 20 
        self.TIME_HORIZON = 1.0 
        self.DT = self.TIME_HORIZON/self.HORIZON
       
        self._set_mpc()            
        self._set_MPC_function()         

    def _set_mpc(self):

        mpc = MPC(T=self.TIME_HORIZON)

        self.model = ...

        # Set parameters
        ...
        ...

        # Initialize parameters
        ...
        ...

        # Initial constraints
        ...

        # Path constraints
        self.model.q = cs.vertcat(
            self.model.q1,
            self.model.q2,
            self.model.q3,
            self.model.q4,
            self.model.q5,
            self.model.q6,
            self.model.q7
        )
        self.model.dq = cs.vertcat(
            self.model.dq1,
            self.model.dq2,
            self.model.dq3,
            self.model.dq4,
            self.model.dq5,
            self.model.dq6,
            self.model.dq7
        )

        # Define path constraints (upper and lower bounds on joint position, velocity and acceleration)
        ...
        ...
        ...

        # Terminal constraints
        ...

        # Objective function
        fk = self.model.fk

        weight_path = 1.0
        weight_terminal = 50.0

        # Add objective for P2P motion
        mpc.add_objective(...)

        # Regularize joint position, joint velocity and joint acceleration in the objective function
        mpc.add_objective(1e-5*mpc.sum(cs.sumsqr(self.model.q)))
        mpc.add_objective(1e-3*mpc.sum(cs.sumsqr(self.model.dq)))
        mpc.add_objective(1e-3*mpc.sum(cs.sumsqr(self.model.u)))

        # Add terminal objective for P2P motion
        mpc.add_objective(...)
        

        # Choose a discretization method and solver
        if self._solver == 'ipopt':
            mpc.method(MultipleShooting(N=self.HORIZON, M=1, intg=self.integrator))
            options = {
                "expand": True,
                "verbose": False,
                "print_time": True,
                "error_on_fail": True,
                "ipopt"
                : {
                    "linear_solver": "ma27",
                    "print_level": 1,
                    # "print_timing_statistics": "yes",
                    "tol": 1e-3,
                    "sb": "yes"
                }
            }
            mpc.solver('ipopt', options)
        elif self._solver == 'fatropy':
            mpc.method(external_method('fatrop',N=self.HORIZON,mode='fatropy',fatrop_options={"tol":1e-3}))
        elif self._solver == 'fatrop':
            mpc.method(external_method('fatrop',N=self.HORIZON,mode='interface',fatrop_options={"tol":1e-3}))

        
        self.mpc = mpc

        # Set the parameters as variables of the controller class
        self.... = ...
        self.... = ...


    def _set_MPC_function(self):
        mpc = self.mpc

        dq, u = self.model.dq, self.model.u

        # Get value of parameters
        ...
        ...

        # Get sample of initial guesses
        ...
        ...

        input_vector = [...]
        input_names = [...]

        # Sample desired outputs
        ...
        ...
        ...

        output_vector = [...]
        output_names = [...]

        self.MPC_function = mpc.to_function('mpc_fun', input_vector, output_vector, input_names, output_names)


    def optimize(self, current_state, desired_position, x_initial_guess, u_initial_guess):

        # Evaluate MPC function

        ... = self.MPC_function(...)

        return ...

    