import casadi as cs
from casadi import pi, cos, sin, tan
import numpy as np
from impact import MPC, MultipleShooting, external_method


class IMPACT_Controller:
    def __init__(self, model_file):
        self._solver = 'ipopt' # 'ipopt', 'fatrop', 'fatropy'

        self.model_file = model_file


        self.integrator = 'rk' # 'expl_euler', 'rk'
        self.HORIZON = 20 # 12, 20
        self.TIME_HORIZON = 1.0 # 0.6, 1.
        self.DT = self.TIME_HORIZON/self.HORIZON
       
        self._set_mpc()            
        self._set_MPC_function()  
        self._export_artifacts()         

    def _set_mpc(self):

        mpc = MPC(T=self.TIME_HORIZON)

        self.model = mpc.add_model('franka_panda', self.model_file)

        x_current = mpc.parameter('x_current', self.model.nx)
        p_final = mpc.parameter('p_final', 3, 1)

        mpc.set_value(x_current, [0]*self.model.nx)
        mpc.set_value(p_final, cs.DM.zeros(3, 1))

        # Initial constraints
        mpc.subject_to(mpc.at_t0(self.model.x)==x_current)

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

        mpc.subject_to(self.model.joint_pos_lb <= (self.model.q <= self.model.joint_pos_ub))
        mpc.subject_to(self.model.joint_vel_lb <= (self.model.dq <= self.model.joint_vel_ub))
        mpc.subject_to([-100]*self.model.nu <= (self.model.u <= [100]*self.model.nu))

        # Terminal constraints
        mpc.subject_to(mpc.at_tf(self.model.dq) == 0)

        # Objective function
        fk = self.model.fk

        weight_path = 1.0
        weight_terminal = 50.0

        mpc.add_objective(weight_path*mpc.sum(cs.sumsqr(fk(self.model.q)[-1][0:3, 3] - p_final)))
        mpc.add_objective(1e-5*mpc.sum(cs.sumsqr(self.model.q)))
        mpc.add_objective(1e-3*mpc.sum(cs.sumsqr(self.model.dq)))
        mpc.add_objective(1e-3*mpc.sum(cs.sumsqr(self.model.u)))

        mpc.add_objective(weight_terminal*mpc.at_tf(cs.sumsqr(fk(self.model.q)[-1][0:3, 3] - p_final)))
        

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

        self.x_current = x_current
        self.p_final = p_final


    def _set_MPC_function(self):
        mpc = self.mpc

        dq, u = self.model.dq, self.model.u

        dq_samp = mpc.sample(dq, grid='control-')[1]

        x_current_samp = mpc.value(self.x_current)
        p_final_samp = mpc.value(self.p_final)

        u_initial_guess = mpc.sample(self.model.u, grid='control-')[1]
        x_initial_guess = mpc.sample(self.model.x, grid='control')[1]

        input_vector = [x_current_samp, p_final_samp, x_initial_guess, u_initial_guess]
        input_names = ['vehicle_state', 'p_final', 'x_initial_guess', 'u_initial_guess']

        time_grid_intg, X_res = mpc.sample(self.model.x, grid='control')
        time_grid_intg, U_res = mpc.sample(self.model.u, grid='control-')

        output_vector = [dq_samp[:,1], X_res, U_res]
        output_names = ['velocity', 'X_res', 'U_res']

        self.MPC_function = mpc.to_function('mpc_fun', input_vector, output_vector, input_names, output_names)

        # self.MPC_function.save('P2P_motion.casadi')

    def _export_artifacts(self):
        pass
        # self.mpc.export('p2p_motion', mode='debug', ros2=True, compile=True)


    def optimize(self, current_state, desired_position, x_initial_guess, u_initial_guess):

        dq_res, X_res, U_res = self.MPC_function(current_state, desired_position, x_initial_guess, u_initial_guess)

        return dq_res, X_res, U_res

    