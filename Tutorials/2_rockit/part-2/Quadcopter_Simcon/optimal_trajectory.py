import numpy as np
import matplotlib.pyplot as plt

from rockit import *
from casadi import *

from quadFiles.quad import Quadcopter
from utils.windModel import Wind

class OptimalTrajectory:

    def __init__(self, quad:Quadcopter, ctrlType:str):
        # backward compatibility parameters
        self.ctrlType = ctrlType
        self.yawType = 3
        self.wps = np.array([[0, 0, 0]])
        self.xyzType = 1

        # quadcopter model
        self.quad = quad

        # initialize desired state
        self.desPos = np.array([0, 0, 0])
        self.desVel = np.array([0, 0, 0])
        self.desAcc = np.array([0, 0, 0])
        self.desThr = np.array([0, 0, 0])
        self.desEul = np.array([0, 0, 0])
        self.desPQR = np.array([0, 0, 0])
        self.desYawRate = 0

        self.sDes = np.hstack((self.desPos, self.desVel, self.desAcc, 
                               self.desThr, self.desEul, self.desPQR, 
                               self.desYawRate)).astype(float)

        # optimize trajectory
        self.optimize()

        # starting position
        self.sDes = self.desiredState(0, None, None)

    def getTf(self):
        return self.samples_tt[-1]

    def interpolateSampledTrajectory(self, t):
        # interpolate sampled trajectory
        if t < self.samples_tt[0]:
            return self.samples_xx[0], self.samples_uu[0]
        elif t > self.samples_tt[-1]:
            return self.samples_xx[-1], self.samples_uu[-1]
        else:
            for i in range(len(self.samples_tt)-1):
                if self.samples_tt[i] <= t and t <= self.samples_tt[i+1]:
                    t_left = self.samples_tt[i]
                    t_right = self.samples_tt[i+1]
                    
                    x_left = self.samples_xx[i]
                    x_right = self.samples_xx[i+1]

                    u_left = self.samples_uu[i]
                    u_right = self.samples_uu[i+1]

                    return x_left + (x_right - x_left) * (t - t_left) / (t_right - t_left), \
                           u_left + (u_right - u_left) * (t - t_left) / (t_right - t_left)

    def desiredState(self, t, Ts, quad):
        interpolated_state, interpolated_control = self.interpolateSampledTrajectory(t)
        
        p = interpolated_state[0:3]
        v = interpolated_state[3:6]
        eul = interpolated_state[6:9]
        phi = eul[0]
        theta = eul[1]
        psi = eul[2]

        omegas = interpolated_control[0:3]
        a = interpolated_control[3]

        R = self.evaluateR(phi, theta, psi)
        Sinv = self.evaluateSinv(phi, theta)

        omegas = interpolated_control[0:3]
        thrust = interpolated_control[3]

        accel = (R @ np.transpose(np.array([0, 0, a])) - \
                 np.array([0, 0, self.quad.params["g"]])).T
        thrust_in_world_frame = (R @ np.transpose(np.array([0, 0, a]))).T

        if t < self.samples_tt[-1]:
            self.desPos = p
            self.desVel = v
            self.desAcc = np.array([float(accel[i]) for i in range(3)])
            self.desThr = np.array([float(thrust_in_world_frame[i]) for i in range(3)])
            self.desEul = eul
            self.desPQR = omegas 
            self.desYawRate = float((Sinv @ omegas)[2,0])
        else:
            self.desPos = p
            self.desVel = np.array([0, 0, 0])
            self.desAcc = np.array([0, 0, 0])
            self.desThr = np.array([0, 0, self.quad.params["g"]])
            self.desEul = np.array([0, 0, 0])
            self.desPQR = np.array([0, 0, 0])
            self.desYawRate = 0

        self.sDes = np.hstack((self.desPos, self.desVel, self.desAcc, 
                               self.desThr, self.desEul, self.desPQR, 
                               self.desYawRate)).astype(float)
                
        return self.sDes
    
    def evaluateR(self, phi, theta, psi):
        cr = cos(phi)
        sr = sin(phi)
        cp = cos(theta)
        sp = sin(theta)
        cy = cos(psi)
        sy = sin(psi)

        return vertcat(horzcat(cy*cp, cy*sp*sr-sy*cr, cy*sp*cr + sy*sr),
                       horzcat(sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
                       horzcat(-sp, cp*sr, cp*cr))
    
    def evaluateSinv(self, phi, theta):
        cr = cos(phi)
        sr = sin(phi)
        cp = cos(theta)
        tp = tan(theta)

        return vertcat(horzcat(1, sr*tp, cr*tp),
                       horzcat(0, cr, -sr),
                       horzcat(0, sr/cp, cr/cp))
    
    def evaluateQuadcopterDynamics(self, current_state, current_controls):
        # for more information, see: https://www.sciencedirect.com/science/article/pii/S2405896323011540?via%3Dihub
        # "Optimal and Reactive Control for Agile Drone flight in Cluttered Environments"
        # by Dries Dirckx, Mathias Bos, Wilm Decré, and Jan Swevers
        
        # for readability, extract symbols from state vector
        p = current_state[0:3]
        v = current_state[3:6]
        eul = current_state[6:9]
        phi = eul[0]
        theta = eul[1]
        psi = eul[2]

        # get R and Sinv
        R = self.evaluateR(phi, theta, psi)
        Sinv = self.evaluateSinv(phi, theta)

        # construct acceleration vectors
        a = R @ vertcat(0, 0, current_controls[3]) - \
            vertcat(0, 0, self.quad.params["g"])

        # stack everything together
        dynamics = vertcat(v,
                           a,
                           Sinv @ current_controls[0:3])
        
        return dynamics
    
    def optimize(self):

        # for plotting purposes, make sure to define any obstacle you include
        # in these class attributes
        self.obstacle_centers = [[2.5, 4, -1],  # list of (x, y, z)
                                 [3, 6, -2.5],
                                 [1.5, 2, 0]]
        self.obstacle_radii = [1.0, 1.5, 1.0]   # list of radii
        
        
        # Define your OCP here
        ocp = Ocp(T=4)

        # Define states
        p = ocp.state(3)
        v = ocp.state(3)
        eul = ocp.state(3)
        x = vcat([p, v, eul])
        phi = x[6]
        theta = x[7]
        psi = x[8]

        # Define controls
        u = ocp.control(4) #(roll rate, pitch rate, yaw rate, thrust)

        # Set dynamics
        ocp.set_der(x, self.evaluateQuadcopterDynamics(x, u))

        # Set initial condition
        ocp.subject_to(ocp.at_t0(x) == 0)

        # Set final condition
        pf = [5.0, 8.0, -2.0]
        ocp.subject_to(ocp.at_tf(x[:3]) == pf)
        ocp.subject_to(ocp.at_tf(x[3:]) == 0)

        # Set thrust bounds
        u_min = 0
        u_max = 15
        ocp.subject_to(u_min <= (u[3] <= u_max))

        # Set rolling velocity limits
        max_roll_rate = 1.0
        ocp.subject_to(-max_roll_rate <= (u[0] <= max_roll_rate))
        ocp.subject_to(-max_roll_rate <= (u[1] <= max_roll_rate))
        ocp.subject_to(-max_roll_rate <= (u[2] <= max_roll_rate))

        # Set tilt limits
        max_tilt = np.pi/2
        ocp.subject_to(-max_tilt <= (phi <= max_tilt))
        ocp.subject_to(-max_tilt <= (theta <= max_tilt))
        ocp.subject_to(-max_tilt <= (psi <= max_tilt))

        # Set obstacle avoidance constraints
        for i in range(len(self.obstacle_centers)):
            p_obs = self.obstacle_centers[i]
            R_obs = self.obstacle_radii[i]
            ocp.subject_to((p[0]-p_obs[0])**2 + (p[1]-p_obs[1])**2 + 
                        (p[2]-p_obs[2])**2 >= R_obs**2)

        # Set objective
        ocp.add_objective(ocp.integral(u.T @ u))

        # Set transcription and solver options
        N = 20; M = 5
        ocp.method(MultipleShooting(N=N, M=M, intg='rk'))
        ocp.solver('ipopt')

        # Add inintial guess
        ocp.set_initial(u, vertcat(0, 0, 0, self.quad.params["g"]))
        ocp.set_initial(p, np.linspace(np.array([0, 0, 0]), np.array(pf), N+1).T)

        # solve
        sol = ocp.solve()


        # Store the trajectory
        self.samples_tt, self.samples_xx = sol.sample(x, grid='integrator')
        _, self.samples_uu = sol.sample(u, grid='integrator')

        # plot solution
        self.plot_ocp_solution()

    def plot_ocp_solution(self):
        plt.figure()

        # plot positions
        plt.subplot(3, 1, 1)
        plt.plot(self.samples_tt, self.samples_xx[:, 0], label='px')
        plt.plot(self.samples_tt, self.samples_xx[:, 1], label='py')
        plt.plot(self.samples_tt, self.samples_xx[:, 2], label='pz')
        plt.legend()

        # plot velocities
        plt.subplot(3, 1, 2)
        plt.plot(self.samples_tt, self.samples_xx[:, 3], label='vx')
        plt.plot(self.samples_tt, self.samples_xx[:, 4], label='vy')
        plt.plot(self.samples_tt, self.samples_xx[:, 5], label='vz')
        plt.legend()

        # plot controls
        plt.subplot(3, 1, 3)
        plt.plot(self.samples_tt, self.samples_uu[:, 0], label='roll rate')
        plt.plot(self.samples_tt, self.samples_uu[:, 1], label='pitch rate')
        plt.plot(self.samples_tt, self.samples_uu[:, 2], label='yaw rate')
        plt.plot(self.samples_tt, self.samples_uu[:, 3], label='thrust')
        plt.legend()

        plt.show()
