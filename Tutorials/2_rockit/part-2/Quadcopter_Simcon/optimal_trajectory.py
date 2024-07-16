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
       
        # Construct dynamics vector
        # 
        # Hint: make use of the methods evaluateR and evaluateSinv
        #
        #        
        
        return None
    
    def optimize(self):

        # for plotting purposes, make sure to define any obstacle you include
        # in these class attributes
        self.obstacle_centers = [[2.5, 4, -1],  # list of (x, y, z)
                                 [3, 6, -2.5],
                                 [1.5, 2, 0]]
        self.obstacle_radii = [1.0, 1.5, 1.0]   # list of radii
        
        
        # Define your OCP here
        # TODO
        #
        sol = None


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
