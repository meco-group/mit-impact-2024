import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from .plot_trailer import *
from math import cos, sin
from impact import MPC, MultipleShooting


class Simulator:
    def __init__(self):
        _mpc = MPC()
        self.model = _mpc.add_model('truck_trailer','truck_trailer.yaml')
        _mpc.method(MultipleShooting(N=50, M=1, intg='rk'))
        _mpc.solver('ipopt')
        self.vehicle_dynamics = _mpc.discrete_system()

        self.objects = []

    def add_object(self, **kwargs):
        self.objects.append(kwargs)

    def simulate_vehicle(self,x0,u,dt):
        return self.vehicle_dynamics(x0=x0, u=u, T=dt)['xf']

    def _compute_truck_position(self, state):
        # state = [x1, y1, theta0, theta1]
        x0 = state[0] + self.model.L1*cos(state[3]) + self.model.M0*cos(state[2])
        y0 = state[1] + self.model.L1*sin(state[3]) + self.model.M0*sin(state[2])
        return x0, y0

    def plot_path(self,x1,y1,x0,y0):
        from pylab import show

        fig, ax1 = plt.subplots(figsize=(10,10))
        ax1.axis('equal')
        ax1.set_xlim(left=-1, right=4)
        ax1.set_ylim(bottom=-1, top=4)

        # Plot objects
        for obj in self.objects:
            if obj['object_type'] == 'lane':
                x_min, x_max, y_min, y_max = obj['params']
                ax1.plot([x_min,x_min], [y_min,y_max], 'k--', linewidth=1.5)
                ax1.plot([x_max,x_max], [y_min,y_max], 'k--', linewidth=1.5)
            
            elif obj['object_type'] == 'roundabout':
                x_center, y_center, inner_radius, external_radius = obj['params']
                circle1 = plt.Circle((x_center, y_center), inner_radius, color='lightgreen')
                ax1.add_patch(circle1)
            
            elif obj['object_type'] == 'wall':
                x_center, y_center, width, height = obj['params']
                ax1.add_patch(
                    Rectangle((x_center, y_center), width, height, edgecolor='pink', facecolor='white', hatch='xx')
                )

        truck_xy_sim = ax1.plot(x1,y1, '--', color='darkgrey')
        trailer_xy_sim = ax1.plot(x0,y0, '--', color='darkred')

        show(block=True)

    def plot_trajectory(self, state_trajectory, control_trajectory, use_simulator=True):
        from pylab import show, pause

        N_sim = state_trajectory.shape[1]

        fig, ax1 = plt.subplots(figsize=(10,10))

        ax1.axis('equal')
        ax1.set_xlim(left=-1, right=4)
        ax1.set_ylim(bottom=-1, top=4)

        W0, L0, M0 = self.model.W0, self.model.L0, self.model.M0
        W1, L1, M1 = self.model.W1, self.model.L1, self.model.M1

        # Plot objects
        for obj in self.objects:
            if obj['object_type'] == 'lane':
                x_min, x_max, y_min, y_max = obj['params']
                ax1.plot([x_min,x_min], [y_min,y_max], 'k--', linewidth=1.5)
                ax1.plot([x_max,x_max], [y_min,y_max], 'k--', linewidth=1.5)
            
            elif obj['object_type'] == 'roundabout':
                x_center, y_center, inner_radius, external_radius = obj['params']
                circle1 = plt.Circle((x_center, y_center), inner_radius, color='lightgreen')
                ax1.add_patch(circle1)
            
            elif obj['object_type'] == 'wall':
                x_center, y_center, width, height = obj['params']
                ax1.add_patch(
                    Rectangle((x_center, y_center), width, height, edgecolor='pink', facecolor='white', hatch='xx')
                )

        # Plot simulation
        for k in range(N_sim - 1):
            x1s     = state_trajectory[0,k]
            y1s     = state_trajectory[1,k]
            theta0s = state_trajectory[2,k]
            theta1s = state_trajectory[3,k]

            delta0s = control_trajectory[0,k]

            x0s, y0s = self._compute_truck_position(state_trajectory[:,k])
            

            truck           = vehic_to_plot(ax1, x0s, y0s, theta0s, W0/2,  W0/2,      L0, M0, color='grey')
            truck_steer     = wheel_to_plot(ax1, x0s, y0s, theta0s,   L0,     0, delta0s,     color='k')
            truck_fixed_1   = wheel_to_plot(ax1, x0s, y0s, theta0s,    0,  W0/2,       0,     color='k')
            truck_fixed_2   = wheel_to_plot(ax1, x0s, y0s, theta0s,    0, -W0/2,       0,     color='k')
            truck_xy        = ax1.plot(x0s, y0s, 'x', color='grey')
            # if use_simulator:
            #     truck_xy_sim = ax1.plot(x0_sim[k], y0_sim[k], '.', color='darkgrey')

            trailer         = vehic_to_plot(ax1, x1s, y1s, theta1s, W1/2,  W1/2,   .8*L1, M1, color='r')
            trailer_fixed_1 = wheel_to_plot(ax1, x1s, y1s, theta1s,    0,  W1/2,       0,     color='k')
            trailer_fixed_2 = wheel_to_plot(ax1, x1s, y1s, theta1s,    0, -W1/2,       0,     color='k')
            trailer_xy      = ax1.plot(x1s, y1s, 'x', color='r')
            # if use_simulator:
            #     trailer_xy_sim = ax1.plot(x1_sim[k], y1_sim[k], '.', color='darkred')

            coupling     = vert_single(x0s, y0s, theta0s, M0, 0)
            coupling_xy  = ax1.plot([x1s, coupling[0][0]], [y1s, coupling[0][1]], '-', color='k')
            coupling_dot = ax1.plot(coupling[0][0], coupling[0][1], 'o', color='k')
    
            truck_xy_sim = ax1.plot(x0s, y0s, '.', color='darkgrey')
            trailer_xy_sim = ax1.plot(x1s, y1s, '.', color='darkred')

            pause(.001)
            if k < N_sim-2:
                truck.pop(0).remove()
                truck_steer.pop(0).remove()
                truck_fixed_1.pop(0).remove()
                truck_fixed_2.pop(0).remove()
                truck_xy.pop(0).remove()
                trailer.pop(0).remove()
                trailer_fixed_1.pop(0).remove()
                trailer_fixed_2.pop(0).remove()
                trailer_xy.pop(0).remove()
                coupling_xy.pop(0).remove()
                coupling_dot.pop(0).remove()

        show(block=True)