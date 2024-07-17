import swift
import roboticstoolbox as rtb

from controller import IMPACT_Controller

############################
# Set controller
############################
controller = IMPACT_Controller(model_file = 'franka_panda.impz')

############################
# Simulator
############################

env = swift.Swift()
env.launch(realtime=True)

panda = rtb.models.Panda()
panda.q = panda.qr

desired_point = [0.7, -0.2, 0.4]


panda.qd = [0]*7

current_state = panda.q.tolist() + panda.qd.tolist()

dq_input, x_opt, u_opt = controller.optimize(current_state, desired_point, [0]*controller.model.nx, [0]*controller.model.nu)

arrived = False
env.add(panda)

dt = 0.05

i = 0

while i <= 500:

    current_state = panda.q.tolist() + panda.qd.tolist()

    dq_input, x_opt, u_opt = controller.optimize(current_state, desired_point, x_opt, u_opt)

    panda.qd = dq_input.full().flatten()
    env.step(dt)

    print(f'## Iteration {i}')
    if i == 75:
        desired_point = [0.2, 0.2, 0.4]
    elif i == 150:
        desired_point = [0.7, -0.2, 0.4]
    elif i == 225:
        desired_point = [0.2, -0.2, 0.4]
    elif i == 300:
        desired_point = [0.5, -0.2, 0.6]

    i += 1

# Uncomment to stop the browser tab from closing
env.hold()