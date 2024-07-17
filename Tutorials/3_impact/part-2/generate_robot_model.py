from robotsmeco import RobotModel

############################
# Create model
############################
urdf_file = "./franka_panda.urdf"
options = {
    'cse':True, 
    'analytical_derivatives':True, 
    'control_input':'acceleration',
    }
robot = RobotModel(urdf_file, options)
export_options = {'functions':['fk','fd','id','J_fd','J_id'], 'compressed':True}
robot.export_model('franka_panda',export_options)

robot.export_ode('franka_panda')

