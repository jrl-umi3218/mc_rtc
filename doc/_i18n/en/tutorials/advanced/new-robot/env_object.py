import mc_rbdyn

path = "/path/to/description"
name = "name"
env = mc_rbdyn.get_robot_module("env", path, name)

# object will load the robot with a floating base
object = mc_rbdyn.get_robot_module("object", path, name)
