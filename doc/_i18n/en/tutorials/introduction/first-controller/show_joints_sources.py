import mc_rbdyn
# Here you can change the robot module name with your own robot
rm = mc_rbdyn.get_robot_module("JVRC1")
print("\n".join(["- {}".format(j.name()) for j in rm.mb.joints() if j.dof() == 1]))
