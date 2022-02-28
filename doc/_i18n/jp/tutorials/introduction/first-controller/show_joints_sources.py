import mc_rbdyn
# 以下のロボットモジュールの名前を自分のロボットのものに変える
rm = mc_rbdyn.get_robot_module("JVRC1")
print("\n".join(["- {}".format(j.name()) for j in rm.mb.joints() if j.dof() == 1]))
