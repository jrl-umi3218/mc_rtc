import mc_rbdyn

path = "/path/to/description"
name = "name"
env = mc_rbdyn.get_robot_module("env", path, name)

# オブジェクトによってロボットと浮遊ベースが読み込まれる
object = mc_rbdyn.get_robot_module("object", path, name)
