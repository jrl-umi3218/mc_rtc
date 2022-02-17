# 現在の目標を取得する
pt = efTask.get_ef_pose()
# 回転と位置の目標を更新する
efTask.set_ef_pose(sva.PTransformd(sva.RotY(-math.pi/2), eigen.Vector3d(0.5, -0.5, 1.2)))
