# Get the current objective
pt = efTask.get_ef_pose()
# Update the rotation and position objective
efTask.set_ef_pose(sva.PTransformd(sva.RotY(-math.pi/2), eigen.Vector3d(0.5, -0.5, 1.2)))
