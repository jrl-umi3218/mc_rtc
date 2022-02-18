// Get the current objective
auto pt = efTask->get_ef_pose();
// Update the rotation and position objective
efTask->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.5, -0.5, 1.2}});
