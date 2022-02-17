// 現在の目標を取得する
auto pt = efTask->get_ef_pose();
// 回転と位置の目標を更新する
efTask->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.5, -0.5, 1.2}});
