#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>

// 「box」のビジュアル記述から「robot_support」ロボットを定義する
auto boxYaml =
    R"(
  name: robot_support
  origin:
    translation: [0, 0, 0]
    rotation: [0, 0, 0]
  material:
    color:
      r: 1.0
      g: 0.0
      b: 0.0
      a: 1.0
  geometry:
    box:
      size: [0.7, 1.2, 1.2]
  inertia:
    mass: 50
  fixed: true
  )";
auto boxConfig = mc_rtc::Configuration::fromYAMLData(boxYaml);

// 上記のビジュアル記述から「robot_support」ロボットモジュールを作成する
auto supportRm = mc_rbdyn::robotModuleFromVisual("robot_support", boxConfig);
// 既存のロボットモジュールからパンダロボットモジュールをロードする
auto pandaRm = mc_rbdyn::RobotLoader::get_robot_module("PandaDefault");

// PandaDefaultモジュールの「world」リンクのベースとrobot_supportリンクの間の変換
auto box_to_robot = sva::PTransformd(Eigen::Vector3d{0., 0., -1.2});

// 既存の「PandaDefault」モジュールを「box」モジュールに接続して新しいロボットモジュールをロードする
auto pandaOnBoxRm = supportRm->connect(
    *pandaRm, // この他のロボットモジュール（パンダ）に接続する
    "robot_support", // 自分のモジュールの参照リンク
    "world", // 他のロボットモジュール（パンダ）のこのリンクに接続する
    "", // リンク名に追加するオプションのプレフィックス
    mc_rbdyn::RobotModule::ConnectionParameters{}.name(robot_name).X_other_connection(box_to_robot) // 接続パラメータ
);

// ctlはmc_control::MCControllerであり、ロボットをコントローラに追加する
auto robot = ctl.loadRobot(*pandaOnBoxRm);
