{% comment %}FIXME Some comments are not translated {% endcomment %}

各チュートリアルを通じて、接触面とは何か、コントローラーで接触面がどのように処理されるか、接触面をどのように動かしたらよいのかが理解できたと思います。それでは、左足を開始位置から動かして少し前に進めてみましょう。

基本的な手順は以下のようになります。

1. 質量中心を右足の上に移動する
2. 左足の接触面を削除する
3. 左足を目的の場所に移動する
4. 接触面を元に戻す

では、簡単にこれを実装してみましょう。

## 簡単な実装

まず、空のコントローラーを作成します。

```bash
$ mc_rtc_new_controller MyController MyController
```

次に、コントローラーの構成要素を以下のように編集します。

- `MyController.h`:

```cpp
// MyController.h
#pragma once
#include <mc_control/mc_controller.h>
#include "api.h"
// If we want to use elements in mc_tasks or mc_solver we first must include their header file
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/SurfaceTransformTask.h>

struct MyController_DLLAPI MyController : public mc_control::MCController
{
    MyController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
    bool run() override;
    void reset(const mc_control::ControllerResetData & reset_data) override;

 protected:
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_tasks::SurfaceTransformTask> footTask;
    bool moved_com = false;
    bool moved_left_foot = false;

};
```

- `MyController.cpp`:

```cpp
#include "MyController.h"

MyController::MyController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & /* config */)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({
      mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"),
      mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")
  });

  mc_rtc::log::success("MyController init done ");
}

bool MyController::run()
{
  bool ret = MCController::run();
  if(!moved_com)
  {
    if(comTask->eval().norm() < 5e-2 && comTask->speed().norm() < 1e-4)
    {
      mc_rtc::log::success("Moved the CoM");
      moved_com = true;
      solver().setContacts({
        mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")
      });
      solver().addTask(footTask);
      footTask->target(sva::PTransformd(Eigen::Vector3d(0.4, 0, 0)));
    }
    return ret;
  }
  if(moved_com && !moved_left_foot)
  {
    if(footTask->eval().norm() < 5e-2 && footTask->speed().norm() < 1e-4)
    {
      mc_rtc::log::success("Moved the left foot");
      moved_left_foot = true;
      solver().setContacts({
          mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"),
          mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")
      });
      solver().removeTask(footTask);
      auto com = comTask->com();

      com.x() =
        (robot().surfacePose("LeftFoot").translation().x()
         +
         robot().surfacePose("RightFoot").translation().x())/2;

      com.y() = 0;
      comTask->com(com);
    }
    return ret;
  }
  return ret;
}

void MyController::reset(const mc_control::ControllerResetData & data)
{
  MCController::reset(data);
  /* Create the task */
  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
  footTask = std::make_shared<mc_tasks::SurfaceTransformTask>("LeftFoot", robots(), 0);
  /* Move the CoM above the right foot */
  comTask->com(comTask->com() + Eigen::Vector3d(0, -1*footTask->surfacePose().translation().y(), 0));
  /* Add the CoM task to the solver */
  solver().addTask(comTask);
}

CONTROLLER_CONSTRUCTOR("MyController", MyController)
```

ロボットの足が床に沿って「スライド」するのが分かると思います。目的の場所に移動するには、これが一番直接的な方法です。この問題を解決するにはさまざまな方法がありますが、ここでは、便利な2つの`MetaTask`を使用する方法について説明します。

- `AddRemoveContactTask`を使用して、左足の接触面の削除と追加を行う
- `MoveContactTask`を使用して、2点間で足を移動させる

## `AddRemoveContactTask`を使用する

`AddRemoveContactTask`を使用するには、ソルバーの[速度制約条件]({{site.baseurl}}/tutorials/recipes/speed-constraint.html)が必要となります。また、タスクそのものを作成する必要があります。さらに、2つの補助ステップを使って、左足を離して元に戻す必要があります。では、コントローラーのヘッダーと`reset()`関数をそのように変更してみましょう。

```cpp
// In the header
#include <mc_solver/BoundedSpeedConstr.h>
#include <mc_tasks/AddRemoveContactTask.h>
std::shared_ptr<mc_solver::BoundedSpeedConstr> bSpeedConstr;
std::shared_ptr<mc_tasks::AddRemoveContactTask> aRContactTask;
bool removed_left_foot = false;
bool added_left_foot = false;
```

```cpp
// In reset
bSpeedConstr = std::make_shared<mc_solver::BoundedSpeedConstr>(robots(), 0, timeStep);
solver().addConstraintSet(*bSpeedConstr);
```

接触面を削除するには、`AddRemoveContactTask`の特殊化関数`RemoveContactTask`を使用します。プロトタイプ宣言は以下のようになります。
```cpp
  RemoveContactTask(mc_rbdyn::Robots & robots,
                 std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                 mc_rbdyn::Contact & contact,
                 double speed, double stiffness, double weight,
                 Eigen::Vector3d * userT_0_s = nullptr);
```

接触面を追加するには、特殊化関数`AddContactTask`を使用します。プロトタイプ宣言は以下のようになります。
```cpp
  AddContactTask(mc_rbdyn::Robots & robots,
                 std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                 mc_rbdyn::Contact & contact,
                 double speed, double stiffness, double weight,
                 Eigen::Vector3d * userT_0_s = nullptr);
```

どちらの場合も引数は非常に似ています。最も重要なのは以下の引数です。
- `mc_rbdyn::Contact & contact` は、追加あるいは削除する接触面
- `double speed` は、挿入・削除の速度

なお、このタスクでは、ソルバーにおける実際の接触面リストの管理や、衝突回避は考慮されていないことに留意してください。

```cpp
bool MyController::run()
{
  bool ret = MCController::run();
  if(!moved_com)
  {
    if(comTask->eval().norm() < 5e-2 && comTask->speed().norm() < 1e-4)
    {
      mc_rtc::log::success("Moved the CoM");
      moved_com = true;
      solver().setContacts({
        mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")
      });
      mc_rbdyn::Contact rmContact(robots(), "LeftFoot", "AllGround");
      aRContactTask.reset(new mc_tasks::RemoveContactTask(robots(),
                                                          bSpeedConstr,
                                                          rmContact,
                                                          0.05,
                                                          2.0, 1000));
      solver().addTask(aRContactTask);
    }
    return ret;
  }
  if(moved_com && !removed_left_foot)
  {
    /* Monitor the foot altitude */
    double lf_z = robot().surfacePose("LeftFoot").translation().z();
    double rf_z = robot().surfacePose("RightFoot").translation().z();
    if(lf_z > rf_z + 0.05)
    {
      mc_rtc::log::success("Left foot contact removed");
      removed_left_foot = true;
      solver().removeTask(aRContactTask);
      solver().addTask(footTask);
      footTask->add_ef_pose(sva::PTransformd(Eigen::Vector3d(0.4, 0, 0)));
    }
    return ret;
  }
  if(removed_left_foot && !moved_left_foot)
  {
    if(footTask->eval().norm() < 5e-2 && footTask->speed().norm() < 1e-4)
    {
      mc_rtc::log::success("Moved the left foot");
      moved_left_foot = true;
      solver().removeTask(footTask);
      mc_rbdyn::Contact addContact(robots(), "LeftFoot", "AllGround");
      aRContactTask.reset(new mc_tasks::AddContactTask(robots(),
                                                       bSpeedConstr,
                                                       addContact,
                                                       0.05,
                                                       2.0, 1000));
      solver().addTask(aRContactTask);
      return ret;
    }
  }
  if(moved_left_foot && !added_left_foot)
  {
    /* Monitor the foot altitude */
    double lf_z = robot().surfacePose("LeftFoot").translation().z();
    double rf_z = robot().surfacePose("RightFoot").translation().z();
    if(fabs(lf_z - rf_z) < 1e-4)
    {
      mc_rtc::log::success("Added left foot");
      added_left_foot = true;
      solver().removeTask(aRContactTask);
      solver().setContacts({
          mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"),
          mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")
      });
      auto com = comTask->com();

      com.x() =
        (robot().surfacePose("LeftFoot").translation().x()
         +
         robot().surfacePose("RightFoot").translation().x())/2;

      com.y() = 0;
      comTask->com(com);
    }
  }
  return ret;
}
```
