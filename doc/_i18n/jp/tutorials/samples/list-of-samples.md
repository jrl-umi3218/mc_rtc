このページでは、本フレームワークで用意されているサンプルコントローラーのリストに加えて、各コントローラーの実行方法と各コントローラーで実現できる内容について概説します。より複雑なコントローラーについては、各コントローラーのチュートリアルのページで詳しく説明されています。

# Posture

`Posture`（姿勢制御）コントローラー（[オンラインデモ](https://mc-rtc-demo.netlify.app/#robot=JVRC1&controller=EndEffector)を参照）は、本フレームワーク内で最もシンプルなコントローラーです。このコントローラーは、以下のタスクと制約条件を二次計画法ソルバーに追加します。



**タスク**
- {% doxygen mc_tasks::PostureTask %}:
指定された関節の位置を、ロボットの各目標自由度として設定します。このコントローラーは、デフォルトでは、エンコーダー値を利用できる場合はロボットの現在のエンコーダー値を目標として設定し、そうでない場合はデフォルトの姿勢（{% doxygen mc\_rbdyn::RobotModule::stance() %}）を目標として設定します。また、GUIのスライダーを使用して関節を動かすことができます（`Tasks -> posture_jvrc1 -> Target`）。

**制約条件**
- {% doxygen mc_solver::ContactConstraint %}: このコントローラーには、必ず接触面制約条件を設定する必要があります。ただし、接触面セットが空でも構いません。
- {% doxygen mc_solver::KinematicsConstraint %}: ジョイントの可動範囲に関する制約条件
- {% doxygen mc_solver::CollisionsConstraint %}: 自己衝突回避に関する制約条件
- {% doxygen mc_solver::CompoundJointConstraint %}: 複合関節（ある関節の可動範囲が別の関節の値によって決まる関節）の関節可動範囲を処理します。

**サポートされているロボット**:
本フレームワークでサポートされているすべての[ロボット]({{site.baseurl}}/robots.html)。カスタムロボットの場合は、必要に応じて、`CompoundJointConstraint`に関する衝突メッシュのペアと可動範囲から成るデフォルトのセットを定義する必要があります（[新しいロボットの組み込み]({{site.baseurl}}/tutorials/advanced/new-robot.html)に関するチュートリアルを参照）。

**実行方法**:
このコントローラーを実行するには、[mc_rtcの設定]({{site.baseurl}}/tutorials/introduction/configuration.html)でこのコントローラーを指定し、任意のインターフェイスを使用して[コントローラーを実行]({{site.baseurl}}/tutorials/introduction/running-a-controller.html)します。

```yaml
MainRobot: JVRC1
Enabled: Posture
```

# CoM

`CoM`（質量中心）サンプルコントローラー（[オンラインデモ](https://mc-rtc-demo.netlify.app/#robot=JVRC1&controller=CoM)を参照）は、前述の`Posture`コントローラーと同様のタスクと制約条件が定義されています。

**タスク**
- {% doxygen mc_tasks::PostureTask %}: すべての関節にデフォルトの目標を設定するための姿勢制御タスク（このタスクには低い重みが設定されています）
- {% doxygen mc_tasks::CoMTask %}: ロボットの質量中心を制御するタスク。デフォルトでは、このコントローラーは現在のタスクの位置を目標として設定します。GUI（RViz）の対話形式のマーカーを使用して、この目標を動かすことができます。

***制約条件**
- {% doxygen mc_solver::ContactConstraint %}: 左右の足の裏と地面の環境との間に接触面を追加します。
- {% doxygen mc_solver::DynamicsConstraint %}: この制約条件を使用すると、{% doxygen mc_solver::KinematicsConstraint %}で指定された関節可動範囲の制約条件に加え、関節のトルクと摩擦円錐の計算が可能となります。
- {% doxygen mc_solver::CollisionsConstraint %}: 自己衝突回避に関する制約条件
- {% doxygen mc_solver::CompoundJointConstraint %}: 複合関節（ある関節の可動範囲が別の関節の値によって決まる関節）の関節可動範囲を処理します。

**サポートされているロボット**:
本フレームワークでサポートされているすべての二足歩行[ロボット]({{site.baseurl}}/robots.html)。カスタムロボットの場合は、[ロボットの記述]({{site.baseurl}}/tutorials/advanced/new-robot.html)において`LeftFoot`（左足）と`RightFoot`（右足）の裏の面を定義する必要があります。


**実行方法**:
このコントローラーを実行するには、[mc_rtcの設定]({{site.baseurl}}/tutorials/introduction/configuration.html)でこのコントローラーを指定し、任意のインターフェイスを使用して[コントローラーを実行]({{site.baseurl}}/tutorials/introduction/running-a-controller.html)します。

```yaml
MainRobot: JVRC1
Enabled: CoM
```

# EndEffector

`EndEffector`（エンドエフェクター）コントローラー（[オンラインデモ](https://mc-rtc-demo.netlify.app/#robot=JVRC1&controller=EndEffector)を参照）は、前述の`CoM`コントローラーと同じタスクと制約条件が定義されています。このコントローラーでは、質量中心の位置の制御に加えて、ロボットのエンドエフェクターの位置と向きを制御するための{% doxygen mc\_tasks::EndEffectorTask %}が追加されています。

**サポートされているロボット**:
本フレームワークでサポートされているすべての二足歩行[ロボット]({{site.baseurl}}/robots.html)。カスタムロボットの場合は、[ロボットの記述]({{site.baseurl}}/tutorials/advanced/new-robot.html)において`LeftFoot`と`RightFoot`の裏と`r_wrist`ボディを定義する必要があります。

**実行方法**:
このコントローラーを実行するには、[mc_rtcの設定]({{site.baseurl}}/tutorials/introduction/configuration.html)でこのコントローラーを指定し、任意のインターフェイスを使用して[コントローラーを実行]({{site.baseurl}}/tutorials/introduction/running-a-controller.html)します。

```yaml
MainRobot: JVRC1
Enabled: EndEffector
```

# Text

`Text`（テキスト）サンプルコントローラーでは、YAMLで記述された設定から[タスク]({{site.baseurl}}/json.html#MetaTask)と[制約条件]({{site.baseurl}}/json.html#ConstraintSet)を読み込むための{% doxygen mc_tasks::MetaTaskLoader %}と{% doxygen mc_solver::ConstraintSetLoader %}の使い方が示されています。

**実行方法**: 
このコントローラーを実行するには、[mc_rtcの設定]({{site.baseurl}}/tutorials/introduction/configuration.html)でこのコントローラーを指定し、任意のインターフェイスを使用して[コントローラーを実行]({{site.baseurl}}/tutorials/introduction/running-a-controller.html)します。

```yaml
MainRobot: JVRC1
Enabled: Text
```

このコントローラーのデフォルトの設定は、`EndEffector`コントローラーと似ています。また、`~/.config/mc_rtc/controllers/Text.yaml`に独自の設定を記述できます。例えば以下の設定は、`CoM`コントローラーを再度構成して

```yaml
Text:
  constraints:
  - type: dynamics
  - type: contact
  - type: collision
  - type: compoundJoint
  tasks:
  - type: com
    weight: 1000
    above: [LeftFoot]
  contacts:
  - r1Surface: LeftFoot
    r2Surface: AllGround
    isFixed: false
  - r1Surface: RightFoot
    r2Surface: AllGround
    isFixed: false
```

# Admittance

`Admittance`（アドミッタンス制御）サンプルコントローラーでは、所定の力で`JVRC1`ロボットを壁に押し付けるシンプルな有限オートマトンの例を使用して、{% doxygen mc_tasks::force::AdmittanceTask %}の使い方が示されています。

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/B_L_xPynhvU" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>


**詳しい説明**: [アドミッタンス制御のサンプルコントローラーのチュートリアル]({{site.baseurl}}/tutorials/samples/sample-admittance.html)を参照してください。

**サポートされているロボット**: `JVRC1`に遷移します。

**前提条件**: [ForceSensorCalibrationコントローラー](https://github.com/jrl-umi3218/mc_force_sensor_calibration_controller)を使用して手の力覚センサーが事前に較正されているか確認してください。

**実行方法**

このコントローラーを実行するには、[mc_rtcの設定]({{site.baseurl}}/tutorials/introduction/configuration.html)でこのコントローラーを指定し、任意の動力学的シミュレーション（Choreonoidなど）使用して[コントローラーを実行]({{site.baseurl}}/tutorials/introduction/running-a-controller.html)します。

**参考**

より高度な例については、[ExternalForces](#externalforces)のサンプルを参照してください。

```yaml
MainRobot: JVRC1
Enabled: AdmittanceSample
```

# Impedance

`Impedance`（インピーダンス制御）サンプルコントローラーでは、ロボットのエンドエフェクターの位置と力を同時に制御するための{% doxygen mc_tasks::force::ImpedanceTask %}の使い方が示されています。

**サポートされているロボット**: `JVRC1Fixed`に遷移します。

**前提条件**: [ForceSensorCalibrationコントローラー](https://github.com/jrl-umi3218/mc_force_sensor_calibration_controller)を使用して手の力覚センサーが事前に較正されているか確認してください。

**実行方法**

このコントローラーを実行するには、[mc_rtcの設定]({{site.baseurl}}/tutorials/introduction/configuration.html)でこのコントローラーを指定し、任意の動力学的シミュレーション（Choreonoidなど）使用して[コントローラーを実行]({{site.baseurl}}/tutorials/introduction/running-a-controller.html)します。Choreonoidでは、動いている手をクリックしてドラッグすることで、外力を加えることができます。このとき、外力を加えられた手は、所定の範囲内でユーザーの操作に追従します。その後、外力を解放すると、それまでの移動経路をたどって所定の位置に手が戻ります。なお、GUIを使用して、フリーな状態のエンドエフェクターに{% doxygen mc_tasks::force::ImpedanceTask %}を追加できます（`Global -> Add Tasks -> ImpedanceTask`）。

```yaml
MainRobot: JVRC1Fixed
Enabled: Impedance
```

# LIPMStabilizer

`LIPMStabilizer`（LIPMスタビライザー）サンプルコントローラーでは、バランスを取りながら立って前進するロボットを実現するシンプルな準静的有限オートマトンの例を使用して、{% doxygen mc_tasks::StabilizerTask %}とこのタスクの実現に役立つ有限オートマトン状態{% doxygen mc_state::StabilizerStandingState %}の使い方が示されています。

**詳しい説明**: [LIPMスタビライザーのチュートリアル]({{site.baseurl}}/tutorials/recipes/lipm-stabilizer.html)を参照してください。

**サポートされているロボット**:  本フレームワークでサポートされているすべての二足歩行[ロボット]({{site.baseurl}}/robots.html)。

**実行方法**

このコントローラーを実行するには、[mc_rtcの設定]({{site.baseurl}}/tutorials/introduction/configuration.html)でこのコントローラーを指定し、任意の動力学的シミュレーション（Choreonoidなど）使用して[コントローラーを実行]({{site.baseurl}}/tutorials/introduction/running-a-controller.html)します。

```yaml
MainRobot: JVRC1
Enabled: LIPMStabilizer
```

# ExternalForces

`ExternalForces`（外力）サンプルコントローラーでは、所定の力で壁を押す`JVRC1`ロボットを実現するシンプルな有限オートマトンの例を使用して、指定された外力を行使する二足歩行ロボットを{% doxygen mc_tasks::lipm\_stabilizer::StabilizerTask %}と{% doxygen mc_tasks::force::ImpedanceTask %}で実現する方法が示されています。

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/in3cUozkU-A" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

**サポートされているロボット**: `JVRC1`に遷移します。

**前提条件**: [ForceSensorCalibrationコントローラー](https://github.com/jrl-umi3218/mc_force_sensor_calibration_controller)を使用して手の力覚センサーが事前に較正されているか確認してください。

**実行方法**

このサンプルを実行するには、力覚センサーをシミュレート可能な動力学的シミュレーターが必要です。このチュートリアルは、{% link mc_openrtm %}、{% link Choreonoid %}、それに本フレームワークで用意されたシミュレーションファイル`sim_mc_wall.cnoid`を使用することを前提としています。他のシミュレーターを使用する場合は、必要に応じて命令を変更し、ロボットと壁が`55cm`離れて配置されているシーンを作成してください。

[mc_rtcの設定]({{site.baseurl}}/tutorials/introduction/configuration.html)でこのコントローラーを指定し、Choreonoidで[コントローラーを実行]({{site.baseurl}}/tutorials/introduction/running-a-controller.html)してください。このロボットは、両手を壁に触れた後、前に倒れながら、指定された正弦波状の外力を壁に行使します。

```yaml
MainRobot: JVRC1
Enabled: ExternalForces
```
