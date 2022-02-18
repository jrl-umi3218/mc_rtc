{% comment %}FIXME Some comments are not translated {% endcomment %}

## 概要

このチュートリアルで示すスタビライザイーは、元々、Stéphane Caron博士の[LIPM歩行コントローラー](https://github.com/stephane-caron/lipm_walking_controller)に実装されたものです。この実装は、階段上りや荒れ地での移動などの歩行制御で重点的に利用されています。特に、エアバスの工場の製造現場で利用されている事例は注目に値します。`mc_tasks::lipm_stabilizer::StabilizerTask`で記述されている実装では、Caron博士のオリジナルの実装で記述されているメソッドが組み込まれています。この安定化アルゴリズムは、概念的にはオリジナルのアルゴリズムと比べて大きな違いはありません。現在、歩行制御は別の[コントローラー](https://github.com/jrl-umi3218/lipm_walking_controller)で提供されています。

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/vFCFKAunsYM" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>


## スタビライザーの概要

安定化アルゴリズムの詳細については、2019年3月にカナダのモントリオールで開催されたICRAにおいてStéphane Caron、Abderrahmane Kheddar、Olivier Tempierによって発表されたオリジナルの論文[Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control（全身のアドミッタンス制御を用いたHRP-4ヒューマノイドロボットの階段上昇動作の安定化）](https://scaron.info/publications/icra-2019.html)を参照してください。

ここで説明するスタビライザーは、`mc_tasks::lipm_stabilizer::StabilizerTask`に`MetaTask`として実装されています。すなわち、以下のタスクを管理するタスクとして実装されています。

- スタビライザーによって計算された質量中心目標位置に追従する`CoMTask`
- ロボットの左右の足の下における目標レンチに追従する2つの`CoPTask`
- 上体の挙動を正規化するために腰と胴体をそれぞれ制御する2つの`OrientationTasks`

`StabilizerTask`は、実際のロボットシステムが線形倒立振子モデルに基づき可能な限り最適な動作を行えるように、目標を計算してこれらのタスクに与えます。したがって、動的な移動経路を安定させるには、以下の入力をスタビライザーに与える必要があります。

- `CoM`の目標位置
- `CoM`の目標速度
- `CoM`の目標加速度
- `ZMP`の目標位置

これらをすべて参照する場合、以下のように記述されます。

```cpp
void StabilizerTask::target(
            const Eigen::Vector3d & com,
            const Eigen::Vector3d & comd,
            const Eigen::Vector3d & comdd,
            const Eigen::Vector3d & zmp);
```

本フレームワークでは、振子の静的な目標位置を与えるための便利な補助関数が用意されています。なお、この補助関数は、ロボットを所定の姿勢で安定させる場合にのみ有効に機能することに注意してください。質量中心を動的に動かす場合はこの補助関数を使用しないでください。

```cpp
void StabilizerTask::staticTarget(const Eigen::Vector3d & com, double zmpHeight = 0);
```

Iこの場合、有効な基準状態はユーザーが与える必要があります。歩行の場合、動的な質量中心の位置は、一般にモデル予測コントローラー（MPC）によって計算されます。これにより、足を踏み出している間の動的な安定性が確保されます。

## 状態の観測

スタビライザーの目的は、基準となる振子の目標状態を追いかけるように実システムを制御することです。これを実現するには、以下に示す実ロボットの状態を観測する必要があります。

- `CoM`の位置
- `CoM`の速度
- 接触面のレンチ

必要な物理量を推定するには、適切な[観測器パイプライン](observers.html)をセットアップする必要があります。以下のパイプラインを使用することができます。これらはオリジナルの実装と同じ機能を実現しています。

```yaml
# Observes real robot state
RunObservers: [Encoder, KinematicInertial]
# Updates the Controller::realRobot() instance from the observed state
UpdateObservers: [Encoder, KinematicInertial]
```

- `Encoder`観測器は、エンコーダーの測定値に基づいて、ボディの位置と速度を計算します。
- `KinematicInertial`は、IMUの測定値と、ロボットの両足の間にある基準アンカーフレームに基づいて、浮遊ベースの姿勢と速度を計算します。

**注:** 有効な連続アンカーフレームはユーザーが与える必要があります。このスタビライザーの実装では、以下の関数を使用して簡単にアンカーフレームを設定できます。

```cpp
sva::PTransformd anchorFrame() const;
sva::PTransformd anchorFrameReal() const;
```

今のところ、ループ処理が実行されるたびに、以下の関数を呼び出してこれらの値を明示的に`KinematicInertial`観測器に渡す必要があります。

```cpp
mc_controller::Controller & ctl = ...; // Controller's instance
mc_tasks::lipm_stabilizer::StabilizerTask & stabilizer = ...; // Stabilizer instance
ctl.anchorFrame(stabilizer.anchorFrame());
ctl.anchorFrameReal(stabilizer.anchorFrameReal());
```

観測器パイプラインは、これらの値が渡されると、`realRobot()`の予測状態を自動的に更新します。スタビライザータスクは、更新された予測状態を使用して、タスクの目標を計算します。

## ゲインの設定

以下のパラメーターによって、振子の目標状態と観測状態とのずれに対してスタビライザーがどのように応答するかが決定されます。これらのゲインの調整方法については、[Stephaneのドキュメント](https://jrl-umi3218.github.io/lipm_walking_controller/doxygen/HEAD/stabilizer.html)を参照してください。

```yaml
####
# Sample stabilizer configuration for the JVRC robot
# The most important entries are the
# - dcm_tracking: controls how the stabilizer reacts to perturbation of the DCM
# - admittance: controls the contact admittance
###
# Sole-floor friction coefficient
friction: 0.7
# Configuration of the tasks managed by the stabilizer
tasks:
  com:
    stiffness: [1000, 1000, 100]
    weight: 1000
    active_joints: [Root,
                    R_HIP_Y, R_HIP_R, R_HIP_P, R_KNEE, R_ANKLE_P, R_ANKLE_R,
                    L_HIP_Y, L_HIP_R, L_HIP_P, L_KNEE, L_ANKLE_P, L_ANKLE_R]
    height: 0.85
  contact:
    damping: 300
    stiffness: 1
    weight: 10000
  pelvis:
    stiffness: 10
    weight: 100
  torso:
    stiffness: 10
    weight: 100
    pitch: 0
fdqp_weights:
  net_wrench: 10000
  ankle_torque: 100
  pressure: 1
vdc:
  frequency: 1
  stiffness: 1000
admittance:
  cop: [0.01, 0.01]
  dfz: 0.0001
  dfz_damping: 0
dcm_tracking:
  gains:
    prop: 5.0
    integral: 10
    deriv: 0.5
  derivator_time_constant: 1
  integrator_time_constant: 10

# If you are using the same configuration file with different robots, you can provide per-robot configuration as well
jvrc1:
  admittance:
    cop: [0.02, 0.02]

  ```

これは難しく見えるかもしれません。でも心配は要りません。通常、新しいロボットのスタビライザーを調整するとき以外は、これらのゲインを明示的に指定する必要はありません。`RobotModule`にはデフォルトのスタビライザーの設定が用意されており、コントローラーから以下のようにアクセスできます。

```cpp
mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
```

また、設定からタスクを簡単に読み込むことができます

```cpp
auto stabilizerTask = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(ctl.solver(), config);
ctl.solver().addTask(stabilizerTask);
```

このとき、スタビライザーは以下のように設定されます。

- `RobotModule::defaultLIPMStabilizerConfiguration()`で定義されているデフォルトの設定。`mc_rtc`でサポートされているすべての[ロボット]({{site.baseurl}}/robots.html)について、有効な設定が用意されています。
- YAMLで記述されたタスクの設定
- 特定のロボットについて、ロボット固有の設定をYAMLで記述することで、これらの値を変更することができます。

YAML形式の設定の詳細については、[JSONスキーマ](../../json.html#MetaTask/LIPMStabilizerTask)を参照してください。

## スタビライザーを使用する（ユーザーが定義する場合）

独自のコントローラー内にスタビライザータスクを作成するには、`mc_tasks::lipm_stabilizer::StabilizerTask`を実体化して以下のように記述する必要があります。

```cpp
// Load default configuration from robot module
auto stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
// Create the stabilizer task
auto t = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
          solver().robots(),
          solver().realRobots(),
          robotIndex,
          stabiConf.leftFootSurface,
          stabiConf.rightFootSurface,
          stabiConf.torsoBodyName,
          solver().dt());
// Reset the task targets and default configuration
t->reset();
// Apply stabilizer configuration (optional, if not provided the default configuration from the RobotModule will be used)
t->configure(stabiConf);
// Set contacts (optional, the stabilizer will be configured in double support using the current foot pose as target for each contact by default)
t->setContacts({ContactState::Left, ContactState::Right});
```

また、必要に応じて`mc_rtc::Configuration`オブジェクトから追加の設定と目標を読み込みます。

```cpp
// Load default configuration from robot module
auto stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
// mc_rtc::Configuration object containing valid stabilizer configuration (see JSON schema documentation)
auto conf = ...
// Optional: Load additional configuration from an mc_rtc::Configuration object
stabiConf.load(config);
// Create the stabilizer task
auto t = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
          solver().robots(),
          solver().realRobots(),
          robotIndex,
          stabiConf.leftFootSurface,
          stabiConf.rightFootSurface,
          stabiConf.torsoBodyName,
          solver().dt());
// Reset the task
t->reset();
// Apply stabilizer configuration
t->configure(stabiConf);
// Load additional properties from configuration (contact targets, com target, etc)
t->load(solver, config);
```

接触面と目標が指定されていない場合、デフォルトでは、両足支持の状態でタスクが起動し、制御対象ロボットの現在の足の姿勢と質量中心の位置を維持するように制御されます。`StabilizerTask::target(com, comd, comdd, zmp)`を使用することで、スタビライザーに目標を与えることができます（上記を参照）。

## スタビライザーを使用する（有限オートマトンを使用する場合）

先のセクションでは、スタビライザーの概要、スタビライザーの作成方法、スタビライザーのゲインの設定方法について説明しました。スタビライザーを使用するには、適切な接触面モード（左足支持、右足支持、両足支持）を選択し、有効な基準状態を与えます。また、スタビライザーをサポートする機能として、ほとんどの事例に対応可能な`FSM`機能が用意されています。

デフォルトの有限オートマトン状態`StabilizerStandingState`が用意されています。この状態を実行すると、設定に基づきスタビライザータスクが自動的に生成され、`CoM`と`Contacts`に関するシンプルな目標が与えられます。また、シンプルなばね・ダンパーモデルに基づき質量中心の移動経路が計算され、LIPMモデルに基づき振子の動的な基準状態が計算されます。この基準状態は、タイムステップごとにスタビライザーに与えられます。

`YAML`を使用して、以下のように状態を設定できます。

```yaml
##
# This state keeps the robot standing at it's current position in double support
##
Stabilizer::Standing:
  base: StabilizerStandingState
  # This stiffness controls the spring-damper computation of the reference CoM position
  # damping is automatically computed as 2*sqrt(stiffness)
  stiffness: 5
  # StabilizerTask configuration (see previous section)
  StabilizerConfig:
    type: lipm_stabilizer
    leftFootSurface: LeftFootCenter
    rightFootSurface: RightFootCenter
    enabled: true
    contacts: [Left, Right]
```

この状態を使用すると、質量中心の移動（または所定の位置への質量中心の固定）が非常に簡単に行えます。

```yaml
##
# Make the CoM move to a point centered above both of the robot's feet contact
#
# Completion:
# - OK when the dcm reaches the provided threshold
##
Stabilizer::GoCenter:
  base: Stabilizer::Standing
  above: Center
  completion:
    dcmEval: [0.005, 0.005, 0.05]

##
# Make the CoM move to a point above the left foot ankle
##
Stabilizer::GoLeft:
  base: Stabilizer::GoCenter
  above: LeftAnkle

##
# Make the CoM move to a point above the right foot ankle
##
Stabilizer::GoRight:
  base: Stabilizer::GoLeft
  above: RightAnkle
```


よく使われる動作に関する状態（上記の状態も含む）が定義されたシンプルなライブラリが、`mc_control/fsm/states/data/StabilizerStanding.yaml`に用意されています。これらの状態を他の状態と同時に実行することで、次のセクションで説明するようなより複雑な動作を行わせることができます。


## LIPMStabilizerを使用した有限オートマトンの例

スタビライザーを実際にお使いになる前に、サンプルコントローラー`LIPMStabilizer`を参照することを推奨します。このコントローラーをテストするには、`mc_rtc.yaml`設定ファイルに以下の記述を追加します。

```yaml
Enabled: LIPMStabilizer
```

そして、動力学的シミュレーションをサポートしているインターフェイス（`mc_vrep`や`choreonoid`など）を実行します。デフォルトの有限オートマトンでは、質量中心を左右に動かすことができます。また、以下の有限オートマトンが用意されています。

- `Stabilizer::Standing` : 現在観測されている位置に質量中心を固定します。この状態が実行されている間、GUIを使用して、`Left`（左足）のくるぶしと`Right`（右足）のくるぶしの間や他の位置に質量中心を動かすことができます。[Tasks（タスク）]の[Stabilizer（スタビライザー）]タブを開くと、スタビライザーのゲインの確認や変更が行えます。また、[Debug（デバッグ）]タブでは、スタビライザーの動作を可視化するライブプロットを追加できます。
- `AlternateFeetLifting` : 左右の足を交互に持ち上げます。この有限オートマトンでは、基本的に、`StabilizerStandingState`による準静的動作が行われます。まず、左足（または右足）のくるぶしの上に質量中心が移され、片足で立つようにスタビライザーが設定されます。それと同時に、右足（または左足）をスイングするタスクが実行されます（足を上げるときには`SurfaceTransform`が実行され、足を下ろすときには`AdmittanceTask`が実行されます）。
- `AlternateFeetLiftingManual`: 上記と同じ動作を行いますが、状態遷移はユーザーによってトリガーされます。これは、スタビライザーの動作を調整するのに便利です。
- `StepForward`: 準静的動作を使用して、前方`20cm`の地点に足を一歩踏み出します。実際のロボットで使用する場合、スイングする足の移動経路は単にスプライン曲線となり、スイング前後の衝撃に対する処理は行われないことに注意してください。
- `StepBackward`: 準静的動作を使用して、後方`10cm`の地点に足を一歩踏み出します。

## 他の有限オートマトン状態によるスタビライザーの操作

スタビライザーの状態には、[データストア](datastore.html)を使用して他の状態を設定するためのさまざまなコールバック関数が用意されています。以下の表に、利用可能なコールバックを示します。

<table class="table">
  <thead>
    <tr>
      <th scope="col">名前</th>
      <th scope="col">型</th>
      <th scope="col">説明</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>StabilizerStandingState::getCoMTarget</td>
      <td><pre>const Eigen::Vector3d & ()</pre></td>
      <td>スタビライザーによって使用される質量中心ターゲットを返す</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setCoMTarget</td>
      <td><pre>void (const Eigen::Vector3d &)</pre></td>
      <td>スタビライザーによって使用される質量中心ターゲットを変更する</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::getStiffness</td>
      <td><pre>double ()</pre></td>
      <td>質量中心の追従剛性を返す</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setStiffness</td>
      <td><pre>void (double)</pre></td>
      <td>質量中心の追従剛性を変更する。<br/>また、減衰値を2×(剛性の平方根)に設定する</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::getDamping</td>
      <td><pre>double ()</pre></td>
      <td>質量中心の追従減衰値を返す</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setDamping</td>
      <td><pre>void (double)</pre></td>
      <td>質量中心の追従減衰値を変更する</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setCoMWeight</td>
      <td><pre>void (double)</pre></td>
      <td>質量中心タスクの重みを変更する</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setCoMStiffness</td>
      <td><pre>void (const Eigen::Vector3d &)</pre></td>
      <td>質量中心タスクの剛性を変更する</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setPelvisWeight</td>
      <td><pre>void (double)</pre></td>
      <td>骨盤制御タスクの重みを変更する</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setPelvisStiffness</td>
      <td><pre>void (double)</pre></td>
      <td>骨盤制御タスクの剛性を変更する</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setTorsoWeight</td>
      <td><pre>void (double)</pre></td>
      <td>胴体制御タスクの重みを変更する</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setTorsoStiffness</td>
      <td><pre>void (double)</pre></td>
      <td>胴体制御タスクの剛性を変更する</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::getConfiguration</td>
      <td><pre>lipm_stabilizer::StabilizerConfiguration ()</pre></td>
      <td>スタビライザーの設定を返す</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setConfiguration</td>
      <td><pre>void (const lipm_stabilizer::StabilizerConfiguration &)</pre></td>
      <td>スタビライザーの設定を定義する</td>
    </tr>
  </tbody>
</table>

例:

```cpp
// Adds a GUI element to the controller instance ctl, providing the ability to read and modify the stiffness of the stabilizer
// This may for instance be used from another FSM state running in parallel with the StabilizerStandingState.
ctl.gui()->addElement({"DatastoreExample"},
    mc_rtc::gui::NumberInput("Stiffness",
      [&ctl]()
      {
        return ctl.datastore().call<double>("StabilizerStandingState::getStiffness");
      },
      [&ctl](double K)
      {
        ctl.datastore().call("StabilizerStandingState::setStiffness", K);
      }));
```
