このチュートリアルでは、本フレームワークで用意されている`AdmittanceSample`サンプルコントローラーの使い方と概念について説明します。このサンプルでは、{% doxygen mc_tasks::force::AdmittanceTask %}を使用して、壁を押しながら`JVRC1`の手に加わる力を調節する方法を示します。

このコントローラーでは、以下の手順で処理を実行します。

1. 壁の前に手を移動させる
2. `AdmittanceTask`を使用して、手と壁を接触させる
3. 法線方向の力を`20N`に調節する
4. キネマティクスに基づいて壁の位置を更新する
5. 力を弱めて手を離す準備をする
6. 手を後ろに引く
7. （手と壁が衝突しないように制御しながら）中腰の姿勢に戻す

## コントローラーを実行する

このサンプルのソースは[src/mc_control/samples/Admittance](https://github.com/jrl-umi3218/mc_rtc/tree/master/src/mc_control/samples/Admittance)にあります。ここには、主に以下のファイルが格納されています。
- [etc/AdmittanceSample.in.yaml](https://github.com/jrl-umi3218/mc_rtc/tree/master/src/mc_control/samples/Admittance/etc/AdmittanceSample.in.yaml): YAMLで記述された有限オートマトンの設定（こちらの[チュートリアル]({{site.baseurl}}/tutorials/recipes/fsm.html)を参照）
- [src/states/UpdateWall.h](https://github.com/jrl-umi3218/mc_rtc/tree/master/src/mc_control/samples/Admittance/src/states/UpdateWall.h), [src/states/UpdateWall.cpp](https://github.com/jrl-umi3218/mc_rtc/tree/master/src/mc_control/samples/Admittance/src/states/UpdateWall.cpp): ロボットのグリッパーの位置に基づいて壁の位置を更新するC++状態

このサンプルを実行するには、力覚センサーをシミュレート可能な動力学的シミュレーターが必要です。このチュートリアルは、{% link mc_openrtm %}、{% link Choreonoid %}、それに本フレームワークで用意されたシミュレーションファイル`sim_mc_wall.cnoid`を使用することを前提としています。他のシミュレーターを使用する場合は、必要に応じて命令を変更し、ロボットと壁が`55cm`離れて配置されているシーンを作成してください。

まず、以下の`~/.config/mc_rtc/mc_rtc.yaml`ファイルを用意します。:

```yaml
MainRobot: JVRC1
Enabled: AdmittanceSample
Timestep: 0.005
```

次に、以下のようにサンプルを実行します。

```sh
$ (roscore &) # roscoreが走っていることを確認 (rvizを用いた可視化のため)
$ cd /usr/local/share/hrpsys/samples/JVRC1
$ choreonoid sim_mc_wall.cnoid
$ roslaunch mc_rtc_ticker controler_display.launch
```

次に、Choreonoidのインターフェイスにある緑の矢印をクリックしてシミュレーションを開始します。出力は以下のようになります。

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/B_L_xPynhvU" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>


グリッパーの`z`軸に沿って力がどのように加えられたかを以下のグラフに示します。`t=6s`の場所にあるスパイクは、グリッパーと壁が衝突したタイミングに対応します。その`1s`後、力の目標値が`-20N`に変更され、その後数秒間この状態が維持されます。その後、圧力が解放されます。

<img src="{{site.baseurl_root}}/assets/tutorials/samples/img/admittance.svg" alt="admittance results" class="img-fluid" />

## 説明

このサンプルのほとんどは、有限オートマトンの設定`etc/AdmittanceSample.in.yaml`で状態とタスクの目標を記述することにより実現されています。まず、状態遷移マップを見てみましょう。ここには、状態をどのように遷移させるかが記述されています。

```yaml
transitions:
- [RightHandToWall,            OK, RightHandPushAdmittance, Auto]
- [RightHandPushAdmittance,    OK, RightHandKeepAdmittance, Auto]
- [RightHandKeepAdmittance,    OK, UpdateWallFromKinematics, Auto]
- [UpdateWallFromKinematics,   OK, RightHandReleaseAdmittance, Auto]
- [RightHandReleaseAdmittance, OK, RightHandMoveBack, Auto]
- [RightHandMoveBack,          OK, StandingHalfSitting, Auto]

# 初期状態
init: RightHandToWall 
```

この状態遷移マップを見れば、このサンプルの手順を簡単に理解できます。次に、主な状態が何を実行するかを見てみましょう。

### StandingBase: 質量中心と胸部を管理する

この実験全体を通じて、ロボットの質量中心を両足の中点の真上に維持します。これを実現するには、{% doxygen mc_tasks::CoMTask %}を使用します。また、胸部の過度な動きを抑えてより人間らしい動きとなるように、ロボットの胸部に{% doxygen mc_tasks::OrientationTask %}を追加します。この状態は、この有限オートマトン内にある他のすべての状態の基準状態として使用され、すべての状態がこの状態の動作を継承します。

```yaml
  # 重心位置を足の中間に、胴体をまっすぐに保つ
  StandingBase:
    base: MetaTasks
    tasks:
      CoM:
        type: com
        above: [LeftFoot, RightFoot]
        weight: 2000
        stiffness: 5
      KeepChest:
        type: orientation
        body: WAIST_R_S
        weight: 100
        stiffness: 1
```

### RightHandToWall: 手の移動経路

次に、ロボットの手を壁の近くまで前に動かします。これを行うには、{% doxygen mc_tasks::BSplineTrajectoryTask %}を使用します。このタスクでは、中間点の情報に基づいて、位置と向きがパラメーター化されたBスプライン曲線の移動経路が出力されます。このタスクでは、指定されたロボットの表面がこの移動経路に沿って制御されます。

```yaml
  # 手先を壁近くに移動させる軌道
  RightHandToWall:
    base: StandingBase
    tasks:
      # StandingBase状態からCoMとKeepChestタスクを継承し
      # bspline_trajectoryタスクを追加
      RightHandTrajectory:
        type: bspline_trajectory
        surface: RightGripper
        weight: 1000
        stiffness: 50
        duration: 4
        dimWeight: [1,1,1, 1, 0.5, 0.5]
        displaySamples: 100
        target:
          translation: [0.45, -0.4, 1.1]
          rotation: [1.57, 0, 1.57]
        controlPoints: [[0.17, -0.5, 0.85]]
        completion:
          timeElapsed: true
```

### Admittance: 力の制御

次に、`RightHandPushAdmittance`状態において、{% doxygen mc_tasks::AdmittanceTask %}を使用して手の法線方向の力を調節し、手と壁を接触させます。アドミッタンス制御タスクでは、レンチの測定値（フォーストルクセンサーから得られた測定値）と目標値との誤差に基づいて、エンドエフェクターの目標移動速度が計算されます。係数`admittance`によって、移動方向の軸に沿った誤差の大きさに基づきエンドエフェクターの移動速度が調節されます。なお、このタスクに速度を追跡させるには、力を追跡させたい方向に沿って、位置制御の剛性を低くし、速度制御の減衰を高くする必要があります。また、すべてのレンチは表面フレーム内で表されます（今回の例では`RightGripper`表面フレーム）。

```yaml
  # 20Nに達するまで作用させる力を増大させる
  RightHandPushAdmittance:
    base: StandingBase
    tasks:
      RightHandAdmittance:
        type: admittance
        surface: RightGripper 
        # グリッパサーフェスの法線方向の力をアドミッタンス係数0.001で追従させる
        # 注意：この軸方向のダンピングは高く、剛性は低い
        # 他の軸は位置制御されており現在のグリッパの位置を可能な限り維持する
        admittance: [0,0,0,0,0,0.001]
        stiffness: [10, 10, 10, 10, 10, 1]
        damping: [6.3, 6.3, 6.3, 6.3, 6.3, 300]
        maxVel:
          linear: [0.2, 0.2, 0.5]
          angular: [0.2, 0.2, 0.2]
        # グリッパサーフェスのz軸方向の力の目標値を20Nに設定
        wrench:
         force: [0, 0, -20]
         couple: [0, 0, 0]
        # グリッパが壁の方向を向くように回転
        targetRotation: [1.57, 0, 1.57]
        # このタスクは-20Nの力が達成されたら終了する。他の全ての軸は無視される。
        completion:
          wrench:
            force: [.nan, .nan, -20]
            couple: [.nan, .nan, .nan]
```

次の`RightHandKeepAdmittance`状態では、ParallelState（{% doxygen mc_control::fsm::ParallelState %}）を使用して、先の状態を所定の時間だけ実行します。さらに、この状態を実行しつつ、壁に加わる力が一定となるように制御します。そのため、ロボットを前に押すと、手が後ろに動いて壁に加わる力が減少し、ロボットを後ろに引くと、それと反対の動作が行われます。

### キネマティクスに基づいて壁の位置を更新する

上記のアドミッタンス制御タスクでは、力が目標値に達するまでグリッパーを前に動かします。このコントローラーでは、壁の位置が以下のように定義されていると仮定しています。

```yaml
robots:
  wall:
    module: env/ground # 地面のモデルを再利用するが垂直方向に回転して壁を模擬
    init_pos:
      # 壁の位置はChoreonoid上の位置よりも5cm前
      translation: [0.50, 0.0, 0.0]
      rotation: [0.0, -1.57, 0.0]
```

ただし、ロボットの動作開始位置が想定された開始位置とわずかに異なる場合があるため、想定された壁の位置が実際の壁の位置と異なる場合があります。しかし、先のアドミッタンス制御状態を使用することで、ロボットのグリッパーが壁に到達したかどうかがわかります。この情報に加えて、ロボットのキネマティクス情報を使用して（さらには、接触面が滑らないものと仮定して）、今回のモデルにおける壁の位置を更新します。これは、C++で実装された`UpdateWall`状態で実行します。この状態は、ロボットの指先の現在の推定位置を読み取り、指先が壁に触れるように壁の位置を更新します。

なお、この状態では、{% doxygen mc_observers::KinematicInertialObserver %}によって観測されたロボットの状態を使用して、実際のロボットにおける実際の手の位置が判定されます（[観測器のチュートリアル]({{site.baseurl}}/tutorials/recipes/observers.html)を参照）。

```cpp
void UpdateWall::start(mc_control::fsm::Controller & ctl)
{
  // [...] YAML 設定パラメータを読み込む

  // JVRC1の推定された指のポーズを取得
  // ステートオブザベーションパイプラインで推定されたもの
  const auto & bodyPose = ctl.realRobots().robot(rName).bodyPosW(bName);
  // 壁の位置を指先の位置と合うようにx軸方向に移動する
  auto posW = ctl.robot(moveRobotName).posW();
  posW.translation().x() = bodyPose.translation().x();
  ctl.robot(moveRobotName).posW(posW);
}
```

次の状態では、更新された壁の位置を使用して、手と壁が衝突しないように制御しながら最初の姿勢に戻します。

### 最初の姿勢に戻す

この有限オートマトンの残りの部分では、ロボットを最初の姿勢に戻します。まず、スムーズに遷移できるように、`RightHandReleaseAdmittance`を使用して手の圧力を解放し、手の圧力をほぼ0にします。次に、現在の位置から`10cm`後方に手を動かします。最後に、{% doxygen mc_control::fsm::HalfSittingState %}状態を使用して、最初の中腰の姿勢に戻します。今回は手の動きを明示的に指定していないため、中腰の姿勢に戻るまでに手と壁が衝突するおそれがあります。それを防ぐため、手と壁との間に衝突制約条件を追加します。

```cpp
  RightHandMoveBack:
    base: StandingBase
    AddCollisionsAfter:
      - r1: jvrc1
        r2: wall
        collisions:
          - body1: R_WRIST_Y_S
            body2: ground
            iDist: 0.15
            sDist: 0.05
            damping: 0.0
    tasks:
    # [...]
```

## 参考

線形倒立振子モデルと外力補正に基づく安定化制御のより高度な例については、[ExternalForces]({{site.baseurl}}/tutorials/samples/list-of-samples.html#externalforces)サンプルコントローラーを参照してください。
