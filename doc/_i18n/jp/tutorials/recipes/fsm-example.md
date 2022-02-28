このチュートリアルでは、[マルチロボットコントローラー]({{site.baseurl}}/tutorials/introduction/multi-robot-controller.html)のチュートリアルで実装したのと同じコントローラーを、有限オートマトンの設定のみを使用して実装します。このチュートリアルでは、フレームワークで用意されている`DoorSample`コントローラーをゼロから再現するのに必要な各手順について、ひとつずつ説明していきます。

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/GAtDC79G1zA" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

有限オートマトンをセットアップする
==

<div class="no_toc_section">

<ul class="nav nav-tabs" id="createTab" role="tablist">
  <li class="nav-item">
    <a class="nav-link active" id="cppCreateTab" data-toggle="tab" href="#cppCreateTabContent" role="tab" aria-controls="cppCreateTabContent" aria-selected="true">C++</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="pythonCreateTab" data-toggle="tab" href="#pythonCreateTabContent" role="tab" aria-controls="pythonCreateTabContent" aria-selected="false">Python</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="githubCreateTab" data-toggle="tab" href="#githubCreateTabContent" role="tab" aria-controls="githubCreateTabContent" aria-selected="false">GitHub</a>
  </li>
</ul>
<div class="tab-content" id="interfaceTabContent">
  <div class="tab-pane show active" id="cppCreateTabContent" role="tabpanel" arial-labelledby="cppCreateTab">
    <div class="card bg-light">
      <div class="card-body">
        {% translate_file tutorials/recipes/fsm-example/create-cpp.html %}
      </div>
    </div>
  </div>
  <div class="tab-pane" id="pythonCreateTabContent" role="tabpanel" arial-labelledby="pythonCreateTab">
    <div class="card bg-light">
      <div class="card-body">
        {% translate_file tutorials/recipes/fsm-example/create-python.html %}
      </div>
    </div>
  </div>
  <div class="tab-pane" id="githubCreateTabContent" role="tabpanel" arial-labelledby="githubCreateTab">
    <div class="card bg-light">
      <div class="card-body">
        <p><a href="https://github.com/mc-rtc/new-fsm-controller">mc-rtc/new-fsm-controller</a>をテンプレートとして使用して下さい。これは<code>mc_rtc_new_fsm_controller</code> ツールを用いて生成されたコードとほぼ同じです。</p>
      </div>
    </div>
  </div>
</div>

</div>

JVRC1ロボットがRViz内で立っているのが分かると思います。それでは、[マルチロボットコントローラー]({{site.baseurl}}/tutorials/introduction/multi-robot-controller.html)のチュートリアルで実装したのと同じコントローラーを、有限オートマトンの機能を使って実装してみましょう。それには、有限オートマトンの設定`etc/MyFirstFSMController.yaml`を編集する必要があります。

追加のロボットを読み込む
==

まず、メインロボット以外にどのロボットをこのコントローラーで使用するかを宣言します。本フレームワークでは、デフォルトでさまざまなロボットと環境が用意されており、ロボットの別名を使って簡単に読み込むことができます。ロボットの別名は、各ロボットを短い名前で表したり、ロボットを読み込むのに必要な情報（ロボット記述パッケージへのパスなど）をmc_rtcに渡すのに使用できます。コントローラーを起動すると、利用可能なロボットの別名の全リストが見られます。あるいは、<code>/usr/local/lib/mc_robots/aliases/</code>でロボットの別名を確認することもできます。今回のコントローラーでは、地面を表す固定の平面を`env/ground`として指定し、ハンドルを備えたドアを表す多関節ロボットを`env/door`として指定します。独自の環境を作成する方法について詳しくは、[環境作成のチュートリアル]({{site.baseurl}}/tutorials/advanced/new-environment.html)を参照してください。

```yaml
robots:
  ground:
    module: env/ground
  door:
    module: env/door
    init_pos:
      translation: [0.70, 0.5, 0.0]
      rotation: [0.0, 0.0, 1.57]
```

グローバルな接触面と制約条件を追加する
==

制約条件
===

有限オートマトンにグローバルな制約条件を追加できます。フレームワークで用意されている制約条件の詳細については、[ConstraintSet JSONスキーマ]({{site.baseurl}}/json-full.html#ConstraintSet/ContactConstraint)のドキュメントを参照してください。独自の制約条件を宣言してここに読み込むこともできます。

```yaml
constraints:
- type: contact
- type: dynamics
  damper: [0.1, 0.01, 0.5]
- type: compoundJoint
```

ここでは3つの制約条件を宣言します。
- `contact`: {% doxygen mc_solver::ContactConstraint %}を追加します。これは、接触面の位置を固定するとともに、生成された力が摩擦円錐の内部に収まるように制限します（力学モードのみ）。
- `dynamics`: {% doxygen mc_solver::DynamicsConstraint %}を追加します。これは、キネマティクスの制約条件と関節可動範囲の制約条件を適用するとともに、関節のトルクを計算します。
- `compoundJoint`: {% doxygen mc_solver::CompoundJointConstraint %}を追加します。これは、相互に関連している関節の可動範囲を処理します（例えば、くるぶしの関節のロール方向の可動範囲は、現在のヨー角度によって決まります）。


接触面
===

次に、接触面の初期セットをコントローラーに追加する方法について見ていきましょう。なお、状態を使用して接触面を後で追加・削除できます。`contacts`要素は、接触面のプロパティが記述された[Contact]({{site.baseurl}}/json-full.html#mc_rbdyn/Contact)の配列で構成されます。この要素が、上で定義した`contact`制約条件に追加されます。

```yaml
# 接触集合の初期値
contacts:
- r1: jvrc1
  r1Surface: LeftFoot
  r2: ground
  r2Surface: AllGround
- r1: jvrc1
  r1Surface: RightFoot
  r2: ground
  r2Surface: AllGround
```

この場合、左足と右足の裏が地面と接触しているとみなされます。また、足の裏が動かないように制限され、二次計画法によって生成された力が線形化された摩擦円錐の内部に収まるように制限されています。

衝突メッシュ
===

次に、衝突メッシュの初期セットを追加する方法について見ていきましょう。接触面の場合と同様に、状態を使用して衝突メッシュを後で追加・削除できます。`collisions`要素は、[CollisionConstraint]({{site.baseurl}}/json-full.html#ConstraintSet/CollisionsConstraint)オブジェクトの配列で構成されます。`MainRobot`については、自己衝突メッシュのデフォルトのリストが{% doxygen mc_rbdyn::RobotModule %}で定義されており、それらをここで使用できます。

```yaml
# 干渉回避制約
collisions:
- type: collision
  useMinimal: true  # 最小限の自己干渉回避セットはロボットモジュールに定義されている
- type: collision
  r1: jvrc1
  r2: door
  collisions: # このロボットの組み合わせに対する干渉回避拘束のリスト
    - body1: L_WRIST_Y_S
      body2: door
      iDist: 0.5  # インタラクション距離：物体間の距離がこの値を下回ると拘束が有効になる
      sDist: 0.02 # 安全距離：物体間の距離がこの値以下にならないように拘束される
      damping: 0.0
```

有限オートマトンの状態を作成する
===

このセクションでは、ドアを開けるのに必要な状態を定義する方法について見ていきます。ここでは、本フレームワークで用意されている`C++`状態を最大限活用し、コードを1行も書かずに複雑な動作を行わせる方法について説明します。このコントローラーでは、以下に示す（非常に一般的な）状態を使用します。

- `MetaTasks`: 設定からタスクを読み込んでソルバーに追加します。また、タスクが完了したかどうかをチェックします。
  - API: {% doxygen mc_control::fsm::MetaTasksState %}
  - YAML: [JSONスキーマのドキュメント]({{site.baseurl}}/json.html#State/MetaTasks)
- `Parallel`: 複数の状態を実行する
  - API: {% doxygen mc_control::fsm::ParallelState %}
  - YAML: [JSONスキーマのドキュメント]({{site.baseurl}}/json.html#State/Parallel)
- `Meta`: 状態内に有限オートマトンを作成する
  - API: {% doxygen mc_control::fsm::MetaState %}
  - YAML: [JSONスキーマのドキュメント]({{site.baseurl}}/json.html#State/Meta)
- `Posture`: グローバルな姿勢制御タスクのゲインと目標を処理します。
  - API: {% doxygen mc_control::fsm::PostureState %}
  - YAML: [JSONスキーマのドキュメント]({{site.baseurl}}/json.html#State/Posture)

状態は、設定の`states`セクションで宣言されています。または、状態の宣言が記述された`.yaml`ファイルを`src/states/data`に作成することもできます。これは、大規模な有限オートマトンの場合に特に便利です。

ここでは、以下のように有限オートマトンを構成します。
1. `Door_Initial`: 「ドアを開く」動作をトリガーするボタンを追加するシンプルなC++状態
2. `Door::OpenDoorFSM`: 手を動かし、ハンドルを開き、ドアを動かすロジックのみで構成される下位の有限オートマトン
3. `Door::Standing`: `CoM`タスクと胸部を垂直に保つための正規化タスクを追加する状態
4. `Door::OpenDoorDemo`: ドアを開けるときに質量中心と胸部の目標が同時に処理されるように、状態`2`と`3`を組み合わせます。

ここでは、マルチロボットの概念を最大限活用します。すなわち、ハンドルに手を接触させ、ハンドルの関節角度とドアの蝶番の関節角度を制御することで、ドアを動かしますこの動きを実現するのに必要な動作は、接触面の制約条件に基づき二次計画法によって自動的に生成されます。

初期状態: 状態遷移をトリガーするボタンを追加する
====

この状態は、以下の内容に関する簡単な例を説明することを目的としています。

1. C++状態を作成する方法
2. 状態内にGUI要素を追加する方法
3. 状態内から状態遷移の流れを制御する方法

この状態は`src/states/Door_Initial.cpp`で定義されており、この処理で必要とされる仮想関数`configure`、`start`、`run`と関数`teardown`を置き換えます（{% doxygen mc_control::fsm::State %}のAPIのドキュメントを参照）。ここでは、状態開始時にGUIにボタンを追加します。そして、このボタンがクリックされたときに、bool型の状態`openDoor_`を変化させます（後で「OpenDoor」状態に遷移するのに使用します）。これを実現するには、`output("OpenDoor")`を呼び出し、`bool Door_Initial::run`関数で`true`を返します。これは、この状態の処理が完了して次の状態に遷移できることを表します。なお、この状態遷移がどのように起こるかは、状態遷移マップの設定によって定義されています（{% doxygen mc_control::fsm::TransitionMap %}を参照）。

```cpp
#include <mc_control/fsm/Controller.h>

#include "Door_Initial.h"

void Door_Initial::configure(const mc_rtc::Configuration &) {}

void Door_Initial::start(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->addElement({}, mc_rtc::gui::Button("Open door", [this]() { openDoor_ = true; }));
}

bool Door_Initial::run(mc_control::fsm::Controller &)
{
  if(openDoor_)
  {
    output("OpenDoor");
    return true;
  }
  return false;
}

void Door_Initial::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("Door_Initial", Door_Initial)
```

EXPORT_SINGLE_STATE`マクロを使って、mc_rtcがライブラリから状態を読み込む際に使用されるシンボルを宣言しています。また、ここで指定した`"Door_Initial"`という名前は、有限オートマトン内でこの状態を識別するのに使用します。

なお、ここでは、この状態をあえて非常にシンプルにしています。実際には、状態内ではもっと複雑な処理が行われます。例えば、タスクの追加と目標の処理、プランナーからのデータの読み取り、ロボットの状態の監視、状態遷移のトリガーといった処理が行われます。そのような状態の例として、今回のサンプルで使用されている`MetaTasks`タスク、`Parallel`タスク、`Meta`タスク、`Posture`タスクがあります。

ドアを開けるための有限オートマトン
====

ドアを開けるには、下位の有限オートマトン、すなわち、ドアのハンドルに向かって手を動かし、ドアのハンドルに手を接触させ、ドアのハンドルと蝶番を回してドアを開ける処理を行う有限オートマトンを状態内で実行します。

まず、この動作の状態遷移マップを見てみましょう。

```yaml
Door::OpenDoorFSM:
  base: Meta
  transitions:
    - [Door_Initial, OpenDoor, Door::ReachHandle, Auto]
    - [Door::ReachHandle, OK, Door::MoveHandle, Auto]
    - [Door::MoveHandle, OK, Door::OpenDoor, Auto]
```

ユーザーがGUIのボタンをクリックすると、上記の`Door_Initial`というC++状態によって`OpenDoor`という状態遷移がトリガーされます。すると、有限オートマトンは次の状態である`Door::ReachHandle`に遷移します。

```yaml
Door::ReachHandle:
  base: MetaTasks
  tasks:
    RightHandTrajectory:
      type: surfaceTransform
      surface: RightGripper
      weight: 1000
      stiffness: 5
      # Target relative to the door's handle surface
      targetSurface:
        robot: door
        surface: Handle
        offset_translation: [0, 0, -0.025]
        offset_rotation: [0, 0, 0]
      completion:
        AND:
          - eval: 0.05
          - speed: 1e-4
```

この状態では、フレームワークで用意されている{% doxygen mc_control::fsm::MetaTasksState %}というC++状態が使用され、YAMLの記述から一連のタスクが読み込まれます。ここでは、`RightHandTrajectory`という名前の`surfaceTransform`型（[YAMLのドキュメントを参照]({{site.baseurl}}/json.html#MetaTask/SurfaceTransformTask)）のタスクが読み込まれます。これにより、{% doxygen mc_tasks::SurfaceTransformTask %}がソルバーに追加され、ドアのハンドル表面を基準として定義された目標がこのタスクに設定されます。また、`completion`要素によって{% doxygen mc_control::CompletionCriteria %}が作成され、タスクの実行が完了したとみなしてよいかどうかをチェックする論理関数が構築されます。デフォルトでは、タスクの完了基準が満たされると`MetaTasks`状態によって`"OK"`が出力されます。

すると、`OpenDoorFSM`は次の状態である`[Door::ReachHandle, OK, Door::MoveHandle, Auto]`に遷移します。

```yaml
Door::MoveHandle:
  base: Posture
  robot: door
  completion:
    eval: 0.01
  postureTask:
    weight: 100
    jointGains:
      - jointName: handle
        stiffness: 50
    target:
      handle: [-1.0]
  AddContacts:
  - r1: jvrc1
    r1Surface: RightGripper
    r2: door
    r2Surface: Handle
```

この状態は、{% doxygen mc_control::fsm::PostureState %}（[JSONのドキュメントを参照]({{site.baseurl}}/json.html#States/Posture)）がベースとなっています。この状態では、フレームワークによって各ロボットに自動的に追加されたグローバルな姿勢制御タスクのゲインと目標が変更されます。まず、二次計画法によってロボットの`RightGripper`とドアの`Handle`の表面の相対位置がずれるのを防ぐため、これらの表面を接触させます。また、ロボットとドアを相互作用させる動的な力を計算するための摩擦円錐制約条件を追加します。これにより、ハンドルの関節が回転すると、JVRC1ロボットの関節も回転するようになります。

この状態の処理が完了したら、次の状態である`[Door::MoveHandle, OK, Door::OpenDoor, Auto]`に遷移します。この状態は、先の状態と非常によく似ています。この状態では、ロボットがドアを開けられるように、ドアの蝶番の目標関節角度を変更します。この状態が先の状態（`base: Door::MoveHandle`）をどのように継承しているかに注目してください。この状態は、新しい目標を再定義している点だけが先の状態と異なります。

```yaml
Door::OpenDoor:
  base: Door::MoveHandle
  postureTask:
    jointGains:
      - jointName: handle
        stiffness: 50
      - jointName: door
        stiffness: 50
    target:
      handle: [-1.0]
      door: [-0.3]
```


現時点では、この有限オートマトンはドアを開く動作のみが考慮されています。ロボットのバランスについては考慮されていません。それでは、左足と右足の中点の真上に質量中心を置いてみましょう。これを実現するには、`Door::Standing`状態を前述の`Door::OpenDoorFSM`と同じ場所に置きます。厳密にいうと、これらの状態はタイムステップごとに交互に実行されます。.

```yaml
Door::OpenDoorDemo:
  base: Parallel
  states: [Door::Standing, Door::OpenDoorFSM]
```

立位状態は以下のように定義されます。

```yaml
Door::Standing:
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

このチュートリアルの完全なソースは、[こちら](https://github.com/jrl-umi3218/mc_rtc/tree/master/src/mc_control/samples/Door)から入手できます。

まとめ
===

このチュートリアルでは、有限オートマトンをゼロから作成する方法のほか、本フレームワークで用意されている主要な有限オートマトン状態と、マルチロボットでタスク空間を制御するという概念を用いて、やや複雑なマルチロボットの動作を実現する方法について見てきました。ここではYAMLの機能を使用しましたが、YAMLの使用は必須ではありません。本フレームワークでは、独自の状態を記述してより複雑な動作の定義と抽象化を簡単に行えます。

以下も参照してください。
- 同様の有限オートマトンに力の制御も加えた[アドミッタンス制御のサンプルチュートリアル]({{site.baseurl}}/tutorials/samples/sample-admittance.html)
