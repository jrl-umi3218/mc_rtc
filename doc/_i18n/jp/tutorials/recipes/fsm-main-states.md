本フレームワークでは、`JSON/YAML`の設定機能を最大限活用して有限オートマトンを簡単に構成できるように、実装済みの状態がデフォルトでいくつか用意されています。これらの状態を使用すると、ロボットの複雑な動作を実現する有限オートマトンを簡単に記述でき、`C++/Python`状態を手作業で記述する必要はほとんど（または全く）ありません。実現したいことのほとんどすべてを有限オートマトンの設定で直接記述できます。このチュートリアルでは、最も役に立つ3つの状態について詳しく見ていきます。

- [MetaTasks]({{site.baseurl}}/json.html#State/MetaTasks): `JSON/YAML`で記述された設定から一連のタスクを読み込む
- [Parallel]({{site.baseurl}}/json.html#State/Parallel): 複数の状態を「同時に」実行する
- [Meta]({{site.baseurl}}/json.html#State/Meta): 有限オートマトン内に有限オートマトンをネストさせる

このチュートリアルの残りの部分では、これらの状態の使い方について見ていきます。ここでは`YAML`の例のみが示されていますが、もちろん`JSON`もサポートされています。類似の例についてはドキュメントを参照してください。利用可能な状態の詳細については、JSON/YAMLのドキュメントの[状態オブジェクト]({{site.baseurl}}/json.html#State)のセクションを参照してください。.

## MetaTasks状態

`MetaTasks`状態（[こちらのドキュメント]({{site.baseurl}}/json.html#State/MetaTasks)を参照)を使用すると、`JSON/YAML`で記述された設定から一連のタスクを読み込むことができます。利用可能なタスクの詳細については、[JSON/YAMLのドキュメント]({{site.baseurl}}/json.html#MetaTask)を参照してください。

それでは例を見てみましょう。ここでは、`FSM`の設定の`states:`セクションに新しい`MetaTasks`状態を追加します。

```yaml
states:
  ExampleState:
    base: MetaTasks
    # 任意：タスクの出力を次の遷移に対する文字列出力として用いる
    outputs: [CoM]
    tasks:
      # CoMタスクの生成
      CoM:
        type: com
        move_com: [0, 0, -0.05]
        completion:
          OR:
            - timeout: 3
            - AND:
              - eval: 0.01
              - speed: 0.005

      # bspline_trajectoryタスクを生成する
      HandTrajectory:
        type: bspline_trajectory
        surface: LeftHand
        stiffness: 10000.0
        duration: 15.0
        weight: 100
        targetSurface:
          # 追加のオブジェクトである、boxロボットはFSMのグローバル設定で読み込まれていると仮定
          robot: box
          surface: Left
        completion:
          - timeElapsed: true
```

`ExampleState`という名前の新しい状態が作成され、以下のものが読み込まれます。
- 質量中心を`5cm`下げる`CoM`という名前の`CoM`タスク
- ロボットの左のグリッパーを他のロボットで定義された`Left`という表面まで動かす`LeftHandTrajectory`という名前の`BSplineTrajectory`タスク

### 完了基準

上記の例の`completion`要素に注目してください。「完了基準」と呼ばれるもので、タスクが完了したかどうかを判断する条件文を生成します。詳細については、{% doxygen mc_control::CompletionCriteria %}のAPIのドキュメントを参照してください。

デフォルトでは、各タスクに以下のエントリが定義されています。

- `eval`: MetaTask::eval()のノルムが指定された値より低いときにtrueになる
- `speed`: MetaTask::speed()のノルムが指定された値より低いときにtrueになる
- `timeout`: 指定された時間（単位: 秒）より長い時間にわたりタスクがソルバーに残っているときにtrueになる

タスクによっては、以下のような追加の完了基準が定義されています。

- `SplineTrajectoryTask`のバリエーション（`bspline_trajectory`、`exact_cubic_trajectory`）では、以下の基準が追加で定義されています。
  - `timeElapsed: true`: タスクの`duration`が経過したときにtrueになる
  - `wrench`: ロボットの表面に加わった力が指定されたしきい値より大きいときにtrueになる（6次元のベクトル。値がNaNの場合は測定値が無視される。値が負の場合は条件が反転する）。表面に力覚センサーが取り付けられていない場合は無視される

タスクに`mc_task::MetaTask::buildCompletionCriteria`を実装することで、独自の完了基準を追加できます。また、下記のように、条件構成体`AND`と`OR`を使用してこれらの完了基準を組み合わせることができます。さらに、通常の遅延評価が適用されます。例えば、トータルの処理時間が規定の時間を超えたとき、あるいは3秒の遅延後に手の表面の法線方向（`z`方向）に15Nを超える力が加わったときに手の移動経路タスクが完了しているかチェックするには、以下のように記述します。

```yaml
# タスクが設定された時間以上アクティブである場合、または15N以上の力がハンドサーフェスの`z`方向に3秒以上の遅延の後に作用した場合を終了と判定する
completion:
  OR:
    - timeElapsed: true
    - AND:
      - timeout: 3
      - wrench: [.NaN, .NaN, .NaN, .NaN, .NaN, 15]
```

完了したかどうかを判断する必要がないタスクについては、完了基準`completion`を省略できます。その場合、有限オートマトンはそのタスクが必ず完了するものとみなします。

### 状態出力

先のチュートリアルで述べたように、各状態は`State::output("output value")`を呼び出して出力文字列を返します。この出力値は、状態遷移マップで次にどの状態を実行すべきかを判断するのに使用されます。`MetaTasks`状態は、状態内のすべてのタスクが完了すると、デフォルトで`"OK"`を返します。タスクの出力に基づいて分岐を判断する必要がない場合、以下のように状態遷移マップを記述します。

```yaml
transtions:
...
# 次の状態への遷移は全てのタスクが完了した時に起きる
- [ExampleState, OK, NextState, Auto]
...
```

一方、より複雑な分岐判断、すなわち、タスクが完了した理由に基づく分岐判断を行いたい場合があります。その場合、`MetaTasks`状態の`output`設定で、完了基準の文字列出力に基づき状態出力を生成させたいタスクの名前のリストを記述します。例:

```yaml
ExampleState:
  base: MetaTasks
  # 任意：タスクの出力を次の遷移に対する文字列出力として用いる
  outputs: [CoM]
```

そうすると、状態遷移マップで`CoM`完了基準に基づき分岐が判断されるようになります。

```yaml
# CoMの終了判定は以下のように定義されている
# CoM:
#   completion:
#     OR:
#       - timeout: 3
#       - AND:
#         - eval: 0.01
#         - speed: 0.005
- [ExampleState, "CoM=timeout", CoMHasNotConvergedState, Auto]
- [ExampleState, "CoM=eval AND speed", NextMotionState, Auto]
# 完了パターンが状態遷移マップのいずれも当てはまらない場合に実行する状態を定義
# これによって全てのパターンを定義する必要がなくなる
- [ExampleState, "DEFAULT", DefaultState, Auto]
```

この例の場合、以下に示す3つの新しい状態を定義します。
- `CoMHasNotConvergedState`: 規定時間内に質量中心タスクが収束しなかった場合に実行される
- `NextMotionState`: 質量中心タスクが収束した場合に実行される
- `DefaultState`: 生成された出力パターンに一致する状態遷移が存在しない場合に実行される。この例では、出力パターンは必ずいずれかの状態遷移と一致するため、この状態が実行されることはありません。

## Parallel状態

Parallel状態（[こちらのドキュメント]({{site.baseurl}}/json.html#State/Parallel)を参照）では、複数の状態を同時に実行させることができます。有限オートマトンは実際にはシングルスレッドで実行されるため、実際にはコントローラーの処理ループ内で各状態がシーケンシャルに実行されます。

```yaml
# 右手を動かすための追加のMetaTasks状態を作成
RightHandState:
  base: MetaTasks
  HandTrajectory:
    type: bspline_trajectory
    surface: RightHand
    stiffness: 10000.0
    duration: 15.0
    weight: 100
    targetSurface:
      robot: box
      surface: Left
    completion:
      - timeElapsed: true

# 例えば右手、左手と重心の下方向への移動を行いたい場合、
# 2つの状態を並列実行すればよい
ExampleParallelState:
  base: Parallel
  # 実行周期毎にまず ExampleState が実行され、続いて RightHandState が実行される
  states: [ExampleState, RightHandState]
  # どちらの状態を出力判定に用いるか指定することも可能
  # デフォルトでは最後の状態の出力が使用される
  outputs: [ExampleState, RightHandState]
```

Parallel状態は、同時に実行されているすべての状態が完了したときに完了したとみなされます。また、`MetaTasks`と同様に、状態出力をどのように生成するかを指定できます。
- デフォルトでは、`states`リスト内の最後の状態の出力が使用されます。
- `outputs`が指定されている場合、


例:

```yaml
[ExampleParallelState, "ExampleState: (CoM=timeout), RightHandState: (timeElapsed)", "StateA"]
[ExampleParallelState, "ExampleState: (CoM=eval AND speed), RightHandState: (timeElapsed)", "StateB"]
# 複雑な状態遷移が起こりうる場合に、全ての出力の組み合わせを記述することは大変です。
# このような場合に特にデフォルト状態の利用は有効です
[ExampleParallelState, "DEFAULT", "DefaultState"]
```

### 同時に実行されている状態間で情報をやり取りする

複数の状態間で情報をやり取りすると便利な場合があります。例えば、床の上の物を拾う処理を行う`Pickup`状態が`StabilizerStandingState`と同時に実行されている場合を考えます。この場合、ロボットが床の上の物を拾えるように、このような場合、状態間で情報をやり取りするには、[データストア]({{site.baseurl}}/tutorials/recipes/datastore.html)を使用します。

## Meta: 有限オートマトン内の有限オートマトン

複雑なシナリオでは、複数の有限オートマトンを同じコントローラーに実装すると便利な場合がよくあります。そのような場合のために、`Meta`状態が用意されています（[こちらのドキュメント]({{site.baseurl}}/json.html#State/Meta)を参照）。この状態を使用すると、

```yaml
ExampleMetaState:
  base: Meta
  transitions:
  - [StateA, OK, StateB]
  - [StateB, OK, StateC]
  - ...
```

そして、この状態を別の有限オートマトンの一部としてや、別の`Meta`状態の一部として実行することができます。

```yaml
transtions:
- [ExampleParallelState, DEFAULT, ExampleMetaState]
- [ExampleMetaState, OK, LastStateOutput]
```

なお、Meta有限オートマトンの出力には、Meta有限オートマトン内で状態遷移しない最後の状態（この例では`LastStateOutput`）の出力が使用されます。
