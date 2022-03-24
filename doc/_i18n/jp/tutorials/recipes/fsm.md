有限オートマトン（FSM: Finite State Machine）は、有限個の状態のいずれかの状態を取る抽象機械です。この機械の状態は、現在の状態における内部または外部の条件に基づき変化します。そのような変化は状態遷移と呼ばれます。こうした機械は、プログラミングの世界では至る所で使用されています。また、構造が簡単でさまざまな構成が可能であるため、特にロボットに関するシナリオを実現するのに非常に適しています。

状態遷移図の概念を拡張したものが有限オートマトンです。有限オートマトンでは2つの主要な原則が導入されています。

- **階層構造**: 状態遷移図内に状態遷移図をネストさせることができる
- **並行性**（または直交性）: 2つ以上の状態を同時に実行することができる

mc\_rtcのコントローラーでは、状態遷移図の概念を実装できます。このページでは、有限オートマトンの実装に関する詳細について説明します。

## 状態、状態遷移、有限オートマトンの実行

### 状態

状態の実装については後ほど説明します。ここでは、状態に関する4つの主なメソッドについて説明します。

- `configure`は、状態を設定するのに使用します。このメソッドは複数回呼ばれます。
  - 状態が継承している各階層の状態について一度ずつ呼ばれます。
  - 実行器の設定について一度呼ばれます。つまり、グローバルな有限オートマトンまたは他の状態を管理する状態（例えば`Meta`や`Parallel`）の`configs`エントリから呼ばれます。
  - デフォルトの実装では単にメソッドに渡された設定を、順に状態の実装からアクセス可能な`config_`オブジェクトにロードします。より複雑なロード機能を実現するために、このメソッドをオーバーライドすることができます。

なお、有限オートマトンの実装では、既存の状態と異なる設定を指定することで、新しい状態を作成することができます。そのため、この関数は複数回呼び出されます。
- `start`は、初期化を実行するのに使用します。1度だけ呼び出されます。
- `run`は、状態で実装されるメインの関数です。その状態の処理が終了するか、状態が変化するまで、ループ処理が実行されるたびに1回ずつ呼び出されます。runの処理が完了したときに、状態の出力として任意の値を設定できます。この値は状態内で明確に定義する必要があります。
- `teardown`は、クリーンアップ関数です。状態が変化したときに呼び出されます。

### 状態遷移

状態遷移は、以下の内容を表す4つの属性で構成されます。

1. 状態からの遷移
2. 状態の出力
3. 状態への遷移
4. 有限オートマトンが状態遷移をどのように処理するかに関するオプションのパラメーターこれらは、`StepByStep`（デフォルト）、`Auto`、`Strict`のいずれかの値を取ります。それぞれの意味については後ほど説明します。

### 有限オートマトンの実行

有限オートマトンコントローラーは、マネージドモードまたは非マネージドモードで実行されます。はじめに、非マネージドモードについて見ていきます。

#### 非マネージドモード

このモードでは、状態の作成・実行と状態間の遷移は有限オートマトンによって処理されます。

有限オートマトンでは、ループ処理内で以下のロジックが実行されます。

* 状態が既に実行されている場合
  * 状態のrunメソッドを実行する
  * trueが返された場合は、状態の出力をチェックし、該当する状態遷移を見つける
    * 状態をクリーンアップする
    * 状態遷移の種類が`Auto`の場合、あるいは状態遷移の種類が`StepByStep`かつ有限オートマトンの`StepByStep`の設定が`false`の場合
      * 次の状態の設定と初期化を行う
    * そうでない場合
      * アイドル状態を設定する
* 状態が実行されていない場合
  * 状態遷移がトリガーされた場合
    * アイドル状態を解除し、次の状態の設定と初期化を行う

上記の概要では、ロジックの流れを簡単にするためにいくつかの点が省略されています。

1. 外部のトリガーによって状態の実行が中断される場合があります。この場合、最初にその状態がクリーンアップされます。その後、有限オートマトンは、状態遷移コマンドが実行されるのを待ちます。
2. 状態と出力のペアに対応する遷移先が存在しない場合、有限オートマトンは自らの処理が完了したとみなします。この場合、状態遷移コマンドを送信することで有限オートマトンの処理を再開できます。
3. アイドル状態は必須ではありません。アイドル状態を開始しようとしたときにアイドル状態が無効になっていた場合、状態遷移がトリガーされるまで、その前の状態のrunメソッドが引き続き呼び出されます。

##### アイドル状態について

アイドル状態は、それまで実行されていた状態によって実現されていたロボットの状態を維持しようと試みます（アイドル状態が有効の場合）。これは、2つのタスクによって実現されます。

- 現在の姿勢を目標として設定する姿勢制御タスク
- 現在の傾きを目標として設定するエンドエフェクタータスク（フリーフライヤーの場合）土台が固定されているロボットの場合、このタスクは実行されません。

処理の中断がトリガーされた場合、この「状態」は常に実行されます。

#### マネージドモード

このモードに関して説明することはそれほどありません。マネージドモードでは、有限オートマトンは状態遷移を処理しません。この場合、状態遷移は外部のツールによって完全に処理されます。状態の存続期間は上記の場合と同様ですが、状態遷移は**すべて**外部のツールによってトリガーされます。 

## テキストベースの継承

ここでは、C++オブジェクトとして記述された状態について説明します。このオブジェクトには、さまざまな設定オプションが用意されています。しかし、わずかな違いしかない2つ以上の状態を定義したい場合、すべてのオプションを毎回設定するのは大変です。そこで、本インターフェイスでは、一連の設定を継承する仕組みが用意されています。

以下の処理を実行する、`StateBase`という名前のC++状態があるとします。

```yaml
MyFirstState: # <-- 新しい状態の名称
  base: StateBase # <-- 基底となる状態のC++クラスの名称
  # 他のオプション
MySecondState: # <-- 新しい状態名称
  base: MyFirstState # <-- テキストで定義した状態を基底として使用することも可能
  # 他のオプション
```

オプションがどのように組み合わされるかは、C++状態の実装方法により異なるため、状態内で明確に定義する必要があります。ただし、mc_rtcで用意されている状態については、一般的な規則が当てはまります。これらの状態については、基本的に`mc_rtc::Configuration`オブジェクトのデフォルトの読み込み規則が適用されます。

- 「生」の値（bool値、数値、文字列）とベクトルは上書きされる
- オブジェクトは以下の規則に従って結合される:
  - 読み込み先オブジェクトにキーが存在しない場合、読み込み元オブジェクトの値が使用される
  - どちらのオブジェクトにもキーが存在し、型も一致する場合、この規則が適用される（オブジェクトの場合は再帰的に適用される）
  - それ以外の場合、読み込み元オブジェクトの値によって読み込み先オブジェクトの値が上書きされる

## mc_rtcに既に実装されている状態


このセクションでは、mc_rtcで用意されている主な状態について説明します。利用可能な状態とその設定に関する詳細については、[状態に関するJSONスキーマ]({{site.baseurl}}/json.html#State-objects)を参照してください。

<div class="no_toc_section">

<ul class="nav nav-tabs" id="statesTab" role="tablist">
  <li class="nav-item">
    <a class="nav-link active" id="PauseTab" data-toggle="tab" href="#PauseTabContent" role="tab" aria-controls="PauseTabContent" aria-selected="true">Pause</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="MetaTasksTab" data-toggle="tab" href="#MetaTasksTabContent" role="tab" aria-controls="MetaTasksTabContent" aria-selected="false">MetaTasks</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="AddRemoveContactTab" data-toggle="tab" href="#AddRemoveContactTabContent" role="tab" aria-controls="AddRemoveContactTabContent" aria-selected="false">AddRemoveContact</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="ParallelTab" data-toggle="tab" href="#ParallelTabContent" role="tab" aria-controls="ParallelTabContent" aria-selected="false">Parallel</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="MetaTab" data-toggle="tab" href="#MetaTabContent" role="tab" aria-controls="MetaTabContent" aria-selected="false">Meta</a>
  </li>
</ul>
<div class="tab-content" id="statesTabContent">
  <div class="tab-pane show active" id="PauseTabContent" role="tabpanel" arial-labelledby="PauseTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>この状態では、しばらく待ってから<code>OK</code>が出力されます。</p>

        <h5>オプション</h5>

        <ul>
          <li><code>duration</code>: 一時停止する期間（単位: 秒）を浮動小数点数の値で指定します。デフォルトは0です。</li>
        </ul>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="MetaTasksTabContent" role="tabpanel" arial-labelledby="MetaTasksTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>この状態では、任意の数のMetaTasksが作成され、各タスクの完了基準が満たされるまでそれらのタスクが実行されます。</p>

        <p>この状態に含まれるタスクは、JSONオブジェクトである<code>tasks</code>エントリを使用して設定します。</p>

        <p>タスクの名前をキーとして指定し、MetaTaskLoaderで必要とされるMetaTaskオブジェクトを値として指定します。また、オプションとして、1つ以上の完了基準を"completion"エントリで指定することもできます。</p>

        <p>タスクの名前はこの状態でのみ有効です。タスクの設定でnameエントリを使用することで、実際のタスク名を変更できます。</p>

        <p>タスクの<code>completion</code>エントリが存在せず、関連するタスクのcompletionエントリが存在しない場合、このタスクは追加されますが、このタスクの完了基準は考慮されません（例えば、CoMTaskとEndEffectorTaskを追加した場合、後者の完了基準のみが考慮されます）。</p>

        <p><code>tasks</code>エントリが複数回読み込まれた場合の動作は以下のようになります。</p>
        <ul>
        <li>新しいタスクが読み込まれた場合、そのタスクが追加されます。</li>
        <li>既存のタスクが再度読み込まれなかった場合、そのタスクについては何も起こりません。</li>
        <li>既存のタスクが読み込まれた場合、既存の設定エントリが新しいエントリに置き換えられます。既存の設定エントリが存在しない場合は、新たに読み込まれた設定エントリが既存の設定に追加されます。</li>
        </ul>

        <h5>例</h5>

        {% highlight yaml %}
        # わかりやすい例とするためタスクの定義を簡略化しています

        # 1つ目のパス
        tasks:
          t1:
            objectiveA: 0.5,
            objectiveB: 1.0,
            completion: { timeout: 5.0 }

        # このパスには1つのタスク

        # 2つ目のパス
        tasks:
          t1:
            objectiveA: 1.0,
            completion: { eval: 1e-6 }
          t2:
            objective: 0.5

        # このパスには2つのタスクがあり、
        # - t1 の objectiveA は 1.0 に変更、objectiveB は同じ
        # - t1 の完了判定は置き換え

        # 3つ目のパス
        tasks:
          t1:
            completion: {}
          t2:
            completion: { eval: 1e-6 }

        # 上と同じように2つのタスクがあり、objective は同じだが、
        # - t1 は完了判定なし
        # - t2 は完了判定あり
        {% endhighlight %}

        <h5>オプション</h5>

        <ul>
          <li><code>tasks</code>: コントローラーのこの状態に追加するタスクが記述されたオブジェクト</li>
        </ul>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="AddRemoveContactTabContent" role="tabpanel" arial-labelledby="AddRemoveContactTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>接触面を削除または追加できる状態を実装します。</p>

        <h5>オプション</h5>

        <p>この状態を設定するには、以下の2つのエントリを定義する必要があります。</p>

        <ul>
          <li><code>type</code>: [<code>addContact</code>、<code>removeContact</code>、 <code>compliance</code>]のいずれか</li>
          <li><code>contact</code>: 削除または追加する接触面</li>
        </ul>

        <h5>接触面の削除に関するオプション</h5>

        <ul>
          <li><code>distance</code>: 接触面とボディがこの距離だけ離れると、この状態が終了します。デフォルトは0.1（10cm）です。</li>
        </ul>

        <h5>コンプライアンス制御における接触面の追加に関するオプション</h5>

        <ul>
          <li><code>velocity</code>: ComplianceTaskの速度のしきい値。デフォルトは1e-4です。</li>
        </ul>

        <pその他のオプションは、接触面を追加・削除するタスクの種類によって異なります。コンプライアンス制御タスクの場合、接触面の値に基づいて <code>body</code>エントリが上書きされます。</p>

        <p>力覚センサーが取り付けられていない接触面をコンプライアンス制御タスクで削除した場合、この状態は自動的に<code>addContact</code>にフォールバックします。</p>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="ParallelTabContent" role="tabpanel" arial-labelledby="ParallelTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>Parallel状態を実装します。</p>

        <p>Tこの状態では、複数の状態が同時に実行されます。厳密に言うと、これらの状態は並列ではなくシーケンシャルに実行されます。</p>

        <p>この状態によって<code>{state_1, ..., state_N}</code>という状態が実行された場合、この状態は、すべての<code>state_i::run()</code>関数がtrueを返したときに完了します。このとき、この状態は<code>state_N</code>を出力します。</p>

        <h5>オプション</h5>

        <ul>
          <li><code>states</code>: この状態によって実行される状態のリスト</li>
          <li><code>configs</code>:　各状態に含まれる状態の設定。このconfigs（state）は、各状態をさらに細かく設定するのに使用します。</li>
        </ul>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="MetaTabContent" role="tabpanel" arial-labelledby="MetaTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>「メタ」状態を実装します。</p>

        <p>この状態は、自分自身の有限オートマトンを実行します。</p>

        <h5>オプション</h5>

        <ul>
          <li><code>Managed</code>: trueに設定した場合、状態遷移は処理されません。</li>
          <li><code>transitions</code>: 有限オートマトンコントローラーと同様の状態遷移マップ（Managedをfalseに設定した場合、このオプションを設定する必要があります）。</li>
          <li><code>StepByStep</code>: 内部有限オートマトンの場合、有限オートマトンの設定と同じになります（デフォルトでは親有限オートマトンの<code>StepByStep</code>設定と同じになります）。</li>

          <li><code>configs</code>: 有限オートマトンに含まれる状態に関する追加の設定を記述できます。</li>
        </ul>
      </div>
    </div>
  </div>
</div>

</div>

### 共通のオプション

一部のオプションはすべての状態に共通です。

- `AddContacts`/`RemoveContacts`: 状態が**実行される前**に、接触面を追加・削除できます。`fsm::Contact`オブジェクトのベクトルとして指定します。
- `AddContactsAfter`/`RemoveContactsAfter`: 状態が**実行された後**に、接触面を追加・削除できます。`fsm::Contact`オブジェクトのベクトルとして指定します。
- `RemovePostureTask`: trueに設定すると、デフォルトの姿勢制御タスクが削除されます。


## 新しい状態の生成

状態を作成するには、{% doxygen mc_control::fsm::State %}から継承します。最低限のインターフェイスを以下に示します。

```cpp
namespace mc_control::fsm
{
  struct MyState : public State
  {
    void configure(mc_rtc::Configuration &) override;

    void start(Controller &) override;

    bool run(Controller &) override;

    void teardown(Controller &) override;
  };
}

EXPORT_SINGLE_STATE("MyState", mc_control::fsm::MyState);
```

関数内で`Controller`インスタンスを渡す場合、 {% doxygen mc_control::fsm::Controller %}インスタンスとして渡します。

### `void configure(mc_rtc::Configuration &)`

この関数は**複数回**呼び出されます。この関数では、設定エントリを「定義」するだけとし、タスクは作成しないでください。

### `void start(Controller &)`

この関数は状態の初期化時に1度だけ呼び出されます。ここでは、定義した設定が実行可能な状態に変換されます。

### `bool run(Controller &)`

この関数は、`start`ループの実行に続き、ループ処理が実行されるたびに1回ずつ呼び出されます（`start`ループから`run`を明示的に呼び出さない限り、`start`と同じループ内では`run`は呼び出されません）。

状態の処理が完了すると、この関数は`true`を返します。このとき、`output(const std::string &)`を呼び出して状態の出力を設定するようにしてください。

### `void teardown(Controller &)`

この関数は、状態が破棄される前に1度だけ呼び出されます。この関数を使用して、状態がコントローラーに影響を与えないようにしてください。

## 有限オートマトンコントローラーに特有の内容

有限オートマトンは、いくつかの点で通常のmc_rtcコントローラーと異なります。

#### 接触面

有限オートマトンでは、接触面構造の簡易版が使用されています。これは以下のように定義されています。

```cpp
struct Contact
{
  std::string r1;
  std::string r2;
  std::string r1Surface;
  std::string r2Surface;
  Eigen::Vector6d dof; // Eigen::Vector6d::Ones()がデフォルト
};
```

`dof`ベクトルは対角行列に変換され、[自由度制約条件](contact-dof.html)として追加されます

有限オートマトンコントローラーで接触面を追加・削除するには、以下の関数を呼び出します。

```cpp
void addContact(const Contact &);

void removeContact(const Contact &);
```

#### 衝突メッシュ

衝突メッシュを追加・削除するには、以下の関数を呼び出します。

```cpp
void addCollisions(const std::string & r1, const std::string & r2,
                   const std::vector<mc_rbdyn::Collision> & collisions);

void removeCollisions(const std::string & r1, const std::string & r2,
                   const std::vector<mc_rbdyn::Collision> & collisions);

// r1 と r2 の間の全ての干渉回避拘束を削除
void removeCollisions(const std::string & r1, const std::string & r2);
```

有限オートマトンによって、必要に応じて衝突制約条件が作成されて追加されます。

#### 姿勢制御タスク

有限オートマトンによって、多関節ロボットの姿勢制御タスクが作成されます。このタスクにアクセスするには、以下の関数を呼び出します。

```cpp
std::shared_ptr<mc_tasks::PostureTask> getPostureTask(const std::string & robot);
```
このタスクは、通常、状態が存在している間はソルバー内に保持されます。そのようにしたくない場合は、`teardown`の呼び出し時にこのタスクをソルバーから取り出して元の場所に戻すことができます。

### その他のメソッド

以下のメソッドは、`State`インターフェイス内の仮想メソッドで、ユーザーが定義したメソッドで上書きすることができます。

#### `void stop(Controller &)`

Tこれは、状態の処理が中断されたときに呼び出されます。

## 有限オートマトンの設定

以下のオプションを使用して有限オートマトンを設定できます。

- `Managed`: trueの場合、有限オートマトンはマネージド型で、そうでない場合は非マネージド型です。
- `StepByStep`: trueの場合、`StepByStep`とタグ付けされた状態遷移は`Strict`の状態遷移として動作します。falseの場合、状態遷移は`Auto`の状態遷移として動作します。
- `IdleKeepState`: trueの場合、ユーザーによって状態遷移がトリガーされるまで同じ状態が維持されます。
- `StatesLibraries`: 状態ライブラリの参照先
- `StatesFiles`: 状態設定ファイルの参照先
- `VerboseStateFactory`: trueの場合、ライブラリの読み込み中に状態ファクトリによって詳細情報が出力されます。これはデバッグに役立ちます。
- `robots`: JSONオブジェクト。各キーはロボットの名前を表します。値は、メインロボットモジュールのほかに追加で読み込むロボットモジュールを表すオブジェクトです。

```json
// Example robots entry
"robots":
{
  "ground":
  {
    "module": "env",
    "params": ["@MC_ENV_DESCRIPTION@", "ground"]
  }
}
```

- `constraints`: 制約条件の配列。各オブジェクトは、JSONスキーマに基づきJSON形式で記述された`mc_solver::ConstraintSet`オブジェクトです。
- `collisions`: `mc_solver::CollisionConstraint`JSONスキーマに基づき定義された衝突制約条件の配列
- `contacts`: 初期接触面の配列

```json
// contactsエントリの例
"contacts":
[
  {
    "r1": "jvrc1",
    "r2": "ground",
    "r1Surface": "LeftFoot",
    "r2Surface": "AllGround"
  },
  {
    "r1": "jvrc1",
    "r2": "ground",
    "r1Surface": "RightFoot",
    "r2Surface": "AllGround"
  }
]
```

-- 各ロボットには<code>r</code>で始まる名前が付けられています。<code>r</code>で始まるエントリを使用して、アイドル状態で使用される姿勢制御タスクとフリーフライヤータスクを設定できます。
- `states`: オブジェクト。各キーは状態の名前を表します。値はこの状態の設定を表します。
- `configs`: オブジェクト。各キーは状態の名前を表します。値は、状態がメインの有限オートマトンで実行されたときにその状態に渡される追加の設定値を表します。
- `transitions`: 配列。配列の各要素は状態遷移を表します。
- `init`: 有限オートマトンを起動したときの初期状態
- `init_pos`: メインロボットの初期位置（7次元の配列）

次のチュートリアルでは、有限オートマトンを使用した実用的な例を実装します。
