`DataStore`は、任意のC++オブジェクトを効率的に保存・取得し、任意のオブジェクトをフレームワーク内で簡単かつ効率的に共有できるようにします。この機能では、型が厳しくチェックされ（保存時と同じ型のオブジェクトのみ取得できます）、C++の標準メカニズムのみを使用できます。

`DataStore`は、例えば以下の用途に使用できます。

- **有限オートマトンの`States`間でデータを共有する**。
  
  - 例えば、既存の有限オートマトンと`StabilizerStandingState`（[LIPMスタビライザーのチュートリアル](lipm-stabilizer.html)を参照）を同じ場所に置くだけで、有限オートマトンを簡単に安定させることができます。また、`DataStore`を使用して`StabilizerStandingState`の目標を指定できます。例えば、`StabilizerStandingState::com`プロパティを設定するだけで、質量中心の目標を変更できます。

- **追加の情報を[観測器](observers.html)に与える**（`KinematicInertial`観測器が必要とする`anchorFrame`など）。

- **プラグインとコントローラーとの間でデータを共有する**。例として、歩行コントローラーと歩容プランナープラグインとの間で情報をやり取りする場合について考えてみます。歩容プランナーは、入力として、*(i) 現在の足の接触面*と*(ii) 目標地点*に関する情報を必要とし、場合によっては追加の情報（障害物など）を必要とします。また、戻り値として、歩行コントローラーが従うべき*(iii) 次の歩容*を出力する必要があります。さらに、オンラインの歩容プランナーの場合、最新の歩容計画を取得する必要があります（目標やロボットの状態が変化したときや、新たな障害物が検出されたときなど）。
  
  - (i) の情報はコントローラーあるいはロボットの状態を推定する他のビジョンプラグインから取得可能
  - (ii) の情報はコントローラーあるいは物体検出などの他のプラグインから取得可能
  - (iii) の歩容計画はコントローラーによって使用される

もちろん、他にもいくつかの使用例が考えられます。この機能は、フレームワーク内でデータを簡単に共有できるようにすることを目的としています。データストアにオブジェクトを追加すると、そのオブジェクトが事実上グローバルになり、外部のプラグイン等を使用してどこからでも編集できるようになるため、この機能は慎重に使用してください。

## 使用方法

`mc_control::MCController`クラスにデフォルトの`DataStore`インスタンスが用意されており、`mc_control::MCController::datastore()`を使ってアクセスできます。コントローラーのインスタンスにアクセスできるあらゆる場所から、この`DataStore`インスタンスに任意の`c++`オブジェクトを作成したり、このインスタンスからオブジェクトを取得することがきます。

- 以下のように、`make`あるいは`make_initializer`を使用してデータストア上にオブジェクトを直接作成できます。

  ```cpp
  // 長さ4、初期値として42.0が入ったdouble型のベクトルを生成
  datastore().make<std::vector<double>>("key", 4, 42.0);
  // 長さ2、初期値として4と42が入ったdouble型のベクトルを生成
  datastore().make_initializer<std::vector<double>>("key", 4, 42.0);
  ```

- 同じ名前のオブジェクトが既に存在する場合は例外が発生します。これを回避するには、そのような要素がデータストアに既に存在するかどうかを以下のようにチェックします。
  ```cpp
  if(!datastore().has("key"))
  {
    datastore().make<Eigen::Vector3d>("key", 1, 2, 3);
  }
  ```

  なお、これらの関数は、作成したオブジェクトへの参照も返すため、その参照を使ってオブジェクトへのアクセスや編集が行えます。

  ```cpp
  auto & vec = datastore().make<Eigen::Vector3d>("EigenVector", Eigen::Vector3d::Zero());
  vec.x() = 42; // ベクトルの中身は42, 0, 0となる
  ```

- 保存されている値への参照は、`get<Type>`を使用していつでも取得できます。

  ```cpp
  auto & vec = datastore().get<Eigen::Vector3d>("EigenVector");
  // ベクトルの中身は42, 0, 0
  ```

  指定したキーがデータストアに存在しない場合や作成時と異なる型を`Type`に指定した場合は、例外がスローされます。`get`のオーバーロードを使用すると、`mc_rtc::Configuration`オブジェクトの場合と同様に、キーが存在しない場合のデフォルト値を指定できます。

  ```cpp
  Eigen::Vector3d vec{1,2,3};
  datastore().get<Eigen::Vector3d>("NotAKey", vec); // ベクトルの中身は1, 2, 3のまま
  datastore().get<Eigen::Vector3d>("EigenVector", vec); // ベクトルの中身は42, 0, 0になる

  bool hasFeature = datastore().get<bool>("HasFeature", false); // hasFeatureには"HasFeature"キーに対応する値が存在する場合はその値が、それ以外の場合はfalseが格納される。
  ```


## 高度な使用方法

- **継承**もサポートされています。ただし、後でオブジェクトを取得するには、オブジェクトの親クラスを明示的に指定する必要があります。以下に、継承とメンバー関数の派生を使用してオブジェクトの保存と取得を行う非常に簡単な例を示します。

  ```cpp
  struct A
  {
    virtual std::string hello() const
    {
      return "A";
    }
    std::string type() const
    {
      return "A";
    }
  };

  struct B : public A
  {
    std::string hello() const override
    {
      return "B";
    }
    std::string type() const
    {
      return "B";
    }
  };
  datastore().make<B, A>("ObjectB");
  auto & parent = store.get<A>("ObjectB");
  auto & derived = store.get<B>("ObjectB");
  parent.type();   // "A"
  parent.hello();  // "B" （hello()は仮想関数であるため）
  derived.type();  // "B"
  derived.hello(); // "B"
  ```

- **ラムダ関数**も保存できますが、特殊な`make_call`を使用する必要があります。

  ```cpp
  // ラムダ関数を生成し、std::functionとして格納する
  datastore().make_call("lambda", [](double t) {});
  // ラムダ関数の取得
  auto & lambdaFun = datastore().get<std::function<void(double)>("lambda");
  // 関数の呼び出し
  lambdaFun(42);
  // データストアからの直接呼び出し (関数の返り値と引数の型を繰り返す必要がある)
  datastore().call<void, double>("lambda", 42);
  ```
