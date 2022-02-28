一般に、制御されているロボットの実際の状態を知る必要があります。しかし、ロボットに組み込まれているセンサーから十分な情報を得られることはまれであるため、残念ながら、実際の状態を完全に把握できることはめったにありません。その代わり、各種センサー（関節エンコーダー、フォーストルクセンサー、IMU（慣性計測装置、カメラなど）の測定値と、コントローラーの制御対象（接触面など）に関する追加の情報から、コントローラーに関するシステムの状態を推測する必要があります。例えば、浮遊ベースロボットの場合、通常、浮遊ベースの状態（位置、向き、速度など）を完全に把握できるセンサーは搭載されていないため、入手できる情報を活用してその状態を推定する必要があります。これを実現する方法として、例えば、IMUやカルマンフィルターの情報を、ロボットのキネマティクスに関する既知の情報と組み合わせるというやり方があります。また、ビジュアルオドメトリを使用する方法や、モーションキャプチャーシステムからグラウンドトゥルース測定値を取得する方法のほか、これらを組み合わせる方法もあります。このようなプロセスは状態観測と呼ばれます。

ロボットに関してどのような状態を観測すべきか、その要件はコントローラーによって異なります。ヒューマノイドロボットの歩行コントローラーでは、ロボットの質量中心の状態を確実に推定することが重要であるため、ロボットのキネマティクス状態（浮遊ベースを含むボディ各部の位置と速度）を完全に把握する必要があります。一方、マニピュレーターアームでは、関節の位置と速度の情報しか必要としません。また、どのような方法で状態を取得するかも重要です（グラウンドトゥルース情報と推定値のどちらを使用するのか、どのようなセンサーやアルゴリズムを使用するのかなど）。

このフレームワークでは、1台または複数のロボットの状態観測を簡素化・一般化するため、**状態観測パイプライン**と呼ばれる仕組みを提供します。これは、状態観測をパイプラインとみなすというコンセプトに基づいています。各パイプラインは、シーケンシャルに実行される複数の観測器で構成されます。また、各観測器は、ロボットの状態を推定する役割を担います。そして、これらがすべて組み合わされ、パイプライン内のすべての観測器によってロボットの目標状態が完全に推定されます。複数のパイプラインを定義して実行することで、複数のロボットの状態の推定や、複数の推定手法の比較が行えます。観測器自体は、コントローラーやタスク、プラグインと同様に、シンプルなインターフェイスを用いてライブラリから読み込むことができるため、独自の観測器を簡単に定義できます。このフレームワークでは、現在、デフォルトで以下の観測器が用意されています。
- **エンコーダー観測器**: ロボットの関節の状態（位置と速度）を推定し、フォワードキネマティクスとフォワード速度の計算を行い、ボディの位置と速度を取得します。エンコーダーの位置、エンコーダーの速度（速度センサーや、位置の有限差分から取得）、ロボットの他の関節の値など、さまざまな入力を使用できます。
- **ボディセンサー観測器**: ロボットのボディに取り付けられたセンサーから得られた測定値と、センサーと浮遊ベースとの間のキネマティクスに基づいて、ロボットの浮遊ベースの状態を設定します。これは通常、シミュレーターで得られたグラウンドトゥルース測定値を利用する場合や、浮遊ベースに関する情報を提供する外部のコンポーネント（モーションキャプチャー、ロボットのプラットフォームに組み込まれた推定器など）の結果を利用する場合に使用されます。
- **キネマティクス慣性観測器**: {% doxygen mc_rbdyn::BodySensor %}（IMUの向き）とキネマティクスアンカーフレームから、ロボットの浮遊ベースの姿勢（位置と向き）と速度（ローパスフィルター後の位置の有限差分）を推定します。

このフレームワークでは、ロボットを表現するための2つのロボットインスタンスのセットが用意されています。
- {% doxygen mc_control::MCController::robots() %}は、ロボットの制御状態（目標状態）を表します。
- {% doxygen mc_control::MCController::realRobots() %}は、ロボットの実際の状態を表します。これらのロボットの状態をどのように推定するかは、観測器パイプラインで定義する必要があります。



# 観測器パイプラインを設定する

状態観測パイプラインの設定は、コントローラーの設定で行えます（各パイプラインの設定は、その前にあるパイプラインの設定よりも優先されます）。

- 全般的な設定: {% ihighlight bash %}$INSTALL_PREFIX/etc/mc_rtc.yaml{% endihighlight %}
- ユーザーによる設定: {% ihighlight bash %}$HOME/.config/mc_rtc/mc_rtc.yaml{% endihighlight %}
- コントローラー固有の設定: {% ihighlight bash %}$HOME/.config/mc_rtc/mc_controllers/YourController.yaml{% endihighlight %}
- コントローラーの有限オートマトンの設定: {% ihighlight bash %} YouController.yaml{% endihighlight %} （推奨）

設定形式については、[観測器のJSONスキーマ](../../json.html#Observers/ObserverPipelines)に詳しく記述されています。

はじめに、エンコーダーの位置測定値と浮遊ベースのロールとピッチを測定するセンサーの出力に基づいて浮遊ベースロボットの状態を推定する簡単な例を見てみましょう。

```yaml
---
ObserverPipelines:
- name: MainPipeline                     # - 新たなパイプラインを生成
  gui: true                              #   このパイプラインをGUIに表示 (デフォルトは非表示)
  log: true                              #   ログを取る (デフォルト)

  observers:                             #   使用するオブザーバを宣言
  - type: Encoder                        # - EncoderObserverを使用
    config:                              #
      position: encoderValues            #    - エンコーダ値から関節位置を設定 (デフォルト)
      velocity: encoderFiniteDifferences #    - 関節速度を有限差分により計算  (デフォルト)
                                         # ここまでの設定で関節の位置・速度、対応するボディの位置・速度が推定できるが、
                                         # フローティングベースの情報が欠けている

  - type: BodySensor                     # - BodySensor オブザーバを使用
    update: false                        #   出力を実ロボットの状態に設定しない
    gui: false                           #   GUIに表示しない
    config:                              #
      bodySensor: FloatingBase           #   シミュレーション時はインタフェースが真値をこのセンサ出力として設定する
                                         #   オブザーバはフローティングベースの位置及び速度を
                                         #   センサの計測値を変換することで計算する

  - type: KinematicInertial              # - フローティングベースの状態をKinematicsInertialオブザーバを用いて推定する
    update: true                         #   出力を実ロボットの状態に設定する
    gui: true                            #   推定された速度を矢印で表示する (デフォルト)
    config:
      imuBodySensor: Accelerometer       # このオブザーバはセンサからのロールとピッチの情報、
                                         # アンカーポイント、ロボットのキネマティクスを利用して
                                         # アンカーポイントはデータストアのコールバックを用いて
                                         # 提供されることが想定されている（詳細は以下を参照）。
```

使用できるオプションの詳細については、該当するJSONスキーマを参照してください。
- [ObserverPipelines](../../json.html#Observers/ObserverPipelines): 複数の状態観測パイプラインから成る配列
- [ObserverPipeline](../../json.html#Observers/ObserverPipeline): 観測器を持つ観測パイプラインの定義
- [EncoderObserver](../../json.html#Observers/Encoder): エンコーダー観測器のオプションr
- [BodySensorObserver](../../json.html#Observers/BodySensor): ボディセンサー観測器のオプション
- [KinematicInertial](../../json.html#Observers/KinematicInertial): キネマティクス慣性観測器のオプション

上記パイプラインの作成時、以下のような簡単な説明がフレームワークに表示されます。

```
ObserverPipelines:
- ExamplePipeline: Encoder (position=encoderValues,velocity=encoderFiniteDifferences) -> [BodySensor (sensor=FloatingBase,update=sensor)] ->  KinematicInertial (sensor=Accelerometer,cutoff=0.010000)
```

この説明には、パイプラインの実行に関する情報と、観測器のシーケンスに関する情報が表示されます。`[..]`括弧に囲まれた観測器は、実行されますが`realRobots`インスタンスの状態には影響を与えませんパイプラインを実行すると、推定されたロボットの状態が`realRobots`インスタンスに格納され、この情報をコントローラーで使用することができます。

例えば、上記のパイプラインでは以下のことが可能です。

- 関節の位置`readRobot().mbc().q()`と速度`realRobot().mbc().alpha()`を取得する
- 浮遊ベースの姿勢`realRobot().posW()`を取得する
- 浮遊ベースの速度`realRobot().velW()`を取得する:
- ボディの姿勢を取得する: `realRobot().bodyPosW("bodyName");`
- ボディの速度を取得する: `realRobot().bodyVelW("bodyName");`
- 質量中心の位置と速度`realRobot().com() / realRobot().comVelocity()`を取得する
- ...

このサンプルパイプラインの最終結果は以下のようになります*（左: Choreonoidによるシミュレーション、右: 制御状態（半透明の表示）と観測された状態（実体表示））*

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/ssoNkV940yc" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

## デフォルト観測器

このセクションでは、このフレームワークで用意されているデフォルト観測器について簡単に説明します。詳細については、各観測器のAPIドキュメントとJSONスキーマを参照してください。

## エンコーダー観測器

- API: {% doxygen mc_observers::EncoderObserver %}
- [JSONスキーマ](../../json.html#Observers/Encoder)

エンコーダー観測器を使用すると、駆動されているすべての関節の位置と速度を取得できます。

- 関節の値は、センサーの値{% doxygen mc_rbdyn::Robot::encoderValues() %}として取得するか、ロボットの他の関節状態{% doxygen mc_rbdyn::Robot::q() %}を使用して取得できます。
- 関節の速度は、関節速度センサーの値{% doxygen mc_rbdyn::Robot::encoderValues() %}として取得するか、（推定された）位置の有限差分による推定値として取得するか、ロボットの他の関節速度{% doxygen mc_rbdyn::Robot::alpha() %}を使用して取得できます。
- この観測器では、フォワードキネマティクス{% doxygen mc_rbdyn::Robot::forwardKinematics() %}とフォワード速度{% doxygen mc_rbdyn::Robot::forwardVelocity() %}が計算され、該当するボディの位置と速度が更新されます（この位置と速度は、後続の観測器で使用されます）。

## ボディセンサー観測器

- API: {% doxygen mc_observers::BodySensorObserver %}
- [JSONスキーマ](../../json.html#Observers/BodySensor)

ボディセンサー観測器では、{% doxygen mc_rbdyn::BodySensor %}によって与えられた情報に基づいて、浮遊ベースの状態を推定できます。ボディセンサーは、ロボットのボディに取り付けられたセンサーで、ボディの状態を測定した結果が出力されます。`BodySensor`には以下の情報が格納されます。

- センサーが取り付けられているボディとセンサーとの間の運動学的変換（必要に応じて）
- ボディの位置と姿勢
- 直線速度と角速度
- 直線加速度と角加速度

ロボットに取り付けられているセンサーの測定値のみを使用することができ、他の測定値はデフォルトで0になります。ボディセンサー観測器は、少なくとも、位置、姿勢、線形速度、角速度の測定値を必要とします。さらに、センサーが浮遊ベースに直接取り付けられていない場合は、センサーと浮遊ベースとの間の運動学的変換が必要となります（これは`EncoderObserver`などを使用して取得できます）。

この観測器は、一般に、`FloatingBase`ボディセンサーで得られた浮遊ベースの状態に関するグラウンドトゥルース測定値をシミュレーターインターフェイスから出力するのに使用されます。

## キネマティクス慣性観測器

- API: {% doxygen mc_observers::KinematicInertialObserver %}
- [JSONスキーマ](../../json.html#Observers/KinematicInertial)
- [この手法に関するStéphane Caronによる詳しい説明](https://scaron.info/teaching/floating-base-estimation.html)

浮遊ベースロボットでは、センサーを使用して浮遊ベースの状態を完全に把握できることはめったにありません。キネマティクス慣性観測器を使用すると、IMUセンサーから得られた向きの推定値に基づいて、浮遊ベースの位置、向き、直線速度、角速度を簡単に推定できます。ここで実装されている手法に関する詳しい説明については、[Stéphane CaronのWebサイト](https://scaron.info/teaching/floating-base-estimation.html)を参照してください。なお、この観測器は、階段を上るロボット（[動画](https://www.youtube.com/embed/vFCFKAunsYM)）で見られるように、[LIPM（線形倒立振子モード）歩行コントローラー](https://github.com/jrl-umi3218/lipm_walking_controller)による歩行制御で実際に広く使用されています。

この手法では、重力を基準としたロール回転角とピッチ回転角をセンサーから取得する必要があります。重力ベクトル周りの回転（ヨー）は、IMUと同様、一般にセンサーでは観測できないため、制御目標ロボットの目標回転角を代用することで、センサーフレームを完全に推定することができます。

位置の推定では、接触面の位置が既知であると仮定して、観測された向きと最もよく一致する位置のみが推定されます。これは、仮定された接触面間のアンカーポイントを与えることで実現されます。静止している場合は、すべての接触面の間にある中心点をアンカーフレームとして選択するのが最適です。歩行などの動作を実行する場合は、接触面の遷移が発生したときにぎくしゃくした動きにならないように、接触面の間でアンカーフレームをスムーズに補間する必要があります。このフレームは、データストアのコールバック関数[datastore]({{site.baseurl}}/tutorials/recipes/datastore.html)によって与えられます。

例えば、先のセクションで説明したパイプラインの場合、アンカーフレームは、想定される2つの足の接触面間のフレーム中心として与えられます。

```cpp
double leftFootRatio = 0.5
ctl.datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                          [this, &leftFootRatio](const mc_rbdyn::Robot & robot)
                          {
                            return sva::interpolate(robot().surfacePose("LeftFoot"),
                                                    robot().surfacePose("RightFoot"),
                                                    leftFootRatio)
                          });
```

なお、この関数は2回呼び出されます。制御目標ロボットインスタンス用に1回、実ロボットインスタンス用に1回呼び出されます（アンカーフレームとセンサーフレームとの間の相対的な姿勢のみが観測器で使用されます）。

右足は地面につけたまま左足をスイングさせて歩行を開始したい場合、まず、接触面の遷移が発生するまで右足の接触面に向けてアンカーフレームをスムーズに動かしてから、ステップ完了時に左足が新たな場所に着地するように、2つの目標接触面の間にアンカーフレームをスムーズに動かす必要があります。アンカーフレームの動きがスムーズでない場合、推定された浮遊ベースの位置が途中でジャンプする可能性があります（向きは影響を受けません）。

その後、この観測器では、推定された位置と向きの有限差分に基づいて、浮遊ベースの直線速度と角速度が計算されます。さらに、この速度がローパスフィルターによって処理されます。

この観測器の使用例については、[アドミッタンス制御の例を使用したコントローラーのチュートリアル]({{site.baseurl}}/tutorials/samples/sample-admittance.html)、{% doxygen mc_control::fsm::StabilizerStandingState %}を参照してください。また、より複雑な使用例については、[LIPM歩行コントローラー](https://github.com/jrl-umi3218/lipm_walking_controller)を参照してください。

## 推定されたロボットを可視化する

推定されたロボットをRVizで可視化できます。デフォルトでは、メインのロボットインスタンスは以下のプロパティを持つ`RealRobot`要素として表示されます。

```yaml
Robot Description path: /real/robot_description
TF Prefix: /real
```

追加のロボットをパブリッシュするには以下のようにします。

```yaml
# ロボットはenv_1, env_2等としてパブリッシュされる
Robot Description path: /real/env_*/robot_description
TF Prefix: /real/env_*
```

# コードを使用して観測器パイプラインを操作する

コードを使用して観測器パイプラインを操作すると便利な場合があります。例えば以下の場合に役に立ちます。
- 特定の観測パイプラインが動作しているかどうかをチェックする
- 特定の観測器が存在するかどうかをチェックする
- 特定の観測器の状態をコードを使用して調べる
- 観測器が存在する場合にのみアクションを実行する
- ...

## 観測器パイプラインの状態を照会する

以下に、観測器パイプラインの状態を照会し、その状態に応じてアクションを実行する方法を示す簡単なコードスニペットを示します。

```cpp
bool checkObserverPipeline(const std::string & observerPipelineName)
{
  if(!hasObserverPipeline(observerPipelineName))
  {
    mc_rtc::log::error("This controller does not have a pipeline named {}", observerPipelineName);
    return false;
  }
  const auto & observerp = observerPipeline(observerPipelineName);
  if(!observerp.success()) // パイプラインが失敗していないか確認
  {
    mc_rtc::log::error("Required pipeline \"{}\" for real robot observation failed to run!", observerPipelineName);
    // どのオブザーバが失敗したかチェック
    for(const auto & observer : observerp.observers())
    {
      if(!observer.success())
      {
        // エラーメッセージを表示
        mc_rtc::log::error("Observer \"{}\" failed with error \"{}\"", observer.observer().name(), observer.observer().error());
        if(observer.observer().name() == "MyObserver")
        {
          // このオブザーバが失敗した場合に特有の処理を実行
        }
      }
    }
    return false;
  }
  return true;
}
```

`checkObserverPipeline("RequiredObserverPipeline");`を呼び出すと、このパイプラインが存在するかどうかの情報と、問題が発生している観測器とその原因が通知され、パイプライン内に観測器`MyObserver`が存在する場合はその観測器に固有のアクションが実行されます。その他の利用可能な機能については、{% doxygen mc_observers::ObserverPipeline %}のドキュメントを参照してください。

# 独自の観測器を作成する

このフレームワークは、ライブラリから観測器を読み込みます。これを正しく機能させるには、観測器を{% doxygen mc_observers::Observer %}から継承し、以下のを実装する必要があります。

```cpp
// YourObserver.h
void configure(const mc_control::MCController & /*ctl*/, const mc_rtc::Configuration & /*config*/) override
void reset (const mc_control::MCController &ctl) override
bool run (const mc_control::MCController &ctl) override
void update(mc_control::MCController &ctl) override
```

さらに、`mc_rtc`が観測器を見つけられるように、`mc_rtc`の観測器ローダーにライブラリを検索させるための読み込みシンボルを定義する必要があります。これを行うには、`mc_observers/ObserverMacros.h`で定義された以下のマクロを使用します。

```cpp
// YourObserver.cpp
#include <mc_observers/ObserverMacros.h>
// オブザーバの実装 (configure, reset, run, update 関数)
EXPORT_OBSERVER_MODULE("YourObserver", your_namespace::YourObserverClassName)
```

独自の観測器をコンパイルするには、提供されたマクロを使用します。このマクロによって、独自に作成した観測器が`mc_observers`とリンクされ、デフォルトの観測器パスにその観測器がインストールされます。

```cmake
add_observer(YourObserverName YourObserver.cpp YourObserver.h)
```

注: フレームワークで提供されたデフォルトの観測器のいずれかから継承したい場合は、その観測器をリンクする必要があります。例えば、{% doxygen mc_observers::BodySensorObserver %}から継承したい場合、これに対応する`mc_observers::BodySensorObserver`ターゲットをリンクする必要があります。

```cmake
target_link_libraries(YourObserverName PUBLIC mc_observers::BodySensorObserver)
```

実際の例については、こちらの[サンプルプロジェクト](https://github.com/arntanguy/mc_observer_example)を参照してください。
