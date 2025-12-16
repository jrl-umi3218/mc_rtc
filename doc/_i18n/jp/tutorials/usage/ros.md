[ROS](https://en.wikipedia.org/wiki/Robot_Operating_System)（ロボットオペレーティングシステム）は、ロボット工学の分野で非常に有名なミドルウェアで、この分野で使用されている数多くのツールがROSを使用してデータをやり取りすることができます（ツールによってはROS専用のものもあります）。ただし、ROSはリアルタイムアプリケーション用に設計されていないため、ROSの製品をmc_rtcと統合する際は、その点を考慮する必要があります。このドキュメントでは、ROSの原理についてある程度習熟していることを前提として説明します。

## mc_rtcはROSとどのようなデータをやり取りするか

mc_rtcは、[提供されているオプション]({{site.baseurl}}/tutorials/introduction/configuration.html)に応じて以下の情報をパブリッシュします。

1. ロボットの制御状態。これは、mc\_rtcが参照情報として使用します。
2. ロボットの実際の状態。インターフェイスによって提供されますが、完全な情報が得られないことがあります。

前者は`control`名前空間でパブリッシュされ、後者は`real`名前空間でパブリッシュされます。

これ以外の情報については、コントローラーのコードで処理する必要があります。特に、<strong>mc_rtcは回転について関知しません。</strong>

外部のツールがロボットの状態をROSにパブリッシュできるように、`mc_rtc::RobotPublisher`クラスが用意されています。

## NodeHandle の取得

以下のコードにより、mc\_rtc によって作成された `ros::NodeHandle` にアクセスできます:
```cpp
#include <mc_rtc/ros.h>

std::shared_ptr<ros::NodeHandle> nh = mc_rtc::ROSBridge::get_node_handle();
```cpp
#include <mc_rtc/ros.h>

std::shared_ptr<ros::NodeHandle> nh = mc_rtc::ROSBridge::get_node_handle();
```

mc\_rtcがROSサポートなしでビルドされた場合、またはROSが初期化できなかった場合（通常はマスターが利用できないため）、返されるポインタはNULLになります。

この機能を使用するには、`mc_rtc::mc_rtc_ros`でリンクしていることを確認してください：

```cmake
target_link_libraries(${YOUR_LIBRARY} PUBLIC mc_rtc::mc_rtc_ros)
```

## mc\_rtc の ROS サポート確認方法

`mc_rtc` が ROS サポートを有するか確認する方法は 3 つあります。

1. コンパイル時プリプロセッサチェック

```cpp
#ifdef MC_RTC_HAS_ROS_SUPPORT
// mc_rtc は ROS サポート付きでコンパイル済み
#endif
```

2. ランタイム定数

```cpp
#include <mc_rtc/config.h>

if(mc_rtc::MC_RTC_SUPPORT_ROS)
{
  // mc_rtc は ROS サポートあり
}
```

3. ランタイムノードチェック

```cpp
#include <mc_rtc/ros.h>

if(auto node = mc_rtc::ROSBridge::get_node_handle())
{
  // mc_rtc は ROS サポートあり、かつ ROS マスターが実行中 (ROS1)
}
```

## 推奨事項

#### リアルタイム処理を行う箇所でROSのコードを使用しない

（mc\_rtcで）ROSを使用する際に最も問題となるのは、リアルタイム処理がサポートされていないことです。そのため、以下の場所ではROSの関数を使用しないことを強く推奨します。

1. `reset()`関数
2. `run()`関数

これらの関数はいずれも、リアルタイムループ内か、リアルタイム性が要求される場面で実行されます。ROSによる操作をすべて別のスレッドで行わせることを推奨します。本ドキュメントでは同期に関する問題については扱いませんが、C++11以降では同期に関するさまざまなツールが用意されています。
