## ソースからビルドする（スクリプトを使用しない場合）

現在、他のプラットフォームでソースからビルドする方法についてドキュメント化されたものがあまりありません。この方法にチャレンジする場合、mc\_rtcをビルドするのに以下のパッケージが必要となります。
- [CMake](https://cmake.org/) 3.1以降
- [Boost](https://www.boost.org/) 1.49以降
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) 3.2以降
- [tinyxml2](https://github.com/leethomason/tinyxml2)
- [GEOS](https://trac.osgeo.org/geos)（C++バインディングが付属）
- [LTDL](https://www.gnu.org/software/libtool/manual/html_node/Libltdl-interface.html) （Windowsユーザーの場合は不要）
- [nanomsg](https://github.com/nanomsg/nanomsg)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [spdlog](https://github.com/gabime/spdlog/) 1.6.1以降
- [hpp-spline](https://github.com/humanoid-path-planner/hpp-spline)
- [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg)
- [RBDyn](https://github.com/jrl-umi3218/RBDyn)
- [eigen-qld](https://github.com/jrl-umi3218/eigen-qld)
- [eigen-quadprog](https://github.com/jrl-umi3218/eigen-quadprog)
- [eigen-lssol](https://gite.lirmm.fr/multi-contact/eigen-lssol) （オプション）
- [sch-core](https://github.com/jrl-umi3218/sch-core)
- [Tasks](https://github.com/jrl-umi3218/Tasks)
- [mc_rtc_data](https://github.com/jrl-umi3218/mc_rtc_data)
- [state-observation](https://github.com/jrl-umi3218/state-observation)

mc\_rtcには、ロボットの状態をROSトピックとしてパブリッシュすることを可能にし、ROSのツール（RVizなど）との統合を容易にするためのROSプラグインも用意されています。これをビルドするには以下のものが必要です。

 * [mc_rtc_msgs](https://github.com/jrl-umi3218/mc_rtc_msgs)

Pythonバインディングを取得したい場合、以下のものが必要です。
 * [Cython](http://cython.org/) 0.2以降
 * [python-pip]()
 * [python-numpy]()
 * [python-nose]()
 * [python-coverage]()
 * [Eigen3ToPython](https://github.com/jrl-umi3218/Eigen3ToPython)
 * [sch-core-python](https://github.com/jrl-umi3218/sch-core-python)

mc\_rtcのツールを実行するには、さらに以下のPythonライブラリが必要です。
 * [python-git]() (pip name: `GitPython`)
 * [python-pyqt5]()

以下のパッケージは必須ではありませんが、追加の機能が使用できるようになります。
- [ROS](http://www.ros.org/)
- [mc\_rtc\_ros](https://github.com/jrl-umi3218/mc_rtc_ros)

ビルド時に`roscpp`を使用できる場合、`tf2_ros`と`sensor_msgs`も必要になります。これを使用すると、`ROS`フレームワークと`mc_rtc`がさらに統合され（ロボットの状態のパブリッシュなど）、コントローラーでROSの機能を使用できるようになります（`mc_rtc`内のどこでも使える`ros::NodeHandle`インスタンスが提供されます）。

## はじめましょう

mc_rtcのインストールが完了したら、[次のセクション]({{site.baseurl}}/tutorials/introduction/configuration.html)に進んでください。
