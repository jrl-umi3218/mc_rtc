`mc_robot_visualization` は、ロボット全体のモジュールおよびその状態を `mc_rtc gui` または ROS（RViZ）で可視化するためのツールです。

### ビジュアライゼーションの起動

ツールは次のように起動します：

```bash
$ mc_robot_visualization[_ros] JVRC1
```

`mc_robot_visualization` は `mc_rtc gui` のみにパブリッシュしますが、`mc_robot_visualization_ros` は `mc_rtc gui` と `ros`（RViZ）の両方にパブリッシュします。
Debian パッケージからインストールした場合、ROS 非対応バージョンは `mc-rtc-utils` によって提供され、ROS 対応バージョンは `ros-<distro>-mc-rtc-utils` によって提供されます。

プログラムへの引数は、`MainRobot` エントリで使用するものと同じである必要があります。エイリアスにも対応しています。例えば：

```bash
# 引数のベクトルを指定
$ mc_robot_visualization_ros env `rospack find mc_env_description` JVRC1
# またはエイリアス
$ mc_robot_visualization_ros env/JVRC1
```

3D 表示では：

- ロボットモデルが現在の姿勢と関節構成で表示されます。
- リンクや関節はロボットの URDF に従って可視化されます。
- 追加のオーバーレイとして、センサーデータ、コリジョン形状、その他の情報が設定に応じて表示される場合があります。これらのオーバーレイは GUI から、または簡易版の [mc_surfaces_visualization]({{site.baseurl}}/tutorials/tools/mc_surfaces_visualization.html) や [mc_convex_visualization]({{site.baseurl}}/tutorials/tools/mc_convex_visualization.html) でも可視化できます。

インターフェースを通じて、表示するロボットの部位を簡単に選択できます。
