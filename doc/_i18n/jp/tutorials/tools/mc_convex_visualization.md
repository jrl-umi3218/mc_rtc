`mc_convex_visualization` は mc_rtc GUI アプリケーションでロボットの凸部を可視化するツールです。 これは、利便性のために提供されている [mc_robot_visualization]({{site.baseurl}}/tutorials/tools/mc_robot_visualization.html) のサブツールです。

### ビジュアライゼーションの起動

ツールの起動方法は以下の通りです：

```bash
$ mc_convex_visualization JVRC1
```

`mc_convex_visualization` は `mc_rtc gui` のみにパブリッシュしますが、`mc_convex_visualization_ros` は `mc_rtc gui` と `ros`（RViZ）の両方にパブリッシュします。
パッケージからインストールした場合、ROS 非対応バージョンは `mc-rtc-utils` によって提供され、ROS 対応バージョンは `ros-<distro>-mc-rtc-utils` によって提供されます。

プログラムの引数は `MainRobot` のエントリと同じものを指定します。エイリアスも扱えます。例えば

```bash
# 引数のベクトルを指定する
$ mc_convex_visualization env `rospack find mc_env_description` ground
# またはエイリアス
$ mc_convex_visualization env/ground
```

mc_rtc GUIを起動した後、これが表示されるはずです：

<img src="{{site.baseurl_root}}/assets/tutorials/tools/img/mc_convex_visualization.png" alt="mc_convex_visualization in action" class="img-fluid" />

インターフェースを通じて表示する凸形状を簡単に選択できます。
