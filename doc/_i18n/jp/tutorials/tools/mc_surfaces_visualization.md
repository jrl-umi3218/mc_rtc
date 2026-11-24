`mc_surfaces_visualization`は、ロボットの表面をRVizで表示するツールです。{% link mc_rtc_ros %}パッケージに含まれています。 これは、利便性のために提供されている [mc_robot_visualization]({{site.baseurl}}/tutorials/tools/mc_robot_visualization.html) のサブツールです。

### 可視化ツールを起動する

ツールの起動方法は以下の通りです：

```bash
$ mc_surfaces_visualization JVRC1
```

`mc_surfaces_visualization` は `mc_rtc gui` のみにパブリッシュしますが、`mc_surfaces_visualization_ros` は `mc_rtc gui` と `ros`（RViZ）の両方にパブリッシュします。
Debian パッケージからインストールした場合、ROS 非対応バージョンは `mc-rtc-utils` によって提供され、ROS 対応バージョンは `ros-<distro>-mc-rtc-utils` によって提供されます。

プログラムの引数は `MainRobot` のエントリと同じものを指定します。エイリアスも扱えます。例えば

```bash
# 引数のベクトルを指定する
$ mc_surfaces_visualization_ros env `rospack find mc_env_description` ground
# またはエイリアス
$ mc_surfaces_visualization_ros env/ground
```

mc_rtc GUIを起動した後、これが表示されるはずです：

<img src="{{site.baseurl_root}}/assets/tutorials/tools/img/mc_surfaces_visualization.png" alt="mc_surfaces_visualization in action" class="img-fluid" />

3次元表示では以下のように表示されます。

- 平面は緑で表示され、面の法線の向きが青い矢印で示されます（スクリーンショットの足の部分を参照）。
- 円筒形の表面は、緑の円筒で表示されます（このスクリーンショットには表示されていません）。
- グリッパーの表面は、グリッパーの先端方向を示すフレームの法線の向きを示す青い矢印で表されます。

インターフェースを通じて表示する表面を簡単に選択できます
