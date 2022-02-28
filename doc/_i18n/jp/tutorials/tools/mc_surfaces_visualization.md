`mc_surfaces_visualization`は、ロボットの表面をRVizで表示するツールです。{% link mc_rtc_ros %}パッケージに含まれています。

### 可視化ツールを起動する

以下のようにツールを起動します。

```bash
$ roslaunch mc_surfaces_visualization display.launch robot:=JVRC1
```

引数`robot`には、`mc_rbdyn::RobotLoader::get_robot_module`関数に渡すのと同じものを指定する必要があります。なお、この引数ではロボットの別名を使用できます。例:

```bash
# 引数のベクトルとして与える方法
$ roslaunch mc_surfaces_visualization display.launch robot:="[env, `rospack find mc_env_description`, ground]"
# エイリアスとして与える方法
$ roslaunch mc_surfaces_visualization display.launch robot:=env/ground
```

このツールを起動すると、以下のように表示されます。

<img src="{{site.baseurl_root}}/assets/tutorials/tools/img/mc_surfaces_visualization.png" alt="mc_surfaces_visualization in action" class="img-fluid" />

3次元表示では以下のように表示されます。

- 平面は緑で表示され、面の法線の向きが青い矢印で示されます（スクリーンショットの足の部分を参照）。
- 円筒形の表面は、緑の円筒で表示されます（このスクリーンショットには表示されていません）。
- グリッパーの表面は、グリッパーの先端方向を示すフレームの法線の向きを示す青い矢印で表されます。

赤い枠で囲まれた領域にあるチェックボックスをオン・オフすることで、選択した表面の表示・非表示を切り替えることができます。
