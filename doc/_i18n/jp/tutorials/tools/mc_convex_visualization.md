`mc_convex_visualization`は、ロボットの凸領域をRVizで表示するツールです。{% link mc_rtc_ros %}パッケージに含まれています。

### 可視化ツールを起動する

以下のようにツールを起動します。

```bash
$ roslaunch mc_convex_visualization display.launch robot:=JVRC1
```

引数`robot`には、`mc_rbdyn::RobotLoader::get_robot_module`関数に渡すのと同じものを指定する必要があります。なお、この引数ではロボットの別名を使用できます。例:

```bash
# 引数のベクトルとして与える方法
$ roslaunch mc_convex_visualization display.launch robot:="[env, `rospack find mc_env_description`, ground]"
# エイリアスを使用する方法
$ roslaunch mc_convex_visualization display.launch robot:=env/ground
```

このツールを起動すると、以下のように表示されます。

<img src="{{site.baseurl_root}}/assets/tutorials/tools/img/mc_convex_visualization.png" alt="mc_convex_visualization in action" class="img-fluid" />

赤い枠で囲まれた領域にあるチェックボックスをオン・オフすることで、選択した凸領域の表示・非表示を切り替えられます。
