`mc_log_visualization`は、ログをRVizで3次元表示するツールです。{% link mc_rtc_ros %}パッケージに含まれています。

### 可視化ツールを起動する

以下のようにリプレイを起動します。

```bash
$ roslaunch mc_log_visualization log_visualizer.launch robot:=JVRC1 log:=/tmp/mc-control-MyController-latest.bin
```

<em>注: ROSの制約により、ログの<strong>絶対パス</strong>を指定する必要があります。</em>

次に、可視化ツールを起動します。

```bash
$ roslaunch mc_rtc_ticker display.launch
```

以下のようにインターフェイスが表示されます。

<img src="{{site.baseurl_root}}/assets/tutorials/tools/img/mc_log_visualization.png" alt="mc_log_visualization in action" class="img-fluid" />

### 時間範囲セレクター

このセレクターでは、可視化したい時間範囲を選択できます。現状では、ログに記録されている時間範囲のみを選択できます。

### Add extra information（追加情報の表示）メニュー

このメニューでは、補足情報をインターフェイスに追加できます。データがどのように表示されるかは、ログに記録されているデータの形式に基づき決定されます。

データを表示する前に補足情報を指定できる場合もあります。上記のスクリーンショットの場合、`RightFootForceSensor`エントリが選択されているため、どの表面フレームに力を表示するかを指定できます。

補足情報を追加すると、その情報が3次元環境に表示され（表示可能な場合）、数値データが`Log visualizer - Extra data`タブに表示されます。このタブでは、表示されているデータを非表示にすることも可能です。

補足データの選択状態は、ログ可視化ツールを次に起動するときも維持されます。
