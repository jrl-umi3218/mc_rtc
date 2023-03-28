mc_rtc_tickerの使用法を {% include link_tutorial.html category="introduction" tutorial="running-a-controller" %} チュートリアルで紹介しました。このセクションでは、mc_rtc_tickerの追加機能、リプレイオプション、および「Replay」プラグインの使用について説明します。

#### mc_rtc_tickerのオプション

mc_rtc_tickerには、次のオプションがあります。

{% highlight bash %}
$ mc_rtc_ticker --help
mc_rtc_ticker options:
--help ヘルプメッセージを表示します
-f [ --mc-config ] arg mc_rtcに与えられる設定
-S [ --step-by-step ] ステップバイステップモードでTickerを開始します
--run-for arg 指定された時間（秒）間Tickerを実行します
-s [ --no-sync ] Ticker時間をリアルタイムと同期させません
-r [ --sync-ratio ] arg 同期目的のSim / Real比率
-l [ --replay-log ] arg リプレイログ
-m [ --datastore-mapping ] arg ログキーからデータストアへのマッピング
-g [ --replay-gui-inputs-only ] GUI入力のみをリプレイします
-e [ --exit-after-replay ] ログリプレイ後に終了します
--replay-outputs 出力リプレイを有効にします（コントローラをオーバーライドします）
{% endhighlight %}

##### `-f [file]`/`--mc-config [file]`

このオプションを使用すると、mc_rtcを構成するために使用されるmc_rtc構成ファイルをオーバーライドできます。提供された`file`は、通常のファイルの後にmc_rtcを構成するために使用されます。

##### `-S`/`--step-by-step`

このオプションは、ステップバイステップモードでコントローラを開始します。このモードでは、コントローラは、一定数のイテレーションのステップが指示されるまで実行されません。これはGUIを介して行うことができます。

##### `--run-for [time]`

指定された`time`の期間（秒）Tickerを実行して終了します。

##### `-s`/`--no-sync`

通常、ティッカーはリアルタイムクロックとシミュレーション時間を同期させ、リアルタイムで1秒経過すると、シミュレーションでも1秒経過するようにします。このオプションを使用すると、同期機構が無効になり、シミュレーション時間は可能な限り速く実行されます。

##### `-r [ratio]`/`--sync-ratio [ratio]`

ターゲットシミュレーション時間/リアルタイムの比率を制御します。たとえば、値が0.5の場合、シミュレーションで1秒経過すると、リアルタイムで2秒経過します。ただし、比率が2の場合、シミュレーションで1秒経過すると、リアルタイムで0.5秒が経過します（CPUがその速度で実行できる場合）。

#### リプレイ機能

以下のオプションは、リプレイ機能に関連しています。リプレイモードでは、ログが読み込まれ、コントローラにいくつかのデータを提供するために使用できます。

##### `-l [log]`/`--replay-log[log]`

このオプションは、指定された `log` 引数をログとして使用してリプレイモードを有効にします。デフォルトでは、これによりすべてのセンサー入力とGUIの相互作用が再生され、プラグイン、観測パイプライン、およびこれらの入力を使用してコントローラが実行されます。リプレイモードの動作は、次のオプションで微調整できます。

##### `-m [yaml]`/`--datastore-mapping [yaml]`

このオプションを使用すると、ログからデータストアにデータを転送するためのログからデータストアへのマッピングを指定できます。リプレイは、ログからデータを取得してデータストアに入れます。データストアに表示されるデータの種類は、ログのデータの種類に依存します。

例えば:

{% highlight yaml %}
# キーはログのエントリーであり、値はデータストアのエントリーです。
ff: Log::FloatingBaseControl
ff_real: Log::FloatingBaseReal
{% endhighlight %}

この機能の目的は、リプレイ中にコントローラに接続されていないシステムからの入力データを再生できるようにすることです。

##### `-g`/`--replay-gui-inputs-only`

センサーの入力とGUIの入力の両方を再生する代わりに、GUIの入力のみを再生します。

##### `-e`/`--exit-after-replay`

再生終了時に、Tickerを終了します。デフォルトの動作は、ログの最後でステップバイステップモードを有効にすることです。

##### `--replay-outputs`

このオプションは、コントローラをループから削除し、以前のコントローラの出力を再生します。

#### Replayプラグインの使用

TickerのReplayモードに加えて、Replayプラグインも利用可能です。プラグインは、mc_rtcの構成またはコントローラの構成に以下のエントリを追加して有効化および構成することができます。

{% highlight yaml %}
Plugins: [Replay]
Replay:
  log: /path/to/log.bin
  with-inputs: true
  with-gui-inputs: true
  with-outputs: false
  with-datastore-config: /path/to/datastore-to-replay.yaml
{% endhighlight %}

このReplayプラグインの最も一般的な構成は、以前のログからGUI入力を再生してシミュレーション環境で使用することです。

#### Replayがアクティブであることを検出する

TickerのReplayまたはReplayプラグインを介してコントローラで再生が実行されている場合、コントローラには`Replay::Log`エントリがあり、再生されている`mc_rtc::log::FlatLog`オブジェクトへの共有ポインタが含まれています。
