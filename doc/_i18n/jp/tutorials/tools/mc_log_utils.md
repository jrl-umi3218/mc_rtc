このドキュメントでは、`~/my_log.bin`にログを記録することを前提として説明します。

## `mc_bin_utils`: 汎用操作ツール

`mc_bin_utils`では、バイナリログに対してさまざまな操作が行えます。

```bash
$ mc_bin_utils --help
mc_bin_utils is a command-line tool to work with mc_rtc bin logs

Available commands:
    show      Display information about the log
    split     Split a log into N part
    extract   Extra part of a log
    convert   Convert binary logs to various formats

Use mc_bin_utils <command> --help for usage of each command
```

ここでは、`extract`コマンドと`convert`コマンドについて説明します。`show`と`split`については、説明の必要はないためここでは割愛します。

### `mc_bin_utils extract`

`mc_bin_utils extract`には3つの操作モードが用意されています。

#### 抽出するデータのキーを `--key` で指定する操作モード

最初の操作モードは、ログ内において指定したキーが存在する箇所をすべて抽出するモードです。

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --key com_task_target
```

上記のコマンドを実行すると、`com_task_target`というキーが存在するセクションごとにログが生成されます。例えば、5秒のマークから15秒のマークの間にこのキーが存在し、30秒のマークから60秒のマークの間にもこのキーが存在する場合、`my_log_out_1.bin`と`my_log_out_2.bin`が生成されます。

#### 時間範囲を `--from --to` で指定する操作モード

2番目の操作モードは、時間`t`の入力に基づいて、指定された時間範囲を直接抽出します。

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --from 50 --to 100
```

この例では、時間`t`が50秒から100秒の間のログが`my_log_out.bin`として生成されます。`from`あるいは`to`のいずれかを省略できます。その場合、出力されるログは、0秒から始まり（前者を省略した場合）、ログの最後で終了します（後者を省略した場合）。

#### 抽出する複数のデータを `--keys` で指定する操作モード

3つ目の操作モードは指定した複数のキーを用いてデータを抽出するモードです。

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --keys RightFootForceSensor com_target com_eval
```

この例では、指定したキーに対応するデータのみを含む `.bin` ファイルが生成されます。なお `t` キーに対応するデータは常に含まれます。ログファイルに指定したキーに対応するデータがいずれも存在しない範囲がある場合、複数のファイルが生成されます。

このコマンドではワイルドカードも使用することができます。

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --keys com_*
```

この例では `com_` で始まる全てのキーに対応するデータが抽出されます。

### `mc_bin_utils convert`

このツールでは、`.bin`ログを以下に示す3つの形式のいずれかに変換できます（ROSサポートが無効になっている場合は2つの形式のみ）。

- `.flat`は、`mc_log_ui`で使用される実際の形式です。同じログを頻繁に開く場合、この形式に変換しておくと時間を節約できます。
- `.csv`は、よく知られたデータ形式で、数多くのツール（特に、MATLABやExcel）に読み込むことができ、外部の共同作業者にデータを共有するのに便利です。
- `.bag`は、`rosbag`で使用される形式です。

以下のスタンドアロンツールは決まったファイル形式間の変換に利用できます。

- `mc_bin_to_flat` は `.bin` ファイルを `.flat` ファイルに変換します。
- `mc_bin_to_log` は `.bin` ファイルを `.csv` ファイルに変換します。
- `mc_bin_to_rosbag` は `.bin` ファイルを `.bag` ファイルに変換します。

## `mc_bin_perf` を用いたパフォーマンスの確認

`mc_bin_perf` を使ってmc_rtcのパフォーマンスに関する簡単な統計的データを確認することができます。このコマンドはログファイルから `perf_` で始まるデータを抽出し、それらの平均値や標準偏差、最大値、最小値を計算、表示します。

実行例

```bash
$ mc_bin_perf /tmp/mc-control-CoM-latest.bin
----------------------------------------------------------------
|                     |  Average |    StdEv |      Min |   Max |
----------------------------------------------------------------
|       ControllerRun |    0.451 |   0.0639 |    0.378 |  1.14 |
|       FrameworkCost |     7.64 |     2.81 |     2.55 |  52.8 |
|           GlobalRun |    0.488 |   0.0703 |    0.401 |  1.18 |
|                 Gui |   0.0035 |  0.00941 | 0.000543 | 0.322 |
|                 Log |  0.00732 |  0.00476 |  0.00563 | 0.163 |
|        ObserversRun | 6.79e-05 | 0.000215 |  2.4e-05 | 0.014 |
|   Plugins_ROS_after |   0.0137 |   0.0124 |  0.00202 | 0.172 |
| SolverBuildAndSolve |    0.422 |   0.0607 |    0.354 |  1.11 |
|         SolverSolve |     0.32 |   0.0498 |    0.268 |     1 |
----------------------------------------------------------------
```

`FrameworkCost` を除く全ての時間の単位はミリ秒です。

表示されるエントリに関する簡単な説明は以下の通りです。

- `GlobalRun` は `mc_control::MCGlobalController::run()` の実行にかかった総時間を表します。すなわち、mc_rtc内部のグローバルプラグイン、オブザーバーパイプライン、コントローラ、ロガー、GUIの全ての実行にかかった時間です。これは`Plugins_* + ObserversRun + ControllerRun + Gui + Log`とほぼ同じ値になります。
- `ControllerRun` はコントローラの実行にかかった時間です。これは`SolverBuildAndSolve + コントローラ内部のコード実行に要した時間` とほぼ同じ時間になります。
- `SolverBuildAndSolve` と `SolverSolve` は `Tasks` の実行に要した時間です。`SolverSolve` は `SolverBuildAndSolve` に含まれ、QP問題を解くのにかかった時間を測っていることになります。
- `FrameworkCost` は `Tasks` 外でかかった時間の比率、すなわち `(GlobalRun - SolverBuildAndSolve) / GlobalRun` です。

## Pythonでログを開く

Pythonでログを開くこともできます。

```python
import mc_log_ui
log = mc_log_ui.read_log('/tmp/mylog.bin')
```

`log`は、mc_log_uiのツリーに表示されるエントリをキーとして持ち（例えば、`v3d`という名前の`Eigen::Vector3d`をログに記録した場合、`v3d_x`、`v3d_y`、`v3d_z`がキーとなります）、Numpy配列を値として持つPythonの辞書であり、これを使えばさまざまなことが行えます。　　

## `mc_rtc::FlatLog`クラス

`mc_rtc::FlatLog`クラスは、`mc_rtc_utils`ライブラリで定義されています。このクラスでは、ログを開き、元の型でデータを操作することができます。例えば、力覚センサーの測定値を`sva::ForceVecd`オブジェクトとして記録した場合、`mc_rtc::FlatLog`を使用してこの値をその形式のまま取得できます。使用方法とAPIの詳細については、クラスのドキュメントを参照してください。C++でのみ使用できます。.
