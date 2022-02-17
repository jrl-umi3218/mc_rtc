{% comment %} FIXME Outdated translation: missing mc_bin_utils extract --keys and mc_bin_perf section {% endcomment %}

このドキュメントでは、`~/my_log.bin`にログを記録することを前提として説明します。

### `mc_bin_utils`: 汎用操作ツール

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

#### `mc_bin_utils extract`

`mc_bin_utils extract`には2つの操作モードが用意されています。

最初の形式は、ログ内において指定したキーが存在する箇所をすべて抽出します。

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --key com_task_target
```

上記のコマンドを実行すると、`com_task_target`というキーが存在するセクションごとにログが生成されます。例えば、5秒のマークから15秒のマークの間にこのキーが存在し、30秒のマークから60秒のマークの間にもこのキーが存在する場合、`my_log_out_1.bin`と`my_log_out_2.bin`が生成されます。

2番目の形式は、時間`t`の入力に基づいて、指定された時間範囲を直接抽出します。

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --from 50 --to 100
```

この例では、時間`t`が50秒から100秒の間のログが`my_log_out.bin`として生成されます。`from`あるいは`to`のいずれかを省略できます。その場合、出力されるログは、0秒から始まり（前者を省略した場合）、ログの最後で終了します（後者を省略した場合）。

#### `mc_bin_utils convert`

このツールでは、`.bin`ログを以下に示す3つの形式のいずれかに変換できます（このうちの2つについては、ROSではサポートされていません）。

- `.flat`は、`mc_log_ui`で使用される実際の形式です。同じログを頻繁に開く場合、この形式に変換しておくと時間を節約できます。
- `.csv`は、よく知られたデータ形式で、数多くのツール（特に、MATLABやExcel）に読み込むことができ、外部の共同作業者にデータを共有するのに便利です。
- `.bag`は、`rosbag`で使用される形式です。

### Pythonでログを開く

Pythonでログを開くこともできます。

```python
import mc_log_ui
log = mc_log_ui.read_log('/tmp/mylog.bin')
```

`log`は、mc_log_uiのツリーに表示されるエントリをキーとして持ち（例えば、`v3d`という名前の`Eigen::Vector3d`をログに記録した場合、`v3d_x`、`v3d_y`、`v3d_z`がキーとなります）、Numpy配列を値として持つPythonの辞書であり、これを使えばさまざまなことが行えます。　　

### `mc_rtc::FlatLog`クラス

`mc_rtc::FlatLog`クラスは、`mc_rtc_utils`ライブラリで定義されています。このクラスでは、ログを開き、元の型でデータを操作することができます。例えば、力覚センサーの測定値を`sva::ForceVecd`オブジェクトとして記録した場合、`mc_rtc::FlatLog`を使用してこの値をその形式のまま取得できます。使用方法とAPIの詳細については、クラスのドキュメントを参照してください。C++でのみ使用できます。.
