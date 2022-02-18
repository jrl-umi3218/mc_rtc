{% comment %}FIXME Some comments are not translated {% endcomment %}

[全般的な設定]({{site.baseurl}}/tutorials/introduction/configuration.html)でファイルのロギングを有効あるいは無効にできます。ただし、コントローラーの実装ではロギングオプションについて気にする必要はありません。コントローラーでは、単一のインターフェイスからファイルのロギング操作を行えます。各コントローラーは専用のログを使用し、コンロトーラーが切り替わると新しいログが作成されます。シンボリックリンクをサポートするプラットフォームでは、使い勝手の観点から、最後に作成されたログを指すように`latest`ログファイルが作成されます。

デフォルトでは、どのコントローラーでも以下の情報がロギングされます。
- 反復時間（0で始まり、設定されたタイムステップごとに増加）
- メインロボットのエンコーダー値（ロボットで参照される関節の順に保存）
- （四元数で表された）フリーフライヤーの向きと（コントローラーで参照される）メインロボットの位置
- メインロボットの関節コマンド（コントローラーで参照。ロボットで参照される関節の順に保存）
- 関節のトルク（ロボットで参照される関節の順に保存）
- 力覚センサーの測定値（トルクと力として保存）
- フリーフライヤーの位置センサーの値（一部のインターフェイスのみ）
- 向きセンサーの値（ロール、ピッチ、ヨー）
- フリーフライヤーの直線速度センサーの値（一部のインターフェイスのみ）
- 角速度センサーの値
- 加速度センサーの値

### ログエントリを追加する

コードにログエントリを追加するには、以下のようにします。

```cpp
logger().addLogEntry("entry_name", [this]() { /* compute, say x; */ return x; });
```

コントローラーで`run()`メソッドが実行されると、用意されたコールバック関数が実行されます。

コントローラーがそのままの形で操作できるほとんどの型（`sva::PTransformd`や`sva::ForceVecd`など）と文字列をログに記録できます。また、これらの型のベクトルもログに記録できます。

通常、`addLogEntry`を呼び出した場合、それと対になる`removeLogEntry`も呼び出す必要があります。例:

```cpp
logger().removeLogEntry("entry_name");
```

#### ロギングソースを指定する

以下のようにロギングソースを指定できます。

```cpp
// In this example, "this" is the source
logger().addLogEntry("entry_A", this, [this]() { return a; });
logger().addLogEntry("entry_B", this, [this]() { return b; });
logger().addLogEntry("entry_C", this, [this]() { return c; });

// You can also group calls with the same source
logger().addLogEntries(this,
                       "entry_A", [this]() { return a; },
                       "entry_B", [this]() { return b; },
                       "entry_C", [this]() { return c; });
```

その後、すべてのエントリを一度に削除できます。

```cpp
logger().removeLogEntries(this);

// This would yield the same result but you would need to keep the addLogEntry and removeLogEntry call in sync
logger().removeLogEntry("entry_A");
logger().removeLogEntry("entry_B");
logger().removeLogEntry("entry_C");
```

#### ロギングのメンバーとメソッド

上記のように、`this`インスタンスに対して呼び出されたメンバーあるいはメソッドを読み込むのが一般的であり、簡単に実現できます。この方法による主なメリットは、以下に示すようなパフォーマンスの問題を回避できることです。

```cpp
struct MyObject
{
  Eigen::Vector3d data_;
  sva::PTransformd something_;

  Eigen::Vector6d computeVector();

  const sva::PTransformd & referenceBody();
  void referenceBody(const std::string &);

  void addToLogger(mc_rtc::Logger & logger)
  {
    // This is the actual syntax required in C++11
    logger.addLogEntry<decltype(&MyObject::data_), &MyObject::data_>("data", this);
    // We have the following macro helper
    MC_RTC_LOG_HELPER("something", something_);
    // Also works with methods
    MC_RTC_LOG_HELPER("computeVector", computeVector);
    // But for overloaded methods we must use another macro to work-around ambiguity issues
    MC_RTC_LOG_GETTER("referenceBody", referenceBody);
  }
};
```

`MC_RTC_LOG_HELPER`と`MC_RTC_LOG_GETTER`は、以下の2つを前提としています。

- ロギングソースは`this`とする
- データの追加先となるロガーインスタンスは`logger`とする

#### Pythonインターフェイス

Pythonインターフェイスも非常によく似ています。

```python
# Add a log entry
self.logger().addLogEntry("my_entry", lambda: self.data)
# Remove data
self.logger().removeLogEntry("my_entry")
# Using partial to invoke a controller method
self.logger().addLogEntry("my_entry", partial(self.get_data))
```

#### パフォーマンスに関する問題

デフォルトでは、ラムダ式は値渡しで値を返します。可能であれば、保存するオブジェクトへのconst参照渡しで値を返すようにするとよいでしょう。

```cpp
logger().addLogEntry("entry", [this]() -> const sva::PTransformd & { return trans_; }
```

### ログを取得する

コントローラーでは、以下のように、どのログが最新なのかを示すメッセージが表示されます。

```console
Will log controller outputs to "/tmp/mc-control-MyControllerName-DATE.bin"
```

シンボリックリンクがサポートされているシステムでは、`/tmp/mc-control-MyControllerName-latest.bin`から最新のログを見ることもできます。

### ログを使用する

コントローラーによって生成されるログを使用する方法については、以下のページを参照してください。

- {% include link_tutorial.html category="tools" tutorial="mc_log_utils" %}
- {% include link_tutorial.html category="tools" tutorial="mc_log_ui" %}
- {% include link_tutorial.html category="tools" tutorial="mc_log_visualization" %}
