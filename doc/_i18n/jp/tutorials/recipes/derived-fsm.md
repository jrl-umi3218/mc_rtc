有限オートマトンをさらに活用する一般的なレシピとして、有限オートマトンコントローラーの派生が挙げられます。これにより以下のことが可能となります。

- データ（タスク、制約条件、パラメーターなど）を状態間で簡単に共有できる
- デフォルトの動作を簡単に実装できる（`Parallel`状態と`Meta`状態を使用）

これを実現するには、以下のようにする必要があります。

- `mc_control::MCController`ではなく、`mc_control::fsm::Controller`からコントローラーを派生させる必要があります。
- 独自のリセット関数で、最初に`mc_control::fsm::Controller::reset`を呼び出します。または、`mc_control::MCController::reset`を呼び出し、独自の初期化処理を実行してから、`mc_control::fsm::Controller::reset`を呼び出します。初期状態において、リセット関数内で正しく初期化されたコントローラーインスタンスからデータを使用する必要がある場合、後者の方法を使用する必要があります。
- ライブラリを`mc_control_fsm`とリンクさせる必要があります。

コントローラー用に実装した状態も、通常の`State`インターフェイスの仕様に従う必要があります。独自のコントローラークラスにアクセスしたい場合は、渡された`Controller`インスタンスをキャストする必要があります。

```cpp
bool MyState::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MyController&>(ctl_);
  // 以降ctlを使って個別のコントローラにアクセスできるようになる
}
```

上記のコントローラーアクセスを簡素化する方法として、`mc_control::fsm::State`クラスを派生させて、状態内で常に独自のコントローラークラスにアクセスさせるというやり方があります。以下のスニペットを参照してください。

```cpp
struct State : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & controller) final
  {
    start(static_cast<Controller&>(controller));
  }
  bool run(mc_control::fsm::Controller & controller) final
  {
    return run(static_cast<Controller&>(controller));
  }
  void teardown(mc_control::fsm::Controller &) final
  {
    teardown(static_cast<Controller&>(controller));
  }

  virtual void start(Controller &) = 0;
  virtual bool run(Controller &) = 0;
  virtual void teardown(Controller &) = 0;
};
```

そして、`mc_control::fsm::State`の代わりにこの`State`クラスを継承します。

この方法を使って、動作が異なる有限オートマトンを実装することができます。以下の例を参照してください。

```cpp
struct State : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & controller) override
  {
    controller_ = &static_cast<Controller&>(controller);
    start();
  }
  bool run(mc_control::fsm::Controller &) override
  {
    if (checkTransitions())
    {
      return true;
    }
    runState();
    return false;
  }
  void teardown(mc_control::fsm::Controller &) override
  {
    teardown();
  }
  Controller & controller()
  {
    return *controller_;
  }

  // 頻繁に利用されるショートカット関数を以下に追加
  mc_rtc::Logger & logger()
  {
    return controller_->logger();
  }

  virtual bool checkTransitions() = 0;
  virtual void runState() = 0;
  virtual void start() = 0;
  virtual void teardown() = 0;

protected:
  Controller * controller_ = nullptr;
};
```

## さあ始めましょう

`mc_rtc_new_fsm_controller`ツールでは、有限オートマトンコントローラーから派生したコントローラーを新たに作成できます。このツールは、`mc_rtc_new_controller`と同じように使用できます。

GitHubの[mc-rtc/new-fsm-controller](https://github.com/mc-rtc/new-fsm-controller)テンプレートを使用することもできます。
