`mc_rtc` コントローラは通常、厳格なタイミング要件を持つリアルタイムスレッド内で動作しています。つまり、`MCGlobalController::run()` は計算 `Timestep`（通常は `2ms` または `5ms`）よりも一貫して速く結果を提供することが求められます。このタイムステップよりも遅くなった計算実行を「ミスイテレーション（missed iteration）」と呼びます。これは、ロボットのハードウェア上で動作する場合、低レベルハードウェアの扱い方によっては重大な影響を及ぼす可能性があります。ミスイテレーションの一般的な症状は、ジッター、関節のノイズ、ぎこちない動きなどです。イテレーションがミスされると、ロボットに送信されるコマンドは前回のイテレーションのものがそのまま維持されます：

- **位置制御の場合**：モータは現在位置で急停止するように指示され、新しいコマンドが届くと再び急始動します（ジッターやアクチュエータの損傷の原因）。
- **速度制御の場合**：速度を0に設定する（位置制御と同じ結果）か、前回計算した速度を維持する（新しいコマンドが届くまでロボットは最新の速度で動き続ける）かの選択肢があります。
- **トルク制御の場合**：トルクを0にすることはできないため、前回のトルクを維持するしかありません。

さらに、多くのアルゴリズムはセンサデータが `timestep` で定義された一定のレートで受信されることを前提としています。

いずれにせよ、特に連続して複数回のイテレーションをミスすることは非常に悪いことであり、絶対に避けるべきです。ミスイテレーションの主な原因を見てみましょう：

- **コントローラが遅すぎる**：コントローラの平均計算時間は制御 `timestep` よりも十分に速い必要があります。平均計算時間は `timestep` の75%未満を目指しましょう。
- **メモリアロケーション**：メモリアロケーションは計算時間に大きなスパイクを引き起こします。コントローラ内でのメモリアロケーションはリアルタイムとは言えません。OSにメモリ割り当てを依頼する必要があるためです。
  *推奨*: 可能な限り避け、事前に大きなメモリを確保しておきましょう。
- **I/O操作**：ディスクへの書き込み/読み込み（特にSSDなしの場合）、ネットワーク操作、ターミナルへの書き込みはすべてリアルタイムではありません。
  *推奨*: これらの操作はリアルタイム制御ループ内で避けるか、制御ループ外のスレッドで実行しましょう。
- **QP**：タスクや制約をQPに追加することは計算時間のスパイクを引き起こす可能性があります。問題サイズに応じて行列を再生成し、ソルバが新しい解を見つける必要があるためです（ウォームスタートできない場合もあります）。
  *推奨*: タスクや制約の追加/削除は避けられませんが、制御ループの重要度が低いタイミングで行いましょう。例えば高速動作中の制約追加は避けてください。
- **アルゴリズムが遅すぎる**：一部のアルゴリズム（例：ビジョン、モデル予測制御など）は制御タイムステップの一部で計算するには遅すぎます。この場合、制御ループを遅らせないようにスレッド化が必要です。

上記の例から明らかなように、複雑なコントローラを書く場合、何らかのスレッド化は避けられません。しかし、これはコードを大幅に複雑化させ、誤解や誤管理がバグ（多くは深刻かつ予測不能な未定義動作）につながりやすい側面です。本チュートリアルの残りでは以下を扱います：

- スレッドとその落とし穴の簡単な解説
- `mc_rtc` におけるスレッド化：安全なこと/危険なこと
- `mc_rtc` が提供するスレッド化支援ツールの紹介

# スレッドの基礎

**スレッド**とは、プログラム内の独立した実行フローです。`mc_rtc` コントローラの文脈では、スレッドはログ記録、ファイルI/O、ビジョン処理など、リアルタイム制御ループには遅すぎる/予測できない処理をオフロードするためによく使われます。これにより、メイン制御ループは厳格なタイミング要件を維持しつつ、遅いタスクは並列で実行できます。

しかし、スレッドを使うと**データ同期の課題**が生じます。複数のスレッドが共有データ（例：センサ値や制御コマンド）にアクセス・変更する必要がある場合、そのアクセスを適切に調整しなければなりません。これを怠ると、スレッドの実行タイミングに依存して結果が変わる「競合状態（race condition）」が発生し、データの不整合や破損につながります。ミューテックスやロックなどの同期プリミティブが役立ちますが、注意が必要です：リアルタイムスレッドでロックを保持すると、他のスレッドがロックを保持している間に遅延やミスイテレーションが発生する可能性があります。

また、**スレッドの生成にはコスト**があります。新しいスレッドを作成するにはOSがリソースを割り当て、スケジューリングを管理する必要があり、制御ループのタイムステップと比べてかなりの時間がかかります（通常は数十マイクロ秒程度ですが、カーネル依存で予測不能です）。そのため、スレッドは事前に作成し再利用するべきで、制御ループ内で頻繁に生成・破棄すべきではありません。代替案としてスレッドプールを使うと、複数のタスクで事前生成したスレッドを再利用できます。

## データ同期：一般的な概要

複数のスレッドが共有データにアクセスする場合、同時にデータを変更しない、または読み書きが干渉しないようにしなければなりません。これを**データ同期**と呼びます。適切な同期がないと、**競合状態**が発生し、プログラムの動作がスレッドの予測不能なタイミングに依存するバグが生じます。リアルタイム制御では、これがセンサデータの破損、不正なコマンド、さらにはロボットの危険な動作につながることもあります。

C++11で最も一般的な同期ツールは `std::mutex` です。ミューテックス（相互排他）は、同時に1つのスレッドだけがクリティカルセクションに入れるようにします。ミューテックスを使うには、共有データにアクセスする前にロックし、アクセス後にアンロックします。C++11の `std::lock_guard` を使うとロック/アンロックを自動管理できます。**注意**：ミューテックスは魔法の道具ではありません。保護したいデータを読み書きするすべてのスレッドでロックする必要があります（コントローラ、プラグインなど両方で）。

### 例：ミューテックスで共有データを保護する

例えば、バックグラウンドスレッドが更新し、リアルタイム制御スレッドが読み取る共有変数 `shared_value` があるとします。

**1. ミューテックスを使った複雑な共有オブジェクト**

```cpp
#include <mutex>
#include <thread>
#include <iostream>

struct SensorData
{
  double position;
  double velocity;
};

SensorData shared_sensor_data{0.0, 0.0};
std::mutex mtx;

void background_thread()
{
  for(int i = 0; i < 10; ++i)
  {
    {
      std::lock_guard<std::mutex> lock(mtx);
      shared_sensor_data.position += 1.0;
      shared_sensor_data.velocity += 0.5;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void control_thread()
{
  for(int i = 0; i < 10; ++i)
  {
    SensorData local_copy;
    {
      std::lock_guard<std::mutex> lock(mtx);
      local_copy = shared_sensor_data;
    }
    std::cout << "Position: " << local_copy.position
              << ", Velocity: " << local_copy.velocity << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
```

**2. 単純なデータにはアトミック型を使う**

整数やブーリアンなど単純な型には、ロックフリー同期のために `std::atomic` を使えます：

```cpp
#include <atomic>
#include <thread>
#include <iostream>

std::atomic<int> shared_counter{0};

void background_thread()
{
  for(int i = 0; i < 10; ++i)
  {
    ++shared_counter; // アトミックインクリメント
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void control_thread()
{
  for(int i = 0; i < 10; ++i)
  {
    int value = shared_counter.load();
    std::cout << "Counter: " << value << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
```

**注意：**
アトミック型は整数やブーリアンなど単純なデータにのみ適しています。複雑なオブジェクトにはミューテックスや専用のロックフリー構造体を使いましょう。

## mc_rtcにおけるデータ同期

`mc_rtc` のデータ構造は**基本的にスレッドセーフではありません**。つまり、`MCGlobalController`、`Robot`、`Task` などの `mc_rtc` データに複数スレッドからアクセス・変更する場合は、未定義動作を避けるために適切な同期が必要です。

以下は、メインスレッドがロボットデータを変更している間に別スレッドがロボットデータを読み取る危険な例です：

### 危険な例

```cpp
#include <mc_rbdyn/Robot.h>
#include <thread>
#include <iostream>

void unsafe_read(mc_rbdyn::Robot * robot)
{
  // この関数は別スレッドでロボットデータを読み取る
  for(int i = 0; i < 100; ++i)
  {
    // 危険：他のスレッドが同時に robot->posW() を変更している可能性あり
    // 結果は未定義動作。正常に動く場合もあれば、クラッシュやゴミ値になることも
    std::cout << "Robot position: " << robot->posW().translation().transpose() << std::endl;
  }
}

void unsafe_example(mc_rbdyn::Robot * robot)
{
  // ロボットデータを読み取るスレッドを開始
  std::thread t(unsafe_read, robot);

  // 一方、メインスレッドはロボットデータを変更
  for(int i = 0; i < 100; ++i)
  {
    // 危険：ロボットの位置を変更
    robot->posW(sva::PTransformd{sva::RotZ(i * mc_rtc::constants::PI), Eigen::Vector3d{i/10., 0, 0}});
  }

  t.join();
}
```

**警告：**
この例は、両方のスレッドが同期なしで同じ `mc_rbdyn::Robot` オブジェクトにアクセス・変更しているため危険です。データ競合や未定義動作の原因となります。

### 安全な例

今度は、`std::mutex` を使って `mc_rbdyn::Robot` オブジェクトへのアクセスを同期した**安全なバージョン**です。これにより同時の読み書きを防ぎ、データ競合を回避します。

```cpp
#include <mc_rbdyn/Robot.h>
#include <thread>
#include <mutex>
#include <iostream>

std::mutex robot_mutex;

void safe_read(mc_rbdyn::Robot * robot)
{
  for(int i = 0; i < 100; ++i)
  {
    std::lock_guard<std::mutex> lock(robot_mutex);
    std::cout << "Robot position: " << robot->posW().translation().transpose() << std::endl;
  }
}

void safe_example(mc_rbdyn::Robot * robot)
{
  std::thread t(safe_read, robot);

  for(int i = 0; i < 100; ++i)
  {
    std::lock_guard<std::mutex> lock(robot_mutex);
    robot->posW(sva::PTransformd{sva::RotZ(i * mc_rtc::constants::PI), Eigen::Vector3d{i/10., 0, 0}});
  }

  t.join();
}
```

これで、`robot` オブジェクトへのすべてのアクセスがミューテックスで保護され、スレッドセーフになります。

## ミューテックスによるデッドロック

**デッドロック**とは、2つ以上のスレッドが互いにリソース（ミューテックスなど）の解放を待ち続け、全スレッドが永久に待機状態になることです。これは、複数のミューテックスを異なる順序でロックする場合によく発生します。

以下の例は、ミューテックスの誤った使い方でデッドロックが発生する様子を示しています：

```cpp
#include <mutex>
#include <thread>
#include <iostream>

std::mutex mutexA;
std::mutex mutexB;

void thread1()
{
  std::lock_guard<std::mutex> lockA(mutexA);
  // 作業のシミュレーション
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::lock_guard<std::mutex> lockB(mutexB); // mutexBを待つ
  std::cout << "Thread 1 acquired both mutexes\n";
}

void thread2()
{
  std::lock_guard<std::mutex> lockB(mutexB);
  // 作業のシミュレーション
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::lock_guard<std::mutex> lockA(mutexA); // mutexAを待つ
  std::cout << "Thread 2 acquired both mutexes\n";
}

void deadlock_example()
{
  std::thread t1(thread1);
  std::thread t2(thread2);
  t1.join();
  t2.join();
}
```

# mc_rtc のスレッド化ユーティリティ

フレームワーク内でよくあるケースに対応するため、`mc_rtc` はスレッド化やデータ同期を支援するユーティリティを提供しています：

- `mc_rtc::threading::AsyncJob`: ジョブを別スレッドで非同期実行する便利な方法を提供します。`std::async` ジョブを作成・管理します。

## AsyncJob

`mc_rtc::threading::AsyncJob` クラスは、ジョブを別スレッドで非同期実行する便利な方法を提供します。内部的には `std::async` を使い、ジョブの完了確認、結果取得、例外処理などのメソッドを提供します。

**利点:**
- 使いやすい
- 入出力の分離が明確

**欠点:**
- ジョブごとにスレッドを生成するため、頻繁かつ短時間のジョブにはコストが高い

### AsyncJob の使い方

`AsyncJob` クラスは CRTP（Curiously Recurring Template Pattern）ベースのクラスで、`mc_rtc` で非同期計算を実行するために使います。ジョブの状態管理、結果取得、ログやGUIとの統合も可能です。

**使い方の手順:**

1. **MyInput クラスを作成**
   ジョブに必要なすべてのデータをコピーで保持します。

2. **MyResult クラスを作成**
   ジョブが生成するすべてのデータをコピーで保持します。

3. **ジョブクラスを定義**
   `mc_rtc::threading::MakeAsyncJob<YourJob, InputType, ResultType>` を継承し、`ResultType computeJob()` メソッドを実装します。

4. **（オプション）ログやGUIの追加**
   `void addToLoggerImpl()` や `void addToGUIImpl()` を実装してカスタムログ/GUI要素を追加できます。

5. **ジョブの作成と利用**
   - 入力をセット（ジョブが実行中でないとき）
   - `startAsync()` で計算を開始
   - 制御ループ内で定期的に `checkResult()` を呼び、状態更新と結果取得
   - `lastResult()` で結果にアクセス

**注意:**
- 入力の変更はジョブが実行中でないときのみ（`running() == false`）行うこと
- 制御ループ内で必ず `checkResult()` を呼び、結果処理と管理を行うこと
- ログやGUIエントリはジョブ破棄時に自動削除されます

**基本例:**

```cpp
// MyAsyncJob.h

struct MyInput
{
  double data; // ジョブへの入力値
};

struct MyResult
{
  double value; // 計算結果
};

struct MyAsyncJob : public mc_rtc::threading::MakeAsyncJob<MyAsyncJob, MyInput, MyResult>
{
  MyResult computeJob()
  {
    MyResult res;
    res.value = input_.data * 2;
    return res;
  }

  void addToLoggerImpl()
  {
    logger_->addLogEntry(loggerPrefix_ + "_my_job_value", this, [this]() { return lastResult_->value; });
  }

  void addToGUIImpl()
  {
    gui_->addElement(this, guiCategory_, mc_rtc::gui::Label("Result", [this]() { return lastResult_->value; }));
  }
};
```

**MyAsyncJob を使ったFSMステートの例:**

```cpp
// SimpleAsyncState.h
#include <mc_control/fsm/State.h>
#include <mc_rtc/threading/AsyncJob.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include "MyAsyncJob.h"

// シンプルなAsyncJobを使ったFSMステート。各結果ごとに再実行
struct SimpleAsyncState : public mc_control::fsm::State
{
  MyAsyncJob job_; // ジョブクラスの生成（ジョブはまだ開始しない）
  int counter_ = 0;
  int maxIterations_ = 5; // 5回で終了

  void configure(const mc_rtc::Configuration & config) override
  {
    // async jobの初期入力をセット
    config("input", job_.input().data);
    config("maxIterations", maxIterations_);
  }

  void start(mc_control::fsm::Controller & ctl) override
  {
    // 最初のasync jobを開始。結果はrun()内でjob_.checkResult()で取得
    job_.startAsync();
    job_.addToLogger(ctl.logger(), name());
    job_.addToGUI(ctl.gui(), {name()});
    counter_ = 0;
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    if(job_.checkResult())
    { // 前回のasync jobが完了したので結果を取得
      const auto & result = *job_.lastResult();
      mc_rtc::log::info("[{}] Async result {}: {}", name(), counter_, result.value);

      counter_++;

      // ジョブが実行中でないので、次の入力を安全に更新できる
      job_.input().data += 1.0;

      if(counter_ < maxIterations_)
      {
        job_.startAsync(); // 次のasync jobを開始
        return false;      // 継続
      }
      else
      {
        output("OK");      // maxIterations_回で終了
        return true;
      }
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    // AsyncJobのデストラクタでクリーンアップ
  }
};
```
