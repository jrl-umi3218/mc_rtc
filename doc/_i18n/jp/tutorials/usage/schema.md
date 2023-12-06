## `mc_rtc::Schema` の紹介

[JSON Schema](https://json-schema.org/)は、JSONオブジェクトから指定されたデータ構造への期待されるデータ形式を文書化する仕様です。

[mc_rtc](https://jrl-umi3218.github.io/mc_rtc/json.html)では、オブジェクトをフレームワークにロードする際の期待されるデータを文書化するために使用されます。

mc_rtc内でコードを記述する際に、おそらく{% include link_tutorial.html category="usage" tutorial="mc_rtc_configuration" %}オブジェクトから多くのパラメータをロードするシナリオに遭遇するでしょう。

この構成オブジェクトをデータ構造にロードするだけでなく、オブジェクトの現在の状態を`mc_rtc::Configuration`ファイルに保存し、それをディスクに保存すること、パラメータをオンラインで編集できるようにすること、構成形式を文書化することにも関心を持つでしょう。

すべてのコードを手動で記述することは可能ですが、特に構造からフィールドを追加または削除する場合は面倒でエラーが発生しやすいです。

これが`mc_rtc::Schema`が登場する場所です。その目標は、シンプルな構造のように振る舞いながらも、追加の機能を備えた構造を記述できるようにすることです。

## 私たちの最初のスキーマベースの構造

```cpp
#include <mc_rtc/Schema.h>

struct SimpleSchema
{
  MC_RTC_NEW_SCHEMA(SimpleSchema)
#define MEMBER(...) MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleSchema, __VA_ARGS__)
  MEMBER(bool, useFeature, "魔法の機能を使用する")
  MEMBER(double, weight, "タスクの重み")
  MEMBER(std::vector<std::string>, names, "いくつかの名前")
  MEMBER(sva::ForceVecd, wrench, "目標トルク")
  MEMBER(sva::PTransformd, pt, "いくつかの変換")
#undef MEMBER
};
```

<h5 class="no_toc">MEMBERマクロについて</h5>

`MEMBER`マクロを使用する場合は、以下のプリプロセッサの制限に注意してください。

<h6 class="no_toc">テンプレート化されたタイプ</h6>

プリプロセッサの制限により、複数のテンプレート・パラメータに依存する型をマクロに直接渡すことはできません。例えば、以下のようにするとコンパイルエラーになります。

```cpp
MEMBER(std::map<std::string, double>, jointValues, "関節名と値のマップ")
```

これは、プリプロセッサが引数を次のように分割するために発生します：

- `std::map<std::string`
- `std::map<std::string`
- `jointValues`
- `"関節名と値のマップ"`

これを避けるには、明示的に型のエイリアスを指定します。

```cpp
using MapType = std::map<std::string, double>;
MEMBER(MapType, jointValues, "関節の名前と値のマップ")
```

このオプションを使用できない場合は、代わりに `BOOST_IDENTIY_TYPE` を使用することができます。

<h6 class="no_toc">MSVCユーザー向けの注意</h6>

上記のコードをMicrosoft Visual C++ Compiler（MSVC）で使用する場合、`MEMBER`の定義を次のように変更する必要があります：

```cpp
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleSchema, __VA_ARGS__))
```

これは、MSVCのプリプロセッサの問題を回避するためのものです。

### 使用法

この方法で定義した構造は通常のC++構造体として使用されます：

```cpp
void do_foo(const sva::ForceVecd &);

// この構造体はデフォルトで構築できます（デフォルトについては後で説明します）
SimpleSchema simple;
// メンバーをシンプルな構造体としてアクセスできます
do_foo(simple.wrench);
// メンバーは指定した型のものなので、次のようにもできます
do_foo(simple.pt * simple.wrench);
```

ただし、この構造には追加の機能が組み込まれています：

```cpp
// mc_rtc::Configuration値からロード
simple.load(config);
// mc_rtc::Configurationオブジェクトに保存
simple.save(config);
// mc_rtc::Configurationのユーザー定義変換も利用できます
config.add("simple", simple);
SimpleSchema simple2 = config("simple");
// コンソールに出力
mc_rtc::log::info("simple:\n{}", simple.dump(true, true));
// simpleをインプレースで編集するGUIフォームを追加
simple.addToGUI(*controller.gui(), {"フォームカテゴリ"}, "フォーム名",
                [this]() {
                  // この時点でsimpleオブジェクトは新しい値で更新されています
                  on_simple_update();
                });
```

また、集成初期化または[指定された初期化](https://en.cppreference.com/w/cpp/language/aggregate_initialization#Designated_initializers)（C++20から）を使用して構造を初期化することもできます：

```cpp
SimpleSchema simple{true, 42.42, {"a", "b"}, wrench, pt};
```

最後に、ここで作成した構造体は、手動で作成した単純な構造体と同じサイズです。

## `MC_RTC_SCHEMA_MEMBER` マクロ

前の例では、`MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER` マクロを使用しました。実際には、より一般的な `MC_RTC_SCHEMA_MEMBER` マクロがあり、そのシグネチャは次のようになっています：

```cpp
MC_RTC_SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, FLAGS, DEFAULT, ...)
```

パラメータの使用方法は次のとおりです：

- `T`はこのメンバーが表示されるスキーマオブジェクトの型です。
- `TYPE`はメンバー変数の型です。
- `NAME`はメンバー変数の名前です。
- `DESCRIPTION`はメンバー変数のドキュメント文字列であり、生成されたフォームでも使用されます。
- `FLAGS`はメンバーに関する追加情報を追加するために使用され、メンバーが必要かどうかなどを特に示します。
- `DEFAULT`はこのメンバーのデフォルト値を指定します。
- `...` / `NAMES`はこれらの追加のパラメータで、2つの目的に使用されます：
  - `TYPE`が`std::string`の場合、有効な値を提供するために使用できます。
  - `TYPE`が`std::variant<T...>`の場合、変数の代替名を割り当てるために使用されます。

### `TYPE`について

少なくともロード/セーブのシナリオでは、どの `TYPE` でも使用できることを意図していますが、フォームベースのエディションではすべての `TYPE` がサポートされるとは限りません。

もし、ある `TYPE` がサポートされていない場合、コンパイルエラーが発生します。

現在のところ、`mc_rtc::Configuration`オブジェクトを通してロード/セーブできるほとんどの型と、スキーマベースの型、そのような型の `std::vector` と `std::map` がサポートされている。

以下はそのような型を使用した例である：

```cpp
struct ComposeSchema
{
  MC_RTC_NEW_SCHEMA(ComposeSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(ComposeSchema, __VA_ARGS__))
  MEMBER(int, integer, "整数値")
  MEMBER(SimpleSchema, simple, "シンプルなスキーマ")
  MEMBER(std::vector<SimpleSchema>, many, "複数のシンプルスキーマ")
#undef MEMBER
};
```

### `FLAGS`について

現在、2つのフラグがあります：

- `mc_rtc::schema::Required`
- `mc_rtc::schema::Interactive`

#### `mc_rtc::schema::Required`

メンバーが必要な場合：

- メンバーは設定で存在している必要があります。
- メンバーは正しくその型にデシリアライズされる必要があります。

それ以外の場合、メンバーは設定で存在する場合にのみ読み込まれます。メンバーは常にConfigurationオブジェクトに保存されますが、必要かどうかにかかわらずです。

#### `mc_rtc::schema::Interactive`

スキーマは次の型を対話型として扱うことができます：

- `Eigen::Vector3d`
- `sva::PTransformd`

これらの型のメンバーがフォームに追加されると、対話型フォーム要素が使用されます。ただし、これが常に意味があるわけではありません。たとえば、3Dゲインは`Eigen::Vector3d`で表すことができ、この値を編集するために3Dマーカーを使用する必要はありません。

この動作を制御するには、対応する対話型フラグを設定します。基本的な`MC_RTC_SCHEMA_MEMBER`以外のすべてのマクロでは、フラグがデフォルトで設定されています。

#### 例

次の例は、`MC_RTC_SCHEMA_MEMBER`マクロの呼び出しでこれらのフラグを使用する方法を示しています：

```cpp
struct InteractiveSchema
{
  MC_RTC_NEW_SCHEMA(InteractiveSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_MEMBER(InteractiveSchema, __VA_ARGS__))
  MEMBER(Eigen::Vector3d,
         point,
         "3Dポイント",
         mc_rtc::schema::Required | mc_rtc::schema::Interactive,
         Eigen::Vector3d::Zero())
  MEMBER(Eigen::Vector3d, gain, "3Dゲイン", mc_rtc::schema::Required, Eigen::Vector3d::Ones())
  MEMBER(Eigen::Vector3d, gainOpt, "3Dオプショナルゲイン", mc_rtc::schema::None, Eigen::Vector3d::Zero())
#undef MEMBER
};
```

### `DEFAULT`について

これにより、メンバーのデフォルト値を指定できます。代入演算子の右側で有効な値であれば、何でも構いません。_DEFAULT_`マクロを使用している場合、これは`mc_rtc::Default<T>`です。これは次のようになります：

- 算術型の場合は`0`（したがって`bool`の場合は`false`）
- Eigenベクトルの場合はゼロ
- Eigen正方行列の場合は単位行列
- `sva::PTransformd`の場合は単位変換行列
- `sva::ForceVecd`、`sva::MotionVecd`、`sva::ImpedanceVecd`、`sva::AdmittanceVecd`の場合はゼロ
- `std::string`の場合は空の文字列
- std::vector<T>`の場合は空のベクトル
- 空のマップ `std::map<K, V>` の場合
- `std::variant<T>`の場合は`mc_rtc::Default<T>`です
- スキーマベースの構造の場合はデフォルト値

### `NAMES`について

これらの追加パラメータには2つの可能な使用例があります：

- `TYPE`が`std::string`の場合、有効な値のリストを提供する場合
- `TYPE`が`std::variant<T...>`の場合、バリアントの代替名を割り当てる場合

#### 例

この例では、バリアントとして表されるゲインを持っています。これは`double`スカラーまたは`Eigen::Vector3d`である可能性があります。`mc_rtc::schema::Choices`が`MEMBER`マクロに渡され、これらの名前がGUIで選択肢を表示する際に使用されます：

```cpp
struct SimpleVariant
{
  MC_RTC_NEW_SCHEMA(SimpleVariant)
  using gain_t = std::variant<double, Eigen::Vector3d>;
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleVariant, __VA_ARGS__))
  MEMBER(gain_t, stiffness, "タスク剛性", mc_rtc::schema::Choices({"スカラー", "次元"}))
#undef MEMBER
};
```

## `MC_RTC_NEW_SCHEMA` および `MC_RTC_SCHEMA` マクロ

これまでのすべての例で、`MC_RTC_NEW_SCHEMA`を使用してスキーマ宣言を導入しました。その名前が示すように、このマクロは新しいスキーマ宣言を導入するために使用されます。

このマクロは、作成している構造体の名前を単一の引数として受け取ります。

ただし、スキーマを追加のメンバーで拡張する場合、`MC_RTC_SCHEMA`マクロを代わりに使用することがあります：

```cpp
struct ExtendedSchema : public SimpleSchema
{
  MC_RTC_SCHEMA(ExtendedSchema, SimpleSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(ExtendedSchema, __VA_ARGS__))
  MEMBER(double, extendedGain, "拡張アルゴリズムのゲイン")
#undef MEMBER
};
```

このようにして、スキーマを拡張することができます。
