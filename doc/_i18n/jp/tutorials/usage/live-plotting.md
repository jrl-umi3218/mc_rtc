コントローラーを実行させながらグラフを表示したい場合があると思います（スタビライザーのゲインを調整する場合など）。それを実現する方法の一つが、現在記録中のログを開き、見たいデータをプロットすることです。しかし、実験が長時間にわたる場合、この方法ではログのサイズが大きくなり、ログを開いて処理するのに時間がかかるため、作業が煩わしくなります。

よりよい方法は、このチュートリアルで説明するように、現在記録中のログをコントローラーに表示させることです。

<h5 class="no_toc">注記</h5>

このページの例では、説明を簡単にするために、以下のコードスニペットが用意されているものとします

{% capture source %}
// 必要なすべての型宣言をインクルードする
#include <mc_rtc/gui/plot.h>

// 頻繁に使用する型の略称を定義する
using Color = mc_rtc::gui::Color;
using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;
using Range = mc_rtc::gui::plot::Range;
using Style = mc_rtc::gui::plot::Style;
using Side = mc_rtc::gui::plot::Side;
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

## 入門用の2つの簡単な例

以下の2つの例では、横軸を共有するグラフと横軸を共有しないグラフを作成する方法について説明します。

最初の例では、`mc_rtc::gui::plot::X`によって生成された横軸と、`mc_rtc::gui::plot::Y`、`mc_rtc::gui::plot::XY`、 `mc_rtc::gui::plot::Polygon(s)`のいずれかによって生成されたプロットデータを指定します。このページでは、以下、この種類のプロットを「標準プロット」と呼ぶことにします。

{% capture source %}
gui()->addPlot(
  "sin(t)",
  mc_rtc::gui::plot::X("t", [this]() { return t; }),
  mc_rtc::gui::plot::Y("sin(t)", [this]() { return sin(t); }, Color::Red)
);
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

2番目の例では、`mc_rtc::gui::plot::XY`または`mc_rtc::gui::plot::Polygon(s)`によって生成されたデータのみを指定します。このページでは、以下、この種類のプロットを「XYプロット」と呼ぶことにします。

{% capture source %}
{% raw %}
gui()->addXYPlot(
  "Circle in a square",
  mc_rtc::gui::plot::XY("Circle",
                        [this]() { return cos(t); }, [this]() { return sin(t); },
                        Color::Red),
  mc_rtc::gui::plot::Polygon("Square",
                             []() { return PolygonDescription({{-1, -1}, {-1, 1}, {1, 1}, {1, -1}}, Color::Blue); })
);
{% endraw %}
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

どちらの種類のプロットも同じ方法で削除できます。

{% capture source %}
gui()->removePlot("sin(t)");
gui()->removePlot("Circle in a square");
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

概要は以上です。次のセクションでは、例で使用されている各要素について詳しく見ていきます。

## `mc_rtc::gui::plot::AxisConfiguration`

このクラスを使用すると、指定した軸の名前と範囲を変更できます。デフォルトでは、範囲は`[-inf, +inf]`に設定されています。この場合、GUIクライアントは範囲を自動的に推測します。

### 横軸の設定（X軸）

#### 標準プロットの場合

標準プロットの場合、横軸の設定は`mc_rtc::gui::plot::X`で指定します。最初の例では第1引数で名前を指定しただけでしたが、この関数ではすべての設定を指定できます。例:

{% capture source %}
double t0 = t;
gui()->addPlot(
  "sin(t)",
  // 軸の最小値をt0に設定する
  // 軸の最大値をt0 + 10に設定する
  mc_rtc::gui::plot::X({"t", {t, t + 10}}, [this]() { return t; }),
  mc_rtc::gui::plot::Y("sin(t)", [this]() { return sin(t); }, Color::Red)
);
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

#### XYプロットの場合

XYプロットの場合、横軸の設定は省略できます。横軸を設定する場合は、`addXYPlot`の第1引数に`mc_rtc::gui::plot::AxisConfiguration`を指定する必要があります。例:

{% capture source %}
gui()->addXYPlot(
  "Circle",
  // 範囲を設定するもう一つの方法
  AxisConfiguration("X").min(-1).max(1),
  mc_rtc::gui::plot::XY("Circle",
                        [this]() { return cos(t); }, [this]() { return sin(t); },
                        Color::Red)
);
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

### 縦軸の設定（左右のY軸）

`AxisConfiguration`オブジェクトを指定することで、左右の縦軸を設定できます。

#### 左側のY軸

- 標準プロットの場合、横軸の設定を指定した後にY軸の設定を指定します。
- XYプロットの場合、X軸の設定を指定した後に左側のY軸の設定を指定します。

#### 右側のY軸

どちらのプロットの場合も、左側のY軸の設定を指定した後に右側のY軸の設定を指定します。

#### 例

{% capture source %}
// 左のY軸のみ設定
double t0 = t;
gui()->addPlot(
  "sin(t)",
  mc_rtc::gui::plot::X({"t", {t, t + 10}}, [this]() { return t; }),
  AxisConfiguration("Y", {-1, 1}),
  mc_rtc::gui::plot::Y("sin(t)", [this]() { return sin(t); }, Color::Red)
);

// 両方のY軸を設定
gui()->addXYPlot(
  "Circle",
  // 上下限を設定する別の方法
  AxisConfiguration("X").min(-1).max(1),
  AxisConfiguration("Left Y"),
  AxisConfiguration("Right Y").max(10),
  mc_rtc::gui::plot::XY("Circle",
                        [this]() { return cos(t); }, [this]() { return sin(t); },
                        Color::Red)
);
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

## `mc_rtc::gui::plot::X`

指定すべき項目はそれほどありません。以下の2つの要素を指定します。
1. 横軸の名前または`AxisConfiguration`
2. `double`型の値を返すコールバック関数

## `mc_rtc::gui::plot::Y`

必須の引数として3つの引数を指定し、オプションの引数として2つの引数を指定します。

1. プロットの凡例に使用する名前（**必須**）
2. `double`型の値を返すコールバック関数（**必須**）
3. `Color`（色）、または`Color`（色）を返すコールバック関数（**必須**）
4. ラインの`Style`（スタイル）。デフォルトは`Style::Solid`（実線）
5. データを左側と右側のどちらのY軸に表示するかを指定する`Side`（左右の位置）。デフォルトは`Side::Left`（左側）

オブジェクトを生成した後に`Style`（スタイル）と`Side`（左右の位置）を簡単に指定できる便利な関数が用意されています。

#### `mc_rtc::gui::plot::Style`について

4つのラインスタイルを指定できます。

1. `Style::Solid`: 実線
2. `Style::Dashed`: 破線
3. `Style::Dotted`: 点線
4. `Style::Point`: 線を1つの点に置き換える

#### 例

{% capture source %}
gui()->addPlot(
  "cos(t)/sin(t)",
  mc_rtc::gui::plot::X("t", [this]() { return t; }),
  mc_rtc::gui::plot::Y("cos(t)", [this]() { return cos(t); }, Color::Red, Style::Dotted),
  // デフォルトのスタイルを変更せずに、左右のどちらのY軸を使用するかを指定する
  mc_rtc::gui::plot::Y("sin(t)", [this]() { return sin(t); }, Color::Blue).side(Side::Right)
);
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

## `mc_rtc::gui::plot::XY`

必須の引数として4つの引数を指定し、オプションの引数として2つの引数を指定します。

1. プロットの凡例に使用する名前（**必須**）
2. `double`型の横軸の値（x）を返すコールバック関数（**必須**）
3. `double`型の縦軸の値（y）を返すコールバック関数（**必須**）
4. `Color`（色）、または`Color`（色）を返すコールバック関数（**必須**）
5. ラインの`Style`（スタイル）。デフォルトは`Style::Solid`（実線）
6. データを左側と右側のどちらのY軸に表示するかを指定する`Side`（左右の位置）。デフォルトは`Side::Left`（左側）

オブジェクトを生成した後に`Style`（スタイル）と`Side`（左右の位置）を簡単に指定できる便利な関数が用意されています。

## `mc_rtc::gui::plot::Polygon`

必須の引数として2つの引数を指定し、オプションの引数として1つの引数を指定します。

1. プロットの凡例に使用する名前（**必須**）
2. 次に説明する`PolygonDescription`を返すコールバック関数（**必須**）
3. データを左側と右側のどちらのY軸に表示するかを指定する`Side`（左右の位置）。デフォルトは`Side::Left`（左側）

オブジェクトを生成した後に`Side`（左右の位置）を簡単に指定できる便利な関数が用意されています。

### `mc_rtc::gui::plot::PolygonDescription`

このオブジェクトは以下の情報を保持します。

- `points()`: 点のリスト（`std::vector<std::array<double, 2>>`）
- `outline()`: 多角形の輪郭の`Color`（色）
- `style()`: 多角形の輪郭の`Style`（スタイル）（デフォルトは`Style::Solid`（実線））
- `fill()`: 閉じた多角形を塗りつぶす`Color`（色）（デフォルトは透明（塗りつぶしなし））
- `closed()`: 多角形が閉じているかどうか（デフォルトはtrue）

_注: 多角形が閉じている場合、`points()`の最後に始点を改めて指定する必要はありません（これはクライアントによって処理されます）。_

#### 例

{% capture source %}
{% raw %}
auto redSquareBlueFill =
  PolygonDescription({{-1, -1}, {-1, 1}, {1, 1}, {1, -1}}, Color::Red).fill(Color::Blue);
auto purpleTriangleYellowFill =
  PolygonDescription({{1, 0}, {1.5, 2}, {2, -2}}, Color::Magenta).fill(Color::Yellow);
auto cyanRectangle =
  PolygonDescription({{-2, -2}, {2, -2}, {2, -3}, {-2, -3}}, Color::Cyan);
{% endraw %}
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

_注: コールバック関数`mc_rtc::gui::plot::Polygon`によって`PolygonDescription`が返されるため、アクティブなプロットの多角形の表示スタイルを自由に変更できます。_

## `mc_rtc::gui::plot::Polygons`

これは`mc_rtc::gui::plot::Polygon`と似ていますが、このコールバック関数は`std::vector<PolygonDescription>`を返します。そのため、アクティブなプロットの多角形を時系列的に変化させて、シンプルなアニメーションを作成できます。

## 実行中にグラフを作成する

実行中にデータを選択してグラフを作成したい場合、イントロダクションで紹介したAPIではうまくいかない場合があります。その場合、以下に示すように{% doxygen mc_rtc::gui::StateBuilder::addPlotData %}メソッドを使用します。


{% capture source %}
// この例ではコールバック関数のマップを使用する。マップのキーによって、ラベルとデータが指定される
// コールバック関数ごとにスタイルを変えることで、処理をカスタマイズできる
std::map<std::string, std::function<double()>> callbacks;

// Y軸のデータなしでプロットを作成する
gui()->addPlot(
  "MyPlot",
  mc_rtc::gui::plot::X("t", [this]() { return t; }));

// N実行時の入力に基づきデータを追加する
for(const auto & it : callbacks)
{
  const auto & label = it.first;
  const auto & callback = it.second;
  gui()->addPlotData("MyPlot", mc_rtc::gui::plot::Y(label, callback));
}
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

> 注: この処理では実行時に以下のチェックが行われます。
1. 指定されたプロットが存在しない場合、この呼び出しは何の効果もありません。
2. データとして1次元データが指定され、グラフとしてXYプロットが指定された場合、この呼び出しは何の効果もありません。
