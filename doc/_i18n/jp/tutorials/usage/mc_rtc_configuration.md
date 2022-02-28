このページでは以下の情報を提供します。

1. `mc_rtc::Configuration`オブジェクトからデータを読み取る方法
2. `mc_rtc::Configuration`オブジェクトにデータを書き込む方法
3. デフォルトでC++/Pythonのどの型がサポートされているか
4. `mc_rtc::Configuration`との間でC++オブジェクトの読み込み・保存を行う独自の関数を記述する方法

このページの例で使用されている`config`は、C++の例では`mc_rtc::Configuration`型のC++オブジェクト、Pythonの例では`mc_rtc.Configuration`型のPythonオブジェクトであると仮定しています。

### データソース

`mc_rtc::Configuration`は、JSONオブジェクトまたはYAMLオブジェクトを抽象化したもので、これを使用すると、オブジェクトの操作と、オブジェクトからのC++/Pythonデータの取得が簡単に行えます。.

設定オブジェクトにデータを読み込むには、以下のようにします。

```cpp
// ファイルから読み込む、JSONデータであることを想定
auto config = mc_rtc::Configuration("myfile.conf");
// YAMLファイルから読み込む、拡張子はymlかyaml
auto config = mc_rtc::Configuration("myfile.yaml");
// すでに読み込まれているJSONデータから読み込む
auto config = mc_rtc::Configuration::fromData(data);
// すでに読み込まれているYAMLデータから読み込む
auto config = mc_rtc::Configuration::fromYAMLData(data);
```

### 設定からデータを読み取る

設定内のデータにアクセスする方法がいくつか用意されています。状況に応じて最も適したものを組み合わせて使用することがきます。以下の3つの方法が用意されています。

1. 厳格なアクセス: 取得するデータの型と同じ型のキーが存在しなければなりません。
2. 条件付きの厳格なアクセス: 厳格なアクセスにおけるキーの存在要件を回避したい場合に使用します。
3. デフォルトのアクセス: 厳格なアクセスの制約条件は全く適用されません。

#### 厳格なアクセス

最初の方法では、アクセスしたい設定エントリが読み込み済みの設定ファイル内に存在しない場合や、取得したい値の型と保存されている値の型が一致しない場合には、例外がスローされます。

以下のようにエントリにアクセスします。
```cpp
Eigen::Vector3d v = config("MyVector3d");
```

設定のサブセクションにアクセスすることもできます。
```cpp
Eigen::VectorXd v = config("section")("MyVectorXd");
```

また、これらをネストさせることもできます。
```cpp
std::vector<std::string> v = config("section")("sub1")("sub2")("SomeStrings");
```

#### 条件付きの厳格なアクセス

以下のようにします。
```cpp
if(conf.has("MyQuaternion"))
{
  Eigen::Quaterniond q = config("MyQuaternion");
}
```

上記のコードでは、設定に`MyQuaternion`エントリが存在するものの、`Eigen::Quaterniond`オブジェクトを取得するのに必要なデータが含まれていない場合にのみ例外がスローされます（先のセクションを参照）。

#### デフォルトのアクセス
この方法を使用すると、設定エントリを安全に取得できます。設定エントリが存在しない場合や、要求した型と実際の型が異なる場合でも、例外はスローされません。

以下のようにエントリにアクセスします。
```cpp
bool b = false;
config("MyBool", b);
```

また、上記のコードを以下のように簡潔に記述することもできます。
```cpp
bool b = config("MyBool", false);
```

エントリが存在しない場合や、要求した型と実際の型が異なる場合は、指定した変数を変更できません。そうでない場合は、設定に格納されている値が指定した変数にコピーされます。特に、ベクトルを取得する場合はこの点を十分理解してください。
```cpp
std::vector<double> v = {1., 2., 3.};
config("MyVector", v); // MyVector が有効なエントリである場合、vの初期値は失われる
```

先の方法と同様に、設定のサブセクションにアクセスすることもできます。
```cpp
double d = 1.0;
config("section")("sub1")("sub2")("MyDouble", d);
```

ただし、先の方法のコードでは、設定ファイル内にセクションやサブセクションが1つも存在しない場合、**例外がスローされます**。

#### サブセクションへのアクセス

上記の厳格なアクセスとデフォルトのアクセス方法は、`mc_rtc::Configuration`オブジェクトのサブセクションへのアクセスに適用できます。例:

```cpp
// 引数がconfigに存在するオブジェクトやセクションで無い場合はthrowする
auto sub = config("section");

// 引数がconfigに存在しない場合、空のオブジェクトを返す
auto sub = config("section", mc_rtc::Configuration{});
```

なお、コピーは浅い階層で行われるため、サブセクションへの変更は親セクションから見えます。

#### Pythonの場合の相違点

Pythonの場合、ユーザーが要求した要素の型をPythonに推測させることはできません。そのため、データを取得する際はPythonの型または値を指定する必要があります。

```python
# c は mc_rtc.Configuration オブジェクト
c = config("key")
# ブーリアン型を取得
b = config("b", bool)
# v は eigen.Vector3d オブジェクト
v = config("v", eigen.Vector3d)
# ブーリアン型をデフォルト値Falseとして取得
b = config("b", False)
# eigen.Vector3d のリストを取得
vlist = config("vlist", [eigen.Vector3d])
# floatのリストをデフォルト値を指定して取得
flist = config("flist", [0., 1., 2.])
```

### `mc_rtc::Configuration`オブジェクトにデータを書き込む

以下に、オブジェクトAPIと配列APIの違いを示す記述の例を示します。

```cpp
// -- オブジェクト --

// セクションキーを指定して新たしい空のオブジェクトを生成
auto section = conf.add("section");

// セクションにデータを追加
section.add("v1", Eigen::Vector3d{1,2,3});
section.add("isFixed", true);
// ここでdataは mc_rtc::Configuration でサポートされている型のデータ
section.add("data", data);

// -- 配列 --

// 新しい空の配列を生成
auto array = conf.array("array");
// 新しい空の配列を生成し、10要素分のメモリを確保
auto array10 = conf.array("array10", 10);

// データを配列に追加
array.push(Eigen::Vector3d{1,2,3});
array.push(true);
array.push(data);
```

PythonのAPIもこれと全く同じです。

### サポートされている型

上記の例で見たように、`Configuration`オブジェクトを使用してさまざまな型のオブジェクトを取得できます。以下の表に、サポートされている型とJSONの要件を示します。

<table class="table">
  <thead>
    <tr>
      <th scope="col">型</th>
      <th scope="col">JSONの要件</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th scope="row"><code>bool</code></th>
      <td>bool型（<code>true</code>/<code>false</code>）または整数（0がfalseに対応し、その他の値がtrueに対応）</td>
    </tr>
    <tr>
      <th scope="row"><code>int</code></th>
      <td>整数</td>
    </tr>
    <tr>
      <th scope="row"><code>unsigned int</code></th>
      <td>符号なし整数、または0以上の整数</td>
    </tr>
    <tr>
      <th scope="row"><code>double</code></th>
      <td>数値エントリ</td>
    </tr>
    <tr>
      <th scope="row"><code>std::string</code></th>
      <td>文字列エントリ</td>
    </tr>
    <tr>
      <th scope="row"><code>Eigen::Vector3d</code></th>
      <td>数値要素から成る3次元配列</td>
    </tr>
    <tr>
      <th scope="row"><code>Eigen::Quaterniond</code></th>
      <td>数値要素から成る4次元配列。正規化された四元数が返されます。</td>
    </tr>
    <tr>
      <th scope="row"><code>Eigen::Vector6d</code></th>
      <td>数値要素から成る6次元配列</td>
    </tr>
    <tr>
      <th scope="row"><code>Eigen::VectorXd</code></th>
      <td>数値要素から成る任意の次元の配列</td>
    </tr>
    <tr>
      <th scope="row"><code>Eigen::Matrix3d</code></th>
      <td>数値要素から成る9次元配列。回転をサポートするため、2種類の代替表現がサポートされています。RPY角度を表す3次元配列、または四元数を表す4次元配列として指定することもできます。</td>
    </tr>
    <tr>
      <th scope="row"><code>std::vector&lt;T&gt;</code> <code>T</code>には上記のいずれかの型を指定できます。</th>
      <td>各要素が型<code>T</code>の要件を満たす配列</td>
    </tr>
    <tr>
      <th scope="row"><code>std::array&lt;T, N&gt;</code> <code>T</code>には上記のいずれかの型を指定できます。`N`は固定のサイズです。</th>
      <td>各要素が型<code>T</code>の要件を満たす<code>N</code>次元配列</td>
    </tr>
    <tr>
      <th scope="row"><code>std::pair&lt;U, V&gt;</code> <code>U</code>と<code>V</code> には上記のいずれかの型を指定できます。</th>
      <td>各次元の要素がそれぞれ型<code>U</code> と`V`の要件を満たす2次元配列</td>
    </tr>
    <tr>
      <th scope="row"><code>std::map&lt;std::string, T&gt;</code> <code>T</code> には上記のいずれかの型を指定できます。</th>
      <td>文字列によってインデックスが付けられ、型<code>T</code>の要件を満たす要素から成るマップ</td>
    </tr>
  </tbody>
</table>

ほかにも、タスクや制約条件といったさまざまな型がサポートされています。mc_rtcに読み込み可能なオブジェクトについては、[こちらのWebサイト]({{site.baseurl}}/json.html)にあるドキュメントを参照してください。

#### 型の組み合わせ

設定でサポートされている一般型を、自由に組み合わせて使うことができます。例えば、以下の記述は完全に有効です。

```cpp
std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<double>> inequalities = config("inequalities");
```

### 独自の型のサポート

この機能では、独自の型を持つデータの読み込みや保存を行えるように`mc_rtc::Configuration`を特殊化することができます。現在、この機能はC++でのみサポートされています。

この機能を使用するには、`mc_rtc`名前空間に以下の形式で特殊化テンプレートを追加します。

```cpp
template<>
struct ConfigurationLoader<MyType>
{
  static MyType load(const mc_rtc::Configuration & config);

  static mc_rtc::Configuration save(const MyType & object);
};
```

この機能は、数多くの`SpaceVecAlg`型、`RBDyn`型、`mc_rbdyn`型をサポートするために`mc_rtc`内で広く使用されています。　　 宣言については`mc_rbdyn/configuration_io.h`を参照してください。

保存関数ではオプションの引数がサポートされているため、以下のような特殊化が可能です。

```cpp
template<>
struct ConfigurationLoader<MyType>
{
  static MyType load(const mc_rtc::Configuration & config);

  static mc_rtc::Configuration save(const MyType & object, bool verbose = false);
};
```
