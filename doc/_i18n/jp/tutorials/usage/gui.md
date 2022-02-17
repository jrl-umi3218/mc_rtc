{% comment %}FIXME Some comments are not translated {% endcomment %}

GUI機能では、コントローラーをリモートで操作するための動的なグラフィカルインターフェイスを構築することができます。この機能をコントローラーに実装して使用する方法は、[ロギング]({{site.baseurl}}/tutorials/usage/logging.html)の手法と似ています。

このページでは以下のトピックについて説明します。

- コントローラーからGUIにアクセスする
- GUIに要素を追加する
- GUIから要素を削除する

### GUIにアクセスする

GUIはサーバー/クライアントモデルで動作します。コントローラーがサーバーとなり、リモートインターフェイスがクライアントとなります。そのため、コントローラー側のコードでは、GUIサーバーからクライアントに送信されるデータのみが出力されます。このデータは、`mc_rtc::gui::StateBuilder`インターフェイスを通じて追加されます。

GUIビルダーにアクセスするには、コントローラーのコードから`gui()`を呼び出します。ロギングインターフェイスと同様に、GUIビルダーインターフェイスは常に有効になっているため、この機能を使用できるかどうかを気にする必要はありません。

### GUIに要素を追加する

GUIはカテゴリーごとに整理されているため、同じカテゴリーに複数の要素を追加すると、それらの要素はGUI上で一緒に表示されます。

カテゴリーは、文字列ベクトル（Pythonの場合は文字列リスト）で表されます。要素は、要素の型とアクセサー（コントローラーが提供するデータへのアクセサー）が記述されたシンプルなオブジェクトで表されます。

要素を追加するには、カテゴリーと1つ以上の要素を指定して`gui()->addElement(...)`を呼び出します。例:

```cpp
// Add a single button named "Push" in the a/b category
gui()->addElement({"a", "b"},
  mc_rtc::gui::Button("Push", []() { std::cout << "Hello!" << std::endl; })
);

// Add multiple elements at once, they will appear in the order they are added in the function
gui()->addElement({"a", "b"},
  mc_rtc::gui::Button("Button 1", []() { return; }),
  mc_rtc::gui::Button("Button 2", []() { return; }),
  mc_rtc::gui::Button("Button 3", []() { return; })
);
```

##### データに関する注記

状態ビルダーには、よく使用する要素への参照用に、いくつかの「静的な」データが用意されています。デフォルトでは、以下のデータ参照エントリを使用できます。
- `robots`: コントローラー内で参照されるロボット名のリスト
- `surfaces`: ロボットの名前と表面のマップ
- `bodies`: ロボットの名前とボディのマップ

データにアクセスするには、`gui()->data()`を呼び出して変更します。デフォルトのデータは、GUIクライアントでいくつかの要素をきれいに表示するのに使用されるため、それらのデータを変更しないことを強く推奨します。

#### シンプルな要素

このセクションでは、GUIに追加可能な各要素について説明します。ここで示す例はいずれも、`gui()->addElement(...)`の呼び出しによってGUIに追加されることを想定しています。コントローラーとデータをやり取りする機能をGUIに追加する場合、形式を指定してデータをインポート・エクスポートする必要はありません。それらの処理はGUIによって行われます。

##### `Label`/`ArrayLabel`

これらの要素は、シンプルなテキストや数値データを表示するのに使用します。

例

```cpp
// String label
Label("LabelText", []() { return "some text"; });

// Display a value
Label("Some value", [this]() { return this->value_; });

// Display a vector of strings
ArrayLabel("Some strings", []() { return {"a", "b", "c"}; });

// Display a vector with labels
ArrayLabel("Vector", {"x", "y", "z"}, []() { return Eigen::Vector3d::Zero(); });

// Some elements have associated labels, e.g. Force also adds an ArrayLabel
Force("LeftFoot", [this]() { return this->robot().surfaceWrench("LFullSole"); }, [this]() { return this->robot().surfacePose("LFullSole"); }),
Force("RightFoot", [this]() { return this->robot().surfaceWrench("RFullSole"); }, [this]() { return this->robot().surfacePose("RFullSole"); }),
Force("LeftHand", [this]() { return this->robot().surfaceWrench("LeftFingers"); }, [this]() { return this->robot().surfacePose("LeftFingers"); }),
Force("RightHand", [this]() { return this->robot().surfaceWrench("RightFingers"); }, [this]() { return this->robot().surfacePose("RightFingers"); })
```

Pythonの場合、パラメーターリストの末尾にラベルを記述します。
```python
ArrayLabel("Vector", lambda: self.v3d, ["x", "y", "z"])
```

##### `Button`

この要素は、単純にボタンを表示します。ボタンをクリックすると、指定したコールバック関数が呼び出されます。

```cpp
Button("Push me", []() { std::cout << "Hello!" << std::endl; });
```

##### `Checkbox`

この要素はチェックボックスを表示します。表示される状態は指定した値により決まります。チェックボックスをクリックすると、値が反転します。

```cpp
Checkbox("Check me", [this]() { return status_; }, [this]() { status_ = !status_; });
```

##### `StringInput`/`IntegerInput`/`NumberInput`/`ArrayInput`

これらの要素は、データを入力する手段をユーザーに提供します。データに最も適した記述を使用してください。

```cpp
StringInput("Your name", [this]() { return name_; }, [this](const std::string & n){ name_ = n; });

NumberInput("Weight", [this]() { return w_; }, [this](double w){ w_ = w; });
```

`ArrayInput`では、データにラベルを追加するためのパラメーターも指定できます。また、配列サイズは固定されているとみなされます。

```cpp
ArrayInput("Your array", {"x", "y", "z"},
           [this]() { return v3_; },
           [this](const Eigen::Vector3d & v) { v3_ = v; });
```

Pythonの場合、パラメーターリストの末尾にラベルを記述します。

##### `ComboInput`/`DataComboInput`

これらの要素は、文字列のリストから値を選択するためのダイアログを作成します。`ComboInput`では、ユーザーに選択させる文字列のリストを指定し、`DataComboInput`では、GUI状態データマップのエントリへの参照を指定します。

```cpp
ComboInput("Choose from list", {"a", "b", "c"},
           [this]() { return choice_; },
           [this](const std::string & c) { choice_ = c; });

DataComboInput("Choose a robot", {"robots"},
               [this]() { return robot_; }
               [this](const std::string & r) { robot_ = r; });

DataComboInput("Choose a surface", {"surfaces", robot().name()},
               [this]() { return surface_; }
               [this](const std::string & s) { surface_ = s; });
```

##### `Point3D`/`Rotation`/`Transform`

これらの要素は、以下の2つの項目を表示します。
- 追加する要素に適した`ArrayInput`
- （オプション）RVizなどの3次元環境でデータを表示するための、対話形式の要素

```cpp
// Read-only variant does not provide a callback to set the data
Point3D("Point", [this]() { return v3_; });

// Read-write variant

// Note that even though we want to display a rotation, we need to provide a
// full transform otherwise the GUI could not guess where to display the element
Rotation("Rot",
         [this]() { return sva::PTransformd{rot_, pos_} },
         [this](const Eigen::Matrix3d & rot) { rot_ = rot; });
```

##### `Trajectory`

この要素は、3次元環境に移動経路を表示します。リアルタイムの移動経路として表示するか（表面がたどった経路の表示など）、あらかじめ計画された移動経路として表示するかは、返されるデータの種類に応じて決定してください。


```cpp
// Real-time trajectory, returns a single point (either Eigen::Vector3d or sva::PTransformd)
Trajectory("RealTimeTrajectory",
           [this]() { return robot().surfacePose("LeftGripper"); });

// Pre-planned trajectory returns a vector of points
Trajectory("Trajectory",
           [this]() { return traj_; });
```

##### `Polygon`

この要素は、単一のポリゴン（1つの平面など）またはポリゴンのリスト（歩容計画など）を表示します。

```cpp
// A single polygon is a vector of Eigen::Vector3d
Polygon("Polygon"
        [this]() -> std::vector<Eigen::Vector3d> { return {p0, p1, p2, p3}; });

// A list of polygon is a vector of vector of Eigen::Vector3d
Polygon("Step plan",
        [this]]() { return step_plan_display_; });
```

##### `Force`

この要素は、3次元環境に力ベクトルを表示します。力の値を`sva::ForceVecd`として指定し、力を適用する対象となるフレームを`sva::PTransformd`として指定する必要があります。

```cpp
Force("LeftFoot", [this]() { return this->robot().surfaceWrench("LFullSole"); }, [this]() { return this->robot().surfacePose("LFullSole"); }),
```

##### `Arrow`

この要素は、3次元環境に矢印を表示します。矢印の始点と終点を指定する必要があります。編集可能な矢印を指定することもできます。

```cpp
// Read-only
Arrow("ArrowRO", [this]() { return start_; }, [this]() { return end_; });

// Editable arrow
Arrow("Arrow",
      [this](){ return start_; },
      [this](const Eigen::Vector3d & start) { start_ = start; },
      [this](){ return end_; },
      [this](const Eigen::Vector3d & end) { end_ = end; });
```

##### `XYTheta`

この要素は、`Transform`要素と同様ですが、X/Y軸方向の平行移動とZ軸周りの回転のみを編集できる点が異なります。

```cpp
// Returns a vector with 3 elements representing the X/Y position and the theta angle
XYTheta("XYThetaOnGround", [this]() -> std::array<double, 3> { return {x, y, theta}; });

// Specify the height with a 4th element
XYTheta("XYTheta", [this]() -> std::array<double, 4> { return {x, y, theta, z}; });
```

##### `Table`

この要素では、任意のデータが格納された表を3次元環境に表示できます。データのコールバック関数は、`std::vector<std::vector<double>>`や`std::vector<std::tuple<X, Y, Z>>`のように配列の配列として表されたオブジェクトを返す必要があります。

```cpp
// Table with a fixed header
Table("Simple table", {"X", "Y", "Z"}, [this]() -> const std::vector<Eigen::Vector3d> & { return points_; });
// Table with a fixed header and formatting information
Table("Simple with format", {"X", "Y", "Z"}, {"{:0.3f}", "{:0.3f}", "{:0.3f}"}, [this]() { return data_; });
// Table with a dynamic header
Table("Dynamic table", [this]() { return header_; }, [this]() { return data_; });
// Table with a dynamic header + formatting
Table("Dynamic with format", [this]() { return header_; }, [this]() { return format_; }, [this]() { return data_; });
```

見出し、書式、データサイズの一貫性については、ユーザーが保証する必要がありますが、一貫性がなくても、ある程度まではクライアント側で正しく処理されます。書式指定文字列は、[{書式}](https://fmt.dev/latest/syntax.html)の規則に従って記述する必要があります。

##### `Form`

`Form`要素を使用すると、より複雑な対話形式のダイアログを構築できます。フォーム自体は他の要素で構成されます。`Form`は以下のように作成します。

```cpp
Form("Push to send", // This will be shown on the "send" button for the form
     [this](const mc_rtc::Configuration & data) {},
     ... // list of element
);
```

以下の要素を使用できます。
- `FormCheckbox(name, required, default)`
- `FormIntegerInput(name, required, default)`
- `FormNumberInput(name, required, default)`
- `FormStringInput(name, required, default)`
- `FormArrayInput(name, required, default, fixed_size (true))`
- `FormComboInput(name, required, values)`
- `FormDataComboInput(name, required, reference)`

`name`は、要素名の表示とコールバック関数によるデータの取得で使用されます。`required`は、このフィールドが必須フィールドかどうかを表します。`default`は、このフィールドのデフォルト値を表します（省略可）。

`default`にデフォルト値を指定する代わりに、互換性のある型を返すコールバック関数を指定できます。

`FormArrayInput`の`fixed_size`は、指定されたデフォルト値のサイズが固定であるか、ユーザーがサイズを拡大・縮小できるかを表します。

`FormComboInput`の`values`は、コンボボックスに表示する値のリストを表します。

`FormDataComboInput`の`reference`は、データエントリへの参照を表します。また、`$`記号を使ってフォーム内の他のフィールドを参照できます。例:

```cpp
Form("Button name",
     [this](const mc_rtc::Configuration & data) {},
     FormDataComboInput("Robot", true, {"robots"}),
     FormDataComboInput("Surface", true, {"surfaces", "$Robot"})
);
```

この例では、`Robot`でユーザーが選択した項目に応じて`Surface`コンボボックスの表示内容が変わります。

##### コールバック関数に関する注記

ユーザーの操作によって呼び出されたコールバック関数は、コントローラーの現在のループ処理が完了し、次のループ処理が開始される前に実行されます。並列処理についてユーザーが気にする必要はありません。

### GUIから要素を削除する

GUIから要素を削除するための2つの関数が用意されています。

```cpp
// Remove a single element by name
gui()->removeElement({"a", "b"}, "element");
// Remove a category and all its sub-categories
gui()->removeCategory({"a", "b"});
```
