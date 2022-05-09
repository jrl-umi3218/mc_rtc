## 接触面自由度とは

{% comment %}FIXME outdated transation{% endcomment %}

接触面自由度は、接触面上で特定の自由度を持った動きを可能にする機能です。具体的には、接触面上で1つ以上の軸に沿った自由な動き（平行移動または回転）を可能にする機能です。

これは、特定の方向の動きを許可したり制限したりするのに使用されます。
- （平らなまたは円筒状の）接触面を挿入・削除する場合、通常、法線軸に沿って動かす必要があります。
- また、接触面が平らな場合、位置はそのままにして、実際の表面の向きに合わせて接触面の向きを調整する必要があります。

## 接触面自由度の作成方法

接触面自由度は、ContactConstraintのみを修飾します。`Tasks::QPContactConstr.h`に、接触面自由度に関するプロトタイプ宣言がありますので、それを見てみましょう。
```cpp
bool addDofContact(const ContactId& contactId, const Eigen::MatrixXd& dof);
bool removeDofContact(const ContactId& contactId);
void updateDofContacts();
void resetDofContacts();
```
接触面自由度は、一般に以下の流れで使用します。
- 動きを許可または制限する接触面を見つける
- 正しい自由度を選択する
- 自由度を制約条件に追加する
- 制約条件を更新する
- 動作が完了したら、自由度を削除して制約条件を再度更新する

ContactIDを見つけるには、以下のように、`mc_rbdyn::Contact`を作成して`contactId`メソッドを呼び出すのが一番よい方法です。
```cpp
mc_rbdyn::Contact contact;
tasks::qp::contactId cId = contact.contactId(robots());
```

- 自由度行列は6行6列の正方行列とする
- 通常は対角要素のみを設定する非対角要素は、「らせん状」の制約条件の作成などに使用できる
- 値が0の要素はその要素に対応する動きが許可されていることを表し、値が1の要素はその要素に対応する動きが制限されていることを表す
- 最初の3つの要素が (x, y, z) 軸周りの回転を表し、次の3つの要素が (x, y, z) 軸方向の平行移動を表す

例:
```
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
```
「仮想的な」接触面です。動きは全く制限されていません。

```
1 0 0 0 0 0
0 1 0 0 0 0
0 0 1 0 0 0
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
```

すべての動きが制限されています（通常の接触面）

```
1 0 0 0 0 0
0 1 0 0 0 0
0 0 1 0 0 0
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 0
```

法線方向の平行移動を除きすべての動きが制限されています（接触面の挿入に便利です）。

```
1 0 0 0 0 0
0 1 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 1
```

典型的なスライドする平らな接触面です。法線軸周りの回転と、接線軸方向（x軸方向とy軸方向）の平行移動が可能です。

> 回転では、接触面の中心が回転軸となります。これは、円筒の周囲でグリッパーを回転させようとしたときに問題となります（グリッパーの回転軸が円筒の軸と異なるため）。

## 例

それでは、HRP2のデフォルトのサンプル（地面に立つサンプル）を試してみましょう。コンストラクターでは以下のように定義されています。
```cpp
    solver().setContacts({
            {robots(), 0, 1, "LeftFoot", "AllGround"},
            {robots(), 0, 1, "RightFoot", "AllGround"}
            });
```

では、右足を垂直に動かしてみます。

```cpp
// reset()で以下を実行
Eigen::Matrix6d dof = Eigen::Matrix6d::Identity();
dof(5,5) = 0;
 
tasks::qp::ContactId ContactId = mc_rbdyn::Contact(robots(), 0, 1, "RightFoot", "AllGround").contactId(robots());
contactConstraint().contactConstr->addDofContact(ContactId, dof);
contactConstraint().contactConstr->updateDofContacts();
```

次に、足が垂直に動くのを*制限*するため、EndEffectorTaskを追加し、この制約条件が正しく機能するか確認します。

```cpp
//ヘッダファイルに以下を追加
std::shared_ptr<mc_tasks::EndEffectorTask> efTask;

//cppファイルに以下を追加
// コンストラクタ部分
efTask = std::make_shared<mc_tasks::EndEffectorTask>("R_FOOT", robots(), 0, 5.0, 100);
efTask->addToSolver(solver());

// reset()関数部分
efTask->resetTask(robots(), 0);
efTask->add_ef_pose(sva::PTransformd(Eigen::Vector3d(0.2, 0., 0.2)));
```
これを実行すると、JVRC1の足が完全に垂直に動くのが分かります。

> 注: この手順はキネマティクスモードで実行してください。そうでない場合、ロボットの体重を左足に乗せるという最初のステップを追加しない限り、ロボットが転倒します。

次に、これを少し変更して、スライドする平らな接触面をシミュレートしてみましょう。行列`dof`を編集し、`EndEffectorTask`のターゲットを変更します。

```cpp
Eigen::Matrix6d dof = Eigen::Matrix6d::Identity();
dof(2,2) = 0;
dof(3,3) = 0;
dof(4,4) = 0;

efTask->add_ef_pose(sva::PTransformd(sva::RotX(1.)*sva::RotY(1.)*sva::RotZ(1.),Eigen::Vector3d(0.2, -0.2, 0.2)));
```

今度は、接触面は同じ高さで水平を保ったままですが、接触面内で垂直軸周りに回転します。

この場合もキネマティクスモードでしか正しく機能しません。
