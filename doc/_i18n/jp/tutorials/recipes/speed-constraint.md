このページでは、制約条件`mc_solver::BoundedSpeedConstr`について説明します。この制約条件は、その名前とこのページのタイトルが示すように、速度と方向を厳密に指定してボディを動かす必要がある場合に使用します。より「緩やかな」条件を適用する場合については、以下の記事を参照してください。

- [タスク空間内の各軸に異なる重みを適用する](dim-weight.html)
- [接触面の自由度制約条件を変更する](contact-dof.html)

## オブジェクトを作成して速度制約条件を追加する

まず、制約条件オブジェクトを作成します。

```cpp
// bSpeedCstrは以下の型であると仮定
// std::shared_ptr<mc_solver::BoundedSpeedConstr>
bSpeedCstr = std::make_shared<mc_solver::BoundedSpeedConstr>(robots(),
                                                             robots().robotIndex(),
                                                             timeStep);
```

これらのパラメーターのうち、変更する必要があるのは、制約条件を適用するロボットを指定する`robotIndex`だけです。


次に、これをソルバーに追加します。

```cpp
solver().addConstraintSet(*bSpeedCstr);
```

衝突制約条件と同様に、この時点ではこの制約条件を有効にする仕組みが設定されただけで、この制約条件は実際には何の効果もありません。

最後に、速度制約条件をソルバーに追加します。

```cpp
bSpeedCstr->addBoundedSpeed(solver(),
                            bodyName,
                            bodyPoint,
                            dof,
                            speed);
```

上記のコードにおいて、
- `bodyName`は、制約条件を適用するボディの名前
- `bodyPoint`は、ボディフレーム内で回転の基準として使用する点（`Eigen::Vector3d`）。`bodyPoint`が0の場合、ボディの原点が使用されます。
- `dof`は、自由度選択行列（詳細については下記を参照）
- `speed`は、6次元の速度制約条件

なお、`speed`はボディフレーム内で表されます。

#### 自由度行列について

自由度行列に関する要件は、[接触面自由度](contact-dof.html)に関する要件と非常に似ています。その要件は以下のとおりです。
- 6行6列の正方行列とする
- 対角要素のみを設定する
- 値が0の要素は制約条件が適用されない自由度を表し、値が1の要素は制約条件が適用される自由度を表す
- 最初の3つの要素が (x, y, z) 軸周りの回転を表し、次の3つの要素が (x, y, z) 軸方向の平行移動を表す

また、上限と下限を指定して以下のように速度制限条件を構築することもできます。

```cpp
bSpeedCstr->addBoundedSpeed(solver(),
                            bodyName,
                            bodyPoint,
                            dof,
                            lowerSpeed,
                            upperSpeed);
```

引数は前記の場合とほぼ同じですが、速度の制限を固定値で指定するのではなく、`lowerSpeed`と`upperSpeed`の間の値として指定する点が異なります。

## 例

この最初の例では、法線軸に沿って手を一定の速度で動かすという制約条件のみを適用します。

```cpp
Eigen::MatrixXd dof = Eigen::MatrixXd::Identity(6,6);
Eigen::VectorXd spd = Eigen::VectorXd::Zero(6);
spd(5) = 0.1;  // r_wrist をz方向に一定の速度 0.1 m/s で動かす
bSpeedConstr->addBoundedSpeed(solver(), "r_wrist",
                              Eigen::Vector3d::Zero(),
                              dof, spd);
```

これを実行すると、ロボットの手があるポイントまで持ち上がりますが、それ以上持ち上げることはできないため、ソルバーはこれを正しく処理できないのが分かります。このような場合はこの制約条件を部分的に適用する必要があります。

より実用的な例として、この制約条件を使用してボディの速度を制限することが挙げられます。今度は、この制約条件を削除し、`efTask`という名前の`mc_tasks::EndEffectorTask`をソルバーに追加し、非常に高い剛性を指定します。

```cpp
efTask = std::make_shared<mc_tasks::EndEffectorTask>("r_wrist", robots(), 0);
efTask->set_ef_pose(sva::PTransformd(sva::RotY<double>(-M_PI/2),
                                     efTask->get_ef_pose().translation() + Eigen::Vector3d(0.3, -0.1, 0.2)));
efTask->positionTask->stiffness(100);
efTask->orientationTask->stiffness(100);
solver().addTask(efTask);
```

これを実行すると、ロボットの手が非常にぎくしゃく動くのが分かります。もちろん、この動きを実現するための他の制約条件が設定されているため、この動きは実際のロボットで完全に実行可能ですが、ここでは、速度制約条件を使用して動作速度を制限してみましょう。

```cpp
Eigen::MatrixXd dof = Eigen::MatrixXd::Identity(6,6);
Eigen::VectorXd spd = Eigen::VectorXd::Zero(6);
// 並進速度と回転速度に異なるスケーリングを施す
for(size_t i = 0; i < 3; ++i) { spd(i) = M_PI*0.1; }
for(size_t i = 3; i < 6; ++i) { spd(i) = 0.1; }
bSpeedConstr->addBoundedSpeed(solver(), "r_wrist",
                              Eigen::Vector3d::Zero(),
                              dof, -spd, spd);
```

これを実行すると、動きがはるかにスムーズになったことが分かります。

**注:** 上記の例は非常に作為的であり、剛性が高いタスクや動きが遅いタスクを構築するためのよい方法とはいえません。
