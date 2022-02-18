タスク空間の各軸に異なる重みを持たせることができます。例えば、質量中心の垂直方向の位置を無視できます。

ほとんどのタスクでは、`Eigen::VectorXd`型の追加パラメーター`dimWeight`を受け取り、それを使って方向ごとに異なる重みを持たせることが可能になります。

- `dimWeight`は、重みをその値で置き換えるのではなく、重みにその値を乗算します。例えば、重みが`1000`で、`dimWeight`が`[1., 1., 1e-3]`である`comTask`は、トータルの重みが`[1000., 1000., 1.]`であるタスクとして動作します。
- `dimWeight`は、サイズが可変のベクトルです。*ユーザー*が正しいサイズを設定する必要があります。なお、このパラメーターに対して`dim()`を呼び出すと、（下位レベルの）タスクサイズを取得できます。
- コンストラクターによる生成時にこのパラメーターを渡すことも、後でその値を変更することも可能です

#### 例
`comTask`の垂直方向の誤差を無視します。
```cpp
Eigen::VectorXd dimWeight(3);
dimWeight.setOnes();
dimWeight[2] = 0.;

comTask = std::make_shared<mc_tasks::CoMTask>(robots, robots().robotIndex(), 5.0, 100.);
comTask->dimWeight(dimWeight);
```

`EndEffectorTask`は、エンドエフェクターの向きと位置を制御するための2つのタスクで構成されています。そのため、`dimWeight`メソッドは6次元のベクトルを必要とします。最初の3つの次元は向きの次元の重みを制御し、残りの3つの次元は位置の次元の重みを制御します。`EndEffectorTask`で`x`方向と`y`方向の平行移動を無視するには、以下のようにします。
```cpp
Eigen::VectorXd dimWeight(6);
dimWeight << 1., 1., 1., 0., 0., 1.;

efTask = std::make_shared<mc_tasks::EndEffectorTask>("RARM_LINK6", robots(), 5.0, 100);
efTask->dimWeight(dimWeight);
```

または、タスクの位置の部分を直接操作します。
```cpp
Eigen::VectorXd dimWeight(3);
dimWeight.setZero();
dimWeight(2) = 1.;

efTask = std::make_shared<mc_tasks::EndEffectorTask>("RARM_LINK6", robots(), 5.0, 100);
efTask->positionTask->dimWeight(dimWeight);
```

> 注: これは制約条件では*ありません*。これらの方向にタスクが何も作用しないことを意味します。特定の方向への動きを制限したい場合は、[接触面自由度](contact-dof.html)または[速度制約条件](speed-constraint.html)を使用します。
