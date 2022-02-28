多くの場合、質量中心が移動できる領域を制限することが推奨されます。これを実現するため、質量中心の移動範囲を凸領域内に制限する新たな制約条件`CoMIncPlaneConstr`を導入します。

この制約条件のAPIを以下に示します。
```cpp
CoMIncPlaneConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt);
void set_planes(QPSolver & solver,
                const std::vector<mc_rbdyn::Plane> & planes,
                const std::vector<Eigen::Vector3d> & speeds = {},
                const std::vector<Eigen::Vector3d> & normalsDots = {},
                double iDist = 0.05,
                double sDist = 0.01,
                double damping = 0.1,
                double dampingOff = 0.);
```
このAPIでは以下のことが行えます。
- 指定されたロボットとタイムステップに適用する制約条件を構築する
- 制約面を設定する（すなわち、質量中心が実際に移動できる領域を更新する）

平面の設定方法
---
まず、凸集合について簡単に説明します。凸集合は、頂点、または一連の1次不等式で等価的に表すことができます。すなわち、凸集合の内部にある任意の点pは、各頂点の重みを`\lambda_i`としたときの頂点の重心として表すことができます。
```
p = \sum_i \lambda_i v_i
\sum_i \lambda_i = 0
\forall i \lambda_i \geq 0
```
ただしこの方法は、基本的に線形計画問題を解く必要があるため、頂点が内部にあるかどうかをテストする方法としてはあまり効率的ではありません。凸集合に含まれるかどうかをテストする実用的な方法としては、超平面表現を使用するほうがはるかに便利です。この方法であれば、1回の行列演算でテストできます。
```
A p + b \geq 0
```
ここで、Aは各平面の法線ベクトルを縦に積み重ねて構築した行列、bはオフセットのベクトルです。

したがって、静的領域の質量中心を制限するには、法線ベクトルとオフセットをいくつか構築し、それらを制約条件に含めればよいことになります。

例
---
ここで、[先に構築した]({{site.baseurl}}/tutorials/introduction/com-controller.html)質量中心コントローラーを振り返ってみましょう。このときの動的シミュレーションでは、平面の端に向かってどこまでも進むようにロボットに命令したため、ロボットが落下してしまいました。それでは、ロボットの移動範囲を一辺が8cmの正方形に制限しましょう。これは以下のように表されます。
```
-0.08 \leq x \leq 0.08
-0.08 \leq y \leq 0.08
```

行列を使用すると以下のように表されます。
```
⎡ -1 0 0 ⎤ ⎡ x ⎤   ⎡ 0.08 ⎤
⎢ 1  0 0 ⎥ ⎢ y ⎥ + ⎢ 0.08 ⎥ \leq 0
⎢ 0 -1 0 ⎥ ⎣ z ⎦   ⎢ 0.08 ⎥
⎣ 0  1 0 ⎦         ⎣ 0.08 ⎦
```

これを制約条件に含めれば完成です。
```cpp
{% raw %}
//以下をヘッダファイルに追加
# include <mc_solver/CoMIncPlaneConstr.h>

std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstraintPtr_;


//cppファイルのコントローラの初期化リストに以下を追加
comIncPlaneConstraintPtr_.reset(new mc_solver::CoMIncPlaneConstr(robots(), robots().robotIndex(), dt) );

//コンストラクタ本体に以下を追加
    std::vector<mc_rbdyn::Plane> planes =
    {{{-1., 0., 0.}, 0.08},
     {{1., 0., 0.}, 0.08},
     {{0., -1., 0.}, 0.08},
     {{0., 1., 0.}, 0.08},
    };
 solver().addConstraintSet(*comIncPlaneConstraintPtr_);
 comIncPlaneConstraintPtr_->set_planes(solver(), planes);
{% endraw %}
```

コントローラーを起動すると、ロボットが見えない「壁」に突き当たり、与えられたタスクを実行できないことが分かります。

追加パラメーター
---

`set_planes`に渡す追加パラメーターは、以下の2つのカテゴリに分類されます。

- 動く平面を扱う
- 相互作用の挙動を変更する

1番目のカテゴリーについては、追加情報を与える追加パラメーターとして、平面の速度と平面の法線の時間微分を渡すことができます。なお、軌跡を追跡する場合は、各タイムステップについて平面とその微分係数を設定する必要があります。

以下のパラメーターを調整することができます。

- 相互作用の距離: 質量中心と平面との距離がこの距離より小さくなったとき、平面によって質量中心が押し返される
- 安全距離: 質量中心と平面がそれ以上近づくことができない距離
- 反発時の挙動を規定する減衰値とそのオフセット
