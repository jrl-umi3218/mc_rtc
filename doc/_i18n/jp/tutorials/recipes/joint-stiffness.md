この機能は`PostureTask`にのみ適用されます。これは、他のタスクの[関節選択](joint-select.html)機能や[次元重み](dim-weight.html)機能と似ています。

この機能を実現するには、`JointStiffness`を指定します。または、`JointGains`を使用して重要でない減衰値を指定することもできます。

この機能は、タスクの他の部分に影響を与えることなく一部の関節をより正確に指定角度に合わせたい場合に便利です。関節を制御するタスクは以下のように分類されます。

- 関節パラメーターの指定などによって、関節角度を直接指定して関節を制御するタスク
- 一部の関節を制御しないタスク（すなわち、JointSelectorを使用する場合）。制御しない関節の角度を維持したい場合は、それらの関節の剛性を高める必要がある

## 使用方法

まず、重み5、剛性1で`PostureTask`を作成します。次に、右腕の関節をいくつか選択し、それらの剛性を4にします。

```cpp
{% raw %}
auto postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), 0, 1, 5);
std::vector<tasks::qp::JointStiffness> stiffnesses = {{"RARM_JOINT3", 4.},
                                                      {"RARM_JOINT4", 4.}
                                                      {"RARM_JOINT5", 4.}
                                                      {"RARM_JOINT6", 4.}};

postureTask.jointsStiffness(solver(), stiffnesses);
{% endraw %}
```

この場合、通常の関節の剛性は1に設定され、選択した関節の剛性は4に設定されます。

これらの関節の減衰値を、ユーザーが定義した他の値に設定したい場合は、`JointGains`を使用します。なお、`JointGains`は、コンストラクターによる生成時に剛性のみを指定することができます。その場合、重要な減衰値にはデフォルト値が使用されます。今度は、剛性を4に設定し、減衰値を10に設定します。

```cpp
{% raw %}
std::vector<tasks::qp::JointGains> gains = {{"RARM_JOINT3", 4., 10.},
                                            {"RARM_JOINT4", 4., 10.}
                                            {"RARM_JOINT5", 4., 10.}
                                            {"RARM_JOINT6", 4., 10.}};

postureTask.jointsGains(solver(), gain);
{% endraw %}
```
