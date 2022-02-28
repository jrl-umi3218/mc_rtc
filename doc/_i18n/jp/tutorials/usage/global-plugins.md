mc_rtcでは、すべてのインタフェースは {% doxygen mc_control::MCGlobalController %} クラスをコントローラの初期化及び実行に使用します。より詳細には、{% doxygen mc_control::MCGlobalController::run() %} が実行周期毎に実行されます。

プラグインシステムはこのrun関数の最初と最後の両方または一方で実行されるコンポーネントを追加する機能を提供します。

これらのプラグインは以下のような様々な用途で利用が可能です。

- サードパーティのミドルウェアを用いてデータを出力する (例：ROSプラグイン)
- mc_rtcとは別のインタフェースを用いて取得されたセンサ情報をmc_rtcに取り込む
- 単一のコントローラよりもより上位の機能を提供する

mc_rtcでのプラグイン実装方法の詳細は [new plugin tutorial]({{site.baseurl}}/tutorials/advanced/new-plugin.html) で確認できます。

プラグインで利用可能なオプションやプラグインの利用方法は各プラグインのドキュメントで確認できます。このチュートリアルではこれらのプラグインの設定方法や有効化の方法を説明します。

## 大域的にプラグインを有効化する方法

プラグインは [mc_rtc configuration]({{site.baseurl}}/tutorials/introduction/configuration.html#possible-locations-for-mc-rtc-configuration) の `Plugins` リストに追加することで有効化できます。例えば以下のように記述します。

```yaml
Plugins: [Joystick, OculusVR]
```

ここにリストアップされたプラグインは指定されているコントローラに関係なく有効化されます。

<div class="alert alert-info">プラグインをインストールする際に自動ロードのオプションが提供されている場合があります。このようなプラグインは `Plugins` リストに記載があるか否かにかかわらず自動でロードされます。</div>

### 参照される設定ファイル

以下の2つのファイルがmc_rtcによって読み込まれます（存在する場合）。

<ol>
  <li>{% ihighlight bash %}${PLUGIN_RUNTIME_INSTALL_PREFIX}/etc/${PLUGIN_NAME}.yaml{% endihighlight %}</li>
  <li>
    Linux/macOS: {% ihighlight bash %}$HOME/.config/mc_rtc/plugins/${PLUGIN_NAME}.yaml{% endihighlight %}<br/>
    Windows: {% ihighlight msshell %}%APPDATA%/mc_rtc/plugins/${PLUGIN_NAME}.yaml{% endihighlight %}
  </li>
</ol>

ここで `${PLUGIN_RUNTIME_INSTALL_PREFIX}` はLinux/macOSにおいては `${INSTALL_PREFIX}/lib/mc_plugins` であり、Windowsにおいては `${INSTALL_PREFIX}/bin/mc_plugins` を意味します。

## コントローラレベルでのプラグインの有効化

プラグインをコントローラレベルで有効化するにはコントローラの設定に `Plugins` エントリを追加します。

### 参照される設定ファイル

<div class="alert alert-warning">これらのファイルはプラグインがコントローラレベルで有効化された場合にのみ参照されます。</div>

大域的な設定ファイルに加えて、以下のファイルが存在する場合プラグインの設定に使用されます。

<ol>
  <li>{% ihighlight bash %}${CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/${CONTROLLER_NAME}/plugins/${PLUGIN_NAME}.yaml{% endihighlight %}</li>
  <li>
    Linux/macOS: {% ihighlight bash %}$HOME/.config/mc_rtc/controllers/${CONTROLLER_NAME}/plugins/${PLUGIN_NAME}.yaml{% endihighlight %}<br/>
    Windows: {% ihighlight msshell %}%APPDATA%/mc_rtc/controllers/${CONTROLLER_NAME}/plugins/${PLUGIN_NAME}.yaml{% endihighlight %}
  </li>
</ol>

ここで `${CONTROLLER_RUNTIME_INSTALL_PREFIX}` はLinux/macOSにおいては `${INSTALL_PREFIX}/lib/mc_controllers` でありWindowsにおいては `${INSTALL_PREFIX}/bin/mc_controllers` を指します。

<div class="alert alert-warning">複数のコントローラが有効化され、かつそれらが同じプラグインを使用している場合、全ての設定ファイルがmc_rtcの`Enabled`エントリに記述されたコントローラの順番で参照されます。</div>
