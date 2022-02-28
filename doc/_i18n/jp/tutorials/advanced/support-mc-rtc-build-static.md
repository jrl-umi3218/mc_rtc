`MC_RTC_BUILD_STATIC`は、mc_rtcのビルドオプションです。主に、[オンラインデモ](https://mc-rtc-demo.netlify.app/)を実現するために使用されます。このオプションを使用すると、コントローラー、有限オートマトンの状態、ロボットモジュール、観測器、プラグインを動的に読み込めなくなります。そのため、これらのコンポーネントをmc_rtcのライブラリに組み込む必要があります。

このページでは、まず、このオプションを使用してプロジェクトをコンパイルする際に必要な作業について説明します。次に、デモをWebに公開するためのガイドラインについて説明します。

## プロジェクトを修正する

まず、コントローラーその他の必要なコンポーネントをmc_rtcのソースツリー内でビルドする必要があります。といっても、mc_rtcの`controllers`フォルダー、`observers`フォルダー、`plugins`フォルダー、`robots`フォルダーのいずれかにプロジェクトを置いてビルドするだけです。各フォルダーは、単にプロジェクトを整理する手段として用意されているだけで、どのフォルダーも機能的には同じです。

### mc_rtcのソースツリー内に移動しているか調べる

通常、mc_rtcのパッケージを見つけるには以下のようにします。

```cmake
find_package(mc_rtc REQUIRED)
```

mc_rtcのソースツリー内に移動しているか調べるには、mc_rtcのパッケージを別の場所から探す必要があるかどうかで判断します。

```cmake
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
```

[jrl-cmakemodules](https://github.com/jrl-umi3218/jrl-cmakemodules) を使用している場合、mc_rtc内でビルドするにはこれをインクルードする必要があります（いずれにせよmc_rtcを介してすべての機能を使用できます）。

### 依存関係を調べる

例えば、コントローラーで[copra](https://github.com/jrl-umi3218/copra)パッケージが必要であったとします。この場合、一般に以下のCMakeコードを使用して依存関係を調べます。

```cmake
find_package(copra REQUIRED)
```

このコントローラーをmc_rtcでビルドすると、`copra`は`mc_rtc`と依存関係を持つようになります。したがって依存関係を調べるコードは以下のようになります。

```cmake
if(NOT MC_RTC_BUILD_STATIC)
  add_required_dependency(copra REQUIRED)
else()
  find_package(copra REQUIRED)
endif()
```

### mc_rtcのコンポーネントを追加する

`find_package(mc_rtc)`によって提供されているマクロ（`add_controller(NAME ...)`, `add_robot(NAME ...)` 等) を使用すれば、面倒な作業を行う必要はほとんどありません。そうでない場合は、mc_rtcのソースツリー内にあるsrc/CMakeLists.txtを参照し、どのように変更すべきかを理解してください。主な違いは、ソースのパスを相対パスではなく絶対パスで与える必要があるという点にあります。

これに関して注意すべき点が1つあります。コントローラーを`mc_control::fsm::Controller`から派生させる場合、`add_fsm_controller(NAME ...)`を使用する必要があります。

また、使用するコンポーネントを外部のライブラリとリンクさせる場合、そのライブラリを関連するmc_rtcのライブラリとリンクさせる必要があります。

例:

```cmake
add_controller(myController "${myController_SRC}" "${myController_HDR}")
target_link_libraries(myController PUBLIC copra::copra)
```

これは以下のようになります。

```cmake
if(NOT MC_RTC_BUILD_STATIC)
  add_controller(myController "${myController_SRC}" "${myController_HDR}")
  target_link_libraries(myController PUBLIC copra::copra)
else()
  add_fsm_controller(myController "${myController_SRC}" "${myController_HDR}")
  target_link_libraries(mc_control_fsm PUBLIC copra::copra)
  # コンポーネントの種類に応じて以下のライブラリを使用します
  # - mc_control （FSMを使用しないコントローラ）
  # - mc_rbdyn （ロボットモジュール）
  # - mc_observers （オブザーバ）
  # - mc_control （プラグイン）
endif()
```

### 例

この[パッチ](https://github.com/gergondet/lipm_walking_controller/commit/f507f63de378a9d80917d9b3f1280a5843bb2b56) を適用すると、[lipm_walking_controller](https://github.com/jrl-umi3218/lipm_walking_controller)で`MC_RTC_BUILD_STATIC`を使用できるようになります。

## mc_rtcのデモをWebに公開する

デモをWebに公開するには、[emscripten](https://emscripten.org/index.html)プロジェクトを使用します。このプロジェクトを使用すると、C++のコードをコンパイルして[WebAssembly](http://webassembly.org/)を作成し、互換性のある任意のブラウザーでWebAssemblyを実行させることができます。なお、私たちの知る限りでは、mc_rtcのすべてのAPIを使用できます。

<div class="row">
  <div class="offset-2 col-8">
    <div class="alert alert-warning" role="alert">
      コントローラーのオンラインデモをデプロイする場合、事実上、（コンパイル済みの）コードとデータが一般に公開される点に注意してください。 機密性が問題になる場合は、オンラインデモをデプロイすることはお控えください。
    </div>
  </div>
</div>

### emscriptenに関する一般的な制約事項

emscriptenに関する一般的な制約事項の詳細については、[emscriptenポーティングガイド](https://emscripten.org/docs/porting/index.html)を参照してください。通常、この制約事項はそれほど大きな影響はありません。

最も大きな制約事項は、ファイルシステムの問題に関するものです。仮想ファイルシステムを使ってWebアプリケーションにアクセスする場合、以下のいずれかで対処する必要があります。

1. ビルドシステムのファイルシステム階層構造と同じ構造を仮想ファイルシステム内に構築する
2. オンラインデモ用に作成された「ダミー」パスでビルドされるように設定する

下記のリンクで示されたmc_rtcオンラインデモは、2番目の方法で公開されています（2番目の方法のほうが、何をデモとして公開するかを管理するのが少し簡単になります）。

### オンラインデモをビルドする

とりあえず、既存の[mc-rtcオンラインデモ](https://github.com/mc-rtc/demo/)を自分のプロジェクトに合わせて変更した上で使用することを推奨します。このビルドスクリプトを使って、オンラインホストにデプロイする前にデモをローカルでビルドすることができます。

### FirefoxとGitHubページに関する注意点

GitHubのページにデモを公開した場合、このデモはFirefoxでは動作しません（このチュートリアルを書いた2020年11月27日時点）。Firefoxを使用する場合、Webサーバーから特定のHTTPヘッダーを与える必要がありますが、GitHubのページではそのヘッダーが提供されません（現在、このヘッダーをカスタマイズする方法はありません）。

そのヘッダーを以下に示します。
```yaml
Cross-Origin-Opener-Policy: same-origin
Cross-Origin-Embedder-Policy: require-corp
```
