## 概要

mc\_rtcは、シミュレーションシステムやロボット制御システムとデータをやり取りするインターフェイスです。これらのシステムは、指定されたロボットの状態（関節の値やセンサーの測定値など）を出力します。一方、mc\_rtcは、ロボットの目標状態（コマンド）を出力します。この処理は、`mc_control::MCGlobalController`クラスを通じて実行されます。このクラス自体は制御を行わず、このタスクを`mc_control::MCController`クラスの派生オブジェクトにデリゲートします。mc\_rtcフレームワーク内でコントローラーを記述するには、`mc_control::MCController`基底クラスから派生したクラスを記述し、必要な機能をそのクラスに実装します。後で説明するチュートリアルでは、そうしたコントローラーを実装します。このページのチュートリアルでは、お使いのマシンに本フレームワークをビルド・インストールする方法について説明します。

<img src="{{site.baseurl_root}}/assets/tutorials/introduction/img/mc_rtc_architecture.jpg" alt="architecture_overview" class="img-fluid" />

## インストール手順

[Homebrew](https://brew.sh/)から、最新のUbuntu LTSとmacOSのバイナリ版を提供しています。また、簡単に使用できるUbuntu、macOS、Windows用のソースコード版と[vcpkg](https://vcpkg.io/en/index.html)のレジストリも提供しています。

UbuntuユーザーとmacOSユーザーにはバイナリ版を推奨します。Windowsユーザーにはvcpkgを推奨します。

### Ubuntu LTS (18.04, 20.04)

{% assign install_apt=site.translations[site.lang].tutorials.introduction["installation-guide"].install_apt %}

{% include show_sources.html sources=install_apt id="install_apt" copy=true %}

*注: mc\_rtcのディストリビューション版では、[eigen-qld](https://github.com/jrl-umi3218/eigen-qld)を介してQLD QPソルバー（線形二次計画法ソルバー）を使用できます。LSSOLへのアクセス権があり、[eigen-lssol](https://gite.lirmm.fr/multi-contact/eigen-lssol)をインストールできる場合、LSSOLのサポート付きで[Tasks](https://github.com/jrl-umi3218/Tasks)をビルドして`/usr`にインストールできます。この2つのバージョンはバイナリレベルで互換性があります。*

### Homebrew (macOS)

公式のインストール手順に従って[Homebrew](https://brew.sh/)をインストールしてください。その後、以下のコマンドを実行してください。

{% capture source %}
brew tap mc-rtc/mc-rtc
brew install mc_rtc
{% endcapture %}

{% include show_source.html id="brew" lang="bash" source=source copy=true %}

### vcpkg

[vcpkg](https://vcpkg.io/)のインストール手順に従って、お使いのシステムにvcpkgをインストールしてください。

その後、`vcpkg`のバイナリまたは`vcpkg.json`マニフェストと共に`vcpkg-configuration.json`ファイルを作成し、レジストリをセットアップしてください。

{% capture source %}
{
  "registries": [
    {
      "kind": "git",
      "baseline": "{下記を参照}",
      "repository": "https://github.com/mc-rtc/vcpkg-registry",
      "packages": [ "spacevecalg", "rbdyn", "eigen-qld", "sch-core", "tasks",
                    "mc-rbdyn-urdf", "mc-rtc-data", "eigen-quadprog", "state-observation",
                    "hpp-spline", "mc-rtc" ]
    }
  ]
}
{% endcapture %}

{% include show_source.html id="vcpkg-configuration" lang="json" source=source copy=true %}

ここで、`baseline`には、[mc-rtc/vcpkg-registry](https://github.com/mc-rtc/vcpkg-registry/)の最新のコミットハッシュを指定してください。

以下のいずれかの方法を選択できます。

- `vcpkg`コマンドを使用してmc_rtcをインストールする: `vcpkg install mc_rtc`
- 以下のように、マニフェスト（`vcpkg.json`ファイル）内でmc\_rtcパッケージを指定する

{% capture source %}
{
  "name": "my-package",
  "version-string": "1.0.0",
  "homepage": "https://my.home",
  "description": "パッケージの説明",
  "dependencies": [
    "mc-rtc"
  ]
}
{% endcapture %}

{% include show_source.html id="vcpkg-json" lang="json" source=source copy=true %}


### mc-rtc-superbuildを使用したインストール

[mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild/) はCMakeを用いて作られたmc_rtcのインストールに必要なソフトウェア及びmc_rtcをインストールするためのツールです。加えて、mc_rtcを必要とするプロジェクトをビルドできるようにするための拡張機能も備えています。

使用方法についてはプロジェクトのホームページを参照して下さい。

### 古いインストール方法

以下のインストール方法も利用可能ですが、これらは現在サポートされていません。
- [build_and_install.sh スクリプトを用いてソースからビルドする方法]({{site.baseurl}}/tutorials/introduction/installation-build-and-install.html)
- [スクリプトを用いずにソースからビルドする方法]({{site.baseurl}}/tutorials/introduction/installation-build-no-script.html)

## はじめましょう

mc_rtcのインストールが完了したら、[次のセクション]({{site.baseurl}}/tutorials/introduction/configuration.html)に進んでください。
