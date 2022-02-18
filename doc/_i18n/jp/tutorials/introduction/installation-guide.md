## 概要

mc\_rtcは、シミュレーションシステムやロボット制御システムとデータをやり取りするインターフェイスです。これらのシステムは、指定されたロボットの状態（関節の値やセンサーの測定値など）を出力します。一方、mc\_rtcは、ロボットの目標状態（コマンド）を出力します。この処理は、`mc_control::MCGlobalController`クラスを通じて実行されます。このクラス自体は制御を行わず、このタスクを`mc_control::MCController`クラスの派生オブジェクトにデリゲートします。mc\_rtcフレームワーク内でコントローラーを記述するには、`mc_control::MCController`基底クラスから派生したクラスを記述し、必要な機能をそのクラスに実装します。後で説明するチュートリアルでは、そうしたコントローラーを実装します。このページのチュートリアルでは、お使いのマシンに本フレームワークをビルド・インストールする方法について説明します。

<img src="{{site.baseurl_root}}/assets/tutorials/introduction/img/mc_rtc_architecture.jpg" alt="architecture_overview" class="img-fluid" />

## インストール手順

[Homebrew](https://brew.sh/)から、最新のUbuntu LTSとmacOSのバイナリ版を提供しています。また、簡単に使用できるUbuntu、macOS、Windows用のソースコード版と[vcpkg](https://vcpkg.io/en/index.html)のレジストリも提供しています。

UbuntuユーザーとmacOSユーザーにはバイナリ版を推奨します。Windowsユーザーにはvcpkgを推奨します。

### Ubuntu LTS (18.04, 20.04)

{% assign install_apt=site.translations[site.lang].tutorials.introduction["installation-guide"].install_apt %}

{% include show_sources.html sources=install_apt copy=false id="install_apt" copy=true %}

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


{% comment %} FIXME Translation out-of-date from this point {% endcomment %}

### ソースからビルドする（スクリプトを使用する場合）

LinuxまたはmacOSでビルドする場合は、「ビルド」のセクションに進んでください。Windowsユーザーの場合は以下に示す追加の指示に従ってください。

#### Windowsの場合の前提条件

- [Visual Studio 2019](https://visualstudio.microsoft.com/vs/) - Pythonバインディングをビルドする場合は、インストール時にPython拡張機能を選択してください。また、Visual Studioの`python`と`pip`の実行可能ファイルを環境変数`PATH`で指定された場所に置いてください。
- [Git Bash](https://git-scm.com/download/win) - このツールは、mc\_rtcのコピーとインストールスクリプトの実行で使用します。
- [CMake](https://cmake.org/download/) - 入手可能な最新版をインストールしてください。
- [Boost](https://www.boost.org/) - [sourceforce](https://sourceforge.net/projects/boost/files/boost-binaries/)で入手可能な最新のバイナリをインストールしてください。お使いのコンピューターとVisual Studioのバージョンに適したバージョンを選択してください。例えば、64ビットコンピューターでBoost 1.72とVisual Studio 2019を使用する場合、[boost_1_72_0-msvc-14.2-64.exe](https://sourceforge.net/projects/boost/files/boost-binaries/1.72.0/boost_1_72_0-msvc-14.2-64.exe/download)を選択してください。インストール後、Boostをインストールした場所を環境変数`BOOST_ROOT`に設定し、Boost DLLへのパスを環境変数`PATH`に設定してください。例えば、`C:\local\boost_1_72_0`にインストールした場合、`C:\local\boost_1_72_0`を`BOOST_ROOT`に設定し、`%BOOST_ROOT%\lib64-msvc-14.2`を`PATH`に設定してください。
- [mingw-w64](https://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download) - WindowsでFortranコードをコンパイルするのに使用します。ダウンロード後、実行可能ファイルを実行してください。すると、バージョン、アーキテクチャー、スレッド、例外、ビルドバージョンを選択するよう求められます。バージョンは最新版（デフォルト）を選択してください。アーキテクチャーは、現在最も一般的な64ビット版Windowsでビルドする場合はx86\_64を選択してください。スレッドはwin32を選択してください。例外はseh（デフォルト）を選択してください。インストール後、インストール先の`bin`フォルダーへのパスが環境変数`PATH`に設定されていることを確認してください（例: `C:\mingw-w64\x86_64-8.1.0-release-win32-seh-rt_v6-rev0\mingw64\bin`）。

注: Visual Studio 2017で作業してコンパイルすることもできます。ただし、定期的なテストはVisual Studio 2019でのみ行っています。

#### ビルド

1. [mc\_rtc](https://github.com/jrl-umi3218/mc_rtc)のリポジトリをコピーします。
2. mc\_rtcのディレクトリに移動し、サブモジュールを更新します: `git submodule update --init`
3. `utils`ディレクトリに移動し、`build_and_install.sh`という名前のファイルを探します。
4. （オプション）カスタム設定ファイル`build_and_install_user_config.sh`を作成します（デフォルトの設定ファイル`build_and_install_default_config.sh`に含まれる変数よりも、カスタム設定ファイルに含まれる変数のほうが優先されます）。
```sh
cp build_and_install_user_config.sample.sh build_and_install_user_config.sh
```
5. （オプション）`build_and_install_user_config.sh`を編集し、目的に応じて次のオプションを選択します: `INSTALL_PREFIX`、`WITH_ROS_SUPPORT`、`ROS_DISTRO`。Ubuntuの場合、ROSのサポートを有効にした場合にのみROSがインストールされます（事前にインストールされていません）。そうでない場合、ROSのサポート付きでmc\_rtcをインストールする前に、ご自身でROSをインストールする必要があります。
6. `./build_and_install.sh`を実行します。

このスクリプトによって、必要な依存パッケージがインストールされ、必要なソースコードがすべてコピーされ、ビルドとインストールが実行されます。処理が完了するまでしばらくかかります。

スクリプトの処理が失敗した場合は、mc\_rtcの課題トラッカーを開いて以下の情報を提供してください。

- システム（コンパイラー、ディストリビューション/OSXのバージョン）
- スクリプトの出力
- 関連すると思われる詳細情報

スクリプトの処理が成功すれば、作業は完了です。[次のセクション]({{site.baseurl}}/tutorials/introduction/configuration.html)に進んでください。

### ソースからビルドする（スクリプトを使用しない場合）

現在、他のプラットフォームでソースからビルドする方法についてドキュメント化されたものがあまりありません。この方法にチャレンジする場合、mc\_rtcをビルドするのに以下のパッケージが必要となります。
- [CMake](https://cmake.org/) 3.1以降
- [Boost](https://www.boost.org/) 1.49以降
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) 3.2以降
- [tinyxml2](https://github.com/leethomason/tinyxml2)
- [GEOS](https://trac.osgeo.org/geos)（C++バインディングが付属）
- [LTDL](https://www.gnu.org/software/libtool/manual/html_node/Libltdl-interface.html) （Windowsユーザーの場合は不要）
- [nanomsg](https://github.com/nanomsg/nanomsg)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [spdlog](https://github.com/gabime/spdlog/) 1.6.1以降
- [hpp-spline](https://github.com/humanoid-path-planner/hpp-spline)
- [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg)
- [RBDyn](https://github.com/jrl-umi3218/RBDyn)
- [eigen-qld](https://github.com/jrl-umi3218/eigen-qld)
- [eigen-quadprog](https://github.com/jrl-umi3218/eigen-quadprog)
- [eigen-lssol](https://gite.lirmm.fr/multi-contact/eigen-lssol) （オプション）
- [sch-core](https://github.com/jrl-umi3218/sch-core)
- [Tasks](https://github.com/jrl-umi3218/Tasks)
- [mc_rtc_data](https://github.com/jrl-umi3218/mc_rtc_data)
- [state-observation](https://github.com/jrl-umi3218/state-observation)

mc\_rtcには、ロボットの状態をROSトピックとしてパブリッシュすることを可能にし、ROSのツール（RVizなど）との統合を容易にするためのROSプラグインも用意されています。これをビルドするには以下のものが必要です。

 * [mc_rtc_msgs](https://github.com/jrl-umi3218/mc_rtc_msgs)

Pythonバインディングを取得したい場合、以下のものが必要です。
 * [Cython](http://cython.org/) 0.2以降
 * [python-pip]()
 * [python-numpy]()
 * [python-nose]()
 * [python-coverage]()
 * [Eigen3ToPython](https://github.com/jrl-umi3218/Eigen3ToPython)
 * [sch-core-python](https://github.com/jrl-umi3218/sch-core-python)

mc\_rtcのツールを実行するには、さらに以下のPythonライブラリが必要です。
 * [python-git]() (pip name: `GitPython`)
 * [python-pyqt5]()

以下のパッケージは必須ではありませんが、追加の機能が使用できるようになります。
- [ROS](http://www.ros.org/)
- [mc\_rtc\_ros](https://github.com/jrl-umi3218/mc_rtc_ros)

ビルド時に`roscpp`を使用できる場合、`tf2_ros`と`sensor_msgs`も必要になります。これを使用すると、`ROS`フレームワークと`mc_rtc`がさらに統合され（ロボットの状態のパブリッシュなど）、コントローラーでROSの機能を使用できるようになります（`mc_rtc`内のどこでも使える`ros::NodeHandle`インスタンスが提供されます）。
