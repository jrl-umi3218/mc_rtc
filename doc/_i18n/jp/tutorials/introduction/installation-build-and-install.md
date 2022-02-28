## ソースからビルドする（スクリプトを使用する場合）

LinuxまたはmacOSでビルドする場合は、「ビルド」のセクションに進んでください。Windowsユーザーの場合は以下に示す追加の指示に従ってください。

### Windowsの場合の前提条件

- [Visual Studio 2019](https://visualstudio.microsoft.com/vs/) - Pythonバインディングをビルドする場合は、インストール時にPython拡張機能を選択してください。また、Visual Studioの`python`と`pip`の実行可能ファイルを環境変数`PATH`で指定された場所に置いてください。
- [Git Bash](https://git-scm.com/download/win) - このツールは、mc\_rtcのコピーとインストールスクリプトの実行で使用します。
- [CMake](https://cmake.org/download/) - 入手可能な最新版をインストールしてください。
- [Boost](https://www.boost.org/) - [sourceforce](https://sourceforge.net/projects/boost/files/boost-binaries/)で入手可能な最新のバイナリをインストールしてください。お使いのコンピューターとVisual Studioのバージョンに適したバージョンを選択してください。例えば、64ビットコンピューターでBoost 1.72とVisual Studio 2019を使用する場合、[boost_1_72_0-msvc-14.2-64.exe](https://sourceforge.net/projects/boost/files/boost-binaries/1.72.0/boost_1_72_0-msvc-14.2-64.exe/download)を選択してください。インストール後、Boostをインストールした場所を環境変数`BOOST_ROOT`に設定し、Boost DLLへのパスを環境変数`PATH`に設定してください。例えば、`C:\local\boost_1_72_0`にインストールした場合、`C:\local\boost_1_72_0`を`BOOST_ROOT`に設定し、`%BOOST_ROOT%\lib64-msvc-14.2`を`PATH`に設定してください。
- [mingw-w64](https://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download) - WindowsでFortranコードをコンパイルするのに使用します。ダウンロード後、実行可能ファイルを実行してください。すると、バージョン、アーキテクチャー、スレッド、例外、ビルドバージョンを選択するよう求められます。バージョンは最新版（デフォルト）を選択してください。アーキテクチャーは、現在最も一般的な64ビット版Windowsでビルドする場合はx86\_64を選択してください。スレッドはwin32を選択してください。例外はseh（デフォルト）を選択してください。インストール後、インストール先の`bin`フォルダーへのパスが環境変数`PATH`に設定されていることを確認してください（例: `C:\mingw-w64\x86_64-8.1.0-release-win32-seh-rt_v6-rev0\mingw64\bin`）。

注: Visual Studio 2017で作業してコンパイルすることもできます。ただし、定期的なテストはVisual Studio 2019でのみ行っています。

### ビルド

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

## はじめましょう

スクリプトの処理が成功すれば、作業は完了です。[次のセクション]({{site.baseurl}}/tutorials/introduction/configuration.html)に進んでください。

