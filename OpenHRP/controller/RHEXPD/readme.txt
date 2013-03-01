改良ControllerBridge対応RHEXPDコントローラコンポーネント

■概要
改良ControllerBridgeに対応したコンポーネントです。改良前と比較してRHEXPDの内容は変わりませんが、設定ファイルに変更があります。

■ファイル
Makefile          ... メークファイル
RHEXPD.cpp      ... コンポーネントソース
RHEXPD.h        ... コンポーネントヘッダ
RHEXPDComp.cpp  ... コンポーネント起動部
etc/              ... パターンファイルディレクトリ
etc/PDgain.dat    ... 制御定数
etc/angle.dat     ... 角度パターン
etc/vel.dat       ... 速度パターン
rtc.conf          ... RTコンポーネント設定ファイル
bridge.conf       ... コントローラブリッジ設定ファイル
RHEXPD_RTC.xml  ... プロジェクトファイル
RHEXPD.sh       ... 実行スクリプト
readme.txt        ... このファイル

■インストール
1. 解凍
ファイルアーカイブを解凍してください。

  $ tar zxvf RHEXPD.tar.gz

2. Makefileを編集する
Makefile 22行めにある以下を、OpenHRPのトップディレクトリを指すように編集してください。

TOP = ../../../


3. コンパイル
Makeを実行してください。

  $ make

以上です。

■起動方法
まず、ControllerBridgeのrtc.confを次のように設定してください。

manager.modules.load_path: (RHEXPDを展開したディレクトリ)
manager.modules.abs_path_allowed: yes

その上で、以下を実行してください。

コマンドラインから設定する場合:

$(OPENHRPHOME)/Controller/Server/ControllerBridge/ControllerBridge \
  --array-mapping-in angle:JOINT_VALUE \
  --array-mapping-out torque:JOINT_TORQUE \
  --port-connection angle:angle \
  --port-connection torque:torque \
  --module RHEXPD.so:MyModuleInit \
  --component RHEXPD0.rtc 

以上でRHEXPD0.rtcの名付けられたコンポーネントに関節速度と値をマッピングします。RHEXPD.shはこのオプションを実行します。

設定ファイルから設定する場合:

$(OPENHRPHOME)/Controller/Server/ControllerBridge/ControllerBridge \
  -p bridge.conf

以上でカレントディレクトリにあるbridge.confを読み込み起動します。

■シミュレーション
同梱のRHEXPD_RTC.xmlをGrxUIにロードし、シミュレーションを実行してください。
適切にコンポーネント同士を接続し、シミュレーションを行います。
