# Meridian_LITE インストール方法

ざっくりメモ。詳細は後ほど検証＆更新。

■ 環境のインストール
PlatformIOを起動
Platformesの検索窓で「ESP32」を検索。
「Espressif 32」が見つかるので、バージョン「3.5.0」をインストール。

■ 新規プロジェクトを作成する
画像の通りに選ぶ。

■ ライブラリのインストール
・IcsHardSerialClassを導入する。

先ほど作成したプロジェクトのフォルダのある場所を探す。
その中に「lib」があるのを確認する。

サイトより
ICS_Library_for_Arduino_V2_1.zip
をDLし、解凍する。さらにその中のIcsClass_V210.zipを解凍する。
IcsClass_V210のフォルダを、さきほどの「lib」に入れる。

・Adafruit_BNO055を導入する
アリ頭のアイコンから「QUICK ACCESS」→「PIO Home」→「Open」を開く。
右画面PIO Homeのタグの左メニューから「Libraries」を選択。
「BNO055」を入れて検索。「Adafruit BNO055」を選び、「Add to Project」を押す。
今回のプロジェクト（Meridian_LITE_xxxxxx）を選択し、Addボタンを押す。

再び画面左上のファイルアイコンを押し、「src」→「main.cpp」を選択。
github（URL）のコードをここにコピペする。

■ ESP32のシリアルピンの設定

PlatformIOを一旦閉じる。
https://qiita.com/Ninagawa_Izumi/items/8ce2d55728fd5973087d
を参考に、
RX1を9番ピンから32番ピンに変更、
TX1を10番ピンから27番ピンに変更する設定をしておく。


■ 環境設定
「platformio.ini」を開くと、

////
となっているので、monitor_speed = 500000を加え以下のようにする。

[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 500000
lib_deps = adafruit/Adafruit BNO055@^1.5.3

とする。



■ コードの設定
#define AP_SSID "xxxxxx"             // アクセスポイントのAP_SSID
#define AP_PASS "xxxxxx"             // アクセスポイントのパスワード
#define SEND_IP "192.168.1.xx"       // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
を変更する。

■ ビルドとアップロード
左下のチェックマークを押すと、ビルドを行う。
押下して「====== [SUCCESS] Took x.xx seconds」と表示されればビルド成功。

PCとESP32をUSBケーブルせ接続し、矢印ボタンを押すとESP32の内容が上書きされるえ。
アップロードが失敗する場合でも、何度か行うことで成功する場合があるので試してみてください。

