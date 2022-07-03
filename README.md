# Meridian_LITE インストール方法
Meridian LITE のボードを使う方法です。<br>
開発環境として、VScodeとPlatformIOを使用します。<br>
※ArduinoIDEを使用した場合、WIFIライブラリの関係でESP32のパフォーマンスが発揮しきれません。<br>
<br>
<br>
## PlatformIOのインストール
ご利用の環境にPlatformIOをインストールしてください。<br>
参考URL  
https://qiita.com/JotaroS/items/1930f156aab953194c9a  
https://platformio.org/  
<br>
<br>
## 開発環境のインストール  
PlatformIOを起動し、「Platformes」の検索窓で「ESP32」を検索します。<br>
<br>
<img width="419" alt="1" src="https://user-images.githubusercontent.com/8329123/176886184-a702c39d-9b57-41f9-8653-66529a109976.png"><br>
「Espressif 32」が見つかるので、バージョン「3.5.0」をインストールします。<br>
新しいバージョン(4.x.x)だとwifi関連がうまく動かない可能性が高いです。<br>
<br>
<br>
## 新規プロジェクトを作成する  
<img width="1383" alt="2" src="https://user-images.githubusercontent.com/8329123/176886274-bbe57e27-7a14-4a1a-8df7-48d7f7bf495f.png"><br>
「Home」タブより「+New Project」を選びます。<br>
<br>
<br>
<img width="911" alt="3" src="https://user-images.githubusercontent.com/8329123/176886509-4ad96264-c89f-4e3f-9b69-21799f7ad162.png"><br>
Name:Meridian_LITE_xxxxxxx<br>
Board:Espressif ESP32 Dev Module<br>
Framework:Arduino Framework<br>
とします。<br>
<br>
<br>
## ライブラリのインストール
### IcsHardSerialClassを導入する
<img width="733" alt="SS 2266" src="https://user-images.githubusercontent.com/8329123/177020842-92d11f38-49b1-4cde-86f8-171febd849da.png">
PCのエクスプローラー(Macはファインダ)で内で先ほど作成したプロジェクトのフォルダのある場所を探します。<br>
その中に「lib」があるのを確認します。<br>　
<br>
<img width="521" alt="SS 2264" src="https://user-images.githubusercontent.com/8329123/177020750-c8210d92-e1d1-4970-a4c3-e04960a4a3f7.png">
近藤科学様のサイトより、ICS_Library_for_Arduino_V2_1.zip をDLし、解凍します。<br>
https://kondo-robot.com/faq/ics-library-a2 <br>
解凍後、さらにその中のIcsClass_V210.zipを解凍します。<br>
IcsClass_V210のフォルダを、さきほどの「lib」に入れます。<br>
<br>
### Adafruit_BNO055を導入する  
アリ頭のアイコンから「QUICK ACCESS」→「PIO Home」→「Open」を開きます。<br>
右画面PIO Homeのタグの左メニューから「Libraries」を選択します。<br>
<img width="839" alt="5" src="https://user-images.githubusercontent.com/8329123/176911609-83cf3795-f890-4c41-88dc-92c2efcfb4ff.png">
今回のプロジェクト（Meridian_LITE_xxxxxx）を選択し、Addボタンを押します。<br>
<br>
<br>
### メインコードの作成
<img width="665" alt="6" src="https://user-images.githubusercontent.com/8329123/176911852-b245204b-b8c9-4379-9db5-7176ab3901d0.png"><br>
再び画面左上のファイルアイコンを押し、「src」→「main.cpp」を選択します。<br>
githubのコード（[URL](https://github.com/Ninagawa123/Meridian_LITE/blob/main/ESP32/Meridian_LITE_20220612/main.cpp)）をmain.cppのコードとしてコピペします。<br>
<br>
<br>
### ESP32のシリアル通信ピンの設定  
ESP32のデフォルトでは一部のUARTシリアル通信が使う事ができないため、設定を変更して使えるようにします。<br>
<br>
PlatformIOを一旦閉じます。<br>
https://qiita.com/Ninagawa_Izumi/items/8ce2d55728fd5973087d  
を参考に、 <br>
RX1を9番ピンから32番ピンに変更、<br>
TX1を10番ピンから27番ピンに変更する設定をしておきます。<br>
<br>
<br>
### platformio.iniの設定  
「platformio.ini」を開くと、<br>
<img width="1169" alt="7" src="https://user-images.githubusercontent.com/8329123/176912503-e552925d-ca64-43e9-a7ae-8ce2ff32dd13.png"><br>
のようになっているので、以下のように書き加え、<br>
シリアルモニタのスピードを500000に、またOTAという無線でのプログラム書き換え機能を削除してメモリ領域を増やす設定にする。<br>
<br>
[env:esp32dev]<br>
platform = espressif32<br>
board = esp32dev<br>
framework = arduino<br>
monitor_speed = 500000<br>
board_build.partitions = no_ota.csv<br>
lib_deps = adafruit/Adafruit BNO055@^1.5.3<br>
<br>
<br>
### メインコードの修正<br>
main.cpp内の<br>
#define AP_SSID "xxxxxx"             // アクセスポイントのAP_SSID<br>
#define AP_PASS "xxxxxx"             // アクセスポイントのパスワード<br>
#define SEND_IP "192.168.1.xx"       // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）<br>
を変更する。<br>
送り先のPCのIPアドレスは、<br>
windowsならターミナルを開いてipconfigコマンド<br>
ubuntuならip aコマンド<br>
macなら画面右上のwifiアイコンから"ネットワーク"環境設定...<br>
で確認できる。<br>
<br>
<br>
### ビルドとアップロード<br>
<img width="612" alt="9" src="https://user-images.githubusercontent.com/8329123/176913879-d05eb45d-15a0-47f6-a538-aa481b31e988.png"><br>
画面左下のチェックマークを押すと、ビルドを行う。<br>
押下して「====== [SUCCESS] Took x.xx seconds」と表示されればビルド成功。<br>
<br>
PCとESP32をUSBケーブルせ接続し、矢印ボタンを押すとESP32の内容が上書きされる。<br>
アップロードが失敗する場合でも、何度か行うことで成功する場合があるので試してみてください。<br>
<br>
<br>
### PC側の設定<br>
以降のテキストは準備中ですが、下記を参考にPC側の設定をすることで、デモを動作できると思います。<br>
https://github.com/Ninagawa123/Meridian_core<br>
<br>
