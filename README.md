# Meridian_LITE v1.0.0

Meridian公式ライブラリ対応版として大幅なアップデートを行いました.(前回までのバージョンはoldディレクトリにzipで格納しています.)  
ソースコードが整理され, 改造の見通しが立ちやすくなりました.  
　　
ライブラリの関数や変数表など, システムの詳細については以下をご参照ください. <br>  
https://ninagawa123.github.io/Meridian_info/#
  
# Meridianとは？  
  
Meridian計画はヒューマノイドの制御システムについてのオープンソースプロジェクトです.  
ホビーロボットのデジタルツイン化を簡単に実装することができ, PC上のシミュレーション空間とロボット実機をWIFI経由で1/100秒単位の更新頻度でデータリンクします.  
    
システムの中核はMeridim配列というロボットの状態データを格納できる軽量で汎用的なデータ配列です.  
このデータ配列がデバイス間を高速に廻ることで, リアルタイムな状態データを共有を可能にします.  
Meridim配列を中間プロトコルとして既存のシステムの間に挟むことで, 複数社のコマンドサーボやセンサ, Unityなどの開発環境, ROSで使用可能な多岐にわたるシミュレーターなどを自由に繋ぎ合わせることができます.  
  
当リポジトリで取り扱う ”Meridian_LITE" はESP32DeckitCを使用するタイプで, 対応ボードはMeridian Board -LITE-となります.  
また, より高度な制御が可能となるESP32DeckitC+Teensy4.0を連携させて動作するMeridian_TWIN(対応ボードはMeridian Board Type.K)も開発済みです.  
  
Meridianは今後も用途に応じて様々なハードウェア, ソフトウェアに対応させていく予定です.  
  
[![sync](https://img.youtube.com/vi/4ymSV_Dot-U/0.jpg)](https://www.youtube.com/watch?v=4ymSV_Dot-U)  
100Hzデータリンクのデモ動画
  
[![dance](https://img.youtube.com/vi/Wfc9j4Pmr3E/0.jpg)](https://www.youtube.com/watch?v=Wfc9j4Pmr3E)  
100Hzダンスのデモ動画

  
# Meridian_LITE ボードについて
<img width="400" src="https://user-images.githubusercontent.com/8329123/177022808-50ccf555-4afd-450c-a07e-3302771d45cf.jpg">
  
"Meridian board -LITE-" は半二重通信回路2系統を搭載とSPI,I2Cなどの出力ピンを備えたボードです. <br>
ESP32devkitCを搭載することで, 半二重通信方式のコマンドサーボを扱えるロボット用ボードとして機能します. <br>
当リポジトリのスクリプトを使用することにより, 100Hzの更新頻度をもつデジタルツインのロボットを実現することができます. <br>
<br>
  
## ピンアサイン
  
<img width="800" alt="lite_pinassign" src="https://user-images.githubusercontent.com/8329123/177044311-0021c4bc-42ca-4f08-afd5-a440fdac624f.png">
ピンアサインは上記の通りです. <br>
IOがESP32DevkitCのピン番号に該当しているので, ESP32Devkitのデータシート等を参考に使うことができます. <br>
またSPIやSDカードを使用しない場合は, アサインされたピンをGNDと組み合わせるなどで他の役割を与えることもできます. <br>
Fとなっている箇所は未接続のピンとなっています. 背面で好きな箇所と導線をはんだ付けすることで自由に機能を与えることができます. <br>
搭載するESP32DevkitCはUSBコネクタがMeridian -LITE-のロゴ側を向くように設置してください. <br>
<br>
特にサーボコネクタを逆やズラして刺すと半二重回路に負荷がかかりボード上のICが一発で壊れるので, 接続は十分ご注意ください. <br>
<br>
  
## マウントと機能拡張
<img width="400" alt="SS 2267" src="https://user-images.githubusercontent.com/8329123/177022972-3c9931ae-cfe3-44bb-9145-84303330a387.png">
上図のようにKHR-3HVのランドセルに本体無改造で固定することができます. <br>
ただしランドセル側とボードの間には1~2mm程度のスペーサーが入れるとボード底面の干渉を回避できます. <br>
秋月電子で販売のSDカードホルダ[AE-MICRO-SD-DIP]をSPI端子にそのまま接続することができます. <br>
ただしSDカードホルダ側にメスのピンヘッダを取り付ける必要があります. <br>
９軸センサについては秋月で販売のBNO055[AE-BNO055-BO]をI2Cに接続することを標準としています. <br>
またリモコン受信機KRR-5FHも内臓できます. 左下のビス穴のみを使いた簡易固定ができます. <br>
蓋もギリギリですが閉じることができます. <br>
<br>
  
# Meridian_LITE インストール方法
Meridian LITE のボードを使う方法です. <br>
開発環境として, VScodeとPlatformIOを使用します. <br>
※ArduinoIDEを使用した場合, WIFIライブラリの関係でESP32のパフォーマンスが発揮しきれません. <br>
<br>
  
## PlatformIOのインストール
ご利用の環境にPlatformIOをインストールしてください. <br>
参考URL  
https://qiita.com/JotaroS/items/1930f156aab953194c9a  
https://platformio.org/   
<br>
  
## 開発環境のインストール  
PlatformIOを起動し, 「Platformes」の検索窓で「ESP32」を検索します. <br>
<br>
<img width="300" alt="1" src="https://user-images.githubusercontent.com/8329123/176886184-a702c39d-9b57-41f9-8653-66529a109976.png"><br>
「Espressif 32」が見つかるので, バージョン「3.5.0」をインストールします. <br>
新しいバージョン(4.x.x)だとwifi関連がうまく動かない可能性が高いです. <br>
<br>
  
## ファイルをDLする  
<img width="419" alt="SS 925" src="https://github.com/Ninagawa123/Meridian_LITE/assets/8329123/cbb6f741-2690-48bd-85e9-90974a6d697a"><br>  
このサイトの右上の「CODE」からzip形式などを選択してDLし, 適切な場所に解凍, 展開してください.   
慣れてている方はもちろんgit cloneなどでもかまいません.   
  
## フォルダを開く
VSCodeのファイルメニューから「フォルダを開く...」を選択し, 展開したファイルの中にある「Meridian_LITE_for_ESP32」を開きます.   
<br>
  
## ライブラリのインストール  
  
必要なライブラリはVSCode上で自動でインストールされます.   
もし, 自動でインストールされない場合には, 下記を参考に必要なモジュールをインストールしてください.   
  
### Meridianを導入する  
アリ頭のアイコンから「QUICK ACCESS」→「PIO Home」→「Open」を開きます. <br>
右画面PIO Homeのタグの左メニューから「Libraries」を選択します. <br>
「Search libraries」となっている検索枠に「Meridian」と入力し, 「Meridian by Ninagawa123」を選択して「Add to Project」を押します. バージョンは0.1.0以上を使用してください. <br>
次に開くウインドの「Select a project」で今回のプロジェクト（Meridian_LITE_for_ESP32）を選択し, Addボタンを押します. <br>  
  
### Adafruit_BNO055を導入する
上記と同様手順で, 「Search libraries」となっている検索枠に「BNO055」と入力し, Adafruit BNO055を選択して「Add to Project」を押します. <br>

<br>

### ESP32のシリアル通信ピンの設定
ESP32のデフォルトではSerial1のUARTシリアル通信が使う事ができないため, 設定を変更して使えるようにします. <br>
<br>
PlatformIOを一旦閉じます. <br>
<br>
https://qiita.com/Ninagawa_Izumi/items/8ce2d55728fd5973087d  
を参考に,  <br>
RX1を9番ピンから32番ピンに変更, <br>
TX1を10番ピンから27番ピンに変更する設定をしておきます. <br>
<br>

### platformio.iniの設定
「platformio.ini」を開くと下記のように設定されています. <br>
シリアルモニタのスピードを115200bpsとし, 自動インストールするモジュールを指定しています. <br>
またOTAという無線でのプログラム書き換え機能を削除してメモリ領域を増やす設定にしています. <br>
[env:esp32dev]<br>
platform = espressif32<br>
board = esp32dev<br>
framework = arduino<br>
monitor_speed = 115200<br>
lib_deps = <br>
	ninagawa123/Meridian@^0.1.0<br>
	adafruit/Adafruit BNO055@^1.5.3<br>
board_build.partitions = no_ota.csv<br>
　　
<br>
  
### keys.hの修正
keys.h内の<br>
>#define AP_SSID "xxxxxx"             // アクセスポイントのAP_SSID<br>
>#define AP_PASS "xxxxxx"             // アクセスポイントのパスワード<br>
>#define SEND_IP "192.168.1.xx"       // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）<br>
<br>
を変更してください. <br>
送り先のPCのIPアドレスは, <br>
windowsならターミナルを開いてipconfigコマンド<br>
ubuntuならip aコマンド<br>
macなら画面右上のwifiアイコンから"ネットワーク"環境設定...<br>
で確認できます. <br>
<br>
  
### config.hの修正 
config.hの内容について, お手持ちの環境にあわせ適度に更新してください. <br>
設定の内容については, コード内にコメントを記しています. <br>
主な修正点は下記の通りです. <br>
  
<br>
#define SD_MOUNT 1 <br>
　→ SDリーダーの搭載 (0:なし, 1:あり)<br>
<br>
#define IMU_MOUNT 1<br>
　→ 6軸or9軸センサーの搭載 (0:なし, 1:BNO055, 2:MPU6050(未実装))<br>
<br>
#define JOYPAD_MOUNT 2<br>
　→ ジョイパッドの搭載 (現在2:KRC-5FHと5:wiimoteのみ有効, ジョイパッドを接続しない場合は0)<br>
<br>
/* 各サーボのマウントありなし（1:サーボあり, 0:サーボなし） */<br>
　→ idl_mt[0]〜がサーボ左側系のサーボID 0〜 です. 接続しているサーボIDはTrue, 非接続のIDにはFalseを設定します. <br>
　→ idr_mt[0]〜がサーボ右側系のサーボID 0〜 です. 上記と同様に設定します. <br>
<br>
/* 各サーボの直立デフォルト値　(KRS値  0deg=7500, +-90deg=7500+-2667  KRS値=deg/0.03375) */<br>
　→ 各サーボについてのトリム値を設定できます. <br>
<br>

### ビルドとアップロード
<img width="612" alt="9" src="https://user-images.githubusercontent.com/8329123/176913879-d05eb45d-15a0-47f6-a538-aa481b31e988.png"><br>
画面左下のチェックマークを押すと, ビルドが行われます. <br>
押下して「====== [SUCCESS] Took x.xx seconds」と表示されればビルド成功です. <br>
<br>
PCとESP32をUSBケーブルせ接続し, 矢印ボタンを押すとESP32の内容が上書きされます. <br>
アップロードが失敗する場合でも, 何度か行うことで成功する場合があるので試してみてください. <br>

### PC側の設定
PC側の設定をすることで, UnityやROSを使用したデジタルツインのデモを実行できます. <br>
下記のサイトより「Meridian consoleを実行する」以降をお試しください.   
[https://github.com/Ninagawa123/Meridian_core](https://github.com/Ninagawa123/Meridian_TWIN)<br>
<br>

### 補足
BNO_055とのI2C通信がうまくいかない場合は, 写真のように10kΩ程度の抵抗でプルアップすることでI2Cの通信品質が改善する場合があります. <br>
<img width="400" src="https://github.com/Ninagawa123/Meridian_LITE/blob/main/images/pullup.jpg"><br>
<br>
現在, 数秒に1フレーム程度のエラーが発生する場合がありますが, 正常な動作です. 
