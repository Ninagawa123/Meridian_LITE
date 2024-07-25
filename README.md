
# Meridian_LITE v1.1.1

より改良・拡張しやすくするため, 大規模なリファクタリングを行いました.  
命名規則はLLVM準拠とし, 内容を「.clang-format」ファイルにコメントしています.  
変数名や関数名のルールもある程度整理しました.  
構成要素となるコードをモジュール化することで, 改造の見通しが立ちやすくなりました.  

  ![meridian_lite_flow_20240721](https://github.com/user-attachments/assets/f8a21144-ca6c-4bed-831c-d543e6ded62d)
  
ライブラリの関数や変数表など, システムの詳細については以下のサイトがありますが, こちらの情報はまだ古いのでご注意ください.  
https://ninagawa123.github.io/Meridian_info/#
  
# Meridianとは？  

Meridianはヒューマノイドの制御システムについてのオープンソースプロジェクトです.  
ホビーロボットのデジタルツイン化を簡単に実現し, PC上のシミュレーション空間とロボット実機をWiFi経由で1/100秒単位の更新頻度でデータリンクすることができます.  
  
システムの中核はMeridim配列という軽量で汎用的なデータ配列で, ロボットの状態データをコンパクトに格納できます.  
このデータ配列がデバイス間を高速に循環することで, リアルタイムな状態データの共有を可能にします.  
Meridim配列を中間プロトコルとして既存のシステムの間に挟むことで, 複数社のコマンドサーボやセンサ, Unityなどの開発環境などを自由に繋ぐことができ, またROSの入出力にも対応するため, シミュレーターなど多岐にわたるリソースを活用することができます.  
  
当リポジトリで取り扱う ”Meridian_LITE" はESP32DeckitCを使用するタイプで, 対応ボードはMeridian Board -LITE-となります.  
また, より高度な制御が可能となるESP32DeckitC+Teensy4.0を連携させて動作するMeridian_TWIN(対応ボードはMeridian Board Type.K)も開発済みです.  
  
Meridianは今後も用途に応じて様々なハードウェア, ソフトウェアに対応させていく予定です.  
  
[![sync](https://img.youtube.com/vi/4ymSV_Dot-U/0.jpg)](https://www.youtube.com/watch?v=4ymSV_Dot-U)  
100Hzデータリンクのデモ動画
  
[![dance](https://img.youtube.com/vi/Wfc9j4Pmr3E/0.jpg)](https://www.youtube.com/watch?v=Wfc9j4Pmr3E)  
100Hzダンスのデモ動画
  
# Meridian_LITE ボードについて
<img width="400" src="https://user-images.githubusercontent.com/8329123/177022808-50ccf555-4afd-450c-a07e-3302771d45cf.jpg">
  
"Meridian board -LITE-" はサーボ制御用の半二重通信回路2系統とSPI,I2Cなどの基本的な入出力ピンを備えたボードです.  
ESP32devkitCを搭載し, 当リポジトリのスクリプトを使用することにより, 手軽に100Hzの更新頻度をもつデジタルツインのロボットを実現することができます.  
  
## ピンアサイン  
  
<img width="800" alt="lite_pinassign" src="https://user-images.githubusercontent.com/8329123/177044311-0021c4bc-42ca-4f08-afd5-a440fdac624f.png">
ピンアサインは上記の通りです. <br>
IOがESP32DevkitCのピン番号に該当しているので, ESP32Devkitのデータシート等を参考に使用することができます.   
またSPIやSDカードを使用しない場合は, アサインされたピンをGNDと組み合わせるなどで他の役割を与えることもできます.   
Fとなっている箇所は未接続のピンとなっています. 背面で好きな箇所と導線をはんだ付けすることで自由に機能を与えることができます.  
搭載するESP32DevkitCはUSBコネクタがMeridian -LITE-のロゴ側を向くように設置してください.  
  
特にサーボコネクタを逆やズラして刺すと半二重回路に負荷がかかりボード上のICが一発で壊れるので, 接続は十分ご注意ください.   
  
## KHR-3HVへのマウントと機能拡張  
<img width="400" alt="SS 2267" src="https://user-images.githubusercontent.com/8329123/177022972-3c9931ae-cfe3-44bb-9145-84303330a387.png">
上図のようにKHR-3HVのランドセルに本体無改造で固定することができます.  
ランドセル側とボードの間に1~2mm程度のスペーサーが入れるとボード底面の干渉を回避できます.  
秋月電子で販売のSDカードホルダ[AE-MICRO-SD-DIP]をSPI端子にそのまま接続することができます. その場合は, SDカードホルダ側にメスのピンヘッダを取り付けてください.   
９軸センサについては秋月で販売のBNO055[AE-BNO055-BO]をI2Cに接続することを標準としています.  
またリモコン受信機KRR-5FHも内臓できます. 左下のビス穴のみを使いた簡易固定ができます.  
蓋もギリギリですが閉じることができます.  
Wiiリモコンにも対応しており, すぐに使うことができます.    
  
# Meridian_LITE インストール方法
Meridian LITE のボードを使う方法です.   
開発環境として, VScodeとPlatformIOを使用します.  
※ArduinoIDEを使用した場合, WIFIライブラリの関係でESP32のパフォーマンスが発揮しきれません.  
    
## PlatformIOのインストール  
ご利用の環境にPlatformIOをインストールしてください.   
参考URL  
https://qiita.com/JotaroS/items/1930f156aab953194c9a  
https://platformio.org/  
  
## 開発環境のインストール  
PlatformIOを起動し, 「Platformes」の検索窓で「ESP32」を検索します.  
  
<img width="300" alt="1" src="https://user-images.githubusercontent.com/8329123/176886184-a702c39d-9b57-41f9-8653-66529a109976.png"><br>「Espressif 32」が見つかるので, バージョン「3.5.0」をインストールします.  
新しいバージョン(4.x.x)だとwifi関連がうまく動かない可能性が高いです.  
    
## ファイルをDLする  
<img width="419" alt="SS 925" src="https://github.com/Ninagawa123/Meridian_LITE/assets/8329123/cbb6f741-2690-48bd-85e9-90974a6d697a"><br>  
このサイトの右上の「CODE」からzip形式などを選択してDLし, 適切な場所に解凍, 展開してください.  
慣れてている方はもちろんgit cloneなどでもかまいません.  
  
## フォルダを開く  
VSCodeのファイルメニューから「フォルダを開く...」を選択し, 展開したファイルの中にある「Meridian_LITE_for_ESP32」を開きます.  
    
## ライブラリのインストール  
必要なライブラリはVSCode上で自動でインストールされます.  
もし, 自動でインストールされない場合には, 下記を参考に必要なモジュールをインストールしてください.   
  
### Meridianのライブラリを導入する   
アリ頭のアイコンから「QUICK ACCESS」→「PIO Home」→「Open」を開きます.  
右画面PIO Homeのタグの左メニューから「Libraries」を選択します.  
「Search libraries」となっている検索枠に「Meridian」と入力し, 「Meridian by Ninagawa123」を選択して「Add to Project」を押します. バージョンは0.1.0以上を使用してください.  
次に開くウインドの「Select a project」で今回のプロジェクト（Meridian_LITE_for_ESP32）を選択し, Addボタンを押します.  
  
### Adafruit_BNO055のライブラリを導入する  
上記と同様手順で, 「Search libraries」となっている検索枠に「BNO055」と入力し, Adafruit BNO055を選択して「Add to Project」を押します.  
  
## ESP32のシリアル通信ピンの設定  
ESP32のデフォルトではSerial1のUARTシリアル通信が使う事ができないため, 設定を変更して使えるようにします.  
  
PlatformIOを一旦閉じます.  

https://qiita.com/Ninagawa_Izumi/items/8ce2d55728fd5973087d  

を参考に,   
RX1を9番ピンから32番ピンに変更,  
TX1を10番ピンから27番ピンに変更する設定をしておきます.  

<img width="512" alt="SS 2059" src="https://github.com/Ninagawa123/Meridian_LITE/assets/8329123/ced42536-6a37-4852-94d7-18f542f1dfa7">

この設定ができていないと, サーボ通信は片方のチャンネルしか機能しません. 　　
　　 
## platformio.iniの設定  
「platformio.ini」を開くと下記のように設定されています.  
シリアルモニタのスピードを115200bpsとし, 自動インストールするモジュールを指定しています.  
またOTAという無線でのプログラム書き換え機能を削除してメモリ領域を増やす設定にしています.  
  
## keys.hの修正  
keys.h内の  
#define AP_SSID "xxxxxx"             // アクセスポイントのAP_SSID  
#define AP_PASS "xxxxxx"             // アクセスポイントのパスワード  
#define SEND_IP "192.168.1.xx"       // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）  
  
を使用環境に合わせて変更してください.  
送り先のPCのIPアドレスは,  
windowsならターミナルを開いてipconfigコマンド  
ubuntuならip aコマンド  
macなら画面右上のwifiアイコンから"ネットワーク"環境設定...  
で確認できます.   
    
## config.hの修正  
config.hの内容について, お手持ちの環境にあわせ適度に更新してください.  
設定の内容については, コード内にコメントを記しています.  
主な修正点は下記の通りです.  
  
#define SD_MOUNT 1  
→ SDリーダーの搭載 (0:なし, 1:あり)  
  
#define IMU_MOUNT 1  
→ 6軸or9軸センサーの搭載 (0:なし, 1:BNO055, 2:MPU6050(未実装))  
  
#define JOYPAD_MOUNT 2  
→ ジョイパッドの搭載 (現在2:KRC-5FHと5:wiimoteのみ有効, ジョイパッドを接続しない場合は0)  
  
/* 各サーボのマウントありなし（1:サーボあり, 0:サーボなし） */  
→ idl_mt[0]〜がサーボ左側系のサーボID 0〜 です. 接続しているサーボIDはTrue, 非接続のIDにはFalseを設定します.  
→ idr_mt[0]〜がサーボ右側系のサーボID 0〜 です. 上記と同様に設定します.  
  
/* 各サーボの直立デフォルト値　(KRS値  0deg=7500, +-90deg=7500+-2667  KRS値=deg/0.03375) */  
→ 各サーボについてのトリム値を設定できます.  
  
# ビルドとアップロード  
<img width="612" alt="9" src="https://user-images.githubusercontent.com/8329123/176913879-d05eb45d-15a0-47f6-a538-aa481b31e988.png"><br>
画面左下のチェックマークを押すと, ビルドが行われます.  
押下して「====== [SUCCESS] Took x.xx seconds」と表示されればビルド成功です.  
  
PCとESP32をUSBケーブルで接続し, 矢印ボタンを押すとESP32の内容が上書きされます.  
##### ESP32のアップロードがうまくいかない場合  
アップロードが失敗する場合でも, 何度か行うことで成功する場合があるので試してみてください.  
アップロード開始時にESP32DeckitCのENボタンを押すことでアップロードがうまくいく場合もあります.  
また, ESP32DeckitCのENとGNDの間に10uFのセラミックコンデンサを入れると, ENボタンを押さずとも書き込みができるようになる場合があります.

# ボードとロボットの起動  
これでボード側の準備が整いました.  
PCとボードをUSBで接続した状態でボードを起動すると,シリアルモニタに起動時のステータスがメッセージとして表示されます.  
ただし,PCとの連携にはPC側でMeridian Consolenを立ち上げておくなどの準備が必要になります.  
  
# Meridian consoleを実行する

Meridianで受け取るデータを表示できるコンソールを用意しました.python3が使える環境で実行可能です.
下記のリポジトリより, PC側の設定を行い, 実行してください.
https://github.com/Ninagawa123/Meridian_console
![meridian_console](https://raw.githubusercontent.com/Ninagawa123/Meridian_console/main/image/console_img.jpg)  


#  Unity版デモを実行する  
  
Meridian_TWINとUnityを連携させることができます.  
下記のリポジトリの内容をお試しください.  
[https://github.com/Ninagawa123/Meridian_Unity/tree/main](https://github.com/Ninagawa123/Meridian_Unity/tree/main)  
  
<img width="500" alt="Meridian_Unity" src="https://github.com/Ninagawa123/Meridian_TWIN/assets/8329123/5b486e83-40b8-4556-8a98-8d0ac643effd">
  
  
# ROS版デモを実行する  
  
Meridian_TWINとUnityを連携させることができます.  
下記のリポジトリより「ROS版デモを実行する」をお試しください.  
[https://github.com/Ninagawa123/Meridian_TWIN/edit/main/README.md](https://github.com/Ninagawa123/Meridian_TWIN/edit/main/README.md)  
  
# Wiiリモコンの使用方法  
config.h内の設定値を以下のように変更してください.    
#define MOUNT_JOYPAD 2       // ジョイパッドの搭載  
#define JOYPAD_POLLING 10    // 上記JOYPADのデータを読みに行くフレーム間隔  
起動後, wiiリモコンのABボタンを同時押し, 接続を確立します.  
接続が確立するとリモコンのランプが点滅から点灯に変化し, シリアルモニタには"Wiimote successfully connected."と表示されます.
  

# バージョン更新履歴  
- 2023.09.15 v1.0.1  
#define ESP32_STDALONE 0 をconfig.hに追加し, 値を1に設定することでESP32単体で通信テストが行えるようにした.
その際,　サーボ値は調べず, 代わりにL0番のサーボ値として+-30度のサインカーブを代入しつづける.  

# トラブルシューティング  
### サーボ通信が片方しか使えない！  
**原因１: ESP32のピン設定が反映されてていない**  
ESP32DeckitCはデフォルト状態ではMeridianBoard-LITE-でシリアルを2チャンネル使うことはできません.  
上記「ESP32のシリアル通信ピンの設定」から, ESP32DevkitCのピン設定を確認してください. PlatformIO側の自動更新などで設定が戻ることもあるかもしれません.  
**2024.05.08追記：ピンアサインの設定方法が変更になったようです。対応方法は[こちら](https://qiita.com/Ninagawa123/items/8ce2d55728fd5973087d)をご覧ください。**  
**原因２: 半二重回路の故障**  
サーボと接続する半二重回路は繊細で, サーボコネクタの逆挿しやズラし挿しなどで故障してしまします.  
ちょっと大変ですが表面実装を修理できる道具やスキルがある方はご自身で修理することも可能です. 修理用のチップをBOOTHにて頒布しています.   
  
### BNO055でデータが取得できない！  
**原因: プルアップが必要です**  
BNO_055とのI2C通信がうまくいかない場合は, 写真のように10kΩ程度の抵抗でプルアップすることでI2Cの通信品質が改善する場合があります.  
<img width="400" src="https://github.com/Ninagawa123/Meridian_LITE/blob/main/images/pullup.jpg">  

### PC連携時に数%の通信エラーが出る！  
**原因１: 仕様の範囲内**  
特にサーボとのシリアル通信はESP32のスピードの限界ギリギリで動作しています.  
数秒に1フレーム程度のエラーが1%未満発生する場合がありますが, 正常な動作で, これをソフトウェア的に補完しています.  
**原因２: サーボへの電力供給不足**  
サーボへの電源供給が不足していると, サーボとの通信が不安定になります. その場合はサーボに正しく電力を供給してください.  
USBポートからの電力供給でもサーボが反応する場合もありますが, 反応しないサーボも多いです.  
**原因３: ボードの加熱**  
ESP32が発熱しすぎた場合も誤動作の原因となります.  
ESP32にヒートシンクをつけるのもよいですし, 扇風機で風を当てるのも効果的です.  
  
### ESP32が不定期にリセットする！  
一時的な電力不足などにより使用中に強制リセットがかかる場合があります. ボードにも十分な電力を供給してください.  
USB供給の場, 供給器の容量が足りなくなる場合もあり得ます.  
