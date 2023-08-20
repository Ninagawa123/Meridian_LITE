//================================================================================================================
//---- M E R I D I A N - L I T E - E S P 3 2 の 配 線  ------------------------------------------------------------
//================================================================================================================
/*
 ESP32devkitC  -  デバイス
   3V3         -  BNO005 VIN
   21          -  BNO005 SCL
   22          -  BNO005 SDA
   GND         -  BNO005 GND

   4  EN       -  ICS変換基板 R系統 EN
   16 RX       -  ICS変換基板 R系統 TX
   17 TX       -  ICS変換基板 R系統 RX
   5V          -  ICS変換基板 IOREF
   GND         -  ICS変換基板 GND

   33 EN       -  ICS変換基板 L系統 EN
   32 RX       -  ICS変換基板 L系統 TX
   27 TX       -  ICS変換基板 L系統 RX
   5V          -  ICS変換基板 IOREF
   GND         -  ICS変換基板 GND

   27          -  SPI_MISO
   23          -  SPI_MOSI
   18          -  SPI_CSK
   15          -  SPI_CS SD
*/

//================================================================================================================
//---- サーボIDとロボット部位、軸との対応表 ----------------------------------------------------------------------------
//================================================================================================================
/*
  ID    Parts-Axis　＜ICS_Left_Upper SIO1,SIO2＞
  [L00] 頭ヨー
  [L01] 左肩ピッチ
  [L02] 左肩ロール
  [L03] 左肘ヨー
  [L04] 左肘ピッチ
  [L05] -
  ID    Parts-Axis　＜ICS_Left_Lower SIO3,SIO4＞
  [L06] 左股ロール
  [L07] 左股ピッチ
  [L08] 左膝ピッチ
  [L09] 左足首ピッチ
  [L10] 左足首ロール
  ID    Parts-Axis  ＜ICS_Right_Upper SIO5,SIO6＞
  [R00] 腰ヨー
  [R01] 右肩ピッチ
  [R02] 右肩ロール
  [R03] 右肘ヨー
  [R04] 右肘ピッチ
  [R05] -
  ID    Parts-Axis  ＜ICS_Right_Lower SIO7,SIO8＞
  [R06] 右股ロール
  [R07] 右股ピッチ
  [R08] 右膝ピッチ
  [R09] 右足首ピッチ
  [R10] 右足首ロール
*/

//================================================================================================================
//---- Meridim配列 一覧表 ------------------------------------------------------------------------------------------
//================================================================================================================
/*
  [00]      マスターコマンド デフォルトは90 で配列数も同時に示す
  [01]      シーケンシャルカウンタ
  [02]-[04] IMU/AHRS:acc＿x,acc＿y,acc＿z    加速度x,y,z
  [05]-[07] IMU/AHRS:gyro＿x,gyro＿y,gyro＿z ジャイロx,y,z
  [08]-[10] IMU/AHRS:mag＿x,mag＿y,mag＿z    磁気コンパスx,y,z
  [11]      IMU/AHRS:temp                   温度
  [12]-[14] IMU/AHRS:DMP ROLL,PITCH,YAW     DMP推定値 ロール,ピッチ,ヨー
  [15]      ボタンデータ1
  [16]      ボタンアナログ1（Stick Left）
  [17]      ボタンアナログ2（Stick Right）
  [18]      ボタンアナログ3（L2,R2 アナログ）
  [19]      モーション設定（フレーム数/イージング）
  [20]      サーボID LO  コマンド
  [21]      サーボID LO  データ値
  ...
  [48]      サーボID L14 コマンド
  [49]      サーボID L14 データ値
  [50]      サーボID RO  コマンド
  [51]      サーボID RO  データ値
  ...
  [78]      サーボID R14 コマンド
  [79]      サーボID R14 データ値
  [80]-[MSG_SIZE-3] free (Meridim90では[87]まで)
  [MSG_SIZE-2] ERROR CODE
  [MSG_SIZE-1] チェックサム
*/

/* Meridimの基本設定 */
#define MSG_SIZE 90       // Meridim配列の長さ設定（デフォルトは90）
#define FRAME_DURATION 10 // 1フレームあたりの単位時間（単位ms）

/* 各種ハードウェアのマウント有無 */
#define MOUNT_ESP32 1        // ESPの搭載 (0:なし-SPI通信およびUDP通信を実施しない, 1:あり)
#define MOUNT_SD 1           // SDカードリーダーの有無 (0:なし, 1:あり)
#define MOUNT_IMUAHRS 3      // IMU/AHRSの搭載状況 0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
#define MOUNT_ICS3 0         // 半二重サーボ信号の3系の有無 (0:なし, 1:あり)
#define MOUNT_JOYPAD 5       // ジョイパッドの搭載
                             // 0:なし, 1:SBDBT(未), 2:KRC-5FH, 3:PS3(未), 4:PS4 ,5:Wii_yoko,
                             // 6:Wii+Nun(未), 7:WiiPRO(未), 8:Xbox(未),
                             // 9:Merimote(未), 10:Retro
#define MOUNT_SERVO_NUM_L 11 // L系統につないだサーボの総数
#define MOUNT_SERVO_NUM_R 11 // R系統につないだサーボの総数
#define MOUNT_SERVO_NUM_3 0  // 3系統につないだサーボの総数

/* 動作チェックモード */
#define CHECK_SD_RW 1 // 起動時のSDカードリーダーの読み書きチェック

/* シリアルモニタリング */
#define MONITOR_JOYPAD 0    // シリアルモニタでリモコンのデータを表示（0:OFF, 1:ON）
#define MONITOR_FLOW 0      // シリアルモニタでフローを表示（0:OFF, 1:ON）
#define MONITOR_SEQ 0       // シリアルモニタでシーケンス番号チェックを表示（0:OFF, 1:ON）
#define MONITOR_SERVO_ERR 0 // シリアルモニタでサーボエラーを表示（0:OFF, 1:ON）

/* Wifiアクセスポイントの設定(SSID,パスワード等は別途keys.hで指定) */
#define UDP_TIMEOUT 4 // UDPの待受タイムアウト（単位ms,推奨値0）

/* ESP32のIPアドレスを固定する場合(固定IPアドレスは別途keys.hで指定) */
#define MODE_FIXED_IP 0 // IPアドレスを固定するか（0:NO, 1:YES）

/* リモコンの設定(ESP32自身のBluetoothMACアドレスは別途keys.hで指定) */
#define BT_PAIR_MAX_DEVICES 20     // BT接続デバイスの記憶可能数
#define BT_REMOVE_BONDED_DEVICES 0 // 0でバインドデバイス情報表示, 1でバインドデバイス情報クリア(BTリモコンがペアリング接続できない時に使用)
// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_WIIMOTE_SOLO[16] = {0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_WIIMOTE_ORIG[16] = {0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};

// PC接続関連設定
#define SERIAL_PC_BPS 115200 // PCとのシリアル速度（モニタリング表示用）

// SPI設定
#define SPI_SPEED 6000000 // SPI通信の速度（6000000kHz推奨）

/* UDP通信のオンオフ */
#define UDP_RESEIVE 1 // PCからのデータ受信（0:OFF, 1:ON, 通常は1）
#define UDP_SEND 1    // PCへのデータ送信（0:OFF, 1:ON, 通常は1）

// I2C設定, I2Cセンサ関連設定
#define I2C_SPEED 400000   // I2Cの速度（400kHz推奨）
#define IMUAHRS_POLLING 10 // IMU/AHRSのセンサの読み取り間隔(ms)
#define IMUAHRS_STOCK 4    // MPUで移動平均を取る際の元にする時系列データの個数

// サーボ関連設定
#define ICS_BAUDRATE 1250000    // ICSサーボの通信速度1.25M
#define ICS_TIMEOUT 2           // ICS返信待ちのタイムアウト時間
#define SERVO_LOST_ERROR_WAIT 4 // 連続何フレームサーボ信号をロストしたら異常とするか

// JOYPAD関連設定
#define JOYPAD_POLLING 10   // 上記JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨,BT系は10推奨)
#define JOYPAD_REFRESH 4    // JOYPADの受信ボタンデータをこのデバイスで0リセットするか、リセットせず論理加算するか （0:overide, 1:reflesh, 通常は1）
#define JOYPAD_GENERALIZE 1 // ジョイパッドの入力値をPS系に一般化する

/* 固定値, マスターコマンド定義 */
#define MCMD_UPDATE_YAW_CENTER 10002    // センサの推定ヨー軸を現在値センターとしてリセット
#define MCMD_ENTER_TRIM_MODE 10003      // トリムモードに入る（全サーボオンで垂直に気おつけ姿勢で立つ）
#define MCMD_CLEAR_SERVO_ERROR_ID 10004 // 通信エラーのサーボのIDをクリア(MSG_ERR_l)

/* ピンアサイン */
#define ERR_LED 25           // LED用 処理が時間内に収まっていない場合に点灯
#define PIN_EN_L 33          // サーボL系統のENピン
#define PIN_EN_R 4           // サーボR系統のENピン
#define SERVO_NUM_L 11       // L系統につないだサーボの数
#define SERVO_NUM_R 11       // R系統につないだサーボの数
#define PIN_CHIPSELECT_SD 15 // SDカード用のCSピン

//-------------------------------------------------------------------------
//---- サ ー ボ 設 定  -----------------------------------------------------
//-------------------------------------------------------------------------
/* 各サーボのマウントありなし（1:サーボあり、0:サーボなし） */
#define IDL_MT0 1  // 頭ヨー
#define IDL_MT1 1  // 左肩ピッチ
#define IDL_MT2 1  // 左肩ロール
#define IDL_MT3 1  // 左肘ヨー
#define IDL_MT4 1  // 左肘ピッチ
#define IDL_MT5 1  // 左股ヨー
#define IDL_MT6 1  // 左股ロール
#define IDL_MT7 1  // 左股ピッチ
#define IDL_MT8 1  // 左膝ピッチ
#define IDL_MT9 1  // 左足首ピッチ
#define IDL_MT10 1 // 左足首ロール
#define IDL_MT11 0 // 追加サーボ用
#define IDL_MT12 0 // 追加サーボ用
#define IDL_MT13 0 // 追加サーボ用
#define IDL_MT14 0 // 追加サーボ用
#define IDR_MT0 1  // 腰ヨー
#define IDR_MT1 1  // 右肩ピッチ
#define IDR_MT2 1  // 右肩ロール
#define IDR_MT3 1  // 右肘ヨー
#define IDR_MT4 1  // 右肘ピッチ
#define IDR_MT5 1  // 右股ヨー
#define IDR_MT6 1  // 右股ロール
#define IDR_MT7 1  // 右股ピッチ
#define IDR_MT8 1  // 右膝ピッチ
#define IDR_MT9 1  // 右足首ピッチ
#define IDR_MT10 1 // 右足首ロール
#define IDR_MT11 0 // 追加サーボ用
#define IDR_MT12 0 // 追加サーボ用
#define IDR_MT13 0 // 追加サーボ用
#define IDR_MT14 0 // 追加サーボ用

/* 各サーボの内外回転プラスマイナス方向補正(1 or -1) */
#define IDL_CW0 1  // 頭ヨー
#define IDL_CW1 1  // 左肩ピッチ
#define IDL_CW2 1  // 左肩ロール
#define IDL_CW3 1  // 左肘ヨー
#define IDL_CW4 1  // 左肘ピッチ
#define IDL_CW5 1  // 左股ヨー
#define IDL_CW6 1  // 左股ロール
#define IDL_CW7 1  // 左股ピッチ
#define IDL_CW8 1  // 左膝ピッチ
#define IDL_CW9 1  // 左足首ピッチ
#define IDL_CW10 1 // 左足首ロール
#define IDL_CW11 1 // 追加サーボ用
#define IDL_CW12 1 // 追加サーボ用
#define IDL_CW13 1 // 追加サーボ用
#define IDL_CW14 1 // 追加サーボ用
#define IDR_CW0 1  // 腰ヨー
#define IDR_CW1 1  // 右肩ピッチ
#define IDR_CW2 1  // 右肩ロール
#define IDR_CW3 1  // 右肘ヨー
#define IDR_CW4 1  // 右肘ピッチ
#define IDR_CW5 1  // 右股ヨー
#define IDR_CW6 1  // 右股ロール
#define IDR_CW7 1  // 右股ピッチ
#define IDR_CW8 1  // 右膝ピッチ
#define IDR_CW9 1  // 右足首ピッチ
#define IDR_CW10 1 // 右足首ロール
#define IDR_CW11 1 // 追加サーボ用
#define IDR_CW12 1 // 追加サーボ用
#define IDR_CW13 1 // 追加サーボ用
#define IDR_CW14 1 // 追加サーボ用

/* 各サーボの直立デフォルト値(degree) 直立状態になるよう、具体的な数値を入れて現物調整する */
#define IDL_TRIM0 0        // 頭ヨー
#define IDL_TRIM1 -2.3625  // 左肩ピッチ
#define IDL_TRIM2 -91.125  // 左肩ロール
#define IDL_TRIM3 0        // 左肘ヨー
#define IDL_TRIM4 89.9775  // 左肘ピッチ
#define IDL_TRIM5 0        // 左股ヨー
#define IDL_TRIM6 0        // 左股ロール
#define IDL_TRIM7 -1.35    // 左股ピッチ
#define IDL_TRIM8 -58.05   // 左膝ピッチ
#define IDL_TRIM9 -20.25   // 左足首ピッチ
#define IDL_TRIM10 -0.675  // 左足首ロール
#define IDL_TRIM11 0       // 追加サーボ用
#define IDL_TRIM12 0       // 追加サーボ用
#define IDL_TRIM13 0       // 追加サーボ用
#define IDL_TRIM14 0       // 追加サーボ用
#define IDR_TRIM0 0        // 腰ヨー
#define IDR_TRIM1 0        // 右肩ピッチ
#define IDR_TRIM2 -89.4375 // 右肩ロール
#define IDR_TRIM3 0        // 右肘ヨー
#define IDR_TRIM4 89.9775  // 右肘ピッチ
#define IDR_TRIM5 0        // 右股ヨー
#define IDR_TRIM6 1.6875   // 右股ロール
#define IDR_TRIM7 -3.375   // 右股ピッチ
#define IDR_TRIM8 -57.375  // 右膝ピッチ
#define IDR_TRIM9 -20.25   // 右足首ピッチ
#define IDR_TRIM10 -2.3625 // 右足首ロール
#define IDR_TRIM11 0       // 追加サーボ用
#define IDR_TRIM12 0       // 追加サーボ用
#define IDR_TRIM13 0       // 追加サーボ用
#define IDR_TRIM14 0       // 追加サーボ用

//-------------------------------------------------------------------------
//---- Meridim90 配列アクセス対応キー  ---------------------------------------
//-------------------------------------------------------------------------
#define MRD_MASTER 0              // マスターコマンド
#define MRD_SEQENTIAL 1           // シーケンス番号
#define MRD_ACC_X 2               // 加速度センサX値
#define MRD_ACC_Y 3               // 加速度センサY値
#define MRD_ACC_Z 4               // 加速度センサZ値
#define MRD_GYRO_X 5              // ジャイロセンサX値
#define MRD_GYRO_Y 6              // ジャイロセンサY値
#define MRD_GYRO_Z 7              // ジャイロセンサZ値
#define MRD_MAG_X 8               // 磁気コンパスX値
#define MRD_MAG_Y 9               // 磁気コンパスY値
#define MRD_MAG_Z 10              // 磁気コンパスZ値
#define MRD_TEMP 11               // 温度センサ値
#define MRD_DIR_ROLL 12           // DMP推定ロール方向値
#define MRD_DIR_PITCH 13          // DMP推定ピッチ方向値
#define MRD_DIR_YAW 14            // DMP推定ヨー方向値
#define MRD_CONTROL_BUTTONS 15    // リモコンの基本ボタン値
#define MRD_CONTROL_STICK_L 16    // リモコンの左スティックアナログ値
#define MRD_CONTROL_STICK_R 17    // リモコンの右スティックアナログ値
#define MRD_CONTROL_L2R2ANALOG 18 // リモコンのL2R2ボタンアナログ値
#define MRD_MOTION_FRAMES 19      // モーション設定のフレーム数
#define HEAD_Y_CMD 20             // 頭ヨーのコマンド
#define HEAD_Y_VAL 21             // 頭ヨーの値
#define L_SHOULDER_P_CMD 22       // 左肩ピッチのコマンド
#define L_SHOULDER_P_VAL 23       // 左肩ピッチの値
#define L_SHOULDER_R_CMD 24       // 左肩ロールのコマンド
#define L_SHOULDER_R_VAL 25       // 左肩ロールの値
#define L_ELBOW_Y_CMD 26          // 左肘ヨーのコマンド
#define L_ELBOW_Y_VAL 27          // 左肘ヨーの値
#define L_ELBOW_P_CMD 28          // 左肘ピッチのコマンド
#define L_ELBOW_P_VAL 29          // 左肘ピッチの値
#define L_HIPJOINT_Y_CMD 30       // 左股ヨーのコマンド
#define L_HIPJOINT_Y_VAL 31       // 左股ヨーの値
#define L_HIPJOINT_R_CMD 32       // 左股ロールのコマンド
#define L_HIPJOINT_R_VAL 33       // 左股ロールの値
#define L_HIPJOINT_P_CMD 34       // 左股ピッチのコマンド
#define L_HIPJOINT_P_VAL 35       // 左股ピッチの値
#define L_KNEE_P_CMD 36           // 左膝ピッチのコマンド
#define L_KNEE_P_VAL 37           // 左膝ピッチの値
#define L_ANKLE_P_CMD 38          // 左足首ピッチのコマンド
#define L_ANKLE_P_VAL 39          // 左足首ピッチの値
#define L_ANKLE_R_CMD 40          // 左足首ロールのコマンド
#define L_ANKLE_R_VAL 41          // 左足首ロールの値
#define L_SERVO_ID11_CMD 42       // 追加サーボ用のコマンド
#define L_SERVO_ID11_VAL 43       // 追加サーボ用の値
#define L_SERVO_ID12_CMD 44       // 追加サーボ用のコマンド
#define L_SERVO_ID12_VAL 45       // 追加サーボ用の値
#define L_SERVO_ID13_CMD 46       // 追加サーボ用のコマンド
#define L_SERVO_ID13_VAL 47       // 追加サーボ用の値
#define L_SERVO_ID14_CMD 48       // 追加サーボ用のコマンド
#define L_SERVO_ID14_VAL 49       // 追加サーボ用の値
#define WAIST_Y_CMD 50            // 腰ヨーのコマンド
#define WAIST_Y_VAL 51            // 腰ヨーの値
#define R_SHOULDER_P_CMD 52       // 右肩ピッチのコマンド
#define R_SHOULDER_P_VAL 53       // 右肩ピッチの値
#define R_SHOULDER_R_CMD 54       // 右肩ロールのコマンド
#define R_SHOULDER_R_VAL 55       // 右肩ロールの値
#define R_ELBOW_Y_CMD 56          // 右肘ヨーのコマンド
#define R_ELBOW_Y_VAL 57          // 右肘ヨーの値
#define R_ELBOW_P_CMD 58          // 右肘ピッチのコマンド
#define R_ELBOW_P_VAL 59          // 右肘ピッチの値
#define R_HIPJOINT_Y_CMD 60       // 右股ヨーのコマンド
#define R_HIPJOINT_Y_VAL 61       // 右股ヨーの値
#define R_HIPJOINT_R_CMD 62       // 右股ロールのコマンド
#define R_HIPJOINT_R_VAL 63       // 右股ロールの値
#define R_HIPJOINT_P_CMD 64       // 右股ピッチのコマンド
#define R_HIPJOINT_P_VAL 65       // 右股ピッチの値
#define R_KNEE_P_CMD 66           // 右膝ピッチのコマンド
#define R_KNEE_P_VAL 67           // 右膝ピッチの値
#define R_ANKLE_P_CMD 68          // 右足首ピッチのコマンド
#define R_ANKLE_P_VAL 69          // 右足首ピッチの値
#define R_ANKLE_R_CMD 70          // 右足首ロールのコマンド
#define R_ANKLE_R_VAL 71          // 右足首ロールの値
#define R_SERVO_ID11_CMD 72       // 追加テスト用のコマンド
#define R_SERVO_ID11_VAL 73       // 追加テスト用の値
#define R_SERVO_ID12_CMD 74       // 追加テスト用のコマンド
#define R_SERVO_ID12_VAL 75       // 追加テスト用の値
#define R_SERVO_ID13_CMD 76       // 追加テスト用のコマンド
#define R_SERVO_ID13_VAL 77       // 追加テスト用の値
#define R_SERVO_ID14_CMD 78       // 追加テスト用のコマンド
#define R_SERVO_ID14_VAL 79       // 追加テスト用の値
#define MRD_USERDATA_80 80        // ユーザー定義用
#define MRD_USERDATA_81 81        // ユーザー定義用
#define MRD_USERDATA_82 82        // ユーザー定義用
#define MRD_USERDATA_83 83        // ユーザー定義用
#define MRD_USERDATA_84 84        // ユーザー定義用
#define MRD_USERDATA_85 85        // ユーザー定義用
#define MRD_USERDATA_86 86        // ユーザー定義用
#define MRD_USERDATA_87 87        // ユーザー定義用
#define MRD_ERROR_CODE 88         // エラーコード
#define MRD_CHECKSUM 89           // チェックサム
