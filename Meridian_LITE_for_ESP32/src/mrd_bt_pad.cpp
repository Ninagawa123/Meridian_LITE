// mrd_bt_pad.cpp
// Bluetoothパッド関数の実装

// ヘッダファイルの読み込み
#include "mrd_bt_pad.h"

// ライブラリ導入

//==================================================================================================
// 定数定義
//==================================================================================================
ESP32Wiimote m_wiimote;
PadUnion pad_array = {0};           // pad値の格納用配列
extern SemaphoreHandle_t pad_mutex; // PADデータアクセス用mutex

// リモコンボタンデータ変換テーブル
constexpr unsigned short PAD_TABLE_WIIMOTE_SOLO[16] = {
    0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_TABLE_WIIMOTE_ORIG[16] = {
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000,
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};
constexpr unsigned short PAD_TABLE_KRC5FH_TO_COMMON[16] = { //
    0, 64, 32, 128, 1, 4, 2, 8, 1024, 4096, 512, 2048, 16, 64, 32, 256};

// KRC-5FH 十字キーコンボ検出用ボタンマスク
constexpr uint16_t KRC_DPAD_LEFT_MASK = 0x000F;   // bit 0-3: 左十字キー (UP,DOWN,LEFT,RIGHT)
constexpr uint16_t KRC_DPAD_RIGHT_MASK = 0x0170;  // bit 4,5,6,8: 右十字キーボタン (368)
constexpr uint16_t KRC_CLEAR_DPAD_LEFT = 0xFFF0;  // bit 0-3 をクリア
constexpr uint16_t KRC_CLEAR_DPAD_RIGHT = 0xFE8F; // bit 4,5,6,8 をクリア
// ボタンテーブル変換から残った可能性のあるbit 1,2 をクリア
constexpr uint16_t KRC_CLEAR_RESIDUAL_BITS = 0xFFF9; // 0b1111111111111001

//==================================================================================================
//  タイプ別JOYPAD読み取り
//==================================================================================================

//----------------------------------------------------------------------
// KRC-5FH 読み取り
//----------------------------------------------------------------------

/// @brief KRC-5FHジョイパッドからデータを読み取り, 指定間隔で更新する
/// @param a_interval 読み取り間隔 (ミリ秒)
/// @param a_ics ICSシリアルクラスインスタンス
/// @return 更新されたジョイパッド状態を64ビット整数で返す
uint64_t mrd_pad_read_krc(uint a_interval, IcsHardSerialClass &a_ics) {
  static uint64_t pre_val_tmp = 0; // 前回値用の静的変数
  int8_t pad_analog_tmp[4] = {0};  // アナログ入力データ組み立て用
  static int calib[4] = {0};       // アナログスティックキャリブレーション値

  static unsigned long last_time_tmp = 0; // 前回呼び出し時刻
  unsigned long current_time_tmp = millis();

  if (current_time_tmp - last_time_tmp >= a_interval) {
    unsigned short krr_button_tmp;     // krrからのボタン入力データ
    int krr_analog_tmp[4];             // krrからのアナログ入力データ
    unsigned short pad_common_tmp = 0; // PS標準変換後のボタンデータ
    bool rcvd_tmp;                     // 受信機が正常にデータを受信したか
    rcvd_tmp = a_ics.getKrrAllData(&krr_button_tmp, krr_analog_tmp);
    delayMicroseconds(2);

    if (rcvd_tmp) { // リモコンデータが受信できていたら
                    // ボタンデータ処理

      int button_tmp = krr_button_tmp; // 受信したボタンデータを読み取り

      if (PAD_GENERALIZE) { // ボタンデータ汎用化処理
        // 左十字キー全押し -> SELECTとして扱う
        if ((button_tmp & KRC_DPAD_LEFT_MASK) == KRC_DPAD_LEFT_MASK) {
          pad_common_tmp += 1;
          button_tmp &= KRC_CLEAR_DPAD_LEFT;
        }

        // 右十字キー全押し -> STARTとして扱う
        if ((button_tmp & KRC_DPAD_RIGHT_MASK) == KRC_DPAD_RIGHT_MASK) {
          pad_common_tmp += 8;
          button_tmp &= KRC_CLEAR_DPAD_RIGHT;
        }

        // ボタン値変換 (汎用化)
        for (int i = 0; i < 16; i++) {
          uint16_t mask_tmp = 1 << i;
          if (PAD_TABLE_KRC5FH_TO_COMMON[i] & button_tmp) {
            pad_common_tmp |= mask_tmp;
          }
        }
        pad_common_tmp &= KRC_CLEAR_RESIDUAL_BITS; // 変換からの残留ビットをクリア

        // アナログ入力データ処理
        if (krr_analog_tmp[0] + krr_analog_tmp[1] + krr_analog_tmp[2] + krr_analog_tmp[3]) {
          for (int i = 0; i < 4; i++) {
            pad_analog_tmp[i] = (krr_analog_tmp[i] - 62) << 2;
            pad_analog_tmp[i] = (pad_analog_tmp[i] < -127) ? -127 : pad_analog_tmp[i];
            pad_analog_tmp[i] = (pad_analog_tmp[i] > 127) ? 127 : pad_analog_tmp[i];
          }
        } else
          for (int i = 0; i < 4; i++) {
            pad_analog_tmp[i] = 0;
          }
      } else {
        pad_common_tmp = button_tmp; // 変換なしの生ボタン値を使用
      }
    }

    // アナログスティックキャリブレーション
    // [WIP]

    // データ組み立て
    uint64_t updated_val_tmp = 0;
    updated_val_tmp = static_cast<uint64_t>(pad_common_tmp);
    updated_val_tmp |= ((uint64_t)pad_analog_tmp[0] & 0xFF) << 16;
    updated_val_tmp |= ((uint64_t)pad_analog_tmp[1] & 0xFF) << 24;
    //   updated_val_tmp |= ((uint64_t)pad_analog_tmp[2] & 0xFF) << 32;
    //   updated_val_tmp |= ((uint64_t)pad_analog_tmp[3] & 0xFF) << 40;

    last_time_tmp = current_time_tmp; // 前回実行時刻を更新
    pre_val_tmp = updated_val_tmp;
    return updated_val_tmp;
  }
  return pre_val_tmp;
}

//----------------------------------------------------------------------
// WIIMOTE 読み取り
//----------------------------------------------------------------------

/// @brief Wiiリモコンからの入力データを受信・処理する
/// @return 更新されたジョイパッド状態を64ビット整数で返す
/// @note 内部でESP32Wiimoteインスタンスwiimoteと定数PAD_GENERALIZEを使用
uint64_t mrd_bt_read_wiimote() {
  static uint64_t pre_val_tmp = 0; // 前回値用の静的変数
  static int calib_l1x = 0;
  static int calib_l1y = 0;

  // 受信データを問い合わせ
  m_wiimote.task();
  ButtonState rcvd_button_tmp;
  NunchukState nunchuk_tmp;
  // AccelState accel_tmp;

  if (m_wiimote.available() > 0) {

    // リモコンデータを取得
    rcvd_button_tmp = m_wiimote.getButtonState();
    nunchuk_tmp = m_wiimote.getNunchukState();

    uint16_t new_pad_tmp[4] = {0}; // アナログ入力データ組み立て用

    // ボタン値変換 (汎用化)
    for (int i = 0; i < 16; i++) {
      uint16_t mask_tmp = 1 << i;
      if ((PAD_GENERALIZE && (PAD_TABLE_WIIMOTE_SOLO[i] & rcvd_button_tmp)) ||
          (!PAD_GENERALIZE && (PAD_TABLE_WIIMOTE_ORIG[i] & rcvd_button_tmp))) {
        new_pad_tmp[0] |= mask_tmp;
      }
    }

    if (rcvd_button_tmp & BUTTON_C) { // ヌンチャクCボタン処理
      if (PAD_GENERALIZE) {
        new_pad_tmp[0] |= 1024;
      } else {
        new_pad_tmp[0] |= 8192;
      }
    }

    if (rcvd_button_tmp & BUTTON_Z) { // ヌンチャクZボタン処理
      if (PAD_GENERALIZE) {
        new_pad_tmp[0] |= 2048;
      } else {
        new_pad_tmp[0] |= 16384;
      }
    }

    if (rcvd_button_tmp & BUTTON_HOME) { // ホームボタンでスティックキャリブレーション
      calib_l1x = nunchuk_tmp.xStick - 127;
      calib_l1y = nunchuk_tmp.yStick - 127;
    }

    // ヌンチャクの値を含める
    new_pad_tmp[1] = ((nunchuk_tmp.xStick - calib_l1x - 127) * 256 //
                      + (nunchuk_tmp.yStick - calib_l1y - 127));

    // データ組み立て
    uint64_t new_val_tmp = 0; // 戻り値用
    new_val_tmp = static_cast<uint64_t>(new_pad_tmp[0]);
    new_val_tmp |= ((uint64_t)new_pad_tmp[1] << 16);
    //  new_val_tmp |= ((uint64_t)new_analog_tmp[2]) << 32;
    //  new_val_tmp |= ((uint64_t)new_analog_tmp[3]) << 40;

    pre_val_tmp = new_val_tmp;
    return new_val_tmp;
  }
  return pre_val_tmp;
}

//==================================================================================================
//  各種パッドへの分岐
//==================================================================================================

/// @brief 指定されたジョイパッドタイプに基づいて最新データを読み取り, 64ビット整数で返す
/// @param a_pad_type ジョイパッドタイプのenum (MERIMOTE, BLUERETRO, SBDBT, KRR5FH)
/// @param a_pad_data 64ビットボタンデータ
/// @param a_ics ICSシリアルクラスインスタンス (KRR5FH使用時)
/// @return 受信データを64ビット整数に変換したもの
/// @note WIIMOTEの場合, スレッドが自動的にpad_array.ui64valを更新
uint64_t mrd_pad_read(PadType a_pad_type, uint64_t a_pad_data, IcsHardSerialClass &a_ics) {

  if (a_pad_type == KRR5FH) { // KRR5FH
    return mrd_pad_read_krc(PAD_INTERVAL, a_ics);
  }

  if (a_pad_type == WIIMOTE) { // Wiimote
    return a_pad_data;
  }
  return 0;
}

//==================================================================================================
//  初期化とセットアップ
//==================================================================================================

//----------------------------------------------------------------------
// Bluetooth, WIIMOTE 初期化
//----------------------------------------------------------------------

/// @brief Bluetoothを設定し, Wiiコントローラー接続を開始する
/// @param a_mount_pad 搭載パッドタイプ
/// @param a_timeout 接続タイムアウト
/// @param a_wiimote Wiimoteインスタンス
/// @param a_led LEDピン
/// @param a_serial 出力シリアル
/// @return 成功時はtrue, タイムアウト時はfalse
bool mrd_bt_settings(PadType a_mount_pad, int a_timeout, int a_led, HardwareSerial &a_serial) {
  // Wiiコントローラー接続を開始
  if (a_mount_pad == WIIMOTE) {
    a_serial.println("Try to connect Wiimote...");
    m_wiimote.init();
    m_wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL);

    uint16_t count_tmp = 0;
    unsigned long start_time = millis();
    while (!m_wiimote.available()) {

      // リモコンを問い合わせ
      m_wiimote.task();

      // タイムアウトチェック
      if (millis() - start_time >= a_timeout) {
        digitalWrite(a_led, LOW);
        a_serial.println("Wiimote connection timed out.");
        return false;
      }

      // LED点滅
      count_tmp++;
      if (count_tmp < 500) {
        digitalWrite(a_led, HIGH);
      } else {
        digitalWrite(a_led, LOW);
      }
      if (count_tmp > 1000) {
        a_serial.print(".");
        count_tmp = 0;
      }

      delay(1); // 1ms待機して再チェック
    }
    digitalWrite(a_led, HIGH);
    a_serial.println("Wiimote successfully connected. ");
    return true;
  }
  digitalWrite(a_led, LOW);
  return false;
}

//----------------------------------------------------------------------
// WIIMOTE スレッド
//----------------------------------------------------------------------

/// @brief サブCPU (Core0) で実行されるBluetooth通信ルーチン
/// @param args この関数に渡される引数. 現在は未使用.
/// @note 内部でPadUnion型pad_array.ui64valと定数PAD_INTERVAL, WIIMOTEを使用
void Core0_BT_r(void *args) { // サブCPU (Core0) で実行されるプログラム
  while (true) {              // Bluetooth受信の無限ループ
    uint64_t new_val = mrd_bt_read_wiimote();
    // mutex保護下で共有変数に書き込み
    if (xSemaphoreTake(pad_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      pad_array.ui64val = new_val;
      xSemaphoreGive(pad_mutex);
    }
    vTaskDelay(PAD_INTERVAL); // 他のタスクにCPU時間を譲る
  }
}

//------------------------------------------------------------------------------------
//  meridimデータ書き込み
//------------------------------------------------------------------------------------

/// @brief PADデータをmeridim配列に書き込む
/// @param a_meridim Meridim配列共用体. 参照渡し.
/// @param a_pad_array PAD受信値用配列
/// @param a_marge PADボタンデータのマージ用ブール値.
///        true: 既存データとビット論理和, false: 新データで上書き.
/// @return 完了時にtrueを返す
bool meriput90_pad(Meridim90Union &a_meridim, PadUnion a_pad_array, bool a_marge) {

  // ボタンデータ処理 (マージまたは上書き)
  if (a_marge) {
    a_meridim.usval[MRD_PAD_BUTTONS] |= a_pad_array.usval[0];
  } else {
    a_meridim.usval[MRD_PAD_BUTTONS] = a_pad_array.usval[0];
  }

  // アナログ入力データ処理 (上書きのみ)
  for (int i = 1; i < 4; i++) {
    a_meridim.usval[MRD_PAD_BUTTONS + i] = a_pad_array.usval[i];
  }
  return true;
}
