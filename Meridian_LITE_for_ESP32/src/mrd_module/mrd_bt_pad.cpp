#include "mrd_bt_pad.h"

// グローバル変数の定義
ESP32Wiimote wiimote;

// KRR-5FH アナログスティック設定
constexpr int KRR_ANALOG_CENTER = 62;      // 中央値
constexpr int KRR_ANALOG_MIN = 4;          // 有効範囲の下限
constexpr int KRR_ANALOG_MAX = 120;        // 有効範囲の上限
constexpr int KRR_ANALOG_DEADZONE = 3;     // デッドゾーン閾値
constexpr int KRR_ANALOG_SCALE = 4;        // 出力スケーリング係数
constexpr int KRR_ANALOG_OUTPUT_MAX = 127; // 出力の最大値

//==================================================================================================
//  タイプ別のJOYPAD読み込み処理
//==================================================================================================

//----------------------------------------------------------------------
// KRC-5FHの読み込み
//----------------------------------------------------------------------

uint64_t mrd_pad_read_krc(uint a_interval, IcsHardSerialClass &a_ics) {
  static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
  int8_t pad_analog_tmp[4] = {0};  // アナログ入力のデータ組み立て用
  static int calib[4] = {0};       // アナログスティックのキャリブレーション値

  static unsigned long last_time_tmp = 0; // 最後に関数が呼ばれた時間を記録
  unsigned long current_time_tmp = millis();

  if (current_time_tmp - last_time_tmp >= a_interval) {
    unsigned short krr_button_tmp;     // krrからのボタン入力データ
    int krr_analog_tmp[4];             // krrからのアナログ入力データ
    unsigned short pad_common_tmp = 0; // PS準拠に変換後のボタンデータ
    bool rcvd_tmp;                     // 受信機がデータを受信成功したか
    rcvd_tmp = ics_R.getKrrAllData(&krr_button_tmp, krr_analog_tmp);
    delayMicroseconds(2);

    if (rcvd_tmp) // リモコンデータが受信できていたら
    {
      // ボタンデータの処理
      int button_tmp = krr_button_tmp; // 受信ボタンデータの読み込み用

      if (PAD_GENERALIZE) {            // ボタンデータの一般化処理
        if ((button_tmp & 15) == 15) { // 左側十字ボタン全部押しなら select押下とみなす
          pad_common_tmp += 1;
          button_tmp &= 0b1111111111110000; // 左十字ボタンのクリア
        }

        if ((button_tmp & 368) == 368) {
          pad_common_tmp += 8;              // 右側十字ボタン全部押しなら start押下とみなす
          button_tmp &= 0b1111111010001111; // 右十字ボタンのクリア
        }

        // ボタン値の変換(一般化)
        for (int i = 0; i < 16; i++) {
          uint16_t mask_tmp = 1 << i;
          if (PAD_TABLE_KRC5FH_TO_COMMON[i] & button_tmp) {
            pad_common_tmp |= mask_tmp;
          }
        }
        pad_common_tmp &= 0b1111111111111001; // 2と4のビットのクリア(データ調整)

        // アナログ入力データの処理
        static int8_t pad_analog_last[4] = {0}; // 前回の正常値を保持
        for (int i = 0; i < 4; i++) {
          int v = krr_analog_tmp[i];
          // 異常値判定: 範囲外なら前回の正常値を使用
          if (v < KRR_ANALOG_MIN || v > KRR_ANALOG_MAX) {
            pad_analog_tmp[i] = pad_analog_last[i];
            continue;
          }
          // 中央値からのオフセット
          v -= KRR_ANALOG_CENTER;
          // デッドゾーン: 中央付近は0
          if (v > -KRR_ANALOG_DEADZONE && v < KRR_ANALOG_DEADZONE) {
            pad_analog_tmp[i] = 0;
            pad_analog_last[i] = 0;
            continue;
          }
          // スケーリングしてクリップ
          int result = v * KRR_ANALOG_SCALE;
          if (result > KRR_ANALOG_OUTPUT_MAX)
            result = KRR_ANALOG_OUTPUT_MAX;
          if (result < -KRR_ANALOG_OUTPUT_MAX)
            result = -KRR_ANALOG_OUTPUT_MAX;
          pad_analog_tmp[i] = result;
          pad_analog_last[i] = result;
        }
      } else {
        pad_common_tmp = button_tmp; // ボタンの変換なし生値を使用
      }

      // アナログスティックのキャリブレーション
      // [WIP]

      // データの組み立て (受信成功時のみ)
      uint64_t updated_val_tmp = 0;
      updated_val_tmp = static_cast<uint64_t>(pad_common_tmp);
      updated_val_tmp |= ((uint64_t)pad_analog_tmp[0] & 0xFF) << 16;
      updated_val_tmp |= ((uint64_t)pad_analog_tmp[1] & 0xFF) << 24;
      updated_val_tmp |= ((uint64_t)pad_analog_tmp[2] & 0xFF) << 32;
      updated_val_tmp |= ((uint64_t)pad_analog_tmp[3] & 0xFF) << 40;

      last_time_tmp = current_time_tmp; // 成功時のみ更新
      pre_val_tmp = updated_val_tmp;    // 成功時のみ更新
      return updated_val_tmp;
    }
    // 受信失敗: last_time_tmp は更新しない (次フレームで即リトライ)
  }
  return pre_val_tmp; // 最後の正常値を返す
}

//----------------------------------------------------------------------
// WIIMOTEの読み込み
//----------------------------------------------------------------------

uint64_t mrd_bt_read_wiimote() {
  static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
  static int calib_l1x = 0;
  static int calib_l1y = 0;

  // 受信データの問い合わせ
  wiimote.task();
  ButtonState rcvd_button_tmp;
  NunchukState nunchuk_tmp;
  // AccelState accel_tmp;

  if (wiimote.available() > 0) {

    // リモコンデータの取得
    rcvd_button_tmp = wiimote.getButtonState();
    nunchuk_tmp = wiimote.getNunchukState();

    uint16_t new_pad_tmp[4] = {0}; // アナログ入力のデータ組み立て用

    // ボタン値の変換(一般化)
    for (int i = 0; i < 16; i++) {
      uint16_t mask_tmp = 1 << i;
      if ((PAD_GENERALIZE && (PAD_TABLE_WIIMOTE_SOLO[i] & rcvd_button_tmp)) ||
          (!PAD_GENERALIZE && (PAD_TABLE_WIIMOTE_ORIG[i] & rcvd_button_tmp))) {
        new_pad_tmp[0] |= mask_tmp;
      }
    }

    if (rcvd_button_tmp & BUTTON_C) { // ヌンチャクCボタンの処理
      if (PAD_GENERALIZE) {
        new_pad_tmp[0] |= 1024;
      } else {
        new_pad_tmp[0] |= 8192;
      }
    }

    if (rcvd_button_tmp & BUTTON_Z) { // ヌンチャクZボタンの処理
      if (PAD_GENERALIZE) {
        new_pad_tmp[0] |= 2048;
      } else {
        new_pad_tmp[0] |= 16384;
      }
    }

    if (rcvd_button_tmp & BUTTON_HOME) { // ホームボタンでスティックのキャリブレーション
      calib_l1x = nunchuk_tmp.xStick - 127;
      calib_l1y = nunchuk_tmp.yStick - 127;
    }

    // ヌンチャクの値を組み入れ
    new_pad_tmp[1] = ((nunchuk_tmp.xStick - calib_l1x - 127) * 256 //
                      + (nunchuk_tmp.yStick - calib_l1y - 127));

    // データの組み立て
    uint64_t new_val_tmp = 0; // 戻り値格納用
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

uint64_t mrd_pad_read(PadType a_pad_type, uint64_t a_pad_data) {

  if (a_pad_type == KRR5FH) { // KRR5FH
    return mrd_pad_read_krc(PAD_INTERVAL, ics_R);
  }

  if (a_pad_type == WIIMOTE) { // Wiimote
    return a_pad_data;
  }
  return 0;
}

//==================================================================================================
//  初期化と準備
//==================================================================================================

//----------------------------------------------------------------------
// Bluetooth, WIIMOTEの初期化
//----------------------------------------------------------------------

bool mrd_bt_settings(int a_mount_pad,
                     int a_timeout,
                     ESP32Wiimote &a_wiimote,
                     int a_led,
                     HardwareSerial &a_serial) {
  // Wiiコントローラの接続開始
  if (a_mount_pad == 5) {
    a_serial.println("Try to connect Wiimote...");
    a_wiimote.init();
    a_wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL);

    uint16_t count_tmp = 0;
    unsigned long start_time = millis();
    while (!a_wiimote.available()) {

      // リモコンへの問い合わせ
      a_wiimote.task();

      // タイムアウトチェック
      if (millis() - start_time >= a_timeout) {
        digitalWrite(a_led, LOW);
        a_serial.println("Wiimote connection timed out.");
        return false;
      }

      // LEDの点滅
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

      delay(1); // 1ms秒待機して再チェック
    }
    digitalWrite(a_led, HIGH);
    a_serial.println("Wiimote successfully connected. ");
    return true;
  }
  digitalWrite(a_led, LOW);
  return false;
}

//----------------------------------------------------------------------
// WIIMOTE用スレッド
//----------------------------------------------------------------------

void Core0_BT_r(void *args) { // サブCPU(Core0)で実行するプログラム
  while (true) {              // Bluetooth待受用の無限ループ
    pad_array.ui64val = mrd_bt_read_wiimote();
    vTaskDelay(PAD_INTERVAL); // 他のタスクにCPU時間を譲る
  }
}

//------------------------------------------------------------------------------------
//  meridimへのデータ書き込み
//------------------------------------------------------------------------------------

bool meriput90_pad(Meridim90Union &a_meridim, PadUnion a_pad_array, bool a_marge) {

  // ボタンデータの処理 (マージ or 上書き)
  if (a_marge) {
    a_meridim.usval[MRD_PAD_BUTTONS] |= a_pad_array.usval[0];
  } else {
    a_meridim.usval[MRD_PAD_BUTTONS] = a_pad_array.usval[0];
  }

  // アナログ入力データの処理 (上書きのみ)
  for (int i = 1; i < 4; i++) {
    a_meridim.usval[MRD_PAD_BUTTONS + i] = a_pad_array.usval[i];
  }
  return true;
}
