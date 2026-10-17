#ifndef __MERIDIAN_BT_PAD_H__
#define __MERIDIAN_BT_PAD_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"
#include "mrd_servo.h" // ICSサーボのインスタンス(KRC-5FH受信機の読み取りに使用)

// ライブラリ導入
#include <ESP32Wiimote.h> // Wiiコントローラー
ESP32Wiimote wiimote;

// WIIMOTE / BlueRetro のパッド値. Core0のスレッドが書き込み, メインループが読む.
// 64bitの値は1命令で読み書きできないため, 書き込み途中の値を読まないようスピンロックで排他する.
uint64_t pad_bt_val = 0;
portMUX_TYPE pad_bt_mux = portMUX_INITIALIZER_UNLOCKED;

/// @brief Core0 のスレッドが読んだパッドの値を, 排他区間で pad_bt_val に書き込む.
inline void mrd_pad_bt_store(uint64_t a_val) {
  portENTER_CRITICAL(&pad_bt_mux);
  pad_bt_val = a_val;
  portEXIT_CRITICAL(&pad_bt_mux);
}

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_TABLE_WIIMOTE_SOLO[16] = {
    0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_TABLE_WIIMOTE_ORIG[16] = {
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000,
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};
constexpr unsigned short PAD_TABLE_KRC5FH_TO_COMMON[16] = {0, 0, 0, 128, 1, 4, 2, 8, 1024, 4096, 512, 2048, 16, 64, 32, 256};

//==================================================================================================
//  タイプ別のJOYPAD読み込み処理
//==================================================================================================

//----------------------------------------------------------------------
// KRC-5FHの読み込み
//----------------------------------------------------------------------

/// @brief KRC-5FHジョイパッドからデータを読み取り, 指定された間隔でデータを更新する.
/// @param a_interval 読み取り間隔(ミリ秒).
/// @param a_ics KRR-5FHを接続したICSのインスタンス.
/// @return 更新されたジョイパッドの状態を64ビット整数で返す.
uint64_t mrd_pad_krc_read(unsigned long a_interval, IcsBaseClass &a_ics) {
  static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
  uint8_t pad_analog_tmp[4] = {0}; // アナログ入力のデータ組み立て用(PA1〜PA4. 1byteずつ)

  static unsigned long last_time_tmp = 0; // 最後に関数が呼ばれた時間を記録
  unsigned long current_time_tmp = millis();

  if (current_time_tmp - last_time_tmp >= a_interval) {
    unsigned short krr_button_tmp;     // krrからのボタン入力データ
    int krr_analog_tmp[4];             // krrからのアナログ入力データ
    unsigned short pad_common_tmp = 0; // PS準拠に変換後のボタンデータ
    bool rcvd_tmp;                     // 受信機がデータを受信成功したか
    rcvd_tmp = a_ics.getKrrAllData(&krr_button_tmp, krr_analog_tmp);
    delayMicroseconds(2);

    if (rcvd_tmp) // リモコンデータが受信できていたら
    {
      // ボタンデータの処理
      int button_tmp = krr_button_tmp; // 受信ボタンデータの読み込み用

      if (cfg.pad_generalize) {        // ボタンデータの一般化処理
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

        // アナログ入力データの処理
        if (krr_analog_tmp[0] + krr_analog_tmp[1] + krr_analog_tmp[2] + krr_analog_tmp[3]) { // すべて0なら0のまま
          for (int i = 0; i < 4; i++) {
            int analog_tmp = (krr_analog_tmp[i] - 62) * 4;                         // 中央からの差を計算して範囲を制限する
            pad_analog_tmp[i] = uint8_t(int8_t(constrain(analog_tmp, -127, 127))); // ±127に収める
          }
        }
      } else {
        pad_common_tmp = button_tmp; // ボタンの変換なし生値を使用
        for (int i = 0; i < 4; i++) {
          pad_analog_tmp[i] = uint8_t(constrain(krr_analog_tmp[i], 0, 255)); // スティックも変換なしの生値(0〜127, 中央約62)
        }
      }
    }

    // データの組み立て
    uint64_t updated_val_tmp = 0;
    updated_val_tmp = static_cast<uint64_t>(pad_common_tmp);
    // スティックは他のパッドと同じく上位byte = X, 下位byte = Y (PA1/PA3 = X, PA2/PA4 = Y)
    updated_val_tmp |= ((uint64_t)pad_analog_tmp[1] & 0xFF) << 16; // [16] 左スティック: 下位 = PA2(Y)
    updated_val_tmp |= ((uint64_t)pad_analog_tmp[0] & 0xFF) << 24; //                   上位 = PA1(X)
    updated_val_tmp |= ((uint64_t)pad_analog_tmp[3] & 0xFF) << 32; // [17] 右スティック: 下位 = PA4(Y)
    updated_val_tmp |= ((uint64_t)pad_analog_tmp[2] & 0xFF) << 40; //                   上位 = PA3(X)

    last_time_tmp = current_time_tmp; // 最後の実行時間を更新
    pre_val_tmp = updated_val_tmp;
    return updated_val_tmp;
  }
  return pre_val_tmp;
}

//----------------------------------------------------------------------
// SBDBTの読み込み(RCB3形式のシリアル)
//----------------------------------------------------------------------
// SBDBT(ランニングエレクトロニクス)の PS3 コントローラ用ファームウェアが出す RCB3 形式の電文を受ける.
//   電文: 8byte. [0] 0x80, [1][2] ボタン, [3]〜[6] アナログ(0〜127, 中央 0x40. 左X, 左Y, 右X, 右Y),
//         [7] チェックサム([1]〜[6] の和の下位7bit). 8N1, 既定 2400bps.
// 空いているハードウェアUARTがないため, PIN_PAD_RX の信号の変化を割り込みで記録し(時刻と状態),
// メインループでビットを読み取って文字と電文に組み立てる. 2400bps では1ビットが417usあり,
// サーボ通信で割り込みを止める区間(最大約80us)があっても読み取りに影響しない.

const int MRD_SBDBT_EDGES = 256; // 記録する信号の変化の数(リングバッファ)

// 割り込みで記録した信号の変化. 割り込みが書き, メインループが読む(書き込み位置と読み出し位置は別の変数)
volatile uint32_t mrd_sbdbt_edge_us[MRD_SBDBT_EDGES]; // 変化した時刻(us)
volatile uint8_t mrd_sbdbt_edge_lv[MRD_SBDBT_EDGES];  // 変化した後の状態(0:LOW, 1:HIGH)
volatile uint32_t mrd_sbdbt_edge_head = 0;            // 書き込み位置(割り込み)
uint32_t mrd_sbdbt_edge_tail = 0;                     // 読み出し位置(メインループ)
int mrd_sbdbt_pin = -1;                               // 受信ピン

// 受信の状態(ビットの読み取り, 電文の組み立て)
struct MrdSbdbtRx {
  uint32_t bit_us = 417;     // 1ビットの時間(us)
  bool in_byte = false;      // 文字の途中か
  uint32_t t0 = 0;           // スタートビットの始まりの時刻(us)
  int next_bit = 0;          // 次に読むビット(0:スタート, 1〜8:データ, 9:ストップ)
  uint8_t data = 0;          // 読み取り中の文字
  int level = 1;             // 今の信号の状態(アイドルはHIGH)
  uint8_t pkt[8] = {0};      // 組み立て中の電文
  int pkt_len = 0;           // 組み立て中の電文の長さ(0:ヘッダ待ち)
  uint64_t value = 0;        // 最後に受けた正しい電文のパッドの値
  unsigned long last_ms = 0; // 最後に正しい電文を受けた時刻(ms)
  uint32_t packets = 0;      // 正しく受けた電文の数
  uint32_t errors = 0;       // 文字の枠の誤り・チェックサムの誤りの数
};
MrdSbdbtRx mrd_sbdbt_rx;

/// @brief RCB3形式の電文をパッドの値(PadUnion の64bit. [0] ボタン, [1] 左スティック, [2] 右スティック)にする.
///        スティックは上位byte = X, 下位byte = Y で, それぞれ (値 − 64) × 2 を ±127 に収める.
/// @param a_pkt 電文(8byte. チェックサムは確認済み).
/// @param a_generalize ボタンをPS系の配置にするか(PAD_GENERALIZE). 0ならボタンは電文の [1][2] のまま.
/// @return パッドの値.
uint64_t mrd_sbdbt_packet_to_pad(const uint8_t *a_pkt, bool a_generalize) {
  uint16_t button_tmp = 0;
  if (a_generalize) {
    uint8_t b2_tmp = a_pkt[2];
    if ((b2_tmp & 0x03) == 0x03) { // ↑↓の同時押しは Start
      button_tmp |= PAD_START;
      b2_tmp &= ~0x03;
    }
    if ((b2_tmp & 0x0C) == 0x0C) { // →←の同時押しは Select
      button_tmp |= PAD_SELECT;
      b2_tmp &= ~0x0C;
    }
    const uint16_t b2_table_tmp[7] = {PAD_UP, PAD_DOWN, PAD_RIGHT, PAD_LEFT, PAD_bU, PAD_bD, PAD_bR}; // ↑↓→←△×○
    const uint16_t b1_table_tmp[5] = {PAD_bL, PAD_L1, PAD_L2, PAD_R1, PAD_R2};                        // □ L1 L2 R1 R2
    for (int k = 0; k < 7; k++) {
      if (b2_tmp & (1 << k)) {
        button_tmp |= b2_table_tmp[k];
      }
    }
    for (int k = 0; k < 5; k++) {
      if (a_pkt[1] & (1 << k)) {
        button_tmp |= b1_table_tmp[k];
      }
    }
  } else {
    button_tmp = uint16_t((a_pkt[1] << 8) | a_pkt[2]);
  }
  uint8_t stick_tmp[4]; // 左X, 左Y, 右X, 右Y
  for (int k = 0; k < 4; k++) {
    stick_tmp[k] = uint8_t(int8_t(constrain((int(a_pkt[3 + k]) - 64) * 2, -127, 127)));
  }
  uint64_t val_tmp = button_tmp;
  val_tmp |= uint64_t((stick_tmp[0] << 8) | stick_tmp[1]) << 16; // [16] 左スティック
  val_tmp |= uint64_t((stick_tmp[2] << 8) | stick_tmp[3]) << 32; // [17] 右スティック
  return val_tmp;
}

/// @brief 受け取った1文字を電文に組み立てる. 正しい電文がそろえばパッドの値を更新する.
/// @param a_rx 受信の状態.
/// @param a_byte 文字.
/// @param a_now_ms 今の時刻(ms).
void mrd_sbdbt_rx_byte(MrdSbdbtRx &a_rx, uint8_t a_byte, unsigned long a_now_ms) {
  if (a_byte == 0x80) { // ヘッダ(データの文字は7bitなので, 0x80 は先頭にしか現れない)
    a_rx.pkt[0] = a_byte;
    a_rx.pkt_len = 1;
    return;
  }
  if (a_rx.pkt_len == 0 || a_rx.pkt_len >= 8) {
    return; // ヘッダ待ち
  }
  a_rx.pkt[a_rx.pkt_len++] = a_byte;
  if (a_rx.pkt_len < 8) {
    return;
  }
  a_rx.pkt_len = 0;
  uint8_t sum_tmp = 0;
  for (int k = 1; k <= 6; k++) {
    sum_tmp += a_rx.pkt[k];
  }
  if ((sum_tmp & 0x7F) != a_rx.pkt[7]) {
    a_rx.errors++;
    return;
  }
  a_rx.value = mrd_sbdbt_packet_to_pad(a_rx.pkt, cfg.pad_generalize);
  a_rx.last_ms = a_now_ms;
  a_rx.packets++;
}

/// @brief 時刻 a_t_us までに中央が来るビットを, 今の信号の状態として読み取る.
void mrd_sbdbt_rx_sample(MrdSbdbtRx &a_rx, uint32_t a_t_us, unsigned long a_now_ms) {
  while (a_rx.in_byte && int32_t(a_t_us - (a_rx.t0 + a_rx.bit_us * a_rx.next_bit + a_rx.bit_us / 2)) >= 0) {
    if (a_rx.next_bit == 0) { // スタートビットはLOWのはず(短いノイズならやめる)
      if (a_rx.level != 0) {
        a_rx.in_byte = false;
        break;
      }
    } else if (a_rx.next_bit <= 8) { // データ(下位bitから)
      a_rx.data |= uint8_t(a_rx.level << (a_rx.next_bit - 1));
    } else { // ストップビットはHIGHのはず
      if (a_rx.level == 1) {
        mrd_sbdbt_rx_byte(a_rx, a_rx.data, a_now_ms);
      } else {
        a_rx.errors++;
      }
      a_rx.in_byte = false;
      break;
    }
    a_rx.next_bit++;
  }
}

/// @brief 信号の変化を1つ処理する. 変化の前の状態でビットを読み, LOWへの変化が文字の始まりになる.
/// @param a_rx 受信の状態.
/// @param a_t_us 変化した時刻(us).
/// @param a_level 変化した後の状態.
/// @param a_now_ms 今の時刻(ms).
void mrd_sbdbt_rx_edge(MrdSbdbtRx &a_rx, uint32_t a_t_us, int a_level, unsigned long a_now_ms) {
  mrd_sbdbt_rx_sample(a_rx, a_t_us, a_now_ms);
  a_rx.level = a_level;
  if (!a_rx.in_byte && a_level == 0) {
    a_rx.in_byte = true;
    a_rx.t0 = a_t_us;
    a_rx.next_bit = 0;
    a_rx.data = 0;
  }
}

/// @brief 信号の変化の割り込み. 時刻と変化した後の状態を記録する.
void IRAM_ATTR mrd_sbdbt_isr() {
  const uint32_t h_tmp = mrd_sbdbt_edge_head;
  mrd_sbdbt_edge_us[h_tmp % MRD_SBDBT_EDGES] = micros();
  mrd_sbdbt_edge_lv[h_tmp % MRD_SBDBT_EDGES] = gpio_ll_get_level(&GPIO, (gpio_num_t)mrd_sbdbt_pin);
  mrd_sbdbt_edge_head = h_tmp + 1;
}

/// @brief SBDBTの受信を開始する(ピンを入力にし, 信号の変化の割り込みを登録する).
/// @param a_pin 受信ピン(PIN_PAD_RX).
/// @param a_bps 通信速度(PAD_SBDBT_BPS).
void mrd_sbdbt_begin(int a_pin, unsigned long a_bps) {
  mrd_sbdbt_pin = a_pin;
  mrd_sbdbt_rx = MrdSbdbtRx();
  mrd_sbdbt_rx.bit_us = 1000000UL / a_bps;
  pinMode(a_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(a_pin), mrd_sbdbt_isr, CHANGE);
}

/// @brief 記録した信号の変化から電文を組み立て, SBDBTのパッドの値を返す.
///        PAD_SBDBT_TIMEOUT が0でなく, その時間正しい電文が届かなければ0を返す.
/// @return パッドの値(PadUnion の64bit).
uint64_t mrd_pad_sbdbt_read() {
  MrdSbdbtRx &rx_tmp = mrd_sbdbt_rx;
  const unsigned long now_ms_tmp = millis();
  const uint32_t head_tmp = mrd_sbdbt_edge_head;
  if (head_tmp - mrd_sbdbt_edge_tail > uint32_t(MRD_SBDBT_EDGES)) { // 読み出しが追いつかず上書きされた
    mrd_sbdbt_edge_tail = head_tmp;
    rx_tmp.in_byte = false;
    rx_tmp.pkt_len = 0;
    rx_tmp.errors++;
  }
  while (mrd_sbdbt_edge_tail != head_tmp) {
    const uint32_t ix_tmp = mrd_sbdbt_edge_tail % MRD_SBDBT_EDGES;
    mrd_sbdbt_rx_edge(rx_tmp, mrd_sbdbt_edge_us[ix_tmp], mrd_sbdbt_edge_lv[ix_tmp], now_ms_tmp);
    mrd_sbdbt_edge_tail++;
  }
  mrd_sbdbt_rx_sample(rx_tmp, micros(), now_ms_tmp); // 変化のない末尾のビット(HIGH)を読み終える
  if (cfg.pad_sbdbt_timeout > 0 && now_ms_tmp - rx_tmp.last_ms >= (unsigned long)cfg.pad_sbdbt_timeout) {
    return 0;
  }
  return rx_tmp.value;
}

//----------------------------------------------------------------------
// BlueRetroの読み込み(Wii extension. I2C0 または I2C1)
//----------------------------------------------------------------------
// BlueRetro(別のESP32に書き込むファームウェア)の Wii extension 出力を I2C0(Wire) か I2C1(Wire1)で読む(PAD_BLUERETRO_I2C).
//   BlueRetro は I2C のスレーブ(アドレス 0x52)で, Wii のクラシックコントローラ Pro として応える.
//   初期化: 0xF0 に 0x55, 0xFB に 0x00 を書くと暗号化なし. 0xFE に 0x03 を書くと 8byte の形式になる.
//   読み取り: レジスタ番号 0x00 を書いてから 8byte を読む.
//     [0] 左X [1] 右X [2] 左Y [3] 右Y (中央 0x80, ±0x66 程度. Y は上が大きい) [4] Lトリガー [5] Rトリガー (0x16〜0xDA 程度)
//     [6][7] ボタン(押すと0). [6] bit1 R, 2 +, 3 HOME, 4 −, 5 L, 6 ↓, 7 → / [7] bit0 ↑, 1 ←, 2 ZR, 3 X, 4 A, 5 Y, 6 B, 7 ZL
//     [6] の bit0 は常に1. 0 なら形式が 8byte ではない(BlueRetro の再起動など)ので初期化をやり直す.

const uint8_t MRD_WIIEXT_ADDR = 0x52;         // Wii extension の I2C アドレス
const uint32_t MRD_WIIEXT_READ_WAIT_US = 200; // レジスタ番号を書いてから読むまでの待ち(us)

// BlueRetroの受信の状態
struct MrdBlueRetro {
  bool ready = false;  // 初期化が済み, 8byte の形式で読めているか
  uint32_t inits = 0;  // 初期化した回数
  uint32_t errors = 0; // 読み取りに失敗した回数
};
MrdBlueRetro mrd_blueretro;

/// @brief Wii extension の1つのレジスタに書く.
/// @return 書けたらtrue.
bool mrd_wiiext_write(TwoWire &a_wire, uint8_t a_reg, uint8_t a_val) {
  a_wire.beginTransmission(MRD_WIIEXT_ADDR);
  a_wire.write(a_reg);
  a_wire.write(a_val);
  return a_wire.endTransmission() == 0;
}

/// @brief Wii extension のレジスタ a_reg から a_len byte 読む(レジスタ番号を書き, 待ってから読む).
/// @return 読めたらtrue.
bool mrd_wiiext_read(TwoWire &a_wire, uint8_t a_reg, uint8_t *a_buf, int a_len) {
  a_wire.beginTransmission(MRD_WIIEXT_ADDR);
  a_wire.write(a_reg);
  if (a_wire.endTransmission() != 0) {
    return false;
  }
  delayMicroseconds(MRD_WIIEXT_READ_WAIT_US);
  if (a_wire.requestFrom(MRD_WIIEXT_ADDR, uint8_t(a_len)) != a_len) {
    return false;
  }
  for (int k = 0; k < a_len; k++) {
    a_buf[k] = a_wire.read();
  }
  return true;
}

/// @brief Wii extension を暗号化なし・8byte の形式に初期化し, クラシックコントローラ(8byte の形式)と確かめる.
/// @return 使えるならtrue.
bool mrd_wiiext_init(TwoWire &a_wire) {
  if (!mrd_wiiext_write(a_wire, 0xF0, 0x55) || !mrd_wiiext_write(a_wire, 0xFB, 0x00) || !mrd_wiiext_write(a_wire, 0xFE, 0x03)) {
    return false;
  }
  uint8_t id_tmp[6];
  if (!mrd_wiiext_read(a_wire, 0xFA, id_tmp, 6)) {
    return false;
  }
  return id_tmp[2] == 0xA4 && id_tmp[3] == 0x20 && id_tmp[4] == 0x03 && id_tmp[5] == 0x01; // クラシックコントローラ, 8byte の形式
}

/// @brief Wii のクラシックコントローラの標準の形式(6byte. データ形式1)を 8byte の形式(各軸8bit)に直す.
///        6byte: [0] 右X上位2bit・左X 6bit, [1] 右X中位2bit・左Y 6bit, [2] 右X下位1bit・Lトリガー上位2bit・右Y 5bit,
///        [3] Lトリガー下位3bit・Rトリガー 5bit, [4][5] ボタン(8byte の [6][7] と同じ).
/// @param a_d6 6byte.
/// @param a_d8 8byte(左X, 右X, 左Y, 右Y, Lトリガー, Rトリガー, ボタン2byte).
void mrd_wiiext_6to8(const uint8_t *a_d6, uint8_t *a_d8) {
  const uint8_t lx_tmp = a_d6[0] & 0x3F; // 6bit
  const uint8_t ly_tmp = a_d6[1] & 0x3F;
  const uint8_t rx_tmp = ((a_d6[0] >> 3) & 0x18) | ((a_d6[1] >> 5) & 0x06) | (a_d6[2] >> 7); // 5bit
  const uint8_t ry_tmp = a_d6[2] & 0x1F;
  const uint8_t lt_tmp = ((a_d6[2] >> 2) & 0x18) | (a_d6[3] >> 5); // 5bit
  const uint8_t rt_tmp = a_d6[3] & 0x1F;
  a_d8[0] = uint8_t(lx_tmp << 2); // 6bit の中央 32 → 128
  a_d8[1] = uint8_t(rx_tmp << 3); // 5bit の中央 16 → 128
  a_d8[2] = uint8_t(ly_tmp << 2);
  a_d8[3] = uint8_t(ry_tmp << 3);
  a_d8[4] = uint8_t(lt_tmp << 3);
  a_d8[5] = uint8_t(rt_tmp << 3);
  a_d8[6] = a_d6[4];
  a_d8[7] = a_d6[5];
}

/// @brief Wii extension の 8byte をパッドの値(PadUnion の64bit. [0] ボタン, [1] 左, [2] 右, [3] L2/R2 のアナログ)にする.
///        スティックは上位byte = X, 下位byte = Y で, (値 − 0x80) × 127 / 0x66 を ±127 に収める(Y は上が正).
///        アナログトリガーは上位byte = L, 下位byte = R で, (値 − 0x16) を 0〜127 に収める.
/// @param a_d 読んだ8byte.
/// @param a_generalize ボタンをPS系の配置にするか(PAD_GENERALIZE). 0なら Wii のボタンの2byteを反転した値(押すと1).
/// @return パッドの値.
uint64_t mrd_wiiext_to_pad(const uint8_t *a_d, bool a_generalize) {
  const uint16_t raw_tmp = uint16_t(~(a_d[6] | (a_d[7] << 8))); // 押すと1
  uint16_t button_tmp = raw_tmp;
  if (a_generalize) {
    // Wii の bit: 1 R, 2 +, 3 HOME, 4 −, 5 L, 6 ↓, 7 →, 8 ↑, 9 ←, 10 ZR, 11 X, 12 A, 13 Y, 14 B, 15 ZL
    const uint16_t table_tmp[16] = {0, PAD_R2, PAD_START, PAD_HOME, PAD_SELECT, PAD_L2, PAD_DOWN, PAD_RIGHT,
                                    PAD_UP, PAD_LEFT, PAD_R1, PAD_bU, PAD_bR, PAD_bL, PAD_bD, PAD_L1};
    button_tmp = 0;
    for (int k = 1; k < 16; k++) {
      if (raw_tmp & (1 << k)) {
        button_tmp |= table_tmp[k];
      }
    }
  }
  uint8_t stick_tmp[4]; // 左X, 左Y, 右X, 右Y
  const int axis_ix_tmp[4] = {0, 2, 1, 3};
  for (int k = 0; k < 4; k++) {
    stick_tmp[k] = uint8_t(int8_t(constrain((int(a_d[axis_ix_tmp[k]]) - 0x80) * 127 / 0x66, -127, 127)));
  }
  const uint8_t trig_l_tmp = uint8_t(constrain(int(a_d[4]) - 0x16, 0, 127));
  const uint8_t trig_r_tmp = uint8_t(constrain(int(a_d[5]) - 0x16, 0, 127));
  uint64_t val_tmp = button_tmp;
  val_tmp |= uint64_t((stick_tmp[0] << 8) | stick_tmp[1]) << 16; // [16] 左スティック
  val_tmp |= uint64_t((stick_tmp[2] << 8) | stick_tmp[3]) << 32; // [17] 右スティック
  val_tmp |= uint64_t((trig_l_tmp << 8) | trig_r_tmp) << 48;     // [18] L2/R2 のアナログ
  return val_tmp;
}

/// @brief BlueRetroを1回読み, パッドの値を返す. 初期化が済んでいなければ先に初期化する.
///        読めないとき(BlueRetro の電源が入っていない等)と, 形式が 8byte でないときは初期化をやり直し, 0 を返す.
/// @param a_wire BlueRetro をつないだ I2C(Wire または Wire1).
/// @return パッドの値(PadUnion の64bit).
uint64_t mrd_blueretro_read(TwoWire &a_wire) {
  MrdBlueRetro &br_tmp = mrd_blueretro;
  if (!br_tmp.ready) {
    br_tmp.inits++;
    br_tmp.ready = mrd_wiiext_init(a_wire);
    if (!br_tmp.ready) {
      return 0;
    }
  }
  uint8_t d_tmp[8];
  if (!mrd_wiiext_read(a_wire, 0x00, d_tmp, 8) || (d_tmp[6] & 0x01) == 0) {
    br_tmp.errors++;
    br_tmp.ready = false;
    return 0;
  }
  return mrd_wiiext_to_pad(d_tmp, cfg.pad_generalize);
}

//----------------------------------------------------------------------
// WIIMOTEの読み込み
//----------------------------------------------------------------------

/// @brief Wiiリモコンからの入力データを受信し, 処理する.
/// @return 更新されたジョイパッドの状態を64ビット整数で返す. 切断されている間は0.
/// @note ESP32Wiimoteインスタンス wiimote, cfg.pad_generalize, cfg.mount_pad を関数内で使用.
uint64_t mrd_bt_wiimote_read() {
  static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
  static int calib_l1x_tmp = 0;
  static int calib_l1y_tmp = 0;

  // 受信データの問い合わせ
  wiimote.task();
  ButtonState rcvd_button_tmp;
  NunchukState nunchuk_tmp;

  if (!wiimote.connected()) { // 切断されたら, 押したままの値が残らないよう0にする
    pre_val_tmp = 0;
    return 0;
  }

  if (wiimote.available() > 0) {
    // クラシックコントローラ(MOUNT_PAD WIIMOTE_C): 拡張の6byteを BlueRetro と同じ変換でパッドの値にする
    if (cfg.mount_pad == WIIMOTE_C && wiimote.classicConnected()) {
      uint8_t d6_tmp[6];
      uint8_t d8_tmp[8];
      wiimote.getExtensionData(d6_tmp);
      mrd_wiiext_6to8(d6_tmp, d8_tmp);
      pre_val_tmp = mrd_wiiext_to_pad(d8_tmp, cfg.pad_generalize);
      return pre_val_tmp;
    }

    // リモコンデータの取得
    rcvd_button_tmp = wiimote.getButtonState();
    nunchuk_tmp = wiimote.getNunchukState();

    uint16_t new_pad_tmp[4] = {0}; // アナログ入力のデータ組み立て用

    // ボタン値の変換(一般化)
    for (int i = 0; i < 16; i++) {
      uint16_t mask_tmp = 1 << i;
      if ((cfg.pad_generalize && (PAD_TABLE_WIIMOTE_SOLO[i] & rcvd_button_tmp)) ||
          (!cfg.pad_generalize && (PAD_TABLE_WIIMOTE_ORIG[i] & rcvd_button_tmp))) {
        new_pad_tmp[0] |= mask_tmp;
      }
    }

    if (rcvd_button_tmp & BUTTON_C) { // ヌンチャクCボタンの処理
      if (cfg.pad_generalize) {
        new_pad_tmp[0] |= PAD_L1;
      } else {
        new_pad_tmp[0] |= PAD_bR;
      }
    }

    if (rcvd_button_tmp & BUTTON_Z) { // ヌンチャクZボタンの処理
      if (cfg.pad_generalize) {
        new_pad_tmp[0] |= PAD_R1;
      } else {
        new_pad_tmp[0] |= PAD_bD;
      }
    }

    if (rcvd_button_tmp & BUTTON_HOME) { // ホームボタンでスティックのキャリブレーション
      calib_l1x_tmp = nunchuk_tmp.xStick - 127;
      calib_l1y_tmp = nunchuk_tmp.yStick - 127;
    }

    // ヌンチャクの値を組み入れ(他のパッドと同じく, 上位byte = X, 下位byte = Y. 各±127)
    const int8_t stick_x_tmp = constrain(nunchuk_tmp.xStick - calib_l1x_tmp - 127, -127, 127);
    const int8_t stick_y_tmp = constrain(nunchuk_tmp.yStick - calib_l1y_tmp - 127, -127, 127);
    new_pad_tmp[1] = uint16_t((uint8_t(stick_x_tmp) << 8) | uint8_t(stick_y_tmp));

    // データの組み立て
    uint64_t new_val_tmp = 0; // 戻り値格納用
    new_val_tmp = static_cast<uint64_t>(new_pad_tmp[0]);
    new_val_tmp |= ((uint64_t)new_pad_tmp[1] << 16);

    pre_val_tmp = new_val_tmp;
    return new_val_tmp;
  }
  return pre_val_tmp;
}

//==================================================================================================
//  各種パッドへの分岐
//==================================================================================================

/// @brief 指定されたジョイパッドタイプに応じて最新データを読み取り, 64ビット整数で返す.
/// @param a_pad_type ジョイパッドのタイプを示す列挙型(KRR5FH, SBDBT, WIIMOTE, WIIMOTE_C, BLUERETRO. それ以外は0).
/// @return 64ビット整数に変換された受信データ
/// @note WIIMOTEの場合は, スレッド(mrd_bt_wiimote_task)が更新した pad_bt_val を返す.
uint64_t mrd_pad_read(PadType a_pad_type) {

  if (a_pad_type == KRR5FH) { // KRR5FH (R系統のICSに接続する受信機. R系統がICS以外なら読まない)
    return (cfg.mount_servo_type_r == KOICS3) ? mrd_pad_krc_read(cfg.pad_interval, ics_r) : 0;
  }

  if (a_pad_type == SBDBT) { // SBDBT (PIN_PAD_RX でソフトウェア受信)
    return mrd_pad_sbdbt_read();
  }

  if (a_pad_type == WIIMOTE || a_pad_type == WIIMOTE_C || a_pad_type == BLUERETRO) { // Wiimote, BlueRetro (Core0 のスレッドが読んだ値)
    portENTER_CRITICAL(&pad_bt_mux);
    uint64_t val_tmp = pad_bt_val;
    portEXIT_CRITICAL(&pad_bt_mux);
    return val_tmp;
  }
  return 0;
}

//==================================================================================================
//  初期化と準備
//==================================================================================================

//----------------------------------------------------------------------
// Bluetooth, WIIMOTEの初期化
//----------------------------------------------------------------------

/// @brief Bluetoothの設定を行い, Wiiコントローラの接続を開始する.
bool mrd_bt_setup(int a_mount_pad,
                  int a_timeout,
                  ESP32Wiimote &a_wiimote,
                  int a_led,
                  HardwareSerial &a_serial) {
  // Wiiコントローラの接続開始
  if (a_mount_pad == WIIMOTE || a_mount_pad == WIIMOTE_C) {
    a_serial.println("Try to connect Wiimote...");
    a_wiimote.init();
    a_wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL);

    uint16_t count_tmp = 0;
    unsigned long start_time_tmp = millis();
    while (!a_wiimote.available()) {

      // リモコンへの問い合わせ
      a_wiimote.task();

      // タイムアウトチェック
      if (millis() - start_time_tmp >= a_timeout) {
        mrd_led_write(a_led, LOW);
        a_serial.println("Wiimote connection timed out.");
        return false;
      }

      // LEDの点滅
      count_tmp++;
      if (count_tmp < 500) {
        mrd_led_write(a_led, HIGH);
      } else {
        mrd_led_write(a_led, LOW);
      }
      if (count_tmp > 1000) {
        a_serial.print(".");
        count_tmp = 0;
      }

      delay(1); // 1ms秒待機して再チェック
    }
    mrd_led_write(a_led, HIGH);
    a_serial.println("Wiimote successfully connected. ");
    return true;
  }
  mrd_led_write(a_led, LOW);
  return false;
}

//----------------------------------------------------------------------
// WIIMOTE用スレッド
//----------------------------------------------------------------------

/// @brief サブCPU (Core0) で実行されるBluetooth通信用のルーチン.
/// @param a_args この関数に渡される引数. 現在は不使用.
/// @note 読み取った値は pad_bt_val に排他区間で書き込む.
void mrd_bt_wiimote_task(void *a_args) { // サブCPU(Core0)で実行するプログラム
  while (true) {                         // Bluetooth待受用の無限ループ
    mrd_pad_bt_store(mrd_bt_wiimote_read());
    vTaskDelay(pdMS_TO_TICKS(cfg.pad_interval)); // 他のタスクにCPU時間を譲る
  }
}

//----------------------------------------------------------------------
// BlueRetro用スレッド
//----------------------------------------------------------------------

/// @brief サブCPU (Core0) で, PAD_INTERVAL ごとに BlueRetro を読む.
///        I2C0 は IMU のスレッド, I2C1 はメインループの PCA9685 も使うが, Wire が排他するため同時には動かない.
/// @param a_args BlueRetro をつないだ I2C (Wire または Wire1. TwoWire へのポインタ).
/// @note 読み取った値は pad_bt_val に排他区間で書き込む.
void mrd_blueretro_task(void *a_args) {
  while (true) {
    mrd_pad_bt_store(mrd_blueretro_read(*static_cast<TwoWire *>(a_args)));
    vTaskDelay(pdMS_TO_TICKS(cfg.pad_interval));
  }
}

//------------------------------------------------------------------------------------
//  meridimへのデータ書き込み
//------------------------------------------------------------------------------------
/// @brief meridim配列にPADデータを書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @param a_pad_array PAD受信値の格納用配列.
/// @param a_marge PADボタンデータをマージするかどうかのブール値.
/// trueの場合は既存のデータにビット単位でOR演算を行い, falseの場合は新しいデータで上書きする.
bool mrd_meriput90_pad(Meridim90Union &a_meridim, PadUnion a_pad_array, bool a_marge) {

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

#endif // __MERIDIAN_BT_PAD_H__
