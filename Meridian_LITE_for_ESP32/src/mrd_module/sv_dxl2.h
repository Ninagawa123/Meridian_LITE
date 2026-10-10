#ifndef __MERIDIAN_SERVO_DYNAMIXEL_H__
#define __MERIDIAN_SERVO_DYNAMIXEL_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバス(MrdHalfDuplexUart). 本ファイルは mrd_servo.h から読み込まれる

//==================================================================================================
//  DYNAMIXEL Protocol 2.0 (Xシリーズ: XL330, XC330, XL430, XC430, 2XL430, XM, XH, XW など)
//==================================================================================================
//
// MOUNT_SERVO_TYPE_L / R を 32 にすると, その系統をこのファイルの処理で駆動する.
// 1系統あたり最大15軸. 1フレーム(10ms)の中で以下を行う.
//   1. 目標位置を Sync Write でまとめて書き込む (トルクONの軸のみ, 応答なし)
//   2. トルクのON/OFFが変わった軸だけ, Sync Write でまとめて切り替える
//   3. 現在位置を1軸ずつ Read で読み出す (応答待ちは SERVO_DXL_TIMEOUT_US)
// L/R系統がどちらもDYNAMIXELの場合は, 分割API(begin_send/end_send/receive)でL/Rの通信を重ねる.
// 実測: XL430 2軸・3Mbpsでサーボ処理 約0.6〜0.8ms (1軸あたり約0.2ms).
//
// サーボ側の事前設定 (DYNAMIXEL Wizard など):
//   - IDを IXL_ID / IXR_ID と一致させる
//   - 通信速度を SERVO_BAUDRATE_L / R と一致させる (3Mbps推奨)
//   - Protocol 2.0, 位置制御モード (Operating Mode = 3)
// Return Delay Time は, 起動後の最初のフレームで0以外なら自動で0に書き換える (値が違うときだけEEPROMに書き込む).

// ---- dxl2 packet begin (Arduino非依存. PC上の単体テストでも使う) ----
namespace dxl2 {

// インストラクション
constexpr uint8_t INST_PING = 0x01;
constexpr uint8_t INST_READ = 0x02;
constexpr uint8_t INST_WRITE = 0x03;
constexpr uint8_t INST_STATUS = 0x55;
constexpr uint8_t INST_SYNC_WRITE = 0x83;
constexpr uint8_t ID_BROADCAST = 0xFE;

// Xシリーズ共通のコントロールテーブル
constexpr uint16_t ADDR_RETURN_DELAY = 9;       // EEPROM, 1byte (単位2us)
constexpr uint16_t ADDR_TORQUE_ENABLE = 64;     // RAM, 1byte
constexpr uint16_t ADDR_HW_ERROR = 70;          // RAM, 1byte (Hardware Error Status)
constexpr uint16_t ADDR_GOAL_POSITION = 116;    // RAM, 4byte
constexpr uint16_t ADDR_PRESENT_POSITION = 132; // RAM, 4byte

// 位置の換算 (1回転4096, 中心2048)
constexpr int POS_CENTER = 2048;
constexpr int POS_MIN = 0;
constexpr int POS_MAX = 4095;
constexpr float POS_PER_DEG = 4096.0f / 360.0f;

constexpr size_t PACKET_MAX = 128; // パケットの最大長 (バス層のエコー照合バッファと同じ)

/// @brief Protocol 2.0 の CRC-16 (多項式 0x8005, 初期値0). 表は初回呼び出し時に作る.
/// @param a_data 対象データ.
/// @param a_len 対象データ数.
/// @return CRC値.
inline uint16_t crc16(const uint8_t *a_data, size_t a_len) {
  static uint16_t table_tmp[256];
  static bool ready_tmp = false;
  if (!ready_tmp) {
    for (int i = 0; i < 256; i++) {
      uint16_t c_tmp = (uint16_t)(i << 8);
      for (int b = 0; b < 8; b++) {
        c_tmp = (c_tmp & 0x8000) ? (uint16_t)((c_tmp << 1) ^ 0x8005) : (uint16_t)(c_tmp << 1);
      }
      table_tmp[i] = c_tmp;
    }
    ready_tmp = true;
  }
  uint16_t crc_tmp = 0;
  for (size_t i = 0; i < a_len; i++) {
    crc_tmp = (uint16_t)((crc_tmp << 8) ^ table_tmp[((crc_tmp >> 8) ^ a_data[i]) & 0xFF]);
  }
  return crc_tmp;
}

/// @brief インストラクションパケットを作る. パラメータ中の FF FF FD にはバイトスタッフィング(FD追加)を行う.
/// @param a_out 出力先.
/// @param a_cap 出力先のサイズ.
/// @param a_id サーボID.
/// @param a_inst インストラクション.
/// @param a_params パラメータ.
/// @param a_nparams パラメータ数.
/// @return パケット長. バッファ不足なら0.
inline size_t make_packet(uint8_t *a_out, size_t a_cap, uint8_t a_id, uint8_t a_inst, const uint8_t *a_params,
                          size_t a_nparams) {
  if (a_cap < 10) {
    return 0;
  }
  a_out[0] = 0xFF; // ヘッダ
  a_out[1] = 0xFF;
  a_out[2] = 0xFD;
  a_out[3] = 0x00; // 予約
  a_out[4] = a_id;
  a_out[7] = a_inst;
  size_t pos_tmp = 8;
  for (size_t i = 0; i < a_nparams; i++) {
    if (pos_tmp + 3 > a_cap) {
      return 0;
    }
    a_out[pos_tmp++] = a_params[i];
    if (pos_tmp >= 11 && a_out[pos_tmp - 3] == 0xFF && a_out[pos_tmp - 2] == 0xFF && a_out[pos_tmp - 1] == 0xFD) {
      a_out[pos_tmp++] = 0xFD; // バイトスタッフィング
    }
  }
  if (pos_tmp + 2 > a_cap) {
    return 0;
  }
  const uint16_t len_tmp = (uint16_t)(pos_tmp - 7 + 2); // 長さ = インストラクション + パラメータ + CRC
  a_out[5] = (uint8_t)(len_tmp & 0xFF);
  a_out[6] = (uint8_t)(len_tmp >> 8);
  const uint16_t crc_tmp = crc16(a_out, pos_tmp);
  a_out[pos_tmp++] = (uint8_t)(crc_tmp & 0xFF);
  a_out[pos_tmp++] = (uint8_t)(crc_tmp >> 8);
  return pos_tmp;
}

/// @brief パラメータ長 a_nparams のステータスパケット(全長 11 + a_nparams)を検査する.
///        データ部にスタッフィングが入った場合は長さが合わないため失敗とする(位置制御モードでは起きない).
/// @param a_rx 受信したステータスパケット.
/// @param a_id 期待するサーボID.
/// @param a_nparams パラメータ数.
/// @param a_err エラーバイトの格納先 (bit7: ハードウェアエラーあり).
/// @return 正常ならtrue (bit7以外のエラーがあれば失敗).
inline bool check_status(const uint8_t *a_rx, uint8_t a_id, size_t a_nparams, uint8_t &a_err) {
  const size_t total_tmp = 11 + a_nparams;
  if (a_rx[0] != 0xFF || a_rx[1] != 0xFF || a_rx[2] != 0xFD || a_rx[3] != 0x00) {
    return false;
  }
  if (a_rx[4] != a_id || a_rx[7] != INST_STATUS) {
    return false;
  }
  const uint16_t len_tmp = (uint16_t)(a_rx[5] | (a_rx[6] << 8));
  if (len_tmp != 4 + a_nparams) {
    return false;
  }
  const uint16_t crc_tmp = (uint16_t)(a_rx[total_tmp - 2] | (a_rx[total_tmp - 1] << 8));
  if (crc_tmp != crc16(a_rx, total_tmp - 2)) {
    return false;
  }
  a_err = a_rx[8];
  return (a_err & 0x7F) == 0;
}

/// @brief Sync Write のパケットを作る.
/// @param a_out 出力先.
/// @param a_cap 出力先のサイズ.
/// @param a_addr 書き込み先アドレス.
/// @param a_len 1軸あたりのデータ長.
/// @param a_ids 対象ID (a_count個).
/// @param a_data 各IDのデータ (a_count * a_len バイト, ID順).
/// @param a_count 対象の軸数.
/// @return パケット長. バッファ不足なら0.
inline size_t make_sync_write(uint8_t *a_out, size_t a_cap, uint16_t a_addr, uint16_t a_len, const uint8_t *a_ids,
                              const uint8_t *a_data, size_t a_count) {
  uint8_t params_tmp[PACKET_MAX];
  size_t n_tmp = 0;
  if (4 + a_count * (1 + a_len) > sizeof(params_tmp)) {
    return 0;
  }
  params_tmp[n_tmp++] = (uint8_t)(a_addr & 0xFF);
  params_tmp[n_tmp++] = (uint8_t)(a_addr >> 8);
  params_tmp[n_tmp++] = (uint8_t)(a_len & 0xFF);
  params_tmp[n_tmp++] = (uint8_t)(a_len >> 8);
  for (size_t i = 0; i < a_count; i++) {
    params_tmp[n_tmp++] = a_ids[i];
    for (uint16_t b = 0; b < a_len; b++) {
      params_tmp[n_tmp++] = a_data[i * a_len + b];
    }
  }
  return make_packet(a_out, a_cap, ID_BROADCAST, INST_SYNC_WRITE, params_tmp, n_tmp);
}

/// @brief Read のパケットを作る.
/// @param a_out 出力先.
/// @param a_cap 出力先のサイズ.
/// @param a_id サーボID.
/// @param a_addr 読み出し元アドレス.
/// @param a_len 読み出すデータ長.
/// @return パケット長. バッファ不足なら0.
inline size_t make_read(uint8_t *a_out, size_t a_cap, uint8_t a_id, uint16_t a_addr, uint16_t a_len) {
  const uint8_t params_tmp[4] = {(uint8_t)(a_addr & 0xFF), (uint8_t)(a_addr >> 8), (uint8_t)(a_len & 0xFF),
                                 (uint8_t)(a_len >> 8)};
  return make_packet(a_out, a_cap, a_id, INST_READ, params_tmp, 4);
}

/// @brief 角度(degree)をサーボの位置の値に換算する (KRSの Deg2Krs と同じ考え方).
/// @param a_deg 角度(degree).
/// @param a_trim トリム値(degree).
/// @param a_cw 回転方向(1:そのまま, -1:逆転).
/// @return 位置の値 (POS_MIN〜POS_MAX に制限).
inline int deg_to_pos(float a_deg, float a_trim, int a_cw) {
  const float x_tmp = POS_CENTER + (a_trim + a_deg * a_cw) * POS_PER_DEG;
  const int v_tmp = (int)(x_tmp >= 0 ? x_tmp + 0.5f : x_tmp - 0.5f);
  return v_tmp < POS_MIN ? POS_MIN : (v_tmp > POS_MAX ? POS_MAX : v_tmp);
}

/// @brief サーボの位置の値を角度(degree)に換算する (KRSの Krs2Deg と同じ考え方).
/// @param a_pos 位置の値.
/// @param a_trim トリム値(degree).
/// @param a_cw 回転方向(1:そのまま, -1:逆転).
/// @return 角度(degree).
inline float pos_to_deg(int32_t a_pos, float a_trim, int a_cw) {
  return ((a_pos - POS_CENTER) / POS_PER_DEG - a_trim) * a_cw;
}

} // namespace dxl2
// ---- dxl2 packet end ----

//==================================================================================================
//  DYNAMIXEL Protocol 2.0 の通信クラス (半二重UARTバス上で動作)
//==================================================================================================

/// @brief MrdHalfDuplexUart 上で DYNAMIXEL Protocol 2.0 のパケットを送受信するクラス.
class Dxl2HalfDuplexClass {
public:
  /// @brief コンストラクタ.
  /// @param a_bus 使用する半二重UARTバス.
  /// @param a_baud 通信速度bps.
  /// @param a_timeout_us 応答待ちのタイムアウト(us).
  Dxl2HalfDuplexClass(MrdHalfDuplexUart &a_bus, unsigned long a_baud, unsigned long a_timeout_us)
      : m_bus(a_bus), m_baud(a_baud), m_timeout_us(a_timeout_us) {}

  /// @brief DYNAMIXEL用のフレーム形式(8N1)でバスを開始する.
  bool begin() { return m_bus.begin(m_baud, SERIAL_8N1); }

  //------------------------------------------------------------------------------------
  //  分割API: L/R系統を並行して送受信する場合に使用する
  //------------------------------------------------------------------------------------

  /// @brief Read の送信を開始する. 送信完了は待たない.
  /// @return 送信を開始できればtrue.
  bool send_read(uint8_t a_id, uint16_t a_addr, uint16_t a_len) {
    uint8_t pkt_tmp[16];
    const size_t n_tmp = dxl2::make_read(pkt_tmp, sizeof(pkt_tmp), a_id, a_addr, a_len);
    return n_tmp && m_bus.begin_send(pkt_tmp, n_tmp);
  }

  /// @brief Sync Write の送信を開始する (応答なし). 送信完了は待たない.
  /// @return 送信を開始できればtrue.
  bool send_sync_write(uint16_t a_addr, uint16_t a_len, const uint8_t *a_ids, const uint8_t *a_data,
                       size_t a_count) {
    uint8_t pkt_tmp[dxl2::PACKET_MAX];
    const size_t n_tmp = dxl2::make_sync_write(pkt_tmp, sizeof(pkt_tmp), a_addr, a_len, a_ids, a_data, a_count);
    return n_tmp && m_bus.begin_send(pkt_tmp, n_tmp);
  }

  /// @brief 送信完了を待ち, 受信に切り替える.
  void end_send() { m_bus.end_send(); }

  /// @brief Read の応答を受信する.
  /// @param a_id 期待するサーボID.
  /// @param a_data 読み出したデータの格納先 (a_len バイト, 最大8).
  /// @param a_len 読み出すデータ長.
  /// @param a_timeout_us 応答待ちのタイムアウト(us). 0なら既定値.
  /// @return 正常な応答を受信できればtrue.
  bool receive_read(uint8_t a_id, uint8_t *a_data, uint16_t a_len, unsigned long a_timeout_us = 0) {
    uint8_t rx_tmp[11 + 8];
    if (a_len > 8 || !m_bus.receive_us(rx_tmp, 11 + a_len, a_timeout_us ? a_timeout_us : m_timeout_us)) {
      return false;
    }
    if (!dxl2::check_status(rx_tmp, a_id, a_len, m_last_err)) {
      return false;
    }
    memcpy(a_data, rx_tmp + 9, a_len);
    return true;
  }

  /// @brief 応答の無いパケット(Sync Write)の送信エコーを照合する.
  /// @return エコーが一致すればtrue (エコーなしの構成では常にtrue).
  bool receive_none() { return m_bus.receive_us(nullptr, 0, m_timeout_us); }

  /// @brief 直前に受信したステータスのエラーバイト (bit7: ハードウェアエラーあり).
  uint8_t last_error() const { return m_last_err; }

  //------------------------------------------------------------------------------------
  //  1回で完結する送受信 (起動時の設定などに使用)
  //------------------------------------------------------------------------------------

  /// @brief コントロールテーブルを読み出す.
  /// @param a_timeout_us 応答待ちのタイムアウト(us). 0なら既定値.
  /// @return 正常な応答を受信できればtrue.
  bool read(uint8_t a_id, uint16_t a_addr, uint16_t a_len, uint8_t *a_data, unsigned long a_timeout_us = 0) {
    if (!send_read(a_id, a_addr, a_len)) {
      return false;
    }
    end_send();
    return receive_read(a_id, a_data, a_len, a_timeout_us);
  }

  /// @brief コントロールテーブルに書き込み, 応答を確認する.
  /// @param a_timeout_us 応答待ちのタイムアウト(us). 0なら既定値.
  /// @return 正常な応答を受信できればtrue.
  bool write(uint8_t a_id, uint16_t a_addr, const uint8_t *a_data, uint16_t a_len, unsigned long a_timeout_us = 0) {
    uint8_t params_tmp[2 + 8];
    if (a_len > 8) {
      return false;
    }
    params_tmp[0] = (uint8_t)(a_addr & 0xFF);
    params_tmp[1] = (uint8_t)(a_addr >> 8);
    memcpy(params_tmp + 2, a_data, a_len);
    uint8_t pkt_tmp[24];
    const size_t n_tmp = dxl2::make_packet(pkt_tmp, sizeof(pkt_tmp), a_id, dxl2::INST_WRITE, params_tmp, 2 + a_len);
    if (!n_tmp || !m_bus.begin_send(pkt_tmp, n_tmp)) {
      return false;
    }
    end_send();
    uint8_t rx_tmp[11];
    return m_bus.receive_us(rx_tmp, sizeof(rx_tmp), a_timeout_us ? a_timeout_us : m_timeout_us) &&
           dxl2::check_status(rx_tmp, a_id, 0, m_last_err);
  }

private:
  MrdHalfDuplexUart &m_bus;   // 使用する半二重UARTバス
  unsigned long m_baud;       // 通信速度(bps)
  unsigned long m_timeout_us; // 応答待ちのタイムアウト(us)
  uint8_t m_last_err = 0;     // 直前に受信したステータスのエラーバイト
};

//==================================================================================================
//  DYNAMIXELサーボの駆動処理
//==================================================================================================

constexpr int MRD_DXL_SLOTS = (IXL_MAX > IXR_MAX) ? IXL_MAX : IXR_MAX; // 1系統あたりの最大軸数

/// @brief 準備中の応答待ちタイムアウト(us). 出荷時の Return Delay Time は500us(設定値250 x 2us)なので長めに待つ.
constexpr unsigned long MRD_DXL_SETUP_TIMEOUT_US = 5000;

/// @brief 1系統ぶんのサーボパラメータへの参照をまとめたもの (L/Rを同じ処理で扱うため).
struct MrdDxlLine {
  Dxl2HalfDuplexClass *dxl; // 通信クラス
  int *mount;               // マウント状態 (0:なし)
  int *id;                  // サーボID
  int *cw;                  // 回転方向
  float *trim;              // トリム値(degree)
  float *tgt;               // 目標値 → 現在位置(degree)
  float *tgt_past;          // 前回の現在位置(degree)
  int *err;                 // 連続エラー回数
  uint16_t *stat;           // 通信異常の状態 (1:異常)
  int cmd_base;             // Meridim配列のコマンドの先頭インデックス(L:20, R:50)
  int8_t *torque;           // 最後に送ったトルク状態(-1:未送信, 0:OFF, 1:ON)
  bool *prepared;           // 最初のフレームの準備(Return Delay Timeの確認)を済ませたか
  const char *name;         // 系統名 (表示用)
};

/// @brief トルクを切り替える軸のリスト
struct MrdDxlTorqueJob {
  uint8_t ids[MRD_DXL_SLOTS];  // 対象ID
  uint8_t vals[MRD_DXL_SLOTS]; // トルク状態 (0:OFF, 1:ON)
  size_t count = 0;            // 対象の軸数
};

// 系統ごとの状態 (最後に送ったトルク状態と準備済みフラグ)
static int8_t mrd_dxl_torque_L[IXL_MAX];
static int8_t mrd_dxl_torque_R[IXR_MAX];
static bool mrd_dxl_prepared_L = false;
static bool mrd_dxl_prepared_R = false;

/// @brief 指定した系統の MrdDxlLine を作る.
/// @param a_line 系統(L, R).
/// @param a_sv サーボパラメータの構造体.
/// @return その系統の MrdDxlLine.
inline MrdDxlLine mrd_dxl_line(UartLine a_line, ServoParam &a_sv) {
  if (a_line == L) {
    return {&dxl_L, a_sv.ixl_mount, a_sv.ixl_id, a_sv.ixl_cw, a_sv.ixl_trim, a_sv.ixl_tgt, a_sv.ixl_tgt_past,
            a_sv.ixl_err, a_sv.ixl_stat, MRD_L_ORIGIDX, mrd_dxl_torque_L, &mrd_dxl_prepared_L, "L"};
  }
  return {&dxl_R, a_sv.ixr_mount, a_sv.ixr_id, a_sv.ixr_cw, a_sv.ixr_trim, a_sv.ixr_tgt, a_sv.ixr_tgt_past,
          a_sv.ixr_err, a_sv.ixr_stat, MRD_R_ORIGIDX, mrd_dxl_torque_R, &mrd_dxl_prepared_R, "R"};
}

/// @brief 最初のフレームで1度だけ行う準備. 各軸の応答を確認し, Return Delay Time を0にする.
///        (EEPROMからのサーボ設定の読み込みが setup() の後半にあるため, 最初のフレームで行う)
///        ハードウェアエラーがある軸は, その内容を起動ログに表示する.
/// @param a_ln 対象の系統.
/// @param a_num 処理する軸数.
inline void mrd_dxl_prepare(MrdDxlLine &a_ln, int a_num) {
  const unsigned long to_tmp = MRD_DXL_SETUP_TIMEOUT_US;
  for (int i = 0; i < a_num; i++) {
    a_ln.torque[i] = -1;
    if (!a_ln.mount[i]) {
      continue;
    }
    const uint8_t id_tmp = (uint8_t)a_ln.id[i];
    uint8_t rdt_tmp = 0;
    Serial.printf("DXL %s[%02d] id:%3d ", a_ln.name, i, id_tmp);
    if (!a_ln.dxl->read(id_tmp, dxl2::ADDR_RETURN_DELAY, 1, &rdt_tmp, to_tmp)) {
      Serial.println("no response");
      continue;
    }
    const bool alert_tmp = a_ln.dxl->last_error() & 0x80;
    if (rdt_tmp != 0) { // EEPROM領域の書き込みにはトルクOFFが必要
      const uint8_t off_tmp = 0, zero_tmp = 0;
      const bool ok_tmp = a_ln.dxl->write(id_tmp, dxl2::ADDR_TORQUE_ENABLE, &off_tmp, 1, to_tmp) &&
                          a_ln.dxl->write(id_tmp, dxl2::ADDR_RETURN_DELAY, &zero_tmp, 1, to_tmp);
      Serial.printf("ok (Return Delay Time %uus -> 0 %s)", rdt_tmp * 2, ok_tmp ? "done" : "FAILED");
    } else {
      Serial.print("ok");
    }
    if (alert_tmp) { // ハードウェアエラー(過負荷, 電圧, 過熱など). 解除にはサーボの電源の入れ直しが必要
      uint8_t hw_tmp = 0;
      if (a_ln.dxl->read(id_tmp, dxl2::ADDR_HW_ERROR, 1, &hw_tmp, to_tmp)) {
        Serial.printf(" [HW ERROR 0x%02X:%s%s%s%s%s]", hw_tmp, (hw_tmp & 0x01) ? " voltage" : "",
                      (hw_tmp & 0x04) ? " overheat" : "", (hw_tmp & 0x08) ? " encoder" : "",
                      (hw_tmp & 0x10) ? " shock" : "", (hw_tmp & 0x20) ? " overload" : "");
      } else {
        Serial.print(" [HW ERROR]");
      }
    }
    Serial.println();
  }
  *a_ln.prepared = true;
}

/// @brief 目標位置の Sync Write の送信を開始し, トルクを切り替える軸を a_job に集める.
/// @param a_ln 対象の系統.
/// @param a_meridim Meridim配列 (各軸のコマンドを参照する).
/// @param a_num 処理する軸数.
/// @param a_job トルクを切り替える軸の格納先.
/// @return 目標位置を送信した場合はtrue (トルクONの軸が無ければ送信しない).
inline bool mrd_dxl_begin_goal(MrdDxlLine &a_ln, Meridim90Union &a_meridim, int a_num, MrdDxlTorqueJob &a_job) {
  uint8_t ids_tmp[MRD_DXL_SLOTS];
  uint8_t data_tmp[MRD_DXL_SLOTS * 4];
  size_t n_tmp = 0;
  a_job.count = 0;
  for (int i = 0; i < a_num; i++) {
    if (!a_ln.mount[i]) {
      continue;
    }
    // コマンドが1ならトルクON. トルクONの軸だけ目標位置を送る
    const int8_t want_tmp = (a_meridim.sval[a_ln.cmd_base + i * 2] == 1) ? 1 : 0;
    if (want_tmp) {
      const int32_t pos_tmp = dxl2::deg_to_pos(a_ln.tgt[i], a_ln.trim[i], a_ln.cw[i]);
      ids_tmp[n_tmp] = (uint8_t)a_ln.id[i];
      data_tmp[n_tmp * 4 + 0] = (uint8_t)(pos_tmp & 0xFF);
      data_tmp[n_tmp * 4 + 1] = (uint8_t)((pos_tmp >> 8) & 0xFF);
      data_tmp[n_tmp * 4 + 2] = (uint8_t)((pos_tmp >> 16) & 0xFF);
      data_tmp[n_tmp * 4 + 3] = (uint8_t)((pos_tmp >> 24) & 0xFF);
      n_tmp++;
    }
    if (a_ln.torque[i] != want_tmp) {
      a_job.ids[a_job.count] = (uint8_t)a_ln.id[i];
      a_job.vals[a_job.count] = (uint8_t)want_tmp;
      a_job.count++;
      a_ln.torque[i] = want_tmp;
    }
  }
  return n_tmp && a_ln.dxl->send_sync_write(dxl2::ADDR_GOAL_POSITION, 4, ids_tmp, data_tmp, n_tmp);
}

/// @brief 現在位置の応答を処理し, サーボ配列の角度とエラー状態を更新する.
/// @param a_ln 対象の系統.
/// @param a_i 軸のインデックス.
/// @param a_sent 読み出しを送信できたか.
inline void mrd_dxl_finish_read(MrdDxlLine &a_ln, int a_i, bool a_sent) {
  uint8_t d_tmp[4];
  if (a_sent && a_ln.dxl->receive_read((uint8_t)a_ln.id[a_i], d_tmp, 4)) {
    const int32_t pos_tmp = (int32_t)((uint32_t)d_tmp[0] | ((uint32_t)d_tmp[1] << 8) | ((uint32_t)d_tmp[2] << 16) |
                                      ((uint32_t)d_tmp[3] << 24));
    a_ln.tgt[a_i] = dxl2::pos_to_deg(pos_tmp, a_ln.trim[a_i], a_ln.cw[a_i]);
    a_ln.err[a_i] = 0;
    a_ln.stat[a_i] = 0;
  } else { // 応答が無い場合は前回の値を使い, 連続した失敗で通信不能とみなす (KRSと同じ)
    a_ln.tgt[a_i] = a_ln.tgt_past[a_i];
    a_ln.err[a_i]++;
    if (a_ln.err[a_i] >= SERVO_LOST_ERR_WAIT) {
      a_ln.err[a_i] = SERVO_LOST_ERR_WAIT;
      a_ln.stat[a_i] = 1;
    }
  }
}

/// @brief DYNAMIXEL を1系統または2系統(L/R並行)で駆動する.
///        2系統の場合は, 各手順で両系統の送信を開始してから送信完了と受信を待つことで, L/Rの通信を重ねる.
/// @param a_lines 駆動する系統 (1または2個).
/// @param a_count 系統数.
/// @param a_meridim Meridim配列.
/// @param a_num 処理する軸数 (num_max).
inline void mrd_dxl_drive(MrdDxlLine *a_lines, int a_count, Meridim90Union &a_meridim, int a_num) {
  for (int k = 0; k < a_count; k++) {
    if (!*a_lines[k].prepared) {
      mrd_dxl_prepare(a_lines[k], a_num);
    }
  }

  // 1. 目標位置の Sync Write. 先に目標位置を送ってからトルクONにし, 古い目標位置へ動かないようにする
  MrdDxlTorqueJob jobs_tmp[2];
  bool sent_tmp[2] = {false, false};
  for (int k = 0; k < a_count; k++) {
    sent_tmp[k] = mrd_dxl_begin_goal(a_lines[k], a_meridim, a_num, jobs_tmp[k]);
  }
  for (int k = 0; k < a_count; k++) {
    if (sent_tmp[k]) {
      a_lines[k].dxl->end_send();
    }
  }
  for (int k = 0; k < a_count; k++) {
    if (sent_tmp[k]) {
      a_lines[k].dxl->receive_none();
    }
  }

  // 2. トルクを切り替える軸がある系統だけ Sync Write
  for (int k = 0; k < a_count; k++) {
    sent_tmp[k] = jobs_tmp[k].count && a_lines[k].dxl->send_sync_write(dxl2::ADDR_TORQUE_ENABLE, 1, jobs_tmp[k].ids,
                                                                       jobs_tmp[k].vals, jobs_tmp[k].count);
  }
  for (int k = 0; k < a_count; k++) {
    if (sent_tmp[k]) {
      a_lines[k].dxl->end_send();
    }
  }
  for (int k = 0; k < a_count; k++) {
    if (sent_tmp[k]) {
      a_lines[k].dxl->receive_none();
    }
  }

  // 3. 現在位置を1軸ずつ読み出す (同じインデックスのL/Rを並行)
  for (int i = 0; i < a_num; i++) {
    for (int k = 0; k < a_count; k++) {
      sent_tmp[k] = a_lines[k].mount[i] &&
                    a_lines[k].dxl->send_read((uint8_t)a_lines[k].id[i], dxl2::ADDR_PRESENT_POSITION, 4);
    }
    for (int k = 0; k < a_count; k++) {
      if (sent_tmp[k]) {
        a_lines[k].dxl->end_send();
      }
    }
    for (int k = 0; k < a_count; k++) {
      if (a_lines[k].mount[i]) {
        mrd_dxl_finish_read(a_lines[k], i, sent_tmp[k]);
      }
    }
  }
}

/// @brief DYNAMIXELを1系統で駆動する.
/// @param a_meridim Meridim配列.
/// @param a_sv サーボパラメータの構造体.
/// @param a_line 駆動する系統(L, R).
inline void mrd_sv_drive_dxl2_line(Meridim90Union &a_meridim, ServoParam &a_sv, UartLine a_line) {
  MrdDxlLine ln_tmp[1] = {mrd_dxl_line(a_line, a_sv)};
  mrd_dxl_drive(ln_tmp, 1, a_meridim, a_sv.num_max);
}

/// @brief DYNAMIXELをL/R系統で並行して駆動する.
/// @param a_meridim Meridim配列.
/// @param a_sv サーボパラメータの構造体.
inline void mrd_sv_drive_dxl2_double(Meridim90Union &a_meridim, ServoParam &a_sv) {
  MrdDxlLine ln_tmp[2] = {mrd_dxl_line(L, a_sv), mrd_dxl_line(R, a_sv)};
  mrd_dxl_drive(ln_tmp, 2, a_meridim, a_sv.num_max);
}

#endif // __MERIDIAN_SERVO_DYNAMIXEL_H__
