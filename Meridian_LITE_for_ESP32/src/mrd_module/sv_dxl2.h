#ifndef __MERIDIAN_SERVO_DYNAMIXEL2_H__
#define __MERIDIAN_SERVO_DYNAMIXEL2_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバスと共通処理. 本ファイルは mrd_servo.h から読み込まれる
#include "sv_util.h"   // 各プロトコル層の共通部品

//==================================================================================================
//  DYNAMIXEL Protocol 2.0 プロトコル層 (半二重UARTバス上で動作)
//==================================================================================================
//
// 対象: Xシリーズ(XL330, XL430, XC430, XM430, XH430, XM540等)およびProtocol 2.0のMXシリーズ.
//       制御テーブルが異なる機種(XL-320, PRO/Pシリーズ)には対応しない.
//
// 1フレームの処理 (1系統15台を100Hzで動かすための構成):
//   [1] 目標位置: SyncWrite(0x83, アドレス116, 4バイト). トルクONのサーボのみ.
//   [2] トルク: ON/OFFが変わったサーボだけSyncWrite(アドレス64, 1バイト).
//       サーボの保護機能等でトルクが切れても復帰できるよう, MRD_SERVO_TORQUE_REFRESH_FRAMES ごとに全サーボへ送り直す.
//       ※ハードウェアエラー(過負荷等)でトルクが切れた場合は, サーボの再起動(電源の入れ直し)まで復帰しない.
//   [3] 現在位置: SyncRead(0x82, アドレス132, 4バイト)で全サーボを1回の要求で読む. 各サーボがID順に続けて応答する.
//   [4] 前のフレームで読めなかったサーボは[3]から外し, 1フレームに1台ずつ個別のREADで確認する.
//       (SyncReadで途中のサーボが応答しない場合に後続のサーボがどうなるかは仕様に記載がないため,
//        応答しないサーボを含めたまま毎フレームSyncReadすることを避ける)
//   L/R系統は mrd_servo_lines_run で並行して進める(もう一方の系統が他のプロトコルでもよい).
//   1Mbps, 15台の試算: [1] 89byte=0.89ms + [3] 要求29byte+応答15x15byte=2.54ms → 約3.5ms/フレーム.
//   ※サーボの応答遅延時間(アドレス9, 初期値250=500us)が0でないとSyncReadが1台あたり最大500us遅れるため,
//     起動時に0にする(SERVO_DXL_FIX_RETURN_DELAY). 初期値の通信速度は57600bpsなので, 事前に
//     DYNAMIXEL Wizard等で1Mbps以上(ESP32は4.5Mbpsまで設定可)に変更しておくこと.
//
// 位置値は0-4095が360deg, 中央2048を0degとする.

// パケット
#define DXL2_ID_BROADCAST    0xFE // ブロードキャストID
#define DXL2_INST_PING       0x01 // 応答確認(モデル番号を返す)
#define DXL2_INST_READ       0x02 // 読み出し
#define DXL2_INST_WRITE      0x03 // 書き込み
#define DXL2_INST_SYNC_READ  0x82 // 一斉読み出し(各サーボがID順に応答)
#define DXL2_INST_SYNC_WRITE 0x83 // 一斉書き込み(応答なし)
#define DXL2_INST_STATUS     0x55 // 応答パケットの命令欄

// レジスタアドレス (Xシリーズ制御テーブル)
#define DXL2_REG_RETURN_DELAY     9   // 応答遅延時間(単位2us, EEPROM. トルクOFF時のみ書き込める)
#define DXL2_REG_OPERATING_MODE   11  // 動作モード(3:位置制御, 4:拡張位置制御, 5:電流制限付き位置制御)
#define DXL2_REG_TORQUE_ENABLE    64  // トルクON/OFF
#define DXL2_REG_HW_ERROR         70  // ハードウェアエラーの内容(応答のエラー欄のアラートビットが立っている場合)
#define DXL2_REG_PROFILE_ACC      108 // 加速度プロファイル(4バイト, 0:制限なし)
#define DXL2_REG_PROFILE_VEL      112 // 速度プロファイル(4バイト, 0:制限なし)
#define DXL2_REG_GOAL_VELOCITY    104 // 目標速度(4バイト, 0.229rpm. 速度制御モード)
#define DXL2_REG_PRESENT_VELOCITY 128 // 現在速度(4バイト, 0.229rpm)
#define DXL2_REG_GOAL_POSITION    116 // 目標位置(4バイト)
#define DXL2_REG_PRESENT_POSITION 132 // 現在位置(4バイト)

#define DXL2_TX_MAX     MRD_HDUART_TX_MAX // 1パケットの最大長
#define DXL2_STATUS_MIN 11                // データなしの応答パケット長
#define DXL2_POS_REPLY  15                // 現在位置(4バイト)の応答パケット長
#define DXL2_POS_RES    4096.0f
#define DXL2_POS_RANGE  360.0f

//------------------------------------------------------------------------------------
//  パケットの生成と検証
//------------------------------------------------------------------------------------
//
// 命令パケット: FF FF FD 00 ID LEN_L LEN_H INST [PARAM...] CRC_L CRC_H   (LEN = PARAM数 + 3)
// 応答パケット: FF FF FD 00 ID LEN_L LEN_H 0x55 ERR [DATA...] CRC_L CRC_H (LEN = DATA数 + 4)
// INST以降に FF FF FD が現れた場合は直後に FD を挿入する(バイトスタッフィング). CRCは挿入後のパケットで計算する.
// CRC: CRC-16(多項式0x8005, 初期値0, 反転なし). 先頭からCRCの手前まで.

/// @brief CRCを計算する.
/// @param a_pkt パケット.
/// @param a_len CRCの手前までのバイト数.
inline uint16_t mrd_dxl2_crc(const uint8_t *a_pkt, size_t a_len) { return mrd_servo_crc16(a_pkt, a_len, 0x8005); }

/// @brief 命令パケットを組み立てる.
///        使い方: start(ID, 命令) → put8/put16/put32で引数を追加 → finish().
struct Dxl2Packet {
  uint8_t buf[DXL2_TX_MAX]; // 完成したパケット
  size_t len = 0;           // finish()後のパケット長(0:組み立て失敗)

  void start(uint8_t a_id, uint8_t a_inst) {
    m_id = a_id;
    m_raw[0] = a_inst;
    m_raw_len = 1;
    m_over = false;
    len = 0;
  }

  void put8(uint8_t a_val) {
    if (m_raw_len >= sizeof(m_raw)) {
      m_over = true;
      return;
    }
    m_raw[m_raw_len++] = a_val;
  }
  void put16(uint16_t a_val) {
    put8(a_val & 0xFF);
    put8(a_val >> 8);
  }
  void put32(uint32_t a_val) {
    put16(a_val & 0xFFFF);
    put16(a_val >> 16);
  }

  /// @brief バイトスタッフィング, LEN, CRCを確定する.
  /// @return パケット長. バッファに入りきらなければ0.
  size_t finish() {
    len = 0;
    if (m_over) {
      return 0;
    }
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0xFD;
    buf[3] = 0x00;
    buf[4] = m_id;
    size_t pos_tmp = 7;
    for (size_t i = 0; i < m_raw_len; i++) {
      if (pos_tmp + 2 + 2 > sizeof(buf)) { // 挿入1バイトとCRC2バイトの余裕
        return 0;
      }
      buf[pos_tmp++] = m_raw[i];
      if (pos_tmp - 7 >= 3 && buf[pos_tmp - 3] == 0xFF && buf[pos_tmp - 2] == 0xFF && buf[pos_tmp - 1] == 0xFD) {
        buf[pos_tmp++] = 0xFD;
      }
    }
    const uint16_t pkt_len_tmp = pos_tmp - 7 + 2;
    buf[5] = pkt_len_tmp & 0xFF;
    buf[6] = pkt_len_tmp >> 8;
    const uint16_t crc_tmp = mrd_dxl2_crc(buf, pos_tmp);
    buf[pos_tmp++] = crc_tmp & 0xFF;
    buf[pos_tmp++] = crc_tmp >> 8;
    len = pos_tmp;
    return len;
  }

private:
  uint8_t m_raw[DXL2_TX_MAX]; // INSTとPARAM(スタッフィング前)
  size_t m_raw_len = 0;
  uint8_t m_id = 0;
  bool m_over = false;
};

/// @brief 応答パケットのヘッダ, ID, 長さ, 命令欄, CRCを検証し, エラー番号(bit7のアラートを除く)を取り出す.
///        エラー番号が0でなくても, 形式が正しければtrueを返す(書き込みの拒否などを区別するため).
/// @param a_rx 受信データ.
/// @param a_rx_len 受信データ数(11 + データ数).
/// @param a_id 期待するID.
/// @param a_err エラー番号の格納先.
inline bool mrd_dxl2_status_check_err(const uint8_t *a_rx, size_t a_rx_len, uint8_t a_id, uint8_t &a_err) {
  if (a_rx_len < DXL2_STATUS_MIN) {
    return false;
  }
  const size_t len_tmp = a_rx[5] | (a_rx[6] << 8);
  const uint16_t crc_tmp = a_rx[a_rx_len - 2] | (a_rx[a_rx_len - 1] << 8);
  a_err = a_rx[8] & 0x7F;
  return a_rx[0] == 0xFF && a_rx[1] == 0xFF && a_rx[2] == 0xFD && a_rx[3] == 0x00 && a_rx[4] == a_id &&
         len_tmp == a_rx_len - 7 && a_rx[7] == DXL2_INST_STATUS && crc_tmp == mrd_dxl2_crc(a_rx, a_rx_len - 2);
}

/// @brief 応答パケットを検証する(形式が正しく, エラー番号が0). bit7(アラート: ハードウェアエラー発生中)は
///        データが有効なため許容する.
inline bool mrd_dxl2_status_check(const uint8_t *a_rx, size_t a_rx_len, uint8_t a_id) {
  uint8_t err_tmp = 0;
  return mrd_dxl2_status_check_err(a_rx, a_rx_len, a_id, err_tmp) && err_tmp == 0;
}

/// @brief 4バイト値(LE, 符号付き)を読み取る.
inline int32_t mrd_dxl2_i32_get(const uint8_t *a_buf) { return int32_t(mrd_pc_le_get(a_buf, 4)); }

/// @brief 位置値の尺度: 0-4095が360deg, 中央2048が0deg.
static const MrdPosScale DXL2_POS_SCALE = {DXL2_POS_RES / DXL2_POS_RANGE, DXL2_POS_RES * 0.5f, 0.0f, DXL2_POS_RES - 1.0f};

/// @brief 角度(degree)をDYNAMIXELの位置値(0-4095)に変換する.
inline int mrd_dxl2_deg2pos(float a_degree, float a_trim, int a_cw) {
  return mrd_servo_scale_deg2pos(a_degree, a_trim, a_cw, DXL2_POS_SCALE);
}

/// @brief DYNAMIXELの位置値を角度(degree)に変換する.
inline float mrd_dxl2_pos2deg(int32_t a_pos, float a_trim, int a_cw) {
  return mrd_servo_scale_pos2deg(a_pos, a_trim, a_cw, DXL2_POS_SCALE);
}

//------------------------------------------------------------------------------------
//  DYNAMIXELサーボ1系統分のクラス
//------------------------------------------------------------------------------------

class Dynamixel2HalfDuplexClass {
public:
  /// @brief コンストラクタ.
  /// @param a_bus 使用する半二重UARTバス.
  /// @param a_baud 通信速度bps(サーボの設定に合わせる. Xシリーズの初期値は57600).
  Dynamixel2HalfDuplexClass(MrdHalfDuplexUart &a_bus, unsigned long a_baud) : m_bus(a_bus), m_baud(a_baud) {}

  /// @brief 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ.
  void set_baud(unsigned long a_baud) { m_baud = a_baud; }

  /// @brief バスを開始(8N1)し, 接続されたサーボの確認と初期設定を行う.
  /// @param a_line_name 表示用の系統名("L"など).
  /// @param a_mount 各サーボのマウント有無.
  /// @param a_id 各サーボのID.
  /// @param a_num サーボ数.
  /// @return バスを開始できればtrue.
  bool begin(const char *a_line_name, const int *a_mount, const int *a_id, int a_num) {
    m_state.reset();
    memset(m_model, 0, sizeof(m_model));
    if (!m_bus.begin(m_baud, SERIAL_8N1)) {
      return false;
    }
    const unsigned long bauds_tmp[] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000};
    mrd_servo_baud_check(a_line_name, "DXL2", m_baud, bauds_tmp, sizeof(bauds_tmp) / sizeof(bauds_tmp[0]),
                         " bps is not a DYNAMIXEL X baud rate. Check SERVO_BAUDRATE_x.");

    // 加速度・速度プロファイルを0(制限なし)にする. 15台分を1パケットにすると最大長を超えるため2つに分ける
    Dxl2Packet acc_tmp;
    Dxl2Packet vel_tmp;
    acc_tmp.start(DXL2_ID_BROADCAST, DXL2_INST_SYNC_WRITE);
    acc_tmp.put16(DXL2_REG_PROFILE_ACC);
    acc_tmp.put16(4);
    vel_tmp.start(DXL2_ID_BROADCAST, DXL2_INST_SYNC_WRITE);
    vel_tmp.put16(DXL2_REG_PROFILE_VEL);
    vel_tmp.put16(4);
    int profile_num_tmp = 0;

    for (int i = 0; i < a_num && i < MRD_SERVO_SLOTS; i++) {
      if (!a_mount[i]) {
        continue;
      }
      const uint8_t id_tmp = a_id[i];
      mrd_servo_slot_print(a_line_name, "DXL2", i, id_tmp);

      uint16_t model_tmp = 0;
      bool found_tmp = false;
      for (int k = 0; k < MRD_SERVO_BOOT_RETRY && !found_tmp; k++) {
        found_tmp = ping(id_tmp, model_tmp);
      }
      if (!found_tmp) {
        Serial.println("no response");
        continue;
      }
      Serial.print("model ");
      Serial.print(model_tmp);
      m_model[i] = model_tmp;

      // 応答遅延時間が0でなければ0にする(SyncReadで1台あたり最大500usの待ちを防ぐ)
      uint8_t delay_tmp = 0;
      bool delay_read_tmp = false;
      for (int k = 0; k < MRD_SERVO_BOOT_RETRY && !delay_read_tmp; k++) {
        delay_read_tmp = read(id_tmp, DXL2_REG_RETURN_DELAY, &delay_tmp, 1);
      }
      Serial.print(" return delay ");
      if (!delay_read_tmp) {
        Serial.print("read failed");
      } else {
        Serial.print(delay_tmp * 2);
        Serial.print("us");
        if (delay_tmp != 0 && cfg.servo_dxl_fix_return_delay) {
          Serial.print(fix_return_delay(id_tmp) ? " -> 0us" : " -> fix failed");
        }
      }

      // 位置制御系のモードでなければ目標位置で動かないため知らせる
      uint8_t mode_tmp = 0;
      if (read(id_tmp, DXL2_REG_OPERATING_MODE, &mode_tmp, 1) && mode_tmp != 3 && mode_tmp != 5) {
        Serial.print(" (operating mode ");
        Serial.print(mode_tmp);
        // 4(拡張位置制御)は動くが, 目標は1回転(0〜4095)の範囲に限られる
        Serial.print(mode_tmp == 4 ? ": goals are limited to one turn)" : " is not position control)");
      }

      // ハードウェアエラー(過負荷, 電圧, 過熱など)を表示する. トルクが切れ, 解除にはサーボの電源の入れ直しが必要
      if (m_alert) {
        uint8_t hw_tmp = 0;
        if (read(id_tmp, DXL2_REG_HW_ERROR, &hw_tmp, 1)) {
          Serial.print(" [HW ERROR 0x");
          Serial.print(hw_tmp, HEX);
          Serial.print(":");
          Serial.print((hw_tmp & 0x01) ? " voltage" : "");
          Serial.print((hw_tmp & 0x04) ? " overheat" : "");
          Serial.print((hw_tmp & 0x08) ? " encoder" : "");
          Serial.print((hw_tmp & 0x10) ? " shock" : "");
          Serial.print((hw_tmp & 0x20) ? " overload" : "");
          Serial.print("]");
        } else {
          Serial.print(" [HW ERROR]");
        }
      }
      Serial.println();

      acc_tmp.put8(id_tmp);
      acc_tmp.put32(0);
      vel_tmp.put8(id_tmp);
      vel_tmp.put32(0);
      profile_num_tmp++;
    }

    if (profile_num_tmp > 0 && acc_tmp.finish() > 0 && vel_tmp.finish() > 0) {
      m_bus.transfer(acc_tmp.buf, acc_tmp.len, nullptr, 0);
      m_bus.transfer(vel_tmp.buf, vel_tmp.len, nullptr, 0);
    }
    return true;
  }

  /// @brief PINGでサーボの応答を確認する.
  /// @param a_id ID.
  /// @param a_model モデル番号の格納先.
  /// @return 正しい応答を受信できればtrue.
  bool ping(uint8_t a_id, uint16_t &a_model) {
    Dxl2Packet pkt_tmp;
    pkt_tmp.start(a_id, DXL2_INST_PING);
    uint8_t rx_tmp[DXL2_STATUS_MIN + 3];
    if (pkt_tmp.finish() == 0 || !m_bus.transfer(pkt_tmp.buf, pkt_tmp.len, rx_tmp, sizeof(rx_tmp)) ||
        !mrd_dxl2_status_check(rx_tmp, sizeof(rx_tmp), a_id)) {
      return false;
    }
    a_model = rx_tmp[9] | (rx_tmp[10] << 8);
    m_alert = (rx_tmp[8] & 0x80) != 0;
    return true;
  }

  /// @brief レジスタを読み出す(1回の送受信).
  /// @return 正しい応答を受信できればtrue.
  bool read(uint8_t a_id, uint16_t a_addr, uint8_t *a_data, uint8_t a_len) {
    Dxl2Packet pkt_tmp;
    pkt_tmp.start(a_id, DXL2_INST_READ);
    pkt_tmp.put16(a_addr);
    pkt_tmp.put16(a_len);
    uint8_t rx_tmp[DXL2_STATUS_MIN + 16];
    const size_t rx_len_tmp = DXL2_STATUS_MIN + a_len;
    if (rx_len_tmp > sizeof(rx_tmp) || pkt_tmp.finish() == 0 ||
        !m_bus.transfer(pkt_tmp.buf, pkt_tmp.len, rx_tmp, rx_len_tmp) ||
        !mrd_dxl2_status_check(rx_tmp, rx_len_tmp, a_id)) {
      return false;
    }
    memcpy(a_data, &rx_tmp[9], a_len);
    return true;
  }

  /// @brief レジスタに書き込み, 応答を確認する(1回の送受信).
  /// @return 正しい応答(エラーなし)を受信できればtrue.
  bool write(uint8_t a_id, uint16_t a_addr, const uint8_t *a_data, uint8_t a_len) {
    Dxl2Packet pkt_tmp;
    pkt_tmp.start(a_id, DXL2_INST_WRITE);
    pkt_tmp.put16(a_addr);
    for (int i = 0; i < a_len; i++) {
      pkt_tmp.put8(a_data[i]);
    }
    uint8_t rx_tmp[DXL2_STATUS_MIN];
    return pkt_tmp.finish() > 0 && m_bus.transfer(pkt_tmp.buf, pkt_tmp.len, rx_tmp, sizeof(rx_tmp)) &&
           mrd_dxl2_status_check(rx_tmp, sizeof(rx_tmp), a_id);
  }

  /// @brief 半二重UARTバス(並行送受信用).
  MrdHalfDuplexUart &bus() { return m_bus; }

  /// @brief 通信速度bps(SyncReadの受信待ち時間の計算用).
  unsigned long baud() const { return m_baud; }

  /// @brief フレームをまたいで保持する状態(トルク状態の記憶等).
  MrdServoLineState m_state;

  /// @brief 起動時に確認した各スロットのモデル番号(0:応答なし). 機種で単位が違う項目(電流)の判定に使う.
  uint16_t m_model[MRD_SERVO_SLOTS] = {0};

private:
  /// @brief 応答遅延時間(EEPROM)を0にする. EEPROMはトルクOFFでないと書き込めないため先にトルクOFFにする
  ///        (起動直後はトルクOFFなので通常は影響なし. トルクは駆動処理が指示に合わせて送り直す).
  bool fix_return_delay(uint8_t a_id) {
    const uint8_t off_tmp = 0;
    const uint8_t delay_tmp = 0;
    bool ok_tmp = write(a_id, DXL2_REG_TORQUE_ENABLE, &off_tmp, 1);
    ok_tmp = ok_tmp && write(a_id, DXL2_REG_RETURN_DELAY, &delay_tmp, 1);
    uint8_t check_tmp = 0xFF;
    return ok_tmp && read(a_id, DXL2_REG_RETURN_DELAY, &check_tmp, 1) && check_tmp == 0;
  }

  MrdHalfDuplexUart &m_bus;
  unsigned long m_baud;
  bool m_alert = false; // 直前のPINGの応答でハードウェアエラーのアラートが立っていたか
};

//==================================================================================================
//  DYNAMIXELサーボの駆動処理
//==================================================================================================

/// @brief 系統のDYNAMIXELサーボクラス.
inline Dynamixel2HalfDuplexClass &mrd_dxl2_line(int a_line) { return (a_line == L) ? dxl2_l : dxl2_r; }

/// @brief 系統の参照を作る.
/// @param a_line 系統(L or R).
/// @param a_meridim Meridimデータの参照.
/// @param a_sv サーボパラメータ.
MrdServoLineRef mrd_dxl2_line_ref(UartLine a_line, Meridim90Union &a_meridim, ServoParam &a_sv) {
  return mrd_servo_line_ref(a_line, DXL2, mrd_dxl2_line(a_line).bus(), mrd_dxl2_line(a_line).m_state, a_meridim, a_sv);
}

//------------------------------------------------------------------------------------
//  パラメータ系コマンド (番号と値の形式は tasks/servo_param_plan.md)
//------------------------------------------------------------------------------------

#define DXL2_MODEL_XL430_W250 1060 // 電流の制御テーブルがない(126番地は負荷)
#define DXL2_MODEL_XL330_M288 1200 // 電流の単位が1mA
#define DXL2_MODEL_XL330_M077 1190

/// @brief パラメータ系コマンドの対応表. EEPROM領域(64番地未満)はトルクOFFでないと書けない(サーボがエラーを返す).
/// @param a_cmd コマンド.
/// @param a_model モデル番号(0:不明).
/// @return レジスタの定義(len=0:未対応).
MrdParamReg mrd_dxl2_param_reg(uint8_t a_cmd, uint16_t a_model) {
  const bool xl330_tmp = (a_model == DXL2_MODEL_XL330_M288 || a_model == DXL2_MODEL_XL330_M077);
  const bool no_current_tmp = (a_model == DXL2_MODEL_XL430_W250);
  const float cur_k_tmp = xl330_tmp ? 1.0f : 2.69f; // 電流の単位(mA)
  const float vel_k_tmp = 0.229f * 6.0f;            // 0.229rpm → deg/s
  switch (a_cmd) {
  case 4:
  case 5:
    return mrd_pc_reg(DXL2_REG_GOAL_POSITION, 4, a_cmd == 4, PC_POS, 1.0f, PC_SIGNED);
  case 6:
    return mrd_pc_reg(DXL2_REG_PRESENT_POSITION, 4, false, PC_POS, 1.0f, PC_SIGNED);
  case 7:
    return mrd_pc_reg(144, 2, false, PC_SCALE, 10.0f); // 0.1V → V x100
  case 8:
  case 9:
    return mrd_pc_reg(20, 4, a_cmd == 9, PC_SCALE, 36000.0f / DXL2_POS_RES, PC_SIGNED); // ホーミングオフセット
  case 12:
  case 13:
    return mrd_pc_reg(52, 4, a_cmd == 13, PC_POS_NATIVE, 1.0f, PC_SIGNED); // 最小位置(CW側)
  case 14:
  case 15:
    return mrd_pc_reg(48, 4, a_cmd == 15, PC_POS_NATIVE, 1.0f, PC_SIGNED); // 最大位置(CCW側)
  case 16:
  case 17:
    return mrd_pc_reg(84, 2, a_cmd == 17);
  case 18:
  case 19:
    return mrd_pc_reg(82, 2, a_cmd == 19);
  case 20:
  case 21:
    return mrd_pc_reg(80, 2, a_cmd == 21);
  case 22:
  case 23:
    return mrd_pc_reg(31, 1, a_cmd == 23);
  case 24:
  case 25:
    return no_current_tmp ? MrdParamReg() : mrd_pc_reg(38, 2, a_cmd == 25, PC_SCALE, cur_k_tmp);
  case 26:
    return mrd_pc_reg(DXL2_REG_PRESENT_VELOCITY, 4, false, PC_SCALE, vel_k_tmp, PC_SIGNED); // 現在速度
  case 27:
    return mrd_pc_reg(DXL2_REG_PROFILE_VEL, 4, true, PC_SCALE, vel_k_tmp); // 速度プロファイル(速度ベース時)
  case 28:
  case 29:
    return mrd_pc_reg(DXL2_REG_PROFILE_ACC, 4, a_cmd == 29); // 加速度プロファイル(時間ベース時はms)
  case 30:
  case 31:
    return mrd_pc_reg(36, 2, a_cmd == 31, PC_SCALE, 0.113f); // PWM制限(0.113%)
  case 32:
  case 33:
    return mrd_pc_reg(DXL2_REG_PROFILE_VEL, 4, a_cmd == 33); // 速度プロファイル(時間ベース時は到達時間ms)
  case 36:
    return mrd_pc_reg(146, 1, false);
  case 37:
    return no_current_tmp ? MrdParamReg() : mrd_pc_reg(126, 2, false, PC_SCALE, cur_k_tmp, PC_SIGNED);
  case 38:
    return mrd_pc_reg(DXL2_REG_TORQUE_ENABLE, 1, false, PC_BOOL);
  case 70:
  case 71:
    return mrd_pc_reg(DXL2_REG_RETURN_DELAY, 1, a_cmd == 71);
  default:
    return MrdParamReg();
  }
}

/// @brief DYNAMIXELのパラメータ系コマンド1件分の処理(1回の送受信).
///        3:PING, 35:ROM保存(EEPROMは書いた時点で保存されるため成功を返す), それ以外は対応表のREAD/WRITE.
///        書き込みの結果は受け取ったVAL(サーボがエラーを返した場合はエラー値).
class Dxl2ParamOp : public MrdParamOpBase {
public:
  void begin(const MrdServoLineRef &a_line, int a_ix, uint8_t a_cmd, int16_t a_val) {
    target(a_line, a_ix, a_val);
    m_scale = DXL2_POS_SCALE;
    if (a_cmd == 3) {
      m_step = PING;
      return;
    }
    if (a_cmd == 35) {
      finish(1);
      return;
    }
    m_reg = mrd_dxl2_param_reg(a_cmd, mrd_dxl2_line(a_line.line).m_model[a_ix]);
    if (m_reg.len == 0) {
      finish(MRD_PCMD_NA);
      return;
    }
    if (!write_value_ok()) { // レジスタに収まらない値は送らない
      finish(MRD_PCMD_NA);
      return;
    }
    m_step = m_reg.write ? WRITE : READ;
  }

  /// @brief 車輪(速度制御モードを事前に設定したサーボ): トルクON(必要なときだけ) → 目標速度(104) → 現在速度(128).
  ///        速度は0.01rpm(回転方向補正を適用). 結果は現在速度.
  void begin_wheel(const MrdServoLineRef &a_line, int a_ix, int16_t a_speed) {
    target(a_line, a_ix, a_speed);
    m_step = wheel_needs_torque() ? W_TORQUE : W_GOAL;
  }

  bool next(MrdHduartXfer &a_x) {
    size_t rx_len_tmp = DXL2_STATUS_MIN;
    switch (m_step) {
    case PING:
      m_pkt.start(id(), DXL2_INST_PING);
      rx_len_tmp += 3;
      break;
    case READ:
      m_pkt.start(id(), DXL2_INST_READ);
      m_pkt.put16(m_reg.addr);
      m_pkt.put16(m_reg.len);
      rx_len_tmp += m_reg.len;
      break;
    case WRITE: {
      m_pkt.start(id(), DXL2_INST_WRITE);
      m_pkt.put16(m_reg.addr);
      uint8_t data_tmp[4];
      mrd_pc_le_put(data_tmp, mrd_pc_encode(to_raw(), m_reg.sign), m_reg.len);
      for (int i = 0; i < m_reg.len; i++) {
        m_pkt.put8(data_tmp[i]);
      }
      break;
    }
    case W_TORQUE:
      m_pkt.start(id(), DXL2_INST_WRITE);
      m_pkt.put16(DXL2_REG_TORQUE_ENABLE);
      m_pkt.put8(1);
      break;
    case W_GOAL:
      m_pkt.start(id(), DXL2_INST_WRITE);
      m_pkt.put16(DXL2_REG_GOAL_VELOCITY);
      m_pkt.put32(uint32_t(int32_t(lroundf(mrd_wheel_rpm(m_val) * m_ln->cw[m_ix] / DXL2_VEL_RPM))));
      break;
    case W_READ:
      m_pkt.start(id(), DXL2_INST_READ);
      m_pkt.put16(DXL2_REG_PRESENT_VELOCITY);
      m_pkt.put16(4);
      rx_len_tmp += 4;
      break;
    default:
      return false;
    }
    if (m_pkt.finish() == 0) {
      finish(MRD_PCMD_NA);
      return false;
    }
    mrd_servo_xfer(a_x, m_ln->bus, m_pkt.buf, m_pkt.len, m_rx, rx_len_tmp);
    return true;
  }

  void complete(const MrdHduartXfer &a_x) {
    uint8_t err_tmp = 0;
    const bool ok_tmp = a_x.ok && mrd_dxl2_status_check_err(m_rx, a_x.rx_len, id(), err_tmp);
    if (!ok_tmp || err_tmp != 0) {
      if (m_step >= W_TORQUE) {
        m_ln->st->torque[m_ix] = -1;
      }
      finish(!ok_tmp ? MRD_PCMD_NO_REPLY : MRD_PCMD_SERVO_ERR);
      return;
    }
    switch (m_step) {
    case PING:
      finish(1);
      break;
    case READ:
      finish(to_val(mrd_pc_decode(mrd_pc_le_get(&m_rx[9], m_reg.len), m_reg.len, m_reg.sign)));
      break;
    case WRITE:
      finish(m_val);
      break;
    case W_TORQUE:
      m_ln->st->torque[m_ix] = 1;
      m_step = W_GOAL;
      break;
    case W_GOAL:
      m_step = W_READ;
      break;
    case W_READ:
      finish(mrd_wheel_val(mrd_dxl2_i32_get(&m_rx[9]) * DXL2_VEL_RPM * m_ln->cw[m_ix]));
      break;
    default:
      break;
    }
  }

private:
  enum Step : uint8_t { PING = 1,
                        READ,
                        WRITE,
                        W_TORQUE,
                        W_GOAL,
                        W_READ };
  static constexpr float DXL2_VEL_RPM = 0.229f; // 速度の単位(rpm)

  Dxl2Packet m_pkt;
  uint8_t m_rx[DXL2_STATUS_MIN + 4];
};

/// @brief DYNAMIXELサーボ1系統分の1フレームの処理を, 1回ずつの送受信に分けて進める.
///        コマンド1のサーボは目標位置へ動かし, それ以外は脱力する. いずれも現在位置を読み取ってtgtに格納する.
///        [1] 目標位置 → [2] トルク → [3] SyncReadで現在位置 → [4] 前フレームで読めなかったサーボを1台だけ個別READ.
class Dxl2LineStepper : public MrdParamLineStepper<Dxl2ParamOp> {
public:
  /// @param a_line 系統の参照.
  /// @param a_all_mount パラメータ系コマンドのスロットを除かないマウント状態(パラメータ系コマンドの実行用).
  /// @param a_num 処理するサーボ数.
  /// @param a_baud 通信速度bps(SyncReadの受信待ち時間の計算用).
  Dxl2LineStepper(const MrdServoLineRef &a_line, const int *a_all_mount, int a_num, unsigned long a_baud)
      : MrdParamLineStepper<Dxl2ParamOp>(a_line, a_all_mount, a_num),
        m_baud(max(a_baud, 9600UL)) {}

  void skip_reads() override { m_skip_reads = true; }

protected:
  bool next_main(MrdHduartXfer &a_x) override {
    while (m_phase != DONE) {
      size_t len_tmp = 0;
      size_t rx_len_tmp = 0;
      unsigned long timeout_tmp = 0;
      const Phase cur_tmp = m_phase;
      switch (m_phase) {
      case GOAL:
        len_tmp = build_goal();
        m_phase = TORQUE;
        break;
      case TORQUE:
        len_tmp = build_torque();
        m_phase = SYNC_READ;
        break;
      case SYNC_READ:
        if (m_skip_reads) { // このフレームは現在位置を読まない(全サーボ前回の値. 目標位置を送った後に行う)
          mrd_servo_past_keep_all(m_ln, m_num);
          m_phase = DONE;
          break;
        }
        len_tmp = build_sync_read();
        rx_len_tmp = m_sync_num * DXL2_POS_REPLY;
        timeout_tmp = sync_timeout_ms(rx_len_tmp);
        m_phase = RETRY;
        break;
      case RETRY:
        len_tmp = build_retry();
        rx_len_tmp = DXL2_POS_REPLY;
        m_phase = DONE;
        break;
      default:
        break;
      }
      if (len_tmp > 0) {
        m_sent_phase = cur_tmp;
        mrd_servo_xfer(a_x, m_ln.bus, m_pkt.buf, len_tmp, m_rx, rx_len_tmp, timeout_tmp);
        return true;
      }
    }
    return false;
  }

  void complete_main(const MrdHduartXfer &a_x) override {
    if (m_sent_phase == SYNC_READ) {
      complete_sync_read(a_x.rx_got);
    } else if (m_sent_phase == RETRY) {
      complete_one(m_retry_ix, a_x.ok && mrd_dxl2_status_check(m_rx, DXL2_POS_REPLY, m_ln.id[m_retry_ix]), m_rx);
    }
  }

private:
  enum Phase : uint8_t { GOAL,
                         TORQUE,
                         SYNC_READ,
                         RETRY,
                         DONE };

  /// @brief 目標位置のSyncWriteを作る(トルクONのサーボのみ).
  size_t build_goal() {
    m_pkt.start(DXL2_ID_BROADCAST, DXL2_INST_SYNC_WRITE);
    m_pkt.put16(DXL2_REG_GOAL_POSITION);
    m_pkt.put16(4);
    int count_tmp = 0;
    for (int i = 0; i < m_num; i++) {
      if (!m_ln.mount[i] || !m_ln.on(i)) {
        continue;
      }
      m_pkt.put8(m_ln.id[i]);
      m_pkt.put32(mrd_dxl2_deg2pos(m_ln.tgt[i], m_ln.trim[i], m_ln.cw[i]));
      count_tmp++;
    }
    return count_tmp > 0 ? m_pkt.finish() : 0;
  }

  /// @brief トルクのSyncWriteを作る. ON/OFFが変わったサーボと, 定期的な送り直しのフレームでは全サーボ.
  ///        ONは目標位置を送った後に行い, 古い目標位置へ動くのを防ぐ.
  size_t build_torque() {
    m_pkt.start(DXL2_ID_BROADCAST, DXL2_INST_SYNC_WRITE);
    m_pkt.put16(DXL2_REG_TORQUE_ENABLE);
    m_pkt.put16(1);
    const int count_tmp = mrd_servo_torque_each(m_ln, m_num, [this](int a_ix, int8_t a_want) {
      m_pkt.put8(m_ln.id[a_ix]);
      m_pkt.put8(a_want);
      return true;
    });
    return count_tmp > 0 ? m_pkt.finish() : 0;
  }

  /// @brief 前フレームで読めたサーボ(err==0)のSyncReadを作る.
  ///        読めなかったサーボはこのフレームの値を前回の値にしておく([4]で読めたものは上書きされる).
  size_t build_sync_read() {
    m_pkt.start(DXL2_ID_BROADCAST, DXL2_INST_SYNC_READ);
    m_pkt.put16(DXL2_REG_PRESENT_POSITION);
    m_pkt.put16(4);
    m_sync_num = 0;
    for (int i = 0; i < m_num; i++) {
      m_excluded[i] = false;
      if (!m_ln.mount[i]) {
        continue;
      }
      m_excluded[i] = (m_ln.err[i] != 0);
      if (m_excluded[i]) {
        mrd_servo_past_keep(m_ln, i);
        continue;
      }
      m_pkt.put8(m_ln.id[i]);
      m_sync_ix[m_sync_num++] = i;
    }
    return m_sync_num > 0 ? m_pkt.finish() : 0;
  }

  /// @brief SyncReadの応答を待つ時間(ms). 全応答の転送時間にバスの既定のタイムアウトを加える.
  unsigned long sync_timeout_ms(size_t a_rx_len) const {
    return (a_rx_len * 10UL * 1000UL + m_baud - 1) / m_baud + m_ln.bus->timeout_ms();
  }

  /// @brief SyncReadの応答を解析する. 応答はID順に続くが, 応答しないサーボがあっても対応を誤らないよう,
  ///        ヘッダとCRCが正しいパケットをIDで照合する.
  ///        見つからなかったサーボのうち, 最後に受信できたサーボより前のものは失敗とする.
  ///        それより後ろは先頭の1台だけを失敗とし, 残りは前回の値のまま次のフレームで読み直す
  ///        (先頭の1台が応答しないために後続が応答できなかった可能性があるため. 先頭の1台は次のフレームで
  ///         SyncReadから外れるので, 後続は次のフレームで読める).
  /// @param a_got 受信できたバイト数.
  void complete_sync_read(size_t a_got) {
    bool done_tmp[MRD_SERVO_SLOTS] = {false};
    size_t pos_tmp = 0;
    while (pos_tmp + DXL2_POS_REPLY <= a_got) {
      const uint8_t *p_tmp = &m_rx[pos_tmp];
      const int k_tmp = find_sync(p_tmp[4]);
      if (k_tmp < 0 || done_tmp[k_tmp] || !mrd_dxl2_status_check(p_tmp, DXL2_POS_REPLY, p_tmp[4])) {
        pos_tmp++; // 同期が外れている場合は1バイトずつずらしてヘッダを探す
        continue;
      }
      complete_one(m_sync_ix[k_tmp], true, p_tmp);
      done_tmp[k_tmp] = true;
      pos_tmp += DXL2_POS_REPLY;
    }
    int last_done_tmp = -1;
    for (int k = 0; k < m_sync_num; k++) {
      if (done_tmp[k]) {
        last_done_tmp = k;
      }
    }
    bool blamed_tmp = false; // 最後に受信できたサーボより後ろで, 失敗とした1台
    for (int k = 0; k < m_sync_num; k++) {
      if (done_tmp[k]) {
        continue;
      }
      if (k < last_done_tmp || !blamed_tmp) {
        complete_one(m_sync_ix[k], false, nullptr);
        blamed_tmp = blamed_tmp || (k > last_done_tmp);
      } else {
        mrd_servo_past_keep(m_ln, m_sync_ix[k]);
      }
    }
  }

  /// @brief SyncReadの対象からIDを探す.
  /// @return 対象内の番号. 見つからなければ-1.
  int find_sync(uint8_t a_id) const {
    for (int k = 0; k < m_sync_num; k++) {
      if (m_ln.id[m_sync_ix[k]] == a_id) {
        return k;
      }
    }
    return -1;
  }

  /// @brief SyncReadから外したサーボを1台だけ個別のREADで確認する(順番に1フレーム1台).
  ///        このフレームのSyncReadで失敗したサーボは対象にしない(同じフレームで二重に数えないため).
  size_t build_retry() {
    for (int n = 0; n < m_num; n++) {
      const int ix_tmp = (m_ln.st->read_cursor + n) % m_num;
      if (m_excluded[ix_tmp]) {
        m_ln.st->read_cursor = (ix_tmp + 1) % m_num;
        m_retry_ix = ix_tmp;
        m_pkt.start(m_ln.id[ix_tmp], DXL2_INST_READ);
        m_pkt.put16(DXL2_REG_PRESENT_POSITION);
        m_pkt.put16(4);
        return m_pkt.finish();
      }
    }
    return 0;
  }

  /// @brief 1台分の現在位置の応答を反映する.
  void complete_one(int a_ix, bool a_ok, const uint8_t *a_reply) {
    mrd_servo_read_apply(m_ln, a_ix, a_ok,
                         a_ok ? mrd_dxl2_pos2deg(mrd_dxl2_i32_get(&a_reply[9]), m_ln.trim[a_ix], m_ln.cw[a_ix]) : 0.0f);
  }

  unsigned long m_baud;
  Phase m_phase = GOAL;
  bool m_skip_reads = false;                      // このフレームは現在位置を読まないか(1系統に2種類のサーボを混在させた場合)
  Phase m_sent_phase = GOAL;                      // 直前に送った処理
  int m_sync_ix[MRD_SERVO_SLOTS];                 // SyncReadの対象のサーボのインデックス(ID順に応答が並ぶ)
  int m_sync_num = 0;                             // SyncReadの対象数
  bool m_excluded[MRD_SERVO_SLOTS] = {false};     // 前フレームで読めずSyncReadから外したサーボ
  int m_retry_ix = -1;                            // 個別READの対象のサーボのインデックス
  Dxl2Packet m_pkt;                               // 送信パケット
  uint8_t m_rx[MRD_SERVO_SLOTS * DXL2_POS_REPLY]; // 応答(SyncReadは全サーボ分が続けて届く)
};

#endif // __MERIDIAN_SERVO_DYNAMIXEL2_H__
