#ifndef __MERIDIAN_SERVO_DYNAMIXEL1_H__
#define __MERIDIAN_SERVO_DYNAMIXEL1_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバスと共通処理. 本ファイルは mrd_servo.h から読み込まれる
#include "sv_ftc.h"    // パケットの生成(FEETECHと同じ形式)
#include "sv_util.h"   // 各プロトコル層の共通部品

//==================================================================================================
//  DYNAMIXEL Protocol 1.0 プロトコル層 (半二重UARTバス上で動作)
//==================================================================================================
//
// 対象: AX(AX-12A/12W/18A), RX(RX-24F/28/64), EX-106+, MX(MX-12W/28/64/106, Protocol 1.0ファームウェア).
//       機種はモデル番号(アドレス0)で判別する. 位置値: AX/RX 0-1023=300deg(中央512), EX 0-4095=251deg,
//       MX 0-4095=360deg(中央2048). (ROBOTIS e-Manual)
// パケットはFEETECHと同じ形式(FF FF ID LEN INST PARAM CHK, CHK=~合計)のため, sv_ftc.h の生成・検証を使う.
//
// 1フレームの処理:
//   [1] 目標位置: SyncWrite(アドレス30, 2バイト). トルクONのサーボのみ.
//   [2] トルク: ON/OFFが変わったサーボだけSyncWrite(アドレス24). MRD_SERVO_TORQUE_REFRESH_FRAMES ごとに全サーボ.
//   [3] 現在位置: 各サーボにREAD(アドレス36, 2バイト). (SyncReadはなく, BulkReadはMXのみのため1台ずつ)
//   機種が分からないサーボ(起動時に応答しなかった)は, 位置の尺度が決まらないため目標位置・トルクを送らず,
//   [3] で現在位置の代わりに機種番号(アドレス0)を読む. 読めた次のフレームから通常どおり動かす.
//   移動速度(アドレス32)は書き換えない(関節モードでは0が最大速度).
//
// 起動時: 応答遅延時間(アドレス5, 初期値250=500us)が0でなければ0にする(SERVO_DXL_FIX_RETURN_DELAY).

#define DXL1_REG_MODEL_NUMBER     0
#define DXL1_REG_RETURN_DELAY     5
#define DXL1_REG_TORQUE_ENABLE    24
#define DXL1_REG_GOAL_POSITION    30
#define DXL1_REG_MOVING_SPEED     32
#define DXL1_REG_PRESENT_POSITION 36
#define DXL1_REG_PRESENT_SPEED    38

/// @brief モデル番号から, 位置値の分解能・角度範囲・速度の単位を求める.
struct Dxl1Model {
  float res;   // 位置値の段数
  float range; // 位置値の全範囲の角度(deg)
  float rpm;   // 速度の単位(rpm)
  bool mx;     // MXシリーズ(PIDゲイン, 電流)
};

inline Dxl1Model mrd_dxl1_model(uint16_t a_model) {
  switch (a_model) {
  case 29:  // MX-28
  case 310: // MX-64
  case 320: // MX-106
    return {4096.0f, 360.0f, 0.114f, true};
  case 360: // MX-12W
    return {4096.0f, 360.0f, 0.916f, true};
  case 107: // EX-106+
    return {4096.0f, 250.92f, 0.111f, false};
  default: // AX, RX, 不明
    return {1024.0f, 300.0f, 0.111f, false};
  }
}

/// @brief 機種の位置値の尺度. 中央(AX/RX:512, MX:2048)が0deg.
inline MrdPosScale mrd_dxl1_scale(const Dxl1Model &a_m) { return {a_m.res / a_m.range, a_m.res * 0.5f, 0.0f, a_m.res - 1.0f}; }

inline int mrd_dxl1_deg2pos(float a_degree, float a_trim, int a_cw, const Dxl1Model &a_m) {
  return mrd_servo_scale_deg2pos(a_degree, a_trim, a_cw, mrd_dxl1_scale(a_m));
}

/// @brief 位置値を角度(degree)に変換する.
inline float mrd_dxl1_pos2deg(int a_pos, float a_trim, int a_cw, const Dxl1Model &a_m) {
  return mrd_servo_scale_pos2deg(a_pos, a_trim, a_cw, mrd_dxl1_scale(a_m));
}

//------------------------------------------------------------------------------------
//  DYNAMIXEL Protocol 1.0 サーボ1系統分のクラス
//------------------------------------------------------------------------------------

class Dynamixel1HalfDuplexClass {
public:
  /// @param a_bus 使用する半二重UARTバス.
  /// @param a_baud 通信速度bps(サーボの設定に合わせる. AXの初期値は1M, MX/RXは57600).
  Dynamixel1HalfDuplexClass(MrdHalfDuplexUart &a_bus, unsigned long a_baud) : m_bus(a_bus), m_baud(a_baud) {}

  /// @brief 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ.
  void set_baud(unsigned long a_baud) { m_baud = a_baud; }

  /// @brief バスを開始(8N1)し, 接続されたサーボのモデル番号を読み, 応答遅延時間を0にする.
  bool begin(const char *a_line_name, const int *a_mount, const int *a_id, int a_num) {
    m_state.reset();
    memset(m_model, 0, sizeof(m_model));
    if (!m_bus.begin(m_baud, SERIAL_8N1)) {
      return false;
    }
    for (int i = 0; i < a_num && i < MRD_SERVO_SLOTS; i++) {
      if (!a_mount[i]) {
        continue;
      }
      const uint8_t id_tmp = a_id[i];
      mrd_servo_slot_print(a_line_name, "DXL1", i, id_tmp);
      uint8_t data_tmp[2] = {0, 0};
      bool found_tmp = false;
      for (int k = 0; k < MRD_SERVO_BOOT_RETRY && !found_tmp; k++) {
        found_tmp = read(id_tmp, DXL1_REG_MODEL_NUMBER, data_tmp, 2);
      }
      if (!found_tmp) {
        Serial.println("no response");
        continue;
      }
      m_model[i] = data_tmp[0] | (data_tmp[1] << 8);
      Serial.print("model ");
      Serial.print(m_model[i]);
      uint8_t delay_tmp = 0;
      if (read(id_tmp, DXL1_REG_RETURN_DELAY, &delay_tmp, 1)) {
        Serial.print(" return delay ");
        Serial.print(delay_tmp * 2);
        Serial.print("us");
        if (delay_tmp != 0 && cfg.servo_dxl_fix_return_delay) {
          const uint8_t zero_tmp = 0;
          Serial.print(write(id_tmp, DXL1_REG_RETURN_DELAY, &zero_tmp, 1) ? " -> 0us" : " -> fix failed");
        }
      }
      Serial.println();
    }
    return true;
  }

  /// @brief メモリを読み出す(1回の送受信).
  bool read(uint8_t a_id, uint8_t a_addr, uint8_t *a_data, uint8_t a_len) {
    uint8_t tx_tmp[8];
    uint8_t rx_tmp[6 + 16];
    const size_t rx_len_tmp = 6 + a_len;
    if (rx_len_tmp > sizeof(rx_tmp)) {
      return false;
    }
    mrd_ftc_packet_make_read(tx_tmp, a_id, a_addr, a_len);
    if (!m_bus.transfer(tx_tmp, 8, rx_tmp, rx_len_tmp) || !mrd_ftc_reply_check(rx_tmp, rx_len_tmp, a_id)) {
      return false;
    }
    memcpy(a_data, &rx_tmp[5], a_len);
    return true;
  }

  /// @brief メモリに書き込み, 応答を確認する(1回の送受信).
  bool write(uint8_t a_id, uint8_t a_addr, const uint8_t *a_data, uint8_t a_len) {
    uint8_t tx_tmp[7 + 16];
    uint8_t rx_tmp[6];
    if (a_len > 16) {
      return false;
    }
    const size_t len_tmp = mrd_ftc_packet_make_write(tx_tmp, a_id, a_addr, a_data, a_len);
    return m_bus.transfer(tx_tmp, len_tmp, rx_tmp, 6) && mrd_ftc_reply_check(rx_tmp, 6, a_id) && rx_tmp[4] == 0;
  }

  MrdHalfDuplexUart &bus() { return m_bus; }
  unsigned long baud() const { return m_baud; }

  /// @brief フレームをまたいで保持する状態(トルク状態の記憶).
  MrdServoLineState m_state;

  /// @brief 起動時に確認した各スロットのモデル番号(0:応答なし).
  uint16_t m_model[MRD_SERVO_SLOTS] = {0};

private:
  MrdHalfDuplexUart &m_bus;
  unsigned long m_baud;
};

//==================================================================================================
//  DYNAMIXEL Protocol 1.0 サーボの駆動処理
//==================================================================================================

/// @brief 系統のDYNAMIXEL Protocol 1.0 サーボクラス.
inline Dynamixel1HalfDuplexClass &mrd_dxl1_line(int a_line) { return (a_line == L) ? dxl1_l : dxl1_r; }

/// @brief 系統の参照を作る.
MrdServoLineRef mrd_dxl1_line_ref(UartLine a_line, Meridim90Union &a_meridim, ServoParam &a_sv) {
  return mrd_servo_line_ref(a_line, DXL1, mrd_dxl1_line(a_line).bus(), mrd_dxl1_line(a_line).m_state, a_meridim, a_sv);
}

//------------------------------------------------------------------------------------
//  パラメータ系コマンド (番号と値の形式は tasks/servo_param_plan.md)
//------------------------------------------------------------------------------------

/// @brief パラメータ系コマンドの対応表. EEPROMは書いた時点で保存される(35は成功を返す).
MrdParamReg mrd_dxl1_param_reg(uint8_t a_cmd, const Dxl1Model &a_m, uint16_t a_model) {
  const bool cur_tmp = (a_model == 310 || a_model == 320); // MX-64/106 のみ電流(68)がある
  const bool cm_tmp = (!a_m.mx && a_model != 0);           // コンプライアンス(26〜29)がある(機種不明は書かない)
  const float vel_k_tmp = a_m.rpm * 6.0f;                  // rpm → deg/s
  switch (a_cmd) {
  case 4:
  case 5:
    return mrd_pc_reg(DXL1_REG_GOAL_POSITION, 2, a_cmd == 4, PC_POS);
  case 6:
    return mrd_pc_reg(DXL1_REG_PRESENT_POSITION, 2, false, PC_POS);
  case 7:
    return mrd_pc_reg(42, 1, false, PC_SCALE, 10.0f); // 0.1V → V x100
  case 12:
  case 13:
    return mrd_pc_reg(6, 2, a_cmd == 13, PC_POS_NATIVE); // CW角度限界
  case 14:
  case 15:
    return mrd_pc_reg(8, 2, a_cmd == 15, PC_POS_NATIVE); // CCW角度限界
  case 16:
  case 17:
    return a_m.mx ? mrd_pc_reg(28, 1, a_cmd == 17) : MrdParamReg(); // MX: Pゲイン
  case 18:
  case 19:
    return a_m.mx ? mrd_pc_reg(27, 1, a_cmd == 19) : MrdParamReg();
  case 20:
  case 21:
    return a_m.mx ? mrd_pc_reg(26, 1, a_cmd == 21) : MrdParamReg();
  case 22:
  case 23:
    return mrd_pc_reg(11, 1, a_cmd == 23);
  case 26: // 現在速度(bit10が回転方向)
    return mrd_pc_reg(DXL1_REG_PRESENT_SPEED, 2, false, PC_SCALE, vel_k_tmp, PC_BIT10);
  case 27: // 移動速度(関節モード. 0は最大速度)
    return mrd_pc_reg(DXL1_REG_MOVING_SPEED, 2, true, PC_SCALE, vel_k_tmp);
  case 30:
  case 31:
    return mrd_pc_reg(34, 2, a_cmd == 31, PC_SCALE, 0.1f); // トルク制限(約0.1%)
  case 36:
    return mrd_pc_reg(43, 1, false);
  case 37: // 電流(MX-64/106: 4.5mA x (値 - 2048))
    return cur_tmp ? mrd_pc_reg(68, 2, false, PC_SCALE, 4.5f) : MrdParamReg();
  case 38:
    return mrd_pc_reg(DXL1_REG_TORQUE_ENABLE, 1, false, PC_BOOL);
  case 54:
  case 55:
    return mrd_pc_reg(48, 2, a_cmd == 55); // パンチ
  case 56:
  case 57: // AX/RX/EX: CW/CCWコンプライアンスマージン
    return cm_tmp ? mrd_pc_reg(26, 1, a_cmd == 57, PC_RAW, 1.0f, PC_UNSIGNED, false, true) : MrdParamReg();
  case 60:
  case 61: // AX/RX/EX: CW/CCWコンプライアンススロープ
    return cm_tmp ? mrd_pc_reg(28, 1, a_cmd == 61, PC_RAW, 1.0f, PC_UNSIGNED, false, true) : MrdParamReg();
  case 70:
  case 71:
    return mrd_pc_reg(DXL1_REG_RETURN_DELAY, 1, a_cmd == 71);
  default:
    return MrdParamReg();
  }
}

/// @brief DYNAMIXEL Protocol 1.0 のパラメータ系コマンド1件分の処理.
///        3:PING, 35:ROM保存(成功を返す), それ以外はREAD/WRITE 1回. 車輪はトルクON(必要なときだけ) → 移動速度(32) → 現在速度(38).
class Dxl1ParamOp : public MrdParamOpBase {
public:
  void begin(const MrdServoLineRef &a_line, int a_ix, uint8_t a_cmd, int16_t a_val) {
    target(a_line, a_ix, a_val);
    const uint16_t model_tmp = mrd_dxl1_line(a_line.line).m_model[a_ix];
    m_model = mrd_dxl1_model(model_tmp);
    m_scale = mrd_dxl1_scale(m_model);
    if (a_cmd == 3) {
      m_step = PING;
      return;
    }
    if (model_tmp == 0) { // 機種が分からない(まだ応答していない): 値の換算が決まらないため送らない
      finish(MRD_PCMD_NO_REPLY);
      return;
    }
    if (a_cmd == 35) {
      finish(1);
      return;
    }
    m_reg = mrd_dxl1_param_reg(a_cmd, m_model, model_tmp);
    if (m_reg.len == 0) {
      finish(MRD_PCMD_NA);
      return;
    }
    if (!write_value_ok()) { // レジスタに収まらない値は送らない
      finish(MRD_PCMD_NA);
      return;
    }
    if (a_cmd == 37) {
      m_current = true;
    }
    m_step = m_reg.write ? WRITE : READ;
  }

  /// @brief 車輪(車輪モード=CW/CCW角度限界0を事前に設定したサーボ). 速度は0.01rpm(回転方向補正を適用).
  void begin_wheel(const MrdServoLineRef &a_line, int a_ix, int16_t a_speed) {
    target(a_line, a_ix, a_speed);
    const uint16_t model_tmp = mrd_dxl1_line(a_line.line).m_model[a_ix];
    if (model_tmp == 0) { // 機種が分からない: 速度の換算が決まらないため送らない
      finish(MRD_PCMD_NO_REPLY);
      return;
    }
    m_model = mrd_dxl1_model(model_tmp);
    m_step = wheel_needs_torque() ? W_TORQUE : W_GOAL;
  }

  bool next(MrdHduartXfer &a_x) {
    size_t len_tmp = 0;
    size_t rx_len_tmp = 6;
    uint8_t data_tmp[2];
    switch (m_step) {
    case PING:
      len_tmp = mrd_ftc_packet_make_ping(m_tx, id());
      break;
    case READ:
      len_tmp = mrd_ftc_packet_make_read(m_tx, id(), m_reg.addr, m_reg.len);
      rx_len_tmp += m_reg.len;
      break;
    case WRITE: {
      const uint32_t raw_tmp = mrd_pc_encode(to_raw(), m_reg.sign);
      data_tmp[0] = raw_tmp & 0xFF;
      data_tmp[1] = m_reg.pair ? data_tmp[0] : uint8_t(raw_tmp >> 8);
      len_tmp = mrd_ftc_packet_make_write(m_tx, id(), m_reg.addr, data_tmp, m_reg.pair ? 2 : m_reg.len);
      break;
    }
    case W_TORQUE:
      data_tmp[0] = 1;
      len_tmp = mrd_ftc_packet_make_write(m_tx, id(), DXL1_REG_TORQUE_ENABLE, data_tmp, 1);
      break;
    case W_GOAL: {
      const long spd_tmp = constrain(lroundf(mrd_wheel_rpm(m_val) * m_ln->cw[m_ix] / m_model.rpm), -1023L, 1023L);
      const uint16_t raw_tmp = uint16_t(mrd_pc_encode(spd_tmp, PC_BIT10));
      data_tmp[0] = raw_tmp & 0xFF;
      data_tmp[1] = raw_tmp >> 8;
      len_tmp = mrd_ftc_packet_make_write(m_tx, id(), DXL1_REG_MOVING_SPEED, data_tmp, 2);
      break;
    }
    case W_READ:
      len_tmp = mrd_ftc_packet_make_read(m_tx, id(), DXL1_REG_PRESENT_SPEED, 2);
      rx_len_tmp += 2;
      break;
    default:
      return false;
    }
    mrd_servo_xfer(a_x, m_ln->bus, m_tx, len_tmp, m_rx, rx_len_tmp);
    return true;
  }

  void complete(const MrdHduartXfer &a_x) {
    const bool ok_tmp = a_x.ok && mrd_ftc_reply_check(m_rx, a_x.rx_len, id());
    const bool err_tmp = ok_tmp && m_rx[4] != 0;
    if (!ok_tmp || err_tmp) {
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
    case READ: {
      long raw_tmp = mrd_pc_decode(mrd_pc_le_get(&m_rx[5], m_reg.len), m_reg.len, m_reg.sign);
      if (m_current) {
        raw_tmp -= 2048; // 電流: 2048が0
      }
      finish(to_val(raw_tmp));
      break;
    }
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
      finish(mrd_wheel_val(mrd_pc_decode(mrd_pc_le_get(&m_rx[5], 2), 2, PC_BIT10) * m_model.rpm * m_ln->cw[m_ix]));
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

  Dxl1Model m_model = {1024.0f, 300.0f, 0.111f, false};
  bool m_current = false;
  uint8_t m_tx[12];
  uint8_t m_rx[8];
};

/// @brief DYNAMIXEL Protocol 1.0 サーボ1系統分の1フレームの処理を, 1回ずつの送受信に分けて進める.
///        コマンド1のサーボは目標位置へ動かし, それ以外は脱力する. 現在位置を1台ずつ読み取ってtgtに格納する.
///        [1] 目標位置 → [2] トルク → [3] 各サーボの現在位置 の順に送る.
class Dxl1LineStepper : public MrdParamLineStepper<Dxl1ParamOp> {
public:
  Dxl1LineStepper(const MrdServoLineRef &a_line, const int *a_all_mount, int a_num)
      : MrdParamLineStepper<Dxl1ParamOp>(a_line, a_all_mount, a_num) {}

  void skip_reads() override { m_skip_reads = true; }

protected:
  bool next_main(MrdHduartXfer &a_x) override {
    while (m_phase != DONE) {
      size_t len_tmp = 0;
      const uint8_t *tx_tmp = m_pkt.buf;
      m_reading = false;
      switch (m_phase) {
      case GOAL:
        len_tmp = build_goal();
        m_phase = TORQUE;
        break;
      case TORQUE:
        len_tmp = build_torque();
        m_phase = READ;
        if (m_skip_reads) {
          mrd_servo_past_keep_all(m_ln, m_num);
          m_phase = DONE;
        }
        break;
      case READ:
        m_ix = m_ln.next_mounted(m_ix, m_num);
        if (m_ix >= m_num) {
          m_phase = DONE;
          break;
        }
        m_reading_model = (model(m_ix) == 0); // 機種が分からなければ, 現在位置の代わりに機種番号を読む
        len_tmp = mrd_ftc_packet_make_read(m_tx, m_ln.id[m_ix], m_reading_model ? DXL1_REG_MODEL_NUMBER : DXL1_REG_PRESENT_POSITION, 2);
        tx_tmp = m_tx;
        m_reading = true;
        break;
      default:
        break;
      }
      if (len_tmp > 0) {
        mrd_servo_xfer(a_x, m_ln.bus, tx_tmp, len_tmp, m_rx, m_reading ? sizeof(m_rx) : 0);
        return true;
      }
    }
    return false;
  }

  void complete_main(const MrdHduartXfer &a_x) override {
    if (!m_reading) {
      return; // SyncWriteは応答なし
    }
    const bool valid_tmp = a_x.ok && mrd_ftc_reply_check(m_rx, sizeof(m_rx), m_ln.id[m_ix]);
    if (m_reading_model) {
      if (valid_tmp) {
        mrd_dxl1_line(m_ln.line).m_model[m_ix] = m_rx[5] | (m_rx[6] << 8);
        mrd_servo_past_keep(m_ln, m_ix); // 位置はまだ読んでいない
        m_ln.st->torque[m_ix] = -1;      // トルク状態は送っていないため不明にし, 次のフレームで送る
      } else {
        mrd_servo_read_apply(m_ln, m_ix, false, 0.0f);
      }
      return;
    }
    const Dxl1Model m_tmp = mrd_dxl1_model(model(m_ix));
    mrd_servo_read_apply(m_ln, m_ix, valid_tmp,
                         valid_tmp ? mrd_dxl1_pos2deg(m_rx[5] | (m_rx[6] << 8), m_ln.trim[m_ix], m_ln.cw[m_ix], m_tmp) : 0.0f);
  }

private:
  enum Phase : uint8_t { GOAL,
                         TORQUE,
                         READ,
                         DONE };

  /// @brief 目標位置のSyncWriteを作る(トルクONのサーボのみ).
  size_t build_goal() {
    m_pkt.start(DXL1_REG_GOAL_POSITION, 2);
    for (int i = 0; i < m_num; i++) {
      if (!m_ln.mount[i] || !m_ln.on(i) || model(i) == 0) {
        continue;
      }
      uint8_t *slot_tmp = m_pkt.add(m_ln.id[i]);
      if (slot_tmp == nullptr) {
        continue;
      }
      const Dxl1Model m_tmp = mrd_dxl1_model(model(i));
      const uint16_t pos_tmp = uint16_t(mrd_dxl1_deg2pos(m_ln.tgt[i], m_ln.trim[i], m_ln.cw[i], m_tmp));
      slot_tmp[0] = pos_tmp & 0xFF;
      slot_tmp[1] = pos_tmp >> 8;
    }
    return m_pkt.finish();
  }

  /// @brief トルクのSyncWriteを作る. ON/OFFが変わったサーボと, 定期的な送り直しのフレームでは全サーボ.
  size_t build_torque() {
    m_pkt.start(DXL1_REG_TORQUE_ENABLE, 1);
    mrd_servo_torque_each(m_ln, m_num, [this](int a_ix, int8_t a_want) {
      uint8_t *slot_tmp = (model(a_ix) != 0) ? m_pkt.add(m_ln.id[a_ix]) : nullptr; // 機種が分からなければ送らない
      if (slot_tmp != nullptr) {
        *slot_tmp = a_want;
      }
      return slot_tmp != nullptr;
    });
    return m_pkt.finish();
  }

  /// @brief スロットの機種番号(起動時か[3]で読んだもの. 0:分からない).
  uint16_t model(int a_ix) const { return mrd_dxl1_line(m_ln.line).m_model[a_ix]; }

  Phase m_phase = GOAL;
  int m_ix = -1;
  bool m_skip_reads = false; // このフレームは現在位置を読まないか(混在時の交互読み出し, マスターコマンド2)
  bool m_reading = false;
  bool m_reading_model = false; // 直前のREADが機種番号か
  FtcSyncPacket m_pkt;          // SyncWrite(パケット形式はFEETECHと同じ)
  uint8_t m_tx[8];
  uint8_t m_rx[8]; // READの応答(位置2バイト)
};

#endif // __MERIDIAN_SERVO_DYNAMIXEL1_H__
