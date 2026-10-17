#ifndef __MERIDIAN_SERVO_KONDO_PMX_H__
#define __MERIDIAN_SERVO_KONDO_PMX_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバスと共通処理. 本ファイルは mrd_servo.h から読み込まれる
#include "sv_util.h"   // 各プロトコル層の共通部品

//==================================================================================================
//  KONDO PMX プロトコル層 (半二重UARTバス上で動作)
//==================================================================================================
//
// 対象: PMX-SCR-5203HV, 5204HV, 9203HV, 9204HV 等(PMXサーボシリーズ オンラインマニュアル Ver.1.2.0).
//
// 信号はRS-485(差動, 半二重)のため, 基板のサーボ端子とは送受信を自動で切り替えるRS-485変換を介してつなぐ.
// ファームの送受信切替はTTLのサーボと同じ. 同じ信号線にTTLのサーボは載らないため, 他の種類とは混在させない.
//
// 起動時の設定(RAMのみ. 電源を切ると戻る):
//   制御モード(501) = 位置制御, 応答データ(502) = 現在位置. MotorWRITEとMotorREADの返信に現在位置が入る.
//
// 1フレームの処理(サーボごとに1〜2回の送受信):
//   [1] トルク: ON/OFFが変わったサーボ(返信のトルク状態が指示と違う場合を含む)だけ MotorWRITE(TorqueON/Free).
//       ※Free中に受けた目標位置はモードエラーで無視されるため, 目標位置より先に送る.
//   [2] トルクONのサーボは MotorWRITE(目標位置), OFFのサーボは MotorREAD.
//       どちらも返信に現在位置が入るので, 全サーボの現在位置を毎フレーム読む([1]で脱力したサーボは[1]の返信を使う).
//   同期書き込みの命令がない(ブロードキャストは返信なし)ため, 目標位置はサーボごとに送る.
//
// 通信時間: 3Mbps, 1台あたり 送信10byte + 返信11byte = 約77us + サーボの応答時間(設定値 + 30us).
//   応答時間が出荷時の値(例: 200us)なら1台約0.31ms, 15台で約4.6ms.
//
// 位置値は0.01deg単位の符号付き値(位置制御モード)で, 中央が0. 可動範囲は±320deg(サーボの角度制限に従う).

// パケット
#define PMX_HDR             0xFE // ヘッダ(2バイト)
#define PMX_CMD_MEM_READ    0xA0 // RAMの読み出し
#define PMX_CMD_MEM_WRITE   0xA1 // RAMへの書き込み
#define PMX_CMD_SAVE        0xA3 // RAMの内容をROMに保存(Freeのときのみ)
#define PMX_CMD_MOTOR_READ  0xA4 // トルク状態と応答データの読み出し
#define PMX_CMD_MOTOR_WRITE 0xA5 // 目標値の指示, トルクON/OFF
#define PMX_CMD_SYSTEM_READ 0xBB // シリアル番号, 型番, バージョン, 応答時間の読み出し
#define PMX_REPLY_MASK      0x7F // 返信のコマンド = 送信のコマンド & 0x7F

#define PMX_OPT_MOTION      0x00 // MotorWRITE: 目標値
#define PMX_TORQUE_ON       0x01 // MotorWRITE: トルクON / 返信のトルク状態
#define PMX_TORQUE_FREE     0x02 // MotorWRITE: 脱力 / 返信のトルク状態
#define PMX_MEM_WRITE_FORCE 0x01 // MemWRITE: トルクONでも書き込む

// RAM
#define PMX_REG_CONTROL_MODE 501  // 制御モード(1バイト)
#define PMX_REG_RECEIVE_DATA 502  // 応答データの選択(1バイト)
#define PMX_MODE_POSITION    0x01 // 制御モード: 位置制御
#define PMX_MODE_SPEED       0x02 // 制御モード: 速度制御(車輪)
#define PMX_RECV_POSITION    0x01 // 応答データ: 現在位置
#define PMX_RECV_SPEED       0x02 // 応答データ: 現在速度(0.1deg/s)

#define PMX_ID_MAX       239
#define PMX_ACK_LEN      8  // MemWRITEの返信(ステータスのみ)
#define PMX_MOTOR_REPLY  11 // MotorREAD/MotorWRITEの返信(ステータス, トルク状態, 現在位置)
#define PMX_SYSTEM_REPLY 21 // SystemREADの返信
#define PMX_POS_PER_DEG  100.0f
#define PMX_POS_LIMIT    32000.0f

//------------------------------------------------------------------------------------
//  パケットの生成と検証
//------------------------------------------------------------------------------------
//
// 送信: FE FE ID LEN CMD OPT [DATA...] CRC_L CRC_H   (LEN = パケット全体の長さ)
// 返信: FE FE ID LEN CMD&0x7F STATUS [DATA...] CRC_L CRC_H
// CRC = CRC-16-CCITT(x^16+x^12+x^5+1, 初期値0, 左送り, 出力反転なし). ヘッダからDATAの末尾まで.
// 2バイト以上の値はリトルエンディアン.

/// @brief CRC16を計算する.
inline uint16_t mrd_pmx_crc16(const uint8_t *a_data, size_t a_len) { return mrd_servo_crc16(a_data, a_len, 0x1021); }

/// @brief パケットを作る.
/// @param a_buf 出力先(8 + データ数).
/// @param a_id サーボのID.
/// @param a_cmd コマンド.
/// @param a_opt オプション.
/// @param a_data データ(なければnullptr).
/// @param a_data_len データ数.
/// @return パケット長.
size_t mrd_pmx_packet_make(uint8_t *a_buf, uint8_t a_id, uint8_t a_cmd, uint8_t a_opt, const uint8_t *a_data,
                           size_t a_data_len) {
  const size_t len_tmp = 8 + a_data_len;
  a_buf[0] = PMX_HDR;
  a_buf[1] = PMX_HDR;
  a_buf[2] = a_id;
  a_buf[3] = uint8_t(len_tmp);
  a_buf[4] = a_cmd;
  a_buf[5] = a_opt;
  if (a_data_len > 0) {
    memcpy(&a_buf[6], a_data, a_data_len);
  }
  const uint16_t crc_tmp = mrd_pmx_crc16(a_buf, len_tmp - 2);
  a_buf[len_tmp - 2] = crc_tmp & 0xFF;
  a_buf[len_tmp - 1] = crc_tmp >> 8;
  return len_tmp;
}

/// @brief 返信パケットのヘッダ, ID, 長さ, コマンド, CRCを検証する.
/// @param a_rx 受信データ.
/// @param a_rx_len 受信データ数.
/// @param a_id 期待するID.
/// @param a_cmd 送信したコマンド.
inline bool mrd_pmx_reply_check(const uint8_t *a_rx, size_t a_rx_len, uint8_t a_id, uint8_t a_cmd) {
  if (a_rx_len < 8 || a_rx[0] != PMX_HDR || a_rx[1] != PMX_HDR || a_rx[2] != a_id || a_rx[3] != a_rx_len ||
      a_rx[4] != (a_cmd & PMX_REPLY_MASK)) {
    return false;
  }
  const uint16_t crc_tmp = mrd_pmx_crc16(a_rx, a_rx_len - 2);
  return a_rx[a_rx_len - 2] == (crc_tmp & 0xFF) && a_rx[a_rx_len - 1] == (crc_tmp >> 8);
}

/// @brief 位置値の尺度: 0.01deg単位, 中央0, ±320deg.
static const MrdPosScale PMX_POS_SCALE = {PMX_POS_PER_DEG, 0.0f, -PMX_POS_LIMIT, PMX_POS_LIMIT};

/// @brief 角度(degree)をPMXの位置値(0.01deg単位, ±32000)に変換する.
inline int mrd_pmx_deg2pos(float a_degree, float a_trim, int a_cw) {
  return mrd_servo_scale_deg2pos(a_degree, a_trim, a_cw, PMX_POS_SCALE);
}

/// @brief PMXの位置値を角度(degree)に変換する.
inline float mrd_pmx_pos2deg(int a_pos, float a_trim, int a_cw) {
  return mrd_servo_scale_pos2deg(a_pos, a_trim, a_cw, PMX_POS_SCALE);
}

//------------------------------------------------------------------------------------
//  PMXサーボ1系統分のクラス
//------------------------------------------------------------------------------------

class PmxHalfDuplexClass {
public:
  /// @brief コンストラクタ.
  /// @param a_bus 使用する半二重UARTバス.
  /// @param a_baud 通信速度bps(サーボの設定に合わせる. 初期値は115200).
  PmxHalfDuplexClass(MrdHalfDuplexUart &a_bus, unsigned long a_baud) : m_bus(a_bus), m_baud(a_baud) {}

  /// @brief 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ.
  void set_baud(unsigned long a_baud) { m_baud = a_baud; }

  /// @brief バスを開始(8N1)し, 接続されたサーボの応答を確認して制御モードと応答データを設定する.
  /// @param a_line_name 表示用の系統名("L"など).
  /// @param a_mount 各サーボのマウント有無.
  /// @param a_id 各サーボのID.
  /// @param a_num サーボ数.
  /// @return バスを開始できればtrue.
  bool begin(const char *a_line_name, const int *a_mount, const int *a_id, int a_num) {
    m_state.reset();
    memset(m_ctrl_mode, 0, sizeof(m_ctrl_mode));
    if (!m_bus.begin(m_baud, SERIAL_8N1)) {
      return false;
    }
    const unsigned long bauds_tmp[] = {57600, 115200, 625000, 1000000, 1250000, 1500000, 2000000, 3000000};
    mrd_servo_baud_check(a_line_name, "PMX", m_baud, bauds_tmp, sizeof(bauds_tmp) / sizeof(bauds_tmp[0]),
                         " bps is not a PMX baud rate. Check SERVO_BAUDRATE_x (default 115200).");

    for (int i = 0; i < a_num && i < MRD_SERVO_SLOTS; i++) {
      if (!a_mount[i]) {
        continue;
      }
      const int id_tmp = a_id[i];
      mrd_servo_slot_print(a_line_name, "PMX", i, id_tmp);
      if (id_tmp < 0 || id_tmp > PMX_ID_MAX) {
        Serial.println("ID out of range (0-239)");
        continue;
      }

      uint8_t rx_tmp[PMX_SYSTEM_REPLY];
      bool found_tmp = false;
      for (int k = 0; k < MRD_SERVO_BOOT_RETRY && !found_tmp; k++) {
        found_tmp = request(id_tmp, PMX_CMD_SYSTEM_READ, 0, nullptr, 0, rx_tmp, PMX_SYSTEM_REPLY);
      }
      if (!found_tmp) {
        Serial.println("no response");
        continue;
      }
      Serial.print("response time ");
      Serial.print(rx_tmp[18]);
      Serial.print("us, ");

      // 制御モードと応答データ(連続するアドレス)をまとめて書く
      const uint8_t mem_tmp[] = {PMX_REG_CONTROL_MODE & 0xFF, PMX_REG_CONTROL_MODE >> 8, PMX_MODE_POSITION,
                                 PMX_RECV_POSITION};
      uint8_t ack_tmp[PMX_ACK_LEN];
      if (!request(id_tmp, PMX_CMD_MEM_WRITE, PMX_MEM_WRITE_FORCE, mem_tmp, sizeof(mem_tmp), ack_tmp, PMX_ACK_LEN)) {
        Serial.println("setup failed (no reply)");
      } else if (ack_tmp[5] != 0) {
        Serial.print("setup failed (status 0x");
        Serial.print(ack_tmp[5], HEX);
        Serial.println(")");
      } else {
        Serial.println("position mode");
        m_ctrl_mode[i] = PMX_MODE_POSITION;
      }
    }
    return true;
  }

  /// @brief 1回の送受信を行う.
  /// @return 正しい返信を受信できればtrue.
  bool request(uint8_t a_id, uint8_t a_cmd, uint8_t a_opt, const uint8_t *a_data, size_t a_data_len, uint8_t *a_rx,
               size_t a_rx_len) {
    uint8_t tx_tmp[8 + 8];
    if (a_data_len > sizeof(tx_tmp) - 8) {
      return false;
    }
    const size_t len_tmp = mrd_pmx_packet_make(tx_tmp, a_id, a_cmd, a_opt, a_data, a_data_len);
    memset(a_rx, 0xFF, a_rx_len);
    return m_bus.transfer(tx_tmp, len_tmp, a_rx, a_rx_len) && mrd_pmx_reply_check(a_rx, a_rx_len, a_id, a_cmd);
  }

  /// @brief 半二重UARTバス(並行送受信用).
  MrdHalfDuplexUart &bus() { return m_bus; }

  /// @brief 通信速度bps.
  unsigned long baud() const { return m_baud; }

  /// @brief フレームをまたいで保持する状態(トルク状態の記憶).
  MrdServoLineState m_state;

  /// @brief 各スロットのサーボに設定した制御モード(0:不明, 1:位置, 2:速度=車輪). RAMのため起動時は位置にする.
  uint8_t m_ctrl_mode[MRD_SERVO_SLOTS] = {0};

private:
  MrdHalfDuplexUart &m_bus;
  unsigned long m_baud;
};

//==================================================================================================
//  PMXサーボの駆動処理
//==================================================================================================

/// @brief 系統のPMXサーボクラス.
inline PmxHalfDuplexClass &mrd_pmx_line(int a_line) { return (a_line == L) ? pmx_l : pmx_r; }

/// @brief 系統の参照を作る.
/// @param a_line 系統(L or R).
/// @param a_meridim Meridimデータの参照.
/// @param a_sv サーボパラメータ.
MrdServoLineRef mrd_pmx_line_ref(UartLine a_line, Meridim90Union &a_meridim, ServoParam &a_sv) {
  return mrd_servo_line_ref(a_line, KOPMX, mrd_pmx_line(a_line).bus(), mrd_pmx_line(a_line).m_state, a_meridim, a_sv);
}

//------------------------------------------------------------------------------------
//  パラメータ系コマンド (番号と値の形式は tasks/servo_param_plan.md)
//------------------------------------------------------------------------------------

#define PMX_SAVE_TIMEOUT_MS 40 // SAVEの返信待ち(応答時間 + 15ms)

/// @brief 制御モードと応答データ(501, 502)を書き込むパケットを作る(トルクONでも書けるオプション1).
/// @param a_speed trueなら速度制御・応答は現在速度(車輪), falseなら位置制御・応答は現在位置.
/// @return パケット長.
inline size_t mrd_pmx_packet_make_mode(uint8_t *a_buf, uint8_t a_id, bool a_speed) {
  const uint8_t data_tmp[] = {PMX_REG_CONTROL_MODE & 0xFF, PMX_REG_CONTROL_MODE >> 8,
                              uint8_t(a_speed ? PMX_MODE_SPEED : PMX_MODE_POSITION),
                              uint8_t(a_speed ? PMX_RECV_SPEED : PMX_RECV_POSITION)};
  return mrd_pmx_packet_make(a_buf, a_id, PMX_CMD_MEM_WRITE, PMX_MEM_WRITE_FORCE, data_tmp, sizeof(data_tmp));
}

/// @brief パラメータ系コマンドの対応表. 書き込みはRAMのみ(保存は35: SAVE).
///        書き込みはトルクONでも書けるオプション1で送る(オフセットはトルクOFFでのみ書けるためオプション0).
/// @param a_cmd コマンド.
/// @return レジスタの定義(len=0:未対応).
MrdParamReg mrd_pmx_param_reg(uint8_t a_cmd) {
  switch (a_cmd) {
  case 5:
    return mrd_pc_reg(506, 2, false, PC_POS, 1.0f, PC_SIGNED);
  case 6:
    return mrd_pc_reg(300, 2, false, PC_POS, 1.0f, PC_SIGNED);
  case 7:
    return mrd_pc_reg(314, 2, false, PC_SCALE, 0.1f); // mV → V x100
  case 8:
  case 9:
    return mrd_pc_reg(72, 2, a_cmd == 9, PC_RAW, 1.0f, PC_SIGNED); // 中央値オフセット(0.01deg)
  case 10:
  case 11:
    return mrd_pc_reg(64, 2, a_cmd == 11, PC_RAW, 1.0f, PC_SIGNED); // 位置制御の不感帯(0.01deg)
  case 12:
  case 13:
    return mrd_pc_reg(96, 2, a_cmd == 13, PC_POS_NATIVE, 1.0f, PC_SIGNED);
  case 14:
  case 15:
    return mrd_pc_reg(100, 2, a_cmd == 15, PC_POS_NATIVE, 1.0f, PC_SIGNED);
  case 16:
  case 17:
    return mrd_pc_reg(0, 4, a_cmd == 17);
  case 18:
  case 19:
    return mrd_pc_reg(4, 4, a_cmd == 19);
  case 20:
  case 21:
    return mrd_pc_reg(8, 4, a_cmd == 21);
  case 22:
  case 23:
    return mrd_pc_reg(88, 2, a_cmd == 23, PC_SCALE, 0.1f, PC_SIGNED); // モータ温度上限(0.1℃)
  case 24:
  case 25:
    return mrd_pc_reg(84, 2, a_cmd == 25); // mA
  case 26:
    return mrd_pc_reg(302, 2, false, PC_SCALE, 0.1f, PC_SIGNED); // 現在速度(0.1deg/s)
  case 27:
    return mrd_pc_reg(104, 2, true, PC_SCALE, 0.1f, PC_SIGNED); // 最大速度指令値
  case 30:
  case 31:
    return mrd_pc_reg(110, 2, a_cmd == 31); // モータ出力制限(%)
  case 36:
    return mrd_pc_reg(310, 2, false, PC_SCALE, 0.1f, PC_SIGNED);
  case 37:
    return mrd_pc_reg(304, 2, false, PC_RAW, 1.0f, PC_SIGNED);
  case 38:
    return mrd_pc_reg(500, 1, false, PC_BOOL);
  case 50:
  case 51:
    return mrd_pc_reg(12, 4, a_cmd == 51); // 位置制御のストレッチ
  case 56:
  case 57:
    return mrd_pc_reg(64, 2, a_cmd == 57, PC_RAW, 1.0f, PC_SIGNED);
  default:
    return MrdParamReg();
  }
}

/// @brief PMXのパラメータ系コマンド1件分の処理(1回の送受信).
///        3:SystemREAD, 4:目標位置(MotorWRITE. Free中はモードエラー), 35:SAVE, それ以外はMemREAD/MemWRITE.
///        書き込みの結果は受け取ったVAL(ステータスにエラーがあればエラー値).
class PmxParamOp : public MrdParamOpBase {
public:
  void begin(const MrdServoLineRef &a_line, int a_ix, uint8_t a_cmd, int16_t a_val) {
    target(a_line, a_ix, a_val);
    m_scale = PMX_POS_SCALE;
    if (a_line.id[a_ix] < 0 || a_line.id[a_ix] > PMX_ID_MAX) {
      finish(MRD_PCMD_NO_REPLY);
      return;
    }
    if (a_cmd == 3) {
      m_step = SYS_READ;
      return;
    }
    if (a_cmd == 4) {
      m_step = GOAL;
      return;
    }
    if (a_cmd == 35) {
      m_step = SAVE;
      return;
    }
    m_reg = mrd_pmx_param_reg(a_cmd);
    if (m_reg.len == 0) {
      finish(MRD_PCMD_NA);
      return;
    }
    if (!write_value_ok()) { // レジスタに収まらない値は送らない
      finish(MRD_PCMD_NA);
      return;
    }
    m_step = m_reg.write ? MEM_WRITE : MEM_READ;
  }

  /// @brief 車輪: 制御モードを速度にし(RAM. 変わったときだけ), トルクONにして(必要なときだけ), 速度を送る.
  ///        速度は0.01rpm. 返信の現在速度(0.1deg/s)を結果(0.01rpm, 回転方向補正を適用)にする.
  void begin_wheel(const MrdServoLineRef &a_line, int a_ix, int16_t a_speed) {
    target(a_line, a_ix, a_speed);
    if (a_line.id[a_ix] < 0 || a_line.id[a_ix] > PMX_ID_MAX) {
      finish(MRD_PCMD_NO_REPLY);
      return;
    }
    m_step = (mrd_pmx_line(a_line.line).m_ctrl_mode[a_ix] != PMX_MODE_SPEED) ? W_MODE
             : (a_line.st->torque[a_ix] != 1)                                ? W_TORQUE
                                                                             : W_SPEED;
  }

  bool next(MrdHduartXfer &a_x) {
    size_t len_tmp = 0;
    unsigned long timeout_tmp = 0; // 0: 系統の設定値
    uint8_t data_tmp[6] = {uint8_t(m_reg.addr & 0xFF), uint8_t(m_reg.addr >> 8)};
    switch (m_step) {
    case SYS_READ:
      m_rx_len = PMX_SYSTEM_REPLY;
      len_tmp = mrd_pmx_packet_make(m_tx, id(), PMX_CMD_SYSTEM_READ, 0, nullptr, 0);
      break;
    case GOAL: {
      const uint16_t pos_tmp = uint16_t(int16_t(mrd_pmx_deg2pos(m_val * 0.01f, m_ln->trim[m_ix], m_ln->cw[m_ix])));
      data_tmp[0] = pos_tmp & 0xFF;
      data_tmp[1] = pos_tmp >> 8;
      m_rx_len = PMX_MOTOR_REPLY;
      len_tmp = mrd_pmx_packet_make(m_tx, id(), PMX_CMD_MOTOR_WRITE, PMX_OPT_MOTION, data_tmp, 2);
      break;
    }
    case SAVE:
      m_rx_len = PMX_ACK_LEN;
      len_tmp = mrd_pmx_packet_make(m_tx, id(), PMX_CMD_SAVE, 0, nullptr, 0);
      timeout_tmp = PMX_SAVE_TIMEOUT_MS;
      break;
    case MEM_READ:
      data_tmp[2] = m_reg.len;
      m_rx_len = 8 + m_reg.len;
      len_tmp = mrd_pmx_packet_make(m_tx, id(), PMX_CMD_MEM_READ, 0, data_tmp, 3);
      break;
    case MEM_WRITE: {
      mrd_pc_le_put(&data_tmp[2], mrd_pc_encode(to_raw(), m_reg.sign), m_reg.len);
      const uint8_t opt_tmp = (m_reg.addr == 72) ? 0 : PMX_MEM_WRITE_FORCE; // オフセットはトルクOFFでのみ書ける
      m_rx_len = PMX_ACK_LEN;
      len_tmp = mrd_pmx_packet_make(m_tx, id(), PMX_CMD_MEM_WRITE, opt_tmp, data_tmp, 2 + m_reg.len);
      break;
    }
    case W_MODE:
      m_rx_len = PMX_ACK_LEN;
      len_tmp = mrd_pmx_packet_make_mode(m_tx, id(), true);
      break;
    case W_TORQUE:
      m_rx_len = PMX_MOTOR_REPLY;
      len_tmp = mrd_pmx_packet_make(m_tx, id(), PMX_CMD_MOTOR_WRITE, PMX_TORQUE_ON, nullptr, 0);
      break;
    case W_SPEED: {
      const uint16_t spd_tmp = uint16_t(int16_t(constrain(lroundf(mrd_wheel_rpm(m_val) * 60.0f * m_ln->cw[m_ix]), -32768L, 32767L))); // 0.1deg/s
      data_tmp[0] = spd_tmp & 0xFF;
      data_tmp[1] = spd_tmp >> 8;
      m_rx_len = PMX_MOTOR_REPLY;
      len_tmp = mrd_pmx_packet_make(m_tx, id(), PMX_CMD_MOTOR_WRITE, PMX_OPT_MOTION, data_tmp, 2);
      break;
    }
    default:
      return false;
    }
    mrd_servo_xfer(a_x, m_ln->bus, m_tx, len_tmp, m_rx, m_rx_len, timeout_tmp);
    return true;
  }

  void complete(const MrdHduartXfer &a_x) {
    const bool ok_tmp = a_x.ok && mrd_pmx_reply_check(m_rx, m_rx_len, id(), m_tx[4]); // m_tx[4]: 送ったコマンド
    if (!ok_tmp) {
      if (m_step >= W_MODE) {
        m_ln->st->torque[m_ix] = -1;
      }
      finish(MRD_PCMD_NO_REPLY);
      return;
    }
    const bool err_tmp = (m_rx[5] != 0); // ステータス
    switch (m_step) {
    case SYS_READ:
      finish(1);
      break;
    case GOAL:
    case MEM_WRITE:
      finish(err_tmp ? MRD_PCMD_SERVO_ERR : m_val);
      break;
    case SAVE:
      finish(err_tmp ? MRD_PCMD_SERVO_ERR : int16_t(1));
      break;
    case MEM_READ:
      finish(err_tmp ? MRD_PCMD_SERVO_ERR : to_val(mrd_pc_decode(mrd_pc_le_get(&m_rx[6], m_reg.len), m_reg.len, m_reg.sign)));
      break;
    case W_MODE:
      if (err_tmp) {
        finish(MRD_PCMD_SERVO_ERR);
        break;
      }
      mrd_pmx_line(m_ln->line).m_ctrl_mode[m_ix] = PMX_MODE_SPEED;
      m_step = (m_ln->st->torque[m_ix] != 1) ? W_TORQUE : W_SPEED;
      break;
    case W_TORQUE:
      m_ln->st->torque[m_ix] = (m_rx[6] == PMX_TORQUE_ON) ? 1 : -1;
      m_step = W_SPEED;
      break;
    case W_SPEED:
      m_ln->st->torque[m_ix] = (m_rx[6] == PMX_TORQUE_ON) ? 1 : -1;
      finish(err_tmp ? MRD_PCMD_SERVO_ERR : mrd_wheel_val(int16_t(m_rx[7] | (m_rx[8] << 8)) / 60.0f * m_ln->cw[m_ix]));
      break;
    default:
      break;
    }
  }

private:
  enum Step : uint8_t { SYS_READ = 1,
                        GOAL,
                        SAVE,
                        MEM_READ,
                        MEM_WRITE,
                        W_MODE,
                        W_TORQUE,
                        W_SPEED };

  size_t m_rx_len = 0;
  uint8_t m_tx[16];
  uint8_t m_rx[PMX_SYSTEM_REPLY];
};

/// @brief PMXサーボ1系統分の1フレームの処理を, 1回ずつの送受信に分けて進める.
///        コマンド1のサーボは目標位置へ動かし, それ以外は脱力する. 全サーボの現在位置を読み取ってtgtに格納する.
///        サーボごとに [1] トルク(変化時のみ) → [2] 目標位置(ON) または 読み出し(OFF) の順に送る.
class PmxLineStepper : public MrdParamLineStepper<PmxParamOp> {
public:
  /// @param a_line 系統の参照.
  /// @param a_all_mount パラメータ系コマンドのスロットを除かないマウント状態(パラメータ系コマンドの実行用).
  /// @param a_num 処理するサーボ数.
  PmxLineStepper(const MrdServoLineRef &a_line, const int *a_all_mount, int a_num)
      : MrdParamLineStepper<PmxParamOp>(a_line, a_all_mount, a_num) {}

protected:
  bool next_main(MrdHduartXfer &a_x) override {
    for (;;) {
      if (m_step == NEXT_SERVO) {
        m_ix = m_ln.next_mounted(m_ix, m_num);
        if (m_ix >= m_num) {
          return false;
        }
        // 車輪(速度制御)から戻ったサーボは, まず位置制御に戻す
        m_step = (mrd_pmx_line(m_ln.line).m_ctrl_mode[m_ix] == PMX_MODE_SPEED) ? RESTORE : TORQUE;
      }
      const int id_tmp = m_ln.id[m_ix];
      a_x.bus = m_ln.bus;
      a_x.tx = m_tx;
      a_x.rx = m_rx;
      a_x.rx_len = sizeof(m_rx);
      if (id_tmp < 0 || id_tmp > PMX_ID_MAX) {
        a_x.tx_len = 0; // 範囲外のIDは送らず失敗扱い
        m_step = NEXT_SERVO;
        return true;
      }
      if (m_step == RESTORE) {
        m_cmd = PMX_CMD_MEM_WRITE;
        a_x.tx_len = mrd_pmx_packet_make_mode(m_tx, id_tmp, false);
        a_x.rx_len = PMX_ACK_LEN;
        return true;
      }
      if (m_step == TORQUE) {
        int8_t want_tmp;
        m_step = MOTION;
        if (m_ln.torque_due(m_ix, false, want_tmp)) {
          m_cmd = PMX_CMD_MOTOR_WRITE;
          a_x.tx_len = mrd_pmx_packet_make(m_tx, id_tmp, m_cmd, want_tmp ? PMX_TORQUE_ON : PMX_TORQUE_FREE, nullptr, 0);
          // 脱力したサーボはこの返信の現在位置を使い, 読み出しを省く
          if (!want_tmp) {
            m_step = NEXT_SERVO;
          }
          return true;
        }
      }
      // MOTION
      m_step = NEXT_SERVO;
      if (m_ln.on(m_ix)) {
        const uint16_t pos_tmp = uint16_t(int16_t(mrd_pmx_deg2pos(m_ln.tgt[m_ix], m_ln.trim[m_ix], m_ln.cw[m_ix])));
        const uint8_t data_tmp[2] = {uint8_t(pos_tmp & 0xFF), uint8_t(pos_tmp >> 8)};
        m_cmd = PMX_CMD_MOTOR_WRITE;
        a_x.tx_len = mrd_pmx_packet_make(m_tx, id_tmp, m_cmd, PMX_OPT_MOTION, data_tmp, sizeof(data_tmp));
      } else {
        m_cmd = PMX_CMD_MOTOR_READ;
        a_x.tx_len = mrd_pmx_packet_make(m_tx, id_tmp, m_cmd, 0, nullptr, 0);
      }
      return true;
    }
  }

  void complete_main(const MrdHduartXfer &a_x) override {
    if (m_step == RESTORE) {
      if (a_x.ok && mrd_pmx_reply_check(m_rx, PMX_ACK_LEN, m_ln.id[m_ix], m_cmd) && m_rx[5] == 0) {
        mrd_pmx_line(m_ln.line).m_ctrl_mode[m_ix] = PMX_MODE_POSITION;
        m_step = TORQUE;
      } else {
        m_step = NEXT_SERVO; // 次のフレームで戻し直す
        mrd_servo_read_apply(m_ln, m_ix, false, 0.0f);
      }
      return;
    }
    const bool valid_tmp = a_x.ok && mrd_pmx_reply_check(m_rx, sizeof(m_rx), m_ln.id[m_ix], m_cmd);
    if (!valid_tmp) {
      m_step = NEXT_SERVO; // トルクの送信に失敗したら, このフレームは目標位置を送らない(エラーは1回と数える)
      mrd_servo_read_apply(m_ln, m_ix, false, 0.0f);
      return;
    }
    // 返信のトルク状態を記憶する. 指示と違えば(サーボの保護機能や再起動で変わった場合)次のフレームで送り直す
    m_ln.st->torque[m_ix] = (m_rx[6] == PMX_TORQUE_ON) ? 1 : ((m_rx[6] == PMX_TORQUE_FREE) ? 0 : -1);
    if (m_step == MOTION) {
      return; // トルクONの直後: tgtは続けて送る目標位置なので, 現在位置は目標位置の返信で受け取る
    }
    const int pos_tmp = int16_t(m_rx[7] | (m_rx[8] << 8));
    mrd_servo_read_apply(m_ln, m_ix, true, mrd_pmx_pos2deg(pos_tmp, m_ln.trim[m_ix], m_ln.cw[m_ix]));
  }

private:
  enum Step : uint8_t { NEXT_SERVO,
                        RESTORE,
                        TORQUE,
                        MOTION };

  int m_ix = -1;                 // 処理中のサーボのインデックス
  Step m_step = NEXT_SERVO;      // 処理中のサーボの次の送信
  uint8_t m_cmd = 0;             // 直前に送ったコマンド
  uint8_t m_tx[12];              // 送信パケット(最長は位置制御に戻すMemWRITE: 8 + 4byte)
  uint8_t m_rx[PMX_MOTOR_REPLY]; // 返信(ステータス, トルク状態, 現在位置)
};

#endif // __MERIDIAN_SERVO_KONDO_PMX_H__
