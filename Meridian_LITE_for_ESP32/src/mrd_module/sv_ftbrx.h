#ifndef __MERIDIAN_SERVO_FUTABA_RSxTTL_H__
#define __MERIDIAN_SERVO_FUTABA_RSxTTL_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバスと共通処理. 本ファイルは mrd_servo.h から読み込まれる
#include "sv_util.h"   // 各プロトコル層の共通部品

//==================================================================================================
//  FUTABA RS30x (TTLコマンド方式) プロトコル層 (半二重UARTバス上で動作)
//==================================================================================================
//
// 対象: RS303MR, RS304MD 等のRS30x系TTLサーボ(取扱説明書 Ver.1.18 のメモリーマップ).
//
// 1フレームの処理:
//   [1] トルク: ON/OFFが変わったサーボだけロングパケット(アドレス36, 1バイト).
//       サーボの保護機能(温度リミット, 不感帯での自動OFF)でトルクが切れても復帰できるよう,
//       MRD_SERVO_TORQUE_REFRESH_FRAMES ごとに全サーボへ送り直す.
//       ※RS30xはトルクOFF中に受けた目標位置を無視するため, 目標位置より先に送る.
//   [2] 目標位置: ロングパケット(アドレス30, 2バイト, 0.1deg単位). トルクONのサーボのみ.
//   [3] 現在位置: 各サーボに個別の読み出し要求(アドレス42, 2バイト).
//       1フレームに読むサーボ数は SERVO_FTB_READ_PER_FRAME 台までとし, 超える場合は次のフレームで続きを読む.
//   L/R系統は mrd_servo_lines_run で並行して進める(もう一方の系統が他のプロトコルでもよい).
//
// 通信時間の制約:
//   通信速度は公式には最大460.8kbps(初期値115.2kbps. 公式資料にない691.2kbpsも設定可), サーボの返信は要求の受信から最短でも100us後.
//   460.8kbps, 15台の試算: [2] 53byte=1.15ms + [3] 1台あたり(要求8byte+応答10byte=0.39ms+100us+α)≒0.52ms.
//   15台すべてを毎フレーム読むと約9ms となり100Hzに収まらないため, SERVO_FTB_READ_PER_FRAME で台数を絞る
//   (既定の8台で約5.3ms. 15台なら各サーボの現在位置は2フレームに1回更新される). 目標位置は毎フレーム全台に送る.
//
// 位置値は0.1deg単位の符号付き値で, 中央が0. 可動範囲は±150deg.

// パケット
#define FTB_HDR_CMD_0       0xFA // 命令パケットのヘッダ
#define FTB_HDR_CMD_1       0xAF
#define FTB_HDR_RET_0       0xFD // 返信パケットのヘッダ
#define FTB_HDR_RET_1       0xDF
#define FTB_FLG_RETURN_ADDR 0x0F // 指定アドレスから指定バイト数の返信を要求
#define FTB_FLG_RETURN_ACK  0x01 // ACK/NACKの返信を要求(書き込み)
#define FTB_FLG_WRITE_ROM   0x40 // フラッシュROMへの書き込み(アドレス4〜29を保存)
#define FTB_ADDR_SPECIAL    0xFF // 特殊命令のアドレス
#define FTB_ACK             0x07 // ACKの返信(1byte)
#define FTB_NACK            0x08 // NACKの返信(1byte)

// メモリーマップ
#define FTB_REG_RETURN_DELAY     7  // 返信ディレイ時間(100us + 50us x 値, ROM)
#define FTB_REG_GOAL_POSITION    30 // 目標位置(2バイト, 0.1deg)
#define FTB_REG_TORQUE_ENABLE    36 // トルクON(0:OFF, 1:ON, 2:ブレーキ)
#define FTB_REG_PRESENT_POSITION 42 // 現在位置(2バイト, 0.1deg)

#define FTB_TX_MAX      MRD_HDUART_TX_MAX // 1パケットの最大長
#define FTB_POS_REPLY   10                // 現在位置(2バイト)の返信パケット長
#define FTB_POS_PER_DEG 10.0f
#define FTB_POS_LIMIT   1500.0f

//------------------------------------------------------------------------------------
//  パケットの生成と検証
//------------------------------------------------------------------------------------
//
// ショートパケット(1台): FA AF ID FLG ADR LEN CNT [DATA...] SUM
// ロングパケット(複数台): FA AF 00 00 ADR LEN CNT [VID DATA...]xCNT SUM   (LEN = 1 + DATA数)
// 返信パケット: FD DF ID FLG ADR LEN CNT DATA SUM   (LEN = DATA数, CNT = 1)
// SUM = ID から DATA の末尾までの XOR

/// @brief チェックサムを計算する(ID以降, SUMの手前まで).
/// @param a_pkt パケット.
/// @param a_len SUMを含むパケット長.
inline uint8_t mrd_ftb_checksum(const uint8_t *a_pkt, size_t a_len) {
  uint8_t sum_tmp = 0;
  for (size_t i = 2; i + 1 < a_len; i++) {
    sum_tmp ^= a_pkt[i];
  }
  return sum_tmp;
}

/// @brief 指定アドレスの返信を要求するショートパケットを作る.
/// @return パケット長(8).
size_t mrd_ftb_packet_make_read(uint8_t *a_buf, uint8_t a_id, uint8_t a_addr, uint8_t a_len) {
  a_buf[0] = FTB_HDR_CMD_0;
  a_buf[1] = FTB_HDR_CMD_1;
  a_buf[2] = a_id;
  a_buf[3] = FTB_FLG_RETURN_ADDR;
  a_buf[4] = a_addr;
  a_buf[5] = a_len;
  a_buf[6] = 0; // 返信要求の専用パケットはCount=0
  a_buf[7] = mrd_ftb_checksum(a_buf, 8);
  return 8;
}

/// @brief 返信パケットのヘッダ, ID, アドレス, 長さ, チェックサムを検証する.
/// @param a_rx 受信データ.
/// @param a_rx_len 受信データ数(8 + データ数).
/// @param a_id 期待するID.
/// @param a_addr 期待するアドレス.
inline bool mrd_ftb_reply_check(const uint8_t *a_rx, size_t a_rx_len, uint8_t a_id, uint8_t a_addr) {
  return a_rx[0] == FTB_HDR_RET_0 && a_rx[1] == FTB_HDR_RET_1 && a_rx[2] == a_id && a_rx[4] == a_addr &&
         a_rx[5] == a_rx_len - 8 && a_rx[6] == 1 && a_rx[a_rx_len - 1] == mrd_ftb_checksum(a_rx, a_rx_len);
}

/// @brief ロングパケットを組み立てる.
///        使い方: start(アドレス, 1台あたりのデータ数) → add(ID)が返す領域にデータを書く → finish().
struct FtbLongPacket {
  uint8_t buf[FTB_TX_MAX];
  uint8_t data_len = 0; // 1台あたりのデータ数
  uint8_t count = 0;    // 追加したサーボ数
  size_t len = 0;       // finish()後のパケット長(0:送信するものなし)

  void start(uint8_t a_addr, uint8_t a_data_len) {
    buf[0] = FTB_HDR_CMD_0;
    buf[1] = FTB_HDR_CMD_1;
    buf[2] = 0x00; // ロングパケットのIDとFlagsは0
    buf[3] = 0x00;
    buf[4] = a_addr;
    buf[5] = a_data_len + 1; // VID + DATA
    data_len = a_data_len;
    count = 0;
    len = 0;
  }

  /// @brief サーボを1台追加する.
  /// @return データの書き込み先. バッファに入りきらない場合はnullptr.
  uint8_t *add(uint8_t a_id) {
    const size_t pos_tmp = 7 + count * (data_len + 1);
    if (pos_tmp + data_len + 1 + 1 > sizeof(buf)) {
      return nullptr;
    }
    buf[pos_tmp] = a_id;
    count++;
    return &buf[pos_tmp + 1];
  }

  /// @brief CNTとSUMを確定する.
  /// @return パケット長. サーボが1台もなければ0.
  size_t finish() {
    if (count == 0) {
      len = 0;
      return 0;
    }
    buf[6] = count;
    len = 7 + count * (data_len + 1) + 1;
    buf[len - 1] = mrd_ftb_checksum(buf, len);
    return len;
  }
};

/// @brief 位置値の尺度: 0.1deg単位, 中央0, ±150deg.
static const MrdPosScale FTB_POS_SCALE = {FTB_POS_PER_DEG, 0.0f, -FTB_POS_LIMIT, FTB_POS_LIMIT};

/// @brief 角度(degree)をRS30xの位置値(0.1deg単位, ±1500)に変換する.
inline int mrd_ftb_deg2pos(float a_degree, float a_trim, int a_cw) {
  return mrd_servo_scale_deg2pos(a_degree, a_trim, a_cw, FTB_POS_SCALE);
}

/// @brief RS30xの位置値を角度(degree)に変換する.
inline float mrd_ftb_pos2deg(int a_pos, float a_trim, int a_cw) {
  return mrd_servo_scale_pos2deg(a_pos, a_trim, a_cw, FTB_POS_SCALE);
}

//------------------------------------------------------------------------------------
//  FUTABAサーボ1系統分のクラス
//------------------------------------------------------------------------------------

class FutabaHalfDuplexClass {
public:
  /// @brief コンストラクタ.
  /// @param a_bus 使用する半二重UARTバス.
  /// @param a_baud 通信速度bps(サーボの設定に合わせる. 初期値は115200).
  FutabaHalfDuplexClass(MrdHalfDuplexUart &a_bus, unsigned long a_baud) : m_bus(a_bus), m_baud(a_baud) {}

  /// @brief 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ.
  void set_baud(unsigned long a_baud) { m_baud = a_baud; }

  /// @brief バスを開始(8N1)し, 接続されたサーボの応答と返信ディレイ時間を確認する.
  /// @param a_line_name 表示用の系統名("L"など).
  /// @param a_mount 各サーボのマウント有無.
  /// @param a_id 各サーボのID.
  /// @param a_num サーボ数.
  /// @return バスを開始できればtrue.
  bool begin(const char *a_line_name, const int *a_mount, const int *a_id, int a_num) {
    m_state.reset();
    if (!m_bus.begin(m_baud, SERIAL_8N1)) {
      return false;
    }
    // 通信速度の設定値 0x00〜0x0A. 0x0B(691200)は公式資料にないが設定できるため含める
    const unsigned long bauds_tmp[] = {9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 153600, 230400, 460800, 691200};
    mrd_servo_baud_check(a_line_name, "FUTABA", m_baud, bauds_tmp, sizeof(bauds_tmp) / sizeof(bauds_tmp[0]),
                         " bps is not a RS30x baud rate. Check SERVO_BAUDRATE_x (default 115200).");

    for (int i = 0; i < a_num && i < MRD_SERVO_SLOTS; i++) {
      if (!a_mount[i]) {
        continue;
      }
      const uint8_t id_tmp = a_id[i];
      mrd_servo_slot_print(a_line_name, "FUTABA", i, id_tmp);

      // 返信ディレイ時間はROM領域のため自動では書き換えない(0が最短の100us)
      uint8_t delay_tmp = 0;
      bool found_tmp = false;
      for (int k = 0; k < MRD_SERVO_BOOT_RETRY && !found_tmp; k++) {
        found_tmp = read(id_tmp, FTB_REG_RETURN_DELAY, &delay_tmp, 1);
      }
      if (!found_tmp) {
        Serial.println("no response");
        continue;
      }
      Serial.print("return delay ");
      Serial.print(100 + delay_tmp * 50);
      Serial.print("us");
      if (delay_tmp != 0) {
        Serial.print(" (set 0 for 100us to speed up the bus)");
      }
      Serial.println();
    }
    return true;
  }

  /// @brief メモリーマップを読み出す(1回の送受信).
  /// @return 正しい返信を受信できればtrue.
  bool read(uint8_t a_id, uint8_t a_addr, uint8_t *a_data, uint8_t a_len) {
    uint8_t tx_tmp[8];
    uint8_t rx_tmp[8 + 16];
    const size_t rx_len_tmp = 8 + a_len;
    if (rx_len_tmp > sizeof(rx_tmp)) {
      return false;
    }
    mrd_ftb_packet_make_read(tx_tmp, a_id, a_addr, a_len);
    if (!m_bus.transfer(tx_tmp, 8, rx_tmp, rx_len_tmp) || !mrd_ftb_reply_check(rx_tmp, rx_len_tmp, a_id, a_addr)) {
      return false;
    }
    memcpy(a_data, &rx_tmp[7], a_len);
    return true;
  }

  /// @brief 半二重UARTバス(並行送受信用).
  MrdHalfDuplexUart &bus() { return m_bus; }

  /// @brief 通信速度bps.
  unsigned long baud() const { return m_baud; }

  /// @brief フレームをまたいで保持する状態(トルク状態の記憶, 順番に読むサーボの位置).
  MrdServoLineState m_state;

private:
  MrdHalfDuplexUart &m_bus;
  unsigned long m_baud;
};

//==================================================================================================
//  FUTABAサーボの駆動処理
//==================================================================================================

/// @brief 系統のFUTABAサーボクラス.
inline FutabaHalfDuplexClass &mrd_ftb_line(int a_line) { return (a_line == L) ? ftb_l : ftb_r; }

/// @brief 系統の参照を作る.
/// @param a_line 系統(L or R).
/// @param a_meridim Meridimデータの参照.
/// @param a_sv サーボパラメータ.
MrdServoLineRef mrd_ftb_line_ref(UartLine a_line, Meridim90Union &a_meridim, ServoParam &a_sv) {
  return mrd_servo_line_ref(a_line, FTBRSX, mrd_ftb_line(a_line).bus(), mrd_ftb_line(a_line).m_state, a_meridim, a_sv);
}

//------------------------------------------------------------------------------------
//  パラメータ系コマンド (番号と値の形式は tasks/servo_param_plan.md)
//------------------------------------------------------------------------------------

/// @brief パラメータ系コマンドの対応表. ROM領域(アドレス4〜29)は書いた時点で有効, 保存は35(ROM保存).
/// @param a_cmd コマンド.
/// @return レジスタの定義(len=0:未対応).
MrdParamReg mrd_ftb_param_reg(uint8_t a_cmd) {
  switch (a_cmd) {
  case 4:
  case 5:
    return mrd_pc_reg(FTB_REG_GOAL_POSITION, 2, a_cmd == 4, PC_POS, 1.0f, PC_SIGNED);
  case 6:
    return mrd_pc_reg(FTB_REG_PRESENT_POSITION, 2, false, PC_POS, 1.0f, PC_SIGNED);
  case 7:
    return mrd_pc_reg(52, 2, false); // 10mV = V x100
  case 10:
  case 11: // CW/CCWコンプライアンスマージン(0.1deg, 同じ値を書く)
    return mrd_pc_reg(24, 1, a_cmd == 11, PC_SCALE, 10.0f, PC_UNSIGNED, false, true);
  case 12:
  case 13:
    return mrd_pc_reg(8, 2, a_cmd == 13, PC_POS_NATIVE, 1.0f, PC_SIGNED);
  case 14:
  case 15:
    return mrd_pc_reg(10, 2, a_cmd == 15, PC_POS_NATIVE, 1.0f, PC_SIGNED);
  case 22: // 温度リミット(読み出しのみ)
    return mrd_pc_reg(14, 2, false, PC_RAW, 1.0f, PC_SIGNED);
  case 26: // 現在速度(deg/s)
    return mrd_pc_reg(46, 2, false, PC_RAW, 1.0f, PC_SIGNED);
  case 30:
  case 31:
    return mrd_pc_reg(35, 1, a_cmd == 31); // 最大トルク(%)
  case 32:
  case 33:
    return mrd_pc_reg(32, 2, a_cmd == 33, PC_SCALE, 10.0f); // 10ms → ms
  case 36:
    return mrd_pc_reg(50, 2, false, PC_RAW, 1.0f, PC_SIGNED);
  case 37:
    return mrd_pc_reg(48, 2, false, PC_RAW, 1.0f, PC_SIGNED);
  case 38:
    return mrd_pc_reg(FTB_REG_TORQUE_ENABLE, 1, false, PC_BOOL);
  case 54:
  case 55: // パンチ(0.01%)
    return mrd_pc_reg(28, 2, a_cmd == 55);
  case 56:
  case 57: // CW/CCWコンプライアンスマージン(0.1deg)
    return mrd_pc_reg(24, 1, a_cmd == 57, PC_RAW, 1.0f, PC_UNSIGNED, false, true);
  case 60:
  case 61: // CW/CCWコンプライアンススロープ(1deg)
    return mrd_pc_reg(26, 1, a_cmd == 61, PC_RAW, 1.0f, PC_UNSIGNED, false, true);
  case 70:
  case 71:
    return mrd_pc_reg(FTB_REG_RETURN_DELAY, 1, a_cmd == 71);
  default:
    return MrdParamReg();
  }
}

/// @brief FUTABAのパラメータ系コマンド1件分の処理(1回の送受信).
///        3:応答確認(アドレス0を読む), 35:ROM保存(返信なし), 読み出しはアドレス指定の返信要求,
///        書き込みはACKの返信(1byte)を要求するショートパケット. 書き込みの結果はACKなら受け取ったVAL, NACKならサーボのエラー.
class FtbParamOp : public MrdParamOpBase {
public:
  void begin(const MrdServoLineRef &a_line, int a_ix, uint8_t a_cmd, int16_t a_val) {
    target(a_line, a_ix, a_val);
    m_scale = FTB_POS_SCALE;
    if (a_cmd == 3) {
      m_reg = mrd_pc_reg(0, 2, false); // モデル番号を読んで応答を確認する
      m_ping = true;
      return;
    }
    if (a_cmd == 35) {
      m_save = true;
      return;
    }
    m_reg = mrd_ftb_param_reg(a_cmd);
    if (m_reg.len == 0 || !write_value_ok()) { // 未対応, またはレジスタに収まらない値は送らない
      finish(MRD_PCMD_NA);
    }
  }

  bool next(MrdHduartXfer &a_x) {
    if (finished()) {
      return false;
    }
    if (m_save) {
      m_tx[0] = FTB_HDR_CMD_0;
      m_tx[1] = FTB_HDR_CMD_1;
      m_tx[2] = id();
      m_tx[3] = FTB_FLG_WRITE_ROM;
      m_tx[4] = FTB_ADDR_SPECIAL;
      m_tx[5] = 0;
      m_tx[6] = 0;
      m_tx[7] = mrd_ftb_checksum(m_tx, 8);
      mrd_servo_xfer(a_x, m_ln->bus, m_tx, 8, m_rx, 0);
      return true;
    }
    if (!m_reg.write) {
      mrd_servo_xfer(a_x, m_ln->bus, m_tx, mrd_ftb_packet_make_read(m_tx, id(), m_reg.addr, m_reg.len), m_rx, 8 + m_reg.len);
      return true;
    }
    const uint8_t len_tmp = m_reg.pair ? 2 : m_reg.len;
    const uint32_t raw_tmp = mrd_pc_encode(to_raw(), m_reg.sign);
    m_tx[0] = FTB_HDR_CMD_0;
    m_tx[1] = FTB_HDR_CMD_1;
    m_tx[2] = id();
    m_tx[3] = FTB_FLG_RETURN_ACK;
    m_tx[4] = m_reg.addr;
    m_tx[5] = len_tmp;
    m_tx[6] = 1;
    mrd_pc_le_put(&m_tx[7], raw_tmp, m_reg.len);
    if (m_reg.pair) {
      m_tx[8] = m_tx[7]; // 続くアドレスにも同じ値
    }
    m_tx[7 + len_tmp] = mrd_ftb_checksum(m_tx, 8 + len_tmp);
    mrd_servo_xfer(a_x, m_ln->bus, m_tx, 8 + len_tmp, m_rx, FTB_ACK_REPLY);
    return true;
  }

  void complete(const MrdHduartXfer &a_x) {
    if (m_save) {
      finish(1); // 返信なし
      return;
    }
    if (!m_reg.write) {
      if (!a_x.ok || !mrd_ftb_reply_check(m_rx, a_x.rx_len, id(), m_reg.addr)) {
        finish(MRD_PCMD_NO_REPLY);
      } else {
        finish(m_ping ? 1 : to_val(mrd_pc_decode(mrd_pc_le_get(&m_rx[7], m_reg.len), m_reg.len, m_reg.sign)));
      }
      return;
    }
    // ACK/NACKの返信は1byte(07h: ACK, 08h: NACK. RS303MR/RS304MD 取扱説明書 4章 Flag の Bit3〜0)
    if (a_x.ok && m_rx[0] == FTB_ACK) {
      finish(m_val);
    } else {
      finish((a_x.ok && m_rx[0] == FTB_NACK) ? int16_t(MRD_PCMD_SERVO_ERR) : int16_t(MRD_PCMD_NO_REPLY));
    }
  }

private:
  static constexpr size_t FTB_ACK_REPLY = 1; // ACK/NACKの返信の長さ(1byte)

  bool m_ping = false;
  bool m_save = false;
  uint8_t m_tx[12];
  uint8_t m_rx[10];
};

/// @brief FUTABAサーボ1系統分の1フレームの処理を, 1回ずつの送受信に分けて進める.
///        コマンド1のサーボは目標位置へ動かし, それ以外は脱力する. 現在位置を読み取ってtgtに格納する
///        (このフレームで読まないサーボは前回の値).
///        [1] トルク → [2] 目標位置 → [3] 最大 SERVO_FTB_READ_PER_FRAME 台の現在位置 の順に送る.
class FtbLineStepper : public MrdParamLineStepper<FtbParamOp> {
public:
  /// @param a_line 系統の参照.
  /// @param a_all_mount パラメータ系コマンドのスロットを除かないマウント状態(パラメータ系コマンドの実行用).
  /// @param a_num 処理するサーボ数.
  FtbLineStepper(const MrdServoLineRef &a_line, const int *a_all_mount, int a_num)
      : MrdParamLineStepper<FtbParamOp>(a_line, a_all_mount, a_num) {}

  void skip_reads() override { m_skip_reads = true; }

protected:
  bool next_main(MrdHduartXfer &a_x) override {
    while (m_phase != DONE) {
      size_t len_tmp = 0;
      const uint8_t *tx_tmp = m_pkt.buf;
      m_reading = false;
      switch (m_phase) {
      case TORQUE:
        len_tmp = build_torque();
        m_phase = GOAL;
        break;
      case GOAL:
        len_tmp = build_goal();
        m_phase = READ;
        select_reads();
        break;
      case READ:
        do {
          m_ix++;
        } while (m_ix < m_num && !m_read[m_ix]);
        if (m_ix >= m_num) {
          m_phase = DONE;
          break;
        }
        len_tmp = mrd_ftb_packet_make_read(m_tx, m_ln.id[m_ix], FTB_REG_PRESENT_POSITION, 2);
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
      return; // ロングパケットは返信なし
    }
    const bool valid_tmp = a_x.ok && mrd_ftb_reply_check(m_rx, sizeof(m_rx), m_ln.id[m_ix], FTB_REG_PRESENT_POSITION);
    const int pos_tmp = int16_t(m_rx[7] | (m_rx[8] << 8));
    mrd_servo_read_apply(m_ln, m_ix, valid_tmp, valid_tmp ? mrd_ftb_pos2deg(pos_tmp, m_ln.trim[m_ix], m_ln.cw[m_ix]) : 0.0f);
  }

private:
  enum Phase : uint8_t { TORQUE,
                         GOAL,
                         READ,
                         DONE };

  /// @brief トルクのロングパケットを作る. ON/OFFが変わったサーボと, 定期的な送り直しのフレームでは全サーボ.
  size_t build_torque() {
    m_pkt.start(FTB_REG_TORQUE_ENABLE, 1);
    mrd_servo_torque_each(m_ln, m_num, [this](int a_ix, int8_t a_want) {
      uint8_t *slot_tmp = m_pkt.add(m_ln.id[a_ix]);
      if (slot_tmp != nullptr) {
        *slot_tmp = a_want;
      }
      return slot_tmp != nullptr;
    });
    return m_pkt.finish();
  }

  /// @brief 目標位置のロングパケットを作る(トルクONのサーボのみ).
  size_t build_goal() {
    m_pkt.start(FTB_REG_GOAL_POSITION, 2);
    for (int i = 0; i < m_num; i++) {
      if (!m_ln.mount[i] || !m_ln.on(i)) {
        continue;
      }
      uint8_t *slot_tmp = m_pkt.add(m_ln.id[i]);
      if (slot_tmp == nullptr) {
        continue;
      }
      const uint16_t pos_tmp = uint16_t(int16_t(mrd_ftb_deg2pos(m_ln.tgt[i], m_ln.trim[i], m_ln.cw[i])));
      slot_tmp[0] = pos_tmp & 0xFF;
      slot_tmp[1] = pos_tmp >> 8;
    }
    return m_pkt.finish();
  }

  /// @brief このフレームで現在位置を読むサーボを, 前回の続きから最大 SERVO_FTB_READ_PER_FRAME 台選ぶ.
  ///        読まないサーボは前回の値にする. 読まないフレーム(skip_reads)は1台も読まず, 順番も進めない.
  void select_reads() {
    int left_tmp = m_skip_reads ? 0 : max(cfg.servo_ftb_read_per_frame, 1);
    const int start_tmp = m_ln.st->read_cursor % max(m_num, 1);
    for (int i = 0; i < m_num; i++) {
      m_read[i] = false;
    }
    for (int n = 0; n < m_num && left_tmp > 0; n++) {
      const int ix_tmp = (start_tmp + n) % m_num;
      if (!m_ln.mount[ix_tmp]) {
        continue;
      }
      m_read[ix_tmp] = true;
      left_tmp--;
      m_ln.st->read_cursor = (ix_tmp + 1) % m_num;
    }
    for (int i = 0; i < m_num; i++) {
      if (m_ln.mount[i] && !m_read[i]) {
        mrd_servo_past_keep(m_ln, i);
      }
    }
  }

  Phase m_phase = TORQUE;
  int m_ix = -1;                // 現在位置を読んでいるサーボのインデックス
  bool m_skip_reads = false;    // このフレームは現在位置を読まないか(混在時の交互読み出し, マスターコマンド2)
  bool m_reading = false;       // 直前に準備したのが読み出しか
  bool m_read[MRD_SERVO_SLOTS]; // このフレームで現在位置を読むサーボ
  FtbLongPacket m_pkt;          // ロングパケット
  uint8_t m_tx[8];              // 読み出し要求パケット
  uint8_t m_rx[FTB_POS_REPLY];  // 返信(位置2バイト)
};

#endif // __MERIDIAN_SERVO_FUTABA_RSxTTL_H__
