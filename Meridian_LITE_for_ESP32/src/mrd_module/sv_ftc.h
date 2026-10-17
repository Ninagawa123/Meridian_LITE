#ifndef __MERIDIAN_SERVO_FEETECH_H__
#define __MERIDIAN_SERVO_FEETECH_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバスと共通処理. 本ファイルは mrd_servo.h から読み込まれる
#include "sv_util.h"   // 各プロトコル層の共通部品

//==================================================================================================
//  FEETECH STS/SCS プロトコル層 (半二重UARTバス上で動作)
//==================================================================================================
//
// STSとSCSは別のサーボとして扱い, 系列はconfigのサーボタイプ(61:STS, 62:SCS)に従う.
// 1系統はどちらか一方の系列で揃えることを推奨する. 混在した場合も動くが, 現在位置はSTSとSCSを
// 1フレームずつ交互に読む(読む台数を減らしてフレームに収めるため. 読まないフレームは前回の値).
// 起動時にサーボのアドレス2(END)で系列を確認し, configと違えば警告する(configの指定を使う).
//
// STSとSCSの違い:
//   項目              STS                 SCS
//   2バイト値の順序   下位が先(LE)        上位が先(BE)
//   位置の範囲        0-4095 (360deg)     0-1023 (300deg, SCS0009等)
//   EEPROMロック      アドレス55          アドレス48
//   共通: ID(5), 応答遅延時間(7), トルク(40), 目標位置(42), 目標時間(44), 目標速度(46), 現在位置(56)
//
// 1フレームの処理 (1系統15台を100Hzで動かすための構成):
//   [1] 目標位置: SyncWrite(0x83)を系列ごとに1パケット(アドレス42から).
//       STS: 位置(2バイト). 目標速度0が最大速度なので, 速度は起動時に0にしておき位置だけを送る.
//       SCS: 位置+目標時間0+目標速度(6バイト). SCSは目標速度0では駆動しないため, 速度を毎回送る
//            (サーボが電源の瞬断等で再起動して速度が初期値に戻っても動き続けられる).
//   [2] トルク: ON/OFFが変わったサーボだけSyncWrite(アドレス40, 1バイト).
//       サーボの保護機能等でトルクが切れても復帰できるよう, MRD_SERVO_TORQUE_REFRESH_FRAMES ごとに全サーボへ送り直す.
//   [3] 現在位置: 各サーボにREAD(アドレス56, 2バイト). STSとSCSが混在する系統は, 1フレームおきに一方の系列だけ読む.
//   1Mbps, 15台の試算: [1] STS 53byte=0.53ms(SCSのみなら113byte=1.13ms) + [3] 15x(要求8byte+応答8byte+α)≒3.0ms
//                     → 約3.5ms/フレーム(SCSのみで約4.1ms).
//   ※サーボの応答遅延時間(アドレス7)が0でないと1台あたり最大500us遅れるため, 起動時に0にする
//     (SERVO_FTC_FIX_RETURN_DELAY).
//
// 角度の扱い, トルク状態の記憶, 結果の反映は sv_util.h の共通部品を使う.

// 命令
#define FTC_ID_BROADCAST    0xFE // ブロードキャストID
#define FTC_INST_PING       0x01 // 応答確認
#define FTC_INST_READ       0x02 // 読み出し
#define FTC_INST_WRITE      0x03 // 書き込み
#define FTC_INST_SYNC_WRITE 0x83 // 一斉書き込み(応答なし)

// レジスタアドレス
#define FTC_REG_END              2  // 2バイト値の順序(0:STS, 1:SCS). 系列判定に使用
#define FTC_REG_RETURN_DELAY     7  // 応答遅延時間(単位2us, EEPROM)
#define FTC_REG_TORQUE_ENABLE    40 // トルクON/OFF
#define FTC_REG_ACC              41 // 加速度(STSのみ, 0:制限なし)
#define FTC_REG_GOAL_POSITION    42 // 目標位置(2バイト)
#define FTC_REG_GOAL_TIME        44 // 目標時間(2バイト), 続く46が目標速度(2バイト). STSは速度0で最大速度
#define FTC_REG_GOAL_SPEED       46 // 目標速度(2バイト. 車輪では回転速度)
#define FTC_REG_PRESENT_POSITION 56 // 現在位置(2バイト)
#define FTC_REG_PRESENT_SPEED    58 // 現在速度(2バイト, bit15が符号)
#define FTC_REG_LOCK_STS         55 // EEPROMロック(STS, 0:書込可, 1:ロック)
#define FTC_REG_LOCK_SCS         48 // EEPROMロック(SCS, 0:書込可, 1:ロック)

#define FTC_TX_MAX    MRD_HDUART_TX_MAX // 1パケットの最大長
#define FTC_READ_NONE -1                // 現在位置を読まないフレームの読み出し系列(FtcLineStepper)

//------------------------------------------------------------------------------------
//  系列ごとの値の変換
//------------------------------------------------------------------------------------

/// @brief サーボタイプがFEETECH(STS/SCS)かを判定する.
/// @param a_type サーボタイプ(61:STS, 62:SCS).
inline bool mrd_ftc_is_ftc(int a_type) {
  return a_type == FTCSTS || a_type == FTCSCS;
}

/// @brief 系列ごとの位置値の分解能(STS:4096, SCS:1024).
inline float mrd_ftc_resolution(int a_type) {
  return (a_type == FTCSCS) ? 1024.0f : 4096.0f;
}

/// @brief 系列ごとの位置値の全範囲に対応する角度(degree).
inline float mrd_ftc_range(int a_type) {
  return (a_type == FTCSCS) ? cfg.servo_ftc_scs_range : cfg.servo_ftc_sts_range;
}

/// @brief 系列ごとの位置値の尺度. 中央(STS:2048, SCS:512)が0deg.
inline MrdPosScale mrd_ftc_scale(int a_type) {
  const float res_tmp = mrd_ftc_resolution(a_type);
  return {res_tmp / mrd_ftc_range(a_type), res_tmp * 0.5f, 0.0f, res_tmp - 1.0f};
}

/// @brief 1ステップあたりの角度(degree). 速度(ステップ/秒)や不感帯(ステップ)の換算に使う.
inline float mrd_ftc_step_deg(int a_type) { return mrd_ftc_range(a_type) / mrd_ftc_resolution(a_type); }

/// @brief 角度(degree)をFEETECHサーボの位置値に変換する. 中央が0deg.
/// @param a_degree 角度(degree).
/// @param a_trim トリム値(degree).
/// @param a_cw 回転方向補正値(+1 or -1).
/// @param a_type サーボタイプ(61:STS, 62:SCS).
/// @return 位置値(STS:0-4095, SCS:0-1023).
int mrd_ftc_deg2pos(float a_degree, float a_trim, int a_cw, int a_type) {
  return mrd_servo_scale_deg2pos(a_degree, a_trim, a_cw, mrd_ftc_scale(a_type));
}

/// @brief FEETECHサーボの位置値を角度(degree)に変換する. mrd_ftc_deg2pos の逆変換.
/// @param a_pos 位置値.
/// @param a_trim トリム値(degree).
/// @param a_cw 回転方向補正値(+1 or -1).
/// @param a_type サーボタイプ(61:STS, 62:SCS).
/// @return 角度(degree).
float mrd_ftc_pos2deg(int a_pos, float a_trim, int a_cw, int a_type) {
  return mrd_servo_scale_pos2deg(a_pos, a_trim, a_cw, mrd_ftc_scale(a_type));
}

/// @brief 2バイト値を系列のバイト順で書き込む(STS:LE, SCS:BE).
inline void mrd_ftc_u16_put(uint8_t *a_buf, uint16_t a_val, int a_type) {
  if (a_type == FTCSCS) {
    a_buf[0] = a_val >> 8;
    a_buf[1] = a_val & 0xFF;
  } else {
    a_buf[0] = a_val & 0xFF;
    a_buf[1] = a_val >> 8;
  }
}

/// @brief 位置値を系列のバイト順で読み取る. STSはbit15が符号(マルチターン時の負の位置).
inline int mrd_ftc_pos_get(const uint8_t *a_buf, int a_type) {
  if (a_type == FTCSCS) {
    return (a_buf[0] << 8) | a_buf[1];
  }
  const uint16_t raw_tmp = a_buf[0] | (a_buf[1] << 8);
  return (raw_tmp & 0x8000) ? -static_cast<int>(raw_tmp & 0x7FFF) : raw_tmp;
}

//------------------------------------------------------------------------------------
//  パケットの生成と検証
//------------------------------------------------------------------------------------
//
// 命令パケット: 0xFF 0xFF ID LEN INST [PARAM...] CHK   (LEN = PARAM数 + 2)
// 応答パケット: 0xFF 0xFF ID LEN ERR  [DATA...]  CHK   (LEN = DATA数 + 2)
// CHK = ~(ID + LEN + INST/ERR + PARAM/DATA の合計)

/// @brief チェックサムを計算する(ID以降, CHKの手前まで).
/// @param a_pkt パケット.
/// @param a_len CHKを含むパケット長.
inline uint8_t mrd_ftc_checksum(const uint8_t *a_pkt, size_t a_len) {
  uint8_t sum_tmp = 0;
  for (size_t i = 2; i + 1 < a_len; i++) {
    sum_tmp += a_pkt[i];
  }
  return ~sum_tmp;
}

/// @brief PING命令パケット(応答確認)を作る.
/// @return パケット長(6).
inline size_t mrd_ftc_packet_make_ping(uint8_t *a_buf, uint8_t a_id) {
  a_buf[0] = 0xFF;
  a_buf[1] = 0xFF;
  a_buf[2] = a_id;
  a_buf[3] = 2;
  a_buf[4] = FTC_INST_PING;
  a_buf[5] = mrd_ftc_checksum(a_buf, 6);
  return 6;
}

/// @brief READ命令パケットを作る.
/// @return パケット長(8).
size_t mrd_ftc_packet_make_read(uint8_t *a_buf, uint8_t a_id, uint8_t a_addr, uint8_t a_len) {
  a_buf[0] = 0xFF;
  a_buf[1] = 0xFF;
  a_buf[2] = a_id;
  a_buf[3] = 4;
  a_buf[4] = FTC_INST_READ;
  a_buf[5] = a_addr;
  a_buf[6] = a_len;
  a_buf[7] = mrd_ftc_checksum(a_buf, 8);
  return 8;
}

/// @brief WRITE命令パケットを作る.
/// @return パケット長(7 + a_len).
size_t mrd_ftc_packet_make_write(uint8_t *a_buf, uint8_t a_id, uint8_t a_addr, const uint8_t *a_data, uint8_t a_len) {
  a_buf[0] = 0xFF;
  a_buf[1] = 0xFF;
  a_buf[2] = a_id;
  a_buf[3] = a_len + 3;
  a_buf[4] = FTC_INST_WRITE;
  a_buf[5] = a_addr;
  memcpy(&a_buf[6], a_data, a_len);
  const size_t len_tmp = 7 + a_len;
  a_buf[len_tmp - 1] = mrd_ftc_checksum(a_buf, len_tmp);
  return len_tmp;
}

/// @brief 応答パケットのヘッダ, ID, 長さ, チェックサムを検証する.
/// @param a_rx 受信データ.
/// @param a_rx_len 受信データ数(6 + データ数).
/// @param a_id 期待するID.
inline bool mrd_ftc_reply_check(const uint8_t *a_rx, size_t a_rx_len, uint8_t a_id) {
  return a_rx[0] == 0xFF && a_rx[1] == 0xFF && a_rx[2] == a_id && a_rx[3] == a_rx_len - 4 &&
         a_rx[a_rx_len - 1] == mrd_ftc_checksum(a_rx, a_rx_len);
}

/// @brief SyncWrite(0x83)パケットを組み立てる.
///        使い方: start(アドレス, 1台あたりのデータ数) → add(ID)が返す領域にデータを書く → finish().
struct FtcSyncPacket {
  uint8_t buf[FTC_TX_MAX];
  uint8_t data_len = 0; // 1台あたりのデータ数
  uint8_t count = 0;    // 追加したサーボ数
  size_t len = 0;       // finish()後のパケット長(0:送信するものなし)

  void start(uint8_t a_addr, uint8_t a_data_len) {
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = FTC_ID_BROADCAST;
    buf[4] = FTC_INST_SYNC_WRITE;
    buf[5] = a_addr;
    buf[6] = a_data_len;
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

  /// @brief LENとCHKを確定する.
  /// @return パケット長. サーボが1台もなければ0.
  size_t finish() {
    if (count == 0) {
      len = 0;
      return 0;
    }
    buf[3] = count * (data_len + 1) + 4;
    len = 7 + count * (data_len + 1) + 1;
    buf[len - 1] = mrd_ftc_checksum(buf, len);
    return len;
  }
};

//------------------------------------------------------------------------------------
//  FEETECHサーボ1系統分のクラス
//------------------------------------------------------------------------------------

class FeetechHalfDuplexClass {
public:
  /// @brief コンストラクタ.
  /// @param a_bus 使用する半二重UARTバス.
  /// @param a_baud 通信速度bps(FEETECHの初期値は1000000).
  FeetechHalfDuplexClass(MrdHalfDuplexUart &a_bus, unsigned long a_baud)
      : m_bus(a_bus), m_baud(a_baud) {}

  /// @brief 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ.
  void set_baud(unsigned long a_baud) { m_baud = a_baud; }

  /// @brief バスを開始(8N1)し, 接続されたサーボの系列確認と初期設定を行う.
  ///        系列はconfigの指定を使い, a_type に書き戻す(駆動時はこの値でSTS/SCSを切り替える).
  /// @param a_line_name 表示用の系統名("L"など).
  /// @param a_mount 各サーボのマウント有無.
  /// @param a_type 各サーボのタイプ. 61/62以外の場合は a_line_type を仮の系列とする.
  /// @param a_id 各サーボのID.
  /// @param a_num サーボ数.
  /// @param a_line_type 系統のサーボタイプ(61 or 62).
  /// @return バスを開始できればtrue.
  bool begin(const char *a_line_name, const int *a_mount, float *a_type, const int *a_id, int a_num,
             int a_line_type) {
    m_state.reset();
    if (!m_bus.begin(m_baud, SERIAL_8N1)) {
      return false;
    }
    const unsigned long bauds_tmp[] = {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};
    mrd_servo_baud_check(a_line_name, "FEETECH", m_baud, bauds_tmp, sizeof(bauds_tmp) / sizeof(bauds_tmp[0]),
                         " bps is not a FEETECH baud rate. Check SERVO_BAUDRATE_x (default 1000000).");

    FtcSyncPacket goal_tmp; // STSの目標時間と目標速度を0(最大速度)にする. SCSは毎フレーム速度を送る
    FtcSyncPacket acc_tmp;  // STSの加速度を0(制限なし)にする
    goal_tmp.start(FTC_REG_GOAL_TIME, 4);
    acc_tmp.start(FTC_REG_ACC, 1);

    for (int i = 0; i < a_num && i < MRD_SERVO_SLOTS; i++) {
      if (!a_mount[i]) {
        continue;
      }
      const uint8_t id_tmp = a_id[i];
      const int type_tmp = mrd_ftc_is_ftc(int(a_type[i])) ? int(a_type[i]) : a_line_type;
      a_type[i] = type_tmp;

      // アドレス2(END)で系列を確認する(応答の有無の確認を兼ねる)
      uint8_t end_tmp = 0xFF;
      bool found_tmp = false;
      for (int k = 0; k < MRD_SERVO_BOOT_RETRY && !found_tmp; k++) {
        found_tmp = read(id_tmp, FTC_REG_END, &end_tmp, 1);
      }

      mrd_servo_slot_print(a_line_name, "FEETECH", i, id_tmp);
      Serial.print(type_tmp == FTCSCS ? "SCS" : "STS");
      if (!found_tmp) {
        Serial.println(" (no response)");
        continue;
      }
      if (end_tmp <= 1 && (end_tmp == 1) != (type_tmp == FTCSCS)) {
        Serial.print(" (servo looks ");
        Serial.print(end_tmp == 1 ? "SCS" : "STS");
        Serial.print(", check config)");
      }

      // 応答遅延時間が0でなければ0にする(1台あたり最大500usの待ちを防ぐ)
      uint8_t delay_tmp = 0;
      bool delay_read_tmp = false;
      for (int k = 0; k < MRD_SERVO_BOOT_RETRY && !delay_read_tmp; k++) {
        delay_read_tmp = read(id_tmp, FTC_REG_RETURN_DELAY, &delay_tmp, 1);
      }
      Serial.print(" return delay ");
      if (!delay_read_tmp) {
        Serial.println("read failed (may slow down the bus)");
      } else {
        Serial.print(delay_tmp * 2);
        Serial.print("us");
        if (delay_tmp != 0 && cfg.servo_ftc_fix_return_delay) {
          Serial.print(fix_return_delay(id_tmp, type_tmp) ? " -> 0us" : " -> fix failed");
        }
        Serial.println();
      }

      if (type_tmp == FTCSTS) {
        uint8_t *slot_tmp = goal_tmp.add(id_tmp);
        if (slot_tmp != nullptr) {
          memset(slot_tmp, 0, 4);
        }
        slot_tmp = acc_tmp.add(id_tmp);
        if (slot_tmp != nullptr) {
          *slot_tmp = 0;
        }
      }
    }

    if (goal_tmp.finish() > 0) {
      m_bus.transfer(goal_tmp.buf, goal_tmp.len, nullptr, 0);
    }
    if (acc_tmp.finish() > 0) {
      m_bus.transfer(acc_tmp.buf, acc_tmp.len, nullptr, 0);
    }
    return true;
  }

  /// @brief レジスタを読み出す(1回の送受信).
  /// @return 正しい応答を受信できればtrue.
  bool read(uint8_t a_id, uint8_t a_addr, uint8_t *a_data, uint8_t a_len) {
    uint8_t tx_tmp[8];
    uint8_t rx_tmp[FTC_TX_MAX];
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

  /// @brief レジスタに書き込み, 応答を確認する(1回の送受信).
  /// @return 正しい応答(エラーなし)を受信できればtrue.
  bool write(uint8_t a_id, uint8_t a_addr, const uint8_t *a_data, uint8_t a_len) {
    uint8_t tx_tmp[FTC_TX_MAX];
    uint8_t rx_tmp[6];
    if (7 + a_len > sizeof(tx_tmp)) {
      return false;
    }
    const size_t len_tmp = mrd_ftc_packet_make_write(tx_tmp, a_id, a_addr, a_data, a_len);
    return m_bus.transfer(tx_tmp, len_tmp, rx_tmp, 6) && mrd_ftc_reply_check(rx_tmp, 6, a_id) && rx_tmp[4] == 0; // [4]: サーボのエラー
  }

  /// @brief 半二重UARTバス(並行送受信用).
  MrdHalfDuplexUart &bus() { return m_bus; }

  /// @brief 通信速度bps.
  unsigned long baud() const { return m_baud; }

  /// @brief フレームをまたいで保持する状態(トルク状態の記憶等).
  MrdServoLineState m_state;

private:
  /// @brief 応答遅延時間(EEPROM)を0にする. EEPROMのロックを解除して書き込み, 再びロックする.
  bool fix_return_delay(uint8_t a_id, int a_type) {
    const uint8_t lock_addr_tmp = (a_type == FTCSCS) ? FTC_REG_LOCK_SCS : FTC_REG_LOCK_STS;
    const uint8_t unlock_tmp = 0;
    const uint8_t lock_tmp = 1;
    const uint8_t delay_tmp = 0;
    bool ok_tmp = write(a_id, lock_addr_tmp, &unlock_tmp, 1);
    ok_tmp = ok_tmp && write(a_id, FTC_REG_RETURN_DELAY, &delay_tmp, 1);
    write(a_id, lock_addr_tmp, &lock_tmp, 1); // 失敗しても必ずロックを戻す
    delay(10);                                // EEPROMの書き込み完了待ち
    uint8_t check_tmp = 0xFF;
    return ok_tmp && read(a_id, FTC_REG_RETURN_DELAY, &check_tmp, 1) && check_tmp == 0;
  }

  MrdHalfDuplexUart &m_bus;
  unsigned long m_baud;
};

//==================================================================================================
//  FEETECHサーボの駆動処理
//==================================================================================================

/// @brief 系統のFEETECHサーボクラス.
inline FeetechHalfDuplexClass &mrd_ftc_line(UartLine a_line) { return (a_line == L) ? ftc_l : ftc_r; }

/// @brief 系統の参照を作る.
/// @param a_line 系統(L or R).
/// @param a_meridim Meridimデータの参照.
/// @param a_sv サーボパラメータ.
MrdServoLineRef mrd_ftc_line_ref(UartLine a_line, Meridim90Union &a_meridim, ServoParam &a_sv) {
  return mrd_servo_line_ref(a_line, FTCSTS, mrd_ftc_line(a_line).bus(), mrd_ftc_line(a_line).m_state, a_meridim, a_sv);
}

//------------------------------------------------------------------------------------
//  パラメータ系コマンド (番号と値の形式は tasks/servo_param_plan.md)
//------------------------------------------------------------------------------------

/// @brief パラメータ系コマンドの対応表. 2バイト値のバイト順は系列に従う(STS:LE, SCS:BE).
///        EEPROM領域はロック解除 → 書き込み → 再ロックで保存まで行う(トルクの状態は問わない).
/// @param a_cmd コマンド.
/// @param a_type サーボタイプ(61:STS, 62:SCS).
/// @return レジスタの定義(len=0:未対応).
MrdParamReg mrd_ftc_param_reg(uint8_t a_cmd, int a_type) {
  const bool sts_tmp = (a_type != FTCSCS);
  const float step_deg_tmp = mrd_ftc_step_deg(a_type); // 1ステップの角度(deg)
  const uint8_t pos_sign_tmp = sts_tmp ? PC_BIT15 : PC_UNSIGNED;
  switch (a_cmd) {
  case 4:
  case 5:
    return mrd_pc_reg(FTC_REG_GOAL_POSITION, 2, a_cmd == 4, PC_POS, 1.0f, pos_sign_tmp);
  case 6:
    return mrd_pc_reg(FTC_REG_PRESENT_POSITION, 2, false, PC_POS, 1.0f, pos_sign_tmp);
  case 7:
    return mrd_pc_reg(62, 1, false, PC_SCALE, 10.0f); // 0.1V → V x100
  case 8:
  case 9: // 位置オフセット(STSのみ, bit11が符号)
    return sts_tmp ? mrd_pc_reg(31, 2, a_cmd == 9, PC_SCALE, step_deg_tmp * 100.0f, PC_BIT11, true) : MrdParamReg();
  case 10:
  case 11: // CW/CCW不感帯(同じ値を書く)
    return mrd_pc_reg(26, 1, a_cmd == 11, PC_SCALE, step_deg_tmp * 100.0f, PC_UNSIGNED, true, true);
  case 12:
  case 13:
    return mrd_pc_reg(9, 2, a_cmd == 13, PC_POS_NATIVE, 1.0f, PC_UNSIGNED, true); // 最小角度(CW側)
  case 14:
  case 15:
    return mrd_pc_reg(11, 2, a_cmd == 15, PC_POS_NATIVE, 1.0f, PC_UNSIGNED, true); // 最大角度(CCW側)
  case 16:
  case 17:
    return mrd_pc_reg(21, 1, a_cmd == 17, PC_RAW, 1.0f, PC_UNSIGNED, true);
  case 18:
  case 19:
    return mrd_pc_reg(23, 1, a_cmd == 19, PC_RAW, 1.0f, PC_UNSIGNED, true);
  case 20:
  case 21:
    return mrd_pc_reg(22, 1, a_cmd == 21, PC_RAW, 1.0f, PC_UNSIGNED, true);
  case 22:
  case 23:
    return mrd_pc_reg(13, 1, a_cmd == 23, PC_RAW, 1.0f, PC_UNSIGNED, true);
  case 24:
  case 25: // 保護電流(STSのみ, 6.5mA)
    return sts_tmp ? mrd_pc_reg(28, 2, a_cmd == 25, PC_SCALE, 6.5f, PC_UNSIGNED, true) : MrdParamReg();
  case 26: // 現在速度(ステップ/秒 → deg/s. STS/SCSともbit15が符号)
    return mrd_pc_reg(FTC_REG_PRESENT_SPEED, 2, false, PC_SCALE, step_deg_tmp, PC_BIT15);
  case 27: // 目標速度
    return mrd_pc_reg(FTC_REG_GOAL_SPEED, 2, true, PC_SCALE, step_deg_tmp, pos_sign_tmp);
  case 30:
  case 31: // STS: トルク制限(SRAM, 0.1%), SCS: 最大トルク(EEPROM)
    return sts_tmp ? mrd_pc_reg(48, 2, a_cmd == 31, PC_SCALE, 0.1f)
                   : mrd_pc_reg(16, 2, a_cmd == 31, PC_SCALE, 0.1f, PC_UNSIGNED, true);
  case 32:
  case 33:
    return mrd_pc_reg(FTC_REG_GOAL_TIME, 2, a_cmd == 33);
  case 36:
    return mrd_pc_reg(63, 1, false);
  case 37: // 現在電流(STSのみ, 6.5mA)
    return sts_tmp ? mrd_pc_reg(69, 2, false, PC_SCALE, 6.5f, PC_BIT15) : MrdParamReg();
  case 38:
    return mrd_pc_reg(FTC_REG_TORQUE_ENABLE, 1, false, PC_BOOL);
  case 54:
  case 55: // 最小起動力(パンチ, 0.1%)
    return mrd_pc_reg(24, 2, a_cmd == 55, PC_RAW, 1.0f, PC_UNSIGNED, true);
  case 56:
  case 57: // CW/CCW不感帯(ステップ)
    return mrd_pc_reg(26, 1, a_cmd == 57, PC_RAW, 1.0f, PC_UNSIGNED, true, true);
  case 70:
  case 71:
    return mrd_pc_reg(FTC_REG_RETURN_DELAY, 1, a_cmd == 71, PC_RAW, 1.0f, PC_UNSIGNED, true);
  default:
    return MrdParamReg();
  }
}

/// @brief FEETECHのパラメータ系コマンド1件分の処理.
///        3:PING, 35:ROM保存(EEPROMは書き込みごとに再ロックで保存しているため成功を返す),
///        読み出しはREAD 1回, RAMの書き込みはWRITE 1回, EEPROMの書き込みはロック解除 → WRITE → 再ロックの3回.
///        書き込みの結果は受け取ったVAL(サーボがエラーを返した場合はエラー値).
class FtcParamOp : public MrdParamOpBase {
public:
  /// @brief 車輪(車輪モードを事前に設定したサーボ): トルクON(必要なときだけ) → 目標速度(46) → 現在速度(58).
  ///        速度は0.01rpm(回転方向補正を適用). STSはbit15, SCSはbit10が回転方向. 結果は現在速度.
  void begin_wheel(const MrdServoLineRef &a_line, int a_ix, int16_t a_speed) {
    target(a_line, a_ix, a_speed);
    m_type = int(a_line.type[a_ix]);
    m_wheel = true;
    m_step = wheel_needs_torque() ? W_TORQUE : W_GOAL;
  }

  void begin(const MrdServoLineRef &a_line, int a_ix, uint8_t a_cmd, int16_t a_val) {
    target(a_line, a_ix, a_val);
    m_type = int(a_line.type[a_ix]);
    m_scale = mrd_ftc_scale(m_type);
    if (a_cmd == 3) {
      m_step = PING;
      return;
    }
    if (a_cmd == 35) {
      finish(1);
      return;
    }
    m_reg = mrd_ftc_param_reg(a_cmd, m_type);
    if (m_reg.len == 0) {
      finish(MRD_PCMD_NA);
      return;
    }
    if (!write_value_ok()) { // レジスタに収まらない値は送らない
      finish(MRD_PCMD_NA);
      return;
    }
    m_step = !m_reg.write ? READ : (m_reg.eeprom ? UNLOCK : WRITE);
  }

  bool next(MrdHduartXfer &a_x) {
    size_t len_tmp = 0;
    size_t rx_len_tmp = 6;
    const uint8_t lock_addr_tmp = (m_type == FTCSCS) ? FTC_REG_LOCK_SCS : FTC_REG_LOCK_STS;
    switch (m_step) {
    case W_TORQUE: {
      const uint8_t on_tmp = 1;
      len_tmp = mrd_ftc_packet_make_write(m_tx, id(), FTC_REG_TORQUE_ENABLE, &on_tmp, 1);
      break;
    }
    case W_GOAL: {
      const long max_tmp = (m_type == FTCSCS) ? 1023L : 32767L; // SCS: bit10, STS: bit15 が回転方向
      const long spd_tmp = constrain(lroundf(mrd_wheel_rpm(m_val) * 6.0f * m_ln->cw[m_ix] / mrd_ftc_step_deg(m_type)), -max_tmp, max_tmp);
      uint8_t data_tmp[2];
      mrd_ftc_u16_put(data_tmp, uint16_t(mrd_pc_encode(spd_tmp, m_type == FTCSCS ? PC_BIT10 : PC_BIT15)), m_type);
      len_tmp = mrd_ftc_packet_make_write(m_tx, id(), FTC_REG_GOAL_SPEED, data_tmp, 2);
      break;
    }
    case W_READ:
      len_tmp = mrd_ftc_packet_make_read(m_tx, id(), FTC_REG_PRESENT_SPEED, 2);
      rx_len_tmp += 2;
      break;
    case PING:
      len_tmp = mrd_ftc_packet_make_ping(m_tx, id());
      break;
    case READ:
      len_tmp = mrd_ftc_packet_make_read(m_tx, id(), m_reg.addr, m_reg.len);
      rx_len_tmp += m_reg.len;
      break;
    case UNLOCK:
    case LOCK: {
      const uint8_t lock_tmp = (m_step == LOCK) ? 1 : 0;
      len_tmp = mrd_ftc_packet_make_write(m_tx, id(), lock_addr_tmp, &lock_tmp, 1);
      break;
    }
    case WRITE: {
      uint8_t data_tmp[2];
      const uint32_t raw_tmp = mrd_pc_encode(to_raw(), m_reg.sign);
      if (m_reg.len == 2) {
        mrd_ftc_u16_put(data_tmp, uint16_t(raw_tmp), m_type);
      } else {
        data_tmp[0] = uint8_t(raw_tmp);
        data_tmp[1] = uint8_t(raw_tmp); // pair: 続くアドレスにも同じ値
      }
      len_tmp = mrd_ftc_packet_make_write(m_tx, id(), m_reg.addr, data_tmp, m_reg.pair ? 2 : m_reg.len);
      break;
    }
    default:
      return false;
    }
    mrd_servo_xfer(a_x, m_ln->bus, m_tx, len_tmp, m_rx, rx_len_tmp);
    return true;
  }

  void complete(const MrdHduartXfer &a_x) {
    const bool ok_tmp = a_x.ok && mrd_ftc_reply_check(m_rx, a_x.rx_len, id());
    const bool err_tmp = ok_tmp && m_rx[4] != 0;
    if (m_wheel && (!ok_tmp || err_tmp)) {
      m_ln->st->torque[m_ix] = -1;
      finish(!ok_tmp ? MRD_PCMD_NO_REPLY : MRD_PCMD_SERVO_ERR);
      return;
    }
    switch (m_step) {
    case W_TORQUE:
      m_ln->st->torque[m_ix] = 1;
      m_step = W_GOAL;
      break;
    case W_GOAL:
      m_step = W_READ;
      break;
    case W_READ:
      finish(mrd_wheel_val(mrd_pc_decode(rx_u16(), 2, PC_BIT15) * mrd_ftc_step_deg(m_type) / 6.0f * m_ln->cw[m_ix]));
      break;
    case PING:
      finish(!ok_tmp ? MRD_PCMD_NO_REPLY : 1);
      break;
    case READ:
      finish(!ok_tmp ? MRD_PCMD_NO_REPLY : (err_tmp ? MRD_PCMD_SERVO_ERR : to_val(get_raw())));
      break;
    case UNLOCK:
      if (!ok_tmp || err_tmp) {
        finish(!ok_tmp ? MRD_PCMD_NO_REPLY : MRD_PCMD_SERVO_ERR);
      } else {
        m_step = WRITE;
      }
      break;
    case WRITE:
      m_result = !ok_tmp ? MRD_PCMD_NO_REPLY : (err_tmp ? MRD_PCMD_SERVO_ERR : m_val);
      if (m_reg.eeprom) {
        m_step = LOCK; // 失敗しても再ロックする
      } else {
        finish(m_result);
      }
      break;
    case LOCK:
      // 書き込みの結果を返す. ただし書き込みは成功して再ロックだけ失敗した場合は, サーボのエラーとする
      finish((!mrd_servo_param_error(m_result) && !(ok_tmp && !err_tmp)) ? int16_t(MRD_PCMD_SERVO_ERR) : m_result);
      break;
    default:
      break;
    }
  }

private:
  enum Step : uint8_t { PING = 1,
                        READ,
                        UNLOCK,
                        WRITE,
                        LOCK,
                        W_TORQUE,
                        W_GOAL,
                        W_READ };

  /// @brief READの応答の2バイト値(系列のバイト順で読む).
  uint16_t rx_u16() const { return (m_type == FTCSCS) ? uint16_t((m_rx[5] << 8) | m_rx[6]) : uint16_t(m_rx[5] | (m_rx[6] << 8)); }

  /// @brief READの応答からレジスタの値を取り出す(符号を解いた値).
  long get_raw() const { return mrd_pc_decode((m_reg.len == 2) ? rx_u16() : m_rx[5], m_reg.len, m_reg.sign); }

  int m_type = FTCSTS;
  bool m_wheel = false;
  uint8_t m_tx[12];
  uint8_t m_rx[8];
};

/// @brief FEETECHサーボ1系統分の1フレームの処理を, 1回ずつの送受信に分けて進める.
///        コマンド1のサーボは目標位置へ動かし, それ以外は脱力する. いずれも現在位置を読み取ってtgtに格納する.
///        [1] 目標位置(STS) → [2] 目標位置(SCS) → [3] トルク → [4] 各サーボの現在位置 の順に送る.
///        STSとSCSが混在する系統では, [4]はフレームごとにSTSとSCSを交互に読む(読まない系列は前回の値).
class FtcLineStepper : public MrdParamLineStepper<FtcParamOp> {
public:
  /// @param a_line 系統の参照.
  /// @param a_all_mount パラメータ系コマンドのスロットを除かないマウント状態(パラメータ系コマンドの実行用).
  /// @param a_num 処理するサーボ数.
  FtcLineStepper(const MrdServoLineRef &a_line, const int *a_all_mount, int a_num)
      : MrdParamLineStepper<FtcParamOp>(a_line, a_all_mount, a_num) {}

  void skip_reads() override { m_skip_reads = true; }

protected:
  bool next_main(MrdHduartXfer &a_x) override {
    while (m_phase != DONE) {
      size_t len_tmp = 0;
      const uint8_t *tx_tmp = m_pkt.buf;
      m_reading = false;
      switch (m_phase) {
      case GOAL_STS:
        len_tmp = build_goal(FTCSTS);
        m_phase = GOAL_SCS;
        break;
      case GOAL_SCS:
        len_tmp = build_goal(FTCSCS);
        m_phase = TORQUE;
        break;
      case TORQUE:
        len_tmp = build_torque();
        m_phase = READ;
        select_read_series();
        keep_unread(); // 目標位置を送った後に行う(tgtを前回の値にするため)
        break;
      case READ:
        do {
          m_ix++;
        } while (m_ix < m_num && (!m_ln.mount[m_ix] || (m_read_series != 0 && int(m_ln.type[m_ix]) != m_read_series)));
        if (m_ix >= m_num) {
          m_phase = DONE;
          break;
        }
        len_tmp = mrd_ftc_packet_make_read(m_tx, m_ln.id[m_ix], FTC_REG_PRESENT_POSITION, 2);
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
    const int type_tmp = int(m_ln.type[m_ix]);
    mrd_servo_read_apply(m_ln, m_ix, valid_tmp,
                         valid_tmp ? mrd_ftc_pos2deg(mrd_ftc_pos_get(&m_rx[5], type_tmp), m_ln.trim[m_ix], m_ln.cw[m_ix], type_tmp)
                                   : 0.0f);
  }

private:
  enum Phase : uint8_t { GOAL_STS,
                         GOAL_SCS,
                         TORQUE,
                         READ,
                         DONE };

  /// @brief 目標位置のSyncWriteを作る(系列ごとに1パケット. STS:位置, SCS:位置+時間+速度).
  /// @return パケット長(対象のサーボがなければ0).
  size_t build_goal(int a_series) {
    m_pkt.start(FTC_REG_GOAL_POSITION, (a_series == FTCSCS) ? 6 : 2);
    for (int i = 0; i < m_num; i++) {
      if (!m_ln.mount[i] || !m_ln.on(i) || int(m_ln.type[i]) != a_series) {
        continue;
      }
      uint8_t *slot_tmp = m_pkt.add(m_ln.id[i]);
      if (slot_tmp == nullptr) {
        continue;
      }
      mrd_ftc_u16_put(slot_tmp, mrd_ftc_deg2pos(m_ln.tgt[i], m_ln.trim[i], m_ln.cw[i], a_series), a_series);
      if (a_series == FTCSCS) {
        mrd_ftc_u16_put(&slot_tmp[2], 0, a_series);                       // 目標時間(0:速度で制御)
        mrd_ftc_u16_put(&slot_tmp[4], cfg.servo_ftc_scs_speed, a_series); // 目標速度
      }
    }
    return m_pkt.finish();
  }

  /// @brief トルクのSyncWriteを作る. ON/OFFが変わったサーボだけ送る.
  ///        ただし MRD_SERVO_TORQUE_REFRESH_FRAMES ごとに全サーボへ送り直す
  ///        (サーボの保護機能でトルクが切れると, ESP32側の記憶と食い違ったまま脱力し続けるため).
  ///        ONは目標位置を送った後に行い, 古い目標位置へ動くのを防ぐ.
  /// @return パケット長(対象のサーボがなければ0).
  size_t build_torque() {
    m_pkt.start(FTC_REG_TORQUE_ENABLE, 1);
    mrd_servo_torque_each(m_ln, m_num, [this](int a_ix, int8_t a_want) {
      uint8_t *slot_tmp = m_pkt.add(m_ln.id[a_ix]);
      if (slot_tmp != nullptr) {
        *slot_tmp = a_want;
      }
      return slot_tmp != nullptr;
    });
    return m_pkt.finish();
  }

  /// @brief このフレームで現在位置を読む系列を決める. STSとSCSが混在する場合だけ, フレームごとに交互にする.
  ///        読まないフレーム(skip_reads)はどの系列も読まず, 交互の順番も進めない.
  void select_read_series() {
    if (m_skip_reads) {
      m_read_series = FTC_READ_NONE;
      return;
    }
    bool sts_tmp = false;
    bool scs_tmp = false;
    for (int i = 0; i < m_num; i++) {
      if (!m_ln.mount[i]) {
        continue;
      }
      if (int(m_ln.type[i]) == FTCSCS) {
        scs_tmp = true;
      } else {
        sts_tmp = true;
      }
    }
    if (!sts_tmp || !scs_tmp) {
      m_read_series = 0; // 一方の系列だけ: 毎フレーム全サーボを読む
      return;
    }
    m_read_series = (m_ln.st->read_cursor == 0) ? FTCSTS : FTCSCS;
    m_ln.st->read_cursor = (m_ln.st->read_cursor == 0) ? 1 : 0;
  }

  /// @brief このフレームで現在位置を読まない系列のサーボを前回の値にする.
  void keep_unread() {
    for (int i = 0; i < m_num; i++) {
      if (m_read_series != 0 && m_ln.mount[i] && int(m_ln.type[i]) != m_read_series) {
        mrd_servo_past_keep(m_ln, i);
      }
    }
  }

  int m_read_series = 0;     // このフレームで現在位置を読む系列(FTCSTS/FTCSCS. 0:全サーボ, FTC_READ_NONE:読まない)
  bool m_skip_reads = false; // このフレームは現在位置を読まないか(混在時の交互読み出し, マスターコマンド2)
  Phase m_phase = GOAL_STS;
  int m_ix = -1;          // 現在位置を読んでいるサーボのインデックス
  bool m_reading = false; // 直前に準備したのがREADか
  FtcSyncPacket m_pkt;    // SyncWriteパケット
  uint8_t m_tx[8];        // READパケット
  uint8_t m_rx[8];        // READの応答(位置2バイト)
};

#endif // __MERIDIAN_SERVO_FEETECH_H__
