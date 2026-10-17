#ifndef __MERIDIAN_SERVO_UTIL_H__
#define __MERIDIAN_SERVO_UTIL_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバス, 送受信, ステッパー. 本ファイルは各プロトコル層(sv_*.h)の先頭で読み込む

//==================================================================================================
//  サーボの各プロトコル層(sv_*.h)の共通部品
//==================================================================================================
//
// 1. 角度と位置値の変換        : どのサーボも「中央からの角度 x 1degあたりの位置値」で位置を表す
// 2. 起動時の共通処理          : 接続の表示, 通信速度の確認
// 3. 系統の状態と参照          : 1系統分のサーボの設定(ID, トリム等)と状態をまとめて扱う
// 4. パラメータ系コマンドの値  : サーボのレジスタの値と, Meridimで送るVAL(int16)の換算
// 5. パラメータ系コマンドの実行: 1件ずつの手順(MrdParamOpBase)と, それを通常の処理の後に行うステッパー
//
// 略語: PC = パラメータ系コマンド(Parameter Command. Meridimのサーボコマンドの下位8bitが3以上).

//------------------------------------------------------------------------------------
//  1. 角度と位置値の変換
//------------------------------------------------------------------------------------

/// @brief 角度(degree)を位置値に変換する.
/// @param a_degree 角度(degree).
/// @param a_trim トリム値(degree).
/// @param a_cw 回転方向補正値(+1 or -1).
/// @param a_k 1degあたりの位置値.
/// @param a_center 0degに対応する位置値.
/// @param a_min 位置値の最小値.
/// @param a_max 位置値の最大値.
/// @return 位置値.
inline int mrd_servo_deg2pos(float a_degree, float a_trim, int a_cw, float a_k, float a_center, float a_min,
                             float a_max) {
  float x_tmp = a_center + (a_trim * a_k) + (a_degree * a_k * a_cw);
  if (x_tmp > a_max) {
    x_tmp = a_max;
  } else if (x_tmp < a_min) {
    x_tmp = a_min;
  }
  return static_cast<int>(lroundf(x_tmp));
}

/// @brief 位置値を角度(degree)に変換する. mrd_servo_deg2pos の逆変換.
inline float mrd_servo_pos2deg(int a_pos, float a_trim, int a_cw, float a_k, float a_center) {
  return (a_pos - a_center - (a_trim * a_k)) / a_k * a_cw;
}

/// @brief サーボの位置値の尺度. プロトコルごと(機種ごと)に1つ決める.
struct MrdPosScale {
  float per_deg; // 1degあたりの位置値
  float center;  // 0degに対応する位置値
  float min;     // 位置値の最小値
  float max;     // 位置値の最大値
};

/// @brief 尺度を使って角度(degree)を位置値にする.
inline int mrd_servo_scale_deg2pos(float a_degree, float a_trim, int a_cw, const MrdPosScale &a_s) {
  return mrd_servo_deg2pos(a_degree, a_trim, a_cw, a_s.per_deg, a_s.center, a_s.min, a_s.max);
}

/// @brief 尺度を使って位置値を角度(degree)にする.
inline float mrd_servo_scale_pos2deg(int a_pos, float a_trim, int a_cw, const MrdPosScale &a_s) {
  return mrd_servo_pos2deg(a_pos, a_trim, a_cw, a_s.per_deg, a_s.center);
}

/// @brief CRC16(左送り, 初期値0, 反転なし)を計算する. 多項式はプロトコルで違う(DYNAMIXEL 2.0: 0x8005, PMX: 0x1021).
inline uint16_t mrd_servo_crc16(const uint8_t *a_data, size_t a_len, uint16_t a_poly) {
  uint16_t crc_tmp = 0;
  for (size_t i = 0; i < a_len; i++) {
    crc_tmp ^= uint16_t(a_data[i]) << 8;
    for (int b = 0; b < 8; b++) {
      crc_tmp = (crc_tmp & 0x8000) ? uint16_t((crc_tmp << 1) ^ a_poly) : uint16_t(crc_tmp << 1);
    }
  }
  return crc_tmp;
}

//------------------------------------------------------------------------------------
//  2. 起動時の共通処理
//------------------------------------------------------------------------------------

#define MRD_SERVO_BOOT_RETRY 3 // 起動時にサーボの応答を確認する試行回数

/// @brief 起動時の1台分の表示の先頭 "UART_L XXX [i] ID n: " を出す(続けて結果を表示する).
/// @param a_line_name 系統名("L"など).
/// @param a_proto プロトコル名("DXL2"など).
/// @param a_ix スロット番号.
/// @param a_id サーボのID.
inline void mrd_servo_slot_print(const char *a_line_name, const char *a_proto, int a_ix, int a_id) {
  Serial.print("UART_");
  Serial.print(a_line_name);
  Serial.print(" ");
  Serial.print(a_proto);
  Serial.print(" [");
  Serial.print(a_ix);
  Serial.print("] ID ");
  Serial.print(a_id);
  Serial.print(": ");
}

/// @brief 通信速度がサーボで設定できる値かを確認し, 違えば警告を表示する.
/// @param a_line_name 系統名.
/// @param a_proto プロトコル名.
/// @param a_baud 確認する通信速度bps.
/// @param a_bauds 設定できる通信速度の一覧.
/// @param a_num 一覧の数.
/// @param a_hint 警告の後半(" bps is not ..." に続く案内).
/// @return 設定できる値ならtrue.
inline bool mrd_servo_baud_check(const char *a_line_name, const char *a_proto, unsigned long a_baud,
                                 const unsigned long *a_bauds, size_t a_num, const char *a_hint) {
  for (size_t i = 0; i < a_num; i++) {
    if (a_bauds[i] == a_baud) {
      return true;
    }
  }
  Serial.print("UART_");
  Serial.print(a_line_name);
  Serial.print(" ");
  Serial.print(a_proto);
  Serial.print(": ");
  Serial.print(a_baud);
  Serial.println(a_hint);
  return false;
}

//------------------------------------------------------------------------------------
//  3. 系統の状態と参照
//------------------------------------------------------------------------------------

#define MRD_SERVO_TORQUE_REFRESH_FRAMES 20 // トルク状態を変化がなくても送り直す間隔(フレーム数)
                                           // サーボの保護機能(過負荷・過熱等)でトルクが切れても復帰させるため

/// @brief 1系統分のサーボの状態のうち, フレームをまたいで保持するもの.
struct MrdServoLineState {
  int8_t torque[MRD_SERVO_SLOTS]; // 各サーボに最後に送ったトルク状態(-1:不明, 0:OFF, 1:ON). 不明なら次のフレームで送る
  uint8_t torque_refresh = 0;     // トルク状態を送り直すまでのフレーム数のカウンタ
  uint8_t read_cursor = 0;        // 順番に読み出す場合の次の位置(サーボのインデックス, またはFEETECHの読む系列)

  void reset() {
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      torque[i] = -1;
    }
    torque_refresh = 0;
    read_cursor = 0;
  }

  /// @brief 1フレームに1回呼び, 全サーボへトルク状態を送り直すフレームならtrueを返す.
  bool torque_tick() {
    if (++torque_refresh >= MRD_SERVO_TORQUE_REFRESH_FRAMES) {
      torque_refresh = 0;
      return true;
    }
    return false;
  }
};

/// @brief ServoParamのうち1系統分の参照をまとめたもの.
struct MrdServoLineRef {
  MrdHalfDuplexUart *bus; // この系統の半二重UARTバス
  MrdServoLineState *st;  // この系統のプロトコル層が保持する状態
  const int *mount;       // 通常の処理をするスロット(パラメータ系コマンド等のスロットは除く)
  const float *type;
  const int *id;
  const int *cw;
  const float *trim;
  float *tgt;
  const float *tgt_past;
  int *err;
  uint16_t *stat;
  const short *meridim; // この系統のMeridim配列の先頭(コマンドは[i*2])
  int8_t line;          // 系統(L or R. パラメータ系コマンドの状態 mrd_servo_param の添字)

  /// @brief サーボのトルクONが指示されているか(コマンドの下位8bitが1).
  bool on(int a_ix) const { return mrd_servo_cmd(meridim[a_ix * 2]) == 1; }

  /// @brief トルク状態を送る必要があるかを判定する. 送る場合は a_want に送る値(0/1)を入れる.
  ///        送ったら st->torque[a_ix] = a_want として記憶すること.
  /// @param a_ix サーボのインデックス.
  /// @param a_refresh 全サーボへ送り直すフレームか(MrdServoLineState::torque_tick()).
  bool torque_due(int a_ix, bool a_refresh, int8_t &a_want) const {
    a_want = on(a_ix) ? 1 : 0;
    return a_refresh || st->torque[a_ix] != a_want;
  }

  /// @brief a_ix より後ろで, 次に処理するスロットを探す.
  /// @return スロット番号. なければ a_num 以上.
  int next_mounted(int a_ix, int a_num) const {
    do {
      a_ix++;
    } while (a_ix < a_num && !mount[a_ix]);
    return a_ix;
  }
};

/// @brief トルク状態を送る必要があるサーボ(ON/OFFが変わったサーボ. 定期的な送り直しのフレームでは全サーボ)ごとに,
///        a_put(スロット, 送る値0/1) を呼ぶ. a_put が true を返した(パケットに入れた)サーボは, その値を記憶する.
/// @return パケットに入れたサーボの数.
template <class F>
int mrd_servo_torque_each(const MrdServoLineRef &a_ln, int a_num, F a_put) {
  const bool refresh_tmp = a_ln.st->torque_tick();
  int count_tmp = 0;
  for (int i = 0; i < a_num; i++) {
    int8_t want_tmp;
    if (a_ln.mount[i] && a_ln.torque_due(i, refresh_tmp, want_tmp) && a_put(i, want_tmp)) {
      a_ln.st->torque[i] = want_tmp;
      count_tmp++;
    }
  }
  return count_tmp;
}

/// @brief 系統の参照を作る.
/// @param a_line 系統(L or R).
/// @param a_type サーボの種類(1系統に2種類を混在させる場合は, その種類のスロットだけを対象にする).
/// @param a_bus この系統の半二重UARTバス.
/// @param a_st この系統のプロトコル層が保持する状態.
/// @param a_meridim Meridimデータの参照.
/// @param a_sv サーボパラメータ.
MrdServoLineRef mrd_servo_line_ref(UartLine a_line, int a_type, MrdHalfDuplexUart &a_bus, MrdServoLineState &a_st,
                                   Meridim90Union &a_meridim, ServoParam &a_sv) {
  const int *mount_tmp = mrd_servo_line_mount(a_line, a_type, a_sv); // パラメータ系コマンド等のスロットを除く
  if (a_line == L) {
    return {&a_bus, &a_st, mount_tmp, a_sv.ixl_type, a_sv.ixl_id, a_sv.ixl_cw, a_sv.ixl_trim, a_sv.ixl_tgt,
            a_sv.ixl_tgt_past, a_sv.ixl_err, a_sv.ixl_stat, &a_meridim.sval[MRD_L_ORIGIDX], int8_t(L)};
  }
  return {&a_bus, &a_st, mount_tmp, a_sv.ixr_type, a_sv.ixr_id, a_sv.ixr_cw, a_sv.ixr_trim, a_sv.ixr_tgt,
          a_sv.ixr_tgt_past, a_sv.ixr_err, a_sv.ixr_stat, &a_meridim.sval[MRD_R_ORIGIDX], int8_t(R)};
}

/// @brief 現在位置の読み取り結果を反映する.
/// @param a_line 系統の参照.
/// @param a_ix サーボのインデックス.
/// @param a_ok 現在位置を読み取れたか.
/// @param a_degree 読み取った角度(degree). a_okがfalseなら使わない.
void mrd_servo_read_apply(const MrdServoLineRef &a_line, int a_ix, bool a_ok, float a_degree) {
  if (a_ok) {
    a_line.tgt[a_ix] = a_degree;
    a_line.err[a_ix] = 0;
    a_line.stat[a_ix] = 0;
    return;
  }
  a_line.tgt[a_ix] = a_line.tgt_past[a_ix]; // 返信を受け取れなかった場合は前回の値
  a_line.st->torque[a_ix] = -1;             // 再起動していてもトルク状態を送り直せるよう不明にする
  a_line.err[a_ix]++;
  if (a_line.err[a_ix] >= cfg.servo_lost_err_wait) { // 一定以上の連続エラーで通信不能とみなす
    a_line.err[a_ix] = cfg.servo_lost_err_wait;
    a_line.stat[a_ix] = 1;
  }
}

/// @brief このフレームで現在位置を読まないサーボの値を前回の値にする(エラー数は変えない).
inline void mrd_servo_past_keep(const MrdServoLineRef &a_line, int a_ix) {
  a_line.tgt[a_ix] = a_line.tgt_past[a_ix];
}

/// @brief 全サーボの値を前回の値にする(現在位置を1台も読まないフレーム).
inline void mrd_servo_past_keep_all(const MrdServoLineRef &a_line, int a_num) {
  for (int i = 0; i < a_num; i++) {
    if (a_line.mount[i]) {
      mrd_servo_past_keep(a_line, i);
    }
  }
}

/// @brief 1回の送受信を準備する. 指定しない項目(hold_us, framed)は既定値のまま.
inline void mrd_servo_xfer(MrdHduartXfer &a_x, MrdHalfDuplexUart *a_bus, const uint8_t *a_tx, size_t a_tx_len,
                           uint8_t *a_rx, size_t a_rx_len, unsigned long a_timeout_ms = 0) {
  a_x.bus = a_bus;
  a_x.tx = a_tx;
  a_x.tx_len = a_tx_len;
  a_x.rx = a_rx;
  a_x.rx_len = a_rx_len;
  a_x.timeout_ms = a_timeout_ms;
}

//------------------------------------------------------------------------------------
//  4. パラメータ系コマンドの値 (レジスタの値 ⇔ VAL)
//------------------------------------------------------------------------------------

/// @brief レジスタの値の符号の形式.
enum MrdPcSign : uint8_t {
  PC_UNSIGNED, // 符号なし
  PC_SIGNED,   // 2の補数(バイト数に応じて)
  PC_BIT15,    // bit15が符号, 残りが大きさ(FEETECH STSの位置・速度・電流)
  PC_BIT11,    // bit11が符号, 残りが大きさ(FEETECH STSの位置オフセット)
  PC_BIT10     // bit10が符号(CW), 残りが大きさ(DYNAMIXEL 1.0の速度)
};

/// @brief レジスタの値とVALの換算の種類.
enum MrdPcConv : uint8_t {
  PC_RAW,        // VAL = レジスタの値
  PC_SCALE,      // VAL = レジスタの値 x k
  PC_POS,        // 角度(deg x100). Meridianのトリム・回転方向補正を通す(目標・現在角度)
  PC_POS_NATIVE, // 角度(deg x100). サーボ自身の角度(トリム0, 回転方向+1で換算. 角度限界)
  PC_BOOL        // レジスタの値が1なら1, それ以外は0(トルク状態)
};

/// @brief パラメータ系コマンド1つ分のレジスタの定義. len が0なら未対応.
struct MrdParamReg {
  uint16_t addr;
  uint8_t len;  // バイト数(0:未対応)
  uint8_t sign; // MrdPcSign
  uint8_t conv; // MrdPcConv
  float k;      // PC_SCALE の倍率
  bool write;   // 書き込み(falseなら読み出し)
  bool eeprom;  // EEPROM領域(FEETECHはロックの解除と再ロックを行う)
  bool pair;    // 続くアドレスにも同じ値を書く(CW/CCWの不感帯など)

  constexpr MrdParamReg(uint16_t a_addr = 0, uint8_t a_len = 0, bool a_write = false, uint8_t a_conv = PC_RAW,
                        float a_k = 1.0f, uint8_t a_sign = PC_UNSIGNED, bool a_eeprom = false, bool a_pair = false)
      : addr(a_addr), len(a_len), sign(a_sign), conv(a_conv), k(a_k), write(a_write), eeprom(a_eeprom), pair(a_pair) {}
};

/// @brief レジスタの定義を作る(各プロトコルの対応表で使う. 引数は MrdParamReg のコンストラクタと同じ).
constexpr MrdParamReg mrd_pc_reg(uint16_t a_addr, uint8_t a_len, bool a_write, uint8_t a_conv = PC_RAW,
                                 float a_k = 1.0f, uint8_t a_sign = PC_UNSIGNED, bool a_eeprom = false,
                                 bool a_pair = false) {
  return MrdParamReg(a_addr, a_len, a_write, a_conv, a_k, a_sign, a_eeprom, a_pair);
}

/// @brief 符号ビットの形式(PC_BIT15/11/10)の符号ビットの位置. それ以外は-1.
inline int mrd_pc_sign_bit(uint8_t a_sign) {
  return (a_sign == PC_BIT15) ? 15 : (a_sign == PC_BIT11) ? 11
                                 : (a_sign == PC_BIT10)   ? 10
                                                          : -1;
}

/// @brief レジスタの値(リトルエンディアンで組み立てた値)を符号付きの整数にする.
inline long mrd_pc_decode(uint32_t a_raw, uint8_t a_len, uint8_t a_sign) {
  if (a_sign == PC_SIGNED) {
    return (a_len == 1) ? long(int8_t(a_raw)) : (a_len == 2) ? long(int16_t(a_raw))
                                                             : long(int32_t(a_raw));
  }
  const int bit_tmp = mrd_pc_sign_bit(a_sign);
  if (bit_tmp < 0) {
    return long(a_raw);
  }
  const uint32_t mag_tmp = a_raw & ((1u << bit_tmp) - 1); // 符号ビットより下が大きさ
  return (a_raw & (1u << bit_tmp)) ? -long(mag_tmp) : long(mag_tmp);
}

/// @brief 符号付きの整数をレジスタの値にする(mrd_pc_decode の逆).
inline uint32_t mrd_pc_encode(long a_val, uint8_t a_sign) {
  const int bit_tmp = mrd_pc_sign_bit(a_sign);
  if (bit_tmp < 0) {
    return uint32_t(a_val);
  }
  const uint32_t mask_tmp = (1u << bit_tmp) - 1;
  return (a_val < 0) ? ((1u << bit_tmp) | (uint32_t(-a_val) & mask_tmp)) : (uint32_t(a_val) & mask_tmp);
}

/// @brief レジスタの値(符号を付ける前)が, レジスタの幅(byte数)と符号の形式で表せるか.
inline bool mrd_pc_fits(long a_raw, uint8_t a_len, uint8_t a_sign) {
  const int bit_tmp = mrd_pc_sign_bit(a_sign);
  if (bit_tmp >= 0) { // 符号ビットの形式: 大きさが符号ビットより下に収まる
    return labs(a_raw) < (1L << bit_tmp);
  }
  if (a_len >= 4) {
    return a_sign == PC_SIGNED || a_raw >= 0;
  }
  const long span_tmp = 1L << (8 * a_len);
  return (a_sign == PC_SIGNED) ? (a_raw >= -span_tmp / 2 && a_raw < span_tmp / 2) : (a_raw >= 0 && a_raw < span_tmp);
}

/// @brief 結果をVALの範囲に収める(エラー値と重ならないよう -32765 以上にする).
inline int16_t mrd_pc_clamp(long a_val) { return int16_t(constrain(a_val, -32765L, 32767L)); }

// 車輪(bit13)の速度の単位: VAL 1 = 0.01rpm (rpm の100倍. DYNAMIXEL 2.0 の rpm 基準に合わせる). 正の向きは角度と同じ
/// @brief 車輪の速度のVALを rpm にする.
inline float mrd_wheel_rpm(int16_t a_val) { return a_val * 0.01f; }

/// @brief rpm を車輪の速度のVAL(0.01rpm)にする(結果の範囲に収める).
inline int16_t mrd_wheel_val(float a_rpm) { return mrd_pc_clamp(lroundf(a_rpm * 100.0f)); }

/// @brief バイト列(LE)から値を組み立てる.
inline uint32_t mrd_pc_le_get(const uint8_t *a_buf, uint8_t a_len) {
  uint32_t v_tmp = 0;
  for (int i = a_len - 1; i >= 0; i--) {
    v_tmp = (v_tmp << 8) | a_buf[i];
  }
  return v_tmp;
}

/// @brief 値をバイト列(LE)にする.
inline void mrd_pc_le_put(uint8_t *a_buf, uint32_t a_val, uint8_t a_len) {
  for (int i = 0; i < a_len; i++) {
    a_buf[i] = (a_val >> (8 * i)) & 0xFF;
  }
}

//------------------------------------------------------------------------------------
//  5. パラメータ系コマンドの実行
//------------------------------------------------------------------------------------

/// @brief パラメータ系コマンド1件分の処理の共通部分. 各プロトコルはこれを継承して次の関数を作る.
///          begin(ln, ix, cmd, val) : 対応表から手順を決める(通信しない場合は finish() で結果を決めて終える)
///          begin_wheel(ln, ix, 速度): 車輪の手順を決める(対応しないプロトコルは作らない → 下の既定で未対応)
///          next(x)                  : 次の送受信を準備する(なければfalse)
///          complete(x)              : 送受信の結果を反映する
///        テンプレート(MrdParamLineStepper<Op>)から呼ぶため virtual にはせず, 同じ名前の関数で置き換える.
class MrdParamOpBase {
public:
  int16_t result() const { return m_result; }

  /// @brief 車輪に対応しないプロトコルの begin_wheel (未対応で終える).
  void begin_wheel(const MrdServoLineRef &a_line, int a_ix, int16_t a_speed) {
    target(a_line, a_ix, a_speed);
    finish(MRD_PCMD_NA);
  }

protected:
  /// @brief 対象のサーボを記憶する(各プロトコルの begin() の最初に呼ぶ).
  ///        書き込みの成功では値をそのまま返すため, 値もエラー値と重ならない範囲に収める.
  void target(const MrdServoLineRef &a_line, int a_ix, int16_t a_val) {
    m_ln = &a_line;
    m_ix = a_ix;
    m_val = mrd_pc_clamp(a_val);
  }
  uint8_t id() const { return uint8_t(m_ln->id[m_ix]); }

  /// @brief 結果を決めて終える.
  void finish(int16_t a_result) {
    m_result = a_result;
    m_step = 0xFF;
  }
  bool finished() const { return m_step == 0xFF; }

  /// @brief 角度(deg)をVAL(deg x100)にする.
  static int16_t deg_to_val(float a_deg) { return mrd_pc_clamp(lroundf(a_deg * 100.0f)); }

  /// @brief レジスタの値(符号を解いた値)をVALにする. 角度は位置値の尺度 m_scale で換算する.
  int16_t to_val(long a_raw) const {
    switch (m_reg.conv) {
    case PC_POS:
      return deg_to_val(mrd_servo_scale_pos2deg(int(a_raw), m_ln->trim[m_ix], m_ln->cw[m_ix], m_scale));
    case PC_POS_NATIVE:
      return deg_to_val(mrd_servo_scale_pos2deg(int(a_raw), 0.0f, 1, m_scale));
    case PC_SCALE:
      return mrd_pc_clamp(lroundf(float(a_raw) * m_reg.k));
    case PC_BOOL:
      return (a_raw == 1) ? 1 : 0;
    default:
      return mrd_pc_clamp(a_raw);
    }
  }

  /// @brief 書き込む値がレジスタの幅と符号に収まるか(読み出しは常にtrue). 収まらない値は送らない(未対応を返す).
  bool write_value_ok() const { return !m_reg.write || mrd_pc_fits(to_raw(), m_reg.len, m_reg.sign); }

  /// @brief 受け取ったVALをレジスタの値(符号を付ける前の値)にする(to_val の逆).
  long to_raw() const {
    switch (m_reg.conv) {
    case PC_POS:
      return mrd_servo_scale_deg2pos(m_val * 0.01f, m_ln->trim[m_ix], m_ln->cw[m_ix], m_scale);
    case PC_POS_NATIVE:
      return mrd_servo_scale_deg2pos(m_val * 0.01f, 0.0f, 1, m_scale);
    case PC_SCALE:
      return (m_reg.k != 0.0f) ? lroundf(float(m_val) / m_reg.k) : long(m_val);
    default:
      return m_val;
    }
  }

  /// @brief 車輪: トルクONを送る必要があるか(状態が不明・OFFのとき, または通常の処理と同じ送り直しの間隔).
  bool wheel_needs_torque() const { return m_ln->st->torque[m_ix] != 1 || m_ln->st->torque_refresh == 0; }

  int16_t m_result = MRD_PCMD_NA;
  uint8_t m_step = 0; // プロトコルごとの手順の位置(0xFF:終了)
  const MrdServoLineRef *m_ln = nullptr;
  int m_ix = 0;
  int16_t m_val = 0;                                       // 受け取ったVAL(車輪は速度0.01rpm)
  MrdParamReg m_reg;                                       // 対応表のレジスタ
  MrdPosScale m_scale = {1.0f, 0.0f, -32768.0f, 32767.0f}; // 角度の換算に使う位置値の尺度
};

/// @brief パラメータ系コマンドに対応するステッパーの共通部分.
///        各プロトコルのステッパーは通常の処理を next_main()/complete_main() に書く. それが終わった後に,
///        実行待ちのスロット(パラメータ系コマンドは変わったフレームだけ, 車輪は毎フレーム)を1件ずつ Op で処理する.
/// @tparam Op 1件分の処理(MrdParamOpBase を継承したクラス).
template <class Op>
class MrdParamLineStepper : public MrdLineStepper {
public:
  /// @param a_line 系統の参照.
  /// @param a_all_mount パラメータ系コマンドのスロットを除かないマウント状態(パラメータ系コマンドの実行対象).
  /// @param a_num 処理するサーボ数.
  MrdParamLineStepper(const MrdServoLineRef &a_line, const int *a_all_mount, int a_num)
      : m_ln(a_line), m_num(min(a_num, MRD_SERVO_SLOTS)), m_prm_mount(a_all_mount) {}

  bool next(MrdHduartXfer &a_x) override {
    if (!m_prm_phase) {
      if (next_main(a_x)) {
        return true;
      }
      m_prm_phase = true; // 通常の処理が終わったのでパラメータ系コマンドへ
    }
    for (;;) {
      if (m_prm_busy && m_op.next(a_x)) {
        return true; // 実行中の1件の次の送受信
      }
      if (m_prm_busy) {
        finish_slot(); // 実行中の1件が終わった
      }
      if (!start_next_slot()) {
        return false; // 実行待ちのスロットはもうない
      }
    }
  }

  void complete(const MrdHduartXfer &a_x) override {
    if (m_prm_phase) {
      m_op.complete(a_x);
    } else {
      complete_main(a_x);
    }
  }

protected:
  virtual bool next_main(MrdHduartXfer &a_x) = 0;
  virtual void complete_main(const MrdHduartXfer &a_x) = 0;

  MrdServoLineRef m_ln; // 系統の参照(Op が処理中に参照する)
  int m_num;            // 処理するサーボ数

private:
  MrdServoParamSlot &slot(int a_ix) { return mrd_servo_param[m_ln.line][a_ix]; }

  /// @brief 結果をスロットに記録する.
  void finish_slot() {
    slot(m_prm_ix).result = m_op.result();
    slot(m_prm_ix).due = false;
    m_prm_busy = false;
  }

  /// @brief 次の実行待ちのスロットを探して1件分の処理を始める.
  /// @return 始めたらtrue. もうなければfalse.
  bool start_next_slot() {
    do {
      m_prm_ix++;
    } while (m_prm_ix < m_num && !(m_prm_mount[m_prm_ix] && slot(m_prm_ix).due));
    if (m_prm_ix >= m_num) {
      return false;
    }
    m_op = Op();
    const MrdServoParamSlot &s_tmp = slot(m_prm_ix);
    if (s_tmp.mode == MRD_SLOT_WHEEL) {
      m_op.begin_wheel(m_ln, m_prm_ix, s_tmp.val);
    } else {
      m_op.begin(m_ln, m_prm_ix, s_tmp.cmd, s_tmp.val);
    }
    m_prm_busy = true;
    return true;
  }

  const int *m_prm_mount;
  int m_prm_ix = -1;        // 処理中のスロット
  bool m_prm_phase = false; // 通常の処理を終えてパラメータ系コマンドを処理中か
  bool m_prm_busy = false;  // 1件分を処理中か
  Op m_op;
};

#endif // __MERIDIAN_SERVO_UTIL_H__
