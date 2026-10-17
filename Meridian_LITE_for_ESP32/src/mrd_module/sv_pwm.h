#ifndef __MERIDIAN_SERVO_PWM_H__
#define __MERIDIAN_SERVO_PWM_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバスと共通処理. 本ファイルは mrd_servo.h から読み込まれる
#include "sv_util.h"   // 各プロトコル層の共通部品
#include <Wire.h>

//==================================================================================================
//  PWMサーボ (本体ピン / I2C PCA9685)
//==================================================================================================
//
// ピン25/26の用途を PWM_PIN_MODE で選ぶ(同時には使えない. いずれもエラーLED/BT LEDは無効になる).
//   1: 本体PWMサーボ2ch(ピン25, 26. LEDCの50Hz出力)
//   2: PCA9685用のI2C(Wire1, SDA=25, SCL=26). PCA9685は最大2枚(16ch x 2)
//
// 指令値: 各出力が従うMeridimのスロット(L00〜R14)を対応表(PWM_PIN_SLOT, PCA9685_SLOT)で指定する.
//   スロットのコマンドが1なら角度をパルス幅にして出力し, それ以外はパルスを止める(サーボは脱力).
//   トリムと回転方向はそのスロットの設定(IXL_TRIM/IXL_CW等)を使う.
//   スロットの種類を1(本体PWM)/11(PCA9685)にすると, そのスロットはUARTの系統では扱わず指令値をそのまま返す.
//   通常のサーボのスロットを指定した場合は, そのサーボと同じ指令で動く.
// 周期: PWMは50Hz. Meridianの100Hzのうち, 本体ピンは偶数フレームで更新, PCA9685は1枚目を偶数フレーム,
//   2枚目を奇数フレームで書き込む(各50Hz. 1枚の書き込みは65byte, 400kHzで約1.5ms).
// 戻り値なし(動作命令のみ).
//
// パルス幅: PWM_PULSE_CENTER(1500us)が0deg, 1degあたり PWM_US_PER_DEG, PWM_PULSE_MIN〜MAXに制限.

#define PWM_PERIOD_US        20000 // 50Hz
#define PWM_LEDC_CH0         8     // 本体ピン用のLEDCチャンネル(8,9は同じタイマーを共有)
#define PWM_LEDC_BITS        16
#define PCA9685_REG_MODE1    0x00
#define PCA9685_REG_MODE2    0x01
#define PCA9685_REG_LED0     0x06 // LED0_ON_L. 以降1chあたり4バイト(ON_L, ON_H, OFF_L, OFF_H)
#define PCA9685_REG_PRESCALE 0xFE
#define PCA9685_CH_NUM       16

/// @brief 角度(degree)をPWMのパルス幅(us)に変換する.
inline uint16_t mrd_pwm_deg2us(float a_degree, float a_trim, int a_cw) {
  return mrd_servo_deg2pos(a_degree, a_trim, a_cw, cfg.pwm_us_per_deg, cfg.pwm_pulse_center, cfg.pwm_pulse_min, cfg.pwm_pulse_max);
}

/// @brief スロットごとの直前の出力(パルスを出したか). 通常以外のスロット(パラメータ系コマンド, 車輪, スルー)は保持する.
static bool mrd_pwm_last_on[2][MRD_SERVO_SLOTS] = {{false}};

/// @brief 対応表のスロットの指令を取得する.
/// @param a_slot スロット(SV_L(n), SV_R(n), SV_NONE).
/// @param a_us パルス幅(us)の格納先.
/// @return パルスを出す(スロットが有効でコマンド1. 通常以外のスロットは直前の出力を保持)ならtrue.
bool mrd_pwm_slot_us(int a_slot, const Meridim90Union &a_meridim, const ServoParam &a_sv, uint16_t &a_us) {
  const bool r_tmp = (a_slot >= SV_R(0));
  const int n_tmp = r_tmp ? a_slot - SV_R(0) : a_slot;
  if (a_slot < 0 || n_tmp >= MRD_SERVO_SLOTS) {
    return false;
  }
  const int orig_tmp = r_tmp ? MRD_R_ORIGIDX : MRD_L_ORIGIDX;
  const short cmd_tmp = a_meridim.sval[orig_tmp + n_tmp * 2];
  bool &last_tmp = mrd_pwm_last_on[r_tmp ? 1 : 0][n_tmp];
  if (mrd_servo_slot_mode(cmd_tmp) == MRD_SLOT_NORMAL) {
    last_tmp = (mrd_servo_cmd(cmd_tmp) == 1);
  }
  if (!last_tmp) {
    return false;
  }
  a_us = r_tmp ? mrd_pwm_deg2us(a_sv.ixr_tgt[n_tmp], a_sv.ixr_trim[n_tmp], a_sv.ixr_cw[n_tmp])
               : mrd_pwm_deg2us(a_sv.ixl_tgt[n_tmp], a_sv.ixl_trim[n_tmp], a_sv.ixl_cw[n_tmp]);
  return true;
}

//------------------------------------------------------------------------------------
//  PCA9685
//------------------------------------------------------------------------------------

/// @brief 50Hzにするプリスケール値. PRESCALE = round(発振周波数 / (4096 x 50)) - 1.
inline uint8_t mrd_pca_prescale(uint32_t a_osc_hz) {
  return uint8_t(lroundf(float(a_osc_hz) / (4096.0f * 1000000.0f / PWM_PERIOD_US)) - 1);
}

/// @brief パルス幅(us)をPCA9685のカウント(0-4095)に変換する. 実際のプリスケール値から周期を求める.
inline uint16_t mrd_pca_us2count(uint16_t a_us, uint32_t a_osc_hz, uint8_t a_prescale) {
  const float count_tmp = float(a_us) * float(a_osc_hz) / (float(a_prescale) + 1.0f) / 1000000.0f;
  return uint16_t(constrain(lroundf(count_tmp), 0L, 4095L));
}

/// @brief 16ch分の出力レジスタ(LED0_ON_L〜LED15_OFF_H, 64バイト)を作る.
///        パルスを出すchは ON=0, OFF=カウント. 出さないchは完全OFF(OFF_Hのbit4).
inline void mrd_pca_regs_fill(uint8_t *a_regs, const uint16_t *a_count, const bool *a_on) {
  for (int ch = 0; ch < PCA9685_CH_NUM; ch++) {
    uint8_t *p_tmp = &a_regs[ch * 4];
    p_tmp[0] = 0;
    p_tmp[1] = 0;
    p_tmp[2] = a_on[ch] ? (a_count[ch] & 0xFF) : 0;
    p_tmp[3] = a_on[ch] ? (a_count[ch] >> 8) : 0x10;
  }
}

class Pca9685Class {
public:
  /// @param a_wire 使用するI2C.
  /// @param a_addr I2Cアドレス.
  Pca9685Class(TwoWire &a_wire, uint8_t a_addr) : m_wire(a_wire), m_addr(a_addr) {}

  /// @brief 応答を確認し, 50Hz・全ch OFFで開始する.
  /// @return 応答があればtrue.
  bool begin() {
    m_found = probe();
    if (m_found) {
      setup();
    }
    return m_found;
  }

  /// @brief 16ch分の出力を1回の通信で書き込む.
  ///        書き込めなくなったら(ケーブルの外れ等)表示し, 約1秒(この関数50回)ごとに初期化からやり直す.
  ///        応答がないあいだは毎回のI2Cのタイムアウト待ちでフレームが遅れないよう, 書き込まない.
  void write_all(const uint16_t *a_count, const bool *a_on) {
    if (!m_found) {
      return; // 起動時に見つからなかった
    }
    if (m_lost) {
      if (++m_retry < 50) {
        return;
      }
      m_retry = 0;
      if (!probe()) {
        return;
      }
      setup();
      m_lost = false;
      print_state(": restored.");
    }
    if (!send_all(a_count, a_on)) {
      m_lost = true;
      m_retry = 0;
      print_state(": no response. Retry every 1 s.");
    }
  }

  bool found() const { return m_found; }
  uint8_t addr() const { return m_addr; }
  /// @brief I2Cアドレスを設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ.
  void set_addr(uint8_t a_addr) { m_addr = a_addr; }
  uint8_t prescale() const { return m_prescale; }

private:
  /// @brief 状態の変化を表示する. 例) "PCA9685 0x40: restored."
  void print_state(const char *a_text) {
    Serial.print("PCA9685 0x");
    Serial.print(int(m_addr), HEX);
    Serial.println(a_text);
  }

  /// @brief I2Cで応答するか.
  bool probe() {
    m_wire.beginTransmission(m_addr);
    return m_wire.endTransmission() == 0;
  }

  /// @brief 50Hz・全ch OFFに設定する.
  void setup() {
    m_prescale = mrd_pca_prescale(cfg.pca9685_osc_hz);
    write_reg(PCA9685_REG_MODE1, 0x10); // SLEEP(プリスケールはスリープ中のみ変更できる)
    write_reg(PCA9685_REG_PRESCALE, m_prescale);
    write_reg(PCA9685_REG_MODE2, 0x04); // OUTDRV: トーテムポール出力
    write_reg(PCA9685_REG_MODE1, 0x20); // 自動インクリメント, スリープ解除
    delayMicroseconds(500);             // 発振器の安定待ち
    write_reg(PCA9685_REG_MODE1, 0xA0); // RESTART
    const uint16_t count_tmp[PCA9685_CH_NUM] = {0};
    const bool on_tmp[PCA9685_CH_NUM] = {false};
    send_all(count_tmp, on_tmp);
  }

  /// @brief 16ch分の出力を書き込む.
  /// @return 書き込めたらtrue.
  bool send_all(const uint16_t *a_count, const bool *a_on) {
    uint8_t regs_tmp[PCA9685_CH_NUM * 4];
    mrd_pca_regs_fill(regs_tmp, a_count, a_on);
    m_wire.beginTransmission(m_addr);
    m_wire.write(PCA9685_REG_LED0);
    m_wire.write(regs_tmp, sizeof(regs_tmp));
    return m_wire.endTransmission() == 0;
  }

  void write_reg(uint8_t a_reg, uint8_t a_val) {
    m_wire.beginTransmission(m_addr);
    m_wire.write(a_reg);
    m_wire.write(a_val);
    m_wire.endTransmission();
  }

  TwoWire &m_wire;
  uint8_t m_addr;
  bool m_found = false; // 起動時に見つかったか
  bool m_lost = false;  // 書き込めなくなったか
  int m_retry = 0;      // 書き込めなくなってからの呼び出し回数(やり直しの間隔)
  uint8_t m_prescale = 121;
};

static Pca9685Class mrd_pca[2] = {Pca9685Class(Wire1, cfg.pca9685_addr_0), Pca9685Class(Wire1, cfg.pca9685_addr_1)};

//------------------------------------------------------------------------------------
//  開始と駆動
//------------------------------------------------------------------------------------

/// @brief PWM出力を開始する(PWM_PIN_MODE に応じて本体ピンまたはPCA9685).
void mrd_pwm_begin() {
  if (cfg.pwm_pin_mode == 1) {
    const int pins_tmp[2] = {PIN_PWM_0, PIN_PWM_1};
    for (int k = 0; k < 2; k++) {
      ledcSetup(PWM_LEDC_CH0 + k, 1000000 / PWM_PERIOD_US, PWM_LEDC_BITS);
      ledcAttachPin(pins_tmp[k], PWM_LEDC_CH0 + k);
      ledcWrite(PWM_LEDC_CH0 + k, 0); // パルスなし(脱力)から開始
    }
    Serial.println("PWM servo pins 25, 26: 50Hz.");
  } else if (cfg.pwm_pin_mode == 2) {
    mrd_pca[0].set_addr(cfg.pca9685_addr_0);
    mrd_pca[1].set_addr(cfg.pca9685_addr_1);
    Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, cfg.i2c1_speed);
    for (int b = 0; b < cfg.pca9685_num && b < 2; b++) {
      Serial.print("PCA9685 [");
      Serial.print(b);
      Serial.print("] 0x");
      Serial.print(mrd_pca[b].addr(), HEX);
      Serial.println(mrd_pca[b].begin() ? ": 50Hz." : ": no response.");
    }
  }
}

/// @brief PWM出力を更新する. 本体ピンは偶数フレーム, PCA9685は1枚目を偶数・2枚目を奇数フレームで書き込む.
///        UARTのサーボの処理より前に呼び, 現在位置で上書きされる前の指令値を使う.
void mrd_pwm_drive(const Meridim90Union &a_meridim, const ServoParam &a_sv) {
  if (cfg.pwm_pin_mode == 1) {
    if (mrd_servo_frame % 2 != 0) {
      return;
    }
    const int slot_tmp[2] = {PWM_PIN_SLOT[0], PWM_PIN_SLOT[1]};
    for (int k = 0; k < 2; k++) {
      uint16_t us_tmp = 0;
      const bool on_tmp = mrd_pwm_slot_us(slot_tmp[k], a_meridim, a_sv, us_tmp);
      ledcWrite(PWM_LEDC_CH0 + k, on_tmp ? uint32_t(us_tmp) * (1UL << PWM_LEDC_BITS) / PWM_PERIOD_US : 0);
    }
  } else if (cfg.pwm_pin_mode == 2) {
    const int board_tmp = mrd_servo_frame % 2;
    if (board_tmp >= cfg.pca9685_num || !mrd_pca[board_tmp].found()) {
      return;
    }
    uint16_t count_tmp[PCA9685_CH_NUM];
    bool on_tmp[PCA9685_CH_NUM];
    for (int ch = 0; ch < PCA9685_CH_NUM; ch++) {
      uint16_t us_tmp = 0;
      on_tmp[ch] = mrd_pwm_slot_us(PCA9685_SLOT[board_tmp][ch], a_meridim, a_sv, us_tmp);
      count_tmp[ch] = on_tmp[ch] ? mrd_pca_us2count(us_tmp, cfg.pca9685_osc_hz, mrd_pca[board_tmp].prescale()) : 0;
    }
    mrd_pca[board_tmp].write_all(count_tmp, on_tmp);
  }
}

#endif // __MERIDIAN_SERVO_PWM_H__
