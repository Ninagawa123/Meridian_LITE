#ifndef __MERIDIAN_SERVO_DISTRIBUTOR_H__
#define __MERIDIAN_SERVO_DISTRIBUTOR_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <hal/uart_ll.h>

//==================================================================================================
//  半二重UARTバス (プロトコル非依存)
//==================================================================================================
//
// ESP32のTX/RX/DIRの3本で半二重のサーボ線を駆動する共通層.
// ICS, DYNAMIXEL, FEETECH, FUTABA等の各プロトコル層はこのクラスの transfer() を利用する.
// ボーレートとフレーム形式(ICS:8E1, その他:8N1等)はプロトコル層が begin() で指定する.
//
// 想定回路 (SN74LXC1T45, VCCA=3.3V / VCCB=サーボ信号電圧):
//   ESP32 TX  ---+-- A    B -- サーボ信号線
//   ESP32 RX  ---+
//   ESP32 DIR ------- DIR  (H: A->B 送信, L: B->A 受信)
//   この構成では送信バイトがRXにエコーされる(ECHO_ON).
//   TXとA端子が直結のため, 受信中(A端子が出力)はTXピンの出力を切ってぶつかりを防ぐ(a_tx_release).
//   旧3ステートバッファ構成(送信中はRX無効)ではエコーされない(ECHO_OFF).
//   ECHO_AUTO の場合は begin() 時にダミーバイトを送ってエコーの有無を判定する.

/// @brief TXピンの出力をON/OFFする. UARTとピンの対応付け(GPIOマトリクス)は変更しない.
///        (gpio_ll_output_disable()は対応付けも外してしまうため使わない)
///        事前に GPIO.func_out_sel_cfg[a_pin].oen_sel = 1 で出力有効の制御をGPIOレジスタ側にしておくこと.
/// @param a_pin TXピン番号. 負の場合は何もしない.
/// @param a_on trueで出力ON, falseで出力OFF(ハイインピーダンス).
__attribute__((always_inline)) static inline void mrd_hduart_tx_output(int8_t a_pin, bool a_on) {
  if (a_pin < 0) {
    return;
  }
  if (a_pin < 32) {
    if (a_on) {
      GPIO.enable_w1ts = (1u << a_pin);
    } else {
      GPIO.enable_w1tc = (1u << a_pin);
    }
  } else {
    if (a_on) {
      GPIO.enable1_w1ts.val = (1u << (a_pin - 32));
    } else {
      GPIO.enable1_w1tc.val = (1u << (a_pin - 32));
    }
  }
}

/// @brief UARTの送信完了を待ってDIRピンをLOW(受信)にする.
///        サーボは送信完了の直後に応答を始めるため, DIRの切替が数us遅れるだけで応答の先頭が欠ける.
///        フラッシュキャッシュのミスや割り込みの影響を受けないよう, IRAM上で割り込みを止めた状態で
///        送信完了をレジスタで待ち, DIRをレジスタで直接切り替える.
///        割り込み停止時間は残りの送信時間に比例する(ICSの3バイト@1.25Mbpsで約26us).
///        (クラス内定義のインライン関数にはIRAM_ATTRを付けられないため, 非インラインの関数として定義)
/// @param a_hw 対象UARTのレジスタ.
/// @param a_pin_dir DIRピン番号.
/// @param a_pin_tx_release 受信中に出力を切るTXピン番号(切らない場合は-1).
/// @param a_mux 割り込み停止用のスピンロック.
static void __attribute__((noinline)) IRAM_ATTR mrd_hduart_wait_tx_and_dir_rx(uart_dev_t *a_hw, int8_t a_pin_dir,
                                                                             int8_t a_pin_tx_release,
                                                                             portMUX_TYPE *a_mux) {
  portENTER_CRITICAL(a_mux);
  while (!uart_ll_is_tx_idle(a_hw)) {
  }
  mrd_hduart_tx_output(a_pin_tx_release, false);      // 先にTXを切ってからA端子を出力にする
  gpio_ll_set_level(&GPIO, (gpio_num_t)a_pin_dir, 0); // LOW:受信
  portEXIT_CRITICAL(a_mux);
}

class MrdHalfDuplexUart {
public:
  enum EchoMode : uint8_t {
    ECHO_OFF = 0, // エコーなし(旧3ステートバッファ構成)
    ECHO_ON = 1,  // エコーあり(SN74LXC1T45構成)
    ECHO_AUTO = 2 // begin()時に自動判定
  };

  /// @brief コンストラクタ.
  /// @param a_serial 使用するHardwareSerial.
  /// @param a_pin_rx RXピン番号.
  /// @param a_pin_tx TXピン番号.
  /// @param a_pin_dir 送受信切替ピン番号(H:送信, L:受信).
  /// @param a_echo_mode 送信エコーの扱い(ECHO_OFF, ECHO_ON, ECHO_AUTO).
  /// @param a_timeout_ms 受信タイムアウト(ms).
  /// @param a_tx_release 受信中にTXピンの出力を切るか(TXとRXが同じ信号線に直結の回路でtrue).
  MrdHalfDuplexUart(HardwareSerial *a_serial, int8_t a_pin_rx, int8_t a_pin_tx, int8_t a_pin_dir,
                    uint8_t a_echo_mode, unsigned long a_timeout_ms, bool a_tx_release = false)
      : serial_(a_serial), pin_rx_(a_pin_rx), pin_tx_(a_pin_tx), pin_dir_(a_pin_dir),
        echo_mode_(a_echo_mode), echo_(a_echo_mode == ECHO_ON), timeout_ms_(a_timeout_ms),
        pin_tx_release_(a_tx_release ? a_pin_tx : -1) {}

  /// @brief 通信を開始する. プロトコル切替時の再設定にも使用できる.
  /// @param a_baud 通信速度bps.
  /// @param a_config フレーム形式(SERIAL_8E1, SERIAL_8N1など).
  /// @return 開始できればtrue.
  bool begin(unsigned long a_baud, uint32_t a_config) {
    if (serial_ == nullptr) {
      return false;
    }
    pinMode(pin_dir_, OUTPUT);
    dir_rx();
    serial_->begin(a_baud, a_config, pin_rx_, pin_tx_);
    uart_hw_ = UART_LL_GET_HW(serial_ == &Serial1 ? 1 : (serial_ == &Serial2 ? 2 : 0));
    if (pin_tx_release_ >= 0) {
      // TXとA端子がどちらも出力しない瞬間に信号線が浮かないようプルアップする
      gpio_pullup_en((gpio_num_t)pin_rx_);
      // TXの出力有効をUARTではなくGPIOレジスタで制御する(begin()のたびに再設定される)
      GPIO.func_out_sel_cfg[pin_tx_release_].oen_sel = 1;
      mrd_hduart_tx_output(pin_tx_release_, false); // 受信状態から開始
    }
    serial_->setTimeout(timeout_ms_);
    if (echo_mode_ == ECHO_AUTO) {
      echo_ = detect_echo();
    }
    began_ = true;
    return true;
  }

  /// @brief 送信エコーを処理しているか(ECHO_AUTO時は判定結果).
  bool echo() const { return echo_; }

  /// @brief 1回の送受信を行う(begin_send → end_send → receive).
  /// @param a_tx 送信データ.
  /// @param a_tx_len 送信データ数.
  /// @param a_rx 受信データ格納先.
  /// @param a_rx_len 受信データ数(0なら送信のみ).
  /// @return 送信エコーが一致し, 指定数の応答を受信できればtrue.
  bool transfer(const uint8_t *a_tx, size_t a_tx_len, uint8_t *a_rx, size_t a_rx_len) {
    if (!begin_send(a_tx, a_tx_len)) {
      return false;
    }
    end_send();
    return receive(a_rx, a_rx_len);
  }

  //------------------------------------------------------------------------------------
  //  分割API: 複数系統を並行して送受信する場合に使用する
  //  例) L.begin_send → R.begin_send → L.end_send → R.end_send → L.receive → R.receive
  //------------------------------------------------------------------------------------

  /// @brief 送信を開始する. 送信完了は待たない.
  /// @param a_tx 送信データ.
  /// @param a_tx_len 送信データ数.
  /// @return 開始できればtrue.
  bool begin_send(const uint8_t *a_tx, size_t a_tx_len) {
    if (!began_) {
      return false;
    }

    // 前回の残りデータを破棄
    while (serial_->available() > 0) {
      serial_->read();
    }

    // エコー照合用に送信データを保持(バッファを超える分は照合せずに読み捨てる)
    tx_len_ = a_tx_len;
    memcpy(tx_copy_, a_tx, min(a_tx_len, sizeof(tx_copy_)));

    dir_tx();
    serial_->write(a_tx, a_tx_len);
    return true;
  }

  /// @brief 送信完了を待ち, 受信に切り替える(詳細は mrd_hduart_wait_tx_and_dir_rx).
  ///        RXバッファは保持される(エコーと応答はこの後 receive() で読む).
  void end_send() { mrd_hduart_wait_tx_and_dir_rx(uart_hw_, pin_dir_, pin_tx_release_, &dir_mux_); }

  /// @brief 送信エコーを照合して読み捨て, 応答を受信する.
  /// @param a_rx 受信データ格納先.
  /// @param a_rx_len 受信データ数(0なら照合のみ).
  /// @return 送信エコーが一致し, 指定数の応答を受信できればtrue.
  bool receive(uint8_t *a_rx, size_t a_rx_len) {
    // エコーは応答より先にRXバッファへ入っているため先頭から照合して読み捨てる
    if (echo_) {
      for (size_t i = 0; i < tx_len_; i++) {
        uint8_t echo_tmp;
        if (serial_->readBytes(&echo_tmp, 1) != 1) {
          return false;
        }
        if (i < sizeof(tx_copy_) && echo_tmp != tx_copy_[i]) {
          return false;
        }
      }
    }

    if (a_rx_len == 0) {
      return true;
    }
    return serial_->readBytes(a_rx, a_rx_len) == a_rx_len;
  }

private:
  /// @brief ダミーバイトを送信し, RXにエコーされるかを判定する.
  ///        0x55はICSではコマンドバイト(MSB=1)ではなく, DXL/FEETECH/FUTABAではヘッダを欠くため,
  ///        いずれのサーボにも無視される.
  /// @return 全試行でエコーを受信できればtrue.
  bool detect_echo() {
    const uint8_t probe_tmp = 0x55;
    const int trials_tmp = 3;
    bool echo_tmp = true;
    for (int i = 0; i < trials_tmp && echo_tmp; i++) {
      while (serial_->available() > 0) {
        serial_->read();
      }
      dir_tx();
      serial_->write(probe_tmp);
      serial_->flush();
      dir_rx();
      uint8_t rcv_tmp;
      echo_tmp = (serial_->readBytes(&rcv_tmp, 1) == 1 && rcv_tmp == probe_tmp);
    }
    while (serial_->available() > 0) {
      serial_->read();
    }
    return echo_tmp;
  }

  // 送信: 先にA端子を入力にしてからTXを出力する / 受信: 先にTXを切ってからA端子を出力にする
  inline void dir_tx() {
    digitalWrite(pin_dir_, HIGH);
    mrd_hduart_tx_output(pin_tx_release_, true);
  }
  inline void dir_rx() {
    mrd_hduart_tx_output(pin_tx_release_, false);
    digitalWrite(pin_dir_, LOW);
  }

  HardwareSerial *serial_;
  int8_t pin_rx_;
  int8_t pin_tx_;
  int8_t pin_dir_;
  uint8_t echo_mode_;
  bool echo_;
  unsigned long timeout_ms_;
  bool began_ = false;
  uart_dev_t *uart_hw_ = nullptr;                          // 送信完了をレジスタで確認するためのUART
  portMUX_TYPE dir_mux_ = portMUX_INITIALIZER_UNLOCKED; // end_send()の割り込み停止用
  int8_t pin_tx_release_;                                // 受信中に出力を切るTXピン(-1:切らない)
  uint8_t tx_copy_[128]; // エコー照合用の送信データの控え
  size_t tx_len_ = 0;   // 直前の送信データ数
};

// 各プロトコル層の読み込み (半二重UARTバスの定義より後に読み込むこと)
#include "mrd_module/sv_ftbrx.h"
#include "mrd_module/sv_ics.h"

//==================================================================================================
//  Servo 関連の処理
//==================================================================================================

//------------------------------------------------------------------------------------
//  各UARTの開始
//------------------------------------------------------------------------------------

/// @brief 指定されたUARTラインとサーボタイプに基づいてサーボの通信プロトコルを設定する.
/// @param a_line UART通信ライン(L, R, またはC).
/// @param a_servo_type サーボのタイプを示す整数値.
/// @return サーボがサポートされている場合はtrueを, サポートされていない場合はfalseを返す.
bool mrd_servo_begin(UartLine a_line, int a_servo_type) {
  switch (a_servo_type) {
  case 1:
    // single PWM [WIP]
    return false;
  case 11:
    // I2C_PCA9685 to PWM [WIP]
    return false;
  case 21:
    // RSxTTL (FUTABA) [WIP]
    return false;
  case 31:
    // DYNAMIXEL Protocol 1.0 [WIP]
    return false;
  case 32:
    // DYNAMIXEL Protocol 2.0 [WIP]
    return false;
  case 43:
    if (a_line == L)
      ics_L.begin(); // サーボモータの通信初期設定. Serial1
    else if (a_line == R)
      ics_R.begin(); // サーボモータの通信初期設定. Serial2
    return true;
  case 44:
    // PMX(KONDO) [WIP]
    return false;
  case 51:
    // XBUS(JR PROPO) [WIP]
    return false;
  case 61:
    // STS(FEETECH) [WIP]
    return false;
  case 62:
    // SCS(FEETECH) [WIP]
    return false;
  default:
    // Not defined.
    return false;
  }
}

//------------------------------------------------------------------------------------
//  サーボ通信フォーメーションの分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたサーボにコマンドを分配する.
/// @param a_meridim サーボの動作パラメータを含むMeridim配列.
/// @param a_L_type L系統のサーボタイプ.
/// @param a_R_type R系統のサーボタイプ.
/// @param a_sv サーボパラメータの構造体.
/// @return サーボの駆動が成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_servo_drive_lite(Meridim90Union &a_meridim, int a_L_type, int a_R_type, ServoParam &a_sv) {
  if (a_L_type == 43 && a_R_type == 43) // ICSサーボがL系R系に設定されていた場合はLR均等送信を実行
  {
    mrd_sv_drive_ics_double(a_meridim, a_sv);
    return true;
  } else {
    return false;
  }
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief 第一引数のMeridim配列のすべてのサーボモーターをオフ(フリー状態)に設定する.
/// @param a_meridim サーボの動作パラメータを含むMeridim配列.
/// @return 設定完了時にtrueを返す.
bool mrd_servo_all_off(Meridim90Union &a_meridim) {
  for (int i = 0; i < 15; i++) {    // 15はサーボ数
    a_meridim.sval[i * 2 + 20] = 0; // サーボのコマンドをオフに設定
    a_meridim.sval[i * 2 + 50] = 0; //
  }
  Serial.println("All servos torque off.");
  return true;
}

/// @brief サーボパラメータからエラーのあるサーボのインデックス番号を作る.
/// @param a_sv サーボパラメータの構造体.
/// @return uint8_tで番号を返す.
///         100-149(L系統 0-49),200-249(R系統 0-49)
uint8_t mrd_servo_make_errcode_lite(const ServoParam &a_sv) {
  uint8_t servo_ix_tmp = 0;
  for (int i = 0; i < a_sv.num_max; i++) {
    if (a_sv.ixl_stat[i]) {
      servo_ix_tmp = uint8_t(i + 100);
    }
    if (a_sv.ixr_stat[i]) {
      servo_ix_tmp = uint8_t(i + 200);
    }
  }
  return servo_ix_tmp;
}

#endif // __MERIDIAN_SERVO_DISTRIBUTOR_H__
