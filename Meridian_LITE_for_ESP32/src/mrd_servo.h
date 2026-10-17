#ifndef __MERIDIAN_SERVO_DISTRIBUTOR_H__
#define __MERIDIAN_SERVO_DISTRIBUTOR_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <hal/uart_ll.h>
#include <new>          // placement new (系統ごとのステッパーの生成)
#include <xtensa/hal.h> // xthal_get_ccount (区切り受信の時間計測)

//==================================================================================================
//  半二重UARTバス (プロトコル非依存)
//==================================================================================================
//
// ESP32のTX/RX/DIRの3本で半二重のサーボ線を駆動する共通層.
// 各プロトコル層は駆動時は mrd_hduart_xfer_start/poll で, 起動時の確認・設定は transfer() でこのクラスを利用する.
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

#define MRD_HDUART_CRITICAL_BYTES 8   // 送信完了待ちで割り込みを止める送信FIFOの残りバイト数
#define MRD_HDUART_STUCK_MS       150 // 割り込みを止めて送信完了を待つ上限(ms). UARTが止まっても割り込みの監視(300ms)で再起動しないため
#define MRD_HDUART_RX_BUF         512 // 受信バッファ(byte). DYNAMIXELのSyncRead(15台で応答225byte)を2系統分受けられる大きさ
#define MRD_HDUART_TX_MAX         128 // 1パケットの最大長(送信エコーの照合バッファ). 各プロトコルのパケットはこれ以下にする

/// @brief UARTの送信完了を待ってDIRピンをLOW(受信)にする.
///        サーボは送信完了の直後に応答を始めるため, DIRの切替が数us遅れるだけで応答の先頭が欠ける.
///        フラッシュキャッシュのミスや割り込みの影響を受けないよう, IRAM上で割り込みを止めた状態で
///        送信完了をレジスタで待ち, DIRをレジスタで直接切り替える.
///        割り込みを止めるのは送信FIFOの残りが MRD_HDUART_CRITICAL_BYTES 以下になってからとし,
///        長いパケット(応答のないFEETECHのSyncWrite等)でも割り込み停止時間を8バイト分(1Mbpsで約80us)以内に抑える.
///        8バイト以下のパケット(ICSの各命令, FEETECHのREAD)は従来どおり送信直後から割り込みを止める.
///        (クラス内定義のインライン関数にはIRAM_ATTRを付けられないため, 非インラインの関数として定義)
/// @param a_hw 対象UARTのレジスタ.
/// @param a_pin_dir DIRピン番号.
/// @param a_pin_tx_release 受信中に出力を切るTXピン番号(切らない場合は-1).
/// @param a_mux 割り込み停止用のスピンロック.
static void __attribute__((noinline)) IRAM_ATTR mrd_hduart_wait_tx_and_dir_rx(uart_dev_t *a_hw, int8_t a_pin_dir,
                                                                              int8_t a_pin_tx_release,
                                                                              portMUX_TYPE *a_mux) {
  while (UART_LL_FIFO_DEF_LEN - uart_ll_get_txfifo_len(a_hw) > MRD_HDUART_CRITICAL_BYTES) {
  }
  const uint32_t limit_tmp = ets_get_cpu_frequency() * 1000UL * MRD_HDUART_STUCK_MS; // CPUサイクル
  portENTER_CRITICAL(a_mux);
  const uint32_t t0_tmp = xthal_get_ccount();
  while (!uart_ll_is_tx_idle(a_hw) && xthal_get_ccount() - t0_tmp < limit_tmp) {
  }
  mrd_hduart_tx_output(a_pin_tx_release, false);      // 先にTXを切ってからA端子を出力にする
  gpio_ll_set_level(&GPIO, (gpio_num_t)a_pin_dir, 0); // LOW:受信
  portEXIT_CRITICAL(a_mux);
}

/// @brief 複数系統の送信開始から受信切替までを, 割り込みを止めた状態でまとめて行う.
///        各系統を送信方向にして送信FIFOへ直接書き込み, 送信が終わった系統から即座にDIRをLOW(受信)にする.
///        送信開始と受信切替の間にソフトウェアの処理を挟まないため, 他の系統の送信開始処理や割り込みで
///        受信切替が遅れて応答の先頭が欠けることがない(応答遅延0のFEETECHサーボでも間に合う).
///        割り込み停止時間は最も長い系統の送信時間(FEETECHのREAD 8バイト@1Mbpsで約80us).
/// @param a_hw 対象UARTのレジスタ(a_num個).
/// @param a_pin_dir DIRピン番号(a_num個).
/// @param a_pin_tx_release 受信中に出力を切るTXピン番号(a_num個, 切らない場合は-1).
/// @param a_tx 送信データ(a_num個, DRAM上にあること).
/// @param a_len 送信データ数(a_num個, 送信FIFO長以下).
/// @param a_num 系統数(1または2).
/// @param a_mux 割り込み停止用のスピンロック.
static void __attribute__((noinline)) IRAM_ATTR mrd_hduart_send_and_dir_rx_multi(
    uart_dev_t *const *a_hw, const int8_t *a_pin_dir, const int8_t *a_pin_tx_release, const uint8_t *const *a_tx,
    const size_t *a_len, int a_num, portMUX_TYPE *a_mux) {
  portENTER_CRITICAL(a_mux);
  for (int i = 0; i < a_num; i++) {
    gpio_ll_set_level(&GPIO, (gpio_num_t)a_pin_dir[i], 1); // 先にA端子を入力(送信方向)にしてから
    mrd_hduart_tx_output(a_pin_tx_release[i], true);       // TXを出力する
    uart_ll_write_txfifo(a_hw[i], a_tx[i], a_len[i]);
  }
  uint32_t pending_tmp = (1u << a_num) - 1;
  uint32_t started_tmp = 0;                                                          // 送信中を一度確認した系統(書き込み直後の状態の反映遅れで誤判定しないため)
  const uint32_t limit_tmp = ets_get_cpu_frequency() * 1000UL * MRD_HDUART_STUCK_MS; // CPUサイクル
  const uint32_t t0_tmp = xthal_get_ccount();
  while (pending_tmp != 0) {
    if (xthal_get_ccount() - t0_tmp >= limit_tmp) { // UARTが止まった: 受信方向に戻して終える(応答なしになる)
      for (int i = 0; i < a_num; i++) {
        mrd_hduart_tx_output(a_pin_tx_release[i], false);
        gpio_ll_set_level(&GPIO, (gpio_num_t)a_pin_dir[i], 0);
      }
      break;
    }
    for (int i = 0; i < a_num; i++) {
      const uint32_t bit_tmp = 1u << i;
      if (!(pending_tmp & bit_tmp)) {
        continue;
      }
      if (!uart_ll_is_tx_idle(a_hw[i])) {
        started_tmp |= bit_tmp;
      } else if (started_tmp & bit_tmp) {
        mrd_hduart_tx_output(a_pin_tx_release[i], false);      // 先にTXを切ってからA端子を出力にする
        gpio_ll_set_level(&GPIO, (gpio_num_t)a_pin_dir[i], 0); // LOW:受信
        pending_tmp &= ~bit_tmp;
      }
    }
  }
  portEXIT_CRITICAL(a_mux);
}

/// @brief 返信をホストの区切りで受ける(Hitec HSRの読み出し命令). 送信完了後(送信方向のまま)に呼ぶ.
///        Hitecのサーボは返信のスタート/ストップビットを出さず, ホストが送る0x00のデータビットの間だけ
///        線をLOWに引いて返信を書き込む. そこで各系統の送信FIFOへ0x00を a_bytes 個書き込み,
///        TXピンでスタートビットを検出した瞬間に線を開放し, 最後のデータビットの読み取り後に送信方向へ戻す.
///        ストップビットはUART自身が出すため, 区切りはUARTの実際の出力と一致する. 返信はRXに入る(エコーの後).
///        信号反転(アイドルLOW, スタートビットHIGH)の線のみ対応. 2系統の区切りは独立して扱う.
///        割り込み停止時間は a_bytes 分の送信時間(19200bps 8N2の2バイトで約1.15ms).
/// @param a_hw 対象UARTのレジスタ(a_num個).
/// @param a_pin_dir DIRピン番号(a_num個).
/// @param a_pin_tx TXピン番号(a_num個, 入力も有効にしておくこと).
/// @param a_pin_tx_release 開放中に出力を切るTXピン番号(a_num個, 切らない場合は-1).
/// @param a_num 系統数(1または2).
/// @param a_bytes 返信のバイト数(8以下).
/// @param a_release_cyc スタートビットの検出から送信方向へ戻すまでのCPUサイクル数(a_num個).
/// @param a_timeout_cyc 各ビットの待ちのタイムアウト(CPUサイクル数).
/// @param a_mux 割り込み停止用のスピンロック.
static void __attribute__((noinline)) IRAM_ATTR mrd_hduart_framed_reply_multi(
    uart_dev_t *const *a_hw, const int8_t *a_pin_dir, const int8_t *a_pin_tx, const int8_t *a_pin_tx_release,
    int a_num, size_t a_bytes, const uint32_t *a_release_cyc, uint32_t a_timeout_cyc, portMUX_TYPE *a_mux) {
  enum : uint8_t { WAIT_START,
                   RELEASED,
                   WAIT_STOP,
                   DONE };
  const uint8_t zeros_tmp[8] = {0};
  uint8_t state_tmp[2] = {DONE, DONE};
  size_t left_tmp[2] = {0, 0};
  uint32_t t0_tmp[2] = {0, 0};
  portENTER_CRITICAL(a_mux);
  for (int i = 0; i < a_num; i++) {
    uart_ll_write_txfifo(a_hw[i], zeros_tmp, a_bytes);
    state_tmp[i] = WAIT_START;
    left_tmp[i] = a_bytes;
    t0_tmp[i] = xthal_get_ccount();
  }
  bool busy_tmp = true;
  while (busy_tmp) {
    busy_tmp = false;
    for (int i = 0; i < a_num; i++) {
      const uint32_t now_tmp = xthal_get_ccount();
      switch (state_tmp[i]) {
      case WAIT_START: // スタートビット(反転時HIGH)を待ち, 線を開放してサーボに書き込ませる
        if (gpio_ll_get_level(&GPIO, (gpio_num_t)a_pin_tx[i])) {
          mrd_hduart_tx_output(a_pin_tx_release[i], false);
          gpio_ll_set_level(&GPIO, (gpio_num_t)a_pin_dir[i], 0);
          state_tmp[i] = RELEASED;
          t0_tmp[i] = now_tmp;
        } else if (now_tmp - t0_tmp[i] > a_timeout_cyc) {
          state_tmp[i] = DONE;
        }
        break;
      case RELEASED: // 最後のデータビットの読み取り後に送信方向へ戻す(TXはデータ0=HIGHを出している)
        if (now_tmp - t0_tmp[i] >= a_release_cyc[i]) {
          gpio_ll_set_level(&GPIO, (gpio_num_t)a_pin_dir[i], 1);
          mrd_hduart_tx_output(a_pin_tx_release[i], true);
          state_tmp[i] = WAIT_STOP;
          t0_tmp[i] = now_tmp;
        }
        break;
      case WAIT_STOP: // ストップビット(LOW)に入るのを待つ(次のスタートビットの検出のため)
        if (!gpio_ll_get_level(&GPIO, (gpio_num_t)a_pin_tx[i]) || now_tmp - t0_tmp[i] > a_timeout_cyc) {
          state_tmp[i] = (--left_tmp[i] > 0) ? WAIT_START : DONE;
          t0_tmp[i] = now_tmp;
        }
        break;
      default:
        break;
      }
      busy_tmp = busy_tmp || state_tmp[i] != DONE;
    }
  }
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
      : m_serial(a_serial), m_pin_rx(a_pin_rx), m_pin_tx(a_pin_tx), m_pin_dir(a_pin_dir),
        m_echo_mode(a_echo_mode), m_echo(a_echo_mode == ECHO_ON), m_timeout_ms(a_timeout_ms),
        m_pin_tx_release(a_tx_release ? a_pin_tx : -1) {}

  /// @brief 通信を開始する. プロトコル切替時の再設定にも使用できる.
  /// @param a_baud 通信速度bps.
  /// @param a_config フレーム形式(SERIAL_8E1, SERIAL_8N1など).
  /// @param a_invert 信号を反転するか(Hitec HSR: アイドルLOW).
  /// @param a_idle_tx 送受信の合間を送信方向で待つか. 線をESP32側が駆動して待機するプロトコル(Hitec, XBUS)で使う.
  ///        falseなら受信方向で待つ(ICS, FEETECH等).
  /// @return 開始できればtrue.
  bool begin(unsigned long a_baud, uint32_t a_config, bool a_invert = false, bool a_idle_tx = false) {
    if (m_serial == nullptr) {
      return false;
    }
    m_idle_tx = a_idle_tx;
    m_baud = a_baud;
    pinMode(m_pin_dir, OUTPUT);
    dir_rx();
    if (!m_began) {
      m_serial->setRxBufferSize(MRD_HDUART_RX_BUF); // begin()より前にしか変更できない
    }
    m_serial->begin(a_baud, a_config, m_pin_rx, m_pin_tx, a_invert);
    m_uart_hw = UART_LL_GET_HW(m_serial == &Serial1 ? 1 : (m_serial == &Serial2 ? 2 : 0));
    if (m_pin_tx_release >= 0) {
      // TXとA端子がどちらも出力しない瞬間に信号線が浮かないようプルアップする
      gpio_pullup_en((gpio_num_t)m_pin_rx);
      // TXの出力有効をUARTではなくGPIOレジスタで制御する(begin()のたびに再設定される)
      GPIO.func_out_sel_cfg[m_pin_tx_release].oen_sel = 1;
      mrd_hduart_tx_output(m_pin_tx_release, false); // 受信状態から開始
    }
    if (a_invert) {
      gpio_ll_input_enable(&GPIO, (gpio_num_t)m_pin_tx); // 区切り受信でスタートビットを検出するため
    }
    m_serial->setTimeout(m_timeout_ms);
    if (m_echo_mode == ECHO_AUTO) {
      m_echo = detect_echo();
    }
    m_began = true;
    idle();
    return true;
  }

  /// @brief 送受信の合間の待機状態にする(begin()で送信方向待機を指定した場合は送信方向, それ以外は何もしない).
  ///        送受信が終わるたびに呼ぶ(transfer() と mrd_hduart_xfer_poll() は自動で呼ぶ).
  void idle() {
    if (m_idle_tx) {
      dir_tx();
    }
  }

  /// @brief 送信エコー・受信タイムアウト・TXの切り離しを設定し直す(起動時に EEPROM の設定を反映するため).
  ///        begin() の前に呼ぶ. 引数はコンストラクタと同じ.
  void configure(uint8_t a_echo_mode, unsigned long a_timeout_ms, bool a_tx_release) {
    m_echo_mode = a_echo_mode;
    m_echo = (a_echo_mode == ECHO_ON);
    m_timeout_ms = a_timeout_ms;
    m_pin_tx_release = a_tx_release ? m_pin_tx : -1;
  }

  /// @brief 送信エコーを処理しているか(ECHO_AUTO時は判定結果).
  bool echo() const { return m_echo; }

  /// @brief 既定の受信タイムアウト(ms).
  unsigned long timeout_ms() const { return m_timeout_ms; }

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
    const bool ok_tmp = receive(a_rx, a_rx_len);
    idle();
    return ok_tmp;
  }

  //------------------------------------------------------------------------------------
  //  分割API: 送信と受信切替・受信を分けて進める場合に使用する(mrd_hduart_xfer_start/poll)
  //  例) begin_send → (tx_done() を待つ) → end_send → receive_now
  //------------------------------------------------------------------------------------

  /// @brief 送信を開始する. 送信完了は待たない.
  /// @param a_tx 送信データ.
  /// @param a_tx_len 送信データ数.
  /// @return 開始できればtrue.
  bool begin_send(const uint8_t *a_tx, size_t a_tx_len) {
    if (!m_began) {
      return false;
    }

    // 前回の残りデータを破棄
    flush_rx();

    // エコー照合用に送信データを保持(バッファを超える分は照合せずに読み捨てる)
    m_tx_len = a_tx_len;
    memcpy(m_tx_copy, a_tx, min(a_tx_len, sizeof(m_tx_copy)));

    dir_tx();
    m_serial->write(a_tx, a_tx_len);
    return true;
  }

  /// @brief 送信完了を待ち, 受信に切り替える(詳細は mrd_hduart_wait_tx_and_dir_rx).
  ///        RXバッファは保持される(エコーと応答はこの後 receive() で読む).
  void end_send() { mrd_hduart_wait_tx_and_dir_rx(m_uart_hw, m_pin_dir, m_pin_tx_release, &m_dir_mux); }

  //------------------------------------------------------------------------------------
  //  複数系統の同時送信API: 応答のある命令を複数系統で重ねて送る場合に使用する
  //  例) prepare_send → send_multi(&bus, 1) → receive_now (mrd_hduart_xfer_start/poll. 2系統を重ねることもできる)
  //------------------------------------------------------------------------------------

  /// @brief 送信の準備(前回の残りデータの破棄とエコー照合用の控え)だけを行う. 送信は send_multi() で行う.
  /// @param a_tx 送信データ.
  /// @param a_tx_len 送信データ数(送信FIFO長以下).
  /// @return 準備できればtrue.
  bool prepare_send(const uint8_t *a_tx, size_t a_tx_len) {
    if (!m_began || a_tx_len == 0 || a_tx_len > sizeof(m_tx_copy) || a_tx_len > UART_LL_FIFO_DEF_LEN) {
      return false;
    }
    flush_rx();
    m_tx_len = a_tx_len;
    memcpy(m_tx_copy, a_tx, a_tx_len);
    return true;
  }

  /// @brief prepare_send() 済みの系統を, 送信開始から受信切替まで割り込みを止めてまとめて行う
  ///        (詳細は mrd_hduart_send_and_dir_rx_multi). RXバッファは保持される(この後 receive() で読む).
  /// @param a_bus 対象の系統(a_num個).
  /// @param a_num 系統数(1または2).
  static void send_multi(MrdHalfDuplexUart *const *a_bus, int a_num) {
    uart_dev_t *hw_tmp[2];
    int8_t dir_tmp[2];
    int8_t rel_tmp[2];
    const uint8_t *tx_tmp[2];
    size_t len_tmp[2];
    const int num_tmp = min(a_num, 2);
    for (int i = 0; i < num_tmp; i++) {
      hw_tmp[i] = a_bus[i]->m_uart_hw;
      dir_tmp[i] = a_bus[i]->m_pin_dir;
      rel_tmp[i] = a_bus[i]->m_pin_tx_release;
      tx_tmp[i] = a_bus[i]->m_tx_copy;
      len_tmp[i] = a_bus[i]->m_tx_len;
    }
    mrd_hduart_send_and_dir_rx_multi(hw_tmp, dir_tmp, rel_tmp, tx_tmp, len_tmp, num_tmp, &a_bus[0]->m_dir_mux);
  }

  /// @brief begin_send() 済みの系統の送信完了を待ち, 返信 a_bytes バイトをホストの区切りで受ける
  ///        (詳細は mrd_hduart_framed_reply_multi). 終了時は送信方向. 返信はこの後 receive() で読む.
  /// @param a_bus 対象の系統(a_num個).
  /// @param a_num 系統数(1または2).
  /// @param a_bytes 返信のバイト数(8以下).
  static void framed_reply_multi(MrdHalfDuplexUart *const *a_bus, int a_num, size_t a_bytes) {
    uart_dev_t *hw_tmp[2];
    int8_t dir_tmp[2];
    int8_t tx_tmp[2];
    int8_t rel_tmp[2];
    uint32_t release_tmp[2];
    const int num_tmp = min(a_num, 2);
    const uint32_t cyc_per_us_tmp = getCpuFrequencyMhz();
    unsigned long baud_min_tmp = 0;
    for (int i = 0; i < num_tmp; i++) {
      MrdHalfDuplexUart *b_tmp = a_bus[i];
      // 先に送ったパケットの送信完了を待つ(区切りの前の待ちは急がない).
      // 書き込み直後の送信状態の反映遅れで誤判定しないよう, 送信FIFOが空であることも確認する
      while (uart_ll_get_txfifo_len(b_tmp->m_uart_hw) < UART_LL_FIFO_DEF_LEN || !uart_ll_is_tx_idle(b_tmp->m_uart_hw)) {
      }
      hw_tmp[i] = b_tmp->m_uart_hw;
      dir_tmp[i] = b_tmp->m_pin_dir;
      tx_tmp[i] = b_tmp->m_pin_tx;
      rel_tmp[i] = b_tmp->m_pin_tx_release;
      // スタート+データ8ビットのうち, 最後のデータビットの中央(8.5ビット)を過ぎるまで開放する
      const unsigned long baud_tmp = max(b_tmp->m_baud, 1200UL);
      release_tmp[i] = uint32_t(lroundf(8.75f * 1000000.0f / baud_tmp * cyc_per_us_tmp));
      baud_min_tmp = (baud_min_tmp == 0) ? baud_tmp : min(baud_min_tmp, baud_tmp);
    }
    if (num_tmp <= 0) {
      return;
    }
    const uint32_t timeout_tmp = uint32_t(lroundf(24.0f * 1000000.0f / baud_min_tmp * cyc_per_us_tmp)); // 2バイト分
    mrd_hduart_framed_reply_multi(hw_tmp, dir_tmp, tx_tmp, rel_tmp, num_tmp, min(a_bytes, size_t(8)), release_tmp,
                                  timeout_tmp, &a_bus[0]->m_dir_mux);
  }

  /// @brief 送信エコーを照合して読み捨て, 応答を受信する.
  /// @param a_rx 受信データ格納先.
  /// @param a_rx_len 受信データ数(0なら照合のみ).
  /// @param a_timeout_ms 応答の受信タイムアウト(ms). 0ならコンストラクタで指定した値.
  ///        複数のサーボが続けて応答する命令(DYNAMIXELのSyncRead等)で長く待つ場合に指定する.
  /// @return 送信エコーが一致し, 指定数の応答を受信できればtrue.
  bool receive(uint8_t *a_rx, size_t a_rx_len, unsigned long a_timeout_ms = 0) {
    m_last_rx_len = 0;
    // エコーは応答より先にRXバッファへ入っているため先頭から照合して読み捨てる
    // (1バイトずつ読むと読み出しのたびにロックを取るため, まとめて読む)
    if (m_echo) {
      size_t done_tmp = 0;
      while (done_tmp < m_tx_len) {
        uint8_t echo_tmp[32];
        const size_t n_tmp = min(sizeof(echo_tmp), m_tx_len - done_tmp);
        if (m_serial->readBytes(echo_tmp, n_tmp) != n_tmp) {
          m_last_rx = RX_ECHO_SHORT;
          return false;
        }
        for (size_t k = 0; k < n_tmp; k++) {
          const size_t idx_tmp = done_tmp + k;
          if (idx_tmp < sizeof(m_tx_copy) && echo_tmp[k] != m_tx_copy[idx_tmp]) {
            m_last_rx = RX_ECHO_MISMATCH;
            return false;
          }
        }
        done_tmp += n_tmp;
      }
    }

    if (a_rx_len == 0) {
      m_last_rx = RX_OK;
      return true;
    }
    if (a_timeout_ms > 0) {
      m_serial->setTimeout(a_timeout_ms);
    }
    const size_t got_tmp = m_serial->readBytes(a_rx, a_rx_len);
    if (a_timeout_ms > 0) {
      m_serial->setTimeout(m_timeout_ms);
    }
    m_last_rx_len = got_tmp;
    m_last_rx = (got_tmp == a_rx_len) ? RX_OK : (got_tmp == 0 ? RX_REPLY_NONE : RX_REPLY_PARTIAL);
    return got_tmp == a_rx_len;
  }

  /// @brief 直前の receive() で受信できた応答のバイト数(途中までの応答を解析する場合に使う).
  size_t last_rx_len() const { return m_last_rx_len; }

  //------------------------------------------------------------------------------------
  //  待たない送受信のための操作: 系統ごとに独立して進める場合に使用する(mrd_hduart_xfer_start/poll)
  //------------------------------------------------------------------------------------

  /// @brief begin_send() で送り始めたデータの送信が終わったか(送信FIFOが空で, 送信中でない).
  bool tx_done() const { return uart_ll_get_txfifo_len(m_uart_hw) >= UART_LL_FIFO_DEF_LEN && uart_ll_is_tx_idle(m_uart_hw); }

  /// @brief 受信バッファにあるバイト数(エコーを含む).
  size_t rx_available() const { return size_t(max(m_serial->available(), 0)); }

  /// @brief 受信バッファにあるデータだけで receive() を行う(待たない).
  ///        エコーが揃っていなければ RX_ECHO_SHORT, 応答が足りなければ途中まで読んで失敗とする.
  /// @param a_rx 受信データ格納先.
  /// @param a_rx_len 受信データ数(0なら照合のみ).
  /// @return 送信エコーが一致し, 指定数の応答を受信できればtrue.
  bool receive_now(uint8_t *a_rx, size_t a_rx_len) {
    m_last_rx_len = 0;
    if (m_echo) {
      if (rx_available() < m_tx_len) {
        flush_rx();
        m_last_rx = RX_ECHO_SHORT;
        return false;
      }
      size_t done_tmp = 0;
      while (done_tmp < m_tx_len) {
        uint8_t echo_tmp[32];
        const size_t n_tmp = m_serial->read(echo_tmp, min(sizeof(echo_tmp), m_tx_len - done_tmp));
        for (size_t k = 0; k < n_tmp; k++) {
          const size_t idx_tmp = done_tmp + k;
          if (idx_tmp < sizeof(m_tx_copy) && echo_tmp[k] != m_tx_copy[idx_tmp]) {
            m_last_rx = RX_ECHO_MISMATCH;
            return false;
          }
        }
        done_tmp += n_tmp;
      }
    }
    if (a_rx_len == 0) {
      m_last_rx = RX_OK;
      return true;
    }
    const size_t got_tmp = m_serial->read(a_rx, min(a_rx_len, rx_available()));
    m_last_rx_len = got_tmp;
    m_last_rx = (got_tmp == a_rx_len) ? RX_OK : (got_tmp == 0 ? RX_REPLY_NONE : RX_REPLY_PARTIAL);
    return got_tmp == a_rx_len;
  }

  /// @brief 通信速度bps.
  unsigned long baud() const { return m_baud; }

  /// @brief 直前の送信データ数(エコーのバイト数).
  size_t tx_len() const { return m_tx_len; }

  /// @brief 通信速度だけを切り替える(1系統に2種類のサーボを混在させる場合. begin()より軽い).
  ///        フレーム形式と信号の向きは変えないため, 8N1のプロトコル同士でのみ使う. 送受信の合間に呼ぶ.
  void set_baud(unsigned long a_baud) {
    if (a_baud == m_baud || !m_began) {
      return;
    }
    m_serial->updateBaudRate(a_baud);
    m_baud = a_baud;
  }

  /// @brief receive() の結果の種類.
  enum RxResult : uint8_t {
    RX_OK = 0,            // 成功
    RX_ECHO_SHORT = 1,    // 送信エコーが足りない
    RX_ECHO_MISMATCH = 2, // 送信エコーが一致しない(信号線上のぶつかり等)
    RX_REPLY_NONE = 3,    // 応答が1バイトも来ない
    RX_REPLY_PARTIAL = 4, // 応答が途中までしか来ない(先頭欠け等)
    RX_RESULT_NUM = 5
  };

  /// @brief 直前の receive() の結果の種類(RxResult).
  uint8_t last_rx() const { return m_last_rx; }

private:
  /// @brief 受信バッファに残っているデータを読み捨てる.
  void flush_rx() {
    while (m_serial->available() > 0) {
      m_serial->read();
    }
  }

  /// @brief ダミーバイトを送信し, RXにエコーされるかを判定する.
  ///        0x55はICSではコマンドバイト(MSB=1)ではなく, DXL/FEETECH/FUTABAではヘッダを欠くため,
  ///        いずれのサーボにも無視される.
  ///        エコーのない回路ではダミーバイトは1バイトも返らないため, 1回でも受信できればエコーありとする
  ///        (通信速度の切替直後の雑音等で1回取りこぼしても誤判定しないため).
  /// @return いずれかの試行でエコーを受信できればtrue.
  bool detect_echo() {
    const uint8_t probe_tmp = 0x55;
    const int trials_tmp = 5;
    bool echo_tmp = false;
    for (int i = 0; i < trials_tmp && !echo_tmp; i++) {
      delay(1); // 切替直後の雑音を読み捨ててから試す
      flush_rx();
      dir_tx();
      m_serial->write(probe_tmp);
      m_serial->flush();
      dir_rx();
      uint8_t rcv_tmp;
      echo_tmp = (m_serial->readBytes(&rcv_tmp, 1) == 1 && rcv_tmp == probe_tmp);
    }
    flush_rx();
    return echo_tmp;
  }

  // 送信: 先にA端子を入力にしてからTXを出力する / 受信: 先にTXを切ってからA端子を出力にする
  inline void dir_tx() {
    digitalWrite(m_pin_dir, HIGH);
    mrd_hduart_tx_output(m_pin_tx_release, true);
  }
  inline void dir_rx() {
    mrd_hduart_tx_output(m_pin_tx_release, false);
    digitalWrite(m_pin_dir, LOW);
  }

  HardwareSerial *m_serial;
  int8_t m_pin_rx;
  int8_t m_pin_tx;
  int8_t m_pin_dir;
  uint8_t m_echo_mode;
  bool m_echo;
  unsigned long m_timeout_ms;
  bool m_began = false;
  bool m_idle_tx = false;                                // 送受信の合間を送信方向で待つか
  unsigned long m_baud = 0;                              // 通信速度bps(区切り受信の時間計算用)
  uart_dev_t *m_uart_hw = nullptr;                       // 送信完了をレジスタで確認するためのUART
  portMUX_TYPE m_dir_mux = portMUX_INITIALIZER_UNLOCKED; // end_send()の割り込み停止用
  int8_t m_pin_tx_release;                               // 受信中に出力を切るTXピン(-1:切らない)
  uint8_t m_tx_copy[MRD_HDUART_TX_MAX];                  // エコー照合用の送信データの控え
  size_t m_tx_len = 0;                                   // 直前の送信データ数
  uint8_t m_last_rx = RX_OK;                             // 直前の receive() の結果の種類
  size_t m_last_rx_len = 0;                              // 直前の receive() で受信できた応答のバイト数
};

//==================================================================================================
//  2系統の並行送受信 (プロトコル非依存)
//==================================================================================================
//
// L/R系統の送受信を並行して行う. 系統ごとにプロトコルが違ってもよい(例: L=FEETECH, R=ICS. L/Rの入れ替えも可).
// 各プロトコル層は MrdLineStepper を実装し, 「次に送るパケット」と「結果の反映」を1回ずつ提供する.
// 各系統は自分のペースで進む(mrd_servo_lines_run). 片方が長い応答を待つ間も, もう片方は次の送受信を進める.

/// @brief 1系統分の1回の送受信.
struct MrdHduartXfer {
  MrdHalfDuplexUart *bus = nullptr; // nullptrまたはtx_len=0なら送らない(ok=falseになる)
  const uint8_t *tx = nullptr;      // 送信データ
  size_t tx_len = 0;                // 送信データ数
  uint8_t *rx = nullptr;            // 受信データ格納先
  size_t rx_len = 0;                // 受信データ数(0なら送信のみ)
  unsigned long timeout_ms = 0;     // 受信タイムアウト(ms, 0:バスの既定値)
  unsigned long hold_us = 0;        // 送信完了後, 次の送信まで線を開放して待つ時間(us). 返信を読まずに
                                    // サーボの返信枠だけ空ける場合(Hitecの位置命令)に使う
  bool framed = false;              // 返信をホストの区切りで受けるか(Hitecの読み出し命令. rx_lenバイト)
  bool ok = false;                  // 結果(エコー照合と受信に成功すればtrue)
  size_t rx_got = 0;                // 受信できたバイト数(途中までの応答の解析用)

  // 待たない送受信(mrd_hduart_xfer_start/poll)の進み具合
  enum Phase : uint8_t {
    XF_TX,   // 送信の完了待ち(応答なし, 区切り受信)
    XF_RX,   // エコーと応答の受信待ち
    XF_HOLD, // 返信枠の待ち(hold_us)
    XF_DONE  // 完了
  };
  Phase phase = XF_DONE;
  uint32_t t0_us = 0;   // 受信待ちを始めた時刻(us)
  uint32_t done_us = 0; // 受信が終わった時刻(us)
};

/// @brief 1回の送受信を始める(待たない). 続けて mrd_hduart_xfer_poll() で完了まで進める.
///        応答あり: 送信開始から受信切替まで割り込みを止めて行う(send_multi. 応答の先頭を欠かさないため).
///        応答なし, 区切り受信: ドライバ経由で送り始め, 送信完了は mrd_hduart_xfer_poll() で見張る.
/// @param a_x 送受信. 結果(ok, rx_got)は完了時に入る.
/// @return 送り始められればtrue. falseなら送っていない(ok=false).
bool mrd_hduart_xfer_start(MrdHduartXfer &a_x) {
  a_x.ok = false;
  a_x.rx_got = 0;
  a_x.phase = MrdHduartXfer::XF_DONE;
  if (a_x.bus == nullptr || a_x.tx_len == 0) {
    return false;
  }
  MrdHalfDuplexUart *bus_tmp = a_x.bus;
  if (a_x.rx_len > 0 && !a_x.framed) {
    if (!bus_tmp->prepare_send(a_x.tx, a_x.tx_len)) {
      return false;
    }
    MrdHalfDuplexUart::send_multi(&bus_tmp, 1);
    a_x.t0_us = micros();
    a_x.phase = MrdHduartXfer::XF_RX;
  } else {
    if (!bus_tmp->begin_send(a_x.tx, a_x.tx_len)) {
      return false;
    }
    a_x.phase = MrdHduartXfer::XF_TX;
  }
  return true;
}

/// @brief mrd_hduart_xfer_start() で始めた送受信を進める(待たない). 呼ぶたびにできるところまで進める.
///        送信完了 → 受信切替(または区切り受信) → エコーと応答の受信 → 返信枠の待ち → 待機状態(idle) の順.
///        受信は「エコーと応答が揃う」か「タイムアウト(転送時間 + 受信タイムアウト)」で終える.
/// @param a_x 送受信.
/// @return 完了したらtrue(結果は a_x.ok, a_x.rx_got).
bool mrd_hduart_xfer_poll(MrdHduartXfer &a_x) {
  MrdHalfDuplexUart *bus_tmp = a_x.bus;
  if (a_x.phase == MrdHduartXfer::XF_TX) {
    if (!bus_tmp->tx_done()) {
      return false;
    }
    if (a_x.framed && a_x.rx_len > 0) {
      MrdHalfDuplexUart::framed_reply_multi(&bus_tmp, 1, a_x.rx_len);
    } else {
      bus_tmp->end_send();
    }
    a_x.t0_us = micros();
    a_x.phase = MrdHduartXfer::XF_RX;
  }
  if (a_x.phase == MrdHduartXfer::XF_RX) {
    const size_t need_tmp = (bus_tmp->echo() ? bus_tmp->tx_len() : 0) + a_x.rx_len;
    // 転送時間(1バイト11ビットで見積もる)に受信タイムアウトを加えた時間まで待つ
    const uint32_t limit_us_tmp = uint32_t(uint64_t(need_tmp) * 11000000ULL / max(bus_tmp->baud(), 1200UL)) +
                                  (a_x.timeout_ms > 0 ? a_x.timeout_ms : bus_tmp->timeout_ms()) * 1000UL;
    if (bus_tmp->rx_available() < need_tmp && micros() - a_x.t0_us < limit_us_tmp) {
      return false;
    }
    a_x.ok = bus_tmp->receive_now(a_x.rx, a_x.rx_len);
    a_x.rx_got = bus_tmp->last_rx_len();
    a_x.done_us = micros();
    a_x.phase = MrdHduartXfer::XF_HOLD;
  }
  if (a_x.phase == MrdHduartXfer::XF_HOLD) {
    // 返信枠を空ける: 受信が終わってから指定時間, 受信方向のまま待つ
    if (a_x.hold_us > 0 && micros() - a_x.done_us < a_x.hold_us) {
      return false;
    }
    bus_tmp->idle();
    a_x.phase = MrdHduartXfer::XF_DONE;
  }
  return a_x.phase == MrdHduartXfer::XF_DONE;
}

/// @brief 1系統分のサーボ処理を, 1回ずつの送受信に分けて進めるためのインタフェース.
class MrdLineStepper {
public:
  virtual ~MrdLineStepper() {}
  /// @brief 次の送受信を準備する.
  /// @return 準備できればtrue. この系統の処理が終わっていればfalse.
  virtual bool next(MrdHduartXfer &a_x) = 0;
  /// @brief next()で準備した送受信の結果を反映する.
  virtual void complete(const MrdHduartXfer &a_x) = 0;
  /// @brief このフレームは現在位置を読まない(混在時の交互読み出し, マスターコマンド2).
  ///        next()より前に呼ぶ. 読まないサーボは前回の値になる. 対応しないプロトコル(ICS, PMX, Hitec, XBUS)は何もしない.
  virtual void skip_reads() {}
};

/// @brief 複数系統の処理を, 系統ごとに独立して最後まで実行する.
///        各系統は送受信が終わり次第, 結果を反映して次の送受信を始める. 片方の系統が長い応答を待つ間も,
///        もう片方は次の送受信を進めるため, 全体の時間は「長い方の系統」でほぼ決まる.
///        系統の並び(L/R)やプロトコルの組み合わせに依存しない.
///        応答の先頭を欠かさないための割り込み停止(送信から受信切替まで)は系統ごとに行い, その間に
///        もう片方の系統へ届いたデータはUARTの受信FIFO(128byte)に保持される.
/// @param a_st 各系統の処理(nullptrなら処理しない). 添字は統計(hduart_stat)の系統番号([0]:L, [1]:R).
/// @param a_num 系統数(2以下).
void mrd_servo_lines_run(MrdLineStepper *const *a_st, int a_num) {
  const int num_tmp = min(a_num, 2);
  MrdHduartXfer x_tmp[2];
  bool active_tmp[2] = {false, false};
  bool busy_tmp[2] = {false, false};
  for (int l = 0; l < num_tmp; l++) {
    active_tmp[l] = (a_st[l] != nullptr);
  }
  while (active_tmp[0] || active_tmp[1]) {
    for (int l = 0; l < num_tmp; l++) {
      if (!active_tmp[l]) {
        continue;
      }
      if (!busy_tmp[l]) {
        x_tmp[l] = MrdHduartXfer();
        if (!a_st[l]->next(x_tmp[l])) {
          active_tmp[l] = false;
          continue;
        }
        if (!mrd_hduart_xfer_start(x_tmp[l])) { // 送れなかった(バスなし, 送信データなし等): 失敗として反映
          a_st[l]->complete(x_tmp[l]);
          continue;
        }
        busy_tmp[l] = true;
      }
      if (!mrd_hduart_xfer_poll(x_tmp[l])) {
        continue;
      }
      busy_tmp[l] = false;
      if (!x_tmp[l].ok) {
        hduart_stat.fail[l][x_tmp[l].bus->last_rx()]++;
      }
      if (x_tmp[l].rx_len > 0) {
        const uint32_t wait_tmp = x_tmp[l].done_us - x_tmp[l].t0_us;
        hduart_stat.sum_us[l] += wait_tmp;
        hduart_stat.max_us[l] = max(hduart_stat.max_us[l], wait_tmp);
        hduart_stat.count[l]++;
      }
      a_st[l]->complete(x_tmp[l]);
    }
  }
}

//==================================================================================================
//  サーボの共通処理 (フレーム番号, スロットの扱い, 混在, パラメータ系コマンドの状態, モーションの補間)
//==================================================================================================
//
// 各プロトコル層(mrd_module/sv_*.h)の共通部品は sv_util.h にある. ここには, どのプロトコルにも関係する
// 「Meridimのサーボコマンドの解釈」と「系統の組み立て」に関わるものを置く.

/// @brief サーボ駆動のフレーム番号. mrd_servo_drive_lite() の呼び出しごとに1増える.
///        50Hzで動かすサーボ(PWM, Hitec, XBUS)は, Meridianの100Hzのフレームを偶数/奇数で使い分ける.
static uint32_t mrd_servo_frame = 0;

//------------------------------------------------------------------------------------
//  サーボのパラメータ系コマンド(Meridim90 汎化コマンド)の状態
//------------------------------------------------------------------------------------
//
// サーボごとのコマンド(Meridimの[i*2])の下位8bitが3以上なら, そのサーボのパラメータを読み書きする.
// 上位ビット: bit15 スルー(通信せず受信値をそのまま返す), bit13 車輪(下位8bitが1のとき, VALを速度0.01rpmとして送り
// 現在速度を返す. 車輪モードはサーボ側で事前に設定しておく. PMXは制御モードを自動で切り替える), bit14 返信時のサーボエラーフラグ.
// 番号と値の形式は tasks/servo_param_plan.md (3〜38: gs2d準拠, 50以降: 各社固有).
// コマンドかVALが変わったフレームで1回だけ実行し, 結果を保持して返し続ける. その間は, そのサーボの
// 通常の処理(トルク, 目標位置, 現在位置)を行わない(トルク状態と位置を保持する). 実行は各プロトコルの
// ステッパーの最後に, そのサーボの位置指示の代わりとして行う.

#define MRD_PCMD_MIN       3      // パラメータ系コマンドの最小値
#define MRD_PCMD_ERR_FLAG  0x4000 // 返信のサーボエラーフラグ(コマンドのbit14)
#define MRD_PCMD_NA        -32768 // 返信のVAL: 未対応(機種にない項目, 対応していないコマンド)
#define MRD_PCMD_NO_REPLY  -32767 // 返信のVAL: 応答なし
#define MRD_PCMD_SERVO_ERR -32766 // 返信のVAL: サーボがエラーを返した(トルクON中のEEPROM書込等)
#define MRD_CMD_THROUGH    0x8000 // コマンドのbit15: スルー
#define MRD_CMD_WHEEL      0x2000 // コマンドのbit13: 車輪(無限回転)

/// @brief スロットの扱い(コマンドの値で決まる).
enum MrdSlotMode : uint8_t {
  MRD_SLOT_NORMAL = 0, // 通常(0/2:脱力, 1:位置)
  MRD_SLOT_PARAM,      // パラメータ系コマンド(下位8bitが3以上)
  MRD_SLOT_WHEEL,      // 車輪(bit13 かつ下位8bitが1)
  MRD_SLOT_THROUGH     // スルー(bit15)
};

/// @brief コマンドの値からスロットの扱いを決める.
inline uint8_t mrd_servo_slot_mode(short a_cmd) {
  const uint16_t cmd_tmp = uint16_t(a_cmd);
  if (cmd_tmp & MRD_CMD_THROUGH) {
    return MRD_SLOT_THROUGH;
  }
  if ((cmd_tmp & 0xFF) >= MRD_PCMD_MIN) {
    return MRD_SLOT_PARAM;
  }
  if ((cmd_tmp & MRD_CMD_WHEEL) && (cmd_tmp & 0xFF) == 1) {
    return MRD_SLOT_WHEEL;
  }
  return MRD_SLOT_NORMAL;
}

/// @brief 1スロット分のパラメータ系コマンド・車輪・スルーの状態.
struct MrdServoParamSlot {
  uint8_t mode = MRD_SLOT_NORMAL; // MrdSlotMode. NORMAL以外のスロットは通常の処理(トルク, 目標位置, 現在位置)から外す
  uint8_t cmd = 0;                // 実行中・保持中のコマンド(0:なし)
  int16_t val = 0;                // 受け取ったVAL(車輪は速度0.01rpm)
  int16_t result = 0;             // 結果(返信のVAL. 車輪は現在速度0.01rpm)
  bool due = false;               // 未実行(このフレームで実行する. 車輪は毎フレーム)
};

/// @brief 系統ごとのパラメータ系コマンドの状態([0]:L, [1]:R). mrd_servo_param_prepare() で更新する.
static MrdServoParamSlot mrd_servo_param[2][MRD_SERVO_SLOTS];

/// @brief Meridimのサーボコマンドの下位8bit(汎化コマンド).
inline uint8_t mrd_servo_cmd(short a_cmd) { return uint8_t(a_cmd & 0xFF); }

/// @brief 系統に通常以外(パラメータ系コマンド, 車輪, スルー)のスロットがあるか.
inline bool mrd_servo_param_in_line(int a_line) {
  if (a_line != L && a_line != R) {
    return false;
  }
  for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
    if (mrd_servo_param[a_line][i].mode != MRD_SLOT_NORMAL) {
      return true;
    }
  }
  return false;
}

/// @brief パラメータ系コマンドの結果がエラー値か.
inline bool mrd_servo_param_error(int16_t a_result) { return a_result <= MRD_PCMD_SERVO_ERR; }

//------------------------------------------------------------------------------------
//  1系統に2種類のサーボを混在させる場合の共通処理
//------------------------------------------------------------------------------------
//
// 系統の種類(MOUNT_SERVO_TYPE_x)に加えて2種類目(MOUNT_SERVO_TYPE_x2)を指定すると, スロットの種類
// (IXL_MT/IXR_MT)ごとにプロトコルを切り替えて同じ信号線で通信する. 1フレームの処理は
// 1種類目 → (通信速度の切替) → 2種類目 の順で, 目標位置は毎フレーム両方に送り, 現在位置は種類ごとに
// 1フレームずつ交互に読む(読まないフレームは前回の値).
// 混在できるのはチェックサム付きで8N1のFUTABA(21), DYNAMIXEL 1.0(31)/2.0(32), FEETECH(61/62)のみ.
// ICS(43)はチェックサムがなく他方の通信を誤って命令と受け取る恐れがあるため, PMX(44)はRS-485のため, Hitec(71)は信号反転のため,
// XBUS(51)とPWMは方式が違うため, 混在させない.

/// @brief 同じプロトコルとして扱うサーボタイプの代表値(FEETECHのSTS/SCSは61).
constexpr int mrd_servo_family(int a_type) { return (a_type == FTCSCS) ? FTCSTS : a_type; }

/// @brief 1系統に混在させられるサーボタイプか.
constexpr bool mrd_servo_is_mixable(int a_type) {
  return mrd_servo_family(a_type) == FTBRSX || a_type == DXL1 || a_type == DXL2 || mrd_servo_family(a_type) == FTCSTS;
}

/// @brief 系統ごとの2種類目のサーボタイプ([0]:L, [1]:R. 0:混在なし). mrd_servo_mix_check() で設定する.
static int mrd_servo_type2[2] = {0, 0};

/// @brief 系統と種類に対応するマウント状態を返す. 混在なしでパラメータ系コマンドもなければ系統のマウント状態
///        そのもの, それ以外はその種類のスロットだけをマウントとした配列(静的領域. 呼ぶたびに作り直す).
/// @param a_line 系統(L or R).
/// @param a_type サーボタイプ(混在なしなら使わない).
/// @param a_sv サーボパラメータ.
/// @param a_drive trueなら通常の駆動用: 通常以外(パラメータ系コマンド, 車輪, スルー)のスロットを除く.
///        falseならパラメータ系コマンドの実行用: 除かない.
const int *mrd_servo_line_mount(UartLine a_line, int a_type, const ServoParam &a_sv, bool a_drive = true) {
  const int *mount_tmp = (a_line == L) ? a_sv.ixl_mount : a_sv.ixr_mount;
  if (a_line != L && a_line != R) {
    return mount_tmp;
  }
  const bool mix_tmp = (mrd_servo_type2[a_line] != 0);
  const bool param_tmp = a_drive && mrd_servo_param_in_line(a_line);
  if (!mix_tmp && !param_tmp) {
    return mount_tmp;
  }
  static int mix_mount_tmp[2][2][2][MRD_SERVO_SLOTS]; // [駆動用/実行用][系統][1種類目/2種類目]
  const float *type_tmp = (a_line == L) ? a_sv.ixl_type : a_sv.ixr_type;
  const int family_tmp = mrd_servo_family(a_type);
  const int group_tmp = (mix_tmp && family_tmp == mrd_servo_family(mrd_servo_type2[a_line])) ? 1 : 0;
  int *out_tmp = mix_mount_tmp[a_drive ? 0 : 1][a_line][group_tmp];
  for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
    out_tmp[i] = (i < a_sv.num_max && mount_tmp[i] && (!mix_tmp || mrd_servo_family(int(type_tmp[i])) == family_tmp) &&
                  !(param_tmp && mrd_servo_param[a_line][i].mode != MRD_SLOT_NORMAL))
                     ? 1
                     : 0;
  }
  return out_tmp;
}

/// @brief 系統の2種類目のサーボタイプを検査して設定する. 混在できない組み合わせなら混在なしとし,
///        どちらの種類にも当たらないスロットはマウントを外す(警告を表示する).
/// @param a_line 系統(L or R).
/// @param a_type 系統のサーボタイプ(1種類目).
/// @param a_type2 2種類目のサーボタイプ(0:混在なし).
/// @param a_sv サーボパラメータ.
/// @return 混在させる場合はtrue.
bool mrd_servo_mix_check(UartLine a_line, int a_type, int a_type2, ServoParam &a_sv) {
  if (a_line != L && a_line != R) {
    return false;
  }
  mrd_servo_type2[a_line] = 0;
  if (a_type2 == NOSERVO) {
    return false;
  }
  const char *name_tmp = (a_line == L) ? "L" : "R";
  if (!mrd_servo_is_mixable(a_type) || !mrd_servo_is_mixable(a_type2) ||
      mrd_servo_family(a_type) == mrd_servo_family(a_type2)) {
    Serial.print("UART_");
    Serial.print(name_tmp);
    Serial.println(" servo mix: only two different types of 21/31/32/61/62 can be mixed. Mix disabled.");
    return false;
  }
  mrd_servo_type2[a_line] = a_type2;
  int *mount_tmp = (a_line == L) ? a_sv.ixl_mount : a_sv.ixr_mount;
  const float *type_tmp = (a_line == L) ? a_sv.ixl_type : a_sv.ixr_type;
  for (int i = 0; i < a_sv.num_max && i < MRD_SERVO_SLOTS; i++) {
    const int family_tmp = mrd_servo_family(int(type_tmp[i]));
    if (mount_tmp[i] && family_tmp != mrd_servo_family(a_type) && family_tmp != mrd_servo_family(a_type2)) {
      mount_tmp[i] = 0;
      Serial.print("UART_");
      Serial.print(name_tmp);
      Serial.print(" servo mix: slot [");
      Serial.print(i);
      Serial.print("] type ");
      Serial.print(int(type_tmp[i]));
      Serial.println(" is neither line type. Unmounted.");
    }
  }
  return true;
}

//------------------------------------------------------------------------------------
//  サーボのパラメータ系コマンドの共通処理
//------------------------------------------------------------------------------------

/// @brief パラメータ系コマンドに対応するサーボタイプか(ICS, DYNAMIXEL 1.0/2.0, FEETECH, FUTABA, PMX, Hitec).
constexpr bool mrd_servo_param_supported(int a_type) {
  return a_type == KOICS3 || a_type == DXL1 || a_type == DXL2 || mrd_servo_family(a_type) == FTCSTS ||
         a_type == FTBRSX || a_type == KOPMX || a_type == HTCHSR;
}

/// @brief 車輪(bit13)に対応するサーボタイプか(DYNAMIXEL 1.0/2.0, FEETECH, PMX). 車輪モードはサーボ側で事前に設定する(PMXは自動で切り替える).
constexpr bool mrd_servo_wheel_supported(int a_type) {
  return a_type == DXL1 || a_type == DXL2 || mrd_servo_family(a_type) == FTCSTS || a_type == KOPMX;
}

/// @brief フレームの最初に, スロットごとのコマンドからスロットの扱いを決め, 実行待ちを決める.
///        パラメータ系コマンド: 受け取ったコマンドとVALを前回と比べ, 変わっていれば実行待ちにする.
///        車輪: 毎フレーム実行待ちにする(VALは速度). スルー・通常: 状態を消す(パラメータ系コマンドは再実行できる).
///        マウントしていないスロットと, 対応しないサーボのスロットは未対応の結果にする(通信しない).
/// @param a_meridim Meridimデータ.
/// @param a_l_type L系統のサーボタイプ.
/// @param a_r_type R系統のサーボタイプ.
/// @param a_sv サーボパラメータ.
void mrd_servo_param_prepare(const Meridim90Union &a_meridim, int a_l_type, int a_r_type, const ServoParam &a_sv) {
  for (int l = 0; l < 2; l++) {
    const int orig_tmp = (l == 0) ? MRD_L_ORIGIDX : MRD_R_ORIGIDX;
    const int line_type_tmp = (l == 0) ? a_l_type : a_r_type;
    const int *mount_tmp = (l == 0) ? a_sv.ixl_mount : a_sv.ixr_mount;
    const float *type_tmp = (l == 0) ? a_sv.ixl_type : a_sv.ixr_type;
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      MrdServoParamSlot &slot_tmp = mrd_servo_param[l][i];
      const short raw_tmp = a_meridim.sval[orig_tmp + i * 2];
      const uint8_t cmd_tmp = mrd_servo_cmd(raw_tmp);
      const int16_t val_tmp = a_meridim.sval[orig_tmp + 1 + i * 2];
      const uint8_t mode_tmp = mrd_servo_slot_mode(raw_tmp);
      if (mode_tmp == MRD_SLOT_NORMAL || mode_tmp == MRD_SLOT_THROUGH) {
        slot_tmp = MrdServoParamSlot();
        slot_tmp.mode = mode_tmp;
        continue;
      }
      if (mode_tmp == MRD_SLOT_PARAM && slot_tmp.mode == MRD_SLOT_PARAM && slot_tmp.cmd == cmd_tmp &&
          slot_tmp.val == val_tmp) {
        continue; // 実行済み(結果を返し続ける)
      }
      if (slot_tmp.mode != mode_tmp) {
        slot_tmp = MrdServoParamSlot();
        slot_tmp.mode = mode_tmp;
      }
      slot_tmp.cmd = cmd_tmp;
      slot_tmp.val = val_tmp;
      slot_tmp.due = true;
      if (mode_tmp == MRD_SLOT_PARAM) {
        slot_tmp.result = 0;
      }
      // 駆動するサーボの種類: 混在する系統はスロットの種類, それ以外は系統の種類
      const int drive_type_tmp = (mrd_servo_type2[l] != 0) ? int(type_tmp[i]) : line_type_tmp;
      const bool supported_tmp = (mode_tmp == MRD_SLOT_PARAM) ? mrd_servo_param_supported(drive_type_tmp)
                                                              : mrd_servo_wheel_supported(drive_type_tmp);
      if (i >= a_sv.num_max || !mount_tmp[i] || line_type_tmp == NOSERVO || !supported_tmp) {
        slot_tmp.result = MRD_PCMD_NA;
        slot_tmp.due = false;
      }
    }
  }
}

//------------------------------------------------------------------------------------
//  モーションの補間 (Meridim[19] MRD_MOTION_FRAMES: 目標位置への移動に使うフレーム数)
//------------------------------------------------------------------------------------

/// @brief 1スロット分の補間の状態.
struct MrdMotion {
  float from = 0.0f; // 補間の始点(degree)
  float goal = 0.0f; // 補間の終点(degree)
  int16_t val = 0;   // 終点として受け取ったVAL
  uint16_t left = 0; // 残りフレーム数
  uint16_t total = 0;
  bool on = false; // 補間中か(トルクONで目標を受け取っている)
};

/// @brief 系統ごとの補間の状態([0]:L, [1]:R).
static MrdMotion mrd_servo_motion[2][MRD_SERVO_SLOTS];

/// @brief 受け取った目標位置から, このフレームの指令位置を求める.
///        目標(VAL)が変わったら, いまの指令位置から新しい目標まで a_frames フレームかけて直線で補間する.
///        a_frames が0か1なら即時. 補間の途中で目標が変わったら, その時点の指令位置から補間し直す.
/// @param a_m 補間の状態.
/// @param a_val 受け取ったVAL(degree x100).
/// @param a_now いまの指令位置(degree. 前回の指令値).
/// @param a_frames 移動に使うフレーム数.
/// @return このフレームの指令位置(degree).
float mrd_servo_motion_target(MrdMotion &a_m, int16_t a_val, float a_now, int a_frames) {
  if (!a_m.on || a_val != a_m.val) {
    a_m.on = true;
    a_m.val = a_val;
    a_m.from = a_now;
    a_m.goal = a_val * 0.01f;
    a_m.total = uint16_t(constrain(a_frames, 1, 65535));
    a_m.left = a_m.total;
  }
  if (a_m.left == 0) {
    return a_m.goal;
  }
  a_m.left--;
  const float k_tmp = float(a_m.total - a_m.left) / float(a_m.total);
  return a_m.from + (a_m.goal - a_m.from) * k_tmp;
}

/// @brief 補間をやめる(トルクOFF, パラメータ系コマンド等). 次に目標を受け取ったときに始点から補間し直す.
inline void mrd_servo_motion_reset(MrdMotion &a_m) { a_m.on = false; }

/// @brief 次のフレームで現在位置を読まないか(マスターコマンド2: 全サーボ脱力, 位置は読まない).
///        mrd_servo_drive_lite() で使い, 使い終わったら戻す.
static bool mrd_servo_skip_reads_frame = false;

// 各プロトコル層の読み込み (半二重UARTバスと共通処理の定義より後に読み込むこと)
// 各プロトコル層は必要な共通部品(sv_util.h 等)を自分で読み込むため, 並び順は問わない
#include "mrd_module/sv_dxl1.h"
#include "mrd_module/sv_dxl2.h"
#include "mrd_module/sv_ftbrx.h"
#include "mrd_module/sv_ftc.h"
#include "mrd_module/sv_hitec.h"
#include "mrd_module/sv_ics.h"
#include "mrd_module/sv_pmx.h"
#include "mrd_module/sv_pwm.h"
#include "mrd_module/sv_util.h"
#include "mrd_module/sv_xbus.h"

/// @brief 2つの値の大きい方(コンパイル時定数用).
constexpr size_t mrd_size_max(size_t a_a, size_t a_b) { return a_a > a_b ? a_a : a_b; }

/// @brief 1系統分のステッパーを置く領域の大きさ(byte). 全プロトコルのステッパーのうち最大のもの.
///        プロトコルを追加したら, ここにそのステッパーを加える.
constexpr size_t MRD_STEPPER_BUF_BYTES =                                                                                      //
    mrd_size_max(sizeof(IcsLineStepper),                                                                                      // ICS
                 mrd_size_max(sizeof(Dxl1LineStepper),                                                                        // DYNAMIXEL 1.0
                              mrd_size_max(sizeof(Dxl2LineStepper),                                                           // DYNAMIXEL 2.0
                                           mrd_size_max(sizeof(FtcLineStepper),                                               // FEETECH
                                                        mrd_size_max(sizeof(FtbLineStepper),                                  // FUTABA
                                                                     mrd_size_max(sizeof(PmxLineStepper),                     // PMX
                                                                                  mrd_size_max(sizeof(XbusLineStepper),       // XBUS
                                                                                               sizeof(HtcLineStepper)))))))); // Hitec

/// @brief 1系統分のステッパー(MrdLineStepperの各実装)を置く領域.
///        ステッパーは毎フレーム作り直すため, スタックではなくこの静的な領域に置く(placement new).
struct MrdLineStepperBuf {
  alignas(8) uint8_t raw[MRD_STEPPER_BUF_BYTES];
};

/// @brief 系統のサーボタイプに応じたステッパーを a_buf に生成する.
/// @param a_line 系統(L or R).
/// @param a_type 系統のサーボタイプ.
/// @param a_buf 生成先.
/// @param a_meridim Meridimデータの参照.
/// @param a_sv サーボパラメータ.
/// @return 生成したステッパー. 未対応のタイプ(サーボなしを含む)ならnullptr.
///         使い終わったら ~MrdLineStepper() を呼ぶこと.
MrdLineStepper *mrd_servo_stepper_make(UartLine a_line, int a_type, MrdLineStepperBuf &a_buf,
                                       Meridim90Union &a_meridim, ServoParam &a_sv) {
  static_assert(alignof(FtcLineStepper) <= 8 && alignof(Dxl2LineStepper) <= 8 && alignof(Dxl1LineStepper) <= 8 &&
                    alignof(FtbLineStepper) <= 8 && alignof(IcsLineStepper) <= 8 && alignof(PmxLineStepper) <= 8 &&
                    alignof(XbusLineStepper) <= 8 && alignof(HtcLineStepper) <= 8,
                "MrdLineStepperBuf is aligned to 8 bytes");
  // パラメータ系コマンドの実行用のマウント状態(パラメータ系コマンドのスロットを除かない)
  const int *all_mount_tmp = mrd_servo_line_mount(a_line, a_type, a_sv, false);
  if (mrd_ftc_is_ftc(a_type)) {
    return new (a_buf.raw) FtcLineStepper(mrd_ftc_line_ref(a_line, a_meridim, a_sv), all_mount_tmp, a_sv.num_max);
  }
  if (a_type == DXL2) {
    return new (a_buf.raw) Dxl2LineStepper(mrd_dxl2_line_ref(a_line, a_meridim, a_sv), all_mount_tmp, a_sv.num_max,
                                           mrd_dxl2_line(a_line).baud());
  }
  if (a_type == DXL1) {
    return new (a_buf.raw) Dxl1LineStepper(mrd_dxl1_line_ref(a_line, a_meridim, a_sv), all_mount_tmp, a_sv.num_max);
  }
  if (a_type == FTBRSX) {
    return new (a_buf.raw) FtbLineStepper(mrd_ftb_line_ref(a_line, a_meridim, a_sv), all_mount_tmp, a_sv.num_max);
  }
  if (a_type == KOICS3) {
    return new (a_buf.raw) IcsLineStepper(mrd_ics_line_ref(a_line, a_meridim, a_sv), all_mount_tmp, a_sv.num_max);
  }
  if (a_type == KOPMX) {
    return new (a_buf.raw) PmxLineStepper(mrd_pmx_line_ref(a_line, a_meridim, a_sv), all_mount_tmp, a_sv.num_max);
  }
  if (a_type == JRXBUS) {
    return new (a_buf.raw) XbusLineStepper(mrd_xbus_line_ref(a_line, a_meridim, a_sv), a_sv.num_max);
  }
  if (a_type == HTCHSR) {
    return new (a_buf.raw) HtcLineStepper(mrd_htc_line_ref(a_line, a_meridim, a_sv), all_mount_tmp, a_sv.num_max,
                                          mrd_htc_line(a_line));
  }
  return nullptr;
}

/// @brief 混在できるサーボタイプの, 系統のプロトコル層が使う半二重UARTバス.
/// @return バス. 混在できないタイプならnullptr.
MrdHalfDuplexUart *mrd_servo_mix_bus(UartLine a_line, int a_type) {
  switch (mrd_servo_family(a_type)) {
  case FTCSTS:
    return &mrd_ftc_line(a_line).bus();
  case DXL1:
    return &mrd_dxl1_line(a_line).bus();
  case DXL2:
    return &mrd_dxl2_line(a_line).bus();
  case FTBRSX:
    return &mrd_ftb_line(a_line).bus();
  default:
    return nullptr;
  }
}

/// @brief 混在できるサーボタイプの, 系統のプロトコル層が使う通信速度.
///        (バスの現在の通信速度ではなく, そのプロトコルに設定した値. バスの値は切り替えるたびに変わるため)
/// @return 通信速度bps. 混在できないタイプなら0.
unsigned long mrd_servo_mix_baud(UartLine a_line, int a_type) {
  switch (mrd_servo_family(a_type)) {
  case FTCSTS:
    return mrd_ftc_line(a_line).baud();
  case DXL1:
    return mrd_dxl1_line(a_line).baud();
  case DXL2:
    return mrd_dxl2_line(a_line).baud();
  case FTBRSX:
    return mrd_ftb_line(a_line).baud();
  default:
    return 0;
  }
}

/// @brief 1系統に混在させた2種類のサーボの処理を, 1種類ずつ順に進める.
///        種類が変わるときに通信速度を切り替える. 現在位置は a_read_first がtrueなら1種類目だけ,
///        falseなら2種類目だけを読む(フレームごとに交互にする).
class MrdMixedLineStepper : public MrdLineStepper {
public:
  /// @param a_bus 系統の半二重UARTバス.
  /// @param a_first 1種類目の処理. a_baud_first はその通信速度.
  /// @param a_second 2種類目の処理. a_baud_second はその通信速度.
  /// @param a_read_first このフレームで現在位置を読むのが1種類目か.
  MrdMixedLineStepper(MrdHalfDuplexUart *a_bus, MrdLineStepper *a_first, unsigned long a_baud_first,
                      MrdLineStepper *a_second, unsigned long a_baud_second, bool a_read_first)
      : m_bus(a_bus), m_st{a_first, a_second}, m_baud{a_baud_first, a_baud_second} {
    m_st[a_read_first ? 1 : 0]->skip_reads();
  }

  ~MrdMixedLineStepper() override {
    m_st[0]->~MrdLineStepper();
    m_st[1]->~MrdLineStepper();
  }

  void skip_reads() override {
    m_st[0]->skip_reads();
    m_st[1]->skip_reads();
  }

  bool next(MrdHduartXfer &a_x) override {
    while (m_cur < 2) {
      m_bus->set_baud(m_baud[m_cur]); // 前の送受信は完了しているので切り替えてよい
      if (m_st[m_cur]->next(a_x)) {
        return true;
      }
      m_cur++;
    }
    return false;
  }

  void complete(const MrdHduartXfer &a_x) override { m_st[m_cur]->complete(a_x); }

private:
  MrdHalfDuplexUart *m_bus;
  MrdLineStepper *m_st[2];
  unsigned long m_baud[2];
  int m_cur = 0; // 処理中の種類([0]:1種類目, [1]:2種類目)
};

/// @brief 系統の処理を作る. 2種類目のサーボタイプが設定された系統(mrd_servo_type2)は,
///        種類ごとの処理をまとめた MrdMixedLineStepper を a_mix に生成する.
/// @param a_line 系統(L or R).
/// @param a_type 系統のサーボタイプ(1種類目).
/// @param a_buf 1種類目の処理の生成先.
/// @param a_buf2 2種類目の処理の生成先.
/// @param a_mix まとめた処理の生成先.
/// @param a_meridim Meridimデータの参照.
/// @param a_sv サーボパラメータ.
/// @return 生成した処理. 未対応のタイプならnullptr. 使い終わったら ~MrdLineStepper() を呼ぶこと.
MrdLineStepper *mrd_servo_line_make(UartLine a_line, int a_type, MrdLineStepperBuf &a_buf, MrdLineStepperBuf &a_buf2,
                                    MrdLineStepperBuf &a_mix, Meridim90Union &a_meridim, ServoParam &a_sv) {
  static_assert(sizeof(MrdMixedLineStepper) <= sizeof(MrdLineStepperBuf), "MrdLineStepperBuf too small");
  static_assert(alignof(MrdMixedLineStepper) <= 8, "MrdLineStepperBuf is aligned to 8 bytes");
  const int type2_tmp = (a_line == L || a_line == R) ? mrd_servo_type2[a_line] : 0;
  MrdLineStepper *first_tmp = mrd_servo_stepper_make(a_line, a_type, a_buf, a_meridim, a_sv);
  if (type2_tmp == NOSERVO || first_tmp == nullptr) {
    return first_tmp;
  }
  MrdLineStepper *second_tmp = mrd_servo_stepper_make(a_line, type2_tmp, a_buf2, a_meridim, a_sv);
  if (second_tmp == nullptr) {
    first_tmp->~MrdLineStepper();
    return nullptr;
  }
  return new (a_mix.raw) MrdMixedLineStepper(mrd_servo_mix_bus(a_line, a_type), first_tmp, mrd_servo_mix_baud(a_line, a_type), second_tmp,
                                             mrd_servo_mix_baud(a_line, type2_tmp), (mrd_servo_frame % 2) == 0);
}

//==================================================================================================
//  Servo 関連の処理
//==================================================================================================

//------------------------------------------------------------------------------------
//  各UARTの開始
//------------------------------------------------------------------------------------

/// @brief 1系統のサーボの通信を, サーボタイプのプロトコルで開始する(接続の確認と初期設定を含む).
/// @param a_line 系統(L: Serial1, R: Serial2).
/// @param a_servo_type サーボタイプ.
/// @param a_sv サーボパラメータ.
/// @return 開始できればtrue. PWM(スロット単位で使う)と未対応のタイプはfalse.
bool mrd_servo_begin(UartLine a_line, int a_servo_type, ServoParam &a_sv) {
  if (a_line != L && a_line != R) {
    return false;
  }
  const char *name_tmp = (a_line == L) ? "L" : "R";
  const int *id_tmp = (a_line == L) ? a_sv.ixl_id : a_sv.ixr_id;
  const int *mount_tmp = mrd_servo_line_mount(a_line, a_servo_type, a_sv); // 混在する系統はその種類のスロットだけ
  const int num_tmp = a_sv.num_max;
  switch (a_servo_type) {
  case FTBRSX:
    return mrd_ftb_line(a_line).begin(name_tmp, mount_tmp, id_tmp, num_tmp);
  case DXL1:
    return mrd_dxl1_line(a_line).begin(name_tmp, mount_tmp, id_tmp, num_tmp);
  case DXL2:
    return mrd_dxl2_line(a_line).begin(name_tmp, mount_tmp, id_tmp, num_tmp);
  case KOICS3:
    mrd_ics_line(a_line).begin();
    return true;
  case KOPMX: // RS-485変換を介して接続する
    return mrd_pmx_line(a_line).begin(name_tmp, mount_tmp, id_tmp, num_tmp);
  case JRXBUS:
    return mrd_xbus_line(a_line).begin(name_tmp, mount_tmp, id_tmp, num_tmp);
  case HTCHSR:
    return mrd_htc_line(a_line).begin(name_tmp, mount_tmp, id_tmp, num_tmp);
  case FTCSTS:
  case FTCSCS: // 系列はスロットごとのタイプ(IXL_MT/IXR_MT)に従う. 混在時は現在位置を交互に読む
    return mrd_ftc_line(a_line).begin(name_tmp, mount_tmp, (a_line == L) ? a_sv.ixl_type : a_sv.ixr_type, id_tmp,
                                      num_tmp, a_servo_type);
  default: // PWM(1/11)はUARTの系統ではなくスロット単位で使う(sv_pwm.h). 未対応のタイプも含む
    return false;
  }
}

/// @brief 1系統分のサーボの通信を初期設定する. 2種類目のサーボタイプがあれば混在を検査し, 種類ごとに初期設定する
///        (最後に初期設定した種類の通信速度になるが, 駆動時は種類ごとに切り替える).
/// @param a_line 系統(L or R).
/// @param a_servo_type 系統のサーボタイプ(1種類目).
/// @param a_servo_type2 2種類目のサーボタイプ(0:混在なし).
/// @param a_sv サーボパラメータ.
/// @return 初期設定に成功した場合はtrue.
bool mrd_servo_line_begin(UartLine a_line, int a_servo_type, int a_servo_type2, ServoParam &a_sv) {
  const bool mix_tmp = mrd_servo_mix_check(a_line, a_servo_type, a_servo_type2, a_sv);
  const bool ok_tmp = mrd_servo_begin(a_line, a_servo_type, a_sv);
  return mix_tmp ? (mrd_servo_begin(a_line, a_servo_type2, a_sv) && ok_tmp) : ok_tmp;
}

//------------------------------------------------------------------------------------
//  サーボ通信フォーメーションの分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたサーボにコマンドを分配する.
/// @param a_meridim サーボの動作パラメータを含むMeridim配列.
/// @param a_l_type L系統のサーボタイプ.
/// @param a_r_type R系統のサーボタイプ.
/// @param a_sv サーボパラメータの構造体.
/// @return サーボの駆動が成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_servo_drive_lite(Meridim90Union &a_meridim, int a_l_type, int a_r_type, ServoParam &a_sv) {
  mrd_servo_frame++;
  mrd_servo_param_prepare(a_meridim, a_l_type, a_r_type, a_sv); // パラメータ系コマンドの実行待ちを決める
  mrd_pwm_drive(a_meridim, a_sv);                               // PWM出力(本体ピン/PCA9685). UARTのサーボが現在位置で上書きする前の指令値を使う

  // 系統ごとのプロトコル(どの組み合わせでも, 片方がサーボなしでもよい)で1回ずつの送受信を作り,
  // L/R系統で並行して実行する.
  // ステッパーはDYNAMIXELで約700byteあるため, ループタスクのスタックではなく静的領域に置く(メインループからのみ呼ぶ)
  // 1系統に2種類のサーボを混在させる場合は, 種類ごとの処理とそれをまとめる処理の3つを置く
  static MrdLineStepperBuf buf_tmp[2][3];
  MrdLineStepper *const st_tmp[2] = {
      mrd_servo_line_make(L, a_l_type, buf_tmp[0][0], buf_tmp[0][1], buf_tmp[0][2], a_meridim, a_sv),
      mrd_servo_line_make(R, a_r_type, buf_tmp[1][0], buf_tmp[1][1], buf_tmp[1][2], a_meridim, a_sv)};
  const bool ok_tmp = (st_tmp[0] != nullptr || a_l_type == NOSERVO) && (st_tmp[1] != nullptr || a_r_type == NOSERVO) &&
                      (st_tmp[0] != nullptr || st_tmp[1] != nullptr); // 未対応のサーボタイプを含む場合は駆動しない
  for (int l = 0; l < 2; l++) {
    if (mrd_servo_skip_reads_frame && st_tmp[l] != nullptr) { // マスターコマンド2: 位置を読まない(前回の値)
      st_tmp[l]->skip_reads();
    }
  }
  mrd_servo_skip_reads_frame = false;
  if (ok_tmp) {
    mrd_servo_lines_run(st_tmp, 2);
  }
  for (int l = 0; l < 2; l++) {
    if (st_tmp[l] != nullptr) {
      st_tmp[l]->~MrdLineStepper();
    }
  }
  return ok_tmp;
}

/// @brief 1スロット分の返信値(Meridimのコマンドと角度)を送信配列に格納する.
///        PWM専用スロット(種類1/11)は指令値, マウントしたサーボは現在位置(戻り値のないサーボは指令値)を返す.
///        マウントしていないスロットは, SERVO_UNMOUNT_REPLY が0ならコマンド・角度とも0にし(tgtも0),
///        1なら受信した値のまま書き換えない.
/// @param a_meridim 送信用のMeridim配列(受信データのコピー).
/// @param a_origidx 系統の先頭インデックス(MRD_L_ORIGIDX or MRD_R_ORIGIDX).
/// @param a_ix スロット番号.
/// @param a_type スロットの種類(IXL_MT/IXR_MT. 範囲外は0).
/// @param a_mounted UARTの系統でサーボを駆動したスロットか.
/// @param a_tgt スロットの現在位置または指令値(degree). 範囲外はnullptr.
/// @param a_prm スロットのパラメータ系コマンド・車輪・スルーの状態(nullptrならなし).
///        パラメータ系コマンドと車輪はその結果(車輪は現在速度)をVALに, 結果がエラー値ならコマンドのbit14を立てて返す.
///        スルーは受信した値をそのまま返す.
void mrd_servo_slot_reply(Meridim90Union &a_meridim, int a_origidx, int a_ix, int a_type, bool a_mounted,
                          float *a_tgt, const MrdServoParamSlot *a_prm = nullptr) {
  short &cmd_tmp = a_meridim.sval[a_origidx + a_ix * 2];     // このスロットのコマンド
  short &val_tmp = a_meridim.sval[a_origidx + 1 + a_ix * 2]; // このスロットの値
  const bool used_tmp = a_tgt != nullptr && (a_mounted || mrd_servo_is_pwm_type(a_type));
  const uint8_t mode_tmp = (a_prm != nullptr) ? a_prm->mode : uint8_t(MRD_SLOT_NORMAL);
  if (used_tmp && mode_tmp == MRD_SLOT_THROUGH) {
    return; // スルー: 受信した値をそのまま返す
  }
  if (used_tmp && (mode_tmp == MRD_SLOT_PARAM || mode_tmp == MRD_SLOT_WHEEL)) {
    // パラメータ系コマンド・車輪: 結果を返す. 結果がエラー値ならbit14を立てる
    cmd_tmp = mrd_servo_param_error(a_prm->result) ? short(cmd_tmp | MRD_PCMD_ERR_FLAG) : short(cmd_tmp & ~MRD_PCMD_ERR_FLAG);
    val_tmp = a_prm->result;
    return;
  }
  if (used_tmp) {
    val_tmp = mrd.float2HfShort(*a_tgt); // 最新のサーボ角度(degree)
    return;
  }
  // マウントしていないスロット
  if (cfg.servo_unmount_reply) {
    return; // 受信した値をそのまま返す
  }
  if (a_tgt != nullptr) {
    *a_tgt = 0;
  }
  cmd_tmp = 0;
  val_tmp = 0;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief 第一引数のMeridim配列のすべてのサーボモーターをオフ(フリー状態)に設定する.
/// @param a_meridim サーボの動作パラメータを含むMeridim配列.
/// @param a_disp "All servos torque off." を表示するか.
/// @return 設定完了時にtrueを返す.
bool mrd_servo_all_off(Meridim90Union &a_meridim, bool a_disp = true) {
  for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
    a_meridim.sval[MRD_L_ORIGIDX + i * 2] = 0; // サーボのコマンドをオフに設定
    a_meridim.sval[MRD_R_ORIGIDX + i * 2] = 0; //
  }
  if (a_disp) {
    Serial.println("All servos torque off.");
  }
  return true;
}

/// @brief サーボパラメータからエラーのあるサーボのインデックス番号を作る.
/// @param a_sv サーボパラメータの構造体.
/// @return uint8_tで番号を返す.
///         100-114(L系統 0-14), 200-214(R系統 0-14). 該当が複数あれば最後のもの(R系統優先). なければ0
uint8_t mrd_servo_errcode_make_lite(const ServoParam &a_sv) {
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
