#ifndef __MERIDIAN_MESSAGE_H__
#define __MERIDIAN_MESSAGE_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"
#include "main.h"
#include "mrd_util.h"

// ライブラリ導入
#include <WiFi.h>

//==================================================================================================
//  シリアルモニタ表示用の関数
//==================================================================================================

class MrdMsgHandler {
private:
  Stream &m_serial; // シリアルオブジェクトの参照を保持

public:
  // コンストラクタでStreamオブジェクトを受け取り, メンバーに保存
  MrdMsgHandler(Stream &a_serial) : m_serial(a_serial) {}

  //------------------------------------------------------------------------------------
  //  起動時メッセージ
  //------------------------------------------------------------------------------------

  /// @brief 指定されたミリ秒数だけキャパシタの充電プロセスを示すメッセージを表示する.
  /// @param a_mill 充電プロセスの期間をミリ秒単位で指定.
  void charging(int a_mill) {
    m_serial.print("Charging the capacitor.");
    for (int i = 0; i < a_mill; i++) {
      if (i % 100 == 0) { // 100msごとにピリオドを表示
        m_serial.print(".");
      }
      delay(1);
    }
    m_serial.println();
  }

  /// @brief システムのバージョン情報と通信速度の設定を表示するためのメッセージを出力する.
  /// @param a_version バージョン情報.
  /// @param a_pc_speed PCとのUSBシリアル通信速度.
  /// @param a_spi_speed SPIの通信速度.
  /// @param a_i2c0_speed I2C0の通信速度.
  void hello_lite_esp(String a_version, int a_pc_speed, int a_spi_speed, int a_i2c0_speed) {
    m_serial.println();
    m_serial.print("Hi, This is ");
    m_serial.println(a_version);
    m_serial.print("Set PC-USB ");
    m_serial.print(a_pc_speed);
    m_serial.println(" bps");
    m_serial.print("Set SPI0   ");
    m_serial.print(a_spi_speed);
    m_serial.println(" bps");
    m_serial.print("Set i2c0   ");
    m_serial.print(a_i2c0_speed);
    m_serial.println(" bps");
  }

  /// @brief マウント設定したサーボのbpsをシリアルモニタに出力する.
  /// @param a_servo_l L系統のサーボbps.
  /// @param a_servo_r R系統のサーボbps.
  void servo_bps_2lines(int a_servo_l, int a_servo_r) {
    m_serial.print("Set UART_L ");
    m_serial.print(a_servo_l);
    m_serial.println(" bps");
    m_serial.print("Set UART_R ");
    m_serial.print(a_servo_r);
    m_serial.println(" bps");
  }

  /// @brief 指定されたUARTラインの送信エコー処理の有無を表示する.
  /// @param a_line UART通信ライン(L, R, またはC).
  /// @param a_echo エコー処理を行う場合はtrue.
  void servo_echo(UartLine a_line, bool a_echo) {
    m_serial.print("Set UART_");
    m_serial.print(mrd_line_name_get(a_line));
    m_serial.print(" echo     : ");
    m_serial.println(a_echo ? "on (SN74LXC1T45)" : "off (3-state buffer)");
  }

  /// @brief 指定されたUARTラインのサーボIDを表示する.
  /// @param a_label UARTラインのラベル.
  /// @param a_max サーボの最大数.
  /// @param a_mount サーボのマウント状態を示す配列.
  /// @param a_id サーボIDの配列.
  void print_servo_ids(const char *a_label, int a_max, const int *a_mount, const int *a_id) {
    m_serial.print(a_label);
    for (int i = 0; i < a_max; i++) {
      if (a_mount[i] != 0) {
        if (a_id[i] < 10) {
          m_serial.print(" ");
        }
        m_serial.print(a_id[i]);
      } else {
        m_serial.print("__");
      }
      m_serial.print(" ");
    }
    m_serial.println();
  }

  /// @brief マウントされているサーボのIDを表示する.
  /// @param a_sv サーボパラメータの構造体.
  void servo_mounts_2lines(const ServoParam &a_sv) {
    print_servo_ids("UART_L Servos mounted: ", a_sv.num_max, a_sv.ixl_mount, a_sv.ixl_id);
    print_servo_ids("UART_R Servos mounted: ", a_sv.num_max, a_sv.ixr_mount, a_sv.ixr_id);
  }

  /// @brief wifiの接続開始メッセージを出力する.
  /// @param a_ssid 接続先のSSID.
  void esp_wifi(const char *a_ssid) {
    m_serial.println("WiFi connecting to => " + String(a_ssid)); // WiFi接続完了通知
  }

  /// @brief 指定されたUARTラインとサーボタイプに基づいてサーボの通信プロトコルを表示する.
  /// @param a_line UART通信ライン(L, R, またはC).
  /// @param a_servo_type サーボのタイプを示す整数値.
  /// @return サーボがサポートされている場合はtrueを, サポートされていない場合はfalseを返す.
  bool servo_protocol(UartLine a_line, int a_servo_type) {
    if (a_servo_type > 0) {
      m_serial.print("Set UART_");
      m_serial.print(mrd_line_name_get(a_line));
      m_serial.print(" protocol : ");

      switch (a_servo_type) {
      case PWM_S:
        m_serial.print("single PWM");
        m_serial.println(" - used per slot (IXL_MT/IXR_MT), not as a line type.");
        break;
      case PCA9685:
        m_serial.print("I2C_PCA9685 to PWM");
        m_serial.println(" - used per slot (IXL_MT/IXR_MT), not as a line type.");
        break;
      case FTBRSX:
        m_serial.println("RS30x TTL(FUTABA)");
        break;
      case DXL1:
        m_serial.println("DYNAMIXEL Protocol 1.0(AX/RX/MX)");
        break;
      case DXL2:
        m_serial.println("DYNAMIXEL Protocol 2.0(X series)");
        break;
      case KOICS3:
        m_serial.println("ICS3.5/3.6(KONDO,KRS)");
        break;
      case KOPMX:
        m_serial.println("PMX(KONDO)");
        break;
      case JRXBUS:
        m_serial.println("XBUS(JR PROPO)");
        break;
      case FTCSTS:
      case FTCSCS:
        m_serial.println("STS/SCS(FEETECH)");
        break;
      case HTCHSR:
        m_serial.println("HSR(Hitec)");
        break;
      default:
        m_serial.println(" Not defined. ");
        break;
      }
      return true;
    }
    return false;
  }

  /// @brief wifiの接続完了メッセージと各IPアドレスを出力する.
  /// @param a_flg_fixed_ip 固定IPかどうか. true:固定IP, false:動的IP.
  /// @param a_send_ip 送信先PCのIPアドレス.
  /// @param a_fixedip 固定IPの場合の値.
  void esp_ip(bool a_flg_fixed_ip, const char *a_send_ip, const char *a_fixedip) {
    m_serial.println("WiFi successfully connected.");                   // WiFi接続完了通知
    m_serial.println("PC's IP address target => " + String(a_send_ip)); // 送信先PCのIPアドレスの表示

    if (a_flg_fixed_ip) {
      m_serial.println("ESP32's IP address => " + String(a_fixedip) + " (*Fixed)"); // ESP32自身のIPアドレスの表示
    } else {
      m_serial.print("ESP32's IP address => "); // ESP32自身のIPアドレスの表示
      m_serial.println(WiFi.localIP().toString());
    }
  }

  /// @brief マウント設定したジョイパッドのタイプをシリアルモニタに出力する.
  /// @param a_mount_pad パッドの定義(PC,MERIMOTE,BLUERETRO,SBDBT,KRR5FH,WIIMOTE,WIIMOTE_C)
  void mounted_pad(int a_mount_pad) {
    m_serial.print("Pad Receiver mounted : ");
    switch (a_mount_pad) {
    case WIIMOTE:
      m_serial.println("Wiimote.");
      break;
    case WIIMOTE_C:
      m_serial.println("Wiimote + Classic Controller.");
      break;
    case MERIMOTE:
      m_serial.println("Merimote.");
      break;
    case BLUERETRO:
      m_serial.println("BlueRetro.");
      break;
    case SBDBT:
      m_serial.println("SBDBT.");
      break;
    case KRR5FH:
      m_serial.println("KRC-5FH.");
      break;
    default:
      m_serial.println("None (PC).");
      break;
    }
  }

  /// @brief システムの動作開始を示すメッセージを出力する.
  void flow_start_lite_esp() {
    m_serial.println();
    m_serial.println("-) Meridian -LITE- system on ESP32 now flows. (-");
  }

  //------------------------------------------------------------------------------------
  //  イベントメッセージ
  //------------------------------------------------------------------------------------

  /// @brief システム内の様々な通信エラーとスキップ数をモニタリングし, シリアルポートに出力する.
  /// @param a_disp モニタリング表示のオンオフ.
  /// @param a_err エラーデータの入った構造体.
  /// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
  bool all_err(bool a_disp, const MrdErr &a_err) {
    if (a_disp) {
      m_serial.print("[ERR] es>pc:");
      m_serial.print(a_err.esp_pc);
      m_serial.print(" pc>es:");
      m_serial.print(a_err.pc_esp);
      m_serial.print(" es>ts:");
      m_serial.print(a_err.esp_tsy);
      m_serial.print(" ts>es:");
      m_serial.print(a_err.tsy_esp);
      m_serial.print(" tsSkp:");
      m_serial.print(a_err.tsy_skip);
      m_serial.print(" esSkp:");
      m_serial.print(a_err.esp_skip);
      m_serial.print(" pcSkp:");
      m_serial.print(a_err.pc_skip);
      m_serial.println();
      return true;
    }
    return false;
  }

  /// @brief デバッグ用のフロー表示. 表示OFF時は何もしない(文字列オブジェクトを生成しない).
  /// @param a_text 表示する文字列.
  /// @param a_disp 表示するかどうかのブール値.
  void flow(const char *a_text, bool a_disp) {
    if (a_disp) {
      m_serial.print(a_text);
    }
  }

  /// @brief フレームの処理時間を集計し, a_frames フレームごとに平均と最大を表示する.
  ///        例) Frame[us] busy 4321/6789 servo 2100/2300 late 0/100 rxL 150/300 rxR 160/310
  ///        (平均/最大). rxL/rxRは系統ごとの応答待ち時間(1回の送受信あたり, hduart_stat).
  ///        系統ごとの応答の速さを表す.
  ///        通信の失敗があった場合は2行目に内訳を表示する.
  ///        例) Fail L: echo 0/0 reply 0/3 | R: echo 0/0 reply 1/0 | slot L01:2 R03:1
  ///        echo 不足/不一致, reply 応答なし/途中まで. slotはサーボごとの失敗フレーム数.
  /// @param a_mon 集計用の構造体.
  /// @param a_busy_us 今回のフレーム処理時間(us).
  /// @param a_servo_us 今回のサーボ通信時間(us).
  /// @param a_late 今回の処理が1フレームの時間を超えたか.
  /// @param a_disp 表示するかどうかのブール値.
  /// @param a_sv サーボパラメータの構造体(通信に失敗しているサーボの表示用).
  /// @param a_frames 表示間隔(フレーム数).
  /// @return 表示した場合はtrueを返す.
  bool frame_time(MrdFrameMon &a_mon, unsigned long a_busy_us, unsigned long a_servo_us, bool a_late, bool a_disp,
                  const ServoParam &a_sv, int a_frames = 100) {
    if (!a_disp) {
      return false;
    }
    a_mon.busy_sum += a_busy_us;
    a_mon.busy_max = max(a_mon.busy_max, a_busy_us);
    a_mon.servo_sum += a_servo_us;
    a_mon.servo_max = max(a_mon.servo_max, a_servo_us);
    a_mon.late += a_late ? 1 : 0;
    for (int i = 0; i < a_sv.num_max && i < MRD_SERVO_SLOTS; i++) { // 毎フレーム全サーボと通信するため, err>0は今回失敗
      a_mon.slot_fail[0][i] += (a_sv.ixl_mount[i] && a_sv.ixl_err[i] > 0) ? 1 : 0;
      a_mon.slot_fail[1][i] += (a_sv.ixr_mount[i] && a_sv.ixr_err[i] > 0) ? 1 : 0;
    }
    if (++a_mon.count < a_frames) {
      return false;
    }
    m_serial.print("Frame[us] busy ");
    m_serial.print(a_mon.busy_sum / a_mon.count);
    m_serial.print("/");
    m_serial.print(a_mon.busy_max);
    m_serial.print(" servo ");
    m_serial.print(a_mon.servo_sum / a_mon.count);
    m_serial.print("/");
    m_serial.print(a_mon.servo_max);
    m_serial.print(" late ");
    m_serial.print(a_mon.late);
    m_serial.print("/");
    m_serial.print(a_mon.count);
    for (int l = 0; l < 2; l++) {
      if (hduart_stat.count[l] == 0) {
        continue;
      }
      m_serial.print(l == 0 ? " rxL " : " rxR ");
      m_serial.print(hduart_stat.sum_us[l] / hduart_stat.count[l]);
      m_serial.print("/");
      m_serial.print(hduart_stat.max_us[l]);
    }
    m_serial.println();

    // 通信失敗の内訳
    bool any_tmp = false;
    for (int l = 0; l < 2; l++) {
      for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
        any_tmp |= a_mon.slot_fail[l][i] > 0;
      }
      for (int k = 1; k < 5; k++) {
        any_tmp |= hduart_stat.fail[l][k] > 0;
      }
    }
    if (any_tmp) {
      for (int l = 0; l < 2; l++) {
        m_serial.print(l == 0 ? "Fail L: echo " : " | R: echo ");
        m_serial.print(hduart_stat.fail[l][1]);
        m_serial.print("/");
        m_serial.print(hduart_stat.fail[l][2]);
        m_serial.print(" reply ");
        m_serial.print(hduart_stat.fail[l][3]);
        m_serial.print("/");
        m_serial.print(hduart_stat.fail[l][4]);
      }
      m_serial.print(" | slot");
      for (int l = 0; l < 2; l++) {
        for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
          if (a_mon.slot_fail[l][i] == 0) {
            continue;
          }
          m_serial.print(l == 0 ? " L" : " R");
          if (i < 10) {
            m_serial.print("0");
          }
          m_serial.print(i);
          m_serial.print(":");
          m_serial.print(a_mon.slot_fail[l][i]);
        }
      }
      m_serial.println();
    }
    hduart_stat = MrdHduartStat();
    a_mon = MrdFrameMon();
    return true;
  }

  /// @brief 期待するシーケンス番号と実際に受信したシーケンス番号を表示する.
  /// @param a_seq_expect 期待するシーケンス番号.
  /// @param a_seq_rcvd 実際に受信したシーケンス番号.
  /// @param a_disp 表示するかどうかのブール値.
  /// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
  bool seq_number(uint16_t a_seq_expect, uint16_t a_seq_rcvd, bool a_disp) {
    if (a_disp) {
      m_serial.print("Seq ep/rv ");
      m_serial.print(a_seq_expect);
      m_serial.print("/");
      m_serial.println(a_seq_rcvd);
      return true;
    }
    return false;
  }
};

#endif // __MERIDIAN_MESSAGE_H__
