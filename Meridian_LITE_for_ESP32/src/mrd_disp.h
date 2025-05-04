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
  /// @param a_mill 充電プロセスの期間を秒単位で指定.
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

  /// @brief 指定されたUARTラインのサーボIDを表示する.
  /// @param a_label UARTラインのラベル.
  /// @param a_max サーボの最大数.
  /// @param a_mount サーボのマウント状態を示す配列.
  /// @param a_id サーボIDの配列.
  void print_servo_ids(const char *a_label, int a_max, int *a_mount, const int *a_id) {
    m_serial.print(a_label);
    for (int i = 0; i <= a_max; i++) {
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
  void servo_mounts_2lines(ServoParam a_sv) {
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
      m_serial.print(mrd_get_line_name(a_line));
      m_serial.print(" protocol : ");

      switch (a_servo_type) {
      case 1:
        m_serial.print("single PWM");
        m_serial.println(" - Not supported yet.");
        break;
      case 11:
        m_serial.print("I2C_PCA9685 to PWM");
        m_serial.println(" - Not supported yet.");
        break;
      case 21:
        m_serial.print("RSxTTL (FUTABA)");
        m_serial.println(" - Not supported yet.");
        break;
      case 31:
        m_serial.print("DYNAMIXEL Protocol 1.0");
        m_serial.println(" - Not supported yet.");
        break;
      case 32:
        m_serial.print("DYNAMIXEL Protocol 2.0");
        m_serial.println(" - Not supported yet.");
        break;
      case 43:
        m_serial.println("ICS3.5/3.6(KONDO,KRS)");
        break;
      case 44:
        m_serial.print("PMX(KONDO)");
        m_serial.println(" - Not supported yet.");
        break;
      case 51:
        m_serial.print("XBUS(JR PROPO)");
        m_serial.println(" - Not supported yet.");
        break;
      case 61:
        m_serial.print("STS(FEETECH)");
        m_serial.println(" - Not supported yet.");
        break;
      case 62:
        m_serial.print("SCS(FEETECH)");
        m_serial.println(" - Not supported yet.");
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
  /// @param a_ssid 接続先のSSID.
  /// @param a_fixedip 固定IPの場合の値.
  void esp_ip(bool a_flg_fixed_ip, const char *a_ssid, const char *a_fixedip) {
    m_serial.println("WiFi successfully connected."); // WiFi接続完了通知
    m_serial.println("PC's IP address target => " +
                     String(WIFI_SEND_IP)); // 送信先PCのIPアドレスの表示

    if (a_flg_fixed_ip) {
      m_serial.println("ESP32's IP address => " + String(FIXED_IP_ADDR) +
                       " (*Fixed)"); // ESP32自身のIPアドレスの表示
    } else {
      m_serial.print("ESP32's IP address => "); // ESP32自身のIPアドレスの表示
      m_serial.println(WiFi.localIP().toString());
    }
  }

  /// @brief マウント設定したジョイパッドのタイプをシリアルモニタに出力する.
  /// @param a_mount_pad パッドの定義(PC,MERIMOTE,BLUERETRO,SBDBT,KRR5FH,WIIMOTE)
  void mounted_pad(int a_mount_pad) {
    m_serial.print("Pad Receiver mounted : ");
    switch (a_mount_pad) {
    case WIIMOTE:
      m_serial.println("Wiimote.");
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
  /// @param mrd_disp_all_err モニタリング表示のオンオフ.
  /// @param a_err エラーデータの入った構造体.
  /// @return エラーメッセージを表示した場合はtrueを, 表示しなかった場合はfalseを返す.
  bool all_err(bool mrd_disp_all_err, MrdErr a_err) {
    if (mrd_disp_all_err) {
      m_serial.print("[ERR] es>pc:");
      m_serial.print(a_err.esp_pc);
      m_serial.print(" pc>es:");
      m_serial.print(a_err.pc_esp);
      m_serial.print(" es>ts:");
      m_serial.print(a_err.esp_tsy);
      m_serial.print(" ts>es:");
      m_serial.print(a_err.esp_tsy);
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

  /// @brief サーボモーターのエラーを検出した場合にエラーメッセージを表示する.
  /// @param a_line サーボモーターが接続されているUARTライン(L, R, C).
  /// @param a_num エラーが発生しているサーボの番号.
  /// @param a_flg_disp エラーメッセージを表示するかどうかのブール値.
  /// @return エラーメッセージが表示された場合はtrueを, 表示されなかった場合はfalseを返す.
  bool servo_err(UartLine a_line, int a_num, bool a_flg_disp) {
    if (a_flg_disp) {
      m_serial.print("Found servo err ");
      if (a_line == L) {
        m_serial.print("L_");
        m_serial.println(a_num);
        return true;
      } else if (a_line == R) {
        m_serial.print("R_");
        m_serial.println(a_num);
        return true;
      } else if (a_line == C) {
        m_serial.print("C_");
        m_serial.println(a_num);
        return true;
      }
    }
    return false;
  }

  /// @brief 期待するシーケンス番号と実施に受信したシーケンス番号を表示する.
  /// @param a_seq_expect 期待するシーケンス番号.
  /// @param a_seq_rcvd 実際に受信したシーケンス番号.
  /// @param a_disp_seq_num 表示するかどうかのブール値.
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
