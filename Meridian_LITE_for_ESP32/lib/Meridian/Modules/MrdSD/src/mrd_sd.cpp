// mrd_sd.cpp
// SDカード関連の関数実装

// ヘッダファイルの読み込み
#include "mrd_sd.h"

// ライブラリ導入
#include <SD.h> // SDカード用

//==================================================================================================
//  SDメモリ関数
//==================================================================================================

//------------------------------------------------------------------------------------
//  初期化
//------------------------------------------------------------------------------------

/// @brief SDカードがマウントされているかとチップセレクトピンの設定に基づいて
///        SDカードの初期化を試みる
/// @param a_sd_mount SDカードがマウントされているかのブール値
/// @param a_sd_chipselect_pin SDカードのチップセレクトピン番号
/// @param a_serial 出力シリアル
/// @return SDカードの初期化が成功した場合はtrue,
///         失敗またはSDカードがマウントされていない場合はfalse
bool mrd_sd_init(bool a_sd_mount, int a_sd_chipselect_pin, HardwareSerial &a_serial) {
  if (a_sd_mount) {
    a_serial.print("Initializing SD card... ");
    if (!SD.begin(a_sd_chipselect_pin)) {
      a_serial.println("Card failed, or not present.");
      return false;
    } else {
      a_serial.println("OK.");
      return true;
    }
  }
  a_serial.println("SD not mounted.");
  return false;
}

//------------------------------------------------------------------------------------
//  読み書きテスト
//------------------------------------------------------------------------------------

/// @brief SDカードの読み書き機能をテストする. SDカードがマウントされており
///        読み書きチェックが要求された場合のみテストを実行する
/// @param a_sd_mount SDカードがマウントされているかのブール値
/// @param a_sd_chipselect_pin SDカードのチップセレクトピン番号
/// @param a_sd_check_rw SDカードの読み書きをチェックするかのブール値
/// @param a_serial 出力シリアル
/// @return SDカードの読み書きが成功した場合はtrue, 失敗した場合はfalse
bool mrd_sd_check(bool a_sd_mount, int a_sd_chipselect_pin, bool a_sd_check_rw, HardwareSerial &a_serial) {
  if (a_sd_mount && a_sd_check_rw) {
    File sd_file; // SDカード用
    sd_file = SD.open("/test.txt", FILE_WRITE);
    delay(1); // SPI安定化用

    if (sd_file) {
      a_serial.print("Checking SD card r/w... ");
      // SD書き込みテスト用のランダム4桁数字を生成
      randomSeed(long(analogRead(A0))); // 未接続ピンのノイズを使用
      int rand_number_tmp = random(1000, 9999);

      a_serial.print("write code ");
      a_serial.print(rand_number_tmp);
      // ファイル書き込みを実行
      sd_file.println(rand_number_tmp);
      delayMicroseconds(1); // SPI安定化用
      sd_file.close();
      delayMicroseconds(10); // SPI安定化用
      // ファイル読み込みを実行
      sd_file = SD.open("/test.txt");
      if (sd_file) {
        a_serial.print(" and read code ");
        while (sd_file.available()) {
          a_serial.write(sd_file.read());
        }
        sd_file.close();
      }
      SD.remove("/test.txt");
      delay(10);
      return true;
    } else {
      a_serial.println("Could not open SD test.txt file.");
      return false;
    }
  } else {
    return false;
  }
}
