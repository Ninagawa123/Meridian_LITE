#ifndef __MERIDIAN_SD_H__
#define __MERIDIAN_SD_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include <SD.h> // SDカード用

//==================================================================================================
//  SDメモリ 関連の処理
//==================================================================================================

//------------------------------------------------------------------------------------
//  初期化処理
//------------------------------------------------------------------------------------

/// @brief SDカードの初期化を試みる. SDカードがマウントされているか,
///        及びチップ選択ピンの設定に基づく.
/// @param a_sd_mount SDカードがマウントされているかどうかのブール値.
/// @param a_sd_chipselect_pin SDカードのチップ選択ピン番号.
/// @return SDカードの初期化が成功した場合はtrueを,
///         失敗またはSDカードがマウントされていない場合はfalseを返す.
bool mrd_sd_init(bool a_sd_mount, int a_sd_chipselect_pin) {
  if (a_sd_mount) {
    Serial.print("Initializing SD card... ");
    // delay(100);
    if (!SD.begin(a_sd_chipselect_pin)) {
      Serial.println("Card failed, or not present.");
      // delay(100);
      return false;
    } else {
      Serial.println("OK.");
      // delay(100);
      return true;
    }
  }
  Serial.println("SD not mounted.");
  // delay(100);
  return false;
}

//------------------------------------------------------------------------------------
//  リードライトテスト
//------------------------------------------------------------------------------------

/// @brief SDカードの読み書き機能をテストする. SDカードがマウントされ,
/// 読み書きのチェックが要求された場合のみテストを実行する.
/// @param a_sd_mount SDカードがマウントされているかどうかのブール値.
/// @param a_sd_chipselect_pin SDカードのチップ選択ピン番号.
/// @param a_sd_check_rw SDカードの読み書きをチェックするかどうかのブール値.
/// @return SDカードの読み書きが成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_sd_check(bool a_sd_mount, int a_sd_chipselect_pin, bool a_sd_check_rw) {
  if (a_sd_mount && a_sd_check_rw) {
    File sd_file; // SDカード用
    sd_file = SD.open("/test.txt", FILE_WRITE);
    delay(1); // SPI安定化検証用

    if (sd_file) {
      Serial.print("Checking SD card r/w... ");
      // SD書き込みテスト用のランダムな4桁の数字を生成
      randomSeed(long(analogRead(A0))); // 未接続ピンのノイズを利用
      int rand_number_tmp = random(1000, 9999);

      Serial.print("write code ");
      Serial.print(rand_number_tmp);
      // ファイルへの書き込みを実行
      sd_file.println(rand_number_tmp);
      delayMicroseconds(1); // SPI安定化検証用
      sd_file.close();
      delayMicroseconds(10); // SPI安定化検証用
      // ファイルからの読み込みを実行
      sd_file = SD.open("/test.txt");
      if (sd_file) {
        Serial.print(" and read code ");
        while (sd_file.available()) {
          Serial.write(sd_file.read());
        }
        sd_file.close();
      }
      SD.remove("/test.txt");
      delay(10);
      return true;
    } else {
      Serial.println("Could not open SD test.txt file.");
      return false;
    }
  } else {
    return false;
  }
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

#endif // __MERIDIAN_SD_H__
