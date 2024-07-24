#ifndef __MERIDIAN_SERVO_DISTRIBUTOR_H__
#define __MERIDIAN_SERVO_DISTRIBUTOR_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"
#include "mrd_module/sv_ftbrx.h"
#include "mrd_module/sv_ics.h"

//================================================================================================================
//  Servo 関連の処理
//------------------------------------------------------------------------------------------
//================================================================================================================

/// @brief 列挙型(L,R,C)から文字列を取得する関数.
/// @param a_line 列挙型 enum UartLine
/// @return 列挙型の内容に応じて文字列"L","R","C"返す.
const char *get_line_name(UartLine a_line) {
  switch (a_line) {
  case L:
    return "L";
  case R:
    return "R";
  case C:
    return "C";
  default:
    return "Unknown";
  }
}

//------------------------------------------------------------------------------------
//  各UARTの開始
//------------------------------------------------------------------------------------

/// @brief 指定されたUARTラインとサーボタイプに基づいてサーボの通信プロトコルを設定する.
/// @param a_line UART通信ライン（L, R, またはC）.
/// @param a_servo_type サーボのタイプを示す整数値.
/// @return サーボがサポートされている場合はtrueを, サポートされていない場合はfalseを返す.
bool mrd_servos_begin(UartLine a_line, int a_servo_type) {
  if (a_servo_type > 0) {
    Serial.print("Set UART_");
    Serial.print(get_line_name(a_line));
    Serial.print(" protocol : ");

    switch (a_servo_type) {
    case 1:
      Serial.print("single PWM");
      Serial.println(" - Not supported yet.");
      break;
    case 11:
      Serial.print("I2C_PCA9685 to PWM");
      Serial.println(" - Not supported yet.");
      break;
    case 21:
      Serial.print("RSxTTL (FUTABA)");
      Serial.println(" - Not supported yet.");
      break;
    case 31:
      Serial.print("DYNAMIXEL Protocol 1.0");
      Serial.println(" - Not supported yet.");
      break;
    case 32:
      Serial.print("DYNAMIXEL Protocol 2.0");
      Serial.println(" - Not supported yet.");
      break;
    case 43:
      if (a_line == L)
        ics_L.begin(); // サーボモータの通信初期設定. Serial2
      else if (a_line == R)
        ics_R.begin(); // サーボモータの通信初期設定. Serial3
      Serial.println("ICS3.5/3.6(KONDO,KRS)");
      break;
    case 44:
      Serial.print("PMX(KONDO)");
      Serial.println(" - Not supported yet.");
      break;
    case 51:
      Serial.print("XBUS(JR PROPO)");
      Serial.println(" - Not supported yet.");
      break;
    case 61:
      Serial.print("STS(FEETECH)");
      Serial.println(" - Not supported yet.");
      break;
    case 62:
      Serial.print("SCS(FEETECH)");
      Serial.println(" - Not supported yet.");
      break;
    default:
      Serial.println(" Not defined. ");
      break;
    }
    return true;
  }
  return false;
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
bool mrd_servos_drive_lite(Meridim90Union a_meridim, int a_L_type, int a_R_type, ServoParam &a_sv) {
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

/// @brief 第一引数のMeridim配列のすべてのサーボモーターをオフ（フリー状態）に設定する.
/// @param a_meridim サーボの動作パラメータを含むMeridim配列.
/// @return 設定完了時にtrueを返す.
bool mrd_servos_all_off(Meridim90Union a_meridim) {
  for (int i = 0; i < 15; i++) {
    a_meridim.sval[i * 2 + 20] = 0; // サーボのコマンドをオフに設定
    a_meridim.sval[i * 2 + 50] = 0; //
  }
  Serial.println("All servos torq off.");
  return true;
}

#endif // __MERIDIAN_SERVO_DISTRIBUTOR_H__
