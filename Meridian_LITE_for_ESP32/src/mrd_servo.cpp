// mrd_servo.cpp
// サーボ関連の関数実装

// ヘッダファイルの読み込み
#include "mrd_servo.h"

// サーボドライバーの条件付きインクルード
// 使用するドライバーのみコンパイルに含める
#if MOUNT_SERVO_TYPE_L == SERVO_TYPE_KOICS3 || MOUNT_SERVO_TYPE_R == SERVO_TYPE_KOICS3
#include "mrd_module/sv_ics.h"
#endif

#if MOUNT_SERVO_TYPE_L == SERVO_TYPE_FTBRSX || MOUNT_SERVO_TYPE_R == SERVO_TYPE_FTBRSX
#include "mrd_module/sv_ftbrx.h"
#endif

#if MOUNT_SERVO_TYPE_L == SERVO_TYPE_DXL2 || MOUNT_SERVO_TYPE_R == SERVO_TYPE_DXL2
#include "mrd_module/sv_dxl2.h"
#endif

#if MOUNT_SERVO_TYPE_L == SERVO_TYPE_FTCSTS || MOUNT_SERVO_TYPE_R == SERVO_TYPE_FTCSTS || \
    MOUNT_SERVO_TYPE_L == SERVO_TYPE_FTCSCS || MOUNT_SERVO_TYPE_R == SERVO_TYPE_FTCSCS
#include "mrd_module/sv_ftc.h"
#endif

// ライブラリの読み込み

//==================================================================================================
//  サーボ関数
//==================================================================================================

//------------------------------------------------------------------------------------
//  各UARTの開始
//------------------------------------------------------------------------------------

/// @brief 指定されたUARTラインとサーボタイプに基づいてサーボ通信プロトコルを設定する
/// @param a_line UART通信ライン (L, R, または C)
/// @param a_servo_type サーボのタイプを示す整数値
/// @return サーボがサポートされている場合はtrue, サポートされていない場合はfalse
bool mrd_servo_begin(UartLine a_line, ServoType a_servo_type, IcsHardSerialClass &a_ics) {
  switch (a_servo_type) {
  case PWM_S:
    // 単体PWM [WIP]
    return false;
  case PCA9685:
    // I2C_PCA9685 to PWM [WIP]
    return false;
  case FTBRSX:
    // RSxTTL (FUTABA) [WIP]
    return false;
  case DXL1:
    // DYNAMIXEL Protocol 1.0 [WIP]
    return false;
  case DXL2:
    // DYNAMIXEL Protocol 2.0 [WIP]
    return false;
  case KOICS3:
    a_ics.begin();
    return true;
  case KOPMX:
    // PMX(KONDO) [WIP]
    return false;
  case JRXBUS:
    // XBUS(JR PROPO) [WIP]
    return false;
  case FTCSTS:
    // STS(FEETECH) [WIP]
    return false;
  case FTCSCS:
    // SCS(FEETECH) [WIP]
    return false;
  default:
    // 未定義
    return false;
  }
}

//------------------------------------------------------------------------------------
//  サーボ通信形成の分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたサーボへコマンドを配信する
/// @param a_meridim サーボ動作パラメータを含むMeridim配列
/// @param a_L_type L系統のサーボタイプ
/// @param a_R_type R系統のサーボタイプ
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @param a_ics_L L系統のICSサーボ通信クラスのインスタンス
/// @param a_ics_R R系統のICSサーボ通信クラスのインスタンス
/// @param a_mrd Meridianクラスのインスタンス
/// @return サーボ駆動が成功した場合はtrue, 失敗した場合はfalse
bool mrd_servo_drive_lite(Meridim90Union &a_meridim, ServoType a_L_type, ServoType a_R_type,
                          ServoParam &a_sv,
                          IcsHardSerialClass &a_ics_L, IcsHardSerialClass &a_ics_R,
                          MERIDIANFLOW::Meridian &a_mrd) {
#if MOUNT_SERVO_TYPE_L == SERVO_TYPE_KOICS3 && MOUNT_SERVO_TYPE_R == SERVO_TYPE_KOICS3
  // LR両系統がICSサーボの場合, LRバランス送信を実行
  if (a_L_type == SERVO_TYPE_KOICS3 && a_R_type == SERVO_TYPE_KOICS3) {
    mrd_sv_drive_ics_double(a_meridim, a_sv, a_ics_L, a_ics_R, a_mrd);
    return true;
  }
#endif
  // 他のサーボタイプの組み合わせは未実装
  (void)a_meridim;
  (void)a_L_type;
  (void)a_R_type;
  (void)a_sv; // 未使用警告抑制
  return false;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief 第一引数のMeridim配列のすべてのサーボモーターをオフ(フリー状態)に設定する
/// @param a_meridim サーボ動作パラメータを含むMeridim配列
/// @return 完了時にtrueを返す
bool mrd_servo_all_off(Meridim90Union &a_meridim) {
  for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
    a_meridim.sval[i * 2 + MRD_L_ORIGIDX] = 0; // L系統サーボコマンドをオフに設定
    a_meridim.sval[i * 2 + MRD_R_ORIGIDX] = 0; // R系統サーボコマンドをオフに設定
  }
  Serial.println("All servos torque off.");
  return true;
}

/// @brief サーボパラメータからエラーのあるサーボのインデックス番号を作成する
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @return uint8_t の番号
///         100-149 (L系統 0-49), 200-249 (R系統 0-49)
/// @note 最大インデックスのエラーを優先して返す (逆順走査で早期終了)
uint8_t mrd_servo_make_errcode_lite(const ServoParam &a_sv) {
  // R系統を逆順に走査 (高インデックス優先、早期終了)
  for (int i = a_sv.num_max - 1; i >= 0; i--) {
    if (a_sv.ixr_stat[i]) {
      return uint8_t(i + 200); // R系統: 200-214
    }
  }
  // L系統を逆順に走査 (高インデックス優先、早期終了)
  for (int i = a_sv.num_max - 1; i >= 0; i--) {
    if (a_sv.ixl_stat[i]) {
      return uint8_t(i + 100); // L系統: 100-114
    }
  }
  return 0; // エラーなし
}
