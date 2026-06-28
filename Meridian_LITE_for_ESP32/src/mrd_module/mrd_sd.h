#ifndef __MERIDIAN_SD_H__
#define __MERIDIAN_SD_H__

// ヘッダファイルの読み込み
#include "../config.h"

// ライブラリ導入
#include <Arduino.h>
#include <SD.h> // SDカード用

//==================================================================================================
//  関数プロトタイプ宣言
//==================================================================================================

/// @brief SDカードの初期化を試みる. SDカードがマウントされているか,
///        及びチップ選択ピンの設定に基づく.
/// @param a_sd_mount SDカードがマウントされているかどうかのブール値.
/// @param a_sd_chipselect_pin SDカードのチップ選択ピン番号.
/// @return SDカードの初期化が成功した場合はtrueを,
///         失敗またはSDカードがマウントされていない場合はfalseを返す.
bool mrd_sd_init(bool a_sd_mount, int a_sd_chipselect_pin);

/// @brief SDカードの読み書き機能をテストする. SDカードがマウントされ,
/// 読み書きのチェックが要求された場合のみテストを実行する.
/// @param a_sd_mount SDカードがマウントされているかどうかのブール値.
/// @param a_sd_chipselect_pin SDカードのチップ選択ピン番号.
/// @param a_sd_check_rw SDカードの読み書きをチェックするかどうかのブール値.
/// @return SDカードの読み書きが成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_sd_check(bool a_sd_mount, int a_sd_chipselect_pin, bool a_sd_check_rw);

#endif // __MERIDIAN_SD_H__
