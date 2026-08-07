#ifndef __MERIDIAN_MOVEMENT_H__
#define __MERIDIAN_MOVEMENT_H__

// ヘッダファイルの読み込み
#include "../config.h"
#include "mrd_types.h"
#include "mv_motionplay.h"

//==================================================================================================
//  動作計算関連の処理
//==================================================================================================

// モーション再生クラスのグローバルインスタンス extern宣言
// main.cpp から mp.set_status(), mp.play(), mp.updateServoData() を呼び出す
extern MotionPlayClass mp;

//==================================================================================================
//  関数プロトタイプ宣言
//==================================================================================================

/// @brief モーション再生の初期化. setup() から呼ぶ.
/// @param a_meridim 使用しない (将来の拡張用).
/// @return 常にfalseを返す.
bool mrd_move_init(Meridim90Union a_meridim);

#endif // __MERIDIAN_MOVEMENT_H__
