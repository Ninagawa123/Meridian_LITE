#ifndef __MERIDIAN_MAIN_FUNC__
#define __MERIDIAN_MAIN_FUNC__

// ヘッダファイルの読み込み
#include "config.h"
#include "mrd_common.h"

// ライブラリ導入
#include <Meridian.h>                   // Meridianのライブラリ導入
#include <IcsHardSerialClass.h>         // ICSサーボのインスタンス設定

// グローバル変数のextern宣言 (実体はmain.cppで定義, 各モジュールから参照)
extern MERIDIANFLOW::Meridian mrd;
extern IcsHardSerialClass ics_L;
extern IcsHardSerialClass ics_R;

//------------------------------------------------------------------------------------
//  列挙型
//------------------------------------------------------------------------------------

// UartLine は mrd_common.h で定義

// ServoType は mrd_common.h で定義

// ImuAhrsType は mrd_common.h で定義

// PadType は mrd_common.h で定義

enum PadButton {  // リモコンボタンの列挙型
  PAD_SELECT = 1, // Select
  PAD_L3 = 2,     // L3
  PAD_R3 = 4,     // R3
  PAD_START = 8,  // Start
  PAD_UP = 16,    // 十字上
  PAD_RIGHT = 32, // 十字右
  PAD_DOWN = 64,  // 十字下
  PAD_LEFT = 128, // 十字左
  PAD_L2 = 256,   // L2
  PAD_R2 = 512,   // R2
  PAD_L1 = 1024,  // L1
  PAD_R1 = 2048,  // R1
  PAD_bU = 4096,  // △ 上
  PAD_bR = 8192,  // o 右
  PAD_bD = 16384, // x 下
  PAD_bL = 32768  // ◻︎ 左
};

enum BinHexDec { // 数値表示タイプの列挙型(Bin, Hex, Dec)
  Bin = 0,       // BIN
  Hex = 1,       // HEX
  Dec = 2,       // DEC
};

//------------------------------------------------------------------------------------
//  変数
//------------------------------------------------------------------------------------

// システム用の変数
// MRDM_BYTE と MRD_CKSM は mrd_common.h で定義
const int MRD_ERR = MRDM_LEN - 2;      // エラーフラグの格納場所(配列の末尾から2つめ)
const int MRD_ERR_u = MRD_ERR * 2 + 1; // エラーフラグの格納場所(上位8ビット)
const int MRD_ERR_l = MRD_ERR * 2;     // エラーフラグの格納場所(下位8ビット)
// PAD_LEN は mrd_common.h で定義
extern TaskHandle_t thp[4];              // マルチスレッドのタスクハンドル格納用
extern SemaphoreHandle_t ahrs_mutex;     // AHRSデータアクセス用mutex (スレッドセーフ用)

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

// Meridim90Union は mrd_common.h で定義
extern Meridim90Union s_udp_meridim;       // Meridim配列データ送信用(short型, センサや角度は100倍値)
extern Meridim90Union r_udp_meridim;       // Meridim配列データ受信用

// MrdFlags は mrd_common.h で定義
extern MrdFlags flg;

// シーケンス番号理用の変数
struct MrdSq {
  int s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
  int r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};
extern MrdSq mrdsq;

// タイマー管理用の変数
struct MrdTimer {
  long frame_ms = FRAME_DURATION; // 1フレームあたりの単位時間(ms)
  int count_loop = 0;             // サイン計算用の循環カウンタ
  int count_loop_dlt = 2;         // サイン計算用の循環カウンタを1フレームにいくつ進めるか
  int count_loop_max = 359999;    // 循環カウンタの最大値
  unsigned long count_frame = 0;  // メインフレームのカウント

  int pad_interval = (PAD_INTERVAL - 1 > 0) ? PAD_INTERVAL - 1 : 1; // パッドの問い合わせ待機時間
};
extern MrdTimer tmr;

// MrdErr は mrd_common.h で定義
extern MrdErr err;

// PadUnion は mrd_common.h で定義
extern PadUnion pad_array; // pad値の格納用配列

// リモコンのアナログ入力データ
struct PadValue {
  unsigned short stick_R = 0;
  int stick_R_x = 0;
  int stick_R_y = 0;
  unsigned short stick_L = 0;
  int stick_L_x = 0;
  int stick_L_y = 0;
  unsigned short stick_L2R2V = 0;
  int R2_val = 0;
  int L2_val = 0;
};
extern PadValue pad_analog;

struct AhrsValue;
extern AhrsValue ahrs;

// ServoParam は mrd_common.h で定義
extern ServoParam sv;

// モニタリング設定
struct MrdMonitor {
  bool flow = MONITOR_FLOW;           // フローを表示
  bool all_err = MONITOR_ERR_ALL;     // 全経路の受信エラー率を表示
  bool servo_err = MONITOR_ERR_SERVO; // サーボエラーを表示
  bool seq_num = MONITOR_SEQ;         // シーケンス番号チェックを表示
  bool pad = MONITOR_PAD;             // リモコンのデータを表示
};
extern MrdMonitor monitor;

// MrdMsgHandler は mrd_disp.h で定義 (main.cpp でインクルード)
class MrdMsgHandler;
extern MrdMsgHandler mrd_disp;

//==================================================================================================
//  関数各種
//==================================================================================================

/// @brief 入力値から予想シーケンス番号を生成する
/// @param a_previous_num 前回のシーケンス番号
/// @return 予想シーケンス番号 (0～59,999)
inline uint16_t mrd_seq_predict_num(uint16_t a_previous_num) {
  uint16_t x_tmp = a_previous_num + 1;
  if (x_tmp > 59999) { // カウンタをリセット
    x_tmp = 0;
  }
  return x_tmp;
}

#endif //__MERIDIAN_MAIN_FUNC__
