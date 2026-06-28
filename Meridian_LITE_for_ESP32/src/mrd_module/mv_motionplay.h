#ifndef __MERIDIAN_MOVEMENT_MOTIONPLAY_H__
#define __MERIDIAN_MOVEMENT_MOTIONPLAY_H__

#include "../config.h"
#include "mrd_types.h"
#include <cmath>

//==================================================================================================
//  モーション再生関連の処理 (atm_motionplay.h より移植 +Hori)
//  - SD/EEPROM処理を除外
//  - ジャイロミキシングを除外 (ESP32 LITE版)
//==================================================================================================

// キーフレームヘッダーのインデックス
#define HD_NEXT_INTERVAL 0 // 時間補間 ms
#define HD_NEXT_CMD      1 // コマンド
#define HD_NEXT_PRM      2 // パラメータ
#define HD_NEXT_POS      3 // 分岐先1
#define HD_NEXT_POS_NOT  4 // 分岐先2

// HD_NEXT_CMD の値
#define HD_NEXT_CMD_MOVE     1   // MOVE
#define HD_NEXT_CMD_SET_REG  2   // SET_REGISTER
#define HD_NEXT_CMD_BRNC_BTN 3   // BRANCH BTN
#define HD_NEXT_CMD_BRNC_BIT 4   // BRANCH BIT
#define HD_NEXT_CMD_JUMP     5   // JUMP
#define HD_NEXT_CMD_GOSUB    6   // GOSUB
#define HD_NEXT_CMD_SET_LP   7   // SET_LOOP
#define HD_NEXT_CMD_BRNC_LP  8   // BRANCH LOOP
#define HD_NEXT_CMD_BRNC_PA1 9   // BRANCH ANALOG1 (stick_L_x)
#define HD_NEXT_CMD_BRNC_PA2 10  // BRANCH ANALOG2 (stick_L_y)
#define HD_NEXT_CMD_BRNC_PA3 11  // BRANCH ANALOG3 (stick_R_x)
#define HD_NEXT_CMD_BRNC_PA4 12  // BRANCH ANALOG4 (stick_R_y)
#define HD_NEXT_CMD_END      255 // END

// サーボコマンド値 (Meridim プロトコル)
#define MP_CMD_READ    2 // サーボ読み取りモード (現在値を保持)
#define MP_CMD_STRETCH 5 // ストレッチ設定
#define MP_CMD_MIX1    6 // ジョイスティックL X軸ミキシング設定
#define MP_CMD_MIX2    7 // ジョイスティックL Y軸ミキシング設定
#define MP_CMD_MIX3    8 // ジョイスティックR X軸ミキシング設定
#define MP_CMD_MIX4    9 // ジョイスティックR Y軸ミキシング設定

// ミキシングテーブルの行インデックス (ジョイスティックのみ, gyro行は未使用)
#define MP_MIX_JOY_X_L 3 // stick_L_x
#define MP_MIX_JOY_Y_L 4 // stick_L_y
#define MP_MIX_JOY_X_R 6 // stick_R_x
#define MP_MIX_JOY_Y_R 7 // stick_R_y

// モーション定数
#define MP_MOT_MAX    5  // 最大モーション数 (ESP32 LITE版省メモリ: 必要なら増やす)
#define MP_DATA_SIZE  40 // 1モーション内の最大キーフレーム数
#define MP_JOINT_SIZE 15 // 1系統のサーボ数

// モーション再生ログ出力フラグ
#define MONITOR_MOT_PLAY 1 // キーフレーム遷移をシリアル出力 (0:OFF, 1:ON)

class MotionPlayClass {

public:
  // ジョイスティックアナログフィードバック用ミキシング係数テーブル
  // 行: 0-2=gyro(未使用/0固定), 3=joy_L_x, 4=joy_L_y, 5=unused, 6=joy_R_x, 7=joy_R_y, 8=unused
  // 列: 各サーボ軸のゲイン (例: 100 -> 0.01*100*pad = 1.0deg の混合量)
  short mv_mix_l[9][MP_JOINT_SIZE] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 0 (gyro_roll 未使用)
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 1 (gyro_pitch 未使用)
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 2 (gyro_yaw 未使用)
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 3 joy_L_x
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 4 joy_L_y
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 5 (未使用)
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 6 joy_R_x
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 7 joy_R_y
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}; // 8 (未使用)

  short mv_mix_r[9][MP_JOINT_SIZE] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 0 (gyro_roll 未使用)
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 1 (gyro_pitch 未使用)
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 2 (gyro_yaw 未使用)
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 3 joy_L_x
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 4 joy_L_y
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 5 (未使用)
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 6 joy_R_x
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // 7 joy_R_y
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}; // 8 (未使用)

  float joint_l[MP_JOINT_SIZE] = {};     // L系統の出力角度 (degree)
  float joint_r[MP_JOINT_SIZE] = {};     // R系統の出力角度 (degree)
  int joint_cmd_l[MP_JOINT_SIZE] = {};   // L系統の出力コマンド
  int joint_cmd_r[MP_JOINT_SIZE] = {};   // R系統の出力コマンド
  float mix_joint_l[MP_JOINT_SIZE] = {}; // L系統のミキシング量
  float mix_joint_r[MP_JOINT_SIZE] = {}; // R系統のミキシング量

  bool mix_enable = true; // ジョイスティックアナログミキシング ON/OFF
  short mp_no = 0;        // 再生中のモーション番号
  short mv_pos = 0;       // 現在のキーフレーム番号
  bool playing = false;   // モーション再生中フラグ
  float play_g = 1.0f;    // 再生速度ゲイン

  // モーションデータクラス
  class MotionClass {
  public:
    uint16_t trigger;                         // 再生トリガーとなるパッドボタン値
    short header[MP_DATA_SIZE][5];            // キーフレームヘッダ [interval, cmd, prm, next, next_not]
    short dst_l[MP_DATA_SIZE][MP_JOINT_SIZE]; // L系統の目標角度 (hundredths of deg)
    short dst_r[MP_DATA_SIZE][MP_JOINT_SIZE]; // R系統の目標角度 (hundredths of deg)
    short cmd_l[MP_DATA_SIZE][MP_JOINT_SIZE]; // L系統のサーボコマンド
    short cmd_r[MP_DATA_SIZE][MP_JOINT_SIZE]; // R系統のサーボコマンド
    short loop_cnt;                           // ループカウンタ
  };

  MotionClass mot[MP_MOT_MAX]; // モーションデータ配列

  /// @brief モーション再生の初期化
  void init();

  /// @brief パッドボタン値からモーション番号を選択
  /// @param pad_key パッドボタン値
  void set_status(uint16_t pad_key);

  /// @brief モーション再生メイン処理
  /// @param pad_key パッドボタン値 (pad_array.usval[0])
  /// @param pad_analog パッドアナログ値へのポインタ
  /// @param s_arr Meridim送信配列 (uint16_t, 現在のサーボ値読み取りにも使用)
  /// @param sv サーボパラメータへのポインタ
  void play(uint16_t pad_key, PadValue *pad_analog, uint16_t s_arr[], ServoParam *sv);

  /// @brief Meridim配列とサーボ構造体に計算結果を書き込む
  /// @param m_arr Meridim送信配列 (uint16_t)
  /// @param sv サーボパラメータへのポインタ
  void updateServoData(uint16_t m_arr[], ServoParam *sv);

private:
  short mv_last_pos = 0;

  const float FRAME_RATE_10MS = FRAME_DURATION * 0.001f; // 1フレームの秒数 (10ms = 0.01s)

  float mv_tmr = 0.0f;           // 現在キーフレーム内の経過時間 (s)
  float mv_next_interval = 0.0f; // 現在キーフレームの目標時間 (s)

  float mv_last_l[MP_JOINT_SIZE] = {}; // キーフレーム開始時のL系統角度
  float mv_last_r[MP_JOINT_SIZE] = {}; // キーフレーム開始時のR系統角度
  float mv_act_l[MP_JOINT_SIZE] = {};  // キーフレーム目標L系統角度
  float mv_act_r[MP_JOINT_SIZE] = {};  // キーフレーム目標R系統角度

  float mix_pad_g = 0.01f; // パッドアナログのミキシングゲイン

  float mv_dst_l[MP_JOINT_SIZE] = {}; // イージング後の目標角度 L
  float mv_dst_r[MP_JOINT_SIZE] = {}; // イージング後の目標角度 R

  float mv_cur_l[MP_JOINT_SIZE] = {};      // エンコーダ現在角度 L
  float mv_cur_r[MP_JOINT_SIZE] = {};      // エンコーダ現在角度 R
  float mv_cur_last_l[MP_JOINT_SIZE] = {}; // 前フレームのエンコーダ角度 L
  float mv_cur_last_r[MP_JOINT_SIZE] = {}; // 前フレームのエンコーダ角度 R

  float mv_div_l[MP_JOINT_SIZE] = {}; // 微分項 (位置差分) L
  float mv_div_r[MP_JOINT_SIZE] = {}; // 微分項 (位置差分) R
  float mv_int_l[MP_JOINT_SIZE] = {}; // 積分項 L
  float mv_int_r[MP_JOINT_SIZE] = {}; // 積分項 R

  // PIDゲイン (デフォルト: k_g=1.0 で直接追従, d_g/i_g=0 でPID無効)
  float k_g = 1.00f;
  float d_g = 0.00f;
  float i_g = 0.00f;

  // イージング関数
  float easeInOutSine(float t);
  float easeInOutQuad(float t);
  float easeInOutCubic(float t);
};

#endif // __MERIDIAN_MOVEMENT_MOTIONPLAY_H__
