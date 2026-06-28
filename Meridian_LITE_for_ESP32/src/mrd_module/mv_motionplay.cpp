#include "mv_motionplay.h"

//------------------------------------------------------------------------------------
//  init : モーション再生の初期化
//         mot[0]: 脱力→ゼロ復帰モーション (trigger = 1 = PAD_SELECT)
//         mot[1]: 脱力して終了 (trigger = 9 = PAD_SELECT + PAD_START)
//         mot[2]: サンプルウォークモーション (PAD_R_UP ボタンで起動)
//------------------------------------------------------------------------------------
void MotionPlayClass::init() {
  playing = false;
  for (int i = 0; i < MP_MOT_MAX; i++) {
    mot[i].trigger = 65535; // 未割り当て状態
  }
  // Note: mv_last_l/r, mv_act_l/r are zero-initialized in class declaration

  // --- mot[0]: 脱力→ゼロ復帰モーション (trigger = 1 = PAD_SELECT) ---
  // ①脱力して現在角度を読み出し (frame0, 100ms, cmd=0)
  // ②トルクONで全関節を1秒かけて0度へ (frame1, 1000ms, cmd=1)
  // ③終了・0度保持 (frame2, END)
  mp_no = 0;
  mot[mp_no].trigger = 1;
  {
    short header[MP_DATA_SIZE][5] = {
        {100, 1, 1, 1, 1},   // 0: 脱力 100ms, 常にframe1へ
        {1000, 1, 1, 2, 2},  // 1: 1秒で0度, 常にframe2へ
        {100, 255, 0, 2, 2}, // 2: END
    };
    short dst[MP_DATA_SIZE][MP_JOINT_SIZE] = {}; // 全ゼロ (0度目標)
    short cmd[MP_DATA_SIZE][MP_JOINT_SIZE] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 0: 脱力 (cmd=0)
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0}, // 1: 位置制御
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0}, // 2: 0度保持
    };
    memcpy(mot[mp_no].header, header, sizeof(header));
    memcpy(mot[mp_no].dst_l, dst, sizeof(dst));
    memcpy(mot[mp_no].dst_r, dst, sizeof(dst));
    memcpy(mot[mp_no].cmd_l, cmd, sizeof(cmd));
    memcpy(mot[mp_no].cmd_r, cmd, sizeof(cmd));
  }

  // --- mot[1]: 脱力して終了 (trigger = 9 = PAD_SELECT + PAD_START) ---
  // ①脱力 (frame0, 100ms, cmd=0)
  // ②終了・脱力維持 (frame1, END)
  mp_no = 1;
  mot[mp_no].trigger = 9;
  {
    short header[MP_DATA_SIZE][5] = {
        {100, 1, 9, 1, 1},   // 0: 脱力 100ms, 常にframe1へ
        {100, 255, 0, 1, 1}, // 1: END (cmd=0のまま脱力維持)
    };
    short dst[MP_DATA_SIZE][MP_JOINT_SIZE] = {}; // 全ゼロ
    short cmd[MP_DATA_SIZE][MP_JOINT_SIZE] = {}; // 全ゼロ (cmd=0 で脱力維持)
    memcpy(mot[mp_no].header, header, sizeof(header));
    memcpy(mot[mp_no].dst_l, dst, sizeof(dst));
    memcpy(mot[mp_no].dst_r, dst, sizeof(dst));
    memcpy(mot[mp_no].cmd_l, cmd, sizeof(cmd));
    memcpy(mot[mp_no].cmd_r, cmd, sizeof(cmd));
  }

  // --- mot[2]: walk motion (PAD_R_UP で起動) ---
  mp_no = 2;
  mot[mp_no].trigger = 16; // PAD_R_UP = 16 (PadButton enum)

  // header: [interval_ms, cmd, prm(btn_compare), next_pos, next_pos_not]
  // prm は ESP32 PAD_R_UP (16) に変換済み (参照元 KB_UP=1)
  short header[MP_DATA_SIZE][5] = {
      // 500-時間, 1-move命令(255-終了命令),16-分岐に使うレジスタ, 1- 次に行く番号を指示, 0-ボタンと不一致の条件分岐
      // 2行目の9は、分岐命令で、ボタンと不一致のときの飛び先
      {500, 1, 16, 1, 0},     // 0 stand(IDLE)   : PAD_R_UPで1へ, 未押下で0ループ
      {500, 1, 16, 2, 9},     // 1 stand stay     : PAD_R_UPで2へ, 未押下で9(END)
      {300, 1, 16, 3, 3},     // 2 swing-left
      {300, 1, 16, 4, 4},     // 3 up-right
      {300, 1, 16, 5, 5},     // 4 down-right1
      {100, 1, 16, 6, 9},     // 5 down-right2    : PAD_R_UPで6へ, 未押下で9(END)
      {300, 1, 16, 7, 7},     // 6 up-left
      {300, 1, 16, 8, 8},     // 7 down-left1
      {100, 1, 16, 3, 9},     // 8 down-left2     : PAD_R_UPで3(ループ), 未押下で9
      {300, 1, 16, 10, 10},   // 9 stand(END)
      {100, 255, 16, 10, 10}, // 10 END (cmd=255=HD_NEXT_CMD_END)
      {100, 0, 0, 0, 0},      // 11
      {100, 0, 0, 0, 0},      // 12
      {100, 0, 0, 0, 0},      // 13
      {100, 0, 0, 0, 0},      // 14
      {100, 0, 0, 0, 0},      // 15
      {100, 0, 0, 0, 0},      // 16
      {100, 0, 0, 0, 0},      // 17
      {100, 0, 0, 0, 0},      // 18
      {100, 0, 0, 0, 0}};     // 19
  memcpy(mot[mp_no].header, header, sizeof(header));

  // dst_* : 角度値は 10.0 deg → 1000 [hundredths of deg]
  short dst_l[MP_DATA_SIZE][MP_JOINT_SIZE] = {
      {0, 0, 1000, 0, 0, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},                // 0 stand(IDLE)
      {0, 0, 1000, 0, 0, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},                // 1 stand stay
      {0, 0, 1000, 0, -3000, 0, -500, -3000, 6000, -3000, 500, 0, 0, 0, 0},       // 2 swing-left
      {0, -3000, 1000, 0, -3000, 0, -1000, -3000, 6000, -3000, 1000, 0, 0, 0, 0}, // 3 up-right
      {0, 0, 1000, 0, -3000, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},            // 4 down-right1
      {0, 0, 1000, 0, -3000, 0, 500, -3000, 6000, -3000, -500, 0, 0, 0, 0},       // 5 down-right2
      {0, 3000, 1000, 0, -3000, 0, 1000, -6000, 9000, -3000, -1000, 0, 0, 0, 0},  // 6 up-left
      {0, 0, 1000, 0, -3000, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},            // 7 down-left1
      {0, 0, 1000, 0, -3000, 0, -500, -3000, 6000, -3000, 500, 0, 0, 0, 0},       // 8 down-left2
      {0, 0, 1000, 0, 0, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},                // 9 stand(END)
      {0, 0, 1000, 0, 0, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},                // 10 END
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 11
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 12
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 13
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 14
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 15
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 16
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 17
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 18
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};                             // 19
  memcpy(mot[mp_no].dst_l, dst_l, sizeof(dst_l));

  short dst_r[MP_DATA_SIZE][MP_JOINT_SIZE] = {
      {0, 0, 1000, 0, 0, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},                // 0 stand(IDLE)
      {0, 0, 1000, 0, 0, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},                // 1 stand stay
      {0, 0, 1000, 0, -3000, 0, 1000, -3000, 6000, -3000, -500, 0, 0, 0, 0},      // 2 swing-left
      {0, 3000, 1000, 0, -3000, 0, 1000, -6000, 9000, -3000, -1000, 0, 0, 0, 0},  // 3 up-right
      {0, 0, 1000, 0, -3000, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},            // 4 down-right1
      {0, 0, 1000, 0, -3000, 0, -500, -3000, 6000, -3000, 500, 0, 0, 0, 0},       // 5 down-right2
      {0, -3000, 1000, 0, -3000, 0, -1000, -3000, 6000, -3000, 1000, 0, 0, 0, 0}, // 6 up-left
      {0, 0, 1000, 0, -3000, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},            // 7 down-left1
      {0, 0, 1000, 0, -3000, 0, 500, -3000, 6000, -3000, -500, 0, 0, 0, 0},       // 8 down-left2
      {0, 0, 1000, 0, 0, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},                // 9 stand(END)
      {0, 0, 1000, 0, 0, 0, 0, -3000, 6000, -3000, 0, 0, 0, 0, 0},                // 10 END
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 11
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 12
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 13
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 14
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 15
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 16
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 17
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},                              // 18
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};                             // 19
  memcpy(mot[mp_no].dst_r, dst_r, sizeof(dst_r));

  // cmd: 1=位置指令, 0=未使用 (サーボ未マウント)
  short cmd_l[MP_DATA_SIZE][MP_JOINT_SIZE] = {
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 0 stand(IDLE)
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 1 stand stay
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 2 swing-left
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 3 up-right
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 4 down-right1
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 5 down-right2
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 6 up-left
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 7 down-left1
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 8 down-left2
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 9 stand(END)
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 10 END
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 11
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 12
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 13
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 14
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 15
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 16
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 17
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0}}; // 18
  memcpy(mot[mp_no].cmd_l, cmd_l, sizeof(cmd_l));

  short cmd_r[MP_DATA_SIZE][MP_JOINT_SIZE] = {
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 0 stand(IDLE)
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 1 stand stay
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 2 swing-left
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 3 up-right
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 4 down-right1
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 5 down-right2
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 6 up-left
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 7 down-left1
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 8 down-left2
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 9 stand(END)
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 10 END
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 11
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 12
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 13
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 14
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 15
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 16
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  // 17
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0}}; // 18
  memcpy(mot[mp_no].cmd_r, cmd_r, sizeof(cmd_r));

  Serial.println("MotionPlayClass initialized.");
}

//------------------------------------------------------------------------------------
//  set_status : パッドボタン値からモーション番号を選択
//------------------------------------------------------------------------------------
void MotionPlayClass::set_status(uint16_t pad_key) {
  for (int i = 0; i < MP_MOT_MAX; i++) {
    if ((mot[i].trigger == pad_key) && (playing == false)) {
      mp_no = i;
      break;
    }
  }
}

//------------------------------------------------------------------------------------
//  play : モーション再生メイン処理
//  @param pad_key      パッドボタン値 (pad_array.usval[0])
//  @param pad_analog   パッドアナログ値へのポインタ
//  @param s_arr        Meridim送信配列 (uint16_t, 現在のサーボ値読み取りにも使用)
//  @param sv           サーボパラメータへのポインタ
//------------------------------------------------------------------------------------
void MotionPlayClass::play(uint16_t pad_key, PadValue *pad_analog, uint16_t s_arr[], ServoParam *sv) {
  float es_t;

  // トリガーボタンが押されたらモーション開始
  if (pad_key && (playing == false) && (pad_key == mot[mp_no].trigger)) {
    mv_pos = 0;
    mv_last_pos = -1;
    mv_tmr = 0.0f;
    mv_next_interval = (float)mot[mp_no].header[mv_pos][HD_NEXT_INTERVAL] * 0.001f;
    playing = true;
  }

  // 時間経過したら次のキーフレームへ遷移
  if ((mv_tmr >= mv_next_interval) && (playing == true)) {
    switch (mot[mp_no].header[mv_pos][HD_NEXT_CMD]) {

    case HD_NEXT_CMD_MOVE:
    case HD_NEXT_CMD_BRNC_BTN:
    case HD_NEXT_CMD_JUMP:
      if (pad_key == (uint16_t)mot[mp_no].header[mv_pos][HD_NEXT_PRM])
        mv_pos = mot[mp_no].header[mv_pos][HD_NEXT_POS];
      else
        mv_pos = mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      break;

    case HD_NEXT_CMD_BRNC_BIT:
      if (pad_key & (uint16_t)mot[mp_no].header[mv_pos][HD_NEXT_PRM])
        mv_pos = mot[mp_no].header[mv_pos][HD_NEXT_POS];
      else
        mv_pos = mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      break;

    case HD_NEXT_CMD_BRNC_LP:
      if (mot[mp_no].loop_cnt > 0) {
        mv_pos = mot[mp_no].header[mv_pos][HD_NEXT_POS];
        mot[mp_no].loop_cnt--;
      } else {
        mv_pos = mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      }
      break;

    case HD_NEXT_CMD_BRNC_PA1: // stick_L_x
      mv_pos = (pad_analog->stick_L_x >= mot[mp_no].header[mv_pos][HD_NEXT_PRM])
                   ? mot[mp_no].header[mv_pos][HD_NEXT_POS]
                   : mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      break;

    case HD_NEXT_CMD_BRNC_PA2: // stick_L_y
      mv_pos = (pad_analog->stick_L_y >= mot[mp_no].header[mv_pos][HD_NEXT_PRM])
                   ? mot[mp_no].header[mv_pos][HD_NEXT_POS]
                   : mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      break;

    case HD_NEXT_CMD_BRNC_PA3: // stick_R_x
      mv_pos = (pad_analog->stick_R_x >= mot[mp_no].header[mv_pos][HD_NEXT_PRM])
                   ? mot[mp_no].header[mv_pos][HD_NEXT_POS]
                   : mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      break;

    case HD_NEXT_CMD_BRNC_PA4: // stick_R_y
      mv_pos = (pad_analog->stick_R_y >= mot[mp_no].header[mv_pos][HD_NEXT_PRM])
                   ? mot[mp_no].header[mv_pos][HD_NEXT_POS]
                   : mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      break;

    case HD_NEXT_CMD_END:
      mv_pos = mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      playing = false;
      break;

    case HD_NEXT_CMD_SET_REG:
      mv_pos = mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      break;

    case HD_NEXT_CMD_SET_LP:
      mot[mp_no].loop_cnt = mot[mp_no].header[mv_pos][HD_NEXT_PRM];
      mv_pos = mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      break;

    case HD_NEXT_CMD_GOSUB:
      mp_no = mot[mp_no].header[mv_pos][HD_NEXT_POS];
      mv_pos = 0;
      break;

    default:
      mv_pos = mot[mp_no].header[mv_pos][HD_NEXT_POS_NOT];
      break;
    }
  }

  if (playing == true) {

    // Meridim配列から現在のサーボ位置を取得
    for (int i = 0; i < MP_JOINT_SIZE; i++) {
      mv_cur_l[i] = (short)s_arr[MRD_L_ORIGIDX + 1 + i * 2] * 0.01f;
      mv_cur_r[i] = (short)s_arr[MRD_R_ORIGIDX + 1 + i * 2] * 0.01f;
      mv_div_l[i] = mv_cur_l[i] - mv_cur_last_l[i];
      mv_div_r[i] = mv_cur_r[i] - mv_cur_last_r[i];
      mv_cur_last_l[i] = mv_cur_l[i];
      mv_cur_last_r[i] = mv_cur_r[i];
    }

    // キーフレーム遷移時の初期化
    if (mv_pos != mv_last_pos) {
      mv_tmr = 0.0f + FRAME_RATE_10MS;
      mv_next_interval = (float)mot[mp_no].header[mv_pos][HD_NEXT_INTERVAL] * 0.001f * play_g;

      for (int i = 0; i < MP_JOINT_SIZE; i++) {
        mv_int_l[i] = 0.0f;
        mv_int_r[i] = 0.0f;

        // 起点 = 現在位置 - ミキシング量
        mv_last_l[i] = mv_cur_l[i] - mix_joint_l[i];
        mv_act_l[i] = (float)mot[mp_no].dst_l[mv_pos][i] * 0.01f;
        // CMD_READのとき目標を現在値で上書き (現在位置キープ)
        if (mot[mp_no].cmd_l[mv_pos][i] == MP_CMD_READ)
          mot[mp_no].dst_l[mv_pos][i] = (short)(mv_cur_l[i] * 100);

        mv_last_r[i] = mv_cur_r[i] - mix_joint_r[i];
        mv_act_r[i] = (float)mot[mp_no].dst_r[mv_pos][i] * 0.01f;
        if (mot[mp_no].cmd_r[mv_pos][i] == MP_CMD_READ)
          mot[mp_no].dst_r[mv_pos][i] = (short)(mv_cur_r[i] * 100);
      }

      mv_last_pos = mv_pos;
    }

    // 各サーボの補間計算とPID制御
    for (int i = 0; i < MP_JOINT_SIZE; i++) {
      joint_cmd_l[i] = mot[mp_no].cmd_l[mv_pos][i];
      joint_cmd_r[i] = mot[mp_no].cmd_r[mv_pos][i];

      // コマンドに応じてミキシング係数を更新
      switch (joint_cmd_l[i]) {
      case MP_CMD_STRETCH:
        joint_l[i] = mot[mp_no].dst_l[mv_pos][i];
        break;
      case MP_CMD_MIX1:
        mv_mix_l[MP_MIX_JOY_X_L][i] = mot[mp_no].dst_l[mv_pos][i];
        joint_cmd_l[i] = MP_CMD_READ;
        break;
      case MP_CMD_MIX2:
        mv_mix_l[MP_MIX_JOY_Y_L][i] = mot[mp_no].dst_l[mv_pos][i];
        joint_cmd_l[i] = MP_CMD_READ;
        break;
      case MP_CMD_MIX3:
        mv_mix_l[MP_MIX_JOY_X_R][i] = mot[mp_no].dst_l[mv_pos][i];
        joint_cmd_l[i] = MP_CMD_READ;
        break;
      case MP_CMD_MIX4:
        mv_mix_l[MP_MIX_JOY_Y_R][i] = mot[mp_no].dst_l[mv_pos][i];
        joint_cmd_l[i] = MP_CMD_READ;
        break;
      default:
        break;
      }

      switch (joint_cmd_r[i]) {
      case MP_CMD_STRETCH:
        joint_r[i] = mot[mp_no].dst_r[mv_pos][i];
        break;
      case MP_CMD_MIX1:
        mv_mix_r[MP_MIX_JOY_X_L][i] = mot[mp_no].dst_r[mv_pos][i];
        joint_cmd_r[i] = MP_CMD_READ;
        break;
      case MP_CMD_MIX2:
        mv_mix_r[MP_MIX_JOY_Y_L][i] = mot[mp_no].dst_r[mv_pos][i];
        joint_cmd_r[i] = MP_CMD_READ;
        break;
      case MP_CMD_MIX3:
        mv_mix_r[MP_MIX_JOY_X_R][i] = mot[mp_no].dst_r[mv_pos][i];
        joint_cmd_r[i] = MP_CMD_READ;
        break;
      case MP_CMD_MIX4:
        mv_mix_r[MP_MIX_JOY_Y_R][i] = mot[mp_no].dst_r[mv_pos][i];
        joint_cmd_r[i] = MP_CMD_READ;
        break;
      default:
        break;
      }

      // イージング補間 (easeInOutQuad)
      es_t = (mv_next_interval > 0.0f)
                 ? easeInOutQuad(mv_tmr / mv_next_interval)
                 : 1.0f;
      mv_dst_l[i] = (1.0f - es_t) * mv_last_l[i] + es_t * (float)mot[mp_no].dst_l[mv_pos][i] * 0.01f;
      mv_dst_r[i] = (1.0f - es_t) * mv_last_r[i] + es_t * (float)mot[mp_no].dst_r[mv_pos][i] * 0.01f;

      // CMD_READは現在位置を維持
      if (joint_cmd_l[i] == MP_CMD_READ)
        mv_dst_l[i] = mv_cur_l[i];
      if (joint_cmd_r[i] == MP_CMD_READ)
        mv_dst_r[i] = mv_cur_r[i];

      // PID制御 (デフォルト: k_g=1.0, d_g=0.0, i_g=0.0 → 直接追従)
      mv_int_l[i] += (mv_dst_l[i] - mv_cur_l[i]);
      mv_int_r[i] += (mv_dst_r[i] - mv_cur_r[i]);
      joint_l[i] = mv_cur_l[i] + k_g * (mv_dst_l[i] - mv_cur_l[i]) - d_g * mv_div_l[i] + i_g * mv_int_l[i];
      joint_r[i] = mv_cur_r[i] + k_g * (mv_dst_r[i] - mv_cur_r[i]) - d_g * mv_div_r[i] + i_g * mv_int_r[i];
    }

    mv_tmr += FRAME_RATE_10MS;

  } else {
    // 非再生時は最後の目標値を保持
    for (int i = 0; i < MP_JOINT_SIZE; i++) {
      joint_l[i] = mv_act_l[i];
      joint_r[i] = mv_act_r[i];
      joint_cmd_l[i] = mot[mp_no].cmd_l[mv_pos][i];
      joint_cmd_r[i] = mot[mp_no].cmd_r[mv_pos][i];
    }
  }

  // ジョイスティックアナログミキシング (gyroは除外)
  if (mix_enable) {
    for (int i = 0; i < MP_JOINT_SIZE; i++) {
      mix_joint_l[i] = 0.0f;
      mix_joint_r[i] = 0.0f;

      mix_joint_l[i] += pad_analog->stick_L_x * (float)(mv_mix_l[MP_MIX_JOY_X_L][i]) * mix_pad_g;
      mix_joint_l[i] += pad_analog->stick_L_y * (float)(mv_mix_l[MP_MIX_JOY_Y_L][i]) * mix_pad_g;
      mix_joint_l[i] += pad_analog->stick_R_x * (float)(mv_mix_l[MP_MIX_JOY_X_R][i]) * mix_pad_g;
      mix_joint_l[i] += pad_analog->stick_R_y * (float)(mv_mix_l[MP_MIX_JOY_Y_R][i]) * mix_pad_g;

      mix_joint_r[i] += pad_analog->stick_L_x * (float)(mv_mix_r[MP_MIX_JOY_X_L][i]) * mix_pad_g;
      mix_joint_r[i] += pad_analog->stick_L_y * (float)(mv_mix_r[MP_MIX_JOY_Y_L][i]) * mix_pad_g;
      mix_joint_r[i] += pad_analog->stick_R_x * (float)(mv_mix_r[MP_MIX_JOY_X_R][i]) * mix_pad_g;
      mix_joint_r[i] += pad_analog->stick_R_y * (float)(mv_mix_r[MP_MIX_JOY_Y_R][i]) * mix_pad_g;

      joint_l[i] += mix_joint_l[i];
      joint_r[i] += mix_joint_r[i];
    }
  }
}

//------------------------------------------------------------------------------------
//  updateServoData : Meridim配列とサーボ構造体に計算結果を書き込む
//  @param m_arr  Meridim送信配列 (uint16_t)
//  @param sv     サーボパラメータへのポインタ
//------------------------------------------------------------------------------------
void MotionPlayClass::updateServoData(uint16_t m_arr[], ServoParam *sv) {
  for (int i = 0; i < MP_JOINT_SIZE; i++) {
    if (sv->ixl_mount[i]) {
      m_arr[MRD_L_ORIGIDX + i * 2]     = (uint16_t)joint_cmd_l[i];
      m_arr[MRD_L_ORIGIDX + 1 + i * 2] = (uint16_t)mrd.float2HfShort(joint_l[i]);
      sv->ixl_tgt[i] = joint_l[i];
    }
    if (sv->ixr_mount[i]) {
      m_arr[MRD_R_ORIGIDX + i * 2]     = (uint16_t)joint_cmd_r[i];
      m_arr[MRD_R_ORIGIDX + 1 + i * 2] = (uint16_t)mrd.float2HfShort(joint_r[i]);
      sv->ixr_tgt[i] = joint_r[i];
    }
  }
}

// Private easing functions
float MotionPlayClass::easeInOutSine(float t) {
  return -0.5f * (cosf(M_PI * t) - 1.0f);
}

float MotionPlayClass::easeInOutQuad(float t) {
  return (t < 0.5f) ? 2.0f * t * t : -1.0f + (4.0f - 2.0f * t) * t;
}

float MotionPlayClass::easeInOutCubic(float t) {
  if (t < 0.5f)
    return 4.0f * t * t * t;
  float f = t - 1.0f;
  return 1.0f + 4.0f * f * f * f;
}
