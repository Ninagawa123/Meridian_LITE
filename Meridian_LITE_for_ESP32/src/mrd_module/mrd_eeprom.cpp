// mrd_eeprom.cpp - EEPROM handling implementation

#include "mrd_eeprom.h"
#include "../keys.h"

//------------------------------------------------------------------------------------
//  グローバル変数
//------------------------------------------------------------------------------------

UnionEEPROM eeprom_write_data; // EEPROM書き込みバッファ
UnionEEPROM eeprom_read_data;  // EEPROM読み込みバッファ

// ネットワーク設定ランタイムバッファ (EEPROM_LOAD=1 時にEEPROMから取得)
char     eep_ssid[33]     = WIFI_AP_SSID;
char     eep_pass[65]     = WIFI_AP_PASS;
char     eep_send_ip[16]  = WIFI_SEND_IP;
char     eep_fixed_ip[16] = FIXED_IP_ADDR;
char     eep_gateway[16]  = FIXED_IP_GATEWAY;
char     eep_subnet[16]   = FIXED_IP_SUBNET;
uint16_t eep_send_port    = UDP_SEND_PORT;
uint16_t eep_recv_port    = UDP_RECV_PORT;
uint16_t eep_frame_ms     = FRAME_DURATION;

//------------------------------------------------------------------------------------
//  ファイルローカルヘルパー関数
//------------------------------------------------------------------------------------

// 文字列を 2文字/word でpackしてEEPROM wordバッファに書き込む.
// マップ仕様: upper byte = 先頭文字, lower byte = 次の文字.
static void s_pack_str(const char *src, uint16_t *dst, int word_count) {
  int len = (int)strlen(src);
  for (int i = 0; i < word_count; i++) {
    uint8_t c0 = (2 * i     < len) ? (uint8_t)src[2 * i]     : 0;
    uint8_t c1 = (2 * i + 1 < len) ? (uint8_t)src[2 * i + 1] : 0;
    dst[i] = ((uint16_t)c0 << 8) | c1;
  }
}

// EEPROMのwordバッファから2文字/wordで文字列をunpackする.
static void s_unpack_str(const uint16_t *src, char *dst, int word_count) {
  for (int i = 0; i < word_count; i++) {
    dst[2 * i]     = (char)((src[i] >> 8) & 0xFF);
    dst[2 * i + 1] = (char)(src[i] & 0xFF);
  }
  dst[word_count * 2] = '\0';
}

// IPアドレス文字列 "A.B.C.D" を2ワードにpackする (upper byte = 先頭オクテット).
static void s_pack_ip(uint16_t *usval, int word_idx, const char *ip_str) {
  unsigned int o[4] = {0, 0, 0, 0};
  sscanf(ip_str, "%u.%u.%u.%u", &o[0], &o[1], &o[2], &o[3]);
  usval[word_idx]     = (uint16_t)(((o[0] & 0xFF) << 8) | (o[1] & 0xFF));
  usval[word_idx + 1] = (uint16_t)(((o[2] & 0xFF) << 8) | (o[3] & 0xFF));
}

// EEPROMの2ワードからIPアドレス文字列 "A.B.C.D" をunpackする.
static void s_unpack_ip(const uint16_t *usval, int word_idx, char *ip_str) {
  snprintf(ip_str, 16, "%u.%u.%u.%u",
    (usval[word_idx] >> 8) & 0xFF,
    usval[word_idx] & 0xFF,
    (usval[word_idx + 1] >> 8) & 0xFF,
    usval[word_idx + 1] & 0xFF);
}

//==================================================================================================
//  EEPROM関連の処理
//==================================================================================================

/// @brief EEPROMの初期化
/// @param a_eeprom_size EEPROMのバイト長
/// @return 初期化が成功すればtrue, 失敗ならfalseを返す.
bool mrd_eeprom_init(int a_eeprom_size) {
  return EEPROM.begin(a_eeprom_size);
}

/// @brief config.h の設定値からEEPROM格納用の配列データを作成する.
/// @param a_sv サーボ設定を保持する構造体.
/// @return EEPROM格納用の配列データ(UnionEEPROM型).
UnionEEPROM mrd_eeprom_make_data_from_config(const ServoParam &a_sv) {
  UnionEEPROM array_tmp = {0};

  // 書き込みカウンタ: 既存EEPROMが初期化済みならカウントを継承してインクリメント
  UnionEEPROM cur = mrd_eeprom_read();
  uint32_t write_count = 0;
  if ((cur.usval[EEP_W_HEADER] & 0xFF) == EEP_INIT_ID) {
    write_count = ((uint32_t)cur.usval[EEP_W_WRCNT_H] << 16) | cur.usval[EEP_W_WRCNT_L];
  }
  write_count++;

  // [Word 0] ヘッダー: upper=LEN(180), lower=0x55識別子
  array_tmp.usval[EEP_W_HEADER] = ((uint16_t)EEP_FIXED_WORDS << 8) | EEP_INIT_ID;

  // [Word 1] EEPROMバージョン
  array_tmp.usval[EEP_W_VERSION] = EEPROM_VERSION;

  // [Word 2-3] 書き込みカウンタ (32bit: 上位word, 下位word)
  array_tmp.usval[EEP_W_WRCNT_H] = (uint16_t)(write_count >> 16);
  array_tmp.usval[EEP_W_WRCNT_L] = (uint16_t)(write_count & 0xFFFF);

  // [Word 4] 起動時モーション番号 (0:無効)
  array_tmp.usval[EEP_W_BOOT_MOT] = 0;

  // [Word 5] ネットワーク設定 (upper=profile#0, lower=mode: 1=固定IP)
  array_tmp.usval[EEP_W_NET_MODE] = (uint16_t)(MODE_FIXED_IP & 0xFF);

  // [Word 6-21] WiFi SSID (16words=32chars)
  s_pack_str(WIFI_AP_SSID, &array_tmp.usval[EEP_W_SSID], 16);

  // [Word 22-53] WiFi PASS (32words=64chars)
  s_pack_str(WIFI_AP_PASS, &array_tmp.usval[EEP_W_PASS], 32);

  // [Word 54-55] 送信先PC IPアドレス
  s_pack_ip(array_tmp.usval, EEP_W_SEND_IP, WIFI_SEND_IP);

  // [Word 56-57] ESP32固定IPアドレス
  s_pack_ip(array_tmp.usval, EEP_W_FIXED_IP, FIXED_IP_ADDR);

  // [Word 58-59] ゲートウェイIPアドレス
  s_pack_ip(array_tmp.usval, EEP_W_GATEWAY, FIXED_IP_GATEWAY);

  // [Word 60-61] サブネットマスク
  s_pack_ip(array_tmp.usval, EEP_W_SUBNET, FIXED_IP_SUBNET);

  // [Word 62] UDP送信ポート
  array_tmp.usval[EEP_W_SEND_PORT] = UDP_SEND_PORT;

  // [Word 63] UDP受信ポート
  array_tmp.usval[EEP_W_RECV_PORT] = UDP_RECV_PORT;

  // [Word 64] UDP待受タイムアウト (lower byte)
  array_tmp.usval[EEP_W_UDP_TMOUT] = (uint16_t)(UDP_TIMEOUT & 0xFF);

  // [Word 65] I2C_0速度 (/1000)
  array_tmp.usval[EEP_W_I2C0_SPD] = (uint16_t)(I2C0_SPEED / 1000);

  // [Word 66] I2C_1速度 (/1000) - 未定義につき0
  array_tmp.usval[EEP_W_I2C1_SPD] = 0;

  // [Word 67] SPI_0速度 (/1000)
  array_tmp.usval[EEP_W_SPI0_SPD] = (uint16_t)(SPI0_SPEED / 1000);

  // [Word 68] SPI_1速度 (/1000) - 未定義につき0
  array_tmp.usval[EEP_W_SPI1_SPD] = 0;

  // [Word 69] 起動待ち時間 (ms)
  array_tmp.usval[EEP_W_CHARGE] = (uint16_t)CHARGE_TIME;

  // [Word 70] SDマウント/チェックフラグ (lower byte: bit0=mount, bit1=check)
  array_tmp.usval[EEP_W_SD_FLAGS] = (uint16_t)(MOUNT_SD ? 0x01 : 0x00);

  // [Word 71] 各種マウントフラグ (将来用)
  array_tmp.usval[EEP_W_MNT_FLAGS] = 0;

  // [Word 72] 拡張シリアル速度(/100) - 未定義につき0
  array_tmp.usval[EEP_W_EXSERIAL_BPS] = 0;

  // [Word 73] 拡張シリアルタイムアウト - 未定義につき0
  array_tmp.usval[EEP_W_EXSERIAL_TMO] = 0;

  // [Word 74] PCシリアル速度 (/100)
  array_tmp.usval[EEP_W_PC_BPS] = (uint16_t)(SERIAL_PC_BPS / 100);

  // [Word 75] PCシリアルタイムアウト
  array_tmp.usval[EEP_W_PC_TMOUT] = (uint16_t)SERIAL_PC_TIMEOUT;

  // [Word 76] サーボプロトコルタイプ (upper=L系, lower=R系)
  array_tmp.usval[EEP_W_SV_TYPE_LR] =
    ((uint16_t)(MOUNT_SV_TYPE_L & 0xFF) << 8) | (MOUNT_SV_TYPE_R & 0xFF);

  // [Word 77] サーボプロトコルタイプ (upper=C系, lower=X系) - 未定義につき0
  array_tmp.usval[EEP_W_SV_TYPE_CX] = 0;

  // [Word 78] サーボ通信速度L (/100)
  array_tmp.usval[EEP_W_SV_BPS_L] = (uint16_t)(SV_BAUDRATE_L / 100);

  // [Word 79] サーボ通信速度R (/100)
  array_tmp.usval[EEP_W_SV_BPS_R] = (uint16_t)(SV_BAUDRATE_R / 100);

  // [Word 80-81] サーボ通信速度C/X - 未定義につき0
  array_tmp.usval[EEP_W_SV_BPS_C] = 0;
  array_tmp.usval[EEP_W_SV_BPS_X] = 0;

  // [Word 82] サーボタイムアウト (upper=L系, lower=R系)
  array_tmp.usval[EEP_W_SV_TMO_LR] =
    ((uint16_t)(SV_TIMEOUT_L & 0xFF) << 8) | (SV_TIMEOUT_R & 0xFF);

  // [Word 83] サーボタイムアウト C/X - 未定義につき0
  array_tmp.usval[EEP_W_SV_TMO_CX] = 0;

  // [Word 84] サーボロスト判定フレーム数 (lower byte)
  array_tmp.usval[EEP_W_SV_LOST] = (uint16_t)(SV_LOST_ERR_WAIT & 0xFF);

  // [Word 85-89] 予約 (0)

  // [Word 90] 動作モード (0=スタンドアロン, 1=Meridian通信)
  array_tmp.usval[EEP_W_OP_MODE] = MODE_ESP32_STANDALONE ? 0 : 1;

  // [Word 91] IMU/AHRSタイプ (lower byte)
  array_tmp.usval[EEP_W_IMU_TYPE] = (uint16_t)(MOUNT_IMUAHRS & 0xFF);

  // [Word 92-97] MPU6050キャリブレーション値 (6軸分)
  array_tmp.usval[EEP_W_IMU_CAL + 0] = (uint16_t)(int16_t)MPU6050_ACCEL_OFFSET_X;
  array_tmp.usval[EEP_W_IMU_CAL + 1] = (uint16_t)(int16_t)MPU6050_ACCEL_OFFSET_Y;
  array_tmp.usval[EEP_W_IMU_CAL + 2] = (uint16_t)(int16_t)MPU6050_ACCEL_OFFSET_Z;
  array_tmp.usval[EEP_W_IMU_CAL + 3] = (uint16_t)(int16_t)MPU6050_GYRO_OFFSET_X;
  array_tmp.usval[EEP_W_IMU_CAL + 4] = (uint16_t)(int16_t)MPU6050_GYRO_OFFSET_Y;
  array_tmp.usval[EEP_W_IMU_CAL + 5] = (uint16_t)(int16_t)MPU6050_GYRO_OFFSET_Z;

  // [Word 98] IMU読み取り間隔(upper), 移動平均数(lower)
  array_tmp.usval[EEP_W_IMU_INTV] =
    ((uint16_t)(IMUAHRS_INTERVAL & 0xFF) << 8) | (IMUAHRS_STOCK & 0xFF);

  // [Word 99-108] C系サーボID0-4 (未使用につき0)

  // [Word 109] フレーム時間 (ms)
  array_tmp.usval[EEP_W_FRAME_MS] = (uint16_t)FRAME_DURATION;

  // [Word 110-139] L系サーボID0-14 (config word + trim word × 15個)
  for (int i = 0; i < 15; i++) {
    uint16_t cfg = 0;
    if (a_sv.ixl_mount[i])                            cfg |= 0x0001;       // bit0: マウント
    cfg |= (uint16_t)(a_sv.ixl_id[i] & 0x7F) << 1;                         // bit1-7: サーボID
    if (a_sv.ixl_cw[i] > 0)                           cfg |= 0x0100;       // bit8: CW方向
    array_tmp.usval[EEP_W_L_SV + i * 2]     = cfg;
    array_tmp.sval[EEP_W_L_SV + i * 2 + 1]  = mrd.float2HfShort(a_sv.ixl_trim[i]);
  }

  // [Word 140-169] R系サーボID0-14 (config word + trim word × 15個)
  for (int i = 0; i < 15; i++) {
    uint16_t cfg = 0;
    if (a_sv.ixr_mount[i])                            cfg |= 0x0001;
    cfg |= (uint16_t)(a_sv.ixr_id[i] & 0x7F) << 1;
    if (a_sv.ixr_cw[i] > 0)                           cfg |= 0x0100;
    array_tmp.usval[EEP_W_R_SV + i * 2]     = cfg;
    array_tmp.sval[EEP_W_R_SV + i * 2 + 1]  = mrd.float2HfShort(a_sv.ixr_trim[i]);
  }

  // [Word 170-174] 予約 (0)

  // [Word 175] リモコンタイプ (upper=padタイプ, bit1=merge, bit0=generalize)
  {
    uint16_t pad_cfg = (uint16_t)((MOUNT_PAD & 0x7F) << 8);
    if (PAD_GENERALIZE)   pad_cfg |= 0x01;
    if (PAD_BUTTON_MERGE) pad_cfg |= 0x02;
    array_tmp.usval[EEP_W_PAD_TYPE] = pad_cfg;
  }

  // [Word 176] リモコン詳細 (upper=timeout/1000[s], lower=interval[frame])
  array_tmp.usval[EEP_W_PAD_DETAIL] =
    ((uint16_t)((PAD_INIT_TIMEOUT / 1000) & 0xFF) << 8) | (PAD_INTERVAL & 0xFF);

  // [Word 177] 予約 (0)

  // [Word 178] モニタリングフラグ (lower byte)
  {
    uint8_t mflags = 0;
    if (MONITOR_FLOW)      mflags |= 0x01;
    if (MONITOR_ERR_ALL)   mflags |= 0x02;
    if (MONITOR_ERR_SERVO) mflags |= 0x04;
    if (MONITOR_SEQ)       mflags |= 0x08;
    if (MONITOR_PAD)       mflags |= 0x10;
    array_tmp.usval[EEP_W_MONITOR] = mflags;
  }

  // [Word 179] CRC16 (word 0-178の範囲)
  array_tmp.usval[EEP_W_CRC] = mrd_crc16(array_tmp.usval, EEP_W_CRC);

  return array_tmp;
}

/// @brief EEPROMの内容を読み込んで返す.
/// @return UnionEEPROM のフォーマットで配列を返す.
UnionEEPROM mrd_eeprom_read() {
  UnionEEPROM read_data_tmp = {0};
  for (int i = 0; i < EEPROM_SIZE; i++) {
    read_data_tmp.bval[i] = EEPROM.read(i);
  }
  return read_data_tmp;
}

/// @brief EEPROMの内容を読み込みランタイム設定に反映する.
/// @param a_sv サーボ設定を保持する構造体.
/// @param a_serial 出力先シリアルの指定.
/// @return 成功時にtrue, CRC不一致などの場合はfalseを返す.
bool mrd_eeprom_load_config(ServoParam &a_sv, HardwareSerial &a_serial) {
  a_serial.println("Load config from EEPROM.");
  UnionEEPROM d = mrd_eeprom_read();

  // EEPROMが初期化済みか確認 (word 0 lower byte == EEP_INIT_ID)
  if ((d.usval[EEP_W_HEADER] & 0xFF) != EEP_INIT_ID) {
    a_serial.println("  EEPROM not initialized. Skip.");
    return false;
  }

  // CRC16 検証
  uint16_t crc_calc = mrd_crc16(d.usval, EEP_W_CRC);
  if (crc_calc != d.usval[EEP_W_CRC]) {
    a_serial.print("  EEPROM CRC mismatch (calc=");
    a_serial.print(crc_calc, HEX);
    a_serial.print(", stored=");
    a_serial.print(d.usval[EEP_W_CRC], HEX);
    a_serial.println("). Skip.");
    return false;
  }

  // === ネットワーク設定 ===
  s_unpack_str(&d.usval[EEP_W_SSID], eep_ssid, 16);
  eep_ssid[32] = '\0';
  s_unpack_str(&d.usval[EEP_W_PASS], eep_pass, 32);
  eep_pass[64] = '\0';
  s_unpack_ip(d.usval, EEP_W_SEND_IP,  eep_send_ip);
  s_unpack_ip(d.usval, EEP_W_FIXED_IP, eep_fixed_ip);
  s_unpack_ip(d.usval, EEP_W_GATEWAY,  eep_gateway);
  s_unpack_ip(d.usval, EEP_W_SUBNET,   eep_subnet);
  eep_send_port = d.usval[EEP_W_SEND_PORT];
  eep_recv_port = d.usval[EEP_W_RECV_PORT];

  a_serial.println("  Network config loaded.");

  // === L系サーボ設定 ===
  for (int i = 0; i < 15; i++) {
    uint16_t cfg = d.usval[EEP_W_L_SV + i * 2];
    a_sv.ixl_mount[i] = (cfg & 0x0001) ? 1 : 0;
    a_sv.ixl_id[i]    = (cfg >> 1) & 0x7F;
    a_sv.ixl_cw[i]    = (cfg & 0x0100) ? 1 : -1;
    a_sv.ixl_trim[i]  = d.sval[EEP_W_L_SV + i * 2 + 1] / 100.0f;
  }

  // === R系サーボ設定 ===
  for (int i = 0; i < 15; i++) {
    uint16_t cfg = d.usval[EEP_W_R_SV + i * 2];
    a_sv.ixr_mount[i] = (cfg & 0x0001) ? 1 : 0;
    a_sv.ixr_id[i]    = (cfg >> 1) & 0x7F;
    a_sv.ixr_cw[i]    = (cfg & 0x0100) ? 1 : -1;
    a_sv.ixr_trim[i]  = d.sval[EEP_W_R_SV + i * 2 + 1] / 100.0f;
  }

  // マウント済みサーボ数からnum_maxを再計算
  a_sv.num_max = max(mrd_max_used_index(a_sv.ixl_mount, IXL_MAX),
                     mrd_max_used_index(a_sv.ixr_mount, IXR_MAX));
  a_serial.print("  Servo config loaded. (num_max=");
  a_serial.print(a_sv.num_max);
  a_serial.println(")");

  // === IMUタイプ / フレーム時間 ===
  // eep_frame_ms にロード: main.cpp でタイマー設定に使用
  eep_frame_ms = d.usval[EEP_W_FRAME_MS];
  if (eep_frame_ms == 0) {
    eep_frame_ms = FRAME_DURATION; // 0は異常値なのでデフォルトに戻す
  }

  a_serial.print("  Frame time: ");
  a_serial.print(eep_frame_ms);
  a_serial.println(" ms.");

  // IMUタイプ (参考情報として表示、ハードウェア初期化には再起動が必要)
  uint8_t imu_type = d.usval[EEP_W_IMU_TYPE] & 0xFF;
  a_serial.print("  IMU type (from EEPROM): ");
  a_serial.println(imu_type);

  a_serial.println("EEPROM config load done.");
  return true;
}

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.
/// @param a_data EEPROM用の配列データ.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_to_serial(UnionEEPROM a_data, int a_bhd, HardwareSerial &a_serial) {
  int len_tmp = EEPROM.length();
  a_serial.print("EEPROM Length ");
  a_serial.print(len_tmp);
  a_serial.println("byte, 16bit Dump:");
  for (int i = 0; i < EEPROM_SIZE / 2; i++) {
    if (a_bhd == 0) {
      a_serial.print(a_data.sval[i], BIN);
    } else if (a_bhd == 1) {
      a_serial.print(a_data.sval[i], HEX);
    } else {
      a_serial.print(a_data.sval[i], DEC);
    }
    if ((i + 1) % 90 == 0 || i == EEPROM_SIZE / 2 - 1) {
      a_serial.println();
    } else {
      a_serial.print("/");
    }
  }
  return true;
}

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.(起動時用)
/// @param a_do_dump 実施するか否か.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_at_boot(bool a_do_dump, int a_bhd, HardwareSerial &a_serial) {
  if (a_do_dump) {
    mrd_eeprom_dump_to_serial(mrd_eeprom_read(), a_bhd, a_serial);
    return true;
  }
  return false;
}

/// @brief EEPROMにEEPROM格納用の配列データを書き込む.
///        変化のないバイトはスキップして書き込み回数を節約する(劣化防止).
/// @param a_write_data EEPROM書き込み用の配列データ.
/// @param a_flg_protect EEPROMの書き込み許可があるかどうかのブール値.
/// @return 書き込みが実施された場合はtrue, スキップ/保護の場合はfalseを返す.
bool mrd_eeprom_write(UnionEEPROM a_write_data, bool a_flg_protect, HardwareSerial &a_serial) {
  if (a_flg_protect) {
    return false;
  }
  if (flg.eeprom_protect) {
    a_serial.println("EEPROM is protected. Set 'EEPROM_PROTECT' to 0 to enable writing.");
    return false;
  }

  // 変化のあるバイトのみ書き込む (劣化防止)
  bool flg_renew_tmp = false;
  for (int i = 0; i < EEPROM_SIZE; i++) {
    if (i >= EEPROM.length()) {
      a_serial.println("Error: EEPROM address out of range.");
      return false;
    }
    uint8_t old_val = EEPROM.read(i);
    if (old_val != a_write_data.bval[i]) {
      EEPROM.write(i, a_write_data.bval[i]);
      flg_renew_tmp = true;
    }
  }

  // サーボ設定の表示
  for (int i = 0; i < 15; i++) {
    a_serial.print("L-idx:");
    a_serial.print(mrd_pddstr(i, 2, 0, false));
    a_serial.print(", id:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usval[EEP_W_L_SV + i * 2], 1, 7), 2, 0, false));
    a_serial.print(", mt:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usval[EEP_W_L_SV + i * 2], 0, 1), 1, 0, false));
    a_serial.print(", cw:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usval[EEP_W_L_SV + i * 2], 8, 1), 1, 0, false));
    a_serial.print(", trm:");
    a_serial.print(mrd_pddstr(a_write_data.sval[EEP_W_L_SV + i * 2 + 1] / 100.0f, 7, 2, true));
    a_serial.print("  R-idx:");
    a_serial.print(mrd_pddstr(i, 2, 0, false));
    a_serial.print(", id:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usval[EEP_W_R_SV + i * 2], 1, 7), 2, 0, false));
    a_serial.print(", mt:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usval[EEP_W_R_SV + i * 2], 0, 1), 1, 0, false));
    a_serial.print(", cw:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usval[EEP_W_R_SV + i * 2], 8, 1), 1, 0, false));
    a_serial.print(", trm:");
    a_serial.println(mrd_pddstr(a_write_data.sval[EEP_W_R_SV + i * 2 + 1] / 100.0f, 7, 2, true));
  }

  if (flg_renew_tmp) {
    EEPROM.commit();
    a_serial.print("EEPROM updated. (write count: ");
    uint32_t cnt = ((uint32_t)a_write_data.usval[EEP_W_WRCNT_H] << 16) |
                    a_write_data.usval[EEP_W_WRCNT_L];
    a_serial.print(cnt);
    a_serial.println(")");
    return true;
  } else {
    a_serial.println("EEPROM: no change.");
  }
  return false;
}

/// @brief EEPROMに設定値を書き込み, その後で読み込んで内容を確認し, シリアルポートに出力する.
/// @param a_write_data EEPROM書き込み用の配列データ.
/// @param a_do EEPROMの読み書きチェックを実施するかのブール値.
/// @param a_protect EEPROMの書き込み許可があるかどうかのブール値.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, それ以外はfalseを返す.
bool mrd_eeprom_write_read_check(UnionEEPROM a_write_data, bool a_do, bool a_protect, int a_bhd) {
  if (!a_do) {
    return false;
  }
  Serial.println("Try to write EEPROM: ");
  mrd_eeprom_dump_to_serial(a_write_data, a_bhd, Serial);

  if (mrd_eeprom_write(a_write_data, a_protect, Serial)) {
    Serial.println("...Write OK.");
  } else {
    Serial.println("...Write failed.");
    return false;
  }

  Serial.println("Read EEPROM: ");
  UnionEEPROM read_data_tmp = mrd_eeprom_read();
  mrd_eeprom_dump_to_serial(read_data_tmp, a_bhd, Serial);
  Serial.println("...Read completed.");
  return true;
}

//------------------------------------------------------------------------------------
//  各種オペレーション (旧インデックス形式, 後方互換)
//------------------------------------------------------------------------------------

/// @brief EEPROMから任意のshort型データを読み込む(旧インデックス形式).
/// @param index_y 配列の一次元目(0~2).
/// @param index_x 配列の二次元目(0~89).
/// @return short型データを返す.
short mrd_eeprom_read_short(int index_y, int index_x) {
  int addr = (index_y * 90 + index_x) * 2;
  uint8_t low_byte  = EEPROM.read(addr);
  uint8_t high_byte = EEPROM.read(addr + 1);
  return static_cast<short>((high_byte << 8) | low_byte);
}

/// @brief EEPROMから任意のbyte型データを読み込む(旧インデックス形式).
/// @param index_y 配列の一次元目(0~2).
/// @param index_x 配列の二次元目(0~179).
/// @param low_high 下位ビットか上位ビットか. (0:low_bit, 1:high_bit)
/// @return byte型データを返す.
int8_t mrd_eeprom_read_byte(int index_y, int index_x, int low_high) {
  return (int8_t)EEPROM.read(index_y * 180 + index_x * 2 + low_high);
}
