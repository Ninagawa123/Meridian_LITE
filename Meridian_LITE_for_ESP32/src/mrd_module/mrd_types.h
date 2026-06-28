#ifndef __MERIDIAN_TYPES_H__
#define __MERIDIAN_TYPES_H__

#include <stdint.h>
#include "../config.h"
#include <Meridian.h>
#include <IcsHardSerialClass.h>
#include <Adafruit_BNO055.h>

//------------------------------------------------------------------------------------
//  定数
//------------------------------------------------------------------------------------

const int MRDM_BYTE = MRDM_LEN * 2;
const int MRD_ERR   = MRDM_LEN - 2;
const int MRD_ERR_u = MRD_ERR * 2 + 1;
const int MRD_ERR_l = MRD_ERR * 2;
const int MRD_CKSM  = MRDM_LEN - 1;
const int PAD_LEN   = 4;

//------------------------------------------------------------------------------------
//  列挙型
//------------------------------------------------------------------------------------

enum UartLine { L, R, C };

enum PadType {
  NONE      = 0,
  PC        = 0,
  MERIMOTE  = 1,
  BLUERETRO = 2,
  SBDBT     = 3,
  KRR5FH    = 4,
  WIIMOTE   = 5,
  WIIMOTE_C = 6,
};

enum ServoType {
  SERVO_TYPE_NONE    = 0,
  SERVO_TYPE_PWM_S   = 1,
  SERVO_TYPE_PCA9685 = 11,
  SERVO_TYPE_FTBRSX  = 21,
  SERVO_TYPE_DXL1    = 31,
  SERVO_TYPE_DXL2    = 32,
  SERVO_TYPE_KRSICS3 = 43,
  SERVO_TYPE_PMX     = 44,
  SERVO_TYPE_JRXBUS  = 51,
  SERVO_TYPE_FTCSTS  = 61,
  SERVO_TYPE_FTCSCS  = 62
};

enum ImuAhrsType {
  IMU_TYPE_NONE    = 0,
  IMU_TYPE_MPU6050 = 1,
  IMU_TYPE_MPU9250 = 2,
  IMU_TYPE_BNO055  = 3
};

enum PadButton {
  PAD_SELECT  = 1,
  PAD_L3      = 2,
  PAD_HOME    = PAD_L3,
  PAD_R3      = 4,
  PAD_START   = 8,
  PAD_R_UP    = 16,
  PAD_R_RIGHT = 32,
  PAD_R_DOWN  = 64,
  PAD_R_LEFT  = 128,
  PAD_L2      = 256,
  PAD_R2      = 512,
  PAD_L1      = 1024,
  PAD_R1      = 2048,
  PAD_L_UP    = 4096,
  PAD_L_RIGHT = 8192,
  PAD_L_DOWN  = 16384,
  PAD_L_LEFT  = 32768
};

enum BinHexDec {
  DISP_TYPE_BIN = 0,
  DISP_TYPE_HEX = 1,
  DISP_TYPE_DEC = 2
};

//------------------------------------------------------------------------------------
//  共用体
//------------------------------------------------------------------------------------

typedef union {
  short          sval[MRDM_LEN + 2];
  unsigned short usval[MRDM_LEN + 2];
  int8_t         bval[MRDM_BYTE + 4];
  uint8_t        ubval[MRDM_BYTE + 4];
} Meridim90Union;

typedef union {
  short    sval[PAD_LEN];
  uint16_t usval[PAD_LEN];
  int8_t   bval[PAD_LEN * 2];
  uint8_t  ubval[PAD_LEN * 2];
  uint64_t ui64val; // [0]button [1]stick_L [2]stick_R [3]L2R2
} PadUnion;

//------------------------------------------------------------------------------------
//  構造体
//------------------------------------------------------------------------------------

struct ServoParam {
  int num_max;
  int    ixl_mount[IXL_MAX];
  int    ixr_mount[IXR_MAX];
  int    ixl_id[IXL_MAX];
  int    ixr_id[IXR_MAX];
  int    ixl_cw[IXL_MAX];
  int    ixr_cw[IXR_MAX];
  float  ixl_trim[IXL_MAX];
  float  ixr_trim[IXR_MAX];
  float  ixl_tgt[IXL_MAX]      = {0};
  float  ixr_tgt[IXR_MAX]      = {0};
  float  ixl_tgt_past[IXL_MAX] = {0};
  float  ixr_tgt_past[IXR_MAX] = {0};
  int    ixl_err[IXL_MAX]      = {0};
  int    ixr_err[IXR_MAX]      = {0};
  uint16_t ixl_stat[IXL_MAX]   = {0};
  uint16_t ixr_stat[IXR_MAX]   = {0};
};

struct MrdFlags {
  bool imuahrs_available  = true;
  bool udp_board_passive  = false;
  bool count_frame_reset  = false;
  bool stop_board_during  = false;
  bool eeprom_write_mode  = false;
  bool eeprom_read_mode   = false;
  bool eeprom_protect     = EEPROM_PROTECT;
  bool eeprom_load        = EEPROM_LOAD;
  bool eeprom_set         = EEPROM_SET;
  bool sdcard_write_mode  = false;
  bool sdcard_read_mode   = false;
  bool wire0_init         = false;
  bool wire1_init         = false;
  bool bt_busy            = false;
  bool spi_rcvd           = true;
  bool udp_rcvd           = false;
  bool udp_busy           = false;
  bool udp_receive_mode   = MODE_UDP_RECEIVE;
  bool udp_send_mode      = MODE_UDP_SEND;
  bool meridim_rcvd       = false;
};

struct MrdSq {
  int s_increment = 0;
  int r_expect    = 0;
};

struct MrdTimer {
  long          frame_ms        = FRAME_DURATION;
  int           count_loop      = 0;
  int           count_loop_dlt  = 2;
  int           count_loop_max  = 359999;
  unsigned long count_frame     = 0;
  int           pad_interval    = (PAD_INTERVAL - 1 > 0) ? PAD_INTERVAL - 1 : 1;
};

struct MrdErr {
  int esp_pc   = 0;
  int pc_esp   = 0;
  int esp_tsy  = 0;
  int tsy_esp  = 0;
  int esp_skip = 0;
  int tsy_skip = 0;
  int pc_skip  = 0;
};

struct PadValue {
  unsigned short stick_R    = 0;
  int            stick_R_x  = 0;
  int            stick_R_y  = 0;
  unsigned short stick_L    = 0;
  int            stick_L_x  = 0;
  int            stick_L_y  = 0;
  unsigned short stick_L2R2V = 0;
  int            R2_val     = 0;
  int            L2_val     = 0;
};

struct AhrsValue {
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
  float yaw_origin  = 0;
  float yaw_source  = 0;
  float read[16];
  float zeros[16]                    = {0};
  float ave_data[16];
  float result[16];
  float stock_data[IMUAHRS_STOCK][16];
  int   stock_count = 0;
  long  temperature;
};

struct MrdMonitor {
  bool flow      = MONITOR_FLOW;
  bool all_err   = MONITOR_ERR_ALL;
  bool servo_err = MONITOR_ERR_SERVO;
  bool seq_num   = MONITOR_SEQ;
  bool pad       = MONITOR_PAD;
};

//------------------------------------------------------------------------------------
//  extern 宣言
//------------------------------------------------------------------------------------

extern MERIDIANFLOW::Meridian mrd;
extern IcsHardSerialClass     ics_L;
extern IcsHardSerialClass     ics_R;
extern TaskHandle_t           thp[4];
extern ServoParam             sv;
extern Meridim90Union         s_udp_meridim;
extern Meridim90Union         r_udp_meridim;
extern Meridim90Union         s_udp_meridim_dummy;
extern MrdFlags               flg;
extern MrdSq                  mrdsq;
extern MrdTimer               tmr;
extern MrdErr                 err;
extern PadUnion               pad_array;
extern PadUnion               pad_i2c;
extern PadValue               pad_analog;
extern AhrsValue              ahrs;
extern MrdMonitor             monitor;

//------------------------------------------------------------------------------------
//  AHRSアクセサ関数（MPU6050依存を避けるため）
//------------------------------------------------------------------------------------

void  mrd_ahrs_set_yaw_origin(float value);
float mrd_ahrs_get_yaw_source();

#endif // __MERIDIAN_TYPES_H__
