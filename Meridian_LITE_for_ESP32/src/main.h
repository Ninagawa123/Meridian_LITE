#ifndef __MERIDIAN_LOCAL_FUNC__
#define __MERIDIAN_LOCAL_FUNC__

#include <cstdint>
#include <string>

// +-------------------------------------------------------------------
// | 関数名　　:  setyawcenter()
// +-------------------------------------------------------------------
// | 機能     :  センサーのヨー軸中央値を再セットする
// +------------------------------------------------------------------
void setyawcenter();

// +----------------------------------------------------------------------
// | 関数名　　:  receiveUDP()
// +----------------------------------------------------------------------
// | 機能     :  UDP通信の受信パケットを確認し、
// | 　　        受信していたら共用体r_udp_meridimに値を格納する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void receiveUDP();

// +----------------------------------------------------------------------
// | 関数名　　:  sendUDP()
// +----------------------------------------------------------------------
// | 機能     :  共用体s_udp_meridimをUDP通信で送信する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void sendUDP();

// +----------------------------------------------------------------------
// | 関数名　　:  joypad_read()
// +----------------------------------------------------------------------
// | 機能     :  リモコンの入力データを受け取り, PS2/3リモコン配列形式で返す
// | 引数１　　:  int. リモコンの種類(現在は2:KRC-5FHのみ)
// | 引数２　　:  uint64_t. 前回の受信値(8バイト分, 共用体データを想定)
// | 引数３　　:  bool. JOYPADの受信ボタンデータをこのデバイスで0リセットするか、
// | 　　　　　　　リセットせず論理加算するか （0:overide, 1:reflesh, 通常は1）
// | 戻値　　　:  uint64_t. リモコン受信値を格納
// +----------------------------------------------------------------------
uint64_t joypad_read(int mount_joypad, uint64_t pre_val, int polling, bool joypad_reflesh);

// +----------------------------------------------------------------------
// | 関数名　　:  monitor_joypad(ushort *arr)
// +----------------------------------------------------------------------
// | 機能     :  リモコンの入力データを表示する.
// | 引数　　　:  ushort配列 4個.
// +----------------------------------------------------------------------
void monitor_joypad(ushort *arr);

// +----------------------------------------------------------------------
// | 関数名　　:  check_sd()
// +----------------------------------------------------------------------
// | 機能     :  SDカードの初期化と読み書きテスト
// +----------------------------------------------------------------------
void check_sd();

// +----------------------------------------------------------------------
// | 関数名　　:  init_wifi()
// +----------------------------------------------------------------------
// | 機能     :  wifiの初期化
// +----------------------------------------------------------------------
void init_wifi(const char *wifi_ap_ssid, const char *wifi_ap_pass);

// +----------------------------------------------------------------------
// | 関数名　　:  init_imuahrs()
// +----------------------------------------------------------------------
// | 機能     :  センサーの初期化(BNO055のみ)
// +----------------------------------------------------------------------
void init_imuahrs(int mount_imuahrs);

// +----------------------------------------------------------------------
// | スレッド用関数　:  Core1_bno055_r(void *args)
// +----------------------------------------------------------------------
// | 機能    　　　 :  bno055のデータを取得し、bno055_read[]に書き込む
// +----------------------------------------------------------------------
void Core1_bno055_r(void *args);

#endif