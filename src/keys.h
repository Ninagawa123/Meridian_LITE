// Wifiアクセスポイントの設定
#define WIFI_AP_SSID "xxxxxx"        // アクセスポイントのWIFI_AP_SSID
#define WIFI_AP_PASS "xxxxxx"        // アクセスポイントのパスワード
#define WIFI_SEND_IP "192.168.1.xx"  // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define UDP_SEND_PORT 22222          // 送り先のポート番号
#define UDP_RESV_PORT 22224          // このESP32のポート番号

// ESP32のIPアドレスを固定する場合は下記の5項目を設定
#define FIXED_IP_ADDR "192. 168. 1. xx"    // ESP32のIPアドレスを固定する場合のESPのIPアドレス
#define FIXED_IP_GATEWAY "192. 168. 1. xx" // ESP32のIPアドレスを固定する場合のルーターのゲートウェイ
#define FIXED_IP_SUBNET "255. 255. 255. 0" // ESP32のIPアドレスを固定する場合のサブネット

// リモコンの設定
#define BT_MAC_ADDR "xx:xx:xx:xx:xx:xx" // ESP32自身のBluetoothMACアドレス（本プログラムを実行しシリアルモニタで確認）
                                        // 現在は不使用.
