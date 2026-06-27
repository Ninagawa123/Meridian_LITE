# Meridim90 EEPROM Map

| word_index | area | variable | upper_8bit | lower_8bit | bit_detail | remarks |
|:---:|:---|:---|:---|:---|:---|:---|
| 0 | ヘッダー：LEN. TYPE | | デフォルト180 | 固定値: 0x55 | uint16_t | LENデフォルト180は、LTVのタイプとしても機能。180以降のタイプ値は予約。0x55はEEPROM初期化識別子を兼ねる |
| 1 | ヘッダー：EEPROMバージョン | | value | value | uint16_t | バージョンはmajor.minor.年月 例:12605 → v1.2.605（2026年5月） |
| 2 | ヘッダー：書き込み回数記録 | | long 1 | long 2 | uint16_t | LTVでも予約領域 |
| 3 | ヘッダー：書き込み回数記録 | | long 3 | long 4 | uint16_t | LTVでも予約領域 |
| 4 | 起動時再生モーション番号 | | (blank) | value | uint8_t / uint8_t | 0:無効, 1-255:番号 |
| 5 | ネットワーク設定 | | 通信プロファイル | 通信モード | uint8_t / uint8_t | 通信プロファイル選択(#0-#2), 通信モード (0:WifiDHCP, 1:固定, 2:有線LAN) |
| 6 | WiFiプロファイル#0 : SSID文字列1 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 7 | WiFiプロファイル#0 : SSID文字列2 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 8 | WiFiプロファイル#0 : SSID文字列3 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 9 | WiFiプロファイル#0 : SSID文字列4 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 10 | WiFiプロファイル#0 : SSID文字列5 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 11 | WiFiプロファイル#0 : SSID文字列6 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 12 | WiFiプロファイル#0 : SSID文字列7 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 13 | WiFiプロファイル#0 : SSID文字列8 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 14 | WiFiプロファイル#0 : SSID文字列9 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 15 | WiFiプロファイル#0 : SSID文字列10 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 16 | WiFiプロファイル#0 : SSID文字列11 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 17 | WiFiプロファイル#0 : SSID文字列12 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 18 | WiFiプロファイル#0 : SSID文字列13 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 19 | WiFiプロファイル#0 : SSID文字列14 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 20 | WiFiプロファイル#0 : SSID文字列15 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 21 | WiFiプロファイル#0 : SSID文字列16 | | ASCII char 1 | ASCII char 2 | char, char | 固定長32文字 |
| 22 | WiFiプロファイル#0 : PASS文字列1 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 23 | WiFiプロファイル#0 : PASS文字列2 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 24 | WiFiプロファイル#0 : PASS文字列3 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 25 | WiFiプロファイル#0 : PASS文字列4 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 26 | WiFiプロファイル#0 : PASS文字列5 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 27 | WiFiプロファイル#0 : PASS文字列6 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 28 | WiFiプロファイル#0 : PASS文字列7 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 29 | WiFiプロファイル#0 : PASS文字列8 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 30 | WiFiプロファイル#0 : PASS文字列9 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 31 | WiFiプロファイル#0 : PASS文字列10 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 32 | WiFiプロファイル#0 : PASS文字列11 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 33 | WiFiプロファイル#0 : PASS文字列12 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 34 | WiFiプロファイル#0 : PASS文字列13 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 35 | WiFiプロファイル#0 : PASS文字列14 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 36 | WiFiプロファイル#0 : PASS文字列15 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 37 | WiFiプロファイル#0 : PASS文字列16 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 38 | WiFiプロファイル#0 : PASS文字列17 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 39 | WiFiプロファイル#0 : PASS文字列18 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 40 | WiFiプロファイル#0 : PASS文字列19 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 41 | WiFiプロファイル#0 : PASS文字列20 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 42 | WiFiプロファイル#0 : PASS文字列21 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 43 | WiFiプロファイル#0 : PASS文字列22 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 44 | WiFiプロファイル#0 : PASS文字列23 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 45 | WiFiプロファイル#0 : PASS文字列24 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 46 | WiFiプロファイル#0 : PASS文字列25 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 47 | WiFiプロファイル#0 : PASS文字列26 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 48 | WiFiプロファイル#0 : PASS文字列27 | | ASCII char 1 | ASCII char 2 | char, char | 固定長64文字 |
| 49 | WiFiプロファイル#0 : PASS文字列28 | | ASCII char 1 | ASCII char 2 | char/char | 固定長64文字 |
| 50 | WiFiプロファイル#0 : PASS文字列29 | | ASCII char 1 | ASCII char 2 | char/char | 固定長64文字 |
| 51 | WiFiプロファイル#0 : PASS文字列30 | | ASCII char 1 | ASCII char 2 | char/char | 固定長64文字 |
| 52 | WiFiプロファイル#0 : PASS文字列31 | | ASCII char 1 | ASCII char 2 | char/char | 固定長64文字 |
| 53 | WiFiプロファイル#0 : PASS文字列32 | | ASCII char 1 | ASCII char 2 | char/char | 固定長64文字 |
| 54 | WiFiプロファイル#0 : 送信先PC IP 文字列1 | | Oct_1 | Oct_2 | uint8_t, uint8_t | |
| 55 | WiFiプロファイル#0 : 送信先PC IP 文字列2 | | Oct_3 | Oct_4 | uint8_t, uint8_t | |
| 56 | WiFiプロファイル#0 : ESP32固定 IP 文字列1 | | Oct_1 | Oct_2 | uint8_t, uint8_t | |
| 57 | WiFiプロファイル#0 : ESP32固定 IP 文字列2 | | Oct_3 | Oct_4 | uint8_t, uint8_t | |
| 58 | WiFiプロファイル#0 : Gateway IP 文字列1 | | Oct_1 | Oct_2 | uint8_t, uint8_t | |
| 59 | WiFiプロファイル#0 : Gateway IP 文字列2 | | Oct_3 | Oct_4 | uint8_t, uint8_t | |
| 60 | WiFiプロファイル#0 : Subnet Mask 文字列1 | | Oct_1 | Oct_2 | uint8_t, uint8_t | |
| 61 | WiFiプロファイル#0 : Subnet Mask 文字列2 | | Oct_3 | Oct_4 | uint8_t, uint8_t | |
| 62 | WiFiプロファイル#0 : UDP SendPort | | value | value | uint16_t | |
| 63 | WiFiプロファイル#0 : UDP RecvPort | | value | value | uint16_t | |
| 64 | UDP待受タイムアウト | UDP_TIMEOUT | (blank) | value | uint8_t, uint8_t | |
| 65 | I2C_0速度（速度/1000） | | value | value | uint16_t | |
| 66 | I2C_1速度（速度/1000） | | value | value | uint16_t | |
| 67 | SPI_0速度（速度/1000） | | value | value | uint16_t | |
| 68 | SPI_1速度（速度/1000） | | value | value | uint16_t | |
| 69 | 起動待ち時間 | CHARGE_TIME | value | value | uint16_t | |
| 70 | SDマウント/起動時SDチェック | | (blank) | bitflag | uint8_t, uint8_t | bit0:マウントenable, bit1:チェックenable |
| 71 | 各種マウント（ビットフラグ） | | bitflag | bitflag | | bit0: 拡張シリアル |
| 72 | 拡張シリアル速度（ボーレート/100） | | value | value | uint16_t | |
| 73 | 拡張シリアルタイムアウト | | value | value | uint16_t | |
| 74 | PCシリアル速度（ボーレート/100） | SERIAL_PC_BPS | value | value | uint16_t | |
| 75 | PCシリアルタイムアウト | SERIAL_PC_TIMEOUT | uint16_t | uint16_t | uint16_t | |
| 76 | サーボプロトコルタイプ（L系, R系） | | L系のタイプ | R系のタイプ | uint8_t, uint8_t | |
| 77 | サーボプロトコルタイプ（C系, X系） | | C系のタイプ | X系のタイプ | uint8_t, uint8_t | |
| 78 | サーボ通信速度 : L系のボーレート(/100) | | value | value | uint16_t | |
| 79 | サーボ通信速度 : R系のボーレート(/100) | | value | value | uint16_t | |
| 80 | サーボ通信速度 : C系のボーレート(/100) | | value | value | uint16_t | |
| 81 | サーボ通信速度 : X系のボーレート(/100) | | value | value | uint16_t | |
| 82 | サーボタイムアウト（L系, R系） | | L系のタイムアウト | R系のタイムアウト | uint8_t, uint8_t | |
| 83 | サーボタイムアウト（C系, X系） | | C系のタイムアウト | X系のタイムアウト | uint8_t, uint8_t | |
| 84 | サーボロスト判定(無返信連続フレーム数) | SV_LOST_ERR_WAIT | (blank) | value | uint8_t, uint8_t | |
| 85 | - | - | - | - | - | |
| 86 | - | - | - | - | - | |
| 87 | - | - | - | - | - | |
| 88 | - | - | - | - | - | |
| 89 | - | - | - | - | - | |
| 90 | 動作モード(スタンドアロンなど) | | bitflag | bitflag | uint16_t | 0:スタンドアロン, 1: Meridan通信 |
| 91 | IMU/AHRSタイプ | | (blank) | value | uint8_t, uint8_t | 0-3 |
| 92 | IMUキャリブレーション値 | | \[Ax\] | \[Ax\] | | |
| 93 | IMUキャリブレーション値 | | \[Ay\] | \[Ay\] | | |
| 94 | IMUキャリブレーション値 | | \[Az\] | \[Az\] | | |
| 95 | IMUキャリブレーション値 | | \[Gx\] | \[Gx\] | | |
| 96 | IMUキャリブレーション値 | | \[Gy\] | \[Gy\] | | |
| 97 | IMUキャリブレーション値 | | \[Gz\] | \[Gz\] | | |
| 98 | IMU読み取り間隔, 移動平均数 | IMUAHRS_INTERVAL, IMUAHRS_STOCK | value | value | uint8_t, uint8_t | |
| 99 | C系サーボ ID0:設定 | | bitflag | bitflag | uint16_t | bit0:マウント有無 / bit1-7:サーボID / bit8:回転方向CW / bit9-15:予約 |
| 100 | C系サーボ ID0:トリム値 | | value | value | uint16_t | トリム値 deg x100 (-18000〜+18000) |
| 101 | C系サーボ ID1:設定 | | bitflag | bitflag | uint16_t | |
| 102 | C系サーボ ID1:トリム値 | | value | value | uint16_t | |
| 103 | C系サーボ ID2:設定 | | bitflag | bitflag | uint16_t | |
| 104 | C系サーボ ID2:トリム値 | | value | value | uint16_t | |
| 105 | C系サーボ ID3:設定 | | bitflag | bitflag | uint16_t | |
| 106 | C系サーボ ID3:トリム値 | | value | value | uint16_t | |
| 107 | C系サーボ ID4:設定 | | bitflag | bitflag | uint16_t | |
| 108 | C系サーボ ID4:トリム値 | | value | value | uint16_t | |
| 109 | 1フレームの時間（単位ms） | | value | value | uint16_t | デフォルト値 10 (100Hz) |
| 110 | L系サーボ ID0:設定 | | bitflag | bitflag | uint16_t | bit0:マウント有無 / bit1-7:サーボID / bit8:回転方向CW / bit9-15:予約 |
| 111 | L系サーボ ID0:トリム値 | | value | value | uint16_t | トリム値 deg x100 (-18000〜+18000) |
| 112 | L系サーボ ID1:設定 | | bitflag | bitflag | uint16_t | |
| 113 | L系サーボ ID1:トリム値 | | value | value | uint16_t | |
| 114 | L系サーボ ID2:設定 | | bitflag | bitflag | uint16_t | |
| 115 | L系サーボ ID2:トリム値 | | value | value | uint16_t | |
| 116 | L系サーボ ID3:設定 | | bitflag | bitflag | uint16_t | |
| 117 | L系サーボ ID3:トリム値 | | value | value | uint16_t | |
| 118 | L系サーボ ID4:設定 | | bitflag | bitflag | uint16_t | |
| 119 | L系サーボ ID4:トリム値 | | value | value | uint16_t | |
| 120 | L系サーボ ID5:設定 | | bitflag | bitflag | uint16_t | |
| 121 | L系サーボ ID5:トリム値 | | value | value | uint16_t | |
| 122 | L系サーボ ID6:設定 | | bitflag | bitflag | uint16_t | |
| 123 | L系サーボ ID6:トリム値 | | value | value | uint16_t | |
| 124 | L系サーボ ID7:設定 | | bitflag | bitflag | uint16_t | |
| 125 | L系サーボ ID7:トリム値 | | value | value | uint16_t | |
| 126 | L系サーボ ID8:設定 | | bitflag | bitflag | uint16_t | |
| 127 | L系サーボ ID8:トリム値 | | value | value | uint16_t | |
| 128 | L系サーボ ID9:設定 | | bitflag | bitflag | uint16_t | |
| 129 | L系サーボ ID9:トリム値 | | value | value | uint16_t | |
| 130 | L系サーボ ID10:設定 | | bitflag | bitflag | uint16_t | |
| 131 | L系サーボ ID10:トリム値 | | value | value | uint16_t | |
| 132 | L系サーボ ID11:設定 | | bitflag | bitflag | uint16_t | |
| 133 | L系サーボ ID11:トリム値 | | value | value | uint16_t | |
| 134 | L系サーボ ID12:設定 | | bitflag | bitflag | uint16_t | |
| 135 | L系サーボ ID12:トリム値 | | value | value | uint16_t | |
| 136 | L系サーボ ID13:設定 | | bitflag | bitflag | uint16_t | |
| 137 | L系サーボ ID13:トリム値 | | value | value | uint16_t | |
| 138 | L系サーボ ID14:設定 | | bitflag | bitflag | uint16_t | |
| 139 | L系サーボ ID14:トリム値 | | value | value | uint16_t | |
| 140 | R系サーボ ID0:設定 | | bitflag | bitflag | uint16_t | |
| 141 | R系サーボ ID0:トリム値 | | value | value | uint16_t | |
| 142 | R系サーボ ID1:設定 | | bitflag | bitflag | uint16_t | |
| 143 | R系サーボ ID1:トリム値 | | value | value | uint16_t | |
| 144 | R系サーボ ID2:設定 | | bitflag | bitflag | uint16_t | |
| 145 | R系サーボ ID2:トリム値 | | value | value | uint16_t | |
| 146 | R系サーボ ID3:設定 | | bitflag | bitflag | uint16_t | |
| 147 | R系サーボ ID3:トリム値 | | value | value | uint16_t | |
| 148 | R系サーボ ID4:設定 | | bitflag | bitflag | uint16_t | |
| 149 | R系サーボ ID4:トリム値 | | value | value | uint16_t | |
| 150 | R系サーボ ID5:設定 | | bitflag | bitflag | uint16_t | |
| 151 | R系サーボ ID5:トリム値 | | value | value | uint16_t | |
| 152 | R系サーボ ID6:設定 | | bitflag | bitflag | uint16_t | |
| 153 | R系サーボ ID6:トリム値 | | value | value | uint16_t | |
| 154 | R系サーボ ID7:設定 | | bitflag | bitflag | uint16_t | |
| 155 | R系サーボ ID7:トリム値 | | value | value | uint16_t | |
| 156 | R系サーボ ID8:設定 | | bitflag | bitflag | uint16_t | |
| 157 | R系サーボ ID8:トリム値 | | value | value | uint16_t | |
| 158 | R系サーボ ID9:設定 | | bitflag | bitflag | uint16_t | |
| 159 | R系サーボ ID9:トリム値 | | value | value | uint16_t | |
| 160 | R系サーボ ID10:設定 | | bitflag | bitflag | uint16_t | |
| 161 | R系サーボ ID10:トリム値 | | value | value | uint16_t | |
| 162 | R系サーボ ID11:設定 | | bitflag | bitflag | uint16_t | |
| 163 | R系サーボ ID11:トリム値 | | value | value | uint16_t | |
| 164 | R系サーボ ID12:設定 | | bitflag | bitflag | uint16_t | |
| 165 | R系サーボ ID12:トリム値 | | value | value | uint16_t | |
| 166 | R系サーボ ID13:設定 | | bitflag | bitflag | uint16_t | |
| 167 | R系サーボ ID13:トリム値 | | value | value | uint16_t | |
| 168 | R系サーボ ID14:設定 | | bitflag | bitflag | uint16_t | |
| 169 | R系サーボ ID14:トリム値 | | value | value | uint16_t | |
| 170 | - | - | - | - | - | |
| 171 | - | - | - | - | - | |
| 172 | - | - | - | - | - | |
| 173 | - | - | - | - | - | |
| 174 | - | - | - | - | - | |
| 175 | リモコンタイプ | | \[MainType\]\[SubType\] | \[Generalize\]\[Merge\] | uint16_t | bit7-10: MainType, bit11-15: SubType, bit0: Generalize, bit1: Merge… |
| 176 | リモコン詳細 | | \[Timeout\] | \[Interval\] | uint8_t, uint8_t | |
| 177 | - | - | - | - | - | |
| 178 | ログレベル/モニタリングフラグ | | value | bitflag | uint8_t, uint8_t | |
| 179 | チェックサム | CRC16(0-178) | - | - | uint16_t | |
| 180〜539 | 可変長LTV領域 | | | | | |
