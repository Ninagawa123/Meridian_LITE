# Meridian_LITE 1.2.0 仕様書

| 項目 | 内容 |
|---|---|
| 対象 | Meridian_LITE_for_ESP32 (ESP32-DevKitC + Meridian Board LITE) |
| ファームウェア版 | `VERSION "Meridian_LITE_v1.2.0_2026_10.09"` (main.cpp) |
| 文書の版 | 1.29 (2026-10-21). 改訂履歴は24章 |
| 根拠 | `src/` のソースコード. 本書とコードが食い違う場合はコードが正 |
| 関連文書 | `CHANGES_servo_if.md` (改修の記録), `tasks/servo_param_plan.md` (パラメータ系コマンドの設計), `Meridian_LITE_config.html` (設定画面), `tests/` (PC で実行するテスト. 21章), リポジトリ直下の `README.md` (導入手順) |

本書は, Meridian_LITE のファームウェアが「何を, どの順で, どの値で」行うかを体系的にまとめたものである.
章は「全体 → ハードウェア → ソフトウェアの構成 → 動作 (起動, 1フレーム) → 通信と PC との取り決め → データ (Meridim90, コマンド) → 機能別 (サーボ, センサ, パッド, 記憶, 表示) → 設定値・データ構造の一覧 → 制約と不具合」の順に並ぶ. 関数の一覧は付録A.

### 記述の区分

本書の記述は次の4つに分ける. 印のない記述は「仕様」である.

| 印 | 区分 | 意味 |
|---|---|---|
| (なし) | 仕様 | ファームウェアが保証する動作. PC 側のプログラムや利用者はこれに依存してよい |
| 〔制約 K-n〕 | 実装上の制約 | 現在の実装はそう動くが, 仕様としては保証しない. 将来変わることがある |
| 〔不具合 B-n〕 | 既知の不具合 | 意図と違う動作. 修正する予定があるため, これに依存してはいけない |
| 〔未検証〕 | 実機で未確認 | コードとホストテストでは確認したが, 実機では確認していない |

K-n と B-n の番号は22章の一覧の番号である. 本文の印から22章を引き, 22章から本文の節を引ける.

## 目次

1. [概要](#1-概要)
2. [ハードウェア](#2-ハードウェア)
3. [ソフトウェアの構成](#3-ソフトウェアの構成)
4. [起動シーケンス](#4-起動シーケンス)
5. [メインループ (1フレームの処理)](#5-メインループ-1フレームの処理)
6. [スレッドと排他](#6-スレッドと排他)
7. [通信 (UDP, WiFi, 有線LAN)](#7-通信-udp-wifi-有線lan)
8. [Meridim90 配列](#8-meridim90-配列)
9. [マスターコマンド](#9-マスターコマンド)
10. [サーボ: 共通仕様](#10-サーボ-共通仕様)
11. [サーボ: プロトコル別仕様](#11-サーボ-プロトコル別仕様)
12. [パラメータ系コマンドと車輪](#12-パラメータ系コマンドと車輪)
13. [PWMサーボ (本体ピン, PCA9685)](#13-pwmサーボ-本体ピン-pca9685)
14. [IMU / AHRS](#14-imu--ahrs)
15. [ジョイパッド](#15-ジョイパッド)
16. [EEPROM と SDカード](#16-eeprom-と-sdカード)
17. [シリアルモニタの表示](#17-シリアルモニタの表示)
18. [設定画面 (Meridian_LITE_config.html)](#18-設定画面-meridian_lite_confightml)
19. [設定値一覧 (config.h / keys.h)](#19-設定値一覧-configh--keysh)
20. [データ構造 (列挙型, 構造体, グローバル変数, 定数)](#20-データ構造-列挙型-構造体-グローバル変数-定数)
21. [検証方法](#21-検証方法)
22. [既知の制約と不具合](#22-既知の制約と不具合)
23. [用語](#23-用語)
24. [改訂履歴](#24-改訂履歴)
- [付録A. 関数一覧](#付録a-関数一覧)

---

## 1. 概要

### 1.1 システムの役割

Meridian は, ロボットとPC(ROS, Unity, Python等)の間で状態データを高速に循環させ, デジタルツインを実現する仕組みである.
Meridian_LITE は ESP32 に書き込むファームウェアで, ロボットに搭載して次を行う.

- PC から UDP で受け取った **Meridim90 配列** (90個の16bit整数) に従ってサーボを動かす.
- サーボの現在位置, IMU/AHRS の値, ジョイパッドの値を Meridim90 に書き込み, PC に UDP で返す.
- これを **10ms (100Hz) 周期**で繰り返す. 周期はハードウェアタイマーで管理する.

```
  PC (Meridian_console, ROS, Unity 等)
        ▲ UDP 180byte (Meridim90)        │ UDP 180byte
        │ ESP32 → PC:22222               ▼ PC → ESP32:22224
  ┌──────────────────────── ESP32 (Meridian_LITE) ────────────────────────┐
  │ Core1: loop() 100Hz  送信 → 受信 → コマンド → センサ/パッド → 目標計算 │
  │                      → サーボ通信(L/R並行) → 返信値 → 送信配列作成    │
  │ Core0: IMU/AHRSスレッド, WIIMOTE/BlueRetroスレッド, WiFi/BTスタック     │
  └───────┬──────────────┬─────────────┬───────────────┬───────────────────┘
          │Serial1 (L系統) │Serial2 (R系統)│I2C0           │SPI
          ▼              ▼             ▼               ▼
     コマンドサーボ    コマンドサーボ   BNO055/MPU6050   SDカード / W5500
```

### 1.2 主な機能

| 機能 | 内容 | 章 |
|---|---|---|
| 通信 | WiFi (STA) または有線LAN (W5500) で UDP. ボード主導で毎フレーム送信 | 7 |
| サーボ | 2系統 (L/R) の半二重UART. 系統ごとに1種類 (一部は2種類まで混在). L/R は並行に送受信 | 10, 11 |
| サーボの種類 | KONDO ICS3.5/3.6・PMX, DYNAMIXEL 1.0/2.0, FUTABA RS30x, FEETECH STS/SCS, JR XBUS, Hitec HSR, PWM (本体ピン・PCA9685) | 11, 13 |
| サーボの拡張コマンド | パラメータ系コマンド (読み書き・ROM保存), 車輪, スルー, Meridim[19] による補間 | 10, 12 |
| センサ | BNO055 (9軸AHRS), MPU6050 (6軸, DMP) | 14 |
| ジョイパッド | KRC-5FH (ICS経由), WIIMOTE + ヌンチャク / クラシックコントローラ (Bluetooth), SBDBT (RCB3形式. `PIN_PAD_RX` でソフトウェア受信), BlueRetro (Wii extension. I2C0 か I2C1) | 15 |
| 設定の記憶 | EEPROM (config.h・keys.h の設定を16ページに. ページごとに config と EEPROM のどちらの値を使うか選べる), SDカード (起動時の読み書き確認のみ) | 16 |
| 表示 | 起動時メッセージ, 処理時間・通信失敗のモニタ, 各種デバッグ表示 | 17 |
| 設定画面 | ブラウザで config.h / keys.h を編集 | 18 |

### 1.3 基本の数値

| 項目 | 値 | 設定 |
|---|---|---|
| フレーム周期 | 10ms (100Hz) | `FRAME_DURATION` |
| Meridim の長さ | 90 要素 (180byte) | `MRDM_LEN` |
| サーボ | 2系統 × 15 スロット | `MRD_SERVO_SLOTS`, `IXL_MAX`, `IXR_MAX` |
| UDP の受信待ち | 最大 4ms | `UDP_TIMEOUT_US` |
| 受信が途絶えたら全サーボ脱力 | 1秒 | `UDP_FAILSAFE_MS` |
| PC との USB シリアル | 921600bps | `SERIAL_PC_BPS` |
| ポート | ESP32 → PC: 22222, PC → ESP32: 22224 | `UDP_SEND_PORT`, `UDP_RECV_PORT` (keys.h) |

### 1.4 互換性 (他の Meridian との関係)

| 項目 | 内容 |
|---|---|
| Meridim90 の形 | Meridian 標準と同じ (90要素, [1] 連番, [88] エラー, [89] チェックサム) |
| マスターコマンド | 標準の番号を使う. LITE で動作があるのは9章の表で「動作」を書いたものだけ. 未実装の番号は通常動作として扱う |
| LITE 独自の拡張 | スロットのコマンドの bit13 (車輪), bit14 (結果のエラー), bit15 (スルー), 下位8bit が3以上のパラメータ系コマンド (12章), Meridim[19] の補間フレーム数 (10.7節). 受信が途絶えたときの脱力 (7.5節) |
| 拡張を使わない PC | スロットのコマンドに 0 / 1 だけを使い, [19] を 0 にすれば, 拡張のない Meridian と同じに動く |
| エラービット | LITE が設定するのは bit14 と bit10 だけ (8.3節). 他のビットは PC が送った値が返る |
| Meridian_console | 表示とトリム設定 (10100〜10102, 10200〜10202) に対応する. LITE 独自の拡張 (EEPROM のページ 10203〜10215 / 10300〜10315, 再起動 10400 を含む) を操作する画面はない |

---

## 2. ハードウェア

### 2.1 ボードの版 (`MRD_BOARD_SERVO_IF`)

サーボIF回路の違いを `MRD_BOARD_SERVO_IF` で選ぶ. 既定は 1.

| 値 | 基板 | R系統 TX / RX | 送信エコー (`SERVO_ECHO_L/R` の既定) | 受信中のTX出力OFF (`SERVO_TX_RELEASE`) |
|---|---|---|---|---|
| 0 | 無印〜L1.43未満 (3ステートバッファ版) | 17 / 16 | なし (0) | しない (0) |
| 1 | Ver L1.43以上 (SN74LXC1T45版) | 14 / 13 | あり (1) | する (1) |

- SN74LXC1T45 版は ESP32 の TX と RX がともに 1T45 の A 端子に直結しているため, 送信したバイトが RX にエコーされる. 受信中は TX ピンの出力を切る (10.2節).
- L系統のピンは両版で共通.
- ESP32-WROVER 版の DevKitC は GPIO16/17 を使えないため, 0 の基板では配線の変更が必要 (README).

### 2.2 ピン割り当て

| 定義 | GPIO | 機能 |
|---|---|---|
| `PIN_EN_L` | 33 | L系統の送受信切替 (DIR. H: 送信, L: 受信) |
| `PIN_TX_L` / `PIN_RX_L` | 27 / 32 | L系統 TX / RX (Serial1) |
| `PIN_EN_R` | 4 | R系統の送受信切替 |
| `PIN_TX_R` / `PIN_RX_R` | 14 / 13 (版0: 17 / 16) | R系統 TX / RX (Serial2) |
| `PIN_I2C0_SDA` / `PIN_I2C0_SCL` | 22 / 21 | I2C0 (Wire): BNO055 / MPU6050 |
| `PIN_CHIPSELECT_SD` | 15 | SDカード CS (起動時に LOW であること. 外部プルアップ禁止) |
| `PIN_CHIPSELECT_LAN` | 5 | 有線LAN (W5500) CS |
| `PIN_RESET_LAN` | 14 | W5500 リセット. **未使用** (版1では `PIN_TX_R` と重なる) |
| `PIN_ERR_LED` | 25 (−1 = なし. そのピンを本体PWM・I2C1・SBDBT の受信に使うときもなし) | エラーLED. 設定エラー等で停止したときに点滅 |
| `PIN_LED_BT` | 26 (同上) | Bluetooth LED. 起動時に点灯. WIIMOTE のときは接続待ちで点滅し, 接続で点灯, タイムアウトで消灯. 他のパッドでは点灯したまま |
| `PIN_PWM_0` / `PIN_PWM_1` | 25 / 26 | 本体PWMサーボ (`PWM_PIN_MODE` 1) |
| `PIN_I2C1_SDA` / `PIN_I2C1_SCL` | 25 / 26 | PCA9685 用 I2C1 (Wire1) (`PWM_PIN_MODE` 2) |
| `PIN_PAD_RX` | 34 | SBDBT の TX をつなぐ (`MOUNT_PAD` SBDBT. ソフトウェアで受信. 入力専用ピン) |
| SPI (固定) | MISO 19, MOSI 23, SCK 18 | SDカード, W5500 (Arduino の既定 VSPI) |

- ピン 25/26 の用途は `PWM_PIN_MODE` で切り替える: 0 = LED (従来), 1 = 本体PWMサーボ2ch, 2 = PCA9685 用 I2C. BlueRetro (`MOUNT_PAD` BLUERETRO) も `PAD_BLUERETRO_I2C` 1 (既定) なら I2C1 を使う.
- LED のピンは `PIN_ERR_LED` / `PIN_LED_BT` (既定 25/26) で変えられる. そのピンを本体PWM (`PIN_PWM_0/1`), I2C1 (`PIN_I2C1_SDA/SCL`. PCA9685 か BlueRetro のとき), SBDBT の受信 (`PIN_PAD_RX`) に使うときは, その LED はなし (`mrd_pin_used_for_other`). 既定のピンのまま 1/2 や BlueRetro (`PAD_BLUERETRO_I2C` 1) を使うと2つの LED はなくなる.
- ピン番号が負の LED は `mrd_led_write()` が何もしない.
- 基板上のボタン入力はない.

### 2.3 通信路の割り当て

| 通信路 | ペリフェラル | 用途 |
|---|---|---|
| UART0 (`Serial`, USB) | 921600bps | PC のシリアルモニタ |
| UART1 (`Serial1`) | `sv_bus_l` | サーボ L系統 |
| UART2 (`Serial2`) | `sv_bus_r` | サーボ R系統 |
| I2C0 (`Wire`) | 400kHz (`I2C0_SPEED`) | BNO055 (アドレス 0x28) / MPU6050. BlueRetro (0x52) を `PAD_BLUERETRO_I2C` 0 でつないだとき. INA226 (0x40〜0x4F, `MOUNT_INA226` 1. 14.1節) |
| I2C1 (`Wire1`) | 400kHz (`I2C1_SPEED`) | PCA9685 ×2 (0x40, 0x41. `PWM_PIN_MODE` 2 のとき) と BlueRetro (0x52. `MOUNT_PAD` BLUERETRO のとき). ピンは `PIN_I2C1_SDA/SCL` (既定 25/26) |
| SPI (VSPI) | SD: 4MHz (ライブラリ既定), W5500: ライブラリ既定 | SDカード, 有線LAN |
| Bluetooth Classic | — | WIIMOTE |
| WiFi | STA | UDP 通信 (`MODE_ETHER` 0 のとき) |

`SPI0_SPEED` (6MHz) は起動メッセージに表示するだけで, SPI の設定には使われていない.

---

## 3. ソフトウェアの構成

### 3.1 ビルド環境 (platformio.ini)

| 項目 | 値 |
|---|---|
| platform / board | espressif32 (版の指定なし. 確認時 6.12.0) / esp32dev |
| framework | arduino (確認時 Arduino-ESP32 2.0.17 系) |
| C++ 規格 | gnu++11 (フレームワークの既定. 集成体に既定のメンバ初期化子を書けない点に注意), 最適化 -Os (同) |
| build_type / build_flags | release / `-D CORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_NONE` |
| monitor_speed | 921600 (`SERIAL_PC_BPS` と合わせる) |
| パーティション | no_ota.csv (アプリ 2MB) |
| ビルド | `Meridian_LITE_for_ESP32` で `pio run` |

ライブラリ:

| ライブラリ | 版 | 用途 |
|---|---|---|
| ninagawa123/Meridian | 0.1.0 | チェックサム, シーケンス番号, `float2HfShort` (float → ×100 の short), ICS の角度変換 |
| adafruit/Adafruit BNO055 | 1.6.4 | BNO055 |
| electroniccats/MPU6050 | 0.5.0 | MPU6050 (DMP, `MPU6050_6Axis_MotionApps20.h`) |
| arduino-libraries/Ethernet | 2.0.2 | W5500 |
| lib/IcsClass_V210 (同梱) | 2.1 | KONDO ICS (`IcsBaseClass`). KRR-5FH 受信機の読み取りにも使う |
| lib/ESP32Wiimote (同梱) | 0.1.3 | WIIMOTE. LITE で追加: 接続状態 (`connected`), クラシックコントローラの識別と拡張の6byte (`classicConnected`, `getExtensionData`). 追加した箇所には "Meridian:" のコメント |
| lib/gs2d (同梱) | — | **未使用** (パラメータ番号の準拠先として参照のみ) |
| フレームワーク同梱 | — | WiFi, WiFiUdp, Wire, SPI, SD, EEPROM, ESP-IDF の gpio/uart HAL |

版は確認時にインストールされていたもの. platformio.ini では platform の版を指定せず, ライブラリは `^` の範囲指定なので, 新しい版が入ることがある.

### 3.2 ファイル構成

すべてのヘッダが main.cpp に展開される **単一の翻訳単位** で構成している (ヘッダに定義を書く).

| ファイル | 役割 |
|---|---|
| `main.cpp` | `setup()`, `loop()`. フレームタイマー, 起動・1フレームの各段階の関数, サーボのバス・プロトコル層の実体 |
| `main.h` | 列挙型, Meridim90 の共用体, フラグ・カウンタ・サーボ値等の構造体とその実体 |
| `config.h` | すべての設定 (`#define`), サーボの配列設定, マスターコマンド, Meridim のキー, エラービット |
| `keys.h` | WiFi・IP・ポート・有線LANの設定 |
| `mrd_servo.h` | 半二重UART (`MrdHalfDuplexUart`), 待たない送受信, 系統ステッパー, L/R 並行実行, 混在, パラメータ系コマンドの状態, 補間, 返信. プロトコル層を読み込む |
| `mrd_module/sv_util.h` | プロトコル層の共通部品 (角度換算, CRC16, 起動表示, 系統の状態, PC値の換算, パラメータ系コマンドの枠組み) |
| `mrd_module/sv_ics.h` | KONDO ICS 3.5/3.6 |
| `mrd_module/sv_pmx.h` | KONDO PMX |
| `mrd_module/sv_dxl1.h` | DYNAMIXEL Protocol 1.0 |
| `mrd_module/sv_dxl2.h` | DYNAMIXEL Protocol 2.0 |
| `mrd_module/sv_ftbrx.h` | FUTABA RS30x (TTL コマンド方式) |
| `mrd_module/sv_ftc.h` | FEETECH STS / SCS |
| `mrd_module/sv_hitec.h` | Hitec HSR (HMI シリアル) |
| `mrd_module/sv_xbus.h` | JR PROPO XBUS |
| `mrd_module/sv_pwm.h` | PWM サーボ (本体ピン / PCA9685) |
| `mrd_module/mv_firstIK.h`, `mv_motionplay.h` | 将来用のひな形 (どこからも読み込まれていない) |
| `mrd_command.h` | マスターコマンド (第1〜3群) |
| `mrd_wire0.h` | I2C0, BNO055 / MPU6050 の初期化と読み取りスレッド, Meridim への書き込み |
| `mrd_bt_pad.h` | ジョイパッド (KRC-5FH, WIIMOTE/WIIMOTE_C, SBDBT, BlueRetro), WIIMOTE・BlueRetro のスレッド, SBDBT の受信割り込み |
| `mrd_eeprom.h` | EEPROM の形式・作成・読み書き・表示 |
| `mrd_sd.h` | SDカードの初期化と読み書き確認 |
| `mrd_wifi.h` | WiFi の接続と UDP 送受信 |
| `mrd_ether.h` | 有線LAN (W5500) の設定解析・接続・UDP 送受信 |
| `mrd_disp.h` | シリアル表示のクラス `MrdMsgHandler` (`mrd_disp`) |
| `mrd_util.h` | 汎用関数 (ビット操作, 表示整形, 停止, 変化量制限) |
| `mrd_move.h` | 将来用のひな形 (未使用) |
| `config.h.bak`, `config.h.ics_bak` | 設定画面が保存のときに作る控えと, 作業時の控え. ビルドには使わない |

ファイル間の読み込み: main.cpp → config.h → keys.h → main.h (→ ライブラリ → mrd_disp.h → mrd_util.h) → mrd_bt_pad.h (→ mrd_servo.h → mrd_module/sv_*.h) → mrd_command.h (→ mrd_eeprom.h) → その他の mrd_*.h.
各 sv_*.h は必要な共通部品 (`sv_util.h`, DXL1 は `sv_ftc.h` も) を自分で読み込むため, 読み込み順に依存しない.

### 3.3 コーディング規則 (.clang-format)

| 規則 | 例 |
|---|---|
| 書式は LLVM 準拠 (ColumnLimit 0, 末尾コメント・マクロの整列) | VSCode 同梱の clang-format で整形 |
| 変数・関数はスネークケース. Meridian 独自の関数は `mrd_` で始める | `mrd_wire0_init` |
| 名前の並びは 大カテゴリ > 中カテゴリ > 内容 > 動詞 > 補足 | `mrd_servo_line_begin`, `mrd_eeprom_servosettings_load` |
| クラス・構造体・列挙型はパスカルケース | `MrdHalfDuplexUart`, `EepromUnion` |
| 関数の引数は `a_`, クラスのメンバ変数は `m_`, 関数内の補助変数は `_tmp` | `a_meridim`, `m_ln`, `count_tmp` |
| struct のフィールドには `m_` を付けない | `sv.ixl_tgt`, `flg.meridim_rcvd` |
| 説明は `/// @brief`, `/// @param`, `/// @return`. 変数は同じ行か1つ上の行に `//` | |
| 定数の接頭辞 | `MODE_` 動作モード, `EEPROM_`, `CHECK_` 起動時チェック, `MONITOR_` 表示, `MOUNT_` 搭載, `PAD_`, `SERVO_`, `MCMD_` マスターコマンド, `PIN_`, `IXL_`/`IXR_` 系統のサーボ設定, `MRD_` Meridim のキー |

---

## 4. 起動シーケンス

`setup()` は次の順に実行する. 失敗しても止まらない項目と, 停止する項目 (`mrd_error_stop`: メッセージ表示後にエラーLEDを250ms間隔で点滅し無限ループ) がある. エラーLED のピンを他の用途 (本体PWM・I2C1・SBDBT の受信) に使うときはエラーLED がないため, 停止はシリアル表示でしか分からない 〔制約 K-12〕.

| # | 処理 | 内容 | 失敗時 |
|---|---|---|---|
| 0 | 設定値 | `mrd_settings_setup()`: EEPROM を開始し, `EEPROM_LOAD` で選んだページは EEPROM の値を使う. `EEPROM_SET` 1 なら他のページを config の値で書く. 決めた値をフラグ・サーボのバスに反映 (16.4節) | EEPROM を使えなければ config の値 |
| 1 | BT LED | `PIN_LED_BT` を出力にして点灯 (WIIMOTE 以外では点灯したまま) | — |
| 2 | USB シリアル | `Serial.begin(SERIAL_PC_BPS)`. 接続は待たない (ESP32 ではシリアルの接続を検出できない. `SERIAL_PC_TIMEOUT` は未使用) | 続行 |
| 3 | エラーLED | `PIN_ERR_LED` を出力に | — |
| 4 | 充電待ち | "Charging the capacitor." を表示し `CHARGE_TIME` (200ms) 待つ | — |
| 5 | 起動メッセージ | 版, USB/SPI/I2C の速度 | — |
| 6 | サーボ | `mrd_servo_setup()`: (1) `sv.num_max` = L/R の `IXx_MT` で0以外が入っている最大の添字+1 の大きい方 (すべて0なら1). (2) `i < num_max` のスロットに config の種類・ID・方向・トリムを写し, マウント = (種類 ≠ 0 かつ PWM 専用 (1/11) でなく, EEPROM の P1 でマウントを外していない). (3) 系統ごとに `mrd_servo_line_begin()` (混在の検査 → 1種類目の起動 → 混在なら2種類目). (4) プロトコル名・エコーの表示. (5) `mrd_pwm_begin()`. (6) マウントしたサーボの ID 表示. 続けて `mrd_servo_link_setup()` でリンク表を検査して表示 (10.14節) | 応答しないサーボがあっても続行 (11章). 使えないリンクの行は警告して使わない |
| 7 | EEPROM の表示 | どのページが EEPROM の値か, 書き込みの結果, 範囲外の値の数 (`mrd_eeprom_boot_report`). `EEPROM_DUMP` 1 なら内容 (16.4節). #6 の前に行う | — |
| 8 | SDカード | `mrd_sd_init()`, `mrd_sd_check()` (16章) | 続行 |
| 9 | I2C / IMU | `mrd_wire0_setup(MOUNT_IMUAHRS, ...)` (14章) | 続行 (スレッドを起動しない) |
| 10 | センサスレッド | 初期化に成功したときだけ, BNO055 → `mrd_wire0_bno055_task`, MPU6050 → `mrd_wire0_mpu6050_task` を Core0 で開始. `MOUNT_INA226` 1 なら `mrd_ina226_task` も開始 (14.1節) | — |
| 11 | ネットワーク | `mrd_network_setup()` (7章) | WiFi のタイムアウトは続行. IP・MAC の設定誤りと有線LANの初期化失敗は停止 |
| 12 | パッド | 種類を表示. BLUERETRO なら `mrd_blueretro_setup()` で I2C を開始し `mrd_blueretro_task` を Core0 で開始 (I2C1 のピンを本体PWM に使うときは開始しない). SBDBT なら `mrd_sbdbt_begin()` で `PIN_PAD_RX` の変化の割り込みを登録. WIIMOTE / WIIMOTE_C なら `mrd_bt_setup()` で接続を待ち (最大 `PAD_INIT_TIMEOUT` 10s), 接続の成否に関係なく `mrd_bt_wiimote_task` を Core0 で開始 | 続行 |
| 13 | 最初の送信データ | `s_udp_meridim` / `r_udp_meridim` の [0] を 90 にしてチェックサムを入れる (他は0). `r_udp_meridim_last` にも同じダミーを入れる | — |
| 14 | フレームタイマー | ハードウェアタイマー0 を 80分周 (1µs) で, `FRAME_DURATION`×1000 µs ごとに割り込み (`mrd_frame_timer_isr`) | — |
| 15 | 開始メッセージ | "-) Meridian -LITE- system on ESP32 now flows. (-". 単体モードなら注意を表示 | — |
| 16 | カウンタ | `count_frame = 0`, `count_timer = 0`. フェイルセーフの基準時刻 `udp_valid_ms` を今の時刻にする (7.5節) | — |

注: `MODE_ESP32_STANDALONE` (ボード単体モード) でもサーボの起動 (#6) は行う.
注: 表の番号は従来の順のまま. 実際の順は #0 → #1〜#5 → #7 → #6 → #8 以降. EEPROM の値は #0 で決まるので, サーボの起動 (#6) は EEPROM の ID・マウント・通信速度で行う.
注: 起動してから PC の最初のパケットを受けるまでは, #13 のダミー (スロットのコマンドがすべて 0) を受信データとして使うため, サーボは脱力のまま (7.5節). `MODE_UDP_RECEIVE` 0 ではダミーの代わりにボードが指令を作る (5.3節).

---

## 5. メインループ (1フレームの処理)

### 5.1 フレームの時間管理

- 割り込み `mrd_frame_timer_isr()` が `FRAME_DURATION` (10ms) ごとに `count_timer` を1増やし, セマフォを与える.
- **ボード主導** (`MODE_UDP_PASSIVE` 0. 既定): `loop()` の最後で `count_frame` を1増やし, `mrd_frame_next_wait()` で `count_timer >= count_frame` になるまでセマフォを待つ (CPU を他のタスクに譲る).
- **PC 主導** (`MODE_UDP_PASSIVE` 1 または 10006): タイマーを待たない. 受信を最大1フレーム (`FRAME_DURATION`) 待ち, 処理の直後にこのフレームの結果を送る ([13-2]). 周期は PC の送信で決まる. `count_frame` は毎回 `count_timer` に合わせ (`mrd_frame_resync()`), ボード主導に戻したとき遅れや先行が残らない. `MODE_UDP_RECEIVE` 0 のときは常にボード主導 (`mrd_udp_passive()`).
- 処理が遅れた場合は待たずに次のフレームへ進み, 遅れを連続実行で取り戻す (フレームを捨てない).
- `flg.count_frame_reset` が立っていれば, 待ちの後に `count_frame = count_timer` として遅れの累積を捨てる (コマンド 10005〜10009, 10102 で立つ). PC 主導では毎回合わせるので, フラグは下ろすだけ.
- 遅延の判定 `mrd_frame_is_late()` (処理の終わりにタイマーが次のフレームに達しているか) は表示 (`MONITOR_FRAME_DELAY`) にだけ使う. Meridim のエラービットには反映しない 〔制約 K-16〕. 時間の予算は10.13節.

### 5.2 処理の流れ

| 段階 | 処理 | 関数 | 備考 |
|---|---|---|---|
| [1] | UDP 送信 | `mrd_udp_send()` | **前のフレームの [12] で作った** `s_udp_meridim` を送る. `MODE_UDP_SEND` 0 なら送らない. PC 主導では送らない ([13-2] で送る) |
| [2-1] | UDP 受信待ち | `mrd_udp_receive_wait()` | 最大 `mrd_udp_wait_us()` (ボード主導は `UDP_TIMEOUT_US` 4ms, PC 主導は `FRAME_DURATION` 10ms), `UDP_POLL_US` (100µs) ごとに確認. 溜まっているパケットは最新の1つを使う (7.2節). `MODE_UDP_RECEIVE` 0 なら待たず, 受信データの代わりにボードが指令を作る (`mrd_meridim_self_make()`. 5.3節) |
| [2-2]〜[2-6] | 受信の確認 | `mrd_meridim_received_check()` | チェックサム (正しければ `r_udp_meridim_last` に保持) → 最後に正しく受信したパケットを送信配列へ写す → エラービット → シーケンス番号 (8.5節). 正しいパケット (届いて, チェックサムが正しい) だったかを返す |
| [2-7] | フェイルセーフ | `mrd_udp_failsafe()` | 正しいパケットが `UDP_FAILSAFE_MS` (1秒) 続けて届かなければ, 全スロットのコマンドを 0 にする (7.5節) |
| [3] | マスターコマンド第1群 | `mrd_command_group1_execute()` | 9章 |
| [4] | センサ値の書き込み | `mrd_meriput90_ahrs()`, `mrd_meriput90_ina226()` | Meridim[2]〜[14]. `MOUNT_INA226` 1 なら [86] [87] (14.1節) |
| [5] | パッド値の書き込み | `mrd_pad_read()`, `mrd_meriput90_pad()` | `MOUNT_PAD` > 0 のとき. Meridim[15]〜[18] |
| [6] | マスターコマンド第2群 | `mrd_command_group2_execute()` | 9章 |
| [7-1] | 目標値の計算 | `mrd_servo_targets_compute()` | 補間と変化量の制限 (10.6節) |
| [7-2] | ESP32 内の動作計算 | (サンプル) | Meridim[15] がちょうど `PAD_RIGHT` / `PAD_LEFT` のとき (他のボタンと同時では動かない. PC が送ったボタン値でも動く), L00 の目標を −30°/+30° に. 制限の基準は前回の現在位置 (`ixl_tgt_past[0]`) で, `SERVO_STEP_MAX` で制限. 既定で有効 〔制約 K-11〕 |
| [7-3] | 受信しないモードの速さの制限 | `mrd_self_speed_limit()` | `MODE_UDP_RECEIVE` 0 のトルクON後だけ. 指令値の1フレームの変化を, 前のフレームに送った指令値から `UDP_SELF_SPEED` × `FRAME_DURATION` 以内にする (5.3節) |
| [7-4] | サーボのリンク | `mrd_servo_link_apply()` | `SERVO_LINK` の行ごとに, フォロワーのスロットにリーダーの指令をコピーする (10.14節) |
| [8-1] | サーボの送受信 | `mrd_servo_drive_lite()` | L/R 並行. 単体モードでは送受信せず (PWM も出力しない), L00 に ±30° のサイン波 (周期1.8s). 他のスロットは [7-1] の指令値を現在位置として返す |
| [9-1] | サーボの返信値 | `mrd_servo_values_reply()` | 全30スロットについて `mrd_servo_slot_reply()` (10.5節) |
| [10-1] | エラー表示 | `mrd_disp.all_err()` | `MONITOR_ERR_ALL` |
| [11] | マスターコマンド第3群 | `mrd_command_group3_execute()` | 9章 |
| [12-1] | シーケンス番号 | `mrd.seq_increase_num()` | 送信用の連番を+1して Meridim[1] に |
| [12-2] | サーボのエラー番号 | `mrd_servo_errcode_make_lite()` | Meridim[88] の下位8bit |
| [12-3] | チェックサム | `mrd.cksm_val()` | Meridim[89] |
| [13-1] | フレームの集計 | `mrd_disp.frame_time()` | `MONITOR_FRAME_DELAY`. PC 主導では遅延 (late) に数えない |
| [13-2] | 次のフレームまで待つ / 返信 | `mrd_frame_next_wait()` / `mrd_udp_send()` | ボード主導: タイマーを待つ. PC 主導: このフレームの結果をすぐ送り (`MODE_UDP_SEND` 1 のとき), `mrd_frame_resync()` |

PC から見ると, ボードの返信は1フレーム前の内容である (送信が次のフレームの先頭のため).

### 5.3 動作モード (`MODE_*`)

既定値は19.2節. ここでは効果だけを書く.

| 設定 | 効果 |
|---|---|
| `MODE_ESP32_STANDALONE` | 1: [8-1] でサーボと通信せず (PWM も出力しない), L00 にサイン波を返す. マスターコマンド 10100/10102 のサーボ駆動も省く 〔未検証〕 |
| `MODE_UDP_RECEIVE` | 0: PC から受信しない. 受信データの代わりにボードが指令を作る (`mrd_meridim_self_make()`). 起動後 50 フレーム (`UDP_SELF_LIMP_FRAMES`, 0.5秒) は全スロットのコマンドを 0 (脱力) にして現在位置を読み, その後は全スロットを 1 (トルクON) にする. [0] は常に 3 (ESP32 内の計算を優先) で, 目標は前回の指令値を保ち (最初は脱力中に読んだ現在位置. つまりトルクONの時点の姿勢を保つ), [7-2] の計算で動かす. トルクON後の指令値の変化は `UDP_SELF_SPEED` (deg/s, 既定 60) の速さに制限する ([7-3]). 目標は書き換えないので, 一度決めた目標にも届くまで近づき続ける. ただし位置を読めない種類 (PWM, XBUS) と応答しないサーボは起動時の角度が分からないため, トルクONの時点で 0° (または前回の値) を指令し, この最初の動きだけはサーボ自身の速さになる. 連番は予想どおりの値にするため bit10 / bit14 は立たず, フェイルセーフも働かない. PC からのマスターコマンドは受けない 〔未検証〕 |
| `MODE_UDP_SEND` | 0: 送信しない |
| `MODE_UDP_PASSIVE` | 通信の周期. 0: ボード主導 (既定. `FRAME_DURATION` ごとに送り, 受信を待つ). 1: PC 主導 (受信したらすぐ処理して返信し, フレームの時刻を待たない. 1フレーム待っても受信がなければ, 最後に正しく受信したパケットで処理して返信する. 受信エラー bit14 が立つ). 起動時の値で `flg.udp_board_passive` を決め, 動作中は 10005 / 10006 で切り替える. `MODE_UDP_RECEIVE` 0 では効かない. 起動時に "UDP cycle: ..." と表示 〔未検証〕 |
| `MODE_ETHER` | 0: WiFi, 1: 有線LAN (W5500) |
| `MODE_FIXED_IP` | 1: WiFi で ESP32 の IP を固定 (keys.h の `FIXED_IP_*`). 有線LANは常に固定IP |

---

## 6. スレッドと排他

| タスク | 関数 | コア | 優先度 | スタック | 周期 | 開始条件 |
|---|---|---|---|---|---|---|
| loopTask (Arduino) | `setup()` / `loop()` | Core1 | 1 | 8192 | 10ms (タイマー同期) | 常時 |
| `wire0_bno055` | `mrd_wire0_bno055_task` | Core0 | 2 | 4096 | 読み取り + `IMUAHRS_INTERVAL` (10ms) | BNO055 の初期化に成功 |
| `wire0_mpu6050` | `mrd_wire0_mpu6050_task` | Core0 | 2 | 4096 | 同上 | MPU6050 の初期化に成功 |
| `bt_wiimote` | `mrd_bt_wiimote_task` | Core0 | 5 | 4096 | `vTaskDelay(PAD_INTERVAL)` (10ms) | `MOUNT_PAD` = WIIMOTE / WIIMOTE_C |
| `blueretro` | `mrd_blueretro_task` | Core0 | 5 | 2048 | `vTaskDelay(PAD_INTERVAL)` (10ms) | `MOUNT_PAD` = BLUERETRO (I2C1 のピンを本体PWMに使うときは開始しない) |
| SBDBT 受信割り込み | `mrd_sbdbt_isr` | — | — | — | 信号の変化ごと | `MOUNT_PAD` = SBDBT |
| フレーム割り込み | `mrd_frame_timer_isr` | — | — | — | 10ms | 常時 |

タスクハンドルは `thp[0]` (センサ), `thp[1]` (INA226), `thp[2]` (WIIMOTE または BlueRetro). スレッドの作成に失敗したら (メモリ不足) 表示して停止する (`mrd_task_start`). WiFi・Bluetooth のスタックも Core0 で動く.

共有データの排他 (すべて `portMUX_TYPE` のスピンロック):

| ロック | 守るもの | 書き手 / 読み手 |
|---|---|---|
| `timer_mux` (main.cpp) | `count_timer` (とリセット時の `count_frame`) | 割り込み / loop |
| `ahrs_mux` (mrd_wire0.h) | `ahrs.read[16]` | センサスレッド / `mrd_meriput90_ahrs()` (16要素を一括でコピーしてから変換) |
| `pad_bt_mux` (mrd_bt_pad.h) | `pad_bt_val` (64bit) | WIIMOTE / BlueRetro スレッド (`mrd_pad_bt_store`) / `mrd_pad_read()` |
| `m_dir_mux` (`MrdHalfDuplexUart`) | 送信完了待ち〜受信への切替 (割り込み停止) | loop のみ |

`ahrs` のロール・ピッチ・ヨーの `*_source` / `*_origin` はロックなしの32bit単独アクセス.

---

## 7. 通信 (UDP, WiFi, 有線LAN)

### 7.1 UDP のパケット

- 中身は Meridim90 そのもの: 90個の int16, 180byte, リトルエンディアン (ESP32 のメモリ表現のまま).
- ESP32 → PC: 宛先 `WIFI_SEND_IP` (有線LANでは `ETHER_GATEWAY`), ポート `UDP_SEND_PORT` (22222).
- PC → ESP32: ESP32 のポート `UDP_RECV_PORT` (22224).
- ボード主導: ボードがフレームの先頭で送り, その後最大 4ms PC からの受信を待つ. PC は受け取ったら1パケット返す.
- PC 側が守ること・受信できなかったときの動作は7.5節.

### 7.2 WiFi (`MODE_ETHER` 0)

1. "WiFi connecting to => (SSID)" を表示.
2. `MODE_FIXED_IP` 1 なら `FIXED_IP_ADDR/GATEWAY/SUBNET` を解析 (`mrd_ip_address_parse`) して検証 (`mrd_network_config_validate`. 同一サブネットでなければ警告のみ) し, `WiFi.config()`. 誤りがあれば "Please Check '#define FIXED_IP_ADDR, ...'" を表示して停止 (書式の誤りでは WiFi でも "Ethernet initialization ABORTED." も出る. 0.0.0.0 などの検証の誤りでは "ERROR: ... is invalid").
3. `mrd_wifi_init()`: 切断 → STA で接続 → 50ms ごとに確認し, 約10秒で "Wifi init TIMEOUT." (**停止せず続行**. 以後 UDP は機能しない 〔制約 K-1〕). 成功したらモデムスリープを無効にし, UDP 受信を開始 (開始できなければ "UDP begin failed" を表示して false).
4. 接続先 PC の IP と ESP32 の IP (固定IP では実際の値ではなく設定の文字列) を表示.

- 受信 `mrd_wifi_udp_receive()`: 届いているパケットをすべて読み, 180byte 以上のうち最後に届いたものの先頭 180byte を使って true (チェックサムはここでは見ない. 最後のものが壊れていれば, 前の正しいパケットがあってもチェックサム不正のフレームになる). 180byte 未満のパケットと, パケットの残りは捨てる. 1つだけ遅れて届いたパケットは次のフレームでそのまま使う (WiFi の遅延の揺れを吸収する) が, 複数溜まっていれば古いものは捨てる (捨てた分は連番の飛び = bit10 になる). 送信元は確認しない (7.6節).
- 送信 `mrd_wifi_udp_send()`: `WIFI_SEND_IP` を最初の1回だけ解析. 解析できなければ送らず, 最初の1回だけ `ERROR: WIFI_SEND_IP "..." is not an IP address ...` と表示する (止まらない). パケットの開始・書き込み・送信のどれかに失敗したら false を返す.

### 7.3 有線LAN (`MODE_ETHER` 1, W5500)

1. `ETHER_MAC` を解析 (`mrd_ether_mac_address_parse`. "XX:XX:XX:XX:XX:XX"). 誤りなら停止.
2. `mrd_ether_init()`: `ETHER_LOCAL_IP/GATEWAY/SUBNET/DNS` を解析・検証 (`ETHER_GATEWAY` が 0.0.0.0 もここで弾く) → `Ethernet.begin()` (DHCP は使わない) → 設定した IP になったか確認 (ケーブルのリンクは確認しない 〔制約 K-14〕) → UDP 受信を開始. 失敗なら "Check Ethernet hardware/config." を表示して停止.
3. 送信先 = `ETHER_GATEWAY` (`ether_send_ip`). **PC を直結し, ゲートウェイ欄に PC の IP を書く運用**.

- 受信 `mrd_ether_udp_receive()`: WiFi と同じく, 届いているパケットをすべて読み, 180byte 以上のうち最後のものを使う.
- 送信 `mrd_ether_udp_send()`.

### 7.4 keys.h

項目の一覧は19.8節. IP の書式は "a.b.c.d" (空白は無視). 実際の SSID・パスワードを書くファイルなので, 扱いは7.6節.

### 7.5 PC とのインターフェース

PC 側のプログラム (Meridian_console 等) が守ること, ボードが保証することをまとめる.

**1フレームのやり取り**

```
0ms         ボード: 前のフレームで作った Meridim を送信 ([1])
0〜4ms      ボード: PC からの受信を待つ ([2-1]. 届いた時点で次へ進む)
            PC   : ボードのパケットを受けたら, 次の指令を1パケット返す
受信〜10ms  ボード: マスターコマンド, センサ, パッド, サーボ通信, 返信の作成 ([3]〜[12])
10ms        次のフレーム
```

| 項目 | 取り決め |
|---|---|
| 方式 | ボード主導 (既定): ボードが 10ms ごとに送り, PC は受け取るたびに1パケット返す. PC 主導 (`MODE_UDP_PASSIVE` 1 / 10006): PC が自分のタイマーで送り, ボードは受信したらすぐ処理して返す. 1フレーム (10ms) 受信がなければ, ボードは前回の指令で処理して返す (最初のパケットを待つ PC でも始められる). PC 主導での周期の上限はボードの処理時間 (主にサーボ通信. 10.13節) で決まる |
| PC の返信の期限 | ボード主導: ボードの送信から `UDP_TIMEOUT_US` (4ms) 以内. 遅れたパケットは捨てられず, 次のフレームで使われる (1フレーム遅れの指令になる) |
| パケット | 180byte (Meridim90). 180byte 未満は捨てる. 長いパケットは先頭 180byte を使う |
| PC が入れる値 | [0] マスターコマンド (通常は 1 か 90), [1] 連番 (毎回+1, 0〜59999 で循環), [19] 補間フレーム数, [20]〜[79] スロットのコマンドと値, [89] チェックサム (8.4節) |
| PC が受け取る値 | 1つ前に PC が送った指令を処理した結果. [1] はボードの連番 (PC の連番とは別), [2]〜[18] センサとパッド, スロットの値は現在位置か結果, [88] エラー |
| そのまま返る値 | ボードが書かない要素は PC が送った値が返る: [0] (応答コードを返すコマンドを除く), [19], スロットのコマンド (bit14 を除く), [80]〜[87], IMU なしの [2]〜[14], パッドなし (`MOUNT_PAD` = PC) の [15]〜[18]. ただしスロットのコマンドは, マウントなし (`SERVO_UNMOUNT_REPLY` 0, 既定) とマスターコマンド 0 / 2 のときは 0 になる. それ以外では, PC は返ってきたスロットのコマンドで指令が届いたことを確認できる |
| 起動直後 | 最初の送信は [0] = 90, 他は 0 のダミー. PC の最初のパケットを受けるまで, サーボは脱力のまま |
| 連番 | ボードは PC の連番が「前回の受信 + 1」かを確かめる. 違えば bit10 を立て, そのフレームのマスターコマンドを実行しない (スロットのコマンドは実行する) |
| 脱力させたいとき | [0] = 0 を送る (受信の成否に関係なく実行される. 9章). [0] を戻した後も脱力を続けるなら, スロットのコマンドも 0 にして送る |

**異常時の動作**

| 事象 | 検出 | 指令に使う値 | マスターコマンド | エラーの表れ方 |
|---|---|---|---|---|
| 受信なし (4ms 以内に届かない) | タイムアウト | 最後に正しく受信した PC のパケット (目標・コマンド・ボタンとも. `r_udp_meridim_last`). 正しいパケットが `UDP_FAILSAFE_MS` (1秒) 続けて届かなければ全スロットのコマンドを 0 (脱力) | 0 / 2 / 3 だけ効く | bit14 = 1, `err.pc_esp` +1 |
| チェックサム不正 | [2-2] | 受信なしと同じ (最後に正しく受信した PC のパケット). 続けば1秒で脱力 | 0 / 2 / 3 だけ効く | bit14 = 1, `err.pc_esp` +1. 連番は比べない. 次の正常なフレームは, 使わなかったパケットの分だけ連番が飛ぶため bit10 = 1 が1回 (受信なしと同じ) |
| 連番が飛んだ | [2-5] | 受信した値 (通常どおり. 脱力の判定では正しいパケットとして数える) | 0 / 2 / 3 だけ効く | bit10 = 1, `err.esp_skip` +1 |
| サーボが応答しない | 読み取りの失敗 | 前回の位置を返す | — | 連続6回で [88] の下位 byte にサーボ番号 (8.3節) |
| パラメータ系コマンドの失敗 | 結果 | — | — | スロットのコマンドの bit14, 値 = エラー値 (10.4節) |
| IMU の初期化失敗 | 起動時 | [2]〜[14] は 0 | — | 起動時の表示のみ |
| WiFi の接続失敗 | 起動時 | 受信しないため脱力のまま | — | 起動時の表示のみ 〔制約 K-1〕 |
| IP・MAC・有線LANの設定誤り | 起動時 | 停止 | — | 表示とエラーLED の点滅 (エラーLED のピンを他の用途に使うときは表示のみ) |
| EEPROM の書き込み禁止 | 10101 | 書かない | — | "EEPROM is protected…" と "write EEPROM failed." |

**通信が途絶えたとき (フェイルセーフ)** 〔未検証〕

- 正しいパケット (届いて, チェックサムが正しい) が `UDP_FAILSAFE_MS` (既定 1000ms) 続けて届かないと, 全スロットのコマンドを 0 にして全サーボを脱力する (`mrd_udp_failsafe()`, [2-7]). 連番の飛びは途絶とみなさない. 時間は `setup()` の終わりから数える.
- それまでの間 (1秒未満) は, 前回の指令を使い続ける (上の表).
- 脱力に入るとき "UDP lost for 1000 ms: all servos torque off (failsafe)." を1回表示し, `flg.udp_failsafe` を true にする. 脱力中に返すスロットのコマンドは 0.
- 正しいパケットが1つ届けば "UDP restored: servos follow the PC commands again." を表示し, そのパケットのコマンドに従う. 位置を読めるサーボは, 脱力中に読んだ現在位置から `SERVO_STEP_MAX` 以内の値を基準にしているため, トルクONに戻ると現在位置の近くから `SERVO_STEP_MAX` 以内ずつ目標へ動く. ただし最後のパケットのマスターコマンドが 2 のまま途絶えた場合は位置を読まないため, 基準は古い目標に向かって動いた値になる.
- `UDP_FAILSAFE_MS` 0 で無効 (最後の指令を保ち続ける). `MODE_UDP_RECEIVE` 0 では働かない.
- WIIMOTE は切断されるとボタン・スティックの値が 0 になる (15.2節).

### 7.6 通信の安全性の前提

- UDP は暗号化も認証もなく, 送信元の IP・ポートも確認しない. 同じネットワークにいれば誰でも, 180byte でチェックサムの合ったパケットを送るだけでロボットを動かせる. 信頼できる閉じたネットワーク (専用のアクセスポイント, PC との直結) で使うこと 〔制約 K-13〕.
- keys.h には実際の SSID・パスワードを書く. 配布・共有の前に必ず消すこと. 設定画面は keys.h を手元で読み書きするだけで, 外部には送らない.
- EEPROM の P0 にも SSID・パスワードが入る (`EEPROM_SET` 1 で P0 が config のときは keys.h の値). 10200 で読み出すときも 10300 で書くときも, パスワードは平文で流れる. EEPROM はフラッシュに暗号化せずに置かれるため, ボードを手元で読み出せば取り出せる (keys.h を書き込んだファームウェアと同じ). 配布・譲渡の前は P5 を消すこと.

---

## 8. Meridim90 配列

### 8.1 基本

- 90要素の int16 (`MRDM_LEN`). 共用体 `Meridim90Union` で short / unsigned short / byte として扱う.
- 送信用 `s_udp_meridim`, 受信用 `r_udp_meridim`, 最後に正しく受信したもの `r_udp_meridim_last`.
- 受信が正常なら受信した180byteを送信配列に丸ごと写し, その上にボードが担当する要素を書き込む. **ボードが書かない要素は PC の値がそのまま返る**. 受信できなかったフレームの扱いは7.5節.
- 角度・センサ値は ×100 の整数 (`float2HfShort`: 四捨五入し ±32767 で飽和).

### 8.2 インデックス表

| 添字 | キー | 意味 | 単位 | PC → ボード | ボード → PC |
|---|---|---|---|---|---|
| 0 | `MRD_MASTER` | マスターコマンド (9章). 既定は 90 (配列長) | — | コマンド | 受信値 (10100/10200〜10215/10300〜10315 では応答コード. 書き込みの失敗は 32766) |
| 1 | `MRD_SEQ` | シーケンス番号 | 0〜59999 | PC の連番 | ボードの連番 |
| 2〜4 | `MRD_ACC_X/Y/Z` | 加速度 | m/s² ×100 | — | IMU (14章) |
| 5〜7 | `MRD_GYRO_X/Y/Z` | 角速度 | deg/s ×100 | — | IMU |
| 8〜10 | `MRD_MAG_X/Y/Z` | 地磁気 | µT ×100 | — | BNO055 のみ (MPU6050 は 0) |
| 11 | `MRD_TEMP` | 温度 (degC) | ×100 | — | BNO055 / MPU6050 の内蔵の温度センサ (14章). IMU なしでは PC の値 〔未検証〕 |
| 12 | `MRD_DIR_ROLL` | ロール (校正点からの角度, −180〜180) | deg ×100 | — | IMU |
| 13 | `MRD_DIR_PITCH` | ピッチ (校正点からの角度, −180〜180) | deg ×100 | — | IMU |
| 14 | `MRD_DIR_YAW` | ヨー (校正点からの角度, −180〜180) | deg ×100 | — | IMU |
| 15 | `MRD_PAD_BUTTONS` | ボタン (8.6節) | bit | PC のボタン値 | パッド搭載時: 論理和か上書き |
| 16 | `MRD_PAD_STICK_L` | 左スティック | 上位/下位 byte | PC 値 | パッド搭載時に上書き |
| 17 | `MRD_PAD_STICK_R` | 右スティック | 同上 | PC 値 | パッド搭載時に上書き (WIIMOTE は 0) |
| 18 | `MRD_PAD_L2R2VAL` | L2/R2 アナログ | 同上 | PC 値 | パッド搭載時に上書き (BlueRetro・WIIMOTE_C のみ値. 上位 byte = L, 下位 byte = R, 0〜127. 他は 0) |
| 19 | `MRD_MOTION_FRAMES` / `MRD_STOP_FRAMES` | 目標へ移動するフレーム数 / 10008 の停止時間 | フレーム / ms | PC 値 | 受信値 |
| 20+2i | `C_HEAD_Y_CMD` 〜 `L_SERVO_IX14_CMD` | L系統 スロット i のコマンド (10.4節) | bit | 指令 | 受信値 (エラー時 bit14, マウントなしは 0) |
| 21+2i | `C_HEAD_Y_VAL` 〜 `L_SERVO_IX14_VAL` | L系統 スロット i の値 | deg ×100 等 | 目標 | 現在位置・結果 |
| 50+2i | `C_WAIST_Y_CMD` 〜 `R_SERVO_IX14_CMD` | R系統 スロット i のコマンド | | | |
| 51+2i | `C_WAIST_Y_VAL` 〜 `R_SERVO_IX14_VAL` | R系統 スロット i の値 | | | |
| 80〜85 | `MRD_USERDATA_80`〜`85` | ユーザー定義 | — | 任意 | 受信値 |
| 86 | `MRD_INA226_VOLT` (`MRD_USERDATA_86`) | INA226 のバス電圧 | V ×100 | 任意 | `MOUNT_INA226` 1 なら INA226 の値 (14.1節), 0 なら受信値 |
| 87 | `MRD_INA226_CURR` (`MRD_USERDATA_87`) | INA226 の電流 | A ×100 | 任意 | 同上 |
| 88 | `MRD_ERR` | エラー (上位8bit: エラービット, 下位8bit: サーボ番号) | — | PC 値 | 8.3節 |
| 89 | `MRD_CKSM` | チェックサム | — | PC の計算値 | ボードの計算値 |

関節のキー (i = スロット番号):

| i | L系統 (20+2i / 21+2i) | R系統 (50+2i / 51+2i) |
|---|---|---|
| 0 | `C_HEAD_Y` 頭ヨー | `C_WAIST_Y` 腰ヨー |
| 1 | `L_SHOULDER_P` 左肩ピッチ | `R_SHOULDER_P` 右肩ピッチ |
| 2 | `L_SHOULDER_R` 左肩ロール | `R_SHOULDER_R` 右肩ロール |
| 3 | `L_ELBOW_Y` 左肘ヨー | `R_ELBOW_Y` 右肘ヨー |
| 4 | `L_ELBOW_P` 左肘ピッチ | `R_ELBOW_P` 右肘ピッチ |
| 5 | `L_HIPJOINT_Y` 左股ヨー | `R_HIPJOINT_Y` 右股ヨー |
| 6 | `L_HIPJOINT_R` 左股ロール | `R_HIPJOINT_R` 右股ロール |
| 7 | `L_HIPJOINT_P` 左股ピッチ | `R_HIPJOINT_P` 右股ピッチ |
| 8 | `L_KNEE_P` 左膝ピッチ | `R_KNEE_P` 右膝ピッチ |
| 9 | `L_ANKLE_P` 左足首ピッチ | `R_ANKLE_P` 右足首ピッチ |
| 10 | `L_ANKLE_R` 左足首ロール | `R_ANKLE_R` 右足首ロール |
| 11〜14 | `L_SERVO_IX11`〜`14` 追加 | `R_SERVO_IX11`〜`14` 追加 |

各キーに `_CMD` / `_VAL` を付けたものが定義されている. コードが直接参照するキーは `MRD_MASTER`, `MRD_SEQ`, `MRD_PAD_BUTTONS`, `MRD_MOTION_FRAMES`, `MRD_STOP_FRAMES` で, 他は説明用.

### 8.3 エラー (Meridim[88])

上位8bit (エラービット):

| bit | 定義 | 意味 | LITE での扱い |
|---|---|---|---|
| 15 | `ERRBIT_15_ESP_PC` | ESP32 → PC の受信エラー | 使わない (PC の値が返る) |
| 14 | `ERRBIT_14_PC_ESP` | PC → ESP32 の受信エラー | ボードが設定: 受信なし (タイムアウト) かチェックサム不正で 1, 正常で 0 |
| 13 | `ERRBIT_13_ESP_TSY` | ESP32 → Teensy | 使わない |
| 12 | `ERRBIT_12_TSY_ESP` | Teensy → ESP32 | 使わない |
| 11 | `ERRBIT_11_BOARD_DELAY` | 処理の遅延 | 使わない |
| 10 | `ERRBIT_10_UDP_ESP_SKIP` | PC → ESP32 のフレームスキップ | ボードが設定: 連番が予想と違えば 1 |
| 9 | `ERRBIT_9_BOARD_SKIP` | PC → ESP32 → Teensy のスキップ | 使わない |
| 8 | `ERRBIT_8_PC_SKIP` | Teensy → ESP32 → PC のスキップ | 使わない |

下位8bit (`ubval[MRD_ERR_l]`) = 通信できないサーボの番号 (`mrd_servo_errcode_make_lite`):

- L系統のスロット i なら `100 + i` (100〜114), R系統なら `200 + i` (200〜214). 0 = 異常なし.
- 複数あるときは最後に見つかった1台 (スロット番号の大きい方, 同じ番号なら R 系統).
- 「通信できない」= そのサーボの読み取りが `SERVO_LOST_ERR_WAIT` (6) 回続けて失敗した (`stat` = 1). 1回成功すれば 0 に戻る.
- 毎フレーム読まない場合 (FUTABA の巡回, 混在系統の交互の読み取り, DXL2 の個別 READ, 全台脱力中の Hitec) は, 読んだときだけ数える. 読まない種類 (XBUS, PWM) は番号が出ない. 動作中の Hitec は番号が更新されない (脱力中に立った番号は残る).
- 起動時に応答しなかったサーボもマウントは外さず, この規則で番号を出す 〔制約 K-20〕.

### 8.4 チェックサム

```
cksm = (short)~( sval[0] + sval[1] + ... + sval[88] )   // int で加算し, ビット反転して下位16bit
```

ボードは [12-3] で計算して [89] に入れる. 受信時は同じ計算の結果が [89] と一致すれば正常.

### 8.5 シーケンス番号

- 範囲 0〜59999. 59999 の次は 0.
- 送信: 毎フレーム [12-1] で +1 (受信の成否に関係しない). 最初の送信は 0.
- 受信: 受信できたフレームだけ予想値を +1 し, Meridim[1] と比べる. 一致 → bit10 = 0, `flg.meridim_rcvd` = true. 不一致 → bit10 = 1, `err.esp_skip` を+1, 予想値を受信値に合わせ直す.
- 受信できなかったフレームとチェックサム不正のフレームは判定せず予想値も進めない (bit10 = 0, bit14 = 1). 次の正常なフレームでは, 使わなかったパケットの分だけ連番が飛ぶので bit10 = 1 が1回出て, 予想値を合わせ直す.
- マスターコマンドは `flg.meridim_rcvd` が true のフレームでだけ実行する (コマンド 0 / 2 / 3 を除く. 9章).

### 8.6 パッドのボタン (Meridim[15])

| bit | 値 | 定義 | ボタン |
|---|---|---|---|
| 0 | 1 | `PAD_SELECT` | Select |
| 1 | 2 | `PAD_L3` / `PAD_HOME` | L3 (HOME と共用) |
| 2 | 4 | `PAD_R3` | R3 |
| 3 | 8 | `PAD_START` | Start |
| 4 | 16 | `PAD_UP` | 十字 上 |
| 5 | 32 | `PAD_RIGHT` | 十字 右 |
| 6 | 64 | `PAD_DOWN` | 十字 下 |
| 7 | 128 | `PAD_LEFT` | 十字 左 |
| 8 | 256 | `PAD_L2` | L2 |
| 9 | 512 | `PAD_R2` | R2 |
| 10 | 1024 | `PAD_L1` | L1 |
| 11 | 2048 | `PAD_R1` | R1 |
| 12 | 4096 | `PAD_bU` | △ |
| 13 | 8192 | `PAD_bR` | ○ |
| 14 | 16384 | `PAD_bD` | × |
| 15 | 32768 | `PAD_bL` | □ |

### 8.7 単位・符号・座標の決まり

| 量 | 決まり |
|---|---|
| 角度 | deg ×100 の int16. 0° はトリムで合わせたサーボの中心. cw = +1 のとき角度が増えるとサーボの位置値が増える (物理的な回転の向きはサーボの機種で決まる). cw = −1 で反転する |
| 速度 (車輪) | rpm ×100 (1 = 0.01rpm. DYNAMIXEL 2.0 の rpm 基準に合わせる. ±327.67rpm). cw を適用し, 正の向きは角度と同じ |
| 速度 (パラメータ系 26/27) | deg/s (×100 しない). cw を適用せず, サーボ自身の向き |
| パラメータ系コマンドの値 | 12章の表の単位. ×100 するのは角度 (deg) と電圧 (V) だけ |
| IMU の軸 | センサの軸をそのまま使う. 基板への取り付けの向きで決まり, 機体の座標への変換はしない |
| ロール / ピッチ | BNO055: ロール = オイラー角の y, ピッチ = z (Adafruit ライブラリの値). MPU6050: DMP の ypr のロール = [2], ピッチ = [1] |
| 校正点 | ロール・ピッチ・ヨーとも校正点からの角度で, −180〜180 に折り返す. 校正点はロール・ピッチ・ヨーが 10003, ヨーだけなら 10002 で, いまの値にする (起動時は 0) |
| スティック [16] [17] | どのパッドも [16] 左, [17] 右. 上位 byte = X (右が正), 下位 byte = Y (上が正). 各 ±127. KRC-5FH: [16] の上位 byte = PA1, 下位 byte = PA2, [17] の上位 byte = PA3, 下位 byte = PA4. WIIMOTE: ヌンチャクのスティックの中心からの差. SBDBT: (値 − 64) × 2. BlueRetro・WIIMOTE_C: (値 − 0x80) × 127 / 0x66 (いずれも ±127 に収める) |
| 連番 | 0〜59999 で循環. ボードと PC がそれぞれ自分の連番を持つ |

---

## 9. マスターコマンド

Meridim[0] の値で, ボード全体への命令を行う. 3つの群に分かれ, それぞれ1フレームの決まった位置で実行する.

| 群 | 実行位置 | 関数 |
|---|---|---|
| 第1群 | [3] 受信の確認の直後 (センサ・パッドの書き込みより前) | `mrd_command_group1_execute()` |
| 第2群 | [6] パッドの書き込みの後, 目標値の計算より前 | `mrd_command_group2_execute()` |
| — | [7-1] 目標値の計算の中 | (コマンド 3) |
| 第3群 | [11] サーボの返信値の格納の後, 連番・チェックサムより前 | `mrd_command_group3_execute()` |

共通: 受信が正常で連番が予想どおりのフレーム (`flg.meridim_rcvd`) でだけ実行する. **PC が同じ値を送り続ける限り, 毎フレーム実行する**. 下表にない値 (既定の 90 や 1 など) は通常動作.

実行条件の例外 (脱力と ESP32 の計算の優先は, 受信できなかったフレームでも続ける):
- コマンド 0 / 2 は受信の成否を見ずに実行する. 受信できなかったフレーム (前回のパケットを使う), チェックサム不正のフレーム (同じく前回の正しいパケットを使う), 連番が飛んだフレームでも, 送信配列の [0] が 0 / 2 なら脱力する.
- コマンド 3 は [7-1] で送信配列の [0] だけを見るため, 同じく受信の成否を見ない.

| 値 | 定義 | 群 | 動作 | ボードの返信 | 注意 |
|---|---|---|---|---|---|
| 0 | `MCMD_TORQUE_ALL_OFF` | 2 | 全スロットのサーボコマンドを 0 にする (全サーボ脱力. 読めるサーボは位置を読む). "All servos torque off." を表示 | CMD 0, VAL 現在位置 | 正しく受信したフレームで受け取るたびに表示する 〔制約 K-7〕 |
| 1 | — | — | 通常動作 (スロットごとのコマンドに従う) | 通常 | |
| 2 | — | 2 | 全サーボ脱力 + そのフレームは位置を読まない. "All servos torque off." を表示 (0 と同じ) | VAL は前回値 | 位置を読まないのは DXL1/DXL2/FEETECH/FUTABA. ICS・PMX・Hitec は読む 〔制約 K-22〕 |
| 3 | — | [7-1] | トルクONのスロットは PC の目標を使わず前回の指令値を保持する (ESP32 内の計算 [7-2] を優先) | 通常 | |
| 10002 | `MCMD_SENSOR_YAW_CALIB` | 2 | いまのヨーを 0° にする (`ahrs.yaw_origin = ahrs.yaw_source`) | 受信値 | |
| 10003 | `MCMD_SENSOR_ALL_CALIB` | 2 | いまのロール・ピッチ・ヨーを校正点にする (それぞれ `ahrs.*_origin = ahrs.*_source`). 以後 [12]〜[14] は校正点からの角度になり, いまの値は 0 になる. "cmd: caliblate sensor's roll, pitch and yaw.[10003]" を表示 | 受信値 | 加速度・角速度・地磁気は補正しない. 〔未検証〕 |
| 10004 | `MCMD_ERR_CLEAR_SERVO_ID` | 1 | エラー番号 (Meridim[88] 下位8bit), 各サーボの失敗回数 (`err`) と通信不能の記録 (`stat`) を 0 にする | 受信値 | 応答しないままのサーボは, 再び6回続けて失敗すると番号が出る (8.3節) |
| 10005 | `MCMD_BOARD_TRANSMIT_ACTIVE` | 1 | ボード主導にする (`flg.udp_board_passive` = false), フレームタイマーのリセット | 受信値 | 5.1節. 設定 (`MODE_UDP_PASSIVE`) は変えない (次の起動で戻る) |
| 10006 | `MCMD_BOARD_TRANSMIT_PASSIVE` | 2 | PC 主導にする (`flg.udp_board_passive` = true), タイマーのリセット | 受信値 | 同上. このフレームは [1] で送ったうえ [13-2] でも送る (切り替えのフレームだけ2回) |
| 10007 | `MCMD_FRAMETIMER_RESET` | 2 | フレームタイマーのリセット (遅れの累積を捨てる) | 受信値 | |
| 10008 | `MCMD_BOARD_STOP_DURING` | 2 | Meridim[19] ミリ秒 (最大 1000ms) だけボードの処理を止める (`delay`), その後タイマーのリセット | 停止中は送受信なし | ループ全体が止まり, その間はフェイルセーフも働かないため 1000ms までにする. [19] は同じフレームで補間フレーム数としても使われる. 0 以下では止まらない 〔制約 K-4〕 |
| 10009 | `MCMD_EEPROM_ENTER_WRITE` | 1 | EEPROM 書き込みモード (`flg.eeprom_write_mode` = true), タイマーのリセット | 受信値 | 書き込みモードは未実装 |
| 10010〜10016 | `MCMD_EEPROM_EXIT_WRITE` / `ENTER_READ` / `EXIT_READ` / `MCMD_SDCARD_*` | — | 未実装 | | |
| 10100 | `MCMD_START_TRIM_SETTING` | 3 | トリム設定の開始 (Meridian_console 連携): (1) EEPROM の P1 のトリムと回転方向を読み込む. (2) 全サーボを目標 0 (EEPROM のトリムで補正した原点) へ駆動. (3) Meridim の値に元のトリムを deg ×100 で入れ, トリムを 0 にし, 目標を元のトリムの角度 (トリム × cw) にして再駆動する. サーボの角度は (2) と同じまま変わらない. 次のフレームの移動量制限の基準 (`cmd_past`) も同じ値にする. (4) EEPROM の P1 の [2]〜[87] を PC に送る. 以後トリムは 0 のまま. (1) で読み込むのはトリムと回転方向だけで, マウント・IDは変えない (次の起動から. 16.5節) | [0] = 10201, [2]〜[87] = EEPROM の P1 ([1][88][89] は通信の値) | 1フレームでサーボを追加で2回駆動する 〔制約 K-6〕 |
| 10101 | `MCMD_EEPROM_SAVE_TRIM` | 1 | 受信した Meridim のスロットの語 (L/R のコマンド・値, `i < num_max`) を EEPROM の P1 に書く. コマンド欄は設定ワード (16.3節: bit0 マウント, bit1〜7 ID, bit8 cw), 値はトリム (deg ×100) として保存される. 他のページは変えない | 受信値 | PC は設定ワードの形式で送ること (通常のコマンド 1 を送ると「マウントあり, ID 0, cw −1」として保存される). `EEPROM_PROTECT` 1 なら書かない. `num_max` 以降は 0 になる. `EEPROM_LOAD` で P1 を選んでいないと, `EEPROM_SET` 1 (既定) では次の起動で config の値に戻る 〔制約 K-5〕 |
| 10102 | `MCMD_EEPROM_LOAD_TRIM` | 1 | EEPROM の P1 のトリムと回転方向を読み込み, 全サーボを目標 0 へ駆動, タイマーのリセット | 通常 | この駆動は `SERVO_STEP_MAX` と補間を通らない. 同じフレームの [8-1] では前回の指令値から制限した値を送り直すため, 1フレームに 0° と制限値を続けて指令する. また駆動の回数 (`mrd_servo_frame`) が余分に進むため, 偶数/奇数フレームの割り振り (PWM, Hitec, XBUS, 混在系統) がずれる (10100 も同じ) 〔制約 K-6〕. 単体モードでは駆動しない |
| 10200〜10215 | `MCMD_EEPROM_BOARDTOPC_DATA0` + p | 3 | EEPROM のページ p の [2]〜[87] を送る (16.5節) | [0] = 10200 + p ([1][88][89] は通信の値) | 10203 以降は LITE の拡張 |
| 10300〜10315 | `MCMD_EEPROM_PCTOBOARD_DATA0` + p | 1 | 受信した [2]〜[87] を EEPROM のページ p に書く. P1 のトリム・回転方向はすぐ反映, 他は次の起動から (16.5節) | [0] = 10300 + p, 失敗は 32766 (`MCMD_NAK`) | 10303 以降は LITE の拡張 |
| 10400 | `MCMD_BOARD_RESTART` | 1 | ボードを再起動する (`ESP.restart()`). **そのパケットの全スロットのコマンド (下位8bit) が 0 のときだけ** 受け付ける. 受け付けないときは "cmd: restart refused ..." を表示 | (再起動するため返信なし) | LITE の拡張 |
| −32768 / −32767 | `MCMD_DUMMY_DATA` / `MCMD_TEST_VALUE` | — | 未使用 | | |
| 32766 | `MCMD_NAK` | — | 10300 + p の書き込みの失敗の返信 | | |
| 32767 | `MCMD_ACK` | — | 未使用 | | |


---

## 10. サーボ: 共通仕様

### 10.1 層の構成

```
main.cpp     [7-1] mrd_servo_targets_compute → [8-1] mrd_servo_drive_lite → [9-1] mrd_servo_values_reply
                                                    │
mrd_servo.h  mrd_servo_param_prepare / mrd_pwm_drive / mrd_servo_line_make / mrd_servo_lines_run
                                                    │ 系統ステッパー (MrdLineStepper) を L/R 並行に進める
sv_*.h       XxxLineStepper (通常の処理) + XxxParamOp (パラメータ系コマンド. XBUS にはない)   ← sv_util.h の共通部品
                                                    │ MrdHduartXfer (1回の送受信)
mrd_servo.h  MrdHalfDuplexUart (sv_bus_l / sv_bus_r)
                                                    │
ハード       Serial1 / Serial2 + DIR ピン + SN74LXC1T45 (または3ステートバッファ)
```

- バスは系統ごとに1つ: `sv_bus_l(&Serial1, PIN_RX_L, PIN_TX_L, PIN_EN_L, SERVO_ECHO_L, SERVO_TIMEOUT_L, SERVO_TX_RELEASE)`, R も同様.
- プロトコル層は全種類を L/R 分つねに作り, 同じバスを共有する: `ics_l/r`, `pmx_l/r`, `dxl1_l/r`, `dxl2_l/r`, `ftb_l/r`, `ftc_l/r`, `xbus_l/r`, `htc_l/r`. 系統ごとの実体は `mrd_xxx_line(L/R)` で取り出す.
- 混在に対応する種類 (FUTABA, DXL1, DXL2, FEETECH) は, 2種類目 (`MOUNT_SERVO_TYPE_L2/R2`) に当たれば `SERVO_BAUDRATE_L2/R2`, それ以外は `SERVO_BAUDRATE_L/R` を使う.

### 10.2 半二重UART (`MrdHalfDuplexUart`)

| 項目 | 仕様 |
|---|---|
| 方向の切替 | DIR ピン H: 送信 (A→B), L: 受信 (B→A) |
| 送信エコー | `ECHO_OFF` 0 (なし), `ECHO_ON` 1 (あり), `ECHO_AUTO` 2 (起動時に判定: 0x55 を最大5回送り, 1回でも返ればあり). 既定は `(MRD_BOARD_SERVO_IF == 1)` |
| エコーの照合 | エコーありなら, 受信の先頭で送信したバイト (控え, 最大 `MRD_HDUART_TX_MAX` = 128byte) と照合する. 不足 = `RX_ECHO_SHORT`, 不一致 = `RX_ECHO_MISMATCH` |
| 受信結果 | `RX_OK` 0, `RX_ECHO_SHORT` 1, `RX_ECHO_MISMATCH` 2, `RX_REPLY_NONE` 3 (応答なし), `RX_REPLY_PARTIAL` 4 (途中まで) |
| TX出力の切り離し | `SERVO_TX_RELEASE` 1 のとき, 受信中は TX ピンの出力を GPIO レジスタで切る (TX と RX が直結の基板用). RX ピンは内部プルアップ |
| 切替の順序 | 送信: DIR=H → TX 出力 ON. 受信: TX 出力 OFF → DIR=L (常に一瞬両方が切れる順) |
| 送信完了の検出 | 返信のない送信と同期の `transfer()` (起動時の処理, ICS ライブラリ, KRC-5FH) (`end_send`): 送信FIFOの残りが `MRD_HDUART_CRITICAL_BYTES` (8) 以下になったら割り込みを止め, 送信が終わった瞬間に受信へ切り替える. ステッパーの返信のある送受信 (`send_multi`): 送信を始めてから受信に切り替えるまで割り込みを止める (パケットは送信FIFO長 128byte 以下). いずれも IRAM の関数でレジスタを直接操作する |
| 受信バッファ | `MRD_HDUART_RX_BUF` 512byte |
| タイムアウト | `SERVO_TIMEOUT_L/R` (2ms). 1回の送受信の待ち上限は「必要バイト数 × 11bit / 通信速度 + タイムアウト」 |
| 待機時の方向 | 通常は受信方向. Hitec と XBUS は送信方向で待つ |
| 信号の反転 | Hitec 用 (UART を反転) |
| 区切り受信 | Hitec 用: ホストが 0x00 を送り, スタートビットを検出した瞬間に線を開放してサーボの返信を受ける |
| 通信速度の変更 | 混在系統で種類を切り替えるときに `set_baud()` (8N1 どうしのみ) |

### 10.3 系統ステッパーと L/R の並行実行

- 1回の送受信は `MrdHduartXfer` で表し, `mrd_hduart_xfer_start()` / `mrd_hduart_xfer_poll()` で待たずに進める.
- 系統ごとの1フレームの処理は `MrdLineStepper` (各プロトコルの `XxxLineStepper`) が「次の送受信を準備 (`next`) → 結果を反映 (`complete`)」の繰り返しで表す.
- `mrd_servo_lines_run()` は L と R のステッパーを交互に確認し, それぞれ自分の送受信が終わり次第に次を始める. 1フレームのサーボ通信時間はおおむね長い方の系統で決まる.
- ステッパーは毎フレーム, 静的な領域 (`MRD_STEPPER_BUF_BYTES` = 全ステッパーの最大サイズ) に作り直す.
- 系統の種類とステッパー: 43 → `IcsLineStepper`, 44 → `PmxLineStepper`, 31 → `Dxl1LineStepper`, 32 → `Dxl2LineStepper`, 21 → `FtbLineStepper`, 61/62 → `FtcLineStepper`, 51 → `XbusLineStepper`, 71 → `HtcLineStepper`. 0・1・11 は系統の種類としては使わない.
- 未対応の種類を指定した系統があると, どちらの系統も駆動しない.

### 10.4 スロットのコマンド (Meridim[20+2i] / [50+2i])

| ビット | 定義 | 意味 |
|---|---|---|
| 0〜7 | — | 0: 脱力, 1: 位置指示 (トルクON), 2: 脱力 (1以外は脱力扱い), **3以上: パラメータ系コマンド** (`MRD_PCMD_MIN` = 3, 12章) |
| 13 | `MRD_CMD_WHEEL` 0x2000 | 車輪. 下位8bitが1のときだけ有効. 値 = 速度 rpm ×100 (12.4節) |
| 14 | `MRD_PCMD_ERR_FLAG` 0x4000 | 返信のみ: パラメータ系コマンド・車輪の結果がエラー |
| 15 | `MRD_CMD_THROUGH` 0x8000 | スルー: そのサーボと通信せず, 受信したコマンド・値をそのまま返す |

判定の順序 (`mrd_servo_slot_mode`): bit15 → スルー, 下位8bit ≥ 3 → パラメータ, bit13 かつ下位8bit = 1 → 車輪, それ以外 → 通常.

値 (VAL) の意味:

| モード | 受信 | 返信 |
|---|---|---|
| 通常 (コマンド1) | 目標角度 deg×100 | 現在角度 deg×100 (戻り値のない種類は指令値) |
| 通常 (コマンド1以外) | 角度 deg×100 (変化量を制限して内部の指令値になるが送信しない) | 現在角度 (読めれば) |
| パラメータ | 書き込む値 (12章の単位) | 結果 (読んだ値 / 書き込んだ値 / エラー値) |
| 車輪 | 速度 rpm ×100 | 現在速度 rpm ×100 |
| スルー | 任意 | 受信値のまま |

パラメータ系コマンド・車輪のエラー値:

| 定義 | 値 | 意味 |
|---|---|---|
| `MRD_PCMD_NA` | −32768 | 未対応 (機種にない項目, 対応しない種類, PWM 専用スロット, レジスタの幅・符号に収まらない書き込み値. 収まらない値は送らない). ただし角度の書き込みは位置値の範囲に丸めて書き, ICS と Hitec (53) は値を各項目の範囲に丸めて書く (どちらも未対応にせず, 受け取った値を返す) |
| `MRD_PCMD_NO_REPLY` | −32767 | 応答なし (タイムアウト, エコー不一致, チェックサム不正, ID が範囲外, DXL1 で機種が分からないサーボへの PING 以外のコマンド・車輪) |
| `MRD_PCMD_SERVO_ERR` | −32766 | サーボがエラーを返した |

読んだ値は −32765〜32767 に収め, エラー値と重ならないようにする. 書き込みが成功したときは受け取った値を返す. 受け取った値も同じ範囲に収めてから使う (−32768〜−32766 を送ると −32765 として書く).

### 10.5 返信の規則 (`mrd_servo_slot_reply`)

| スロットの状態 | 返信 |
|---|---|
| 使用中 (マウント済み, または種類 1/11 の PWM 専用) かつスルー | コマンド・値とも受信値のまま |
| 使用中 かつパラメータ/車輪 | 値 = 結果. 結果がエラー値ならコマンドに bit14 を立て, そうでなければ落とす |
| 使用中 かつ通常 | 値 = 現在位置 (×100). コマンドは受信値のまま |
| マウントなし | `SERVO_UNMOUNT_REPLY` 0: コマンド・値・内部の指令値を 0. 1: 受信値のまま |

- マウントなし = 系統の種類が 0, スロットの種類が 0, EEPROM (P1 の bit0) で外した, 混在の検査で外れた, `num_max` 以降.
- 戻り値のない種類 (XBUS, 動作中の Hitec, PWM) と PWM 専用スロットは指令値を返す.
- マウントしていないスロットにパラメータ系コマンドを送っても, 返信は上の「マウントなし」の規則に従う.

### 10.6 目標値の計算 (`mrd_servo_targets_compute`, [7-1])

`frames` = constrain(Meridim[19], 0, `SERVO_MOTION_FRAMES_MAX`), マスターコマンドが 3 なら「ボード優先」. 各系統の `i < num_max` について:

1. `tgt_past = tgt` (前回の値. 前フレームで読んだ現在位置か指令値).
2. スロットが通常以外 → 補間をやめ, `tgt` を保持 (位置とトルクはサーボ側で保持).
3. コマンドが 1 以外 (脱力) → 補間をやめ, `tgt = mrd_step_limit(値/100, tgt_past, SERVO_STEP_MAX)`. 脱力中は実際の角度が基準になるので, トルクを入れたときはいまの姿勢から動き出す.
4. コマンド 1 かつボード優先 → 補間をやめ, `tgt = cmd_past` (前回の指令値).
5. コマンド 1 → `goal` = 補間の値 (`SERVO_MOTION_FRAMES_MAX` が 0 なら値/100), `tgt = mrd_step_limit(goal, cmd_past, SERVO_STEP_MAX)`.
6. `cmd_past = tgt` (2 以外).

`mrd_step_limit(t, base, m)` = constrain(t, base − m, base + m). `SERVO_STEP_MAX` = 15°/フレーム.

### 10.7 補間 (Meridim[19])

- スロットごとに `MrdMotion {from, goal, val, left, total, on}` を持つ (`mrd_servo_motion[2][15]`).
- 受信した値が前回の終点と違うとき (または補間中でないとき) に開始: `from` = いまの指令値, `goal` = 値/100, `total = left` = constrain(frames, 1, 65535).
- 毎フレーム `left` を1減らし, 指令 = from + (goal − from) × (total − left) / total. frames が 0/1 なら即時.
- 補間の途中で frames だけ変わっても反映しない (値が変わったときに反映).
- 脱力・パラメータ・車輪・スルー・マスターコマンド3 で補間をやめる. 補間の後にも `SERVO_STEP_MAX` の制限がかかる.

### 10.8 角度と位置値の換算

共通式 (`mrd_servo_deg2pos` / `mrd_servo_pos2deg`, 定数は `MrdPosScale {per_deg, center, min, max}`):

```
pos = clamp( round(center + trim × k + deg × k × cw), min, max )
deg = (pos − center − trim × k) / k × cw
```

トリムは回転方向 (cw) を掛けずにサーボの座標で足す. パラメータ系コマンドの角度は, Meridian の補正 (トリム・cw) を通すもの (`PC_POS`) と, サーボ自身の角度 (`PC_POS_NATIVE`: トリム0, cw +1) がある.

| 種類 | k (位置値/°) | 中心 | 範囲 | 角度範囲 | 備考 |
|---|---|---|---|---|---|
| ICS 43 | 29.63 (1値 = 0.03375°) | 7500 | 3500〜11500 | ±135° | Meridian ライブラリ `Deg2Krs`/`Krs2Deg` (切り捨て) |
| PMX 44 | 100 (0.01°) | 0 | −32000〜32000 | ±320° | `PMX_POS_SCALE` |
| DXL1 31 AX/RX | 1024/300 | 512 | 0〜1023 | 300° | モデル番号で判別. 起動時に応答しなかったサーボ (モデル番号 0) は, モデル番号を読めるまで目標・トルクを送らない (11.4節) |
| DXL1 31 EX-106+ | 4096/250.92 | 2048 | 0〜4095 | 250.92° | |
| DXL1 31 MX | 4096/360 | 2048 | 0〜4095 | 360° | |
| DXL2 32 | 4096/360 | 2048 | 0〜4095 | 360° | `DXL2_POS_SCALE` |
| FUTABA 21 | 10 (0.1°) | 0 | −1500〜1500 | ±150° | `FTB_POS_SCALE` |
| FEETECH STS 61 | 4096/`SERVO_FTC_STS_RANGE` (360) | 2048 | 0〜4095 | 360° | |
| FEETECH SCS 62 | 1024/`SERVO_FTC_SCS_RANGE` (300) | 512 | 0〜1023 | 300° | |
| Hitec 71 | `SERVO_HTC_US_PER_DEG` (10µs) | 1500µs | 550〜2450µs | ±95° | |
| XBUS 51 | 独自式 | 0x7FFF (1500µs) | 0x1249〜0xEDB6 (900〜2100µs) | ±60° (`SERVO_XBUS_ANGLE180` 1 で ±90°) | |
| PWM 1/11 | (+90° のパルス − 中央) / 90 (既定 11.11µs) | スロットごとの `IXL_PWM_CENTER` (既定 1500µs) | 0〜20000µs | サーボ側の角度を `IXL_PWM_MIN_DEG`〜`IXL_PWM_MAX_DEG` (既定 ±90°) に制限 | 13章 |

### 10.9 マウント, 通信不能, エラー番号

- マウント (`sv.ixl_mount` 等) = config の種類が 0 以外で, PWM 専用 (1/11) でなく, EEPROM で外していない (`cfg.ixl_unmount`). 起動時に応答しなかったサーボもマウントは外さない (表示のみ) 〔制約 K-20〕. 実行中に外すのは混在の検査でどちらの種類にも当たらないスロットだけ.
- 読み取りの反映 (`mrd_servo_read_apply`): 成功 → `tgt` = 角度, 失敗回数 `err` = 0, `stat` = 0. 失敗 → `tgt` = 前回の値, トルク状態を「不明」に, `err` を+1. `err` が `SERVO_LOST_ERR_WAIT` (6) に達したら `stat` = 1 (通信不能) にして 6 で止める. ICS は `mrd_ics_result2deg()` で同じ処理をする.
- 応答を読まない種類 (XBUS, 動作中の Hitec, PWM) は `err` が増えない.
- `stat` から Meridim[88] の下位8bit を作る (8.3節).

### 10.10 トルク状態の送り直し

- 系統ごとに, 最後に送ったトルク状態 (−1 不明 / 0 / 1) を覚え, 変わったときだけ送る.
- `MRD_SERVO_TORQUE_REFRESH_FRAMES` (20) フレームごとに全サーボへ送り直す (サーボの保護機能でトルクが切れても戻すため).
- PMX は返信に入るトルク状態で判断し, 指示と違えば送り直す (定期の送り直しはしない). ICS は毎フレーム位置命令か脱力命令を送る. Hitec は全台脱力 (0xEF) を送り直す. XBUS は偶数フレームごとに 位置 または 0 を送る.

### 10.11 1系統に2種類のサーボ (混在)

| 項目 | 仕様 |
|---|---|
| 設定 | `MOUNT_SERVO_TYPE_L2/R2` (0 = 混在なし), `SERVO_BAUDRATE_L2/R2`. スロットごとの種類は `IXL_MT` / `IXR_MT` |
| 混在できる種類 | 21 (FUTABA), 31 (DXL1), 32 (DXL2), 61/62 (FEETECH) のうち, 系列の異なる2種類. 61 と 62 は同じ系列 (FEETECH 層の中で STS/SCS の混在を扱う) |
| 混在できない種類 | ICS (チェックサムなし), PMX (RS-485), Hitec (信号反転), XBUS・PWM (方式違い) |
| 不可の組み合わせ | 警告を表示して混在なしにする |
| 起動 | 1種類目 → 2種類目の順に `begin()` |
| 1フレーム | `MrdMixedLineStepper` が 1種類目 → (通信速度を変更) → 2種類目 の順に進める. 目標は毎フレーム両方に送る. 現在位置は偶数フレームで1種類目, 奇数フレームで2種類目だけ読み, 読まない方は前回の値 |
| スロットの振り分け | `mrd_servo_line_mount()` が種類ごとのマウント配列を返す. どちらの種類でもないスロットはマウントを外して警告 |

### 10.12 起動時の処理 (種類別)

応答確認の再試行は `MRD_SERVO_BOOT_RETRY` (3) 回.

| 種類 | フレーム形式 | 起動時の処理 |
|---|---|---|
| 43 ICS | 8E1 | バスの開始のみ (応答確認・設定なし 〔制約 K-21〕) |
| 44 PMX | 8N1 (RS-485 変換経由) | 通信速度の検査 → ID 0〜239 外は警告 → SystemREAD で応答確認と応答時間の表示 → 制御モード (501) = 位置, 応答データ (502) = 現在位置 を RAM に書く |
| 31 DXL1 | 8N1 | モデル番号 (アドレス0) を読む → 応答遅延 (5) が 0 でなく `SERVO_DXL_FIX_RETURN_DELAY` 1 なら 0 を書く |
| 32 DXL2 | 8N1 | 通信速度の検査 → PING とモデル番号 → `SERVO_DXL_FIX_RETURN_DELAY` 1 で応答遅延 (9) が 0 でなければ, トルクOFF → 0 を書く → 読み返し → 動作モード (11) が位置制御系でなければ警告 → PING の返信のアラート (bit7) が立っていればハードウェアエラーを表示 → 応答したサーボの加速度・速度プロファイル (108, 112) を 0 |
| 21 FUTABA | 8N1 | 通信速度の検査 → 返信ディレイ (7) を読む (応答確認を兼ねる). 0 でなければ「0 にすると速い」と表示 (ROM なので自動では書き換えない) |
| 61/62 FEETECH | 8N1 | 通信速度の検査 → スロットの種類が 61/62 でなければ系統の種類に書き換える → 系列 (END, アドレス2) を読み, config と違えば警告 (config を使う) → 応答遅延 (7) が 0 でなく `SERVO_FTC_FIX_RETURN_DELAY` 1 なら ロック解除 → 0 → 再ロック → 読み返し → STS は目標時間・速度・加速度を 0 |
| 51 XBUS | 8N1 (送信方向で待つ) | 250000 以外なら警告. ID 1〜50 外は警告 (返信を使わないので応答確認なし) |
| 71 Hitec | 8N2, 反転 (送信方向で待つ) | 速度を `SERVO_HTC_SPEED` に. 19200 以外・ID 0〜127 外は警告. 台数が多いと各サーボの更新周期を表示 (応答確認なし) |

通信速度の検査 (`mrd_servo_baud_check`. 一覧にない値は警告のみ):

| 種類 | 有効な通信速度 (bps) |
|---|---|
| DXL2 | 9600, 57600, 115200, 1M, 2M, 3M, 4M, 4.5M |
| FEETECH | 1M, 500k, 250k, 128k, 115200, 76800, 57600, 38400 |
| FUTABA | 9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 153600, 230400, 460800, 691200 |
| PMX | 57600, 115200, 625000, 1M, 1.25M, 1.5M, 2M, 3M |

### 10.13 時間の予算

1フレーム (10ms) の中で, 受信待ち ([2-1], 最大 4ms) とサーボ通信 ([8-1]) が大半を占める. サーボ通信は L/R 並行なので, 長い方の系統で決まる. 目安として **受信待ち + サーボ通信が 9ms 程度に収まる**ようにする.

| 種類 | 条件 | サーボ通信の時間の目安 | 根拠 |
|---|---|---|---|
| ICS | 1.25Mbps | 1台 約53µs (6byte の転送) + サーボの応答時間 | 計算 |
| PMX | 3Mbps, 応答時間 200µs (出荷時) | 1台 約0.31ms, 15台 約4.6ms | 計算 (11.3節) |
| DXL1 | 1Mbps | 目標と READ (1台ずつ) で台数に比例 | 未計測 |
| DXL2 | 1Mbps, 15台 | 約3.5ms | 計算. 実測は XL430×2 @3Mbps で 0.64〜0.8ms |
| FUTABA | 460.8kbps | 目標 約1.15ms + 1台の読み 約0.52ms × `SERVO_FTB_READ_PER_FRAME` (既定8台で 約5.3ms) | 計算 (11.6節) |
| FEETECH | 1Mbps, 15台 | STS 約3.5ms, SCS 約4.1ms | 計算 (11.7節) |
| XBUS | 250kbps, 15台 | 偶数フレームだけ 約2.6ms (65byte) | 計算 |
| Hitec | 19200bps | 1台 約4.2ms × `SERVO_HTC_PER_FRAME` | 計算 (11.9節) |
| KRC-5FH | R系統の ICS | `PAD_INTERVAL` ごとに受信機への問い合わせ1回分が [5] に加わる | — |

パラメータ系コマンドと車輪は, そのフレームの時間を延ばす (12.1節).

時間を超えたとき:
- 遅れたフレームは捨てず, 次のフレームを待たずに始めて取り戻す (5.1節). 一時的に送信の間隔が縮む.
- 遅れはエラービットに出ない. `MONITOR_FRAME_DELAY` 1 の表示で確認する 〔制約 K-16〕.

### 10.14 サーボのリンク (`SERVO_LINK`, [7-4])

フォロワーのスロットに, リーダーのスロットの指令を毎フレームコピーする. 2台のサーボで1つの関節を両側から支える場合などに使う. 2台とも ID が別の通常のサーボとして通信する (同じ ID にはしない).

| 項目 | 仕様 |
|---|---|
| 設定 | config.h の `int SERVO_LINK[][3]`. 1行 = `{フォロワー, リーダー, 向き}`. スロットは `SV_L(n)` / `SV_R(n)` (13章と同じ). 向きは 1 (同じ値) / −1 (符号を反転). 行はいくつでも書ける. 使わない行は `{SV_NONE, SV_NONE, 1}`. EEPROM には保存しない |
| 起動時の検査 | `mrd_servo_link_setup()`. 範囲外のスロット, フォロワーとリーダーが同じ, 向きが ±1 以外, フォロワーが他の行のリーダー (連鎖), フォロワーが前の行のフォロワーと同じ, の行は "Servo link ignored (check SERVO_LINK): L07 = 15" のように表示して使わない. 使う行は "Servo link: L04 = -L03" と表示する |
| 実行 | [7-1]〜[7-3] の後, [8-1] の前. リーダーが通常・車輪のとき, フォロワーのコマンド = リーダーのコマンド, VAL = 向き × リーダーの VAL (−32768 の反転は 32767), 指令値 (degree) = 向き × リーダーの指令値 (補間・変化量の制限・[7-2]・[7-3] を通った後の値). 2台は同じフレームで同じだけ動く |
| 単位 | Meridim の値 (トリム・回転方向を掛ける前) をコピーする. フォロワー自身のトリムと `IXL_CW` / `IXR_CW` はそのまま効く. 車輪の速度 (0.01 rpm) にも向きを掛ける |
| PC が送るフォロワーの値 | 使わない (上書きする). 返信はフォロワー自身の現在位置 (または速度), コマンドはコピーした値 |
| パラメータ系コマンド・スルー | PC がフォロワーのスロットに送ったものは, コピーせずそのまま実行する (フォロワーだけゲインを変えるなど). リーダーがパラメータ系コマンド・スルーの間は, フォロワーは直前にコピーした指令 (コマンド・VAL・指令値) を保つ. 起動直後でまだコピーしていなければコマンド 0 (脱力) |
| 制約 | フォロワーのスロットは通常どおりマウントしておく (種類・ID). `num_max` 以降のスロットは駆動されない. 単体モード (`MODE_ESP32_STANDALONE`) でも実行するが, サーボは駆動しない. 実機では未検証 〔未検証〕 |

---

## 11. サーボ: プロトコル別仕様

### 11.1 機能の対応

| 種類 | 現在位置 | パラメータ系 | 車輪 | 混在 | マスター2 で読まない | トルクの送り直し |
|---|---|---|---|---|---|---|
| 43 ICS | 毎フレーム全台 | ○ | × | × | × (読む) | 不要 (毎回命令) |
| 44 PMX | 毎フレーム全台 (命令の返信) | ○ | ○ | × | × | 返信のトルク状態で判断 |
| 31 DXL1 | 毎フレーム全台 (1台ずつ READ) | ○ | ○ | ○ | ○ | 20フレームごと |
| 32 DXL2 | 毎フレーム SyncRead | ○ | ○ | ○ | ○ | 20フレームごと |
| 21 FUTABA | `SERVO_FTB_READ_PER_FRAME` 台ずつ巡回 | ○ | × | ○ | ○ | 20フレームごと |
| 61/62 FEETECH | 毎フレーム (STS/SCS 混在時は交互) | ○ | ○ | ○ | ○ | 20フレームごと |
| 51 XBUS | なし (指令値) | × | × | × | — | なし |
| 71 Hitec | 全台脱力中だけ巡回して読む | 3/6/53 のみ | × | × | × | 0xEF を20フレームごと |
| 1/11 PWM | なし (指令値) | × | × | — | — | — |

### 11.2 KONDO ICS 3.5/3.6 (43, `sv_ics.h`)

- 8E1. 通信速度 115200 / 625000 / 1250000 (サーボの設定に合わせる). ID 0〜31.
- 位置命令: `0x80+ID, POS_H(7bit), POS_L(7bit)` → 返信3byte (`ID`, 現在位置). 位置0は脱力 (返信は現在位置). 返信の先頭が ID でなければ (他のサーボの返信, ずれた受信) 読み取りの失敗とする.
- 1フレーム: マウントしたスロットを順に, コマンド1なら位置命令, それ以外は脱力を送り, 返信の位置を角度にする. ID が範囲外のスロットは送らず失敗として数える (続けば通信不能になる).
- パラメータ: 読み `0xA0+ID, SC` / 書き `0xC0+ID, SC, 値` (SC: 1 ストレッチ, 2 スピード, 3 電流, 4 温度, 5 現在位置 [ICS3.6]). 書き込みは RAM のみ. EEPROM は 64byte 単位で読み書きする (読む → 1項目を変える → 書く). 温度は表 (`ICS_TEMP_TABLE`) で換算.
- KRR-5FH 受信機 (15章) は R系統が ICS のとき `ics_r` で読む.

### 11.3 KONDO PMX (44, `sv_pmx.h`)

- PMX-SCR-5203HV / 5204HV / 9203HV / 9204HV 等. 信号は RS-485 のため, **自動方向切替の RS-485 変換**を介して接続する. 推奨 3Mbps. ID 0〜239 (範囲外のスロットは送らず失敗として数える). 〔未検証〕
- パケット: 送信 `FE FE ID LEN CMD OPT [DATA] CRC_L CRC_H`, 返信 `FE FE ID LEN CMD&0x7F STATUS [DATA] CRC`. CRC-16-CCITT (0x1021, 初期値0). 値はリトルエンディアン.
- コマンド: 0xA0 MemREAD, 0xA1 MemWRITE (OPT 1 = トルクON中でも書く), 0xA3 SAVE (Free のときのみ), 0xA4 MotorREAD, 0xA5 MotorWRITE (OPT 0 目標値, 1 トルクON, 2 Free), 0xBB SystemREAD.
- 1フレーム (サーボごとに1〜3回): 車輪から戻ったサーボは位置制御に戻す (RESTORE) → トルク状態が指示と違えば MotorWRITE (ON/Free) → コマンド1なら MotorWRITE (目標位置), それ以外は MotorREAD. 返信 (11byte: ステータス, トルク状態, 現在位置) で全台の現在位置を毎フレーム読む. 同期書き込みがないため1台ずつ送る.
  - Free を送ったサーボは, その返信の位置を使い MotorREAD を省く.
  - トルクON の送信に失敗したら, そのフレームは目標位置を送らない.
- 時間の目安: 3Mbps で1台 約77µs + 応答時間 (設定値+30µs). 出荷時 200µs なら1台 約0.31ms, 15台 約4.6ms.

### 11.4 DYNAMIXEL Protocol 1.0 (31, `sv_dxl1.h`)

- AX, RX, EX-106+, MX (Protocol 1.0). モデル番号で位置・速度の単位を決める. 起動時に応答しなかったサーボ (モデル番号 0) は, 尺度が決まらないため目標位置・トルクを送らず, 1フレームの [3] で現在位置の代わりにモデル番号 (アドレス0) を読む. 読めた次のフレームから通常どおり動かす (トルク状態は送り直す). それまでのパラメータ系コマンド・車輪は応答なし (−32767), PING (3) は送る. 〔未検証〕 パケットは FEETECH と同じ形式 (`FF FF ID LEN INST PARAM CHK`) なので `sv_ftc.h` の生成・検証を使う.
- 1フレーム: [1] SyncWrite 30 (目標位置, コマンド1のみ) → [2] SyncWrite 24 (トルク, 変化時と20フレームごと) → [3] 各サーボに READ 36 (現在位置) を1台ずつ (SyncRead がなく, BulkRead は MX のみのため). 移動速度 (32) は書かない.
- 通信速度の検査はしない (AX の初期値 1M, MX/RX は 57600).

### 11.5 DYNAMIXEL Protocol 2.0 (32, `sv_dxl2.h`)

- Xシリーズ (XL330, XL430, XC430, XM430, XH430, XM540 等) と Protocol 2.0 の MX. XL-320, PRO/P は対象外. 初期値の 57600bps から, 事前に 1Mbps 以上に変えておくこと.
- パケット: `FF FF FD 00 ID LEN_L LEN_H INST [PARAM] CRC_L CRC_H`, バイトスタッフィングあり. CRC-16 (0x8005).
- 1フレーム: [1] SyncWrite 116 (目標位置 4byte, コマンド1のみ) → [2] SyncWrite 64 (トルク, 変化時と20フレームごと) → [3] SyncRead 132 (現在位置) で読めているサーボを1回の要求で読む → [4] 読めなかったサーボを1フレーム1台ずつ個別に READ.
- SyncRead の返信が途中で途切れたとき: 最後に受信できたサーボより後ろの未受信は, 先頭の1台だけを失敗とし, 残りは前回の値を保って次のフレームで読み直す.
- 時間の目安: 1Mbps 15台で 約3.5ms. 実測 XL430×2 @3Mbps で 0.64〜0.8ms.

### 11.6 FUTABA RS30x (21, `sv_ftbrx.h`)

- RS303MR, RS304MD 等. 最大 460.8kbps (初期値 115.2kbps. 通信速度の検査は 691200 も有効とする).
- パケット: ショート `FA AF ID FLG ADR LEN CNT [DATA] SUM`, ロング `FA AF 00 00 ADR LEN CNT [VID DATA]... SUM`, 返信 `FD DF ...`. SUM = XOR. パラメータ系の書き込みは FLG = 01 (ACK/NACK を返す) で送り, 返信は1byte (07h: ACK, 08h: NACK = サーボのエラー. RS303MR/RS304MD 取扱説明書 p.19).
- 1フレーム: [1] ロングパケット 36 (トルク. RS30x はトルクOFF中の目標を無視するため先に送る) → [2] ロングパケット 30 (目標位置, 0.1°) → [3] READ 42 (現在位置) を前回の続きから `SERVO_FTB_READ_PER_FRAME` 台まで. 読まないサーボは前回の値.
- 時間の目安: 460.8kbps で 目標 約1.15ms + 1台の読み 約0.52ms. 15台を毎フレーム読むと約9ms で100Hz に収まらないため, 読む台数を絞る (既定 8台で約5.3ms) 〔制約 K-23〕.

### 11.7 FEETECH STS / SCS (61/62, `sv_ftc.h`)

- 系列はスロットの種類 (61 STS / 62 SCS) で決まる. 同じ系統に STS と SCS を混ぜてよい (現在位置は系列ごとに1フレームずつ交互に読む).

| 項目 | STS | SCS |
|---|---|---|
| 2byte 値の順序 | 下位が先 (LE) | 上位が先 (BE) |
| 位置の範囲 | 0〜4095 (360°) | 0〜1023 (300°) |
| EEPROM のロック | アドレス 55 | アドレス 48 |
| 目標の送信 | 位置 (2byte) | 位置 + 目標時間0 + 目標速度 `SERVO_FTC_SCS_SPEED` (SCS は速度0では動かない) |

- 1フレーム: [1] SyncWrite 42 (目標, 系列ごと) → [2] SyncWrite 40 (トルク, 変化時と20フレームごと) → [3] 各サーボに READ 56 (現在位置) を1台ずつ.
- SCS の目標の SyncWrite は, コマンド 1 (トルクON) の通常のスロットに, 毎フレーム目標時間 (44) に 0, 目標速度 (46) に `SERVO_FTC_SCS_SPEED` を書く.
- 時間の目安: 1Mbps 15台 STS で 約3.5ms (SCS のみで 約4.1ms).

### 11.8 JR PROPO XBUS (51, `sv_xbus.h`)

- NX8921 等. 250000bps 8N1 固定. 返信なし (動作命令のみ).
- パケット: `A4 LEN 00 00 {ID 00 POS_H POS_L}×N CRC8` (CRC-8, 多項式 0x8C).
- 偶数フレームだけ (50Hz), マウントした ID 1〜50 の全サーボを1パケットで送る. コマンド1なら位置, それ以外は 0x0000 (脱力).
- 現在位置は読まない (指令値を返す). パラメータ系コマンド・車輪のスロットには送らないため, 約1.5秒でサーボが脱力することがある 〔制約 K-25〕.

### 11.9 Hitec HSR (71, `sv_hitec.h`)

- HSR-8498HB 等 (HMI シリアル). 19200bps 8N2, 信号反転.
- パケット: `0x80 CMD DATA1 DATA2 SUM` の5byte, 続く2byte の枠にサーボが返信を書く. 位置命令 CMD = ID (パルス幅 µs). 0xE9 = 速度の設定と位置の読み出し. 0xEF = 全台脱力.
- 1フレーム (1台 約4.2ms. `SERVO_HTC_PER_FRAME` × 2 台を超えると (既定では3台以上) 各サーボの更新が 50Hz 未満 〔制約 K-24〕):
  - 動作中 (トルクONのサーボがある): コマンド1のサーボに順番に位置命令を `SERVO_HTC_PER_FRAME` 台まで. 前のフレームで送ったサーボには送らない (各50Hz 上限). 返信は読まない (指令値を返す).
  - 全台脱力中: OFF になったときと20フレームごとに 0xEF (これも `SERVO_HTC_PER_FRAME` の枠を1つ使う). 枠が余れば, 同じフレームで 0xE9 で巡回して位置を読む.
  - 一部だけ OFF: そのサーボには送らず, 最後の位置を保つ (個別に脱力する命令がない).

---

## 12. パラメータ系コマンドと車輪

### 12.1 実行の規則

- スロットのコマンドの下位8bitが 3 以上のとき, その番号のパラメータ系コマンドとして扱う (番号と値の形式は `tasks/servo_param_plan.md` と同じ).
- **コマンド (下位8bit) か値が前回と変わったフレームで1回だけ実行**し, 以後は同じ結果を返し続ける. 通常・スルーに戻すと状態を消す (同じコマンドを送り直せば再実行).
- そのフレームでは位置指示の代わりに実行する. 各系統の通常の処理が終わった後に, 実行待ちのスロットを1件ずつ行う (1件は1〜3回の送受信).
- パラメータ・車輪・スルーのスロットは通常の処理 (トルク・目標・現在位置) から外す. **トルク状態と位置はサーボ側で保持**される.
- 書き込みの結果は受け取った値 (読み返しはしない. 値の範囲は10.4節) 〔制約 K-31〕. 3 (ping) と 35 (ROM 保存) の成功は 1.
- 対応する種類: 43, 44, 31, 32, 21, 61/62, 71 (71 は 3/6/53 のみ). それ以外の種類と PWM 専用スロット (1/11) は未対応 (−32768). マウントなし・`num_max` 以降のスロットは 10.5節の「マウントなし」の規則で返す.
- 1フレームの時間が延びるもの: ICS の EEPROM 書き込み (返信待ち最大50ms), PMX の SAVE (40ms), FEETECH の EEPROM 書き込み (3回の送受信), Hitec (1件 約4.2ms), 車輪 (1スロットにつき毎フレーム最大3回の送受信). これらのフレームは 10ms を超えることがある 〔制約 K-26〕.
- 角度の補正: 角度限界はサーボ自身の角度 (`NAT`: トリム 0, cw +1 で換算). オフセットは倍率による換算 (DXL2, STS) か生値 (PMX: 0.01°) で, やはりサーボ自身の角度になる. 目標・現在角度 (4〜6) は Meridian のトリム・cw を通す (`POS`). 両方の補正が二重に効かないよう区別している.
- 実機での確認は済んでいない 〔未検証〕.

### 12.2 番号表 3〜38

凡例: `addr/len` = アドレス/バイト数, E = EEPROM/ROM 領域 (FEETECH ではロック解除 → 書き込み → 再ロックを伴う. 他社は参考表示), `s` = 2の補数, `b15/b11/b10` = その bit が符号で残りが大きさ, ×k = 値 = レジスタ値 × k, 「27 にも同値」= 隣のアドレスにも同じ値を書く, — = 未対応.

| 番号 | 内容 (値の単位) | R/W | DXL1 | DXL2 | FUTABA | FEETECH STS | FEETECH SCS | PMX | ICS | Hitec |
|---|---|---|---|---|---|---|---|---|---|---|
| 3 | 応答確認 (成功=1) | R | PING | PING | 0番地 2byte | PING | PING | SystemREAD | SC1 読み | 0xE9 |
| 4 | 目標角度 (deg×100, POS) | W | 30/2 | 116/4 s | 30/2 s (トルクOFF中は無視) | 42/2 b15 | 42/2 | MotorWRITE (Free 中はエラー) | 位置命令 (トルクが入る) | — |
| 5 | 目標角度 (POS) | R | 30/2 | 116/4 s | 30/2 s | 42/2 b15 | 42/2 | 506/2 s | — | — |
| 6 | 現在角度 (POS) | R | 36/2 | 132/4 s | 42/2 s | 56/2 b15 | 56/2 | 300/2 s | SC5 (ICS3.6) | 0xE9 |
| 7 | 電圧 (V×100) | R | 42/1 ×10 | 144/2 ×10 | 52/2 (10mV) | 62/1 ×10 | 62/1 ×10 | 314/2 ×0.1 | — | — |
| 8/9 | オフセット (deg×100, NAT) | R/W | — | 20/4 s ×8.79 E | — | 31/2 b11 ×8.79 E | — | 72/2 s (書きはトルクOFF時) | — | — |
| 10/11 | 不感帯 (deg×100) | R/W | — | — | 24/1 ×10 E (25 にも同値) | 26/1 ×8.79 E (27 にも同値) | 26/1 ×29.3 E (27 にも同値) | 64/2 s | — | — |
| 12/13 | CW 角度限界 (NAT) | R/W | 6/2 E | 52/4 s E | 8/2 s E | 9/2 E | 9/2 E | 96/2 s | EEPROM 上限パルス | — |
| 14/15 | CCW 角度限界 (NAT) | R/W | 8/2 E | 48/4 s E | 10/2 s E | 11/2 E | 11/2 E | 100/2 s | EEPROM 下限パルス | — |
| 16/17 | P ゲイン | R/W | MX: 28/1 | 84/2 | — | 21/1 E | 21/1 E | 0/4 | SC1 ストレッチ | — |
| 18/19 | I ゲイン | R/W | MX: 27/1 | 82/2 | — | 23/1 E | 23/1 E | 4/4 | — | — |
| 20/21 | D ゲイン | R/W | MX: 26/1 | 80/2 | — | 22/1 E | 22/1 E | 8/4 | — | — |
| 22/23 | 温度上限 (℃) | R/W | 11/1 E | 31/1 E | 22 のみ: 14/2 E | 13/1 E | 13/1 E | 88/2 s ×0.1 | 読: EEPROM / 書: SC4 (RAM) | — |
| 24/25 | 電流上限 (mA) | R/W | — | 38/2 ×2.69 E | — | 28/2 ×6.5 E | — | 84/2 | 読: EEPROM / 書: SC3 (RAM) | — |
| 26 | 現在速度 (deg/s) | R | 38/2 b10 ×rpm×6 | 128/4 s ×1.374 | 46/2 s | 58/2 b15 ×0.088 | 58/2 b15 ×0.293 | 302/2 s ×0.1 | — | — |
| 27 | 速度の設定 (deg/s) | W | 32/2 ×rpm×6 | 112/4 ×1.374 | — | 46/2 b15 ×0.088 | 46/2 ×0.293 (コマンド 1 に戻すと上書き) | 104/2 s ×0.1 | — | — |
| 28/29 | 加速時間 (ms) | R/W | — | 108/4 | — | — | — | — | — | — |
| 30/31 | 最大出力 (%) | R/W | 34/2 ×0.1 | 36/2 ×0.113 E | 35/1 | 48/2 ×0.1 | 16/2 ×0.1 E | 110/2 | — | — |
| 32/33 | 到達時間 (ms) | R/W | — | 112/4 | 32/2 ×10 | 44/2 | 44/2 (コマンド 1 に戻すと上書き) | — | — | — |
| 34 | 工場出荷に戻す | — | — | — | — | — | — | — | — | — |
| 35 | ROM 保存 (成功=1) | W | 1 (書いた時点で保存) | 1 (同) | ROM 書き込み (返信なし. 送れば 1) | 1 (同) | 1 (同) | SAVE (Free のみ) | EEPROM を読んで書き戻す (RAM の値が保存される) | — |
| 36 | 現在温度 (℃) | R | 43/1 | 146/1 | 50/2 s | 63/1 | 63/1 | 310/2 ×0.1 | SC4 (表で換算) | — |
| 37 | 現在電流 (mA) | R | MX-64/106: 68/2 (値−2048) ×4.5 | 126/2 s ×2.69 | 48/2 s | 69/2 b15 ×6.5 | — | 304/2 s | SC3 ×100 (逆転側も大きさのみ) | — |
| 38 | トルク状態 (0/1) | R | 24/1 | 64/1 | 36/1 | 40/1 | 40/1 | 500/1 | — | — |

偶数が読み, 奇数が書きの組 (8/9, 10/11, ...). 補足:

- 倍率: ×rpm×6 は DXL1 の機種で決まる (AX/RX/EX 0.666, MX-28/64/106 0.684, MX-12W 5.496). ×0.088 = 360/4096 (STS の1ステップ), ×0.293 = 300/1024 (SCS), ×8.79 = 36000/4096, ×29.3 = 30000/1024.
- DXL1: 機種が分からないサーボ (モデル番号 0) には, PING 以外のパラメータ系コマンドを送らず応答なし (−32767) を返す (11.4節).
- DXL2: EEPROM 領域 (64番地未満) はトルクOFFでないと書けない (サーボエラー) 〔制約 K-27〕. 先にコマンド0で脱力しておく. 電流の単位は機種で異なる (XL330 は 1mA, XL430-W250 は電流の項目なし, 他は 2.69mA).
- FEETECH: E の項目は ロック解除 → 書き込み → 再ロック の3回で保存まで行う. 現在速度 (26) の符号は STS・SCS とも bit15. SCS の 27 と 32/33 は, 通常のトルクON (コマンド 1) に戻すと毎フレームの目標の送信 (11.7節) で上書きされる (脱力中とパラメータ系を続けている間は残る) 〔制約 K-29〕.
- FUTABA: ROM 領域は書いた時点で有効, 電源で消えるため 35 で保存する.
- PMX: 書き込みはすべて RAM. 保存は 35.
- ICS: SC1〜4 の書き込みは RAM のみ. EEPROM の項目は 64byte 単位で読み書きする. EEPROM を読むと RAM の値 (SC1〜4 で書いた値) が返るため, 35 と EEPROM の項目の書き込みでは, その RAM の値も一緒に保存される 〔制約 K-30〕.

### 12.3 番号表 50 以降 (各社固有, 値は生値)

| 番号 | 内容 | DXL1 | DXL2 | FUTABA | FEETECH | PMX | ICS | Hitec |
|---|---|---|---|---|---|---|---|---|
| 50/51 | ストレッチ | — | — | — | — | 12/4 | SC1 (1〜127) | — |
| 52/53 | スピード | — | — | — | — | — | SC2 (1〜127) | 53 のみ: 1〜255 (RAM. 以後の 0xE9 で送る) |
| 54/55 | パンチ | 48/2 | — | 28/2 E | 24/2 E | — | EEPROM (0〜10) | — |
| 56/57 | デッドバンド | AX/RX/EX: 26/1 (27 にも同値) | — | 24/1 E (25 にも同値) | 26/1 E (27 にも同値) | 64/2 s | EEPROM (0〜5) | — |
| 58/59 | ダンピング | — | — | — | — | — | EEPROM (1〜255) | — |
| 60/61 | コンプライアンススロープ | AX/RX/EX: 28/1 (29 にも同値) | — | 26/1 E (27 にも同値) | — | — | — | — |
| 62〜67 | 特性変化ストレッチ1〜3 | — | — | — | — | — | EEPROM | — |
| 68/69 | セーフタイマー | — | — | — | — | — | EEPROM (10〜255) | — |
| 70/71 | 応答遅延 | 5/1 E (2µs) | 9/1 (2µs) E | 7/1 E (100+50n µs) | 7/1 E (2µs) | — | EEPROM レスポンス (1〜5) | — |

### 12.4 車輪 (bit13 + コマンド1)

速度制御 (車輪) モードはサーボ側で事前に設定しておく (PMX は RAM で自動的に切り替える). 値は rpm の100倍 (1 = 0.01rpm. DYNAMIXEL 2.0 の rpm 基準に合わせた単位. 例: 45.8rpm → 4580. 回転方向の補正 cw だけを適用), 返信は現在速度 (同じ単位). 毎フレーム実行する. 1rpm = 6 deg/s.

| 種類 | 手順 | 速度の換算 |
|---|---|---|
| DXL2 | [トルクON (必要時)] → 目標速度 104 (int32) → 現在速度 128 | 1 = 0.229rpm (Meridim の値 ÷ 22.9) |
| DXL1 | [トルクON] → 移動速度 32 (bit10 が方向, ±1023) → 現在速度 38 | 1 = 機種ごとの rpm (AX 0.111rpm, MX 0.114rpm など). ±1023 に制限 |
| FEETECH | [トルクON] → 目標速度 46 (STS: bit15 / SCS: bit10 が方向) → 現在速度 58 (bit15 が方向) | 1 step/s = 360/4096 deg/s (STS, 0.0146rpm), 300/1024 deg/s (SCS). STS は ±32767 step/s, SCS は ±1023 step/s (約 ±50rpm) に制限 |
| PMX | [制御モードを速度に, 応答データ (502) を現在速度に (変化時, RAM)] → [トルクON] → MotorWRITE 速度 (int16) | 1 = 0.1 deg/s (Meridim の値 × 0.6) |

「必要時」= 記憶したトルク状態が1でないか, 送り直しのフレーム (PMX は記憶したトルク状態が1でないときだけ). 通常に戻すと PMX は位置制御に戻す. ICS, FUTABA, XBUS, Hitec, PWM は未対応 (−32768). 実機での確認は済んでいない (AX・SCS の単位を含む) 〔未検証〕.

---

## 13. PWMサーボ (本体ピン, PCA9685)

| 項目 | 仕様 |
|---|---|
| 方式 | `PWM_PIN_MODE` 1: 本体ピン 25/26 の2ch (LEDC ch8/9, 16bit, 50Hz). 2: I2C1 (Wire1, SDA25/SCL26) の PCA9685 を最大2枚 (`PCA9685_NUM`, 16ch×2) |
| 指令の元 | 出力ごとに対応表 `PWM_PIN_SLOT[2]` (本体ピン [0]=25, [1]=26), `PCA9685_SLOT[2][16]` ([枚][ch]) で Meridim のスロットを指定. `SV_L(n)` = L系統 n, `SV_R(n)` = R系統 n, `SV_NONE` = 使わない |
| 出力 | スロットのコマンドが1なら指令値 (角度) をパルス幅にして出力, それ以外はパルスを止める (脱力). 通常以外のモードのスロットは直前の出力を保つ. トリム・cw はそのスロットの設定 |
| パルス幅 | スロットごと (L は `IXL_*`, R は `IXR_*`). サーボ側の角度 a = トリム + 指令 × cw を `IXL_PWM_MIN_DEG`〜`IXL_PWM_MAX_DEG` (既定 −90〜90°) に制限し, パルス幅 = `IXL_PWM_CENTER` + a × (`IXL_PWM_P90` − `IXL_PWM_CENTER`) / 90 (`mrd_pwm_deg2us`). −90° は中央に対して対称 (2 × 中央 − +90° のパルス). +90° のパルスを中央より小さくすると逆向き. 結果は 0〜20000µs に収める. 既定 (中央 1500µs, +90° 2500µs, ±90°) は 500〜2500µs で, 版 1.23 までの全体共通の設定 (`PWM_PULSE_CENTER` など, 削除) の既定と同じ出力 |
| 更新周期 | 本体ピンは偶数フレーム (50Hz). PCA9685 は1枚目を偶数, 2枚目を奇数フレームで16ch 分をまとめて書く (各50Hz) |
| PCA9685 の初期化 | 応答確認 → SLEEP → PRESCALE (round(`PCA9685_OSC_HZ`/(4096×50)) − 1) → MODE2 = OUTDRV → MODE1 = 自動インクリメント → 再開 → 全ch OFF |
| PCA9685 の書き込みの失敗 | 書き込めなくなったら (ケーブルの外れ等) "PCA9685 0x40: no response. Retry every 1 s." を表示し, 書き込みを止める (毎回の I2C のタイムアウト待ちでフレームが遅れないため). 約1秒ごとに応答を確かめ, 戻ったら初期化からやり直して "restored." を表示する |
| スロットの種類 | 1 (本体PWM) / 11 (PCA9685) にすると UART の系統では扱わず, 指令値を返す. 通常のサーボのスロットを指定すれば, そのサーボと同じ指令で動く. `num_max` 以降のスロットを指定すると, [7-1] で指令値が計算されないため更新されない |
| 呼び出し | `mrd_servo_drive_lite()` の中, `mrd_servo_param_prepare()` の直後 (UART のサーボが現在位置で指令値を上書きする前). 単体モードでは呼ばれない (出力しない) |

---

## 14. IMU / AHRS

| 項目 | BNO055 (`BNO055_AHRS` 3) | MPU6050 (`MPU6050_IMU` 1) |
|---|---|---|
| 接続 | I2C0 (SDA22, SCL21, 400kHz), アドレス 0x28 | I2C0 |
| 初期化 | `bno.begin()` (NDOF), 外部水晶なし | DMP の初期化, 固定オフセット, 自動校正 (6回), DMP 開始 |
| 読み取り | Core0 スレッド. 読み取りの後 `IMUAHRS_INTERVAL` (10ms) 待つ (周期は 読み取り時間 + 10ms) | 同左 (新しい DMP パケットがあるとき) |
| 加速度 [2]〜[4] | m/s² ×100 | m/s² ×100 (1g = 8192 を換算) |
| 角速度 [5]〜[7] | deg/s ×100 | deg/s ×100 (±2000deg/s, 16.4 = 1deg/s を換算) |
| 地磁気 [8]〜[10] | µT ×100 | 0 |
| 温度 [11] | `getTemp()` (1℃単位) | `getTemperature()` / 340 + 36.53 (データシートの換算式) |
| ロール / ピッチ [12] [13] | オイラー角 (ロール = y, ピッチ = z) − 校正点 を −180〜180 に折り返し, deg ×100 | DMP の ypr (ロール = [2], ピッチ = [1]) − 校正点 を同様に |
| ヨー [14] | 方位 − 校正点 を −180〜180 に折り返し, deg ×100 | 同左 |

- 読み取りの失敗: BNO055 は読む前にチップID (0x00 = 0xA0) を読んで確かめる (ライブラリは失敗を返さないため). MPU6050 は新しいデータが 500ms 以上届かなければ失敗とする. 失敗している間は値を更新せず (前回の値のまま), 途絶えたとき "BNO055: no response. Sensor values are not updated." と, 戻ったとき "...: reading restored." を表示する.
- MPU6050 の換算は実機で確認していない 〔未検証〕.
- `MPU9250_IMU` (2) は未対応 (初期化で失敗を返す). `NO_IMU` (0) では [2]〜[14] を書かない (PC の値が返る).
- 初期化に失敗した場合はスレッドを開始せず, [2]〜[14] には 0 が入る.
- 校正: マスターコマンド 10003 でいまのロール・ピッチ・ヨーを, 10002 でいまのヨーだけを 0 にする (`mrd_wire0_deg_from_origin()` で引いて折り返す).
- スレッドは `ahrs.read[16]` を `ahrs_mux` の中で書き, メインループの `mrd_meriput90_ahrs()` が一括でコピーしてから ×100 する.

### 14.1 INA226 (電流・電圧センサ)

| 項目 | 仕様 |
|---|---|
| 設定 | `MOUNT_INA226` (0: なし 既定, 1: あり), `INA226_ADDR` (0x40. A0/A1 で 0x40〜0x4F), `INA226_SHUNT_MOHM` (シャント抵抗 mΩ. 既定 100.0), `INA226_INTERVAL` (読み取り間隔 ms. 既定 10) |
| 接続 | I2C0 (`Wire`, IMU と同じバス, `I2C0_SPEED`). アドレスは BNO055 (0x28) と重ならない. IMU がなければ INA226 のために I2C0 を開始する. I2C1 には PCA9685 (0x40 / 0x41) がいるので, つなぐなら I2C0 |
| 起動時 | メーカーID (0xFE = 0x5449) を確かめ, 設定 (0x00) に 0x4327 (平均4回, 変換 1.1ms, 連続測定. 約9ms ごとに更新) を書く. "INA226 0x40: found" か "... not found (retried while running)" を表示. 見つからなくてもスレッドを開始する |
| 読み取り | Core0 のスレッド `mrd_ina226_task` (`thp[1]`) が `INA226_INTERVAL` ごとにバス電圧 (0x02, 1.25mV/LSB) とシャント電圧 (0x01, 2.5µV/LSB) を読み, `ina226` に `ina226_mux` の中で書く. 電流はシャント電圧 ÷ シャント抵抗で計算する (校正レジスタは使わない) |
| Meridim | [86] = 電圧 (V × 100, `mrd_ina226_volt100`), [87] = 電流 (A × 100, ±32767 で飽和. 符号はシャントの向き, `mrd_ina226_amp100`). メインループの [4] で `mrd_meriput90_ina226()` が書く. `MOUNT_INA226` 0 では書かない (PC の値が返る). 電力は PC で 電圧 × 電流 として求める |
| 範囲 | 電圧 0〜36V. 電流 ±81.92mV ÷ シャント抵抗 (100mΩ で ±0.82A, 2mΩ で ±41A) |
| 異常 | 読めないあいだは値を更新しない (前回の値). 途絶えたとき "INA226: no response. Sensor values are not updated.", 戻ったとき "INA226: reading restored." を表示し, 戻ったら設定を書き直す 〔未検証〕 |

---

## 15. ジョイパッド

| 設定 `MOUNT_PAD` | 値 | 対応 |
|---|---|---|
| `PC` (= `NONE`) | 0 | PC が [15]〜[18] を書く (ボードは何もしない) |
| `MERIMOTE` | 1 | 未対応 |
| `BLUERETRO` | 2 | BlueRetro (別の ESP32 の Wii extension 出力を I2C1 で読む) |
| `SBDBT` | 3 | SBDBT (PS3 コントローラ用ファームウェアの RCB3 形式のシリアル. `PIN_PAD_RX` でソフトウェア受信) |
| `KRR5FH` | 4 | KRC-5FH (受信機 KRR-5FH を R系統の ICS に接続) |
| `WIIMOTE` | 5 | WIIMOTE + ヌンチャク (Bluetooth) |
| `WIIMOTE_C` | 6 | WIIMOTE + クラシックコントローラ / Pro (15.2節) |

### 15.1 KRC-5FH

- R系統の種類が ICS (43) のときだけ読む (`mrd_pad_krc_read(PAD_INTERVAL, ics_r)`). メインループ [5] で, 前回から `PAD_INTERVAL` ms 以上たったときだけ受信機に問い合わせる (それ以外は前回の値). 問い合わせは R系統の ICS のバスを同期的に使うため, その分フレームの時間が延びる. 〔未検証〕
- `PAD_GENERALIZE` 1: 左十字4つ同時押し → Select, 右側4つ同時押し → Start とし, 変換表 `PAD_TABLE_KRC5FH_TO_COMMON` で PS 系の配置にする. アナログ値は (値 − 62) × 4 を ±127 に収め, 他のパッドと同じく [16] の上位 byte に PA1 (X), 下位 byte に PA2 (Y), [17] の上位 byte に PA3 (X), 下位 byte に PA4 (Y) を入れる (右スティックを PA3/PA4 につないだ場合. 上に倒すと PA2/PA4 の値が増え, Y が正. 〔未検証〕). スティックの中心の校正は未実装.
- `PAD_GENERALIZE` 0: ボタンもアナログも受信機の生値. アナログは PA1〜PA4 の値 (0〜127, 中央約62) をそのまま1 byte ずつ, 1 と同じ並び ([16] 上位 = PA1, 下位 = PA2, [17] 上位 = PA3, 下位 = PA4) で入れる.
- 受信に失敗したフレームはボタン・アナログとも 0.

### 15.2 WIIMOTE

- 起動時に `mrd_bt_setup()` で接続を待つ (BT LED が点滅, 最大 `PAD_INIT_TIMEOUT`). 接続で BT LED を点灯, タイムアウトで消灯. 失敗してもスレッドは開始する 〔制約 K-2〕. 〔未検証〕
- `mrd_bt_wiimote_task` (Core0) が `PAD_INTERVAL` ごとに読み, `pad_bt_val` に排他して書く. メインループは `mrd_pad_read()` で排他して読む.
- 新しいデータがないときは前回の値を返し続ける (押し続けているボタンを保つ). 切断されると (同梱ライブラリに追加した `ESP32Wiimote::connected()` で判定), ボタンとスティックの値を 0 にする. 判定は Bluetooth の切断の通知によるため, 電波が届かなくなった場合はリンクの監視時間が切れるまで (数秒〜数十秒) 前の値が残る. 再接続後も次のデータが届くまでは 0. 〔未検証〕
- ボタンは `PAD_GENERALIZE` 1 なら `PAD_TABLE_WIIMOTE_SOLO`, 0 なら `PAD_TABLE_WIIMOTE_ORIG` で変換. ヌンチャクの C → L1 (0: ○), Z → R1 (0: ×). HOME でスティックの中心を校正する. HOME はボタンにも割り当てられる (SOLO では bit1, ORIG では bit15).
- スティック: [16] の上位 byte = X, 下位 byte = Y. それぞれヌンチャクのスティックの中心からの差を ±127 に収めた値 (KRC-5FH と同じく1 byte ずつ).
- **クラシックコントローラ** (`MOUNT_PAD` WIIMOTE_C): WIIMOTE に差したクラシックコントローラ / クラシックコントローラ Pro を読む. 同梱のライブラリが識別 (`.. .. A4 20 .. 01`) し, 拡張の6byte (標準のデータ形式1. 互換品でも使える形式) を報告させる. `mrd_wiiext_6to8()` で 8byte の形 (各軸8bit) に直し, BlueRetro と同じ `mrd_wiiext_to_pad()` で値にする: ボタンの割り当て, スティック [16] 左 / [17] 右, [18] アナログトリガーは15.4節と同じ (左スティックは6bit, 右スティックとトリガーは5bit の精度). クラシックコントローラがつながっていなければ WIIMOTE と同じ (本体のボタンとヌンチャク). 切断の扱いも WIIMOTE と同じ. 〔未検証〕

### 15.3 SBDBT

- SBDBT (ランニングエレクトロニクス) に「PS3 コントローラ / USB ゲームパッド UART 変換ファームウェア」を RCB3 形式 (`RCB4DATA` 0) で書き込んで使う. SBDBT の TX を `PIN_PAD_RX` (既定 GPIO34) に, GND を共通にする (3.3V の信号なので直結). 通信速度は `PAD_SBDBT_BPS` (既定 2400) をファームウェアの `BAUDRATE1` と合わせる.
- 電文: 8byte. [0] 0x80, [1][2] ボタン, [3]〜[6] アナログ (0〜127, 中央 0x40. 左X, 左Y, 右X, 右Y), [7] チェックサム ([1]〜[6] の和の下位7bit). チェックサムの合わない電文と, 枠の誤り (ストップビットが LOW) の文字は捨てる. 0x80 で電文の先頭を合わせ直す.
- **受信の方法:** ESP32 のハードウェア UART は3つともふさがっている (USB, サーボ L / R) ため, `PIN_PAD_RX` の信号の変化を割り込み (`mrd_sbdbt_isr`) で時刻と状態をリングバッファ (256) に記録し, メインループ [5] の `mrd_pad_sbdbt_read()` でビットを読み取って文字と電文に組み立てる. 各ビットはスタートビットの始まりからの時刻で, ビットの中央の状態を読む. サーボ通信で割り込みを止める区間は通常 約80µs (FEETECH の READ @1Mbps) で, 2400bps (1ビット 417µs) では読み取りに影響しない (ホストテストで, 変化の時刻を最大80µs遅らせても読めることを確認). ただし Hitec の区切り受信 (約1.15ms) や, 低い通信速度で長いパケットを送る系統 (返信のある送信は送信の完了まで割り込みを止める) では1ビットを超えて止まるため, 文字を取りこぼすことがある. 通信速度を上げるほど余裕は減る (9600bps で1ビット 104µs). 読み出しが追いつかずリングバッファがあふれたときは捨てて数え直す. 〔未検証〕
- `PAD_GENERALIZE` 1: [1] の bit0〜4 = □ L1 L2 R1 R2, [2] の bit0〜6 = ↑ ↓ → ← △ × ○ を PS 系の配置にする. ↑↓の同時押しは Start, →←の同時押しは Select (SBDBT の電文の決まり). L3・R3・PS ボタンは RCB3 形式にない. 0: ボタンは [1] を上位 byte, [2] を下位 byte にした生値.
- スティック: [16] = 左, [17] = 右. それぞれ上位 byte = X, 下位 byte = Y で, (値 − 64) × 2 を ±127 に収める. [18] (L2/R2 のアナログ) は 0.
- 新しい電文が届かない間は前回の値を返す. `PAD_SBDBT_TIMEOUT` (ms) が 0 でなければ, その時間正しい電文が届かないとき 0 を返す (SBDBT のファームウェアが変化のあったときだけ送る設定 `SEND_ON_DIFFERENT_DATA` 1 では, 0 のまま使うこと).

### 15.4 BlueRetro

- BlueRetro (別の ESP32 に書き込むファームウェア. Bluetooth のコントローラを昔のゲーム機の信号に変える) の **Wii extension 出力** を読む. BlueRetro 側には `BlueRetro_hw1_wii_extension.bin` を書き込む (Espressif の Flash Download Tool で 0x1000 bootloader, 0x8000 partition-table, 0xD000 ota_data_initial, 0x10000 本体). 確認した版: v25.04.
- 配線: つなぐ I2C を `PAD_BLUERETRO_I2C` で選ぶ. **0 = I2C0** (`Wire`, GPIO22/21, `I2C0_SPEED`. IMU と同じバス. 新しいピンは不要で LED も残る): BlueRetro の SCL (GPIO25) → `PIN_I2C0_SCL` (21), SDA (GPIO26) → `PIN_I2C0_SDA` (22). **1 = I2C1** (既定. `Wire1`, `PIN_I2C1_SDA/SCL` 既定 25/26, `I2C1_SPEED`. PCA9685 と同じバスに並べられる): SCL → `PIN_I2C1_SCL` (26), SDA → `PIN_I2C1_SDA` (25). どちらも GND 共通. アドレス 0x52 は BNO055 (0x28), MPU6050 (0x68), PCA9685 (0x40/0x41) と重ならない.
- I2C1 につなぐとき, 既定のピンのまま (25/26) では, エラーLED と BT LED はなくなる (`mrd_pin_err_led()` / `mrd_pin_led_bt()` が −1). `PIN_ERR_LED` / `PIN_LED_BT` を空いているピンにすれば LED を残せる. 本体PWM (`PWM_PIN_MODE` 1) とピンが重なるときは "BlueRetro: I2C1 pins are used by the body PWM ..." を表示して開始しない. `PIN_I2C1_SDA/SCL` を他のピン (例: 16/17) にすれば LED と本体PWM を残せる.
- **読み取り:** Core0 のスレッド `mrd_blueretro_task` が `PAD_INTERVAL` ごとに `mrd_blueretro_read()` を選んだ I2C (`Wire` / `Wire1`) で呼び, `pad_bt_val` に排他して書く (WIIMOTE と同じ). メインループは待たない. I2C0 では IMU のスレッド, I2C1 ではメインループの PCA9685 の書き込みと, Wire が排他する.
- **初期化** (`mrd_wiiext_init`): 0xF0 = 0x55, 0xFB = 0x00 (暗号化なし), 0xFE = 0x03 (8byte の形式) を書き, 識別 (0xFA〜) が `.. .. A4 20 03 01` (クラシックコントローラ, 8byte の形式) かを確かめる. 応答がない・識別が違うときは 0 を返し, 次の読み取りで初期化をやり直す.
- **読み取り:** レジスタ番号 0x00 を書き, 200µs 待ってから 8byte を読む. [0] 左X [1] 右X [2] 左Y [3] 右Y (中央 0x80, ±0x66), [4] L トリガー [5] R トリガー (0x16〜), [6][7] ボタン (押すと 0). [6] の bit0 が 0 なら 8byte の形式ではない (BlueRetro の再起動など) ので 0 を返して初期化をやり直す. 読めないとき (BlueRetro の電源断など) も 0.
- **ボタン** (`PAD_GENERALIZE` 1): A / B / X / Y → ○ / × / △ / □, ZL / ZR → L1 / R1, L / R → L2 / R2, + / − → Start / Select, HOME → `PAD_HOME`, 十字キーはそのまま (BlueRetro の既定の割り当てで PS 系のコントローラのボタンに戻る形). L3 / R3 は BlueRetro が L / R にまとめるため区別できない. 0: Wii のボタンの2byte を反転 (押すと 1) した値.
- **スティック:** [16] 左, [17] 右. 上位 byte = X, 下位 byte = Y で, (値 − 0x80) × 127 / 0x66 を ±127 に収める (Y は上が正). [18]: 上位 byte = L トリガー, 下位 byte = R トリガー, (値 − 0x16) を 0〜127 に収める.
- BlueRetro の割り当て・モードは BlueRetro の BLE の設定画面 (Web) で変えられる. 〔未検証〕 (ESP32 同士の I2C, 200µs の待ちで足りるか, PCA9685 との同居)

### 15.5 Meridim への書き込み (`mrd_meriput90_pad`)

- ボタン [15]: `PAD_BUTTON_MARGE` 1 なら PC の値との論理和, 0 なら上書き.
- アナログ [16]〜[18]: 上書き (WIIMOTE (ヌンチャク) の [17] と, BlueRetro・WIIMOTE_C 以外の [18] は 0).
- 未対応のパッドを指定した場合, [16]〜[18] は 0 で上書きされる.

---

## 16. EEPROM と SDカード

### 16.1 考え方

- config.h と keys.h の設定 (基板の配線と配列の大きさなどを除く, 16.6節) は, すべて EEPROM に保存できる. どちらの値を使うかは **ページごとに選ぶ** (`EEPROM_LOAD`).
- 設定値は起動の最初に1つの構造体 `MrdSettings cfg` (main.h) に決まり, コードはマクロではなく `cfg` の値を使う (20.3節). 既定値は config.h / keys.h. スロットの設定 (`IXL_MT` など), PWM のスロットごとの設定 (`IXL_PWM_CENTER` など) と PWM の対応表 (`PWM_PIN_SLOT`, `PCA9685_SLOT`) は config.h の配列をそのまま実行時の値に使う.
- **1ページ = Meridim90 1つ分 (90語, 180byte).** ページ単位で Meridian 通信 (10200+p / 10300+p) で読み書きできる.
- `EEPROM_SIZE` = 2880byte = 16ページ. 使うのは P0〜P7 で, P8〜P15 は将来用の予備 (書かない). ESP32 の EEPROM はフラッシュの NVS 領域 (`no_ota.csv` で 20KB. WiFi と共用) に1つのまとまりとして置かれ, 書き換え時は新旧2つ分の空きが要るため, 3KB 程度に抑えている. 作業用の配列 (`mrd_eep_buf`, 2880byte) はスタックに置かない.

### 16.2 ページの形式

| 語 | EEPROM の中 | 通信 (Meridim) の中 |
|---|---|---|
| [0] | ページの印 `0x4D00 + ページ番号` | マスターコマンド (10200+p / 10300+p) |
| [1] | 配置の版 (ページごと. P4 は 4, P0 は 3, P5〜P7 は 2, P1〜P3 と P8〜 は 1) | 連番 |
| [2]〜[87] | データ (86語) | データ |
| [88] | 予備 (0) | エラー |
| [89] | ページのチェックサム (Meridim と同じ計算) | チェックサム |

- **配置の版:** ページの中の語の並びを変えたら, そのページの版を上げる (`MRD_EEP_VERSIONS`). 古い並びのデータは版が合わず無効になり, 誤って別の項目として読むことはない (config の値を使う. `EEPROM_SET` 1 なら次の起動で新しい並びで書き直す). 版 1.25 で P0 と P4〜P7 を並べ直して 2 に, 版 1.27 で P0 (システム → ネットワーク) と P4 (ネットワーク → システム) を入れ替えて, この2ページを 3 にした. P1〜P3 は 1 のままで, 保存済みの値をそのまま使える.
- **有効なページ** = 印・版・チェックサムが合う. 空 (すべて 0xFF) や壊れたページは使わない (config の値).
- **旧形式:** P1 だけは, 印がなくても空でなければ版 1.4 以前の形として読む (今までに保存したトリムを失わないため. 語の位置は同じ).
- 値は int16. 小数は決まった倍率の整数, 32bit の値は2語 (下位, 上位), 文字列・IP・MAC は byte の並びのまま (IP は a, b, c, d の4byte, MAC は6byte).
- **読み込むときの範囲の確認:** 項目ごとに範囲を決めており (`MRD_EEP_FIELDS` の min / max), 範囲外の値は使わず config の値のままにして, 起動時に数を表示する. 語と設定値の対応は mrd_eeprom.h の項目表 `MRD_EEP_FIELDS` が正 (下の表と同じ).

### 16.3 ページの内容

#### P0 ネットワーク (WiFi の SSID・パスワードなど)

| 語 | 名前 | 型 |
|---|---|---|
| 2 | `MODE_ETHER` | 0/1 |
| 3 | `MODE_FIXED_IP` | 0/1 |
| 4 | `UDP_SEND_PORT` | ポート (符号なし) |
| 5 | `UDP_RECV_PORT` | ポート (符号なし) |
| 6, 7 | `WIFI_SEND_IP` | IP (2語) |
| 8〜13 | `FIXED_IP_ADDR` / `_GATEWAY` / `_SUBNET` | IP × 3 |
| 14〜21 | `ETHER_LOCAL_IP` / `_GATEWAY` / `_SUBNET` / `_DNS` | IP × 4 |
| 22〜24 | `ETHER_MAC` | 6byte (3語) |
| 25〜40 | `WIFI_AP_SSID` | 文字列 32byte (UTF-8. 足りない分は 0 で埋める. 32byte ちょうどなら終端の 0 なし) |
| 41〜72 | `WIFI_AP_PASS` | 文字列 64byte (同上. WPA2 は 8〜63 文字, または 16進 64 文字) |
| 73〜87 | 予備 | |

文字列は, 1語に2文字 (byte) を, Meridim の byte の並び (`bval[2k]`, `bval[2k+1]`) のまま入れる.

**SSID とパスワードの扱い:**

| 場面 | 動作 |
|---|---|
| EEPROM に保存 | する (`EEPROM_SET` 1 で P0 が config のときは, keys.h の値を書く) |
| PC へ送る (10200), PC から書く (10300) | 他の項目と同じく, そのまま送る・書く (伏せない). 読んだ P0 をそのまま書き戻せば, パスワードもそのまま残る |
| 起動時の EEPROM の表示 (`EEPROM_DUMP`) | そのまま表示する |
| 使う値 | P0 が EEPROM のとき, EEPROM の SSID が空でなければ EEPROM の SSID・パスワード. 空なら keys.h |

- 注意: Meridian 通信は暗号化がないため, 10200 / 10300 ではパスワードが平文で流れる. また EEPROM はフラッシュに暗号化せずに置かれるため, ボードを手元で読み出せばパスワードを取り出せる (keys.h をビルドしたファームウェアと同じ). 配布・譲渡の前は, P0 を消すか keys.h と同じく値を消すこと.

#### P1 サーボのスロット (Meridian_console と同じ形)

| 語 | 内容 |
|---|---|
| 20 + 2i (L), 50 + 2i (R) | 設定ワード: bit0 マウント, bit1〜7 ID の下位7bit, bit8 回転方向 (1 = 正, 0 = 逆). bit9〜15 は予備 (0) |
| 21 + 2i (L), 51 + 2i (R) | トリム deg ×100 |
| 2〜19, 80〜87 | 予備 |

- マウント (bit0): 1 = config のマウントのまま, 0 = そのスロットを外す (`cfg.ixl_unmount`). **P1 で新しくマウントすることはない** (config で種類が 0 / PWM 専用のスロットは扱わない). ただし P2 を読み込むとスロットの種類 (`IXL_MT` / `IXR_MT`) が変わるため, config で種類が 0 のスロットも P2 の種類でマウントされる.
- config から作るとき, マウント = 種類が 0 でも PWM 専用 (1/11) でもなく, 外していないこと.

#### P2 スロットの拡張

| 語 | 内容 |
|---|---|
| 20 + 2i (L), 50 + 2i (R) | スロットの種類 (`IXL_MT` / `IXR_MT`: 0, 1, 11, 21, 31, 32, 43, 44, 51, 61, 62, 71) |
| 21 + 2i (L), 51 + 2i (R) | ID (0〜253, 8bit まで) |
| 2〜19, 80〜87 | 予備 (将来: スロットごとの角度の上下限など) |

- ID の決め方: P2 の ID の下位7bit が P1 の ID と同じなら P2 の ID (128 以上もそのまま使える). 違えば P1 の ID (Meridian_console で変えた ID. 0〜127).

#### P3 サーボの系統と共通設定

| 語 | 名前 | 型・倍率 | 既定 |
|---|---|---|---|
| 2, 3 | `MOUNT_SERVO_TYPE_L` / `_R` | 種類 | config の値 |
| 4, 5 | `MOUNT_SERVO_TYPE_L2` / `_R2` | 種類 | 0 / 0 |
| 6, 7 | `SERVO_BAUDRATE_L` | bps (32bit) | 1250000 |
| 8, 9 | `SERVO_BAUDRATE_R` | bps (32bit) | 1250000 |
| 10, 11 | `SERVO_BAUDRATE_L2` | bps (32bit) | 1000000 |
| 12, 13 | `SERVO_BAUDRATE_R2` | bps (32bit) | 1000000 |
| 14, 15 | `SERVO_TIMEOUT_L` / `_R` | ms | 2 / 2 |
| 16, 17 | `SERVO_ECHO_L` / `_R` | 0/1/2 | 基板の設定 |
| 18 | `SERVO_TX_RELEASE` | 0/1 | 基板の設定 |
| 19 | `SERVO_LOST_ERR_WAIT` | 回 | 6 |
| 20 | `SERVO_STEP_MAX` | deg ×100 | 1500 |
| 21 | `SERVO_UNMOUNT_REPLY` | 0/1 | 0 |
| 22 | `SERVO_MOTION_FRAMES_MAX` | フレーム | 1000 |
| 23 | `SERVO_FTC_STS_RANGE` | deg ×10 | 3600 |
| 24 | `SERVO_FTC_SCS_RANGE` | deg ×10 | 3000 |
| 25 | `SERVO_FTC_SCS_SPEED` | 位置値/s | 1500 |
| 26 | `SERVO_FTC_FIX_RETURN_DELAY` | 0/1 | 1 |
| 27 | `SERVO_DXL_FIX_RETURN_DELAY` | 0/1 | 1 |
| 28 | `SERVO_XBUS_ANGLE180` | 0/1 | 0 |
| 29 | `SERVO_HTC_PER_FRAME` | 台 | 1 |
| 30 | `SERVO_HTC_US_PER_DEG` | µs/deg ×100 | 1000 |
| 31 | `SERVO_HTC_SPEED` | 1〜255 | 255 |
| 32 | `SERVO_FTB_READ_PER_FRAME` | 台 | 8 |
| 33〜87 | 予備 | | 0 |

#### P4 システム・動作モード

語をグループに分け, グループごとに予備を置く (項目を足すときは, そのグループの予備を使う).

| 語 | 名前 (config) | 型・倍率 | 既定 |
|---|---|---|---|
| **2〜19** | **動作モード・UDP** | | |
| 2 | `FRAME_DURATION` | ms | 10 |
| 3 | `CHARGE_TIME` | ms | 200 |
| 4 | `MOUNT_SD` | 0/1 | 1 |
| 5 | `MOUNT_IMUAHRS` | 種類 (0〜3) | 3 |
| 6 | `MOUNT_PAD` | 種類 (0〜6) | 0 |
| 7 | `MODE_ESP32_STANDALONE` | 0/1 | 0 |
| 8 | `MODE_UDP_RECEIVE` | 0/1 | 1 |
| 9 | `MODE_UDP_SEND` | 0/1 | 1 |
| 10 | `UDP_TIMEOUT_US` | µs | 4000 |
| 11 | `UDP_POLL_US` | µs | 100 |
| 12 | `UDP_FAILSAFE_MS` | ms | 1000 |
| 13 | `UDP_SELF_SPEED` | deg/s ×10 | 600 |
| 14 | `CHECK_SD_RW` | 0/1 | 1 |
| 15 | `MODE_UDP_PASSIVE` | 0/1 | 0 |
| 16〜19 | 予備 | | 0 |
| **20〜29** | **モニタ (表示)** | | |
| 20 | `MONITOR_FRAME_DELAY` | 0/1 | 0 |
| 21 | `MONITOR_FLOW` | 0/1 | 0 |
| 22 | `MONITOR_ERR_ALL` | 0/1 | 0 |
| 23 | `MONITOR_SEQ` | 0/1 | 0 |
| 24 | `MONITOR_UDP_TIMEOUT` | 0/1 | 0 |
| 25 | `MONITOR_SUPPRESS_DURATION` | ms | 8000 |
| 26〜29 | 予備 | | 0 |
| **30〜39** | **シリアル・I2C0・IMU** | | |
| 30, 31 | `SERIAL_PC_BPS` | bps (32bit) | 921600 |
| 32, 33 | `I2C0_SPEED` | Hz (32bit) | 400000 |
| 34 | `IMUAHRS_INTERVAL` | ms | 10 |
| 35 | `MOUNT_INA226` | 0/1 | 0 |
| 36 | `INA226_ADDR` | 0x40〜0x4F | 0x40 |
| 37 | `INA226_SHUNT_MOHM` | mΩ ×10 | 1000 |
| 38 | `INA226_INTERVAL` | ms | 10 |
| 39 | 予備 | | 0 |
| **40〜59** | **パッド** | | |
| 40 | `PAD_INIT_TIMEOUT` | ms | 10000 |
| 41 | `PAD_INTERVAL` | ms | 10 |
| 42 | `PAD_BUTTON_MARGE` | 0/1 | 1 |
| 43 | `PAD_GENERALIZE` | 0/1 | 1 |
| 44, 45 | `PAD_SBDBT_BPS` | bps (32bit) | 2400 |
| 46 | `PAD_SBDBT_TIMEOUT` | ms | 0 |
| 47 | `PAD_BLUERETRO_I2C` | 0/1 | 1 |
| 48〜59 | 予備 | | 0 |
| 60〜87 | 予備 | | 0 |

#### P5 PWM の共通設定と対応表

| 語 | 名前 | 型・倍率 | 既定 |
|---|---|---|---|
| 2 | `PWM_PIN_MODE` | 0/1/2 | 0 |
| 3, 4 | `I2C1_SPEED` | Hz (32bit) | 400000 |
| 5 | `PCA9685_NUM` | 枚 (0〜2) | 2 |
| 6, 7 | `PCA9685_ADDR_0` / `_1` | I2C アドレス | 0x40 / 0x41 |
| 8, 9 | `PCA9685_OSC_HZ` | Hz (32bit) | 25000000 |
| 10, 11 | `PWM_PIN_SLOT[0]` / `[1]` | スロット (SV_L(n) = n, SV_R(n) = 100 + n, なし = −1) | −1 |
| 12〜27 | `PCA9685_SLOT[0][0〜15]` | 同上 | −1 |
| 28〜43 | `PCA9685_SLOT[1][0〜15]` | 同上 | −1 |
| 44〜87 | 予備 | | 0 |

#### P6 PWM の L系統のスロット / P7 PWM の R系統のスロット

P6 は L系統 (`IXL_*`), P7 は R系統 (`IXR_*`) で, 形は同じ (P1 と同じく系統ごとに1ページ).

| 語 | 名前 (P6 / P7) | 型・倍率 | 既定 |
|---|---|---|---|
| 2〜16 | `IXL_PWM_CENTER[0〜14]` / `IXR_PWM_CENTER[0〜14]` | µs (0〜20000) | 1500 |
| 17〜31 | `IXL_PWM_P90[0〜14]` / `IXR_PWM_P90[0〜14]` | 同上 | 2500 |
| 32〜46 | `IXL_PWM_MAX_DEG[0〜14]` / `IXR_PWM_MAX_DEG[0〜14]` | deg ×100 | 9000 |
| 47〜61 | `IXL_PWM_MIN_DEG[0〜14]` / `IXR_PWM_MIN_DEG[0〜14]` | 同上 | −9000 |
| 62〜87 | 予備 | | 0 |

- 配列の形は項目表の `MRD_EEP_INTS` (パルス幅) と `MRD_EEP_FLOATS` (角度. float の配列を ×100 の整数で).

### 16.4 起動時の処理 (`mrd_settings_setup` → `mrd_eeprom_settings_boot`)

起動の **最初** (シリアルとサーボの開始より前) に行う. 表示はシリアルの開始後 (`mrd_eeprom_boot_report`).

1. `EEPROM.begin(EEPROM_SIZE)`. 失敗したらすべて config の値.
2. P0〜P7 のそれぞれについて, `EEPROM_LOAD` で選んでいて有効 (P1 は旧形式も可) なら「EEPROM の値を使う」. 選んだのに空・壊れていれば config の値 (表示する).
3. `EEPROM_SET` 1 (既定) なら, EEPROM の値を使わないページを config の値で作り, 内容が違うページだけ書く (`EEPROM_PROTECT` 1 なら書かない). EEPROM の値を使うページは書かない.
4. EEPROM の値を使うページを P0 から順に `cfg` と config の配列に反映する. 範囲外の値は反映しない.
5. 決めた値を, フラグ (`flg.udp_receive_mode` など), サーボのバス (`configure`) とプロトコル層 (`set_baud`. 通信速度は `mrd_line_baud()` で決める) に反映する. これらはグローバルの生成時に config の値で作られているため. 表示の設定は `cfg.monitor_*` を直接使う. PCA9685 のアドレスは PWM の開始時 (`mrd_pwm_begin`) に `set_addr` で反映する.
6. シリアルの開始後, "Settings: P0 net=config, P1 servo=EEPROM, ..." のように表示する. `EEPROM_DUMP` 1 なら P0〜P7 を `EEPROM_STYLE` で表示する.

- 以後のサーボの起動 (`mrd_servo_setup`) は, EEPROM の値 (ID・マウント・種類・通信速度) で行う. 起動時の接続確認・機種の読み取りと, 実際に使う ID が食い違わない.
- 互換: 版 1.4 以前の 540byte の EEPROM は, 起動時に 2880byte に広がる (増えた部分は 0xFF = 空). P1 は旧形式として読める. P0, P2 は 0 で埋まっているため無効 (config の値). v1.1.1 の行0 (P0) は何も書かれていない (0) ため, P0 をネットワークにしても古いデータを誤って読むことはない.

### 16.5 動作中の読み書き (Meridian 通信)

| コマンド | 動作 |
|---|---|
| 10200 + p | ページ p の [2]〜[87] を PC に送る (10200 = P0 の SSID・パスワードもそのまま) |
| 10300 + p | PC の [2]〜[87] をページ p に書く (印・版・チェックサムはボードが付ける). 書き込み禁止なら書かず `MCMD_NAK` |
| 10101 | P1 のスロットの語 ([20]〜[79], `i < num_max`) だけを書く. 他の語と `num_max` 以降のスロットは 0. 他のページは変えない |
| 10100 / 10102 | P1 のトリムと回転方向だけを読み込む (`mrd_eeprom_servosettings_load`). EEPROM が空, または P1 が無効なら表示して読み込まない |
| 10400 | 再起動. 書いた設定 (P1 のトリム・回転方向以外) を反映する. 全スロットのコマンドが 0 のときだけ受け付ける (そうでなければ "restart refused ..." を表示して何もしない) |

- **反映の時期:** 書いたページのうち, すぐ反映するのは P1 のトリム・回転方向だけ (10300+1 の直後に読み込む). マウント・ID・種類・通信速度・ネットワークなどは, そのページを `EEPROM_LOAD` で選んでいれば次の起動から反映する. 起動時の初期化と食い違わせないため.
- 書き込み (`mrd_eeprom_write`): `EEPROM_PROTECT` 1 なら "EEPROM is protected. To unprotect, please set 'EEPROM_PROTECT' to false." を表示して書かない (false). 内容が違う byte だけを書いて `EEPROM.commit()`.

### 16.6 EEPROM に入れないもの

| 設定 | 理由 |
|---|---|
| `MRD_BOARD_SERVO_IF`, `PIN_*` | 基板の配線. コンパイル時に決まる (R系統のピンは `#if`) |
| `MRDM_LEN`, `MRD_L_ORIGIDX`, `MRD_R_ORIGIDX`, `MRD_SERVO_SLOTS`, `IXL_MAX`, `IXR_MAX` | 配列の大きさ・Meridim の構造. 変えられない |
| `EEPROM_*` | EEPROM の使い方そのもの |
| `MCMD_*`, `MRD_*` (Meridim の添字), `ERRBIT_*` | 通信の取り決め |
| `SERIAL_PC_TIMEOUT`, `IMUAHRS_STOCK`, `CHECK_EEPROM_RW`, `MONITOR_ERR_SERVO`, `MONITOR_PAD`, `PIN_RESET_LAN`, `SPI0_SPEED` | 未使用 (表示のみを含む) |

### 16.7 SDカード

- `MOUNT_SD` 1 なら `SD.begin(PIN_CHIPSELECT_SD)`.
- `CHECK_SD_RW` 1 なら `/test.txt` に乱数を書いて読み返し, 書いた値と比べて "OK." / "mismatch. SD card r/w failed." を表示し, 削除する (起動時の確認のみ). `SD.begin` に失敗しても確認を行い "Could not open SD test.txt file." を表示する.
- それ以外の用途では使わない.

---

## 17. シリアルモニタの表示

### 17.1 表示の設定 (`MONITOR_*`)

| 設定 | 既定 | 表示 |
|---|---|---|
| `MONITOR_FRAME_DELAY` | 0 | 100フレームごとに処理時間・サーボ通信時間・遅延数・応答待ち時間, 通信失敗があればその内訳とスロット (17.2節) |
| `MONITOR_FLOW` | 0 | 処理の段階 "[1]"〜"[13]", チェックサムの結果 |
| `MONITOR_ERR_ALL` | 0 | 毎フレーム "[ERR] es>pc:.. pc>es:.. es>ts:.. ts>es:.. tsSkp:.. esSkp:.. pcSkp:.." |
| `MONITOR_SEQ` | 0 | 受信のたびに "Seq ep/rv 予想/受信" |
| `MONITOR_UDP_TIMEOUT` | 0 | 受信がタイムアウトしたら "UDP timeout" (起動後 `MONITOR_SUPPRESS_DURATION` ms は出さない) |
| `MONITOR_ERR_SERVO`, `MONITOR_PAD` | 0 | 未使用 |

### 17.2 フレームとサーボ通信のモニタ (`MONITOR_FRAME_DELAY`)

```
Frame[us] busy 平均/最大 servo 平均/最大 late 遅延数/100 rxL 平均/最大 rxR 平均/最大
Fail L: echo 不足/不一致 reply なし/途中 | R: echo .../... reply .../... | slot L01:2 R03:1 ...
```

- busy = ループの先頭から [13-1] までの時間, servo = [8-1] の時間, rxL/rxR = 受信に切り替えてから受信し終わるまでの時間.
- 2行目は通信の失敗があったときだけ. slot は失敗回数 `err` が 0 でなかったフレームの数 (L/R とスロット番号).

### 17.3 起動時の表示 (順序)

充電 → 版と通信速度 → EEPROM (設定の出どころとダンプ) → サーボの通信速度 → 各プロトコルの接続表示 → 系統のプロトコル名とエコー → PWM (本体ピン / PCA9685) → マウントしたサーボの ID → サーボのリンク (`SERVO_LINK` に行があるとき) → SDカード → I2C・IMU → センサスレッド → WiFi/有線LAN の接続と IP → パッド → "-) Meridian -LITE- system on ESP32 now flows. (-" (ボード単体モードなら続けて "CAUTION: This is ESP32 Standalone Mode. ...").

### 17.4 主なエラー表示

| 種類 | 表示 | 動作 |
|---|---|---|
| 停止 | "Please Check '#define FIXED_IP_ADDR, FIXED_IP_GATEWAY, FIXED_IP_SUBNET' in 'keys.h'" | エラーLED 点滅で停止 (LED は `PIN_ERR_LED` が本体PWM・I2C1・SBDBT の受信と同じピンでないときだけ. 2.2節) |
| 停止 | "Please check '#define ETHER_MAC' in 'keys.h'" | 同上 |
| 停止 | "ERROR: Ethernet initialization failed. Check Ethernet hardware/config." / "... Fix ETHER_GATEWAY and restart" | 同上 |
| 続行 | "UDP lost for 1000 ms: all servos torque off (failsafe)." / "UDP restored: ..." (7.5節), "Wifi init TIMEOUT.", "Wiimote connection timed out.", "No BNO055 detected ...", "IMU/AHRS DMP Initialization FAILED!", "Card failed, or not present.", "UDP timeout" | 続行 |
| サーボのリンク | "Servo link: L04 = -L03" / "Servo link ignored (check SERVO_LINK): ..." (10.14節) | 使えない行は使わない |
| サーボの混在 | "UART_x servo mix: only two different types of 21/31/32/61/62 can be mixed. Mix disabled." / "... slot [i] type t is neither line type. Unmounted." | 混在なし / そのスロットを外す |
| コマンド | "cmd: ...[番号]", "All servos torque off." | — |

---

## 18. 設定画面 (Meridian_LITE_config.html)

| 項目 | 仕様 |
|---|---|
| 形式 | 外部依存のない単一の HTML. Chrome / Edge の File System Access API でフォルダを選び, `config.h` (必須) と `keys.h` (任意) を読み書きする |
| 基準のフォルダ | ページの場所 (URL の `./`) の最後のフォルダ名 (通常 `Meridian_LITE_for_ESP32`). 選んだフォルダの名前が違えば, その中の同名フォルダを使う. 既定の相対パスは `src/config.h`, `src/keys.h` (変更可. `..` は使えない) |
| 再接続 | 選んだフォルダをブラウザに保存し, 次回は許可があれば自動で読み込む |
| 読み込み | `#define 名前 値 // コメント` を解析. 読み取り専用になるのは, 同じ名前が複数ある (`#if` で分岐) 項目, 数値の項目なのに値が数値でないもの, 括弧なしで空白を含む値. 括弧で囲んだ式は1つの値として編集できる (例: `SERVO_ECHO_L/R` の選択肢). サーボの配列 `IXL/IXR_MT, _ID, _CW, _TRIM` は表で編集. `main.cpp` の `VERSION` を表示 |
| 編集できない項目 | `MCMD_*`, Meridim のキー, `ERRBIT_*`, `IXL_MAX/IXR_MAX`, `SV_*`, `PWM_PIN_SLOT`, `PCA9685_SLOT` (直接編集と案内), `PIN_TX_R/RX_R`・`SERVO_TX_RELEASE` (表示のみ) |
| 表示の切替 | 親の設定に応じて子の項目を表示 (例: `MODE_ETHER` で WiFi/有線の項目, サーボの種類で通信速度の選択肢と種類別の設定). `I2C0_SPEED` は I2C0 を使う機器 (IMU, INA226, `PAD_BLUERETRO_I2C` 0 の BlueRetro) のどれかがあるとき「周辺機器」に表示. 未使用の項目のうち6つ (`CHECK_EEPROM_RW`, `IMUAHRS_STOCK`, `MONITOR_ERR_SERVO`, `MONITOR_PAD`, `PIN_RESET_LAN`, `SERIAL_PC_TIMEOUT`) に「未使用」バッジ (`SPI0_SPEED` は起動時の表示に使うため付かない). スロットの種類が系統の種類と合わなければ警告 |
| 検査 | 書式: IP (4つの 0〜255), MAC (`xx:xx:xx:xx:xx:xx`), ポート (1〜65535), 整数 (10進数か `0x` の16進数. `INA226_ADDR`, `PCA9685_ADDR_0/1`, `SERVO_HTC_SPEED`). 変更した項目だけを検査し, 書式の誤りがあれば保存しない. 範囲: 数値と整数の項目の下限・上限は EEPROM に保存できる範囲 (16章の項目表. 小数は倍率で割った値. 例: `FRAME_DURATION` 1〜100, `SERVO_TIMEOUT_L/R` 1〜100, `SERVO_BAUDRATE_*` 1200〜5000000, `INA226_ADDR` 0x40〜0x4F, `INA226_SHUNT_MOHM` 0.1〜3276.7) に合わせ, 外なら項目の横に説明を出す (保存は止めない. ファームウェアは範囲外の EEPROM の値を使わず config.h の値を使う) |
| 保存 | 値の部分だけを書き換え, コメントの位置と改行コードを保つ. 読み込んだ後にファイルが変わっていれば確認. 保存直前のディスク上の config.h を `config.h.bak` に残す (保存のたびに上書き. keys.h の控えは作らない) |
| 非対応のブラウザ | config.h をファイル選択で開き, 保存はダウンロード (keys.h は編集できない) |
| サーボのリンク | 「サーボのリンク」のカードで `SERVO_LINK` の行を追加・削除し, フォロワー・向き・リーダーを選ぶ (スロットは部位と ID を添えて表示). ファームウェアの起動時の検査 (10.14節) と同じ規則で, 使われない行に理由を表示し, フォロワー・リーダーのスロットの種類が 0 なら注意を表示する. 保存では表の行だけを書き換える (変わらない行はコメントごと残し, 変えた行は書き直してコメントを消し, 減った行は消し, 増えた行は最後の行の後に足す. 0行なら使わない行を1つ書く). 両方 `SV_NONE` の行は使わない行として表に出さない. 1行に1組 `{F, L, D}` でない書き方 (複数行にまたがる行など) があれば編集できず, 直接編集を案内する |
| PWM のスロットごとの設定 | 「PWMサーボ」のカードの表で, 中央・+90° のパルス幅, 角度の上限・下限を編集する (−90° は計算して表示). 表に出すのは種類 1/11 のスロットと, `PWM_PIN_MODE` に合う対応表 (`PWM_PIN_SLOT` / `PCA9685_SLOT`) で指定したスロット (「すべてのスロットを表示」で全30スロット). 下限 > 上限, パルス幅が 0〜20000µs の外なら注意を表示 |
| 表示言語 | ヘッダ右のタブで日本語 / English を切り替える. 選んだ言語はブラウザに保存し, 未選択ならブラウザの言語 (ja 以外は English). 画面の文字列は日本語で書き, 表示時に英訳表 `EN` で置き換える (サーボ表の部位名は単語ごとに訳し, 訳せなければそのまま). 編集中の値と状態表示は切り替えても保つ. config.h / keys.h のコメントは翻訳しない |

---

## 19. 設定値一覧 (config.h / keys.h)

「画面」列: ○ = 設定画面で編集可, △ = 表示のみ, × = 項目なし. config.h の並び順に記載する. 「未使用」と書いた設定はコードから参照されていない 〔制約 K-9〕 (`SPI0_SPEED` は起動時の表示だけに使う).

### 19.1 基本設定

| 名前 | 既定 | 単位 | 値 | 意味 | 画面 |
|---|---|---|---|---|---|
| `MRD_BOARD_SERVO_IF` | 1 | — | 0 / 1 | サーボIF回路 (2.1節) | ○ |
| `MRDM_LEN` | 90 | 要素 | 90 | Meridim の長さ | ○ |
| `FRAME_DURATION` | 10 | ms | ≥ 1 | 1フレームの周期 | ○ |
| `CHARGE_TIME` | 200 | ms | ≥ 0 | 起動時のコンデンサ充電待ち | ○ |
| `MRD_L_ORIGIDX` | 20 | 添字 | — | L系統の先頭 | ○ |
| `MRD_R_ORIGIDX` | 50 | 添字 | — | R系統の先頭 | ○ |
| `MRD_SERVO_SLOTS` | 15 | 台 | — | 1系統のスロット数 | ○ |

### 19.2 搭載と動作モード

`MODE_*` の効果の詳細は5.3節.

| 名前 | 既定 | 値 | 意味 | 画面 |
|---|---|---|---|---|
| `MOUNT_SD` | 1 | 0 / 1 | SDカードの有無 | ○ |
| `MOUNT_IMUAHRS` | `BNO055_AHRS` | `NO_IMU`, `MPU6050_IMU`, `MPU9250_IMU` (未対応), `BNO055_AHRS` | IMU/AHRS の種類 | ○ |
| `MOUNT_PAD` | `PC` | `PC`, `KRR5FH`, `WIIMOTE`, `WIIMOTE_C`, `SBDBT`, `BLUERETRO` (`MERIMOTE` は未対応) | ジョイパッドの種類 | ○ |
| `MODE_ESP32_STANDALONE` | 0 | 0 / 1 | ボード単体モード | ○ |
| `MODE_UDP_RECEIVE` | 1 | 0 / 1 | PC からの受信 | ○ |
| `MODE_UDP_SEND` | 1 | 0 / 1 | PC への送信 | ○ |
| `MODE_UDP_PASSIVE` | 0 | 0 / 1 | 通信の周期 (0: ボード主導, 1: PC 主導. 5.3節) | ○ |
| `MODE_ETHER` | 0 | 0: WiFi, 1: 有線LAN | 通信方式 | ○ |
| `MODE_FIXED_IP` | 0 | 0 / 1 | WiFi で IP を固定 | ○ |
| `UDP_TIMEOUT_US` | 4000 | µs | 受信待ちの上限 | ○ |
| `UDP_POLL_US` | 100 | µs | 受信待ち中の確認間隔 | ○ |
| `UDP_FAILSAFE_MS` | 1000 | ms | 正しい受信がこの時間ないと全サーボを脱力する. 0 で無効 (7.5節) | ○ |
| `UDP_SELF_SPEED` | 60.0 | deg/s | `MODE_UDP_RECEIVE` 0 のとき, トルクON後にサーボを目標へ動かす速さの上限 (5.3節) | ○ (受信 OFF のときだけ表示) |

### 19.3 EEPROM, 起動時チェック, 表示

| 名前 | 既定 | 意味 | 画面 |
|---|---|---|---|
| `EEPROM_SIZE` | 2880 | EEPROM のサイズ (byte. 180 × 16ページ. 180 の倍数で 1440 以上) | ○ |
| `EEPROM_SET` | 1 | 起動時に, EEPROM の値を使わないページを config の値で書く (16.4節) | ○ |
| `EEPROM_PROTECT` | 0 | 1: EEPROM への書き込みを禁止 | ○ |
| `EEPROM_LOAD` | 0 | EEPROM の値を使うページ (ビットの和. 0 = すべて config, 255 = すべて EEPROM). `EEPROM_P_NET` 1, `EEPROM_P_SERVO` 2, `EEPROM_P_SLOT_EXT` 4, `EEPROM_P_LINE` 8, `EEPROM_P_SYSTEM` 16, `EEPROM_P_PWM` 32, `EEPROM_P_PWM_L` 64, `EEPROM_P_PWM_R` 128 | ○ (ページごとのチェック) |
| `EEPROM_DUMP` | 0 | 起動時に EEPROM の内容を表示 | ○ |
| `EEPROM_STYLE` | `Dec` | 表示形式 (`Bin`, `Hex`, `Dec`) | ○ |
| `CHECK_SD_RW` | 1 | 起動時に SD カードの読み書きを確認 | ○ |
| `CHECK_EEPROM_RW` | 0 | 未使用 | ○ (未使用) |
| `MONITOR_FRAME_DELAY` | 0 | 処理時間・通信失敗のモニタ (17.2節) | ○ |
| `MONITOR_FLOW` | 0 | 処理の段階を表示 | ○ |
| `MONITOR_ERR_SERVO` | 0 | 未使用 | ○ (未使用) |
| `MONITOR_ERR_ALL` | 0 | 全エラーの数を毎フレーム表示 | ○ |
| `MONITOR_SEQ` | 0 | シーケンス番号を表示 | ○ |
| `MONITOR_PAD` | 0 | 未使用 | ○ (未使用) |
| `MONITOR_UDP_TIMEOUT` | 0 | 受信のタイムアウトを表示 | ○ |
| `MONITOR_SUPPRESS_DURATION` | 8000 | 起動直後のタイムアウト表示を抑える時間 (ms) | ○ |

### 19.4 I2C, SPI, PC, パッド

| 名前 | 既定 | 単位 | 意味 | 画面 |
|---|---|---|---|---|
| `I2C0_SPEED` | 400000 | Hz | I2C0 (IMU) の速度 | ○ |
| `IMUAHRS_INTERVAL` | 10 | ms | IMU 読み取りスレッドの周期 | ○ |
| `IMUAHRS_STOCK` | 4 | 個 | 未使用 | ○ (バッジなし) |
| `MOUNT_INA226` | 0 | 0 / 1 | INA226 の搭載 (14.1節) | ○ |
| `INA226_ADDR` | 0x40 | — | INA226 の I2C アドレス (0x40〜0x4F) | ○ |
| `INA226_SHUNT_MOHM` | 100.0 | mΩ | シャント抵抗 | ○ |
| `INA226_INTERVAL` | 10 | ms | 読み取り間隔 | ○ |
| `SPI0_SPEED` | 6000000 | Hz | 表示のみ (SPI の設定には使わない) | ○ (バッジなし) |
| `SERIAL_PC_BPS` | 921600 | bps | USB シリアルの速度 (platformio.ini の monitor_speed と合わせる) | ○ |
| `SERIAL_PC_TIMEOUT` | 2000 | ms | 未使用 (ESP32 ではシリアルの接続を検出できないため待たない. 他のボードとの互換のために残す) | ○ (未使用) |
| `PAD_INIT_TIMEOUT` | 10000 | ms | WIIMOTE の接続待ち | ○ |
| `PAD_INTERVAL` | 10 | ms | パッドを読む間隔 (KRC-5FH は 4 推奨) | ○ |
| `PAD_BUTTON_MARGE` | 1 | — | 0: パッドのボタンで上書き, 1: PC の値との論理和 | ○ |
| `PAD_GENERALIZE` | 1 | — | ボタン配置を PS 系にそろえる | ○ |
| `PAD_SBDBT_BPS` | 2400 | bps | SBDBT の通信速度 (ファームウェアの `BAUDRATE1` と合わせる) | ○ (SBDBT のとき) |
| `PAD_SBDBT_TIMEOUT` | 0 | ms | SBDBT の電文が途絶えたらパッドの値を 0 にする時間 (0: しない) | ○ (SBDBT のとき) |
| `PAD_BLUERETRO_I2C` | 1 | — | BlueRetro をつなぐ I2C (0: I2C0. IMU と同じバス, 1: I2C1) | ○ (BlueRetro のとき) |

### 19.5 ピン

| 名前 | 既定 | 意味 | 画面 |
|---|---|---|---|
| `PWM_PIN_MODE` | 0 | ピン25/26 の用途 (0 LED, 1 本体PWM, 2 PCA9685用 I2C) | ○ |
| `PIN_ERR_LED` | 25 | エラーLED (−1: なし. 本体PWM・I2C1・SBDBT の受信と同じピンなら使わない) | ○ |
| `PIN_EN_L` / `PIN_TX_L` / `PIN_RX_L` | 33 / 27 / 32 | L系統 | ○ |
| `PIN_EN_R` | 4 | R系統の送受信切替 | ○ |
| `PIN_TX_R` / `PIN_RX_R` | 14 / 13 (版0: 17 / 16) | R系統 | △ |
| `PIN_CHIPSELECT_SD` | 15 | SD の CS | ○ |
| `PIN_CHIPSELECT_LAN` | 5 | W5500 の CS | ○ |
| `PIN_RESET_LAN` | 14 | 未使用 | ○ (未使用) |
| `PIN_I2C0_SDA` / `PIN_I2C0_SCL` | 22 / 21 | I2C0 | ○ |
| `PIN_LED_BT` | 26 | Bluetooth LED (同上) | ○ |
| `PIN_PWM_0` / `PIN_PWM_1` | 25 / 26 | 本体PWM | ○ |
| `PIN_I2C1_SDA` / `PIN_I2C1_SCL` | 25 / 26 | I2C1 (PCA9685, BlueRetro) | ○ |
| `PIN_PAD_RX` | 34 | SBDBT の受信ピン (入力専用ピン 34/35/36/39 を推奨. 12 は起動時の制約で不可) | ○ |

### 19.6 サーボ

サーボの種類の番号: 0 なし, 1 本体PWM (スロット専用), 11 PCA9685 (スロット専用), 21 FUTABA RS30x, 31 DYNAMIXEL 1.0, 32 DYNAMIXEL 2.0, 43 KONDO ICS3.5/3.6, 44 KONDO PMX, 51 JR XBUS, 61 FEETECH STS, 62 FEETECH SCS, 71 Hitec HSR.

| 名前 | 既定 | 単位 | 意味 | 画面 |
|---|---|---|---|---|
| `MOUNT_SERVO_TYPE_L` / `_R` | 43 / 43 | 種類 | 系統のサーボの種類 | ○ |
| `MOUNT_SERVO_TYPE_L2` / `_R2` | 0 / 0 | 種類 | 2種類目 (混在. 10.11節) | ○ |
| `SERVO_BAUDRATE_L` / `_R` | 1250000 / 1250000 | bps | 通信速度 (ICS 1.25M, FEETECH 1M, DXL はサーボの設定値, FUTABA 460.8k 推奨, XBUS 250k, Hitec 19.2k, PMX 3M 推奨) | ○ |
| `SERVO_BAUDRATE_L2` / `_R2` | 1000000 | bps | 2種類目の通信速度 | ○ |
| `SERVO_TIMEOUT_L` / `_R` | 2 | ms | 返信待ちのタイムアウト | ○ |
| `SERVO_ECHO_L` / `_R` | `(MRD_BOARD_SERVO_IF == 1)` | — | 送信エコー (0 なし, 1 あり, 2 起動時に判定) | ○ |
| `SERVO_TX_RELEASE` | `(MRD_BOARD_SERVO_IF == 1)` | — | 受信中に TX 出力を切る | △ |
| `SERVO_LOST_ERR_WAIT` | 6 | フレーム | 連続で失敗したら通信不能とする回数 | ○ |
| `SERVO_STEP_MAX` | 15.0 | deg | 1フレームの指令値の最大変化量 | ○ |
| `SERVO_UNMOUNT_REPLY` | 0 | — | マウントなしスロットの返信 (0: 0 を返す, 1: 受信値のまま) | ○ |
| `SERVO_MOTION_FRAMES_MAX` | 1000 | フレーム | Meridim[19] の上限 (0 で補間しない) | ○ |
| `SERVO_FTC_STS_RANGE` | 360.0 | deg | STS の位置 0〜4095 の角度範囲 | ○ |
| `SERVO_FTC_SCS_RANGE` | 300.0 | deg | SCS の位置 0〜1023 の角度範囲 | ○ |
| `SERVO_FTC_SCS_SPEED` | 1500 | 位置値/s | SCS の目標速度 (0 では動かない) | ○ |
| `SERVO_FTC_FIX_RETURN_DELAY` | 1 | — | 起動時に FEETECH の応答遅延を 0 にする | ○ |
| `SERVO_DXL_FIX_RETURN_DELAY` | 1 | — | 起動時に DYNAMIXEL の応答遅延を 0 にする | ○ |
| `SERVO_XBUS_ANGLE180` | 0 | — | XBUS の角度 (0: ±60°, 1: ±90°) | ○ |
| `SERVO_HTC_PER_FRAME` | 1 | 台 | Hitec の1フレームの送信/読み出し台数 (1台 約4.2ms) | ○ |
| `SERVO_HTC_US_PER_DEG` | 10.0 | µs/deg | Hitec の1°あたりのパルス幅 | ○ |
| `SERVO_HTC_SPEED` | 0xFF | — | Hitec の位置の読み出し (0xE9) のたびに一緒に送る速度 (1〜255. パラメータ系コマンド 53 で変えられる) | ○ |
| `I2C1_SPEED` | 400000 | Hz | I2C1 (PCA9685) の速度 | ○ |
| `PCA9685_NUM` | 2 | 枚 | PCA9685 の枚数 (0〜2) | ○ |
| `PCA9685_ADDR_0` / `_1` | 0x40 / 0x41 | — | PCA9685 のアドレス | ○ |
| `PCA9685_OSC_HZ` | 25000000 | Hz | PCA9685 の内蔵発振器の周波数 (個体差の補正用) | ○ |
| `SERVO_FTB_READ_PER_FRAME` | 8 | 台 | FUTABA の1フレームに位置を読む台数 | ○ |
| `IXL_MAX` / `IXR_MAX` | 15 / 15 | 台 | 系統の配列の長さ | × |
| `SV_L(n)` / `SV_R(n)` / `SV_NONE` | n / 100+n / −1 | — | PWM の対応表のスロット指定 | × |

### 19.7 サーボの配列 (スロットごとの設定)

| 配列 | 型 | 意味 |
|---|---|---|
| `IXL_MT[15]` / `IXR_MT[15]` | int | スロットのサーボの種類 (0 = マウントなし) |
| `IXL_ID[15]` / `IXR_ID[15]` | int | サーボの ID |
| `IXL_CW[15]` / `IXR_CW[15]` | int | 回転方向の補正 (+1 / −1) |
| `IXL_TRIM[15]` / `IXR_TRIM[15]` | float | トリム (deg) |
| `IXL_PWM_CENTER[15]` / `IXR_PWM_CENTER[15]` | int | PWM の 0° のパルス幅 (µs. 既定 1500) |
| `IXL_PWM_P90[15]` / `IXR_PWM_P90[15]` | int | PWM の +90° のパルス幅 (µs. 既定 2500. −90° は中央に対して対称) |
| `IXL_PWM_MAX_DEG[15]` / `IXR_PWM_MAX_DEG[15]` | float | PWM のサーボ側の角度の上限 (deg. 既定 90) |
| `IXL_PWM_MIN_DEG[15]` / `IXR_PWM_MIN_DEG[15]` | float | PWM のサーボ側の角度の下限 (deg. 既定 −90) |
| `PWM_PIN_SLOT[2]` | int | 本体PWM [0]=ピン25, [1]=ピン26 の指令元 |
| `PCA9685_SLOT[2][16]` | int | PCA9685 [枚][ch] の指令元 |
| `SERVO_LINK[][3]` | int | サーボのリンク {フォロワー, リーダー, 向き} (10.14節) |

既定値 (公式 v1.1.1 と同じ. L/R とも ICS の11台. 手元の config.h はロボットに合わせて変えて使う):

| 添字 | 部位 (L / R) | IXL_MT | IXL_ID | IXL_CW | IXL_TRIM | IXR_MT | IXR_ID | IXR_CW | IXR_TRIM |
|---|---|---|---|---|---|---|---|---|---|
| 00 | 頭ヨー / 腰ヨー | 43 | 0 | 1 | 0.0 | 43 | 0 | 1 | −4.28 |
| 01 | 肩ピッチ | 43 | 1 | 1 | −20.42 | 43 | 1 | 1 | 0.68 |
| 02 | 肩ロール | 43 | 2 | 1 | −103.55 | 43 | 2 | 1 | −89.41 |
| 03 | 肘ヨー | 43 | 3 | 1 | 0.85 | 43 | 3 | 1 | 0.0 |
| 04 | 肘ピッチ | 43 | 4 | 1 | 88.41 | 43 | 4 | 1 | 91.83 |
| 05 | 股ヨー | 43 | 5 | 1 | 0.0 | 43 | 5 | 1 | 0.0 |
| 06 | 股ロール | 43 | 6 | 1 | −2.0 | 43 | 6 | 1 | 1.0 |
| 07 | 股ピッチ | 43 | 7 | 1 | −17.12 | 43 | 7 | 1 | −21.42 |
| 08 | 膝ピッチ | 43 | 8 | 1 | −68.54 | 43 | 8 | 1 | −53.68 |
| 09 | 足首ピッチ | 43 | 9 | 1 | −25.7 | 43 | 9 | 1 | −24.12 |
| 10 | 足首ロール | 43 | 10 | 1 | 0.0 | 43 | 10 | 1 | 0.0 |
| 11〜14 | 追加 | 0 | 11〜14 | 1 | 0.0 | 0 | 11〜14 | 1 | 0.0 |

`PWM_PIN_SLOT`, `PCA9685_SLOT` は既定ですべて `SV_NONE`. `SERVO_LINK` は既定で使わない行 `{SV_NONE, SV_NONE, 1}` が1つ.

### 19.8 keys.h

keys.h の項目はこの表を正とする (7.4節は参照のみ). 扱いの注意は7.6節.

| 名前 | 既定 | 意味 | 画面 |
|---|---|---|---|
| `WIFI_AP_SSID` / `WIFI_AP_PASS` | (各自の値) | アクセスポイント | ○ |
| `WIFI_SEND_IP` | (各自の値) | 送信先 PC の IP | ○ (IP 検査) |
| `UDP_SEND_PORT` | 22222 | 送信先のポート | ○ |
| `UDP_RECV_PORT` | 22224 | ESP32 の受信ポート | ○ |
| `FIXED_IP_ADDR` / `FIXED_IP_GATEWAY` / `FIXED_IP_SUBNET` | "192.168.xxx.xxx" 等 | WiFi で IP を固定するときの値 | ○ |
| `ETHER_LOCAL_IP` / `ETHER_GATEWAY` / `ETHER_SUBNET` / `ETHER_DNS` | "192.168.xxx.xxx" / ... / "255.255.255.0" / "8.8.8.8" | 有線LAN. `ETHER_GATEWAY` は送信先 PC を兼ねる | ○ |
| `ETHER_MAC` | "xx:xx:xx:xx:xx:xx" | W5500 の MAC アドレス | ○ (MAC 検査) |

### 19.9 マスターコマンドと Meridim のキー

マスターコマンド `MCMD_*` は9章, Meridim のキー `MRD_*` と関節のキーは8.2節, エラービット `ERRBIT_*` は8.3節を参照. いずれも設定画面では編集しない. 「※LITEでは未使用」と注記した定義は, 他の Meridian ボードとの互換のために残している.

### 19.10 設定の組み合わせのルール

| ルール | 守らないと |
|---|---|
| `MOUNT_PAD` が `SBDBT` のとき, `PIN_PAD_RX` は他で使っていないピン (入力専用ピン 34/35/36/39 を推奨. 12 は使わない) | 受信できない. 12 では SBDBT の HIGH の出力で起動時のフラッシュの電圧設定が変わり起動しない |
| `MOUNT_PAD` を `KRR5FH` にするのは, `MOUNT_SERVO_TYPE_R` が 43 (ICS) のとき | パッドを読まず, [16]〜[18] は 0 で上書きされる (`PAD_BUTTON_MARGE` 0 なら [15] も. PC の値も消える) |
| LED のピン (`PIN_ERR_LED` / `PIN_LED_BT`) は, 本体PWM・I2C1 (PCA9685, BlueRetro)・SBDBT の受信と別のピンにする | その LED がなく, 停止しても点滅しない 〔制約 K-12〕 |
| `MOUNT_SERVO_TYPE_L2/R2` は 21/31/32/61/62 のうち, 1種類目と系列が違うもの (10.11節) | 警告して混在なしにする |
| スロットの種類 `IXL_MT` / `IXR_MT` は, 系統の種類 (1種類目か2種類目) か 1/11 (PWM) | 混在の系統では警告してマウントを外す. 混在なしの系統では警告せず系統の種類として扱う (FEETECH は書き換える. 10.12節) |
| PWM で動かすスロットは `PWM_PIN_SLOT` / `PCA9685_SLOT` で指定し, `PWM_PIN_MODE` を 1 / 2 に合わせる. スロットの種類 1 / 11 は「UART で扱わない」印で, 出力するかは対応表と `PWM_PIN_MODE` だけで決まる | 出力されず, 指令値だけを返す |
| `SERVO_BAUDRATE_*` は10.12節の一覧の値で, サーボの設定と同じ | 警告のみ. 通信できない |
| PMX は自動方向切替の RS-485 変換を介す. DXL2 はサーボを事前に 1Mbps 以上にする | 通信できない / 時間が足りない |
| `MRDM_LEN` は 90, `EEPROM_SIZE` は 180 の倍数で 1440 以上 | 他の値はコンパイルエラーか, 使えないページが出る |
| `FRAME_DURATION` を変えるときは, 受信待ちとサーボ通信が収まること (10.13節). `SERVO_FTB_READ_PER_FRAME`, `SERVO_HTC_PER_FRAME` も同じ | 遅れが続く |
| `MODE_UDP_RECEIVE` 0 は, PC なしでボード内の計算 ([7-2]) だけで動かすときに使う | PC からの指令とマスターコマンドを受けない. 起動の約0.5秒後に全サーボがトルクONになる (5.3節) |
| 10101 や 10300 + p で保存した値を使うときは, そのページを `EEPROM_LOAD` で選ぶ | 選ばないと使われず, `EEPROM_SET` 1 では次の起動で config の値に書き戻される |
| `MODE_FIXED_IP` は WiFi のときだけ意味がある (有線LANは常に固定IP) | — |
| `PCA9685_NUM` は 0〜2 | — |

---

## 20. データ構造 (列挙型, 構造体, グローバル変数, 定数)

### 20.1 列挙型

| 列挙型 | 値 |
|---|---|
| `UartLine` | `L` 0, `R` 1, `C` 2 |
| `ServoType` | `NOSERVO` 0, `PWM_S` 1, `PCA9685` 11, `FTBRSX` 21, `DXL1` 31, `DXL2` 32, `KOICS3` 43, `KOPMX` 44, `JRXBUS` 51, `FTCSTS` 61, `FTCSCS` 62, `HTCHSR` 71 |
| `ImuAhrsType` | `NO_IMU` 0, `MPU6050_IMU` 1, `MPU9250_IMU` 2 (未対応), `BNO055_AHRS` 3 |
| `PadType` | `NONE` / `PC` 0, `MERIMOTE` 1, `BLUERETRO` 2, `SBDBT` 3, `KRR5FH` 4, `WIIMOTE` 5, `WIIMOTE_C` 6 |
| `PadButton` | 8.6節 |
| `BinHexDec` | `Bin` 0, `Hex` 1, `Dec` 2 |
| `MrdEepPage` (mrd_eeprom.h) | `MRD_EEP_P_SYSTEM` 0, `MRD_EEP_P_SERVO` 1, `MRD_EEP_P_SLOT_EXT` 2, `MRD_EEP_P_LINE` 3, `MRD_EEP_P_PWM` 4, `MRD_EEP_P_NET` 5, `MRD_EEP_P_USED` 6 (使っているページの数) |
| `MrdEepKind` (mrd_eeprom.h) | `MRD_EEP_INT`, `MRD_EEP_U16`, `MRD_EEP_U32`, `MRD_EEP_FLOAT`, `MRD_EEP_INTS`, `MRD_EEP_STR`, `MRD_EEP_BYTES_IP`, `MRD_EEP_BYTES_MAC` (項目の値の形. 16.3節) |
| `MrdSlotMode` (mrd_servo.h) | `MRD_SLOT_NORMAL` 0, `MRD_SLOT_PARAM` 1, `MRD_SLOT_WHEEL` 2, `MRD_SLOT_THROUGH` 3 |
| `MrdHalfDuplexUart::EchoMode` | `ECHO_OFF` 0, `ECHO_ON` 1, `ECHO_AUTO` 2 |
| `MrdHalfDuplexUart::RxResult` | `RX_OK` 0, `RX_ECHO_SHORT` 1, `RX_ECHO_MISMATCH` 2, `RX_REPLY_NONE` 3, `RX_REPLY_PARTIAL` 4 |

### 20.2 共用体

| 共用体 | 要素 | 実体 |
|---|---|---|
| `Meridim90Union` | `sval[MRDM_LEN+4]`, `usval[MRDM_LEN+2]`, `bval[MRDM_BYTE+4]`, `ubval[MRDM_BYTE+4]` | `s_udp_meridim` (送信), `r_udp_meridim` (受信), `r_udp_meridim_last` (最後に正しく受信したもの) |
| `PadUnion` | `sval[5]`, `usval[5]`, `bval[10]`, `ubval[10]`, `ui64val` ([0] ボタン, [1] 左スティック, [2] 右スティック, [3] L2/R2) | `pad_array` |
| `EepromUnion` | `bval[2880]`, `saval[16][90]`, `usaval[16][90]` (ページ, 語) | `mrd_eep_buf` (作業用. 16.1節) |

### 20.3 構造体とその実体

`MrdFlags flg` (フラグ):

| フィールド | 初期値 | 意味 |
|---|---|---|
| `udp_board_passive` | `MODE_UDP_PASSIVE` | PC 主導か (5.1節). 10005 / 10006 で切り替え |
| `count_frame_reset` | false | フレームタイマーのリセットの指示 |
| `eeprom_write_mode` | false | EEPROM 書き込みモード (未実装. 参照なし) |
| `udp_receive_mode` | `MODE_UDP_RECEIVE` | 受信を行うか |
| `udp_send_mode` | `MODE_UDP_SEND` | 送信を行うか |
| `meridim_rcvd` | false | 受信が正常で連番が予想どおり (マスターコマンドの実行条件) |
| `udp_failsafe` | false | 受信が途絶えて全サーボを脱力している (7.5節) |

`MrdSq mrdsq` (シーケンス番号): `s_increment` (送信の連番), `r_expect` (受信の予想値).

`MrdSettings cfg` (設定値. main.h): config.h・keys.h の設定の実行時の値. 起動の最初に決まり, `EEPROM_LOAD` で選んだページは EEPROM の値になる (16章). フィールド名はマクロ名の小文字 (例: `SERVO_STEP_MAX` → `cfg.servo_step_max`, `WIFI_AP_SSID` → `cfg.wifi_ap_ssid`). 並びと型は 16.3節の表と同じ (32bit は `unsigned long`, 小数は `float`, IP・MAC は文字列). 加えて `ixl_unmount[15]` / `ixr_unmount[15]` (P1 でマウントを外したスロット). LED のピンは `mrd_pin_err_led()` / `mrd_pin_led_bt()` (`PIN_ERR_LED` / `PIN_LED_BT`. そのピンを本体PWM・I2C1 (PCA9685, BlueRetro の I2C1)・SBDBT の受信に使うときは −1).

`MrdTimer tmr` (ボード単体モードのサイン波): `count_loop` (0), `count_loop_dlt` (2, 1フレームの増分), `count_loop_max` (359999).

`MrdErr err` (エラーの数): `pc_esp` (受信エラー), `esp_skip` (連番のずれ). `esp_pc`, `esp_tsy`, `tsy_esp`, `tsy_skip`, `pc_skip` は LITE では増えない.

`AhrsValue ahrs` (IMU/AHRS):

| フィールド | 意味 |
|---|---|
| `bno` | BNO055 (アドレス 0x28, Wire) |
| `mpu6050`, `dev_status`, `packet_size`, `fifo_buffer[64]`, `q`, `gravity`, `aa`, `gyro` | MPU6050 と DMP の作業領域 |
| `ypr[3]` | [yaw, pitch, roll] |
| `yaw_origin` / `roll_origin` / `pitch_origin` | 校正点 (deg. 10003 で3つとも, 10002 でヨーだけ設定) |
| `yaw_source` / `roll_source` / `pitch_source` | 最新の値 (deg. 校正前) |
| `read[16]` | 読み取った値: 0〜2 加速度, 3〜5 角速度, 6〜8 地磁気, 9〜11 重力方向 (MPU6050), 12 ロール, 13 ピッチ, 14 ヨー, 15 温度 |

`ServoParam sv` (サーボ. `ixl_` = L系統, `ixr_` = R系統, 各15要素):

| フィールド | 型 | 意味 |
|---|---|---|
| `num_max` | int | 処理するスロット数 (4章 #6) |
| `ixl_mount` / `ixr_mount` | int | マウント (UART の系統で扱うか) |
| `ixl_id` / `ixr_id` | int | サーボの ID |
| `ixl_cw` / `ixr_cw` | int | 回転方向の補正 ±1 |
| `ixl_trim` / `ixr_trim` | float | トリム (deg) |
| `ixl_type` / `ixr_type` | float | スロットの種類 (`IXx_MT` の値) |
| `ixl_tgt` / `ixr_tgt` | float | 今回の指令値 (サーボの処理の後は現在位置) (deg) |
| `ixl_tgt_past` / `ixr_tgt_past` | float | 前回の `tgt` |
| `ixl_cmd_past` / `ixr_cmd_past` | float | 前回の指令値 (`SERVO_STEP_MAX` の基準) |
| `ixl_err` / `ixr_err` | int | 連続の失敗回数 (上限 `SERVO_LOST_ERR_WAIT`) |
| `ixl_stat` / `ixr_stat` | uint16_t | 1 = 通信不能 |

表示用: `MrdFrameMon frame_mon` (`busy_sum/max`, `servo_sum/max`, `late`, `count`, `slot_fail[2][15]`), `MrdHduartStat hduart_stat` (`sum_us[2]`, `max_us[2]`, `count[2]`, `fail[2][5]`).

サーボ層 (mrd_servo.h): `mrd_servo_param[2][15]` (パラメータ系コマンドの状態 `MrdServoParamSlot`: mode, cmd, val, result, due), `mrd_servo_motion[2][15]` (補間 `MrdMotion`), `mrd_servo_type2[2]` (2種類目), `mrd_servo_frame` (駆動の回数. 50Hz の偶奇に使う), `mrd_servo_skip_reads_frame` (マスターコマンド2).

### 20.4 その他のグローバル変数

| 変数 | 場所 | 意味 |
|---|---|---|
| `mrd` | main.cpp | Meridian ライブラリ (チェックサム, 連番, 変換) |
| `sv_bus_l` / `sv_bus_r` | main.cpp | 系統のバス |
| `ics_l` 〜 `dxl1_r` | main.cpp | プロトコル層の実体 (10.1節) |
| `timer`, `timer_semaphore`, `timer_mux` | main.cpp | フレームタイマー |
| `count_frame` / `count_timer` | main.cpp | 処理したフレーム数 / タイマーの割り込み数 |
| `ether_send_ip` | main.cpp | 有線LANの送信先 |
| `udp_valid_ms` | main.cpp | 最後に正しいパケットを受信した時刻 (ms. フェイルセーフの判定) |
| `udp_self_frames` | main.cpp | 受信しないモードで, 起動後に脱力したフレーム数 |
| `udp_self_cmd_l` / `udp_self_cmd_r` | main.cpp | 受信しないモードで, 前のフレームに送った指令値 (`UDP_SELF_SPEED` の基準) |
| `thp[4]` | main.h | タスクハンドル ([0] センサ, [1] INA226, [2] WIIMOTE / BlueRetro) |
| `ina226`, `ina226_mux` | mrd_wire0.h | INA226 の電圧・電流 (V×100, A×100) と排他 |
| `mrd_disp` | main.h | シリアル表示 (`MrdMsgHandler`) |
| `wiimote`, `pad_bt_val`, `pad_bt_mux` | mrd_bt_pad.h | WIIMOTE |
| `cfg` | main.h | 設定値 (20.3節) |
| `mrd_eep_buf` | mrd_eeprom.h | EEPROM の内容の作業用 (2880byte. スタックに置かないため) |
| `mrd_eep_boot` | mrd_eeprom.h | 起動時の EEPROM の処理の結果 (表示用) |
| `MRD_EEP_FIELDS` | mrd_eeprom.h | EEPROM の項目表 (ページ, 語, 値の形, 設定値の場所, 倍率, 範囲. 16.3節) |
| `ahrs_mux` | mrd_wire0.h | IMU の排他 |
| `udp` / `udp_et` | mrd_wifi.h / mrd_ether.h | UDP |
| `mrd_sbdbt_edge_*`, `mrd_sbdbt_pin`, `mrd_sbdbt_rx` | mrd_bt_pad.h | SBDBT の信号の変化の記録 (リングバッファ, 割り込みで書く) と電文の組み立て |
| `mrd_blueretro` | mrd_bt_pad.h | BlueRetro の読み取りの状態 |
| `mrd_pca[2]`, `mrd_pwm_last_on` | sv_pwm.h | PCA9685 の実体, PWM の前回の出力の有無 |

### 20.5 コード内の定数

| 定数 | 値 | 場所 | 意味 |
|---|---|---|---|
| `MRDM_BYTE` | 180 | main.h | Meridim の byte 数 |
| `MRD_ERR` / `MRD_CKSM` | 88 / 89 | main.h | エラー / チェックサムの添字 |
| `MRD_ERR_l` | 176 | main.h | エラーの下位 byte の位置 |
| `MRD_EEP_PAGE_WORDS` / `MRD_EEP_MARK` / `MRD_EEP_VERSIONS` | 90 / 0x4D00 / {3, 1, 1, 1, 4, 2, 2, 2} | mrd_eeprom.h | EEPROM のページ (語数, 印, ページごとの配置の版) |
| `MRD_EEP_NET_SSID` / `MRD_EEP_NET_PASS` | 25 / 41 | mrd_eeprom.h | P0 の SSID・パスワードの語の位置 |
| `MRD_SBDBT_EDGES` | 256 | mrd_bt_pad.h | SBDBT の信号の変化の記録数 |
| `PAD_LEN` | 5 | main.h | パッドの配列の長さ |
| `MRD_PCMD_MIN` | 3 | mrd_servo.h | パラメータ系コマンドの最小番号 |
| `MRD_PCMD_ERR_FLAG` | 0x4000 | mrd_servo.h | 返信のエラービット |
| `MRD_PCMD_NA` / `MRD_PCMD_NO_REPLY` / `MRD_PCMD_SERVO_ERR` | −32768 / −32767 / −32766 | mrd_servo.h | エラー値 |
| `MRD_CMD_WHEEL` / `MRD_CMD_THROUGH` | 0x2000 / 0x8000 | mrd_servo.h | 車輪 / スルー |
| `MRD_HDUART_CRITICAL_BYTES` / `MRD_HDUART_RX_BUF` / `MRD_HDUART_TX_MAX` | 8 / 512 / 128 | mrd_servo.h | 半二重UART |
| `MRD_STEPPER_BUF_BYTES` | (計算値) | mrd_servo.h | ステッパーの領域 |
| `MRD_SERVO_BOOT_RETRY` | 3 | sv_util.h | 起動時の応答確認の回数 |
| `MRD_SERVO_TORQUE_REFRESH_FRAMES` | 20 | sv_util.h | トルクの送り直しの間隔 |
| `ICS_EEPROM_LEN` / `ICS_EEPROM_TIMEOUT_MS` | 64 / 50 | sv_ics.h | ICS の EEPROM |
| `PMX_SAVE_TIMEOUT_MS` / `PMX_ID_MAX` | 40 / 239 | sv_pmx.h | PMX |
| `XBUS_ID_MIN` / `XBUS_ID_MAX` | 1 / 50 | sv_xbus.h | XBUS |
| `PWM_PERIOD_US` / `PWM_LEDC_CH0` / `PWM_LEDC_BITS` | 20000 / 8 / 16 | sv_pwm.h | 本体PWM |
| `UDP_SELF_LIMP_FRAMES` | 50 | main.cpp | 受信しないモードで, 起動後に脱力して現在位置を読むフレーム数 |

---

## 21. 検証方法

| 方法 | 内容 |
|---|---|
| ビルド | `pio run` (`MRD_BOARD_SERVO_IF` 1 / 0 の両方). 設定の分岐は `#if` ではなく通常の `if` で書いているため, 1回のビルドで全分岐がコンパイルされる (`MRD_BOARD_SERVO_IF` のピン定義だけが `#if`) |
| 書式 | VSCode 同梱の clang-format で `--style=file` を当て, 差分がないこと (`tests/run_all.sh` で確認できる) |
| 命名 | clang-tidy の `readability-identifier-naming` (引数 `a_`, private/protected メンバ `m_`, ローカル `_tmp`, 型は CamelCase) で違反がないこと. 規則は `.clang-tidy`, 確認は `tests/tidy/run.sh` |
| ホストテスト (PC 上) | `tests/host/run.sh`. `src` から生成したソースを, 模擬のサーボバス・UDP・パッド・EEPROM でコンパイル・実行する test〜test20 と test4 (PWM 2方式). サーボ層 (パケットの形式, 角度換算, 系統ステッパーの手順, 混在, パラメータ系コマンド, 車輪, 補間, 返信の規則) と, メインループの関数 (受信の確認, フェイルセーフ, マスターコマンドの実行条件, 10100, 10004, 受信しないモード, WIIMOTE, UDP 受信, EEPROM の書き込み禁止, 10003, KRC-5FH の右スティック, 空の EEPROM) を確認する. test4 は PWM のスロットごとの換算 (中央・+90°・角度の範囲) も, test14 は EEPROM のページ (形式・ページごとの版・項目の重なりと範囲・往復・範囲の確認・旧形式・起動時の選択・通信での読み書き・パスワード・10101・10400・P6/P7), test15 は SBDBT のソフトウェア受信, test16 は BlueRetro (Wii extension), test17 は WIIMOTE + クラシックコントローラ, test18 はサーボのリンク (行の検査, 向きを掛けたコピー, 車輪, 保持, フォロワー宛てのパラメータ系コマンド), test19 は通信の周期 (ボード主導 / PC 主導の受信待ちの時間, 10005 / 10006, 受信しないモード, EEPROM), test20 は INA226 (電圧・電流の換算, シャント抵抗, Meridim[86]・[87], EEPROM. I2C の読み取りのスレッドは実機でのみ). ループ本体の送る順序とタイマーを待たないことは実機でだけ確かめられる |
| 設定画面のテスト | `tests/config_html/core_test.js` (node). 画面の項目がすべて config.h / keys.h にあること, 読み込み → 値を変更 → 書き戻したとき変更した行だけが変わり, 元の値に戻すと元に戻ること (コメントの前の空白を除く). リンク表の書き換え (行の変更・追加・削除・空, 表の外の行が変わらないこと, 複数行の行は編集しない) も確かめる. `tests/config_html/i18n_test.js` (node). 画面の日本語の文字列すべてに英訳があり, 使われない英訳・{n} の食い違い・英訳中の日本語がないこと |
| 実機 | `MONITOR_FRAME_DELAY` 1 で処理時間と通信の失敗を確認する. Meridian_console で Meridim の値を確認する |

テスト一式は `tests/` にある (使い方は `tests/README.md`). 実機以外の確認は `tests/run_all.sh` でまとめて行える.

---

## 22. 既知の制約と不具合

本文の 〔不具合 B-n〕 〔制約 K-n〕 の一覧. 「節」は本文で説明している場所.

### 22.1 既知の不具合

修正予定の不具合はない. 修正済みの不具合 (版 1.2〜1.4. 番号は欠番にせず残す):

| 番号 | 以前の動作 | 修正後 | 節 |
|---|---|---|---|
| B-1 | 受信が途絶えても, 最後の指令でトルクONのまま保ち続けた (期限なし) | 正しい受信が `UDP_FAILSAFE_MS` (1秒) 続けてないと全サーボを脱力する | 7.5 |
| B-2 | チェックサム不正のフレームで, ボード自身の前回の返信 (値 = 現在位置など) を指令として使い, 続く正常なフレームも連番が不一致になった | 受信なしと同じく, 最後に正しく受信した PC のパケットを使い, 連番は比べない (版 1.4) | 7.5, 8.5 |
| B-3 | マスターコマンド 0 / 2 は受信が正常なフレームでしか効かなかった | 受信の成否を見ずに実行する (3 と同じ) | 9 |
| B-4 | 10100 の再駆動の目標が, 読んだ現在位置だった (元のトリムの角度を保たない) | 目標を元のトリムの角度 (トリム × cw) にする | 9 |
| B-5 | 10004 でエラー番号が消えなかった (`stat` を消さない) | `stat` も消す | 9 |
| B-6 | `MODE_UDP_RECEIVE` 0 では全スロットのコマンドが 0 になりサーボが動かず, bit10 が立ち続けた | ボードが指令を作る (脱力して現在位置を読んだ後, トルクON) | 5.3 |
| B-7 | WIIMOTE が切断されても, 最後のボタンとスティックの値が残った | 切断されたら 0 にする | 15.2 |
| B-8 | `SERIAL_PC_TIMEOUT` が効かなかった (ESP32 では `Serial` がすぐ真になる) | 効かない待ちを削除し, 設定は未使用とした | 4, 19.4 |
| B-9 | UDP は1フレームに1パケットしか取り出さず, 未読が溜まると数フレーム古い指令を使い続けた | 溜まっていれば最新の1つを使う | 7.2, 7.3 |
| B-10 | WIIMOTE のスティック値 [16] が byte ごとに区切られず, Y が負のとき上位 byte から借りた | 上位 byte = X, 下位 byte = Y に詰める | 8.7, 15.2 |
| B-11 | `EEPROM_PROTECT` 1 のとき "EEPROM is protected…" が表示されなかった | 表示する | 16.5 |

### 22.2 実装上の制約

全体は K-1〜, サーボは K-20〜 の番号.

| 番号 | 内容 | 節 |
|---|---|---|
| K-1 | WiFi の接続がタイムアウトしても起動を続ける (有線LANの失敗は停止する) | 7.2 |
| K-2 | WIIMOTE の接続に失敗しても Bluetooth スレッドを開始する | 15.2 |
| K-3 | 10009 (EEPROM 書き込みモード) はフラグを立てるだけで, 効くのはフレームタイマーのリセットのみ (書き込みは 10300+p). 10005 / 10006 は版 1.26 で実装した | 9 |
| K-4 | 10008 は `delay()` でループ全体を止めるため, その間 (最大 1000ms) は送受信もフェイルセーフもしない. [19] を補間フレーム数と共用する | 9 |
| K-5 | 10101 は P1 の `num_max` 以降のスロットを 0 にする. P1 を `EEPROM_LOAD` で選んでいないと, `EEPROM_SET` 1 (既定) では次の起動で config の値に書き戻される | 9 |
| K-6 | 10100 と 10102 は1フレームの中でサーボを追加で駆動する. そのフレームは 10ms を超えることがあり, 偶数/奇数フレームの割り振り (PWM, Hitec, XBUS, 混在系統) がずれる. 10102 は 0° と制限値を続けて指令する | 9 |
| K-7 | マスターコマンド 0 / 2 を受けている間, "All servos torque off." を毎フレーム表示する (正しく受信したフレームだけ. 途絶えている間は脱力を続けるが表示しない) | 9 |
| K-8 | (解消: 版 1.5 から EEPROM は起動の最初に読む. 番号は欠番にせず残す) | 4 |
| K-9 | `IMUAHRS_STOCK`, `CHECK_EEPROM_RW`, `MONITOR_ERR_SERVO`, `MONITOR_PAD`, `PIN_RESET_LAN`, `SERIAL_PC_TIMEOUT` は使われていない (`SPI0_SPEED` は表示のみ) | 19 |
| K-10 | (解消: 版 1.6 から BNO055 / MPU6050 の温度を返す. 番号は欠番にせず残す) | 8.2 |
| K-11 | [7-2] のサンプル (パッドの十字右/左で L00 を ±30°) が既定で有効. 使わない場合は main.cpp の [7-2] を消す | 5.2 |
| K-12 | LED のピンを本体PWM・I2C1・SBDBT の受信に使うときは LED がなく, 停止しても点滅しない (既定のピンでは `PWM_PIN_MODE` 1 / 2 と BlueRetro) | 4, 19.10 |
| K-13 | UDP に認証がなく, 送信元も確認しない | 7.6 |
| K-14 | 有線LANはケーブルのリンクを確認しない (IP の一致だけで成功とする) | 7.3 |
| K-15 | (解消: 版 1.17 から SD カードの確認で書いた値と読んだ値を比べる. 番号は欠番にせず残す) | 16.7 |
| K-16 | フレームの遅れはエラービットに出ない (表示のみ) | 5.1, 10.13 |
| K-20 | 起動時に応答しなかったサーボもマウントを外さない (表示のみ). 通信不能は連続6回の読み取り失敗で判定する | 8.3, 10.9 |
| K-21 | ICS は起動時に応答を確認しない | 10.12 |
| K-22 | マスターコマンド 2 で位置を読まないのは DXL1/DXL2/FEETECH/FUTABA だけ | 9 |
| K-23 | FUTABA は全台の位置を毎フレーム読むと 100Hz に収まらないため, `SERVO_FTB_READ_PER_FRAME` 台ずつ巡回する | 11.6 |
| K-24 | Hitec は1台あたり約4.2ms かかり, 動作中は現在位置を読まない. `SERVO_HTC_PER_FRAME` × 2 台を超えると (既定では3台以上) 各サーボの更新が 50Hz 未満になる | 11.9 |
| K-25 | XBUS は返信がないため, 現在位置は指令値を返す. パラメータ系コマンド・車輪のスロットには送信しないので, 約1.5秒でサーボが脱力することがある | 11.8 |
| K-26 | パラメータ系コマンドの EEPROM・ROM の書き込み, Hitec, 車輪は1フレームの時間を超えることがある | 12.1 |
| K-27 | DYNAMIXEL 2.0 の EEPROM 領域はトルクOFF でないと書けない (サーボエラーが返る) | 12.2 |
| K-28 | (解消: 版 1.17 から DXL1 で機種が分からないサーボは, モデル番号を読めるまで動かさない. 番号は欠番にせず残す) | 10.8, 11.4 |
| K-29 | FEETECH SCS の 27 と 32/33 は, 通常のトルクON (コマンド 1) に戻すと毎フレームの目標の送信で上書きされる | 12.2 |
| K-30 | ICS の 35 と EEPROM の項目の書き込みは, RAM の値 (SC1〜4) も一緒に保存する | 12.2 |
| K-31 | パラメータ系コマンドの書き込みは読み返さず, 受け取った値を結果として返す | 12.1 |

### 22.3 実機で未確認のもの

PMX, 混在系統, 10003, KRC-5FH の右スティック (PA3/PA4), パラメータ系コマンド, 車輪 (AX・SCS の単位), Meridim[19] の補間, マスターコマンド 2/3, DXL1, Hitec のパラメータ系コマンド, MPU6050, WIIMOTE (切断の検出を含む), KRC-5FH, ボード単体モード, 受信が途絶えたときの脱力, 受信しないモード (`MODE_UDP_RECEIVE` 0), UDP の溜まったパケットの扱い, 10100 のトリムの保持.

---

## 23. 用語

| 用語 | 意味 |
|---|---|
| Meridian | デバイスやソフトを中間プロトコルでつなぐリアルタイムシステム |
| Meridim / Meridim90 | Meridian のコアとなる配列 / 90要素 (180byte) の標準形 |
| フレーム | 10ms の処理の単位 |
| 系統 (L / R) | サーボをつなぐ UART の通信路. L = Serial1, R = Serial2 |
| スロット | 系統の中のサーボの位置 (0〜14). Meridim のコマンドと値の組に対応する |
| マウント | UART の系統でそのスロットのサーボを扱うこと. config の種類が 0 でも PWM 専用 (1/11) でもなく, 混在の検査で外れていないスロット |
| トリム | サーボの取り付け誤差の補正角 (deg) |
| cw | 回転方向の補正 (+1 / −1) |
| 指令値 (tgt) | そのフレームでサーボに送る角度. サーボの処理の後は現在位置で上書きされる |
| ステッパー | 系統の1フレームの送受信を1回ずつに分けて進めるオブジェクト |
| パラメータ系コマンド | スロットのコマンドの下位8bitが3以上の命令. サーボのパラメータを読み書きする |
| 車輪 | サーボを無限回転 (速度制御) で使うモード |
| スルー | ボードがそのサーボと通信せず, 受け取った値をそのまま返すモード |
| 送信エコー | 半二重の回路で, 送信したバイトが自分の受信に戻ってくること |
| ボード主導 | ボードが一定周期で送信し, PC が受信のたびに返信する方式 |

---

## 24. 改訂履歴

| 版 | 日付 | 内容 |
|---|---|---|
| 1.0 | 2026-10-16 | 初版 |
| 1.1 | 2026-10-16 | レビューの反映. 記述の区分 (仕様 / 制約 / 不具合 / 未検証) を導入. PC とのインターフェースと異常時の動作 (7.5), 通信の安全性 (7.6), 互換性 (1.4), 単位・符号・座標 (8.7), 時間の予算 (10.13), 設定の組み合わせ (19.10) を追加. 誤り・不足を修正 (10100 の手順, 10004, `SERIAL_PC_TIMEOUT`, `SERVO_HTC_SPEED`, 12章の倍率・符号・E 印 ほか). 重複 (5.3/19.2, 7.4/19.8) を整理し, 関数一覧を付録Aへ. ファームウェアの修正3件を反映 (FEETECH 26 の符号を bit15 に, DXL1 の機種不明のサーボに 56/57・60/61 を書かない, パラメータ系コマンドの値をエラー値と重ならない範囲に収める) |
| 1.2 | 2026-10-17 | 既知の不具合 B-1, B-3〜B-11 の修正を反映: 受信が1秒途絶えたら全サーボ脱力 (`UDP_FAILSAFE_MS`), マスターコマンド 0 / 2 を受信の成否に関係なく実行, 10100 でトリムの角度を保つ, 10004 で `stat` も消す, `MODE_UDP_RECEIVE` 0 でボードが指令を作る (トルクON後の速さは `UDP_SELF_SPEED`), WIIMOTE の切断で 0, スティック値を byte ごとに, UDP は最新のパケットを使う, `SERIAL_PC_TIMEOUT` を未使用に, `EEPROM_PROTECT` の表示. 22.1 に修正済みの表を追加. テスト一式を `tests/` に同梱し 21章を更新 (メインループのホストテスト test13 を追加) |
| 1.3 | 2026-10-17 | 10003 (ロール・ピッチ・ヨーの校正) を実装. KRC-5FH の右スティック (PA3/PA4) を [17] に. EEPROM が空のときは読み込まない |
| 1.4 | 2026-10-17 | B-2 を修正: チェックサム不正のフレームを受信なしと同じに扱い, 最後に正しく受信した PC のパケット (`r_udp_meridim_last`) を使う. 修正予定の不具合はなくなった |
| 1.5 | 2026-10-17 | 設定の EEPROM 化: config.h・keys.h の設定を16ページ (2880byte) に保存し, ページごとに config / EEPROM を選べる (`EEPROM_LOAD` をビットに). コードの設定値を構造体 `cfg` に. EEPROM を起動の最初に読む (K-8 解消). 10200〜10215 / 10300〜10315 でページの読み書き, 10400 で再起動. WiFi の SSID・パスワードも P5 に保存 (送受信・表示とも伏せない). 16章を作り直し |
| 1.6 | 2026-10-17 | 温度 (Meridim[11]) を BNO055 / MPU6050 の内蔵の温度センサから返す (K-10 解消) |
| 1.7 | 2026-10-17 | SBDBT (RCB3 形式) に対応: `PIN_PAD_RX` (GPIO34) でソフトウェア受信 (15.3節). 設定 `PAD_SBDBT_BPS`, `PAD_SBDBT_TIMEOUT`, `PIN_PAD_RX` を追加 (EEPROM の P0 語30〜32) |
| 1.8 | 2026-10-17 | BlueRetro (Wii extension 出力) に対応: I2C1 (`Wire1`, ピン 25/26) で Core0 のスレッドが読む (15.4節). I2C1 を使うときは LED なし |
| 1.9 | 2026-10-17 | LED のピンを設定に (`PIN_ERR_LED` / `PIN_LED_BT`, 既定 25/26). 本体PWM・I2C1・SBDBT の受信と同じピンのときだけ LED なし |
| 1.10 | 2026-10-17 | BlueRetro をつなぐ I2C を選べるように (`PAD_BLUERETRO_I2C`: 0 = I2C0, 1 = I2C1. EEPROM の P0 語33) |
| 1.11 | 2026-10-17 | WIIMOTE + クラシックコントローラ (`WIIMOTE_C`) に対応 (同梱ライブラリにクラシックの識別と拡張データの取得を追加. 値の換算は BlueRetro と共通) |
| 1.12 | 2026-10-17 | 重複・冗長なコードの整理 (動作は変えない): 表示の設定の写し `monitor` を廃止して `cfg.monitor_*` を直接使う, 使われていない変数・フラグの削除, 10100/10102 の共通部分 (`mrd_command_trim_home`) とコマンドの表示 (`mrd_command_log`), サーボの通信速度の決め方を `mrd_line_baud` にまとめる, BlueRetro の開始 (`mrd_blueretro_setup`), サーボのトルクの送信 (`mrd_servo_torque_each`) と各ステッパーの系統の参照を共通化. 付録A を更新 |
| 1.13 | 2026-10-17 | KRC-5FH のスティックの byte の並びを他のパッドに合わせた ([16] [17] とも上位 byte = X (PA1/PA3), 下位 byte = Y (PA2/PA4)). どのパッドも Y は上が正 |
| 1.14 | 2026-10-17 | KRC-5FH の `PAD_GENERALIZE` 0 でスティックが常に 0 だったのを, 受信機の生値 (0〜127) を入れるように修正 |
| 1.15 | 2026-10-17 | 車輪 (bit13) の速度の単位を deg/s から rpm ×100 (0.01rpm. DYNAMIXEL 2.0 の rpm 基準) に変更 (返信の現在速度も). FEETECH STS の車輪の速度の ±1023 step/s の制限を外した (SCS は ±1023 のまま). パラメータ系 26/27 は deg/s のまま |
| 1.16 | 2026-10-17 | WiFi の送り先 IP (`WIFI_SEND_IP`) が解析できないとき, 送らないことを1回表示する. 10100 で Meridim に入れる元のトリムを deg ×100 に |
| 1.17 | 2026-10-17 | 厳しめのレビューの指摘を修正: PMX の送信バッファのあふれ, EEPROM の書き込みの確定の失敗を返す, FUTABA の ACK を1byteに, DXL1 の機種不明のサーボ (K-28 解消), パラメータ系の書き込み値の範囲 (収まらなければ未対応), 10008 は最大 1000ms, WIIMOTE のスレッドのスタック 4096, ICS の返信の ID, 型変換 (float), WiFi・SD (K-15 解消)・IMU・PCA9685・PMX・FEETECH・タスク作成のエラー処理, 割り込み停止中の待ちの上限, static_assert |
| 1.18 | 2026-10-17 | ソースとの照合による見直し: パッド (SBDBT, BlueRetro, WIIMOTE_C) の記述をスレッド表・起動・Meridim [17][18] などに反映, 10100 の返信の範囲, 未対応 (−32768) と応答なし (−32767) の条件, EEPROM でのマウントの外し方, 起動時の表示の順, LED の条件, データ構造の表 (EEPROM の列挙型・定数, SBDBT・BlueRetro・PCA9685 の変数), 節の参照. サーボの既定値の表 (19章, P3) を公式 v1.1.1 の既定値 (L/R とも ICS 43, 1.25Mbps, 11台) にそろえた |
| 1.19 | 2026-10-17 | 設定画面の更新に合わせて18章を修正 (`IMUAHRS_STOCK` にも「未使用」バッジ) |
| 1.29 | 2026-10-21 | 設定画面を最新の config.h に合わせた: `I2C0_SPEED` を IMU がなくても INA226 / BlueRetro (I2C0) で表示, 数値の項目に EEPROM の範囲の上限・下限と範囲外の説明, 整数の文字の項目 (アドレスなど) の書式と範囲の検査. 2.3, 18章 |
| 1.28 | 2026-10-20 | INA226 (電流・電圧センサ, I2C0) に対応: `MOUNT_INA226`, `INA226_ADDR`, `INA226_SHUNT_MOHM`, `INA226_INTERVAL` (EEPROM P4 の語35〜38. P4 の版を 4 に). 電圧を Meridim[86] (V×100), 電流を [87] (A×100) に. 4, 6, 8.2, 14.1, 16, 19, 20, 21章 |
| 1.27 | 2026-10-19 | EEPROM の P0 をネットワーク (WiFi の SSID・パスワードなど), P4 をシステム・動作モードに入れ替え (`EEPROM_P_NET` 1, `EEPROM_P_SYSTEM` 16). P0 と P4 の配置の版を 3 に. 通信ではネットワークが 10200 / 10300. 7, 16, 19, 20章 |
| 1.26 | 2026-10-19 | 通信の周期の PC 主導を実装 (`MODE_UDP_PASSIVE`, P0 の語15, 10005 / 10006 で切り替え, `mrd_udp_passive`, `mrd_udp_wait_us`, `mrd_frame_resync`). 5.1, 5.2, 5.3, 7.5, 9, 16, 19, 20, 22章 (K-3), 21章 (test19) |
| 1.25 | 2026-10-18 | EEPROM のページを整理: P0 の語をグループ (動作モード・UDP / モニタ / シリアル・I2C0・IMU / パッド) ごとに分けて予備を置く. P4 をネットワーク, P5 を PWM の共通設定と対応表 (穴を詰める), P6 / P7 を PWM の L / R 系統のスロットに. 配置の版をページごとに (P0, P4〜P7 は 2). P1〜P3 は変えない. 16章, 7章, 20章 |
| 1.24 | 2026-10-18 | PWM のパルス幅と角度の範囲をスロットごとに (`IXL_PWM_CENTER`, `IXL_PWM_P90`, `IXL_PWM_MAX_DEG`, `IXL_PWM_MIN_DEG` と R). 全体共通の `PWM_PULSE_CENTER`, `PWM_US_PER_DEG`, `PWM_PULSE_MIN`, `PWM_PULSE_MAX` を削除. EEPROM に P6 (パルス幅), P7 (角度の範囲) を追加 (`EEPROM_SIZE` は 1440 以上). 10.8, 13, 16, 18, 19章 |
| 1.23 | 2026-10-18 | 18章: 設定画面でサーボのリンク (`SERVO_LINK`) を編集. 21章: 設定画面のテストにリンク表を追加 |
| 1.22 | 2026-10-18 | サーボのリンク (`SERVO_LINK`) を追加: 10.14節, 5章 [7-4], 4章 #6, 17章, 18章, 19章, 21章 (test18), 付録A |
| 1.21 | 2026-10-17 | 18章: 設定画面の日英切り替え (表示言語の行). 21章: 設定画面の言語のテスト |
| 1.20 | 2026-10-17 | 16.3: P1 では新しくマウントしないが, P2 を読み込むとスロットの種類が変わり, config で種類が 0 のスロットもマウントされることを明記 (コードの動作に合わせた) |

---

## 付録A. 関数一覧

ソースの `/// @brief` を機械的に集めた一覧 (クラスのメソッドを含む). 引数と戻り値はソースの `@param` / `@return` を見ること. ソースの関数を変えたら, この一覧も作り直す. 「種別」が関数のものはグローバル関数, メソッドはクラスのメンバ関数.
テンプレート関数 `mrd_servo_torque_each` (sv_util.h) と, クラス `MrdParamLineStepper` (sv_util.h) のテンプレートも含む.

#### main.cpp

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_frame_timer_isr` | 関数 | count_timerを保護しつつ1ずつインクリメント |
| `mrd_task_start` | 関数 | Core0 のスレッドを開始する. 作れなければ(メモリ不足)表示して停止する. |
| `mrd_servo_setup` | 関数 | スロットの設定(config の配列. EEPROM の P1/P2 を使う場合はその値)をサーボパラメータに反映し, サーボの通信を開始する. |
| `mrd_line_baud` | 関数 | 系統のプロトコル層の通信速度. 2種類目のサーボ(MOUNT_SERVO_TYPE_x2)の系列なら, その通信速度. |
| `mrd_settings_setup` | 関数 | 起動の最初に設定値(cfg)を決める. EEPROM_LOAD で選んだページは EEPROM の値を使う. 決めた値を, フラグ・表示の設定と, サーボのバス・プロトコル層に反映する. |
| `mrd_network_setup` | 関数 | WiFi(MODE_ETHER 0)または有線LAN(MODE_ETHER 1)を開始する. 設定の誤りがあれば停止する. |
| `mrd_blueretro_setup` | 関数 | BlueRetro(Wii extension)を読むスレッドを開始する. PAD_BLUERETRO_I2C 0 なら I2C0, 1 なら I2C1. I2C1 のピンを本体PWM(PWM_PIN_MODE 1)に使う場合は開始しない. |
| `mrd_udp_send` | 関数 | Meridimを送信する(WiFiまたは有線LAN). |
| `mrd_udp_receive_wait` | 関数 | Meridimを受信するまで待つ(最大 mrd_udp_wait_us(). UDP_POLL_US ごとに確認). |
| `mrd_meridim_self_make` | 関数 | PCから受信しない場合(MODE_UDP_RECEIVE 0)に, 受信データの代わりの指令をボードが作る. |
| `mrd_udp_passive` | 関数 | 通信の周期がPC主導か. 受信しないモード(MODE_UDP_RECEIVE 0)は常にボード主導. |
| `mrd_udp_wait_us` | 関数 | 受信を待つ最大時間(us). ボード主導は UDP_TIMEOUT_US, PC主導は1フレーム(FRAME_DURATION). |
| `mrd_frame_resync` | 関数 | フレーム数(count_frame)をタイマー(count_timer)に合わせる. 遅れも先行も捨てる. |
| `mrd_self_speed_limit` | 関数 | 受信しないモード(MODE_UDP_RECEIVE 0)のトルクON後に, 指令値の1フレームの変化量を UDP_SELF_SPEED に制限する. |
| `mrd_servo_link_slot` | 関数 | SV_L(n)/SV_R(n) の値を系統とスロット番号に分ける. |
| `mrd_servo_link_print` | 関数 | スロットを "L04" の形で表示する(範囲外は数値のまま). |
| `mrd_servo_link_setup` | 関数 | リンク表を検査して使う行を決め, 内容を表示する. 使えない行は警告して使わない. |
| `mrd_servo_link_val` | 関数 | 向きを掛けたVAL. 反転で int16_t の範囲を超える -32768 は 32767 にする. |
| `mrd_servo_link_apply` | 関数 | リンク表に従い, フォロワーのスロットにリーダーの指令をコピーする. [7-1]〜[7-3] の後, [8] の前に呼ぶ. |
| `mrd_meridim_received_check` | 関数 | 受信したMeridimのチェックサムとシーケンス番号を確認し, 送信配列に転写してエラービットを更新する. |
| `mrd_udp_failsafe` | 関数 | PCからの受信が UDP_FAILSAFE_MS 以上途絶えたら, 全スロットのコマンドを0(脱力)にする(フェイルセーフ). |
| `mrd_servo_targets_compute` | 関数 | 受信した目標値から, 各サーボの今回の指令値(tgt)を求める. |
| `mrd_servo_values_reply` | 関数 | 各スロットの返信値(現在位置, 指令値, パラメータ系コマンドの結果)を送信配列に格納する. |
| `mrd_frame_is_late` | 関数 | 今のフレームの処理が, 次のフレームの時刻までに終わらなかったか. |
| `mrd_frame_next_wait` | 関数 | タイマー(count_timer)がフレーム数(count_frame)に追いつくまで待つ. タイマー割り込みでセマフォが与えられるまでブロックし, その間CPUを空ける. フレームタイマーのリセットを指示されていれば(MCMD_FRAMETIMER_RESET), 遅れの累積を捨てる. |

#### main.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_led_write` | 関数 | LEDを点灯/消灯する. ピン番号が負(ピンをPWM等に使うためLEDなし)なら何もしない. |
| `mrd_servo_is_pwm_type` | 関数 | UARTを使わないスロットのサーボタイプか(1:本体PWM, 11:PCA9685). このスロットはUARTの系統では扱わず, PWM出力の対応表(sv_pwm.h)から指令値として参照される. |
| `mrd_pin_err_led` | 関数 | エラーLEDのピン(PIN_ERR_LED). そのピンを LED 以外に使うときは LED なし(-1). |
| `mrd_pin_used_for_other` | 関数 | ピンを LED 以外に使うか: 本体PWM(PWM_PIN_MODE 1), I2C1(PCA9685 の PWM_PIN_MODE 2, BlueRetro), SBDBT の受信(PIN_PAD_RX). |
| `mrd_pin_led_bt` | 関数 | Bluetooth接続確認用LEDのピン(PIN_LED_BT). そのピンを LED 以外に使うときは LED なし(-1). |

#### mrd_util.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_index_max_used_get` | 関数 | 配列の中で0以外が入っている最大のIndexを求める. |
| `mrd_bit16_set` | 関数 | 指定された位置のビットをセットする(16ビット変数用). |
| `mrd_bit16_clear` | 関数 | 指定された位置のビットをクリアする(16ビット変数用). |
| `mrd_pddstr` | 関数 | 数値をシリアルモニタ表示するときにパディングする. |
| `mrd_line_name_get` | 関数 | 列挙型(L,R,C)から文字列を取得する関数. |
| `mrd_error_stop` | 関数 | エラーメッセージ＆LED点滅と共にシステム停止. |
| `mrd_step_limit` | 関数 | 目標値の変化量を基準値から±a_step_maxの範囲に制限する. |

#### mrd_disp.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `charging` | メソッド | 指定されたミリ秒数だけキャパシタの充電プロセスを示すメッセージを表示する. |
| `hello_lite_esp` | メソッド | システムのバージョン情報と通信速度の設定を表示するためのメッセージを出力する. |
| `servo_bps_2lines` | メソッド | マウント設定したサーボのbpsをシリアルモニタに出力する. |
| `servo_echo` | メソッド | 指定されたUARTラインの送信エコー処理の有無を表示する. |
| `print_servo_ids` | メソッド | 指定されたUARTラインのサーボIDを表示する. |
| `servo_mounts_2lines` | メソッド | マウントされているサーボのIDを表示する. |
| `esp_wifi` | メソッド | wifiの接続開始メッセージを出力する. |
| `servo_protocol` | メソッド | 指定されたUARTラインとサーボタイプに基づいてサーボの通信プロトコルを表示する. |
| `esp_ip` | メソッド | wifiの接続完了メッセージと各IPアドレスを出力する. |
| `mounted_pad` | メソッド | マウント設定したジョイパッドのタイプをシリアルモニタに出力する. |
| `flow_start_lite_esp` | メソッド | システムの動作開始を示すメッセージを出力する. |
| `all_err` | メソッド | システム内の様々な通信エラーとスキップ数をモニタリングし, シリアルポートに出力する. |
| `flow` | メソッド | デバッグ用のフロー表示. 表示OFF時は何もしない(文字列オブジェクトを生成しない). |
| `frame_time` | メソッド | フレームの処理時間を集計し, a_frames フレームごとに平均と最大を表示する. 例) Frame[us] busy 4321/6789 servo 2100/2300 late 0/100 rxL 150/300 rxR 160/310 err L06 R04 (平均/最大). rxL/rxRは系統ごとの応答待ち時間(1回の送受信あたり, hduart_stat). 系統ごとの応答の速さを表す. 通信の失敗があった場合は2行目に内訳を表示する. 例) Fail L: echo 0/0 reply 0/3 \| R: echo 0/0 reply 1/0 \| slot L01:2 R03:1 echo 不足/不一致, reply 応答なし/途中まで. slotはサーボごとの失敗フレーム数. |
| `seq_number` | メソッド | 期待するシーケンス番号と実際に受信したシーケンス番号を表示する. |

#### mrd_eeprom.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_eeprom_text_to_bytes` | 関数 | "a.b.c.d"(区切り '.', 10進)や "XX:XX:..."(区切り ':', 16進)の文字列を byte の並びにする. |
| `mrd_eeprom_bytes_to_text` | 関数 | byte の並びを "a.b.c.d"(10進) か "XX:XX:..."(16進) の文字列にする. |
| `mrd_eeprom_page_blank` | 関数 | ページのすべての語が 0xFFFF(一度も書いていない)か. |
| `mrd_eeprom_page_seal` | 関数 | ページに印・版・チェックサムを付ける(語[88]は0). |
| `mrd_eeprom_page_valid` | 関数 | ページが有効(印・版・チェックサムが合う)か. |
| `mrd_eeprom_page_old_servo` | 関数 | 旧形式(版1.4以前. 印なし)のサーボの設定のページか. P1 だけ, 空でなければ旧形式として読む. |
| `mrd_eeprom_page_make` | 関数 | 設定値(cfg, config の配列)からページを作る. |
| `mrd_eeprom_page_apply` | 関数 | ページの値を設定値(cfg, config の配列)に反映する. 範囲外の値は反映せず config の値のまま. |
| `mrd_eeprom_read` | 関数 | EEPROMの内容を作業用の配列(mrd_eep_buf)に読み込む. |
| `mrd_eeprom_is_blank` | 関数 | EEPROMが空(一度も書き込んでいない. 全バイトが0xFF)か. |
| `mrd_eeprom_write` | 関数 | EEPROMにEEPROM格納用の配列データを書き込む(内容が違うbyteだけ書き, まとめて確定する). |
| `mrd_eeprom_settings_boot` | 関数 | 起動の最初に, EEPROM を開始して設定値を決める(シリアルの開始より前. 表示はしない). |
| `mrd_eeprom_boot_report` | 関数 | 起動時の処理の結果を表示する. |
| `mrd_eeprom_page_to_meridim` | 関数 | EEPROM のページを Meridim の [2]〜[87] に入れる. |
| `mrd_eeprom_page_store` | 関数 | Meridim の [2]〜[87] を EEPROM のページに書く(印・版・チェックサムはボードが付ける). |
| `mrd_eeprom_servosettings_load` | 関数 | EEPROM の P1(サーボのスロット)のトリムと回転方向をサーボの設定に反映する(10100, 10102, 10300+1). |
| `mrd_eeprom_dump_to_serial` | 関数 | 使っているページの内容をシリアルに表示する. |
| `mrd_eeprom_dump_at_boot` | 関数 | EEPROM格納用の配列データをシリアルにダンプ出力する.(起動時用) |

#### mrd_sd.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_sd_init` | 関数 | SDカードの初期化を試みる. SDカードがマウントされているか, 及びチップ選択ピンの設定に基づく. 失敗またはSDカードがマウントされていない場合はfalseを返す. |
| `mrd_sd_check` | 関数 | SDカードの読み書き機能をテストする. SDカードがマウントされ, 読み書きのチェックが要求された場合のみテストを実行する. |

#### mrd_wifi.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_wifi_init` | 関数 | wifiを初期化する. |
| `mrd_wifi_udp_receive` | 関数 | 第一引数のMeridim配列にUDP経由でデータを受信, 格納する. 届いた順に1つずつ読む. 遅れて届いたパケットが溜まっていても捨てずに次のフレームで使うため, WiFiの遅延の揺れを吸収でき, 受信待ちのタイムアウトとフレームのスキップが減る. |
| `mrd_wifi_udp_send` | 関数 | 第一引数のMeridim配列のデータをUDP経由で送り先(cfg.wifi_send_ip, cfg.udp_send_port)に送信する. |

#### mrd_ether.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_network_config_validate` | 関数 | IPアドレス設定の妥当性をチェックする |
| `mrd_ip_address_parse` | 関数 | 文字列形式のIPアドレスをIPAddressオブジェクトに変換する |
| `mrd_ether_init` | 関数 | Ethernetを初期化する（文字列IP設定対応版） |
| `mrd_ether_udp_receive` | 関数 | 第一引数のMeridim配列にUDP経由でデータを受信, 格納する. |
| `mrd_ether_udp_send` | 関数 | 第一引数のMeridim配列のデータをUDP経由で指定したIPアドレス、ポートに送信する. |

#### mrd_wire0.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_wire0_i2c_init` | 関数 | Wire0 I2C通信を初期化し, 指定されたクロック速度で設定する. |
| `mrd_wire0_mpu6050_dmp_init` | 関数 | MPU6050センサーのDMP(デジタルモーションプロセッサ)を初期化し, ジャイロスコープと加速度センサーのオフセットを設定する. |
| `mrd_wire0_bno055_init` | 関数 | BNO055センサーの初期化を試みます. |
| `mrd_wire0_setup` | 関数 | 指定されたIMU/AHRSタイプに応じて適切なセンサの初期化を行います. |
| `mrd_wire0_deg_from_origin` | 関数 | 角度から補正センター値を引き, -180以上180未満に折り返す. |
| `mrd_wire0_health_report` | 関数 | センサの読み取りの成否が変わったときだけ表示する(途絶えたとき, 戻ったとき). |
| `mrd_wire0_bno055_alive` | 関数 | BNO055 がI2Cで応答するか(チップIDのレジスタ 0x00 が 0xA0 か). Adafruit のライブラリは読み取りの失敗を返さないため, 読む前にこれで確かめる. |
| `mrd_wire0_bno055_task` | 関数 | bno055からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVALの間隔で実行する. I2Cの読み取りを先に済ませ, ahrs.read への書き込みだけを排他区間で行う. BNO055 が応答しないあいだは値を更新しない(前回の値のまま. 表示で知らせる). |
| `mrd_wire0_mpu6050_read` | 関数 | MPU6050からI2C経由でDMPのデータを読み取り, ahrs.read に格納する. I2Cの読み取りと計算を先に済ませ, ahrs.read への書き込みだけを排他区間で行う. |
| `mrd_ina226_volt100` / `mrd_ina226_amp100` | 関数 | INA226 のバス電圧のレジスタ値を V × 100 に / シャント電圧のレジスタ値とシャント抵抗(mΩ)から電流を A × 100 にする. |
| `mrd_ina226_reg_read` / `mrd_ina226_begin` | 関数 | INA226 の16bitのレジスタを読む / メーカーIDを確かめて設定を書く. |
| `mrd_ina226_task` | 関数 | INA226 から電圧と電流を読むスレッド用関数. INA226_INTERVAL の間隔. 応答しないあいだは前回の値. |
| `mrd_meriput90_ina226` | 関数 | INA226 の電圧と電流を Meridim[86], [87] に書く. 搭載していなければ書かない. |
| `mrd_wire0_mpu6050_task` | 関数 | MPU6050からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVALの間隔で実行する. 新しいデータが 500ms 以上届かなければ, 応答なしとして表示する(値は前回のまま). |
| `mrd_meriput90_ahrs` | 関数 | 指定されたIMU/AHRSタイプに基づいて, 計測したAHRSデータを読み込む. |

#### mrd_bt_pad.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_pad_bt_store` | 関数 | Core0 のスレッドが読んだパッドの値を, 排他区間で pad_bt_val に書き込む. |
| `mrd_pad_krc_read` | 関数 | KRC-5FHジョイパッドからデータを読み取り, 指定された間隔でデータを更新する. |
| `mrd_sbdbt_packet_to_pad` | 関数 | RCB3形式の電文をパッドの値(PadUnion の64bit. [0] ボタン, [1] 左スティック, [2] 右スティック)にする. |
| `mrd_sbdbt_rx_byte` | 関数 | 受け取った1文字を電文に組み立てる. 正しい電文がそろえばパッドの値を更新する. |
| `mrd_sbdbt_rx_sample` | 関数 | 時刻 a_t_us までに中央が来るビットを, 今の信号の状態として読み取る. |
| `mrd_sbdbt_rx_edge` | 関数 | 信号の変化を1つ処理する. 変化の前の状態でビットを読み, LOWへの変化が文字の始まりになる. |
| `mrd_sbdbt_isr` | 関数 | 信号の変化の割り込み. 時刻と変化した後の状態を記録する. |
| `mrd_sbdbt_begin` | 関数 | SBDBTの受信を開始する(ピンを入力にし, 信号の変化の割り込みを登録する). |
| `mrd_pad_sbdbt_read` | 関数 | 記録した信号の変化から電文を組み立て, SBDBTのパッドの値を返す. |
| `mrd_wiiext_write` | 関数 | Wii extension の1つのレジスタに書く. |
| `mrd_wiiext_read` | 関数 | Wii extension のレジスタ a_reg から a_len byte 読む(レジスタ番号を書き, 待ってから読む). |
| `mrd_wiiext_init` | 関数 | Wii extension を暗号化なし・8byte の形式に初期化し, クラシックコントローラ(8byte の形式)と確かめる. |
| `mrd_wiiext_6to8` | 関数 | Wii のクラシックコントローラの標準の形式(6byte. データ形式1)を 8byte の形式(各軸8bit)に直す. |
| `mrd_wiiext_to_pad` | 関数 | Wii extension の 8byte をパッドの値(PadUnion の64bit. [0] ボタン, [1] 左, [2] 右, [3] L2/R2 のアナログ)にする. |
| `mrd_blueretro_read` | 関数 | BlueRetroを1回読み, パッドの値を返す. 初期化が済んでいなければ先に初期化する. |
| `mrd_bt_wiimote_read` | 関数 | Wiiリモコンからの入力データを受信し, 処理する. |
| `mrd_pad_read` | 関数 | 指定されたジョイパッドタイプに応じて最新データを読み取り, 64ビット整数で返す. |
| `mrd_bt_setup` | 関数 | Bluetoothの設定を行い, Wiiコントローラの接続を開始する. |
| `mrd_bt_wiimote_task` | 関数 | サブCPU (Core0) で実行されるBluetooth通信用のルーチン. |
| `mrd_blueretro_task` | 関数 | サブCPU (Core0) で, PAD_INTERVAL ごとに BlueRetro を I2C1(Wire1) で読む. |
| `mrd_meriput90_pad` | 関数 | meridim配列にPADデータを書き込む. trueの場合は既存のデータにビット単位でOR演算を行い, falseの場合は新しいデータで上書きする. |

#### mrd_command.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_command_log` | 関数 | コマンドの実行をシリアルに表示する. 例) "cmd: reset servo error id.[10004]" |
| `mrd_command_trim_home` | 関数 | EEPROM の P1 からトリムと回転方向を読み込み, 全サーボを原点(目標値0)へ動かす(10100, 10102). |
| `mrd_command_group1_execute` | 関数 | Master Commandの第1群を実行する. 受信コマンドに基づき, 異なる処理を行う. |
| `mrd_command_group2_execute` | 関数 | Master Commandの第2群を実行する. 受信コマンドに基づき, 異なる処理を行う. |
| `mrd_command_group3_execute` | 関数 | Master Commandの第3群を実行する. 受信コマンドに基づき, 異なる処理を行う. |

#### mrd_move.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_move_init` | 関数 | モーション初期化のためのスタブ関数. 現在は何も行わず, 常にfalseを返す. |

#### mrd_servo.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `__attribute__` | 関数 | TXピンの出力をON/OFFする. UARTとピンの対応付け(GPIOマトリクス)は変更しない. (gpio_ll_output_disable()は対応付けも外してしまうため使わない) 事前に GPIO.func_out_sel_cfg[a_pin].oen_sel = 1 で出力有効の制御をGPIOレジスタ側にしておくこと. |
| `__attribute__` | 関数 | UARTの送信完了を待ってDIRピンをLOW(受信)にする. サーボは送信完了の直後に応答を始めるため, DIRの切替が数us遅れるだけで応答の先頭が欠ける. フラッシュキャッシュのミスや割り込みの影響を受けないよう, IRAM上で割り込みを止めた状態で 送信完了をレジスタで待ち, DIRをレジスタで直接切り替える. 割り込みを止めるのは送信FIFOの残りが MRD_HDUART_CRITICAL_BYTES 以下になってからとし, 長いパケット(応答のないFEETECHのSyncWrite等)でも割り込み停止時間を8バイト分(1Mbpsで約80us)以内に抑える. 8バイト以下のパケット(ICSの各命令, FEETECHのREAD)は従来どおり送信直後から割り込みを止める. (クラス内定義のインライン関数にはIRAM_ATTRを付けられないため, 非インラインの関数として定義) |
| `__attribute__` | 関数 | 複数系統の送信開始から受信切替までを, 割り込みを止めた状態でまとめて行う. 各系統を送信方向にして送信FIFOへ直接書き込み, 送信が終わった系統から即座にDIRをLOW(受信)にする. 送信開始と受信切替の間にソフトウェアの処理を挟まないため, 他の系統の送信開始処理や割り込みで 受信切替が遅れて応答の先頭が欠けることがない(応答遅延0のFEETECHサーボでも間に合う). 割り込み停止時間は最も長い系統の送信時間(FEETECHのREAD 8バイト@1Mbpsで約80us). |
| `__attribute__` | 関数 | 返信をホストの区切りで受ける(Hitec HSRの読み出し命令). 送信完了後(送信方向のまま)に呼ぶ. Hitecのサーボは返信のスタート/ストップビットを出さず, ホストが送る0x00のデータビットの間だけ 線をLOWに引いて返信を書き込む. そこで各系統の送信FIFOへ0x00を a_bytes 個書き込み, TXピンでスタートビットを検出した瞬間に線を開放し, 最後のデータビットの読み取り後に送信方向へ戻す. ストップビットはUART自身が出すため, 区切りはUARTの実際の出力と一致する. 返信はRXに入る(エコーの後). 信号反転(アイドルLOW, スタートビットHIGH)の線のみ対応. 2系統の区切りは独立して扱う. 割り込み停止時間は a_bytes 分の送信時間(19200bps 8N2の2バイトで約1.15ms). |
| `MrdHalfDuplexUart` | メソッド | コンストラクタ. |
| `begin` | メソッド | 通信を開始する. プロトコル切替時の再設定にも使用できる. falseなら受信方向で待つ(ICS, FEETECH等). |
| `idle` | メソッド | 送受信の合間の待機状態にする(begin()で送信方向待機を指定した場合は送信方向, それ以外は何もしない). 送受信が終わるたびに呼ぶ(transfer() と mrd_hduart_xfer_poll() は自動で呼ぶ). |
| `configure` | メソッド | 送信エコー・受信タイムアウト・TXの切り離しを設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ. |
| `echo` | メソッド | 送信エコーを処理しているか(ECHO_AUTO時は判定結果). |
| `timeout_ms` | メソッド | 既定の受信タイムアウト(ms). |
| `transfer` | メソッド | 1回の送受信を行う(begin_send → end_send → receive). |
| `begin_send` | メソッド | 送信を開始する. 送信完了は待たない. |
| `end_send` | メソッド | 送信完了を待ち, 受信に切り替える(詳細は mrd_hduart_wait_tx_and_dir_rx). RXバッファは保持される(エコーと応答はこの後 receive() で読む). |
| `prepare_send` | メソッド | 送信の準備(前回の残りデータの破棄とエコー照合用の控え)だけを行う. 送信は send_multi() で行う. |
| `send_multi` | メソッド | prepare_send() 済みの系統を, 送信開始から受信切替まで割り込みを止めてまとめて行う (詳細は mrd_hduart_send_and_dir_rx_multi). RXバッファは保持される(この後 receive() で読む). |
| `framed_reply_multi` | メソッド | begin_send() 済みの系統の送信完了を待ち, 返信 a_bytes バイトをホストの区切りで受ける (詳細は mrd_hduart_framed_reply_multi). 終了時は送信方向. 返信はこの後 receive() で読む. |
| `receive` | メソッド | 送信エコーを照合して読み捨て, 応答を受信する. 複数のサーボが続けて応答する命令(DYNAMIXELのSyncRead等)で長く待つ場合に指定する. |
| `last_rx_len` | メソッド | 直前の receive() で受信できた応答のバイト数(途中までの応答を解析する場合に使う). |
| `tx_done` | メソッド | begin_send() で送り始めたデータの送信が終わったか(送信FIFOが空で, 送信中でない). |
| `rx_available` | メソッド | 受信バッファにあるバイト数(エコーを含む). |
| `receive_now` | メソッド | 受信バッファにあるデータだけで receive() を行う(待たない). エコーが揃っていなければ RX_ECHO_SHORT, 応答が足りなければ途中まで読んで失敗とする. |
| `baud` | メソッド | 通信速度bps. |
| `tx_len` | メソッド | 直前の送信データ数(エコーのバイト数). |
| `set_baud` | メソッド | 通信速度だけを切り替える(1系統に2種類のサーボを混在させる場合. begin()より軽い). フレーム形式と信号の向きは変えないため, 8N1のプロトコル同士でのみ使う. 送受信の合間に呼ぶ. |
| `last_rx` | メソッド | 直前の receive() の結果の種類(RxResult). |
| `flush_rx` | メソッド | 受信バッファに残っているデータを読み捨てる. |
| `detect_echo` | メソッド | ダミーバイトを送信し, RXにエコーされるかを判定する. 0x55はICSではコマンドバイト(MSB=1)ではなく, DXL/FEETECH/FUTABAではヘッダを欠くため, いずれのサーボにも無視される. エコーのない回路ではダミーバイトは1バイトも返らないため, 1回でも受信できればエコーありとする (通信速度の切替直後の雑音等で1回取りこぼしても誤判定しないため). |
| `mrd_hduart_xfer_start` | 関数 | 1回の送受信を始める(待たない). 続けて mrd_hduart_xfer_poll() で完了まで進める. 応答あり: 送信開始から受信切替まで割り込みを止めて行う(send_multi. 応答の先頭を欠かさないため). 応答なし, 区切り受信: ドライバ経由で送り始め, 送信完了は mrd_hduart_xfer_poll() で見張る. |
| `mrd_hduart_xfer_poll` | 関数 | mrd_hduart_xfer_start() で始めた送受信を進める(待たない). 呼ぶたびにできるところまで進める. 送信完了 → 受信切替(または区切り受信) → エコーと応答の受信 → 返信枠の待ち → 待機状態(idle) の順. 受信は「エコーと応答が揃う」か「タイムアウト(転送時間 + 受信タイムアウト)」で終える. |
| `next` | メソッド | 次の送受信を準備する. |
| `complete` | メソッド | next()で準備した送受信の結果を反映する. |
| `skip_reads` | メソッド | このフレームは現在位置を読まない(1系統に2種類のサーボを混在させた場合の交互読み出し). next()より前に呼ぶ. 読まないサーボは前回の値になる. 混在に対応しないプロトコルは何もしない. |
| `mrd_servo_lines_run` | 関数 | 複数系統の処理を, 系統ごとに独立して最後まで実行する. 各系統は送受信が終わり次第, 結果を反映して次の送受信を始める. 片方の系統が長い応答を待つ間も, もう片方は次の送受信を進めるため, 全体の時間は「長い方の系統」でほぼ決まる. 系統の並び(L/R)やプロトコルの組み合わせに依存しない. 応答の先頭を欠かさないための割り込み停止(送信から受信切替まで)は系統ごとに行い, その間に もう片方の系統へ届いたデータはUARTの受信FIFO(128byte)に保持される. |
| `mrd_servo_slot_mode` | 関数 | コマンドの値からスロットの扱いを決める. |
| `mrd_servo_cmd` | 関数 | Meridimのサーボコマンドの下位8bit(汎化コマンド). |
| `mrd_servo_param_in_line` | 関数 | 系統に通常以外(パラメータ系コマンド, 車輪, スルー)のスロットがあるか. |
| `mrd_servo_param_error` | 関数 | パラメータ系コマンドの結果がエラー値か. |
| `mrd_servo_family` | 関数 | 同じプロトコルとして扱うサーボタイプの代表値(FEETECHのSTS/SCSは61). |
| `mrd_servo_is_mixable` | 関数 | 1系統に混在させられるサーボタイプか. |
| `mrd_servo_line_mount` | 関数 | 系統と種類に対応するマウント状態を返す. 混在なしでパラメータ系コマンドもなければ系統のマウント状態 そのもの, それ以外はその種類のスロットだけをマウントとした配列(静的領域. 呼ぶたびに作り直す). falseならパラメータ系コマンドの実行用: 除かない. |
| `mrd_servo_mix_check` | 関数 | 系統の2種類目のサーボタイプを検査して設定する. 混在できない組み合わせなら混在なしとし, どちらの種類にも当たらないスロットはマウントを外す(警告を表示する). |
| `mrd_servo_param_supported` | 関数 | パラメータ系コマンドに対応するサーボタイプか(ICS, DYNAMIXEL 1.0/2.0, FEETECH, FUTABA, PMX, Hitec). |
| `mrd_servo_wheel_supported` | 関数 | 車輪(bit13)に対応するサーボタイプか(DYNAMIXEL 1.0/2.0, FEETECH, PMX). 車輪モードはサーボ側で事前に設定する. |
| `mrd_servo_param_prepare` | 関数 | フレームの最初に, スロットごとのコマンドからスロットの扱いを決め, 実行待ちを決める. パラメータ系コマンド: 受け取ったコマンドとVALを前回と比べ, 変わっていれば実行待ちにする. 車輪: 毎フレーム実行待ちにする(VALは速度). スルー・通常: 状態を消す(パラメータ系コマンドは再実行できる). マウントしていないスロットと, 対応しないサーボのスロットは未対応の結果にする(通信しない). |
| `mrd_servo_motion_target` | 関数 | 受け取った目標位置から, このフレームの指令位置を求める. 目標(VAL)が変わったら, いまの指令位置から新しい目標まで a_frames フレームかけて直線で補間する. a_frames が0か1なら即時. 補間の途中で目標が変わったら, その時点の指令位置から補間し直す. |
| `mrd_servo_motion_reset` | 関数 | 補間をやめる(トルクOFF, パラメータ系コマンド等). 次に目標を受け取ったときに始点から補間し直す. |
| `mrd_size_max` | 関数 | 2つの値の大きい方(コンパイル時定数用). |
| `mrd_servo_stepper_make` | 関数 | 系統のサーボタイプに応じたステッパーを a_buf に生成する. 使い終わったら ~MrdLineStepper() を呼ぶこと. |
| `mrd_servo_mix_bus` | 関数 | 混在できるサーボタイプの, 系統のプロトコル層が使う半二重UARTバス. |
| `mrd_servo_mix_baud` | 関数 | 混在できるサーボタイプの, 系統のプロトコル層が使う通信速度. (バスの現在の通信速度ではなく, そのプロトコルに設定した値. バスの値は切り替えるたびに変わるため) |
| `mrd_servo_line_make` | 関数 | 系統の処理を作る. 2種類目のサーボタイプが設定された系統(mrd_servo_type2)は, 種類ごとの処理をまとめた MrdMixedLineStepper を a_mix に生成する. |
| `mrd_servo_begin` | 関数 | 1系統のサーボの通信を, サーボタイプのプロトコルで開始する(接続の確認と初期設定を含む). |
| `mrd_servo_line_begin` | 関数 | 1系統分のサーボの通信を初期設定する. 2種類目のサーボタイプがあれば混在を検査し, 種類ごとに初期設定する (最後に初期設定した種類の通信速度になるが, 駆動時は種類ごとに切り替える). |
| `mrd_servo_drive_lite` | 関数 | 指定されたサーボにコマンドを分配する. |
| `mrd_servo_slot_reply` | 関数 | 1スロット分の返信値(Meridimのコマンドと角度)を送信配列に格納する. PWM専用スロット(種類1/11)は指令値, マウントしたサーボは現在位置(戻り値のないサーボは指令値)を返す. マウントしていないスロットは, SERVO_UNMOUNT_REPLY が0ならコマンド・角度とも0にし(tgtも0), 1なら受信した値のまま書き換えない. パラメータ系コマンドと車輪はその結果(車輪は現在速度)をVALに, 結果がエラー値ならコマンドのbit14を立てて返す. スルーは受信した値をそのまま返す. |
| `mrd_servo_all_off` | 関数 | 第一引数のMeridim配列のすべてのサーボモーターをオフ(フリー状態)に設定する. |
| `mrd_servo_errcode_make_lite` | 関数 | サーボパラメータからエラーのあるサーボのインデックス番号を作る. 100-114(L系統 0-14), 200-214(R系統 0-14). 該当が複数あれば最後のもの(R系統優先). なければ0 |

#### mrd_module/sv_dxl1.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_dxl1_scale` | 関数 | 機種の位置値の尺度. 中央(AX/RX:512, MX:2048)が0deg. |
| `mrd_dxl1_pos2deg` | 関数 | 位置値を角度(degree)に変換する. |
| `set_baud` | メソッド | 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ. |
| `begin` | メソッド | バスを開始(8N1)し, 接続されたサーボのモデル番号を読み, 応答遅延時間を0にする. |
| `read` | メソッド | メモリを読み出す(1回の送受信). |
| `write` | メソッド | メモリに書き込み, 応答を確認する(1回の送受信). |
| `mrd_dxl1_line` | 関数 | 系統のDYNAMIXEL Protocol 1.0 サーボクラス. |
| `mrd_dxl1_line_ref` | 関数 | 系統の参照を作る. |
| `mrd_dxl1_param_reg` | 関数 | パラメータ系コマンドの対応表. EEPROMは書いた時点で保存される(35は成功を返す). |
| `begin_wheel` | メソッド | 車輪(車輪モード=CW/CCW角度限界0を事前に設定したサーボ). 速度は0.01rpm(回転方向補正を適用). |
| `build_goal` | メソッド | 目標位置のSyncWriteを作る(トルクONのサーボのみ). |
| `build_torque` | メソッド | トルクのSyncWriteを作る. ON/OFFが変わったサーボと, 定期的な送り直しのフレームでは全サーボ. |
| `model` | メソッド | スロットの機種番号(起動時か[3]で読んだもの. 0:分からない). |

#### mrd_module/sv_dxl2.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_dxl2_crc` | 関数 | CRCを計算する. |
| `finish` | メソッド | バイトスタッフィング, LEN, CRCを確定する. |
| `mrd_dxl2_status_check_err` | 関数 | 応答パケットのヘッダ, ID, 長さ, 命令欄, CRCを検証し, エラー番号(bit7のアラートを除く)を取り出す. エラー番号が0でなくても, 形式が正しければtrueを返す(書き込みの拒否などを区別するため). |
| `mrd_dxl2_status_check` | 関数 | 応答パケットを検証する(形式が正しく, エラー番号が0). bit7(アラート: ハードウェアエラー発生中)は データが有効なため許容する. |
| `mrd_dxl2_i32_get` | 関数 | 4バイト値(LE, 符号付き)を読み取る. |
| `mrd_dxl2_deg2pos` | 関数 | 角度(degree)をDYNAMIXELの位置値(0-4095)に変換する. |
| `mrd_dxl2_pos2deg` | 関数 | DYNAMIXELの位置値を角度(degree)に変換する. |
| `Dynamixel2HalfDuplexClass` | メソッド | コンストラクタ. |
| `set_baud` | メソッド | 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ. |
| `begin` | メソッド | バスを開始(8N1)し, 接続されたサーボの確認と初期設定を行う. |
| `ping` | メソッド | PINGでサーボの応答を確認する. |
| `read` | メソッド | レジスタを読み出す(1回の送受信). |
| `write` | メソッド | レジスタに書き込み, 応答を確認する(1回の送受信). |
| `bus` | メソッド | 半二重UARTバス(並行送受信用). |
| `baud` | メソッド | 通信速度bps(SyncReadの受信待ち時間の計算用). |
| `fix_return_delay` | メソッド | 応答遅延時間(EEPROM)を0にする. EEPROMはトルクOFFでないと書き込めないため先にトルクOFFにする (起動直後はトルクOFFなので通常は影響なし. トルクは駆動処理が指示に合わせて送り直す). |
| `mrd_dxl2_line` | 関数 | 系統のDYNAMIXELサーボクラス. |
| `mrd_dxl2_line_ref` | 関数 | 系統の参照を作る. |
| `mrd_dxl2_param_reg` | 関数 | パラメータ系コマンドの対応表. EEPROM領域(64番地未満)はトルクOFFでないと書けない(サーボがエラーを返す). |
| `begin_wheel` | メソッド | 車輪(速度制御モードを事前に設定したサーボ): トルクON(必要なときだけ) → 目標速度(104) → 現在速度(128). 速度は0.01rpm(回転方向補正を適用). 結果は現在速度. |
| `build_goal` | メソッド | 目標位置のSyncWriteを作る(トルクONのサーボのみ). |
| `build_torque` | メソッド | トルクのSyncWriteを作る. ON/OFFが変わったサーボと, 定期的な送り直しのフレームでは全サーボ. ONは目標位置を送った後に行い, 古い目標位置へ動くのを防ぐ. |
| `build_sync_read` | メソッド | 前フレームで読めたサーボ(err==0)のSyncReadを作る. 読めなかったサーボはこのフレームの値を前回の値にしておく([4]で読めたものは上書きされる). |
| `sync_timeout_ms` | メソッド | SyncReadの応答を待つ時間(ms). 全応答の転送時間にバスの既定のタイムアウトを加える. |
| `complete_sync_read` | メソッド | SyncReadの応答を解析する. 応答はID順に続くが, 応答しないサーボがあっても対応を誤らないよう, ヘッダとCRCが正しいパケットをIDで照合する. 見つからなかったサーボのうち, 最後に受信できたサーボより前のものは失敗とする. それより後ろは先頭の1台だけを失敗とし, 残りは前回の値のまま次のフレームで読み直す (先頭の1台が応答しないために後続が応答できなかった可能性があるため. 先頭の1台は次のフレームで SyncReadから外れるので, 後続は次のフレームで読める). |
| `find_sync` | メソッド | SyncReadの対象からIDを探す. |
| `build_retry` | メソッド | SyncReadから外したサーボを1台だけ個別のREADで確認する(順番に1フレーム1台). このフレームのSyncReadで失敗したサーボは対象にしない(同じフレームで二重に数えないため). |
| `complete_one` | メソッド | 1台分の現在位置の応答を反映する. |

#### mrd_module/sv_ftbrx.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_ftb_checksum` | 関数 | チェックサムを計算する(ID以降, SUMの手前まで). |
| `mrd_ftb_packet_make_read` | 関数 | 指定アドレスの返信を要求するショートパケットを作る. |
| `mrd_ftb_reply_check` | 関数 | 返信パケットのヘッダ, ID, アドレス, 長さ, チェックサムを検証する. |
| `add` | メソッド | サーボを1台追加する. |
| `finish` | メソッド | CNTとSUMを確定する. |
| `mrd_ftb_deg2pos` | 関数 | 角度(degree)をRS30xの位置値(0.1deg単位, ±1500)に変換する. |
| `mrd_ftb_pos2deg` | 関数 | RS30xの位置値を角度(degree)に変換する. |
| `FutabaHalfDuplexClass` | メソッド | コンストラクタ. |
| `set_baud` | メソッド | 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ. |
| `begin` | メソッド | バスを開始(8N1)し, 接続されたサーボの応答と返信ディレイ時間を確認する. |
| `read` | メソッド | メモリーマップを読み出す(1回の送受信). |
| `bus` | メソッド | 半二重UARTバス(並行送受信用). |
| `baud` | メソッド | 通信速度bps. |
| `mrd_ftb_line` | 関数 | 系統のFUTABAサーボクラス. |
| `mrd_ftb_line_ref` | 関数 | 系統の参照を作る. |
| `mrd_ftb_param_reg` | 関数 | パラメータ系コマンドの対応表. ROM領域(アドレス4〜29)は書いた時点で有効, 保存は35(ROM保存). |
| `build_torque` | メソッド | トルクのロングパケットを作る. ON/OFFが変わったサーボと, 定期的な送り直しのフレームでは全サーボ. |
| `build_goal` | メソッド | 目標位置のロングパケットを作る(トルクONのサーボのみ). |
| `select_reads` | メソッド | このフレームで現在位置を読むサーボを, 前回の続きから最大 SERVO_FTB_READ_PER_FRAME 台選ぶ. 読まないサーボは前回の値にする. 読まないフレーム(skip_reads)は1台も読まず, 順番も進めない. |

#### mrd_module/sv_ftc.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_ftc_is_ftc` | 関数 | サーボタイプがFEETECH(STS/SCS)かを判定する. |
| `mrd_ftc_resolution` | 関数 | 系列ごとの位置値の分解能(STS:4096, SCS:1024). |
| `mrd_ftc_range` | 関数 | 系列ごとの位置値の全範囲に対応する角度(degree). |
| `mrd_ftc_scale` | 関数 | 系列ごとの位置値の尺度. 中央(STS:2048, SCS:512)が0deg. |
| `mrd_ftc_step_deg` | 関数 | 1ステップあたりの角度(degree). 速度(ステップ/秒)や不感帯(ステップ)の換算に使う. |
| `mrd_ftc_deg2pos` | 関数 | 角度(degree)をFEETECHサーボの位置値に変換する. 中央が0deg. |
| `mrd_ftc_pos2deg` | 関数 | FEETECHサーボの位置値を角度(degree)に変換する. mrd_ftc_deg2pos の逆変換. |
| `mrd_ftc_u16_put` | 関数 | 2バイト値を系列のバイト順で書き込む(STS:LE, SCS:BE). |
| `mrd_ftc_pos_get` | 関数 | 位置値を系列のバイト順で読み取る. STSはbit15が符号(マルチターン時の負の位置). |
| `mrd_ftc_checksum` | 関数 | チェックサムを計算する(ID以降, CHKの手前まで). |
| `mrd_ftc_packet_make_ping` | 関数 | PING命令パケット(応答確認)を作る. |
| `mrd_ftc_packet_make_read` | 関数 | READ命令パケットを作る. |
| `mrd_ftc_packet_make_write` | 関数 | WRITE命令パケットを作る. |
| `mrd_ftc_reply_check` | 関数 | 応答パケットのヘッダ, ID, 長さ, チェックサムを検証する. |
| `add` | メソッド | サーボを1台追加する. |
| `finish` | メソッド | LENとCHKを確定する. |
| `FeetechHalfDuplexClass` | メソッド | コンストラクタ. |
| `set_baud` | メソッド | 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ. |
| `begin` | メソッド | バスを開始(8N1)し, 接続されたサーボの系列確認と初期設定を行う. 系列はconfigの指定を使い, a_type に書き戻す(駆動時はこの値でSTS/SCSを切り替える). |
| `read` | メソッド | レジスタを読み出す(1回の送受信). |
| `write` | メソッド | レジスタに書き込み, 応答を確認する(1回の送受信). |
| `bus` | メソッド | 半二重UARTバス(並行送受信用). |
| `baud` | メソッド | 通信速度bps. |
| `fix_return_delay` | メソッド | 応答遅延時間(EEPROM)を0にする. EEPROMのロックを解除して書き込み, 再びロックする. |
| `mrd_ftc_line` | 関数 | 系統のFEETECHサーボクラス. |
| `mrd_ftc_line_ref` | 関数 | 系統の参照を作る. |
| `mrd_ftc_param_reg` | 関数 | パラメータ系コマンドの対応表. 2バイト値のバイト順は系列に従う(STS:LE, SCS:BE). EEPROM領域はロック解除 → 書き込み → 再ロックで保存まで行う(トルクの状態は問わない). |
| `begin_wheel` | メソッド | 車輪(車輪モードを事前に設定したサーボ): トルクON(必要なときだけ) → 目標速度(46) → 現在速度(58). 速度は0.01rpm(回転方向補正を適用). STSはbit15, SCSはbit10が回転方向. 結果は現在速度. |
| `rx_u16` | メソッド | READの応答の2バイト値(系列のバイト順で読む). |
| `get_raw` | メソッド | READの応答からレジスタの値を取り出す(符号を解いた値). |
| `build_goal` | メソッド | 目標位置のSyncWriteを作る(系列ごとに1パケット. STS:位置, SCS:位置+時間+速度). |
| `build_torque` | メソッド | トルクのSyncWriteを作る. ON/OFFが変わったサーボだけ送る. ただし MRD_SERVO_TORQUE_REFRESH_FRAMES ごとに全サーボへ送り直す (サーボの保護機能でトルクが切れると, ESP32側の記憶と食い違ったまま脱力し続けるため). ONは目標位置を送った後に行い, 古い目標位置へ動くのを防ぐ. |
| `select_read_series` | メソッド | このフレームで現在位置を読む系列を決める. STSとSCSが混在する場合だけ, フレームごとに交互にする. 読まないフレーム(skip_reads)はどの系列も読まず, 交互の順番も進めない. |
| `keep_unread` | メソッド | このフレームで現在位置を読まない系列のサーボを前回の値にする. |

#### mrd_module/sv_hitec.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_htc_packet_make` | 関数 | 5バイトの命令パケットを作る. |
| `mrd_htc_deg2us` | 関数 | 角度(degree)をHitecのパルス幅(us)に変換する(550-2450usに制限). |
| `mrd_htc_us2deg` | 関数 | Hitecのパルス幅(us)を角度(degree)に変換する. mrd_htc_deg2us の逆変換. |
| `HitecHalfDuplexClass` | メソッド | コンストラクタ. |
| `set_baud` | メソッド | 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ. |
| `begin` | メソッド | バスを開始(8N2, 信号反転, 送信方向で待機)し, 設定を表示する. 読み出しの宛先なし命令は複数台で返信が衝突するため, 起動時のサーボの確認はしない. |
| `reply_hold_us` | メソッド | 返信枠の待ち時間(us). |
| `bus` | メソッド | 半二重UARTバス(並行送受信用). |
| `mrd_htc_line` | 関数 | 系統のHitecサーボクラス. |
| `mrd_htc_line_ref` | 関数 | 系統の参照を作る. |
| `complete_main` | メソッド | 読み出しの結果を反映する. 位置命令と脱力の返信は読まない(指令値をそのまま返す). |

#### mrd_module/sv_ics.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `IcsHalfDuplexClass` | メソッド | コンストラクタ. |
| `set_baud` | メソッド | 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ. |
| `begin` | メソッド | ICS用のフレーム形式(8E1)でバスを開始する. |
| `bus` | メソッド | 半二重UARTバス(並行送受信用). |
| `mrd_ics_result2deg` | 関数 | ICSサーボの応答(ポジション値)をエラー処理して角度に変換する. |
| `mrd_ics_raw2temp` | 関数 | ICSの温度値を℃にする(表の折れ線で補間, 表の外は端の傾きで延長). |
| `mrd_ics_temp2raw` | 関数 | ℃をICSの温度値にする(mrd_ics_raw2temp の逆. 1〜127). |
| `eeprom_addr` | メソッド | 50以降のEEPROMの項目のアドレス(4bit単位の位置). 対応しなければ-1. |
| `get_field` | メソッド | EEPROMの項目を読む(4bitずつ, 上位が先). |
| `put_field` | メソッド | EEPROMの項目を書く. |
| `from_raw` | メソッド | 生値をVALにする. |
| `to_raw` | メソッド | VALを生値にする(EEPROMの項目). |
| `sub_value` | メソッド | VALをサブコマンドの書き込み値にする. |
| `mrd_ics_line` | 関数 | 系統のICSサーボクラス. |
| `mrd_ics_line_ref` | 関数 | 系統の参照を作る. |

#### mrd_module/sv_pmx.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_pmx_crc16` | 関数 | CRC16を計算する. |
| `mrd_pmx_packet_make` | 関数 | パケットを作る. |
| `mrd_pmx_reply_check` | 関数 | 返信パケットのヘッダ, ID, 長さ, コマンド, CRCを検証する. |
| `mrd_pmx_deg2pos` | 関数 | 角度(degree)をPMXの位置値(0.01deg単位, ±32000)に変換する. |
| `mrd_pmx_pos2deg` | 関数 | PMXの位置値を角度(degree)に変換する. |
| `PmxHalfDuplexClass` | メソッド | コンストラクタ. |
| `set_baud` | メソッド | 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ. |
| `begin` | メソッド | バスを開始(8N1)し, 接続されたサーボの応答を確認して制御モードと応答データを設定する. |
| `request` | メソッド | 1回の送受信を行う. |
| `bus` | メソッド | 半二重UARTバス(並行送受信用). |
| `baud` | メソッド | 通信速度bps. |
| `mrd_pmx_line` | 関数 | 系統のPMXサーボクラス. |
| `mrd_pmx_line_ref` | 関数 | 系統の参照を作る. |
| `mrd_pmx_packet_make_mode` | 関数 | 制御モードと応答データ(501, 502)を書き込むパケットを作る(トルクONでも書けるオプション1). |
| `mrd_pmx_param_reg` | 関数 | パラメータ系コマンドの対応表. 書き込みはRAMのみ(保存は35: SAVE). 書き込みはトルクONでも書けるオプション1で送る(オフセットはトルクOFFでのみ書けるためオプション0). |
| `begin_wheel` | メソッド | 車輪: 制御モードを速度にし(RAM. 変わったときだけ), トルクONにして(必要なときだけ), 速度を送る. 速度は0.01rpm. 返信の現在速度(0.1deg/s)を結果(0.01rpm, 回転方向補正を適用)にする. |

#### mrd_module/sv_pwm.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_pwm_deg2us` | 関数 | 角度(degree)をPWMのパルス幅(us)に変換する. サーボ側の角度(トリム + 指令 × 回転方向)を下限〜上限に制限し, 中央と+90degのパルス幅で直線に換算する. |
| `mrd_pwm_slot_us` | 関数 | 対応表のスロットの指令を取得する. |
| `mrd_pca_prescale` | 関数 | 50Hzにするプリスケール値. PRESCALE = round(発振周波数 / (4096 x 50)) - 1. |
| `mrd_pca_us2count` | 関数 | パルス幅(us)をPCA9685のカウント(0-4095)に変換する. 実際のプリスケール値から周期を求める. |
| `mrd_pca_regs_fill` | 関数 | 16ch分の出力レジスタ(LED0_ON_L〜LED15_OFF_H, 64バイト)を作る. パルスを出すchは ON=0, OFF=カウント. 出さないchは完全OFF(OFF_Hのbit4). |
| `begin` | メソッド | 応答を確認し, 50Hz・全ch OFFで開始する. |
| `write_all` | メソッド | 16ch分の出力を1回の通信で書き込む. 書き込めなくなったら(ケーブルの外れ等)表示し, 約1秒(この関数50回)ごとに初期化からやり直す. 応答がないあいだは毎回のI2Cのタイムアウト待ちでフレームが遅れないよう, 書き込まない. |
| `set_addr` | メソッド | I2Cアドレスを設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ. |
| `print_state` | メソッド | 状態の変化を表示する. 例) "PCA9685 0x40: restored." |
| `probe` | メソッド | I2Cで応答するか. |
| `setup` | メソッド | 50Hz・全ch OFFに設定する. |
| `send_all` | メソッド | 16ch分の出力を書き込む. |
| `mrd_pwm_begin` | 関数 | PWM出力を開始する(PWM_PIN_MODE に応じて本体ピンまたはPCA9685). |
| `mrd_pwm_drive` | 関数 | PWM出力を更新する. 本体ピンは偶数フレーム, PCA9685は1枚目を偶数・2枚目を奇数フレームで書き込む. UARTのサーボの処理より前に呼び, 現在位置で上書きされる前の指令値を使う. |

#### mrd_module/sv_util.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_servo_deg2pos` | 関数 | 角度(degree)を位置値に変換する. |
| `mrd_servo_pos2deg` | 関数 | 位置値を角度(degree)に変換する. mrd_servo_deg2pos の逆変換. |
| `mrd_servo_scale_deg2pos` | 関数 | 尺度を使って角度(degree)を位置値にする. |
| `mrd_servo_scale_pos2deg` | 関数 | 尺度を使って位置値を角度(degree)にする. |
| `mrd_servo_crc16` | 関数 | CRC16(左送り, 初期値0, 反転なし)を計算する. 多項式はプロトコルで違う(DYNAMIXEL 2.0: 0x8005, PMX: 0x1021). |
| `mrd_servo_slot_print` | 関数 | 起動時の1台分の表示の先頭 "UART_L XXX [i] ID n: " を出す(続けて結果を表示する). |
| `mrd_servo_baud_check` | 関数 | 通信速度がサーボで設定できる値かを確認し, 違えば警告を表示する. |
| `torque_tick` | メソッド | 1フレームに1回呼び, 全サーボへトルク状態を送り直すフレームならtrueを返す. |
| `on` | メソッド | サーボのトルクONが指示されているか(コマンドの下位8bitが1). |
| `torque_due` | メソッド | トルク状態を送る必要があるかを判定する. 送る場合は a_want に送る値(0/1)を入れる. 送ったら st->torque[a_ix] = a_want として記憶すること. |
| `next_mounted` | メソッド | a_ix より後ろで, 次に処理するスロットを探す. |
| `mrd_servo_torque_each` | 関数 | トルク状態を送る必要があるサーボ(ON/OFFが変わったサーボ. 定期的な送り直しのフレームでは全サーボ)ごとに a_put(スロット, 送る値0/1) を呼ぶ. a_put が true を返した(パケットに入れた)サーボは, その値を記憶する. |
| `mrd_servo_line_ref` | 関数 | 系統の参照を作る. |
| `mrd_servo_read_apply` | 関数 | 現在位置の読み取り結果を反映する. |
| `mrd_servo_past_keep` | 関数 | このフレームで現在位置を読まないサーボの値を前回の値にする(エラー数は変えない). |
| `mrd_servo_past_keep_all` | 関数 | 全サーボの値を前回の値にする(現在位置を1台も読まないフレーム). |
| `mrd_servo_xfer` | 関数 | 1回の送受信を準備する. 指定しない項目(hold_us, framed)は既定値のまま. |
| `mrd_pc_reg` | 関数 | レジスタの定義を作る(各プロトコルの対応表で使う. 引数は MrdParamReg のコンストラクタと同じ). |
| `mrd_pc_sign_bit` | 関数 | 符号ビットの形式(PC_BIT15/11/10)の符号ビットの位置. それ以外は-1. |
| `mrd_pc_decode` | 関数 | レジスタの値(リトルエンディアンで組み立てた値)を符号付きの整数にする. |
| `mrd_pc_encode` | 関数 | 符号付きの整数をレジスタの値にする(mrd_pc_decode の逆). |
| `mrd_pc_fits` | 関数 | レジスタの値(符号を付ける前)が, レジスタの幅(byte数)と符号の形式で表せるか. |
| `mrd_pc_clamp` | 関数 | 結果をVALの範囲に収める(エラー値と重ならないよう -32765 以上にする). |
| `mrd_wheel_rpm` | 関数 | 車輪の速度のVALを rpm にする. |
| `mrd_wheel_val` | 関数 | rpm を車輪の速度のVAL(0.01rpm)にする(結果の範囲に収める). |
| `mrd_pc_le_get` | 関数 | バイト列(LE)から値を組み立てる. |
| `mrd_pc_le_put` | 関数 | 値をバイト列(LE)にする. |
| `begin_wheel` | メソッド | 車輪に対応しないプロトコルの begin_wheel (未対応で終える). |
| `target` | メソッド | 対象のサーボを記憶する(各プロトコルの begin() の最初に呼ぶ). |
| `finish` | メソッド | 結果を決めて終える. |
| `deg_to_val` | メソッド | 角度(deg)をVAL(deg x100)にする. |
| `to_val` | メソッド | レジスタの値(符号を解いた値)をVALにする. 角度は位置値の尺度 m_scale で換算する. |
| `write_value_ok` | メソッド | 書き込む値がレジスタの幅と符号に収まるか(読み出しは常にtrue). 収まらない値は送らない(未対応を返す). |
| `to_raw` | メソッド | 受け取ったVALをレジスタの値(符号を付ける前の値)にする(to_val の逆). |
| `wheel_needs_torque` | メソッド | 車輪: トルクONを送る必要があるか(状態が不明・OFFのとき, または通常の処理と同じ送り直しの間隔). |
| `finish_slot` | メソッド | 結果をスロットに記録する. |
| `start_next_slot` | メソッド | 次の実行待ちのスロットを探して1件分の処理を始める. |

#### mrd_module/sv_xbus.h

| 名前 | 種別 | 説明 |
|---|---|---|
| `mrd_xbus_crc8` | 関数 | CRC-8(反転多項式0x8C, 初期値0). 先頭からCRCの手前まで. |
| `mrd_xbus_deg2pos` | 関数 | 角度(degree)をXBUSの位置値に変換する(900-2100usの範囲に制限). |
| `XbusHalfDuplexClass` | メソッド | コンストラクタ. |
| `set_baud` | メソッド | 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ. |
| `begin` | メソッド | バスを開始(8N1, 送信方向で待機)し, 設定を表示する. 返信を使わないためサーボの確認はしない. |
| `bus` | メソッド | 半二重UARTバス(並行送受信用). |
| `mrd_xbus_line` | 関数 | 系統のXBUSサーボクラス. |
| `mrd_xbus_line_ref` | 関数 | 系統の参照を作る. |
| `build_channel_data` | メソッド | チャンネルデータを作る: A4 \| LEN \| 00(key) \| 00(type) \| {ID, 00, 位置H, 位置L} x N \| CRC |
