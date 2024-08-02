バージョン更新履歴
===

## [Meridian_LITE v1.1.1 (2024.07.25)](https://github.com/Ninagawa123/Meridian_LITE/tree/main)

### 概要

コードをモジュールに分割し, Meridian_TWIN v1.1.0 と同等の構成にした.
命名規則を導入した. 今後, Meridian_TWIN v1.1.0 にも同ルールを適用し1.1.1とする.

### コメント

より改良・拡張しやすくするため, 大規模なリファクタリングを行いました.
命名規則はLLVM準拠とし, 内容を "Meridian_LITE_for_ESP32/src/.clang-format" ファイルにコメントしています.
変数名や関数名のルールもある程度整理しました.
構成要素となるコードをヘッダーファイルで切り分け, モジュール化することで, 改造や拡張の見通しを立ちやすくしました.
またフローチャートもDocumentsにて公開しています.

### ディレクトリ構成

```txt
Meridian_LITE_for_ESP32
│
├── lib
│   └── IcsClass_V210  // KONDOサーボのライブラリ
├── platformio.ini
└── src
    ├── .clang-format  // VSCODEでのコードフォーマット設定ファイル
    ├── config.h       // Meridianの主なconfig設定
    ├── keys.h         // wifiのSSIDやパスワード
    ├── main.cpp       // メインプログラム
    ├── main.h         // メインプログラムのヘッダファイル
    │
    ├── mrd_eeprom.h   // EEPROM関連
    ├── mrd_move.h     // モーション設定
    ├── mrd_msg.h      // メッセージ関連
    ├── mrd_pad.h      // リモコンパッド関連
    ├── mrd_sd.h       // SDメモリ関連
    ├── mrd_servo.h    // サーボ処理
    ├── mrd_wifi.h     // WiFi関連
    ├── mrd_wire0.h    // I2C関連
    └── mrd_module     // モジュールディレクトリ
        ├── mv_firstIK.h    // 簡易IK関連（未定義）
        ├── mv_motionplay.h // モーション再生（未定義）
        ├── sv_dxl2.h       // ダイナミクセル制御（未定義）
        ├── sv_ftbrx.h      // 双葉サーボ制御（未定義）
        ├── sv_ftc.h        // Feetechサーボ制御（未定義）
        └── sv_ics.h        // KONDOサーボ制御
```

ライブラリの関数や変数表など, システムの詳細については以下のサイトがありますが, こちらの情報はまだ古いのでご注意ください.
<https://ninagawa123.github.io/Meridian_info/#>

****************************************

## [Meridian_LITE v1.0.1 (2023.09.15)](https://github.com/realteck-ky/Meridian_LITE/tree/v1.0.1)

### 概要

`# define ESP32_STDALONE 0` をconfig.hに追加し, 値を1に設定することでESP32単体で通信テストが行えるようにした.
その際,　サーボ値は調べず, 代わりにL0番のサーボ値として+-30度のサインカーブを代入しつづける.

### コメント

Meridian公式ライブラリ対応版として大幅なアップデートを行いました.(前回までのバージョンはoldディレクトリにzipで格納しています.)
ソースコードが整理され, 改造の見通しが立ちやすくなりました.

ライブラリの関数や変数表など, システムの詳細については以下をご参照ください.
https://ninagawa123.github.io/Meridian_info/#
