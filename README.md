Meridianとは？
===

Meridianはヒューマノイドの制御システムについてのオープンソースプロジェクトです.
ホビーロボットのデジタルツイン化を簡単に実現し, PC上のシミュレーション空間とロボット実機をWiFi経由で1/100秒単位の更新頻度でデータリンクすることができます.

システムの中核はMeridim配列という軽量で汎用的なデータ配列で, ロボットの状態データをコンパクトに格納できます.
このデータ配列がデバイス間を高速に循環することで, リアルタイムな状態データの共有を可能にします.
Meridim配列を中間プロトコルとして既存のシステムの間に挟むことで, 複数社のコマンドサーボやセンサ, Unityなどの開発環境などを自由に繋ぐことができ, またROSの入出力にも対応するため, シミュレーターなど多岐にわたるリソースを活用することができます.

当リポジトリで取り扱う ”Meridian_LITE" はESP32DeckitCを使用するタイプで, 対応ボードはMeridian Board -LITE-となります.
また, より高度な制御が可能となるESP32DeckitC+Teensy4.0を連携させて動作するMeridian_TWIN(対応ボードはMeridian Board Type.K)も開発済みです.

Meridianは今後も用途に応じて様々なハードウェア, ソフトウェアに対応させていく予定です.

[![sync](https://img.youtube.com/vi/4ymSV_Dot-U/0.jpg)](https://www.youtube.com/watch?v=4ymSV_Dot-U)
100Hzデータリンクのデモ動画

[![dance](https://img.youtube.com/vi/Wfc9j4Pmr3E/0.jpg)](https://www.youtube.com/watch?v=Wfc9j4Pmr3E)
100Hzダンスのデモ動画

# コンテンツ

* [ボードについて](./doc/BOARD.md)
* [Getting Started](./doc/GETTING_STARTED.md)
* [QA トラブルシューティング](./doc/QA.md)
* [連携デモ](./doc/DEMO.md)
* [リリースノート](./doc/RELEASE_NOTE.md)
