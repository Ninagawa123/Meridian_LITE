
# 開発環境ノウハウ

## 個人情報の保護のための対策

* 問題: Wifiのパスワードなどの個人情報を含むファイルは、リポジトリに含めないようにする必要がありますが、**Key.h**ファイルに記述対応しています。

**Key.hファイルを管理対象から外すため、<font color="Red">下記のコマンドを実行してください。</font>**

<details open>
<summary>gitの管理対象から外すコマンド</summary>

```bash
git update-index --skip-worktree    ./Meridian_LITE_on_Arduino/src/keys.h
git update-index --skip-worktree    ./Meridian_LITE_for_ESP32/src/keys.h
```

</details>

<details>
<summary>gitの管理対象に戻したい場合のコマンド</summary>

```bash
git update-index --no-skip-worktree ./Meridian_LITE_on_Arduino/src/keys.h
git update-index --no-skip-worktree ./Meridian_LITE_for_ESP32/src/keys.h
```

</details>

## 開発環境の構築

### Meridian_console

<details open>
<summary>WindowsのFirewallの設定</summary>

* Windowsで"Meridian_console"を使用する場合、Firewallの設定が必要です。
  1. Windowsの検索バーに「ファイヤーウォールとネットワーク保護」と入力し、ファイアウォールの設定を開きます。
  2. 下側にあるメニューから「詳細設定」をクリックします。
  3. 「受信の規則」
     1. 「受信の規則」をクリックし、右側の「新しい規則」をクリックします。
     2. 「ポート」を選択し、「次へ」をクリックします。
     3. 「UDP」を選択し、「特定のローカルポート」に「22222」を入力し、「次へ」をクリックします。
     4. 「接続の許可する」を選択し、「次へ」をクリックします。
     5. 「ドメイン」「プライベート」「パブリック」の全てにチェックを入れ、「次へ」をクリックします。
     6. 「名前」に「Meridian_22222」を入力し、「完了」をクリックします。
  4. 「送信の規則」
     1. 「送信の規則」をクリックし、右側の「新しい規則」をクリックします。
     2. 「ポート」を選択し、「次へ」をクリックします。
     3. 「UDP」を選択し、「特定のローカルポート」に「22224」を入力し、「次へ」をクリックします。
     4. 「接続の許可する」を選択し、「次へ」をクリックします。
     5. 「ドメイン」「プライベート」「パブリック」の全てにチェックを入れ、「次へ」をクリックします。
     6. 「名前」に「Meridian_22224」を入力し、「完了」をクリックします。

</details>
