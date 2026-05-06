使い方（簡潔手順）
- **入手**: リポジトリ内にあるなら ntripcaster に展開されています（サブモジュールを追加済み）。
- **ビルド/インストール**:
  - ソースディレクトリへ移動し、通常は autotools ベースの手順です:
```bash
cd src/driver/gnss/ntripcaster
./configure --prefix=/usr/local/ntripcaster
make
sudo make install
```
  - `--prefix` でインストール先を変更できます（デフォルトは `/usr/local/ntripcaster`）。
- **設定**:
  - インストール先の `conf` ディレクトリにあるサンプル設定をリネームして使います（例: `sourcetable.dat.dist` → `sourcetable.dat`、`ntripcaster.conf.dist` → `ntripcaster.conf`）。
  - `ntripcaster.conf` 内で「サーバ名」をホスト名（IPではなく）で指定します。ポートや同時接続数、アクセス制御などもここで設定します。
  - `sourcetable.dat` に配信するストリームのエントリを追加します（READMEに推奨行あり）。
- **起動**:
  - バイナリディレクトリで実行します:
```bash
/usr/local/ntripcaster/bin/ntripcaster
```
  - ソースビルド直下で動かす場合はバイナリ位置に移動して `./ntripcaster`。
- **注意点**:
  - ライセンスはGPL（README参照）。サポートは限定的です。
  - README にある同時接続数やテスト済みディストリビューションの情報を確認してください。
  - `ntrip` プロトコルや sourcetable の書式に馴染みがない場合、README の関連リンク（IGS / RTCM）を参照すると理解が早いです。
