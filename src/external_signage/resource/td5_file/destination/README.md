# 行先表示 td5 ファイル配置ディレクトリ (SYS-HMI-03)

車外サイネージの行先表示に使う td5 ファイルをここに配置する。

## 命名規約

```
{prefix}_{width}x{height}.td5
```

- `prefix` … `config/destination_mapping.yaml` で `destination.id` に対応付けた prefix
- サイズ … front / back = `128x16`、side = `80x24` の両方を用意する

例) prefix `shibuya` の場合:

```
shibuya_128x16.td5
shibuya_80x24.td5
```

## destination.id との紐付け

`destination.id` と prefix の対応は外部ファイル **`/opt/autoware/destination_mapping.yaml`** に記載する
（`signage_settings.json` と同じ場所。リビルド不要で編集反映）。同ファイルが無い場合は
パッケージ同梱テンプレート `config/destination_mapping.yaml` が起動時に自動コピーされる。

## 補足

- td5 ファイルはディスプレイベンダ提供ツールで事前作成する（実行時に動的生成しない）。
- 未配置・ロード失敗時は空白（`null` td5）へ自動フォールバックする。
- スケジュール完了時の回送中表示は `resource/td5_file/kaiso_{width}x{height}.td5` を使用する
  （こちらも未配置時は `null` へフォールバック）。
