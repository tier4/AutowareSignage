# 行先表示 td5 ファイル配置ディレクトリ (SYS-HMI-03)

車外サイネージの行先表示に使う td5 ファイルをここに配置する。

## 命名規約

```
{prefix}_{width}x{height}.td5
```

- `prefix` … `config/destination_mapping.yaml` で `destination.point_id` に対応付けた prefix
- サイズ … front / back = `128x16`、side = `80x24` の両方を用意する

例) prefix `busstop2` の場合:

```
busstop2_128x16.td5
busstop2_80x24.td5
```

## destination.point_id との紐付け

active_schedule (FMS) の move タスク `destination.point_id` (整数) と prefix の対応は
外部ファイル **`/opt/autoware/destination_mapping.yaml`** に記載する
（`signage_settings.json` と同じ場所。リビルド不要で編集反映）。同ファイルが無い場合は
パッケージ同梱テンプレート `config/destination_mapping.yaml` が起動時に自動コピーされる。

## 補足

- td5 ファイルはディスプレイベンダ提供ツールで事前作成する（実行時に動的生成しない）。
- 未登録 id・td5 未配置・ロード失敗時は空白（`null` td5）へ自動フォールバックする。
- 有効な行先が取れない状態（スケジュール無し/未受信/未登録/全完了）は回送中
  `resource/td5_file/kaiso_{width}x{height}.td5` を表示する（未配置時は `null` へフォールバック）。
- 行先表示モードでは走行中の自動運行中（`auto`）表示は行わない。

## 【暫定 2026-07-27】テスト用 td5（128x16 のみ）

行先表示テスト用に `miraikan` / `shinagawa` / `teleport`（行先）と `kaiso`（回送, `td5_file/kaiso_128x16.td5`）
を用意している。ただし **128x16（front/back）のみで 80x24（side）版は未用意**。
このため side は空白（`null`）表示となる（`_display_state` は「td5 を持つディスプレイだけ表示」する）。
side 用の `*_80x24.td5` を配置すれば、コード変更なしで side にも自動表示される。
