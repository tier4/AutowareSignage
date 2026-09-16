# -*- coding: utf-8 -*-
"""車外サイネージの表示モード (OFF / 自動運転状態表示 / 行先表示) の定義と正規化。

表示モードは settings.json の "display_mode" キー 1 本で永続化する。
旧実装は destination_mode (bool) を持っていたが、OFF を足して 3 値になったため
「出処は 1 キー」に揃えた (bool 2 つの組合せだと OFF かつ行先表示という
矛盾した状態が表現できてしまう)。

ROS / シリアル に依存しないため、単体テストから直接 import できる。
"""

OFF = "off"
AUTONOMOUS = "autonomous"
DESTINATION = "destination"

VALID_MODES = (OFF, AUTONOMOUS, DESTINATION)

# 設定ファイルが無い / 値が壊れている場合に使う固定の既定値。
# OFF (消灯) にしてあるのは、どのモードで表示すべきか分からない状態で
# 走行状態や行先を車外へ出してしまうより、何も出さない方が安全なため。
# 表示するかどうかは MOT から明示的に選ばせる。
DEFAULT_MODE = OFF

SETTINGS_KEY = "display_mode"
LEGACY_SETTINGS_KEY = "destination_mode"


def normalize(value):
    """未知の値・型は既定値へ倒す (fail-safe)。"""
    return value if value in VALID_MODES else DEFAULT_MODE


def from_settings(settings):
    """settings から表示モードを取り出す。

    display_mode が無く旧 destination_mode だけがある設定ファイル (既存車両) から
    引き継ぐのは「行先表示を明示的に ON にしていた」場合だけ。それ以外は
    既定値と同じ OFF から始める — 旧 destination_mode=False は「行先表示では
    ない」という意味しか持たず、自動運転状態を車外へ出してよいと明示した
    設定ではないため。
    """
    if SETTINGS_KEY in settings:
        return normalize(settings.get(SETTINGS_KEY))
    if LEGACY_SETTINGS_KEY in settings:
        return DESTINATION if settings.get(LEGACY_SETTINGS_KEY) else OFF
    return DEFAULT_MODE


def migrate(settings):
    """settings を display_mode 1 本の表現へ揃える。

    settings を破壊的に更新し、正規化後のモードを返す。
    """
    mode = from_settings(settings)
    settings.pop(LEGACY_SETTINGS_KEY, None)
    settings[SETTINGS_KEY] = mode
    return mode
