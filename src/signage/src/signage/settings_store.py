# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

import json
import os

# signage の永続設定を集約する単一 JSON ファイル
SETTINGS_PATH = "/opt/autoware/signage_settings.json"

# 旧: 設定項目ごとの個別ファイル。JSON へ統合したため移行にのみ使用する。
LEGACY_VOLUME_PATH = "/opt/autoware/volume.txt"
LEGACY_STANDING_MODE_PATH = "/opt/autoware/standing_mode.txt"

# 設定キー
KEY_VOLUME = "volume"
KEY_STANDING_MODE = "standing_mode"


class SettingsStore:
    """signage の永続設定を単一 JSON (/opt/autoware/signage_settings.json) で保持する。

    従来は設定項目ごとに個別ファイル (volume.txt / standing_mode.txt) へ保存していたが、
    ファイルが増えて管理しづらくなるため 1 ファイルへ統合した。初回起動時に旧ファイルが
    残っていれば読み込んで JSON へ移行し、移行できた旧ファイルは削除する。

    複数のインターフェース (音量 / 立席モード) が同じファイルを読み書きするため、
    ファイルアクセスは本クラスに集約し、書き込み競合を避ける。
    """

    def __init__(self, node):
        self._node = node
        self._data = self._load()
        self._migrate_legacy_files()

    def _load(self):
        try:
            if os.path.isfile(SETTINGS_PATH):
                with open(SETTINGS_PATH, "r") as f:
                    data = json.load(f)
                    if isinstance(data, dict):
                        return data
        except Exception as e:
            self._node.get_logger().error(
                "not able to load settings, ERROR: {}".format(str(e))
            )
        return {}

    def _save(self):
        os.makedirs(os.path.dirname(SETTINGS_PATH), exist_ok=True)
        with open(SETTINGS_PATH, "w") as f:
            json.dump(self._data, f, indent=2)

    def get(self, key, default=None):
        return self._data.get(key, default)

    def set(self, key, value):
        # 永続化に成功した場合のみメモリ上の値を確定させる。失敗時は旧値へロールバックし、
        # 例外を呼び出し元へ伝播する (呼び出し側で失敗を通知できるようにするため)。
        sentinel = object()
        previous = self._data.get(key, sentinel)
        self._data[key] = value
        try:
            self._save()
        except Exception:
            if previous is sentinel:
                self._data.pop(key, None)
            else:
                self._data[key] = previous
            raise

    def _migrate_legacy_files(self):
        migrated = False
        if KEY_VOLUME not in self._data:
            value = self._read_legacy_volume()
            if value is not None:
                self._data[KEY_VOLUME] = value
                migrated = True
        if KEY_STANDING_MODE not in self._data:
            value = self._read_legacy_standing_mode()
            if value is not None:
                self._data[KEY_STANDING_MODE] = value
                migrated = True

        if migrated:
            try:
                self._save()
            except Exception as e:
                # 移行値の保存に失敗した場合は旧ファイルを残し、次回起動で再試行する。
                self._node.get_logger().error(
                    "not able to migrate legacy settings, ERROR: {}".format(str(e))
                )
                return

        # 移行済み (= JSON 側に値がある) の旧ファイルは削除する。
        self._remove_legacy_files()

    def _read_legacy_volume(self):
        try:
            if os.path.isfile(LEGACY_VOLUME_PATH):
                with open(LEGACY_VOLUME_PATH, "r") as f:
                    value = f.readline().strip()
                    if value != "":
                        return float(value)
        except Exception as e:
            self._node.get_logger().error(
                "not able to read legacy volume, ERROR: {}".format(str(e))
            )
        return None

    def _read_legacy_standing_mode(self):
        try:
            if os.path.isfile(LEGACY_STANDING_MODE_PATH):
                with open(LEGACY_STANDING_MODE_PATH, "r") as f:
                    value = f.readline().strip().lower()
                    if value in ["true", "false"]:
                        return value == "true"
        except Exception as e:
            self._node.get_logger().error(
                "not able to read legacy standing mode, ERROR: {}".format(str(e))
            )
        return None

    def _remove_legacy_files(self):
        for path, key in (
            (LEGACY_VOLUME_PATH, KEY_VOLUME),
            (LEGACY_STANDING_MODE_PATH, KEY_STANDING_MODE),
        ):
            # 値が JSON へ取り込めている旧ファイルのみ削除する。
            if key not in self._data:
                continue
            try:
                if os.path.isfile(path):
                    os.remove(path)
            except Exception as e:
                self._node.get_logger().warning(
                    "not able to remove legacy file {}, ERROR: {}".format(path, str(e))
                )
