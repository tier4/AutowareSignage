# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

import json
import os

# signage の永続設定を集約する単一 JSON ファイル。
# /opt/autoware 直下は他アプリと共用で所有者/権限が他の都合で決まり、サービスユーザーから
# 書き込めないことがあるため、ecu_system_setup が用意する signage 専用ディレクトリに置く。
SETTINGS_DIR = "/opt/autoware/signage"
SETTINGS_PATH = os.path.join(SETTINGS_DIR, "signage_settings.json")

# 旧配置。新しいファイルが無いときの移行元としてのみ読む (削除はしない)。
LEGACY_SETTINGS_PATH = "/opt/autoware/signage_settings.json"
LEGACY_VOLUME_PATH = "/opt/autoware/volume.txt"
LEGACY_STANDING_MODE_PATH = "/opt/autoware/standing_mode.txt"

# 設定キー
KEY_VOLUME = "volume"
KEY_STANDING_MODE = "standing_mode"


class SettingsStore:
    """signage の永続設定を単一 JSON (/opt/autoware/signage/signage_settings.json) で保持する。

    従来は設定項目ごとに個別ファイル (volume.txt / standing_mode.txt) へ保存し、その後
    /opt/autoware/signage_settings.json へ統合していた。新しいファイルに値が無い項目は
    旧配置 (統合 JSON → 個別ファイルの順) から読み込んで移行する。旧ファイルは削除しない
    (/opt/autoware 直下は書き込めないことがあり、旧バージョンへ戻した場合にも値を残すため)。

    複数のインターフェース (音量 / 立席モード) が同じファイルを読み書きするため、
    ファイルアクセスは本クラスに集約し、書き込み競合を避ける。
    """

    def __init__(self, node):
        self._node = node
        self._data = self._load()
        self._migrate_legacy_files()

    def _load(self):
        data = self._load_json(SETTINGS_PATH)
        if data:
            self._node.get_logger().info("loaded settings from {}".format(SETTINGS_PATH))
        return data

    def _load_json(self, path):
        try:
            if os.path.isfile(path):
                with open(path, "r") as f:
                    data = json.load(f)
                    if isinstance(data, dict):
                        return data
                    self._node.get_logger().error(
                        "not able to load settings {}, ERROR: not a JSON object".format(path)
                    )
        except Exception as e:
            self._node.get_logger().error(
                "not able to load settings {}, ERROR: {}".format(path, str(e))
            )
        return {}

    def _save(self):
        os.makedirs(os.path.dirname(SETTINGS_PATH), exist_ok=True)
        # 一時ファイルへ書いてから置き換える (書き込み途中の電源断で壊れたファイルを残さないため)。
        tmp_path = SETTINGS_PATH + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(self._data, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, SETTINGS_PATH)

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
        legacy_settings = self._load_json(LEGACY_SETTINGS_PATH)
        for key, legacy_file_path, read_legacy_file in (
            (KEY_VOLUME, LEGACY_VOLUME_PATH, self._read_legacy_volume),
            (KEY_STANDING_MODE, LEGACY_STANDING_MODE_PATH, self._read_legacy_standing_mode),
        ):
            if key in self._data:
                continue
            source = LEGACY_SETTINGS_PATH
            value = legacy_settings.get(key)
            if value is None:
                source = legacy_file_path
                value = read_legacy_file()
            if value is not None:
                # どの旧配置から引き継いだかを後から追えるよう警告で残す。
                self._node.get_logger().warning("migrated settings {} from {}".format(key, source))
                self._data[key] = value
                migrated = True

        if migrated:
            try:
                self._save()
            except Exception as e:
                # 保存に失敗しても移行値はメモリ上で使い、次回起動で再試行する。
                self._node.get_logger().error(
                    "not able to migrate legacy settings, ERROR: {}".format(str(e))
                )

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
