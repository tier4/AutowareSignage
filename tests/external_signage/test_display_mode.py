# tests/external_signage/test_display_mode.py
from external_signage import display_mode


class TestNormalize:
    def test_valid_modes_pass_through(self):
        for mode in display_mode.VALID_MODES:
            assert display_mode.normalize(mode) == mode

    def test_unknown_value_falls_back_to_default(self):
        assert display_mode.normalize("desutination") == display_mode.DEFAULT_MODE

    def test_wrong_type_falls_back_to_default(self):
        # 設定ファイルを手で編集して bool/None が入っても落ちない
        for value in (True, False, None, 0, []):
            assert display_mode.normalize(value) == display_mode.DEFAULT_MODE


class TestFromSettings:
    def test_display_mode_key_wins(self):
        settings = {"display_mode": display_mode.OFF, "destination_mode": True}
        assert display_mode.from_settings(settings) == display_mode.OFF

    def test_legacy_destination_true_becomes_destination(self):
        assert display_mode.from_settings({"destination_mode": True}) == display_mode.DESTINATION

    def test_legacy_destination_false_becomes_off(self):
        # 旧 destination_mode=False は「行先表示ではない」だけで、自動運転状態を
        # 車外へ出してよいと明示した設定ではないので OFF から始める
        assert display_mode.from_settings({"destination_mode": False}) == display_mode.OFF

    def test_empty_settings_uses_default(self):
        assert display_mode.from_settings({}) == display_mode.DEFAULT_MODE

    def test_default_is_off(self):
        # 設定が無い/壊れている時は消灯する (誤った情報を車外へ出さない)
        assert display_mode.DEFAULT_MODE == display_mode.OFF


class TestMigrate:
    def test_legacy_key_is_removed_and_carried_over(self):
        settings = {"in_experiment": True, "airport": False, "destination_mode": True}
        mode = display_mode.migrate(settings)
        assert mode == display_mode.DESTINATION
        assert settings["display_mode"] == display_mode.DESTINATION
        assert "destination_mode" not in settings

    def test_other_keys_are_preserved(self):
        settings = {"in_experiment": True, "airport": True}
        display_mode.migrate(settings)
        assert settings["in_experiment"] is True
        assert settings["airport"] is True

    def test_broken_value_is_repaired(self):
        settings = {"display_mode": "on"}
        assert display_mode.migrate(settings) == display_mode.DEFAULT_MODE
        assert settings["display_mode"] == display_mode.DEFAULT_MODE

    def test_is_idempotent(self):
        settings = {"destination_mode": True}
        first = display_mode.migrate(settings)
        second = display_mode.migrate(dict(settings))
        assert first == second == display_mode.DESTINATION
