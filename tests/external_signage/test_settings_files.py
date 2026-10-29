# tests/external_signage/test_settings_files.py
import json
import os
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import yaml

from external_signage import display_mode
from external_signage import external_signage_core as core
from external_signage.external_signage_core import ExternalSignage


@pytest.fixture
def paths(tmp_path, monkeypatch):
    # 設定ファイルの配置を tmp 配下へ差し替える
    settings_dir = tmp_path / "signage"
    p = SimpleNamespace(
        dir=settings_dir,
        settings=settings_dir / "external_signage_settings.json",
        legacy_settings=tmp_path / "home" / "settings.json",
        mapping=settings_dir / "destination_mapping.yaml",
        share=tmp_path / "share",
    )
    monkeypatch.setattr(core, "SETTINGS_DIR", str(p.dir))
    monkeypatch.setattr(core, "SETTINGS_PATH", str(p.settings))
    monkeypatch.setattr(core, "LEGACY_SETTINGS_PATH", str(p.legacy_settings))
    monkeypatch.setattr(core, "DESTINATION_MAPPING_PATH", str(p.mapping))
    monkeypatch.setattr(core, "get_package_share_directory", lambda _pkg: str(p.share))
    return p


@pytest.fixture
def signage():
    # コンストラクタはシリアルポートに触れるため通さず、必要な属性だけ用意する
    s = ExternalSignage.__new__(ExternalSignage)
    s.node = MagicMock()
    s._settings = {"in_experiment": True, "airport": False, "display_mode": display_mode.AUTONOMOUS}
    s.display_signage = MagicMock()
    s._render_destination = MagicMock()
    s._restore_existing_display = MagicMock()
    return s


def _write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


class TestLoadSettings:
    def test_new_file_is_preferred(self, signage, paths):
        _write_json(paths.settings, {"airport": True})
        _write_json(paths.legacy_settings, {"airport": False})
        assert signage._load_settings() == {"airport": True}
        signage.node.get_logger().warning.assert_not_called()

    def test_legacy_file_is_used_when_new_file_is_missing(self, signage, paths):
        _write_json(paths.legacy_settings, {"airport": True})
        assert signage._load_settings() == {"airport": True}
        signage.node.get_logger().warning.assert_called_once()

    @pytest.mark.parametrize("content", ["{broken", "[1, 2]"])
    def test_invalid_new_file_is_reported(self, signage, paths, content):
        paths.dir.mkdir(parents=True)
        paths.settings.write_text(content)
        _write_json(paths.legacy_settings, {"airport": True})
        assert signage._load_settings() == {"airport": True}
        signage.node.get_logger().error.assert_called_once()

    def test_default_is_used_when_no_file_exists(self, signage, paths):
        assert signage._load_settings() == {"in_experiment": True, "airport": False}
        signage.node.get_logger().warning.assert_called_once()


class TestSaveSettings:
    def test_creates_directory_and_leaves_no_temp_file(self, signage, paths):
        signage._save_settings()
        assert json.loads(paths.settings.read_text()) == signage._settings
        assert os.listdir(paths.dir) == [paths.settings.name]


class TestApplyDisplayMode:
    def test_failed_save_keeps_display_and_mode(self, signage):
        signage._save_settings = MagicMock(side_effect=OSError("read-only"))
        response = signage.set_display_enable(SimpleNamespace(data=False), SimpleNamespace())
        assert response.success is False
        assert signage._settings["display_mode"] == display_mode.AUTONOMOUS
        signage.display_signage.assert_not_called()

    def test_successful_save_updates_display(self, signage):
        signage._save_settings = MagicMock()
        response = signage.set_display_enable(SimpleNamespace(data=False), SimpleNamespace())
        assert response.success is True
        assert signage._settings["display_mode"] == display_mode.OFF
        signage.display_signage.assert_called_once_with("null")


@pytest.mark.skipif(not hasattr(yaml, "safe_load"), reason="PyYAML が無い環境")
class TestDestinationMapping:
    def _write_template(self, paths, mapping):
        template = paths.share / "config" / "destination_mapping.yaml"
        template.parent.mkdir(parents=True)
        template.write_text(yaml.safe_dump({"destination_mapping": mapping}))

    def test_seeds_from_template(self, signage, paths):
        self._write_template(paths, {91737: "miraikan"})
        assert signage._load_destination_mapping() == {"91737": "miraikan"}
        assert paths.mapping.is_file()

    def test_existing_file_is_not_overwritten(self, signage, paths):
        self._write_template(paths, {91737: "miraikan"})
        paths.dir.mkdir(parents=True)
        paths.mapping.write_text(yaml.safe_dump({"destination_mapping": {"1": "edited"}}))
        assert signage._load_destination_mapping() == {"1": "edited"}

    def test_template_is_read_directly_when_seed_fails(self, signage, paths):
        self._write_template(paths, {91737: "miraikan"})
        paths.dir.write_text("")  # ディレクトリを作れない状態にする
        assert signage._load_destination_mapping() == {"91737": "miraikan"}
