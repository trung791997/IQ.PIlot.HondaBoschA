"""StockBrakeFeel is off by default and opts in per device through the IQStockBrakeFeel flag file."""
import pytest

from iqpilot.selfdrive.controls.lib import longitudinal_planner as lp


@pytest.fixture
def params_dir(tmp_path, monkeypatch):
  monkeypatch.setenv("PARAMS_ROOT", str(tmp_path))
  monkeypatch.delenv("OPENPILOT_PREFIX", raising=False)
  d = tmp_path / "d"
  d.mkdir()
  return d


def test_off_by_default(params_dir):
  assert lp.STOCK_BRAKE_FEEL is False
  assert lp.stock_brake_feel_enabled() is False


@pytest.mark.parametrize("value, enabled", [("1", True), ("0", False)])
def test_flag_file(params_dir, value, enabled):
  (params_dir / lp.STOCK_BRAKE_FEEL_PARAM).write_text(value)
  assert lp.stock_brake_feel_enabled() is enabled
