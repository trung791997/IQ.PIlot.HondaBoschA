"""Settings ported from StarPilot that the checked-in params library does not list.

The key list lives in the prebuilt aarch64 common/params_pyx.so, and its C++ source is not in this repo, so a key
can't simply be added. Params().get / put on any of these raises UnknownKeyName. Each one is a plain file in the
same params directory the C++ Params uses (<Paths.params()>/d/<key>), with the same text encoding ("0"/"1" for
bools), so a later library rebuild that adds the key reads the stored value unchanged. When the library does know a
key, Params is used directly.

Reads never raise: a missing file, unreadable file or bad value gives the default below.
"""
import os
import tempfile

from iqpilot.common.params import Params, UnknownKeyName

# key -> default. StarPilot's defaults (common/params_keys.h on ns-bosch-radar-testing) unless noted.
EXTRA_PARAM_DEFAULTS: dict[str, bool | int | float] = {
  # Lateral (modified-EPS Clarity / Civic Bosch only; see latcontrol_honda_eps.py)
  "NrdrLatEpsFirmwareFF": False,    # steer with LatControlHondaEps (James's controller); read when controlsd starts
  "NrdrLatUseFirmwareVgr": False,   # EPS firmware VGR table instead of the road-measured ratio curve
  "NrdrLatAngleRateLimit": 300,     # deg/s ceiling on desired wheel-angle slew; 0 disables
  "NrdrLatVfnOverride": False,      # vfn's driver-override policy in the Honda carcontroller (modified EPS only)
  # Longitudinal
  "StockBrakeFeel": False,          # D-086 stock Honda ACC brake law in the planner
  # Bosch-A radar
  # D-076 range offset -335/128 (-2.617) m instead of -3.0 m; read once at startup. StarPilot's default is off; on here
  # because this port was asked for the -2.617 m offset.
  "BoschARangeOffsetFallback": True,
  "BoschABirthRailRamps": False,       # radard birth-rail ramps
}


def _params_dir() -> str:
  from iqpilot.system.hardware.hw import Paths
  return os.path.join(Paths.params(), os.environ.get("OPENPILOT_PREFIX", "d"))


def _read_raw(key: str) -> str | None:
  try:
    value = Params().get(key)
  except UnknownKeyName:
    try:
      with open(os.path.join(_params_dir(), key), "rb") as f:
        value = f.read()
    except OSError:
      return None
  except Exception:
    return None
  if value is None:
    return None
  if isinstance(value, bytes):
    value = value.decode("utf-8", errors="ignore")
  return str(value).strip()


def _write_raw(key: str, value: str) -> None:
  try:
    Params().put(key, value)
    return
  except UnknownKeyName:
    pass
  directory = _params_dir()
  os.makedirs(directory, exist_ok=True)
  fd, tmp_path = tempfile.mkstemp(prefix=f".tmp_{key}_", dir=directory)
  try:
    with os.fdopen(fd, "w") as f:
      f.write(value)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp_path, os.path.join(directory, key))
  except Exception:
    try:
      os.unlink(tmp_path)
    except OSError:
      pass
    raise


def _default(key: str, default):
  if default is not None:
    return default
  return EXTRA_PARAM_DEFAULTS.get(key)


def get_extra_bool(key: str, default: bool | None = None) -> bool:
  default = bool(_default(key, default))
  raw = _read_raw(key)
  if raw is None or raw == "":
    return default
  if raw.lower() in ("1", "true"):
    return True
  if raw.lower() in ("0", "false"):
    return False
  return default


def get_extra_float(key: str, default: float | None = None) -> float:
  default = float(_default(key, default))
  raw = _read_raw(key)
  try:
    return float(raw) if raw else default
  except ValueError:
    return default


def put_extra_bool(key: str, value: bool) -> None:
  _write_raw(key, "1" if value else "0")


def put_extra_value(key: str, value) -> None:
  _write_raw(key, str(value))
