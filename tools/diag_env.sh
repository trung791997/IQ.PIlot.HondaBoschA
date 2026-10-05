#!/usr/bin/env bash
# Shared env for the on-device diagnostics: reuse the running manager's interpreter/env if found,
# otherwise rebuild the environment exactly the way launch_chffrplus.sh does for a prebuilt tree.
DIR=$(readlink -f /data/openpilot 2>/dev/null || echo /data/openpilot)  # /data/openpilot may be a symlink to /data/iqpilot
PY=""
for pid in $(pgrep -f "manager.py"); do
  if [ "$(readlink -f /proc/$pid/cwd)" = "$DIR/iqpilot/system/manager" ]; then
    PY=$(readlink /proc/$pid/exe)
    while IFS= read -r -d '' kv; do
      case "$kv" in PYTHONPATH=*|PYTHONSAFEPATH=*|IQPILOT_*=*|AGNOS_VERSION=*|QCOM_*=*) export "$kv";; esac
    done < /proc/$pid/environ
    echo "env: from running manager (pid $pid)"
    break
  fi
done
if [ -z "$PY" ]; then
  echo "env: manager not running, rebuilding launcher env"
  export HOME="${HOME:-/home/comma}"
  export IQPILOT_PROPRIETARY_ROOT="$DIR/artifacts"
  [ -f "$DIR/launch_env.sh" ] && source "$DIR/launch_env.sh"
  export IQPILOT_SOURCE_ROOT="$DIR/iqpilot"
  export PYTHONSAFEPATH=1
  PP="$DIR/artifacts/package_runtime::$DIR"
  [ -d /usr/libexec/iqpilot/python ] && PP="/usr/libexec/iqpilot/python:$PP"
  export PYTHONPATH="$PP"
  PY=/usr/local/venv/bin/python3
  [ -x "$PY" ] || PY=$(command -v python3)
fi
cd "$DIR" || exit 1
