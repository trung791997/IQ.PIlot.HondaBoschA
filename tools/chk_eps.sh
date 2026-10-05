#!/usr/bin/env bash
source "$(dirname "$(readlink -f "$0")")/diag_env.sh"
exec "$PY" "$(dirname "$(readlink -f "$0")")/chk_eps.py"
