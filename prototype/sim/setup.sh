#!/usr/bin/env bash
set -euo pipefail
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
export NO_PROXY='*' no_proxy='*' PIP_PROXY=''
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
SIM_BASE_PYTHON=${SIM_BASE_PYTHON:-python3.11}
export PIP_CACHE_DIR="$ROOT/.local/cache/sim"
if [[ ! -x "$ROOT/.local/envs/sim/bin/python" ]]; then
  "$SIM_BASE_PYTHON" -m venv "$ROOT/.local/envs/sim"
fi
"$ROOT/.local/envs/sim/bin/python" -m pip --proxy '' install \
  --index-url https://download.pytorch.org/whl/cu128 'torch==2.10.0' 'torchvision==0.25.0'
"$ROOT/.local/envs/sim/bin/python" -m pip --proxy '' install -r "$ROOT/prototype/sim/requirements.txt"
if (( $# )); then
  "$ROOT/.local/envs/sim/bin/python" "$ROOT/prototype/sim/prepare_assets.py" "$@"
fi
