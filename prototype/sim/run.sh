#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source "$ROOT/prototype/gpu.sh"
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY
export NO_PROXY='*' no_proxy='*'
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export YOLO_CONFIG_DIR="$ROOT/.local/cache/perception/ultralytics"
# Ultralytics resolves the prompt encoder by filename in its working directory.
cd "$ROOT/.local/models/perception"
exec "$ROOT/.local/envs/sim/bin/python" "$ROOT/prototype/sim/server.py" "$@"
