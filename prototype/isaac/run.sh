#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
export LIGHTNAV_GPU="${LIGHTNAV_GPU:-0}"
export CUDA_VISIBLE_DEVICES="$(nvidia-smi -i "$LIGHTNAV_GPU" --query-gpu=uuid --format=csv,noheader)"
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY
export NO_PROXY='*' no_proxy='*' OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export OMNI_KIT_ACCEPT_EULA=YES
exec /home/baoquanc/anaconda3/envs/isaaclab/bin/python -u "$ROOT/prototype/isaac/worker.py" "$@"
