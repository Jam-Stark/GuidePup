#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
source "${LIGHTNAV_PROJECT_ROOT}/prototype/gpu.sh"
mkdir -p "${TMPDIR}" "${LIGHTNAV_CACHE}/server" "${XDG_CACHE_HOME}/torch/kernels"
export LIGHTNAV_ATTN=sdpa
export PORT="${LIGHTNAV_PORT:-8050}"
exec "${LIGHTNAV_VENV}/bin/lightnav-serve" \
  --task vln --model_path "${LIGHTNAV_MODEL}" --backend hf \
  --device cuda --max_batch_size 1 --host 127.0.0.1 \
  --ready_file "${LIGHTNAV_CACHE}/server/ready" "$@"
