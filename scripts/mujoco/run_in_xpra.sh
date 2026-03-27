#!/usr/bin/env bash
set -euo pipefail

if [[ $# -eq 0 ]]; then
  echo "Usage: $0 <command> [args ...]" >&2
  exit 64
fi

XPRA_DISPLAY="${XPRA_DISPLAY:-:100}"
XPRA_XAUTHORITY="${XPRA_XAUTHORITY:-$HOME/.Xauthority}"
MUJOCO_ENV_NAME="${MUJOCO_ENV_NAME:-mujoco}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if ! xpra info "${XPRA_DISPLAY}" >/dev/null 2>&1; then
  echo "[ERROR] Xpra session ${XPRA_DISPLAY} is not running." >&2
  echo "[ERROR] Start it first with: bash scripts/mujoco/start_xpra_server.sh" >&2
  exit 1
fi

CONDA_BIN="$(command -v conda || true)"
if [[ -z "${CONDA_BIN}" && -x "/home/baoquanc/anaconda3/bin/conda" ]]; then
  CONDA_BIN="/home/baoquanc/anaconda3/bin/conda"
fi

if [[ -z "${CONDA_BIN}" ]]; then
  echo "[ERROR] Could not find a conda executable in PATH." >&2
  exit 1
fi

cd "${REPO_ROOT}"
export DISPLAY="${XPRA_DISPLAY}"
export XAUTHORITY="${XPRA_XAUTHORITY}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-llvmpipe}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

exec "${CONDA_BIN}" run -n "${MUJOCO_ENV_NAME}" "$@"
