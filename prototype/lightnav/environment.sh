#!/usr/bin/env bash
# Source from setup/serve scripts; keep packages, weights, and caches on the data disk.
LIGHTNAV_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
LIGHTNAV_LOCAL="${LIGHTNAV_PROJECT_ROOT}/.local"
LIGHTNAV_VENDOR="${LIGHTNAV_LOCAL}/vendor/LightNav-0"
LIGHTNAV_VENV="${LIGHTNAV_LOCAL}/envs/lightnav"
LIGHTNAV_MODEL="${LIGHTNAV_LOCAL}/models/LightNav-0"
LIGHTNAV_CACHE="${LIGHTNAV_LOCAL}/cache/lightnav"
# tmux can retain proxy variables from when its server was first started.
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy PIP_PROXY
export NO_PROXY='*'
export no_proxy='*'
export PIP_CONFIG_FILE=/dev/null
export HF_HOME="${LIGHTNAV_CACHE}/huggingface"
export PIP_CACHE_DIR="${LIGHTNAV_CACHE}/pip"
export XDG_CACHE_HOME="${LIGHTNAV_CACHE}/xdg"
export TORCH_HOME="${LIGHTNAV_CACHE}/torch"
export TMPDIR="${LIGHTNAV_CACHE}/tmp"
export PYTHONUNBUFFERED=1
