#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
mkdir -p "${LIGHTNAV_LOCAL}/vendor" "${LIGHTNAV_LOCAL}/envs" \
  "${LIGHTNAV_MODEL}" "${TMPDIR}" "${PIP_CACHE_DIR}" "${HF_HOME}"

if [[ ! -d "${LIGHTNAV_VENDOR}" ]]; then
  GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_GLOBAL=/dev/null \
    git -c http.proxy= clone --depth 1 \
    https://github.com/lightorigins/LightNav-0.git "${LIGHTNAV_VENDOR}"
fi
if [[ ! -x "${LIGHTNAV_VENV}/bin/python" ]]; then
  "${LIGHTNAV_PYTHON:-python3.11}" -m venv "${LIGHTNAV_VENV}"
fi
"${LIGHTNAV_VENV}/bin/python" -m pip install \
  --index-url https://download.pytorch.org/whl/cu128 \
  "torch==2.10.0" "torchvision==0.25.0"
"${LIGHTNAV_VENV}/bin/python" -m pip install \
  -r "${LIGHTNAV_PROJECT_ROOT}/prototype/lightnav/requirements.txt" \
  -e "${LIGHTNAV_VENDOR}[video]"
"${LIGHTNAV_VENV}/bin/hf" download LightOriginsHQ/LightNav-0 \
  --local-dir "${LIGHTNAV_MODEL}"
