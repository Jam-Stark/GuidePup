#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
mkdir -p "${HF_HOME}" "${TMPDIR}" "${LIGHTNAV_MODEL}"
exec "${LIGHTNAV_VENV}/bin/hf" download LightOriginsHQ/LightNav-0 \
  --local-dir "${LIGHTNAV_MODEL}"
