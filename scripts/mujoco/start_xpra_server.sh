#!/usr/bin/env bash
set -euo pipefail

XPRA_DISPLAY="${XPRA_DISPLAY:-:100}"
XPRA_BIND_HOST="${XPRA_BIND_HOST:-127.0.0.1}"
XPRA_HTML_PORT="${XPRA_HTML_PORT:-14500}"

if xpra info "${XPRA_DISPLAY}" >/dev/null 2>&1; then
  echo "[INFO] Xpra session already running on ${XPRA_DISPLAY}"
  echo "[INFO] HTML5 endpoint: http://${XPRA_BIND_HOST}:${XPRA_HTML_PORT}/"
  exit 0
fi

xpra start "${XPRA_DISPLAY}" \
  --bind-tcp="${XPRA_BIND_HOST}:${XPRA_HTML_PORT}" \
  --tcp-auth=none \
  --html=on \
  --mdns=no \
  --daemon=yes \
  --exit-with-children=no \
  --pulseaudio=no \
  --notifications=no \
  --printing=no \
  --file-transfer=no \
  --webcam=no

echo "[INFO] Started Xpra session on ${XPRA_DISPLAY}"
echo "[INFO] HTML5 endpoint: http://${XPRA_BIND_HOST}:${XPRA_HTML_PORT}/"
