#!/usr/bin/env bash
set -euo pipefail

XPRA_DISPLAY="${XPRA_DISPLAY:-:100}"
XPRA_BIND_HOST="${XPRA_BIND_HOST:-127.0.0.1}"
XPRA_HTML_PORT="${XPRA_HTML_PORT:-14500}"
XPRA_START_TIMEOUT_S="${XPRA_START_TIMEOUT_S:-15}"

DISPLAY_NUMBER="${XPRA_DISPLAY#:}"
XPRA_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
XPRA_LOG_PATH="${XPRA_RUNTIME_DIR}/xpra/${XPRA_DISPLAY}.log"
X_LOCK_PATH="/tmp/.X${DISPLAY_NUMBER}-lock"
X_SOCKET_PATH="/tmp/.X11-unix/X${DISPLAY_NUMBER}"

show_log_tail() {
  if [ -f "${XPRA_LOG_PATH}" ]; then
    echo "[INFO] Xpra log tail: ${XPRA_LOG_PATH}" >&2
    tail -n 40 "${XPRA_LOG_PATH}" >&2 || true
  fi
}

print_recovery_steps() {
  cat >&2 <<EOF
[INFO] Recovery steps for ${XPRA_DISPLAY}:
  1. Inspect the display owner:
     ps -fp \$(tr -d '[:space:]' < "${X_LOCK_PATH}") || true
  2. If it is an orphaned Xvfb-for-Xpra process, stop it:
     kill \$(tr -d '[:space:]' < "${X_LOCK_PATH}") || true
  3. If the process is already gone, remove leftover lock files:
     rm -f "${X_LOCK_PATH}" "${X_SOCKET_PATH}"
  4. Start Xpra again:
     bash scripts/mujoco/start_xpra_server.sh
EOF
}

if xpra info "${XPRA_DISPLAY}" >/dev/null 2>&1; then
  echo "[INFO] Xpra session already running on ${XPRA_DISPLAY}"
  echo "[INFO] HTML5 endpoint: http://${XPRA_BIND_HOST}:${XPRA_HTML_PORT}/"
  exit 0
fi

if [ -f "${X_LOCK_PATH}" ] || [ -S "${X_SOCKET_PATH}" ]; then
  LOCK_PID=""
  LOCK_CMD=""

  if [ -f "${X_LOCK_PATH}" ]; then
    LOCK_PID="$(tr -d '[:space:]' < "${X_LOCK_PATH}" 2>/dev/null || true)"
  fi

  if [ -n "${LOCK_PID}" ] && kill -0 "${LOCK_PID}" >/dev/null 2>&1; then
    LOCK_CMD="$(ps -p "${LOCK_PID}" -o args= 2>/dev/null || true)"
    echo "[ERROR] Display ${XPRA_DISPLAY} is occupied by PID ${LOCK_PID}: ${LOCK_CMD}" >&2
  else
    echo "[ERROR] Display ${XPRA_DISPLAY} has leftover X11 lock/socket state but no live Xpra server." >&2
  fi

  echo "[ERROR] Refusing to print a false success message for ${XPRA_DISPLAY}." >&2
  show_log_tail
  print_recovery_steps
  exit 1
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

for ((attempt = 1; attempt <= XPRA_START_TIMEOUT_S; attempt++)); do
  if xpra info "${XPRA_DISPLAY}" >/dev/null 2>&1; then
    echo "[INFO] Started Xpra session on ${XPRA_DISPLAY}"
    echo "[INFO] HTML5 endpoint: http://${XPRA_BIND_HOST}:${XPRA_HTML_PORT}/"
    exit 0
  fi

  if [ -f "${XPRA_LOG_PATH}" ] && grep -Eqi 'Xvfb command has terminated|xpra initialization error|Fatal server error' "${XPRA_LOG_PATH}"; then
    echo "[ERROR] Xpra failed to become ready on ${XPRA_DISPLAY}." >&2
    show_log_tail
    if [ -f "${X_LOCK_PATH}" ] || [ -S "${X_SOCKET_PATH}" ]; then
      print_recovery_steps
    fi
    exit 1
  fi

  sleep 1
done

echo "[ERROR] Timed out waiting for Xpra to become ready on ${XPRA_DISPLAY}." >&2
show_log_tail
if [ -f "${X_LOCK_PATH}" ] || [ -S "${X_SOCKET_PATH}" ]; then
  print_recovery_steps
fi
exit 1
