#!/usr/bin/env bash
set -euo pipefail

EXPERIMENT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_ROOT="$(dirname "$EXPERIMENT_ROOT")"
TASK_DATA_ROOT="$(cd "$PI_ROOT/../.." && pwd)/.local"
TASK_NODE="$TASK_DATA_ROOT/envs/node-v22.23.2-linux-x64/bin/node"
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY
export NO_PROXY='*' no_proxy='*'

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  exec "$TASK_NODE" "$EXPERIMENT_ROOT/replay-case.mjs" --help
fi

"$TASK_NODE" "$EXPERIMENT_ROOT/replay-case.mjs" --model-label qwen "$@"
bash "$PI_ROOT/zenmux.sh" gemini --script "$EXPERIMENT_ROOT/replay-case.mjs" --model-label gemini "$@"
bash "$PI_ROOT/zenmux.sh" deepseek --script "$EXPERIMENT_ROOT/replay-case.mjs" --model-label deepseek "$@"
