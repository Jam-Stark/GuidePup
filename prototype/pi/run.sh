#!/usr/bin/env bash
set -euo pipefail

PI_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_DATA_ROOT="$(cd "$PI_ROOT/../.." && pwd)/.local"
# Both the lab model service and simulator are reached directly.
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY
export NO_PROXY='*' no_proxy='*'
exec "$TASK_DATA_ROOT/envs/node-v22.23.2-linux-x64/bin/node" "$PI_ROOT/dist/cli.js" "$@"
