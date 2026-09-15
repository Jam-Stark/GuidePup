#!/usr/bin/env bash
set -euo pipefail
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
export NO_PROXY='*' no_proxy='*'

PI_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_DATA_ROOT="$(cd "$PI_ROOT/../.." && pwd)/.local"
NODE_VERSION="22.23.2"
NODE_PREFIX="$TASK_DATA_ROOT/envs/node-v$NODE_VERSION-linux-x64"
PI_ENV="$TASK_DATA_ROOT/envs/pi"

mkdir -p "$TASK_DATA_ROOT/envs" "$TASK_DATA_ROOT/cache/npm" "$PI_ENV"
if [[ ! -x "$NODE_PREFIX/bin/node" ]]; then
  curl --noproxy '*' --proxy '' --fail --location --silent --show-error \
    "https://nodejs.org/dist/v$NODE_VERSION/node-v$NODE_VERSION-linux-x64.tar.xz" \
    | tar -xJ -C "$TASK_DATA_ROOT/envs"
fi
cp "$PI_ROOT/package.json" "$PI_ROOT/package-lock.json" "$PI_ENV/"
PATH="$NODE_PREFIX/bin:$PATH" npm ci --prefix "$PI_ENV" \
  --cache "$TASK_DATA_ROOT/cache/npm" --no-audit --no-fund \
  --proxy='' --https-proxy='' --noproxy='*'
ln -sfn "$PI_ENV/node_modules" "$PI_ROOT/node_modules"
if [[ ! -f "$PI_ROOT/.env" ]]; then
  cp "$PI_ROOT/.env.example" "$PI_ROOT/.env"
fi
PATH="$NODE_PREFIX/bin:$PATH" npm --prefix "$PI_ROOT" run build
"$NODE_PREFIX/bin/node" "$PI_ROOT/dist/cli.js" model
