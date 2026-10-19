#!/usr/bin/env bash
# Use this host's bundled OpenUSD without starting SimulationApp or allocating a GPU.
set -euo pipefail
ISAAC_ENV=/home/baoquanc/anaconda3/envs/isaaclab
USD_LIB_ROOT="$ISAAC_ENV/lib/python3.11/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311"
export PYTHONPATH="$USD_LIB_ROOT"
export LD_LIBRARY_PATH="$USD_LIB_ROOT/bin:$ISAAC_ENV/lib"
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
export NO_PROXY='*' no_proxy='*'
exec "$ISAAC_ENV/bin/python" "$@"
