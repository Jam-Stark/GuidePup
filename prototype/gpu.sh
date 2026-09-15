#!/usr/bin/env bash
# Select a physical GPU on the current machine; this task defaults to GPU0.
export LIGHTNAV_GPU="${LIGHTNAV_GPU:-0}"
export CUDA_VISIBLE_DEVICES="$(nvidia-smi -i "$LIGHTNAV_GPU" --query-gpu=uuid --format=csv,noheader)"
export MUJOCO_GL=egl
# server.py matches the EGL UUID in its own process before importing MuJoCo.
unset MUJOCO_EGL_DEVICE_ID
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
