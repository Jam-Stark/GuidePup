"""Bind CUDA and EGL to the allocated physical GPU by their common device UUID."""
from __future__ import annotations

import ctypes
import os
import subprocess
import uuid


def configure_gpu():
    physical_index = int(os.environ.get('LIGHTNAV_GPU', '0'))
    target = subprocess.check_output(
        ['nvidia-smi', '-i', str(physical_index), '--query-gpu=uuid', '--format=csv,noheader'], text=True).strip()
    os.environ['CUDA_VISIBLE_DEVICES'] = target
    os.environ['MUJOCO_GL'] = 'egl'
    os.environ['__EGL_VENDOR_LIBRARY_FILENAMES'] = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
    egl = ctypes.CDLL('libEGL.so.1')
    egl.eglGetProcAddress.restype = ctypes.c_void_p
    egl.eglGetProcAddress.argtypes = [ctypes.c_char_p]
    query_devices = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_int,
        ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_int))(
            egl.eglGetProcAddress(b'eglQueryDevicesEXT'))
    query_uuid = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_void_p, ctypes.c_int,
        ctypes.c_int, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))(
            egl.eglGetProcAddress(b'eglQueryDeviceBinaryEXT'))
    count = ctypes.c_int()
    if not query_devices(0, None, ctypes.byref(count)):
        raise RuntimeError('EGL device enumeration failed')
    devices = (ctypes.c_void_p * count.value)()
    if not query_devices(count.value, devices, ctypes.byref(count)):
        raise RuntimeError('EGL device enumeration failed')
    for index in range(count.value):
        value, size = (ctypes.c_ubyte * 16)(), ctypes.c_int()
        if not query_uuid(devices[index], 0x335C, 16, value, ctypes.byref(size)):
            raise RuntimeError('EGL physical device UUID query failed')
        if 'GPU-' + str(uuid.UUID(bytes=bytes(value))) == target:
            os.environ['MUJOCO_EGL_DEVICE_ID'] = str(index)
            return {'physical_gpu_index': physical_index, 'device_uuid': target, 'egl_device_index': index}
    raise RuntimeError(f'The allocated physical GPU{physical_index} has no matching EGL device')
