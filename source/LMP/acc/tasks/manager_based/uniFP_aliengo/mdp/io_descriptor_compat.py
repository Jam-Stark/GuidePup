from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import ModuleType


def _load_official_source_module() -> ModuleType:
    """Load IsaacLab IO descriptor helpers from the official source checkout.

    Some installed IsaacLab builds used in this repository do not expose
    `isaaclab.envs.utils.io_descriptors`, while the official source tree under the
    sibling workspace does. This fallback keeps the implementation aligned with the
    official source instead of reimplementing the protocol locally.
    """

    repo_root = Path(__file__).resolve().parents[7]
    module_path = (
        repo_root.parent / "IsaacLab" / "source" / "isaaclab" / "isaaclab" / "envs" / "utils" / "io_descriptors.py"
    )
    if not module_path.is_file():
        raise ModuleNotFoundError(
            "Unable to locate IsaacLab official io_descriptors.py at "
            f"'{module_path}'."
        )

    spec = importlib.util.spec_from_file_location("isaaclab_official_io_descriptors", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Failed to build import spec for '{module_path}'.")

    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


try:
    from isaaclab.envs.utils.io_descriptors import (  # type: ignore[attr-defined]
        GenericActionIODescriptor,
        generic_io_descriptor,
        record_dtype,
        record_shape,
    )
except ModuleNotFoundError:
    _official_module = _load_official_source_module()
    GenericActionIODescriptor = _official_module.GenericActionIODescriptor
    generic_io_descriptor = _official_module.generic_io_descriptor
    record_dtype = _official_module.record_dtype
    record_shape = _official_module.record_shape


__all__ = [
    "GenericActionIODescriptor",
    "generic_io_descriptor",
    "record_dtype",
    "record_shape",
]
