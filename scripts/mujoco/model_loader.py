from __future__ import annotations

import os
import re
from pathlib import Path


def _relative_posix(path: Path, start: Path) -> str:
    return os.path.relpath(path, start).replace("\\", "/")


def _asset_keys(file_path: Path, model_path: Path, *, urdf_mode: bool) -> set[str]:
    if urdf_mode:
        keys = {file_path.name}
    else:
        keys = {_relative_posix(file_path, model_path.parent)}
    return {key for key in keys if key not in {".", ""}}


def _strip_xml_comments(xml_text: str) -> str:
    return re.sub(r"<!--.*?-->", "", xml_text, flags=re.DOTALL)


def _referenced_files(xml_text: str) -> set[str]:
    text = _strip_xml_comments(xml_text)
    matches = re.findall(r'(?:file|filename)="([^"]+)"', text)
    return {match for match in matches if match}


def build_mujoco_asset_map(model_path: Path, xml_text: str) -> dict[str, bytes]:
    model_path = model_path.resolve()
    resource_root = model_path.parent.parent if model_path.parent.parent.exists() else model_path.parent
    urdf_mode = model_path.suffix.lower() == ".urdf"
    referenced = _referenced_files(xml_text)
    referenced_basenames = {Path(ref).name for ref in referenced}

    assets: dict[str, bytes] = {}
    for file_path in resource_root.rglob("*"):
        if not file_path.is_file() or file_path == model_path:
            continue
        keys = _asset_keys(file_path, model_path=model_path, urdf_mode=urdf_mode)
        if referenced and not any(key in referenced or key in referenced_basenames for key in keys):
            continue
        payload = file_path.read_bytes()
        for key in keys:
            assets.setdefault(key, payload)
    return assets


def load_mujoco_model_from_path(mujoco, model_path: Path):
    model_path = model_path.resolve()
    xml_text = model_path.read_text()
    assets = build_mujoco_asset_map(model_path, xml_text)
    try:
        return mujoco.MjModel.from_xml_string(xml_text, assets=assets)
    except TypeError:
        return mujoco.MjModel.from_xml_string(xml_text, assets)
