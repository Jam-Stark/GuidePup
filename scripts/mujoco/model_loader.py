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


def _compiler_meshdirs(xml_text: str) -> set[str]:
    text = _strip_xml_comments(xml_text)
    matches = re.findall(r"<compiler\b[^>]*\bmeshdir=\"([^\"]+)\"", text)
    return {match.strip("/") for match in matches if match}


def _has_ground_plane(xml_text: str) -> bool:
    text = _strip_xml_comments(xml_text)
    return re.search(r"<geom\b[^>]*\btype=\"plane\"", text) is not None


def _has_world_light(xml_text: str) -> bool:
    text = _strip_xml_comments(xml_text)
    return re.search(r"<light\b", text) is not None


def _has_visual_block(xml_text: str) -> bool:
    text = _strip_xml_comments(xml_text)
    return re.search(r"<visual\b", text) is not None


def _insert_before(pattern: str, replacement: str, xml_text: str) -> str:
    return re.sub(pattern, replacement + r"\g<0>", xml_text, count=1)


def augment_basic_scene(xml_text: str) -> str:
    """Add the A2 runtime's default viewer ground and lighting when missing."""
    augmented = xml_text

    if "ground_grid_mat" not in augmented and not _has_ground_plane(augmented):
        ground_assets = """
    <texture
      name="ground_grid_tex"
      type="2d"
      builtin="checker"
      rgb1="0.22 0.24 0.27"
      rgb2="0.12 0.13 0.15"
      width="512"
      height="512"
      mark="edge"
      markrgb="0.7 0.7 0.7"/>
    <material
      name="ground_grid_mat"
      texture="ground_grid_tex"
      texrepeat="8 8"
      reflectance="0.15"/>
"""
        if re.search(r"</asset>", augmented):
            augmented = _insert_before(r"</asset>", ground_assets, augmented)
        else:
            augmented = _insert_before(r"<worldbody\b", f"  <asset>{ground_assets}  </asset>\n\n", augmented)

    if not _has_visual_block(augmented):
        visual_block = """
  <visual>
    <headlight ambient="0.18 0.18 0.18" diffuse="0.45 0.45 0.45" specular="0.2 0.2 0.2"/>
    <rgba haze="0.15 0.17 0.20 1"/>
  </visual>

"""
        augmented = _insert_before(r"<worldbody\b", visual_block, augmented)

    worldbody_insertions: list[str] = []
    if not _has_world_light(augmented):
        worldbody_insertions.append(
            '    <light name="sun" pos="0 0 4" dir="0.2 0.2 -1" '
            'type="directional" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3" castshadow="true"/>'
        )
        worldbody_insertions.append(
            '    <light name="fill" pos="-3 -2 3" dir="0.5 0.4 -0.8" '
            'type="directional" diffuse="0.35 0.35 0.4" specular="0.1 0.1 0.1" castshadow="false"/>'
        )
    if not _has_ground_plane(augmented):
        worldbody_insertions.append(
            '    <geom name="ground_plane" type="plane" size="25 25 0.1" '
            'material="ground_grid_mat" friction="1.0 1.0 0.005" condim="3"/>'
        )
    if worldbody_insertions:
        augmented = re.sub(
            r"(<worldbody\b[^>]*>)",
            r"\1\n" + "\n".join(worldbody_insertions),
            augmented,
            count=1,
        )

    return augmented


def build_mujoco_asset_map(model_path: Path, xml_text: str) -> dict[str, bytes]:
    model_path = model_path.resolve()
    urdf_mode = model_path.suffix.lower() == ".urdf"
    resource_root = model_path.parent.parent if urdf_mode and model_path.parent.parent.exists() else model_path.parent
    referenced = _referenced_files(xml_text)
    referenced_lookup = set(referenced)
    referenced_lookup.update(Path(ref).name for ref in referenced)
    for meshdir in _compiler_meshdirs(xml_text):
        referenced_lookup.update(str(Path(meshdir) / ref).replace("\\", "/") for ref in referenced)

    assets: dict[str, bytes] = {}
    for file_path in resource_root.rglob("*"):
        if not file_path.is_file() or file_path == model_path:
            continue
        keys = _asset_keys(file_path, model_path=model_path, urdf_mode=urdf_mode)
        if referenced and not any(key in referenced_lookup for key in keys):
            continue
        payload = file_path.read_bytes()
        for key in keys:
            assets.setdefault(key, payload)
    return assets


def load_mujoco_model_from_path(mujoco, model_path: Path):
    model_path = model_path.resolve()
    xml_text = model_path.read_text()
    augmented_xml_text = augment_basic_scene(xml_text) if model_path.suffix.lower() in {".xml", ".mjcf"} else xml_text
    if augmented_xml_text == xml_text and model_path.suffix.lower() in {".xml", ".mjcf", ".urdf"}:
        try:
            return mujoco.MjModel.from_xml_path(str(model_path))
        except Exception:
            pass
    assets = build_mujoco_asset_map(model_path, augmented_xml_text)
    try:
        return mujoco.MjModel.from_xml_string(augmented_xml_text, assets=assets)
    except TypeError:
        return mujoco.MjModel.from_xml_string(augmented_xml_text, assets)
