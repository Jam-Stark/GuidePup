from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_DIR = REPOSITORY_ROOT / "resources/aliengoZ1/mujoco/visual_meshes"


@dataclass(frozen=True)
class MeshConversionSpec:
    name: str
    source: Path
    target_name: str


CONVERSION_SPECS: tuple[MeshConversionSpec, ...] = (
    MeshConversionSpec(
        name="trunk",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/trunk.dae",
        target_name="trunk.obj",
    ),
    MeshConversionSpec(
        name="hip",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/hip.dae",
        target_name="hip.obj",
    ),
    MeshConversionSpec(
        name="thigh",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/thigh.dae",
        target_name="thigh.obj",
    ),
    MeshConversionSpec(
        name="thigh_mirror",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/thigh_mirror.dae",
        target_name="thigh_mirror.obj",
    ),
    MeshConversionSpec(
        name="calf",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/calf.dae",
        target_name="calf.obj",
    ),
    MeshConversionSpec(
        name="z1_Link00",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link00.dae",
        target_name="z1_Link00.obj",
    ),
    MeshConversionSpec(
        name="z1_Link01",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link01.dae",
        target_name="z1_Link01.obj",
    ),
    MeshConversionSpec(
        name="z1_Link02",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link02.dae",
        target_name="z1_Link02.obj",
    ),
    MeshConversionSpec(
        name="z1_Link03",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link03.dae",
        target_name="z1_Link03.obj",
    ),
    MeshConversionSpec(
        name="z1_Link04",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link04.dae",
        target_name="z1_Link04.obj",
    ),
    MeshConversionSpec(
        name="z1_Link05",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link05.dae",
        target_name="z1_Link05.obj",
    ),
    MeshConversionSpec(
        name="z1_Link06",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link06.dae",
        target_name="z1_Link06.obj",
    ),
    MeshConversionSpec(
        name="z1_GripperStator",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_GripperStator.dae",
        target_name="z1_GripperStator.obj",
    ),
    MeshConversionSpec(
        name="z1_GripperMover",
        source=REPOSITORY_ROOT / "resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_GripperMover.dae",
        target_name="z1_GripperMover.obj",
    ),
)


def _find_text(element: ET.Element | None, path: str) -> str | None:
    if element is None:
        return None
    match = element.find(path, COLLADA_NS)
    if match is None or match.text is None:
        return None
    return match.text.strip()


def _parse_float_list(text: str) -> list[float]:
    return [float(token) for token in text.split()]


def _parse_int_list(text: str) -> list[int]:
    return [int(token) for token in text.split()]


def _vector_length(values: tuple[float, float, float]) -> float:
    return math.sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2])


def _normalize(values: tuple[float, float, float]) -> tuple[float, float, float]:
    length = _vector_length(values)
    if length < 1e-12:
        return (0.0, 0.0, 1.0)
    return (values[0] / length, values[1] / length, values[2] / length)


def _convert_up_axis(
    values: tuple[float, float, float],
    up_axis: str,
    *,
    is_direction: bool,
) -> tuple[float, float, float]:
    if up_axis == "Z_UP":
        converted = values
    elif up_axis == "Y_UP":
        converted = (values[0], values[2], -values[1])
    elif up_axis == "X_UP":
        converted = (values[1], -values[0], values[2])
    else:
        raise ValueError(f"Unsupported COLLADA up_axis '{up_axis}'.")
    return _normalize(converted) if is_direction else converted


def _chunk(values: list[float], stride: int) -> list[tuple[float, ...]]:
    return [tuple(values[index : index + stride]) for index in range(0, len(values), stride)]


def _source_stride(source: ET.Element) -> int:
    accessor = source.find("c:technique_common/c:accessor", COLLADA_NS)
    if accessor is None:
        raise ValueError(f"Missing accessor for source '{source.get('id', '<unknown>')}'.")
    return int(accessor.attrib.get("stride", "1"))


def _source_floats(source: ET.Element) -> list[tuple[float, ...]]:
    float_array = source.find("c:float_array", COLLADA_NS)
    if float_array is None or float_array.text is None:
        raise ValueError(f"Missing float_array for source '{source.get('id', '<unknown>')}'.")
    stride = _source_stride(source)
    return _chunk(_parse_float_list(float_array.text), stride)


def _geometry_sources(mesh_element: ET.Element) -> dict[str, list[tuple[float, ...]]]:
    sources: dict[str, list[tuple[float, ...]]] = {}
    for source in mesh_element.findall("c:source", COLLADA_NS):
        source_id = source.attrib.get("id")
        if not source_id:
            continue
        sources[f"#{source_id}"] = _source_floats(source)
    return sources


def _vertices_bindings(mesh_element: ET.Element) -> dict[str, str]:
    bindings: dict[str, str] = {}
    for vertices in mesh_element.findall("c:vertices", COLLADA_NS):
        vertices_id = vertices.attrib.get("id")
        if not vertices_id:
            continue
        position_input = vertices.find("c:input[@semantic='POSITION']", COLLADA_NS)
        if position_input is None:
            raise ValueError(f"Vertices block '{vertices_id}' does not reference POSITION.")
        bindings[f"#{vertices_id}"] = position_input.attrib["source"]
    return bindings


def _triangulate_polylist(vcount: list[int], flat_indices: list[int], index_stride: int) -> list[list[int]]:
    triangles: list[list[int]] = []
    cursor = 0
    for polygon_vertices in vcount:
        polygon = flat_indices[cursor : cursor + polygon_vertices * index_stride]
        cursor += polygon_vertices * index_stride
        if polygon_vertices < 3:
            continue
        for triangle_index in range(1, polygon_vertices - 1):
            start = polygon[0:index_stride]
            middle = polygon[triangle_index * index_stride : (triangle_index + 1) * index_stride]
            end = polygon[(triangle_index + 1) * index_stride : (triangle_index + 2) * index_stride]
            triangles.append(start + middle + end)
    if cursor != len(flat_indices):
        raise ValueError("polylist index buffer length does not match vcount metadata.")
    return triangles


def _primitive_inputs(primitive: ET.Element, vertices_map: dict[str, str]) -> tuple[dict[int, tuple[str, str]], int]:
    offset_to_input: dict[int, tuple[str, str]] = {}
    max_offset = -1
    for input_element in primitive.findall("c:input", COLLADA_NS):
        semantic = input_element.attrib["semantic"]
        source = input_element.attrib["source"]
        offset = int(input_element.attrib.get("offset", "0"))
        if semantic == "VERTEX":
            source = vertices_map[source]
            semantic = "POSITION"
        offset_to_input[offset] = (semantic, source)
        max_offset = max(max_offset, offset)
    if max_offset < 0:
        raise ValueError("Primitive has no <input> definitions.")
    return offset_to_input, max_offset + 1


def _primitive_records(
    primitive: ET.Element,
    vertices_map: dict[str, str],
) -> tuple[list[list[int]], dict[int, tuple[str, str]], int]:
    offset_to_input, index_stride = _primitive_inputs(primitive, vertices_map)
    raw_indices = _find_text(primitive, "c:p")
    if raw_indices is None:
        raise ValueError("Primitive is missing index buffer <p>.")
    indices = _parse_int_list(raw_indices)
    if primitive.tag.endswith("triangles"):
        if len(indices) % (3 * index_stride) != 0:
            raise ValueError("Triangle index buffer length is not divisible by 3 * stride.")
        triangles = [indices[index : index + 3 * index_stride] for index in range(0, len(indices), 3 * index_stride)]
    elif primitive.tag.endswith("polylist"):
        vcount_text = _find_text(primitive, "c:vcount")
        if vcount_text is None:
            raise ValueError("polylist is missing <vcount>.")
        triangles = _triangulate_polylist(_parse_int_list(vcount_text), indices, index_stride)
    else:
        raise ValueError(f"Unsupported primitive type '{primitive.tag}'.")
    return triangles, offset_to_input, index_stride


def convert_dae_to_obj(source_path: Path, output_path: Path) -> dict[str, object]:
    tree = ET.parse(source_path)
    root = tree.getroot()

    up_axis = _find_text(root, "c:asset/c:up_axis") or "Z_UP"
    unit_meter = float(root.find("c:asset/c:unit", COLLADA_NS).attrib.get("meter", "1.0") if root.find("c:asset/c:unit", COLLADA_NS) is not None else 1.0)

    positions_out: list[tuple[float, float, float]] = []
    normals_out: list[tuple[float, float, float]] = []
    texcoords_out: list[tuple[float, float]] = []
    faces_out: list[tuple[tuple[int, int | None, int | None], tuple[int, int | None, int | None], tuple[int, int | None, int | None]]] = []

    position_offset = 0
    normal_offset = 0
    texcoord_offset = 0

    for geometry in root.findall("c:library_geometries/c:geometry", COLLADA_NS):
        mesh_element = geometry.find("c:mesh", COLLADA_NS)
        if mesh_element is None:
            continue

        sources = _geometry_sources(mesh_element)
        vertices_map = _vertices_bindings(mesh_element)

        primitive_records: list[tuple[list[list[int]], dict[int, tuple[str, str]], int]] = []
        position_source_names: set[str] = set()
        normal_source_names: set[str] = set()
        texcoord_source_names: set[str] = set()

        for primitive in list(mesh_element.findall("c:triangles", COLLADA_NS)) + list(mesh_element.findall("c:polylist", COLLADA_NS)):
            triangles, offset_to_input, index_stride = _primitive_records(primitive, vertices_map)
            primitive_records.append((triangles, offset_to_input, index_stride))
            for semantic, source_name in offset_to_input.values():
                if semantic == "POSITION":
                    position_source_names.add(source_name)
                elif semantic == "NORMAL":
                    normal_source_names.add(source_name)
                elif semantic == "TEXCOORD":
                    texcoord_source_names.add(source_name)

        local_position_count = 0
        local_normal_count = 0
        local_texcoord_count = 0

        source_position_offsets: dict[str, int] = {}
        source_normal_offsets: dict[str, int] = {}
        source_texcoord_offsets: dict[str, int] = {}

        for source in mesh_element.findall("c:source", COLLADA_NS):
            source_id = source.attrib.get("id")
            if not source_id:
                continue
            source_name = f"#{source_id}"
            source_values = sources[source_name]
            if not source_values:
                continue

            stride = len(source_values[0])
            if source_name in position_source_names:
                if stride < 3:
                    raise ValueError(f"Position source '{source_name}' in '{source_path}' has stride {stride}, expected >= 3.")
                converted_positions = [
                    _convert_up_axis((value[0] * unit_meter, value[1] * unit_meter, value[2] * unit_meter), up_axis, is_direction=False)
                    for value in source_values
                ]
                source_position_offsets[source_name] = position_offset + local_position_count
                positions_out.extend(converted_positions)
                local_position_count += len(converted_positions)
            elif source_name in texcoord_source_names:
                if stride < 2:
                    raise ValueError(f"Texcoord source '{source_name}' in '{source_path}' has stride {stride}, expected >= 2.")
                source_texcoord_offsets[source_name] = texcoord_offset + local_texcoord_count
                texcoords_out.extend((value[0], value[1]) for value in source_values)
                local_texcoord_count += len(source_values)

        for source in mesh_element.findall("c:source", COLLADA_NS):
            source_id = source.attrib.get("id")
            if not source_id:
                continue
            key = f"#{source_id}"
            if key not in normal_source_names:
                continue
            source_values = sources[key]
            stride = len(source_values[0])
            if stride < 3:
                raise ValueError(f"Normal source '{key}' in '{source_path}' has stride {stride}, expected >= 3.")
            converted_normals = [
                _convert_up_axis((value[0], value[1], value[2]), up_axis, is_direction=True)
                for value in source_values
            ]
            source_normal_offsets[key] = normal_offset + local_normal_count
            normals_out.extend(converted_normals)
            local_normal_count += len(converted_normals)

        for triangles, offset_to_input, index_stride in primitive_records:
            for triangle in triangles:
                obj_vertices: list[tuple[int, int | None, int | None]] = []
                for corner_index in range(3):
                    record = triangle[corner_index * index_stride : (corner_index + 1) * index_stride]
                    vertex_position_index: int | None = None
                    vertex_normal_index: int | None = None
                    vertex_texcoord_index: int | None = None
                    for offset, source_binding in offset_to_input.items():
                        semantic, source_name = source_binding
                        source_index = record[offset]
                        if semantic == "POSITION":
                            vertex_position_index = source_position_offsets[source_name] + source_index + 1
                        elif semantic == "NORMAL" and source_name in source_normal_offsets:
                            vertex_normal_index = source_normal_offsets[source_name] + source_index + 1
                        elif semantic == "TEXCOORD" and source_name in source_texcoord_offsets:
                            vertex_texcoord_index = source_texcoord_offsets[source_name] + source_index + 1
                    if vertex_position_index is None:
                        raise ValueError(f"Primitive in '{source_path}' does not resolve a POSITION index.")
                    obj_vertices.append((vertex_position_index, vertex_texcoord_index, vertex_normal_index))
                faces_out.append((obj_vertices[0], obj_vertices[1], obj_vertices[2]))

        position_offset += local_position_count
        normal_offset += local_normal_count
        texcoord_offset += local_texcoord_count

    if not positions_out or not faces_out:
        raise ValueError(f"No renderable mesh geometry found in '{source_path}'.")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as file:
        file.write(f"# Converted from {source_path.relative_to(REPOSITORY_ROOT)}\n")
        file.write(f"# unit_meter={unit_meter} up_axis={up_axis}\n")
        for position in positions_out:
            file.write(f"v {position[0]:.9g} {position[1]:.9g} {position[2]:.9g}\n")
        for texcoord in texcoords_out:
            file.write(f"vt {texcoord[0]:.9g} {texcoord[1]:.9g}\n")
        for normal in normals_out:
            file.write(f"vn {normal[0]:.9g} {normal[1]:.9g} {normal[2]:.9g}\n")
        for face in faces_out:
            face_tokens: list[str] = []
            for position_index, texcoord_index, normal_index in face:
                if texcoord_index is not None and normal_index is not None:
                    face_tokens.append(f"{position_index}/{texcoord_index}/{normal_index}")
                elif texcoord_index is not None:
                    face_tokens.append(f"{position_index}/{texcoord_index}")
                elif normal_index is not None:
                    face_tokens.append(f"{position_index}//{normal_index}")
                else:
                    face_tokens.append(str(position_index))
            file.write(f"f {' '.join(face_tokens)}\n")

    xs = [position[0] for position in positions_out]
    ys = [position[1] for position in positions_out]
    zs = [position[2] for position in positions_out]
    return {
        "source": str(source_path.relative_to(REPOSITORY_ROOT)),
        "target": str(output_path.relative_to(REPOSITORY_ROOT)),
        "unit_meter": unit_meter,
        "up_axis": up_axis,
        "vertex_count": len(positions_out),
        "normal_count": len(normals_out),
        "texcoord_count": len(texcoords_out),
        "face_count": len(faces_out),
        "bbox_min": [min(xs), min(ys), min(zs)],
        "bbox_max": [max(xs), max(ys), max(zs)],
        "extent": [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Convert approved Aliengo/Z1 DAE visual meshes to MuJoCo-compatible OBJ assets.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory where converted OBJ assets and manifest JSON will be written.",
    )
    parser.add_argument(
        "--only",
        nargs="*",
        default=None,
        help="Optional subset of conversion spec names to process.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing OBJ outputs if they already exist.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    selected_names = set(args.only) if args.only else None
    specs = tuple(spec for spec in CONVERSION_SPECS if selected_names is None or spec.name in selected_names)
    if not specs:
        parser.error("No conversion specs selected.")

    manifest: dict[str, dict[str, object]] = {}
    for spec in specs:
        output_path = args.output_dir / spec.target_name
        if output_path.exists() and not args.overwrite:
            raise FileExistsError(
                f"Refusing to overwrite existing asset '{output_path}'. Use --overwrite to regenerate it."
            )
        metadata = convert_dae_to_obj(spec.source, output_path)
        manifest[spec.name] = metadata
        print(
            f"Converted {spec.name}: {metadata['vertex_count']} vertices, {metadata['face_count']} faces, "
            f"extent={metadata['extent']}"
        )

    manifest_path = args.output_dir / "visual_mesh_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Wrote manifest: {manifest_path.relative_to(REPOSITORY_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
