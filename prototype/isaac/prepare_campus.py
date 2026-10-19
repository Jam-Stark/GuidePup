"""Reference the supplied MF-R1 campus scene without changing its geometry."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / '.local/assets/campus_mf_r1/多楼层场景包_MF-R1'
OUTPUT = ROOT / '.local/assets/campus_mf_r1/integration/environment.usda'


def prepare(package: Path, output: Path):
    package, output = package.resolve(), output.resolve()
    source = package / '多楼层场景.usd'
    stage = Usd.Stage.Open(str(source))
    if str(stage.GetDefaultPrim().GetPath()) != '/World':
        raise ValueError('MF-R1 must expose /World as its default prim')
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.0 or UsdGeom.GetStageUpAxis(stage) != 'Z':
        raise ValueError('MF-R1 is expected to be metre-scale and Z-up')
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(source))
    if unresolved:
        raise FileNotFoundError(f'Unresolved campus USD dependencies: {unresolved}')
    physics = [p.GetPath() for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    colliders = sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse())
    runtime = json.loads((package / '多楼层场景资源/config/runtime.json').read_text())
    output.parent.mkdir(parents=True, exist_ok=True)
    host = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageMetersPerUnit(host, 1.0)
    UsdGeom.SetStageUpAxis(host, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(host, '/Campus').GetPrim()
    host.SetDefaultPrim(root)
    root.GetReferences().AddReference(os.path.relpath(source, output.parent), '/World')
    for path in physics:
        host.OverridePrim(path.ReplacePrefix('/World', '/Campus')).SetActive(False)
    host.GetRootLayer().Save()
    receipt = {'source_scene': str(source), 'environment': str(output),
        'meters_per_unit': 1.0, 'up_axis': 'Z', 'scale_applied': False,
        'source_layers': len(layers), 'source_assets': len(assets), 'unresolved_dependencies': unresolved,
        'source_collision_prims': colliders, 'disabled_imported_physics_scenes': list(map(str, physics)),
        'physics_owner': 'Existing A2 IsaacLab SimulationContext',
        'geometry_and_materials': 'References unchanged source assets; no mesh, collider, door or texture edits',
        'floor_z_m': runtime['floor_z_m'], 'floor_height_measured': runtime['height_is_measured'],
        'elevator_runtime': 'Provided controller preserved; first-floor approach task uses the native closed initial state',
        'geometry_usage': 'Initialization and offline scoring only; not sent to the navigation models'}
    output.with_suffix('.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(receipt, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, default=PACKAGE)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    prepare(args.package, args.output)
