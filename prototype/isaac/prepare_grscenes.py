"""Reference a shipped GRScenes navigation scene in the host's metre/Z-up world."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom


def prepare(scene: Path, output: Path, meters_per_unit: float, yaw_deg: float):
    scene = scene.resolve()
    if not scene.is_file():
        raise FileNotFoundError(scene)
    layer = Sdf.Layer.FindOrOpen(str(scene))
    if layer.pseudoRoot.GetInfo('upAxis') != 'Z' or layer.defaultPrim != 'Root':
        raise ValueError('This importer requires the inspected GRScenes Z-up /Root layout')
    materials = scene.parent.parent.parent / 'Materials'
    if not materials.is_dir():
        raise FileNotFoundError(materials)
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, '/GRScenes')
    stage.SetDefaultPrim(root.GetPrim())
    reference = UsdGeom.Xform.Define(stage, '/GRScenes/Scene')
    reference.AddRotateZOp().Set(yaw_deg)
    reference.AddScaleOp().Set(Gf.Vec3d(meters_per_unit))
    reference.GetPrim().GetReferences().AddReference(os.path.relpath(scene, output.parent), '/Root')
    # The existing A2 plant owns the single PhysicsScene, dt, gravity and device.
    # Preserve all native colliders/meshes; deactivate only embedded simulation contexts.
    contexts = []
    def visit(spec):
        if spec.typeName == 'PhysicsScene':
            path = spec.path.ReplacePrefix(Sdf.Path('/Root'), reference.GetPath())
            stage.OverridePrim(path).SetActive(False)
            contexts.append(str(spec.path))
        for child in spec.nameChildren:
            visit(child)
    visit(layer.GetPrimAtPath('/Root'))
    stage.GetRootLayer().Save()
    receipt = {'source_scene': str(scene), 'environment': str(output),
               'source_meters_per_unit_authored': layer.pseudoRoot.HasInfo('metersPerUnit'),
               'explicit_scene_scale': meters_per_unit, 'yaw_deg': yaw_deg,
               'axis': 'Z', 'host_meters_per_unit': 1., 'materials_root': str(materials),
               'disabled_embedded_physics_scenes': contexts,
               'physics_owner': 'Existing A2 IsaacLab SimulationContext; native scene colliders unchanged',
               'geometry_usage': 'Initialization and offline scoring only; not online navigation input'}
    output.with_suffix('.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--meters-per-unit', type=float, required=True,
                   help='Explicitly inspected source geometry scale, not inferred from unauthored metadata.')
    p.add_argument('--yaw-deg', type=float, default=0.)
    args = p.parse_args()
    prepare(args.scene, args.output, args.meters_per_unit, args.yaw_deg)
