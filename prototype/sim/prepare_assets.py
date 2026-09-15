"""Copy the selected old A2 artifacts; never read the current handle MJCF."""
import argparse
import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--source', type=Path, default=ROOT.parent / 'ForceControl')
args = parser.parse_args()
run = args.source / 'logs/rsl_rl/uniFP_aliengo_position_force/2026-05-23_14-25-38_A2_base_only_v2'
dest = ROOT / '.local/assets/a2-old'
dest.mkdir(parents=True, exist_ok=True)
shutil.copy2(args.source / 'resources/a2_description/source/a2_original.xml', dest / 'a2.xml')
shutil.copytree(args.source / 'resources/a2_description/meshes', dest / 'meshes', dirs_exist_ok=True)
for name in ('policy.pt', 'policy.json'):
    shutil.copy2(run / 'mujoco_exported' / name, dest / name)
(dest / 'provenance.json').write_text(json.dumps({
    'branch_base': '4605004 (A2_v2, 2026-05-25)',
    'training_run': str(run),
    'checkpoint': 'model_59999.pt',
    'export': str(run / 'mujoco_exported/policy.pt'),
    'robot_xml': str(args.source / 'resources/a2_description/source/a2_original.xml'),
    'note': 'The May 23 training run had a dirty v2 config; May 25 is the committed v2 base, not a claim of an exact clean training revision.',
}, indent=2) + '\n')
print(dest)
