# Same-frame YOLOE semantic depth

`prototype/sim/semantic_depth.py` loads an open-vocabulary **text-prompt segmentation** checkpoint with classes `door`, `door frame`, `elevator door`, `wall`. It passes the owned `DepthFrame.rgb` as PIL RGB, asks Ultralytics for `retina_masks=True`, and measures only mask member pixels on that exact frame's depth grid and transforms. No simulator geometry identities or target truth enter this module.

Python API: `YOLOESemanticDepth(checkpoint, device).predict(depth_frame)` returns a `SemanticDepthFrame`. `payload()` contains frame-local `S1` IDs, model metadata, confidence, mask bbox, depth statistics, representative measured pixel/base/world point, all-mask plane fit and labeled image packets. `measurement('S1')` returns the existing depth-layer schema with mask-only pixel membership. Cache the result with its `DepthFrame`; serialize calls to the model owner. An instance ID is not a tracking ID. Empty detections remain empty, invalid depth remains unknown, and the plane RMS exposes mixed/nonplanar mask geometry. RGB/mask inspection remains necessary before deciding object identity or doorway geometry.

Mask and layer measurements also expose actual world support bounds. `mask_relations` reports pixel overlap, representative XY distance and observed support gap for each unordered mask pair. These are generic measured relations, not class-specific identity rules. The task chooses the YOLO vocabulary; framework judgments remain generic. Current integration and validation limits: [NAVIGATION_FRAMEWORK.md](../NAVIGATION_FRAMEWORK.md).

## Environment and execution

The task-local `.local/envs/sim` environment contains Ultralytics8.4.152, Torch2.10.0 and Torchvision0.25.0. The official `yoloe-26l-seg.pt` and `mobileclip2_b.ts` assets are in `.local/models/perception`; the Ultralytics CLIP tokenizer is installed. Official [YOLOE documentation](https://docs.ultralytics.com/models/yoloe/) describes these text-prompt segmentation assets; the `-pf.pt` variant does not accept class prompts.

All install and first-run network operations must clear both cases of proxy variables. Example after Main selects `PERCEPTION_PYTHON` and a checkpoint working directory:

```bash
env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u http_proxy -u https_proxy -u all_proxy NO_PROXY='*' no_proxy='*' PIP_CONFIG_FILE=/dev/null PIP_PROXY='' "$PERCEPTION_PYTHON" -m pip install 'ultralytics==8.4.152' 'torchvision==0.25.0' 'https://github.com/ultralytics/CLIP/archive/refs/heads/main.zip'
```

Run from the model cache directory so class setup finds its text encoder. Source the task GPU entrypoint, which currently binds CUDA by physical GPU0 UUID; pass logical `--device cuda:0`:

```bash
source /home/baoquanc/workspace/ForceControl-lightnav/prototype/gpu.sh
cd /home/baoquanc/workspace/ForceControl-lightnav/.local/models/perception
env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u http_proxy -u https_proxy -u all_proxy NO_PROXY='*' no_proxy='*' /home/baoquanc/workspace/ForceControl-lightnav/.local/envs/sim/bin/python /home/baoquanc/workspace/ForceControl-lightnav/prototype/perception/predict_frame.py --frame /absolute/path/to/captured-frame.npz --checkpoint yoloe-26l-seg.pt --device cuda:0 --output /absolute/path/to/new-proof-directory
```

The archive schema is the one already produced by `Prototype.archive_frame`: RGB, optical depth, capture pose, intrinsics, both transforms, frame ID and simulation time. Output includes JSON and inspectable per-instance RGB/mask/depth images. This command proves execution on the archived frame; it does not establish navigation success or detector quality over all views.

Current service entrypoint is `prototype/sim/run.sh`: it sources `prototype/gpu.sh`, selects the allocated physical GPU (currentlyGPU0), sets a task-local YOLO config directory, and runs from `.local/models/perception` so the prompt encoder resolves locally. Segmentation runs on demand for each captured monitor observation, never substitutes labels for raw LightNav RGB, and archives the JSON/overlay beside its RGB-D frame.

The fixed candidate confidence threshold is0.03 with non-E2E NMS. Calibration on3 archived simulator views found scores around0.04–0.10 and real semantic mislabels; threshold0.25 emitted no instances. This exposes tentative candidates with their raw scores, never confirms target identity. Exact boolean masks are archived separately as `semantic_masks.npz`.

The completed Qwen36/Gemini36 comparison, including negative results and API limitations, is in [NAVIGATION_YOLO.md](../NAVIGATION_YOLO.md).
