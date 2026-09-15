<!-- managed-by: jam-coding-role -->
# Explicit artifact and cloud Pro handoff

Artifact handoff is an optional stage-delivery capability. Run it only when the Owner requests a bundle/cloud review or a named scientific stage explicitly defines artifact delivery. It is not part of ordinary task closure.

## Cloud Pro handoff order

An Owner request to send the current stage to a cloud Pro reviewer authorizes the **in-scope handoff commit and push** unless the Owner explicitly says otherwise. Main must still exclude unrelated changes and respect branch protection.

Before generating the cloud prompt:

1. inspect the Git diff and commit only the stage's Git-visible source/config/document changes;
2. push the configured current/review branch;
3. verify the remote-tracking branch resolves to the same commit as local `HEAD`;
4. record repository URL、branch and full commit SHA;
5. pack and upload the selected Worker artifacts to the Drive task folder;
6. generate `PRO_REVIEW_PROMPT.md` naming that folder and the exact Worker ZIP files;
7. Owner submits the prompt to cloud Pro;
8. cloud Pro returns a concise response and attaches `pro_delivery__full_review.zip` in the Pro conversation;
9. Owner uploads that ZIP in the local Worker conversation;
10. local Worker stores and extracts it under the project's configured Pro review document root.

The cloud Pro does **not** upload its answer ZIP to Google Drive. If commit/push is not authorized, branch policy blocks it, or the remote commit cannot be verified, stop and report the handoff as blocked. Do not give the cloud reviewer a prompt that claims access to unpublished code.

## Selection boundary

Use a positive allowlist over relevant untracked/ignored outputs. Scan sensitive names and small text content, reject symlinks, enforce file/bundle limits, and require explicit checkpoint opt-in. Never package the entire untracked tree.

## Compressed-ZIP cloud limit and semantic splitting

The default limit applies to the **final compressed size of each generated `.zip` file**, not to the raw source files inside it. Every cloud-facing Worker ZIP and the Pro full-review ZIP must be at most **95 MiB (99,614,720 bytes)**.

When a generated Worker ZIP exceeds the compressed-size limit:

1. create ordinary, independently readable ZIP archives grouped by meaning;
2. split a still-oversized semantic group into further ordinary ZIP files;
3. never create `.z01/.z02/.zip`、`.zip.001` or similar reconstruction-dependent volumes;
4. never binary-slice a checkpoint; exclude nonessential models or use a separately authorized authenticated rclone exception.

## Worker Drive delivery

Use one immutable Drive task folder for Worker inputs:

```text
Pro_Space/<project>/<worktree>/<stage>/<timestamp>__<git-short-sha>/
```

New Worker artifacts use:

```text
worker_delivery__BUNDLE_INDEX.md
worker_delivery__BUNDLE_MANIFEST.json
worker_delivery__PRO_HANDOFF.md
worker_delivery__source_and_configs.zip
worker_delivery__logs_and_metrics.zip
worker_delivery__plots_and_evidence.zip
worker_delivery__checkpoints_part01.zip
```

Historical releases do not need renaming. Create new releases only; do not overwrite, move or delete previous releases.

## Cloud Pro output contract

The cloud Pro gives two synchronized outputs.

### A. Concise Owner response in the conversation

Use this order:

1. any requested preliminary answer;
2. high-value insights and findings;
3. the requested diagnosis、stage acceptance、fact-check or QA result;
4. optional research novelty or overlooked algorithm、engineering or data contribution;
5. the attached `pro_delivery__full_review.zip` filename and a copy-ready prompt for the local Worker AI.

This response is intentionally concise so the Owner can review it quickly.

### B. Full local-Worker delivery

Create a normal standard ZIP named `pro_delivery__full_review.zip`, at most 95 MiB compressed, containing:

- `FULL_REVIEW.md`: detailed insights/findings; detailed diagnosis/acceptance/fact-check/QA with evidence、inference、unknowns and local-only checks separated; optional novelty section;
- `LOCAL_WORKER_PARSE_PROMPT.md`: exactly the same copy-ready Worker prompt shown in concise item 5.

Attach the ZIP in the Pro conversation for the Owner to download. Do not attempt to upload it to Google Drive. If attachment creation is unavailable, provide the two files and a downloadable ZIP if possible, write `NOT_ATTACHED`, and do not invent a Drive URL.

## Owner transfer and local Worker placement

The Owner transfers the Pro ZIP by uploading it in the local Worker conversation. The Worker must:

1. treat the conversation attachment as the authoritative Pro package and not search Drive for it;
2. read the `Pro review document root` from `.ai/PROJECT.md`;
3. preserve the original ZIP and extract its two files into a new stage/release/commit-specific subdirectory under that root;
4. compare the Pro source lock with local HEAD/diff before adopting any recommendation;
5. keep cloud gates advisory until local production evidence supports them.

If the project has not configured a Pro review document root, ask the Owner before writing rather than choosing an unrelated directory.

## Role boundary

The cloud Pro must think independently and research cautiously, but cannot inspect unbundled local logs、resolved runtime、IsaacLab/GPU/process state or hardware. Cloud assumptions must not become over-strict local scientific gates or release blockers. The local AI retains reasonable authority over production feasibility、commands、resources and admission thresholds.

## Authorization and upload

Packing and Worker-artifact Drive upload require an explicit Owner request or named stage-closure trigger. The public-write Drive permission is standing authorization for create-only Worker stage artifacts, not authorization to upload arbitrary files, credentials or an unscreened worktree.

Choose Worker upload capability in this order: connected Google Drive upload action; reliable browser/computer-use; authenticated rclone; otherwise produce the release locally and report `NOT_UPLOADED`.

A public editor link does not become anonymous Drive API credentials. Record an upload receipt only after the destination is verified.
