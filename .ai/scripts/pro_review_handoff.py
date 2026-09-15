#!/usr/bin/env python3
"""Verify a Git-published stage release and generate the cloud Pro prompt."""
from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path, PurePosixPath

try:
    import tomllib
except ModuleNotFoundError as exc:  # pragma: no cover
    raise SystemExit("Python 3.11+ is required") from exc

MIB = 1024 * 1024
MAX_ZIP_BYTES = 95 * MIB
WORKER_PREFIX = "worker_delivery__"
PRO_PREFIX = "pro_delivery__"
PRO_DELIVERY_ZIP = "pro_delivery__full_review.zip"
DEFAULT_CONFIG = Path(".ai/artifact-sync.toml")
DEFAULT_PRO_DOC_ROOT = "docs/pro-reviews"
OWNER_PLACEHOLDER = "[OWNER: 请填写诊断问题/阶段验收/事实核查/QA等需求类型]"
REQUEST_PLACEHOLDER = "[OWNER: 请填写本轮具体问题、前置回答要求和验收范围]"


def run(root: Path, *args: str, check: bool = True) -> str:
    completed = subprocess.run(
        ["git", *args], cwd=root, text=True, capture_output=True, check=False
    )
    if check and completed.returncode != 0:
        raise SystemExit((completed.stderr or completed.stdout).strip())
    return completed.stdout.strip()


def git_root(start: Path) -> Path:
    return Path(run(start.expanduser().resolve(), "rev-parse", "--show-toplevel")).resolve()


def normalize_remote(url: str) -> str:
    value = url.strip()
    match = re.fullmatch(r"git@github\.com:([^/]+/[^/]+?)(?:\.git)?", value)
    if match:
        return f"https://github.com/{match.group(1)}"
    if value.startswith("https://github.com/"):
        return value[:-4] if value.endswith(".git") else value
    return value


def verify_git_publish(root: Path, remote: str, branch: str | None) -> tuple[str, str, str]:
    dirty = run(root, "status", "--porcelain", "--untracked-files=no")
    if dirty:
        raise SystemExit("tracked Git changes remain; commit the in-scope handoff changes before cloud review")
    current = branch or run(root, "branch", "--show-current")
    if not current:
        raise SystemExit("detached HEAD; specify a published review branch")
    head = run(root, "rev-parse", "HEAD")
    remote_ref = f"refs/remotes/{remote}/{current}"
    remote_head = run(root, "rev-parse", remote_ref, check=False)
    if not remote_head:
        raise SystemExit(
            f"missing local remote-tracking ref {remote}/{current}; run git push -u {remote} HEAD:{current}"
        )
    if remote_head != head:
        raise SystemExit(
            f"local HEAD {head} is not the verified pushed commit {remote_head}; push and verify before handoff"
        )
    return normalize_remote(run(root, "remote", "get-url", remote)), current, head


def worker_archives(release: Path) -> list[Path]:
    archives = sorted(path for path in release.glob("*.zip") if path.is_file())
    preferred = [path for path in archives if path.name.startswith(WORKER_PREFIX)]
    if preferred:
        return preferred
    return [path for path in archives if not path.name.startswith(PRO_PREFIX)]


def zip_lines(release: Path) -> list[str]:
    archives = worker_archives(release)
    if not archives:
        raise SystemExit(f"no Worker ZIP packages found in {release}")
    lines: list[str] = []
    for path in archives:
        size = path.stat().st_size
        if size > MAX_ZIP_BYTES:
            raise SystemExit(
                f"compressed ZIP exceeds 95 MiB: {path.name} ({size} bytes); split semantically first"
            )
        lines.append(f"  - `{path.name}` ({size} compressed bytes)")
    return lines


def load_pro_doc_root(root: Path, config: Path, explicit: str | None) -> str:
    if explicit and explicit.strip():
        value = explicit.strip()
    else:
        path = config if config.is_absolute() else root / config
        value = DEFAULT_PRO_DOC_ROOT
        if path.is_file():
            with path.open("rb") as handle:
                data = tomllib.load(handle)
            configured = data.get("pro_review", {}).get("local_review_root")
            if isinstance(configured, str) and configured.strip():
                value = configured.strip()
    pure = PurePosixPath(value)
    if pure.is_absolute() or ".." in pure.parts or value in {"", "."}:
        raise SystemExit("Pro review document root must be a non-empty repository-relative path without '..'")
    return pure.as_posix().rstrip("/")


def local_worker_parse_prompt(
    *,
    repo_url: str,
    branch: str,
    commit: str,
    drive_location: str,
    pro_delivery_zip: str,
    pro_doc_destination: str,
) -> str:
    return f"""请解析 Owner 在当前本地 Worker 对话中上传的 Cloud Pro 全量交付包 `{pro_delivery_zip}`：

1. 该 Pro ZIP 由 Owner 从 Cloud Pro 对话转交；不要去 Google Drive 寻找 Pro 交付包。Google Drive `{drive_location}` 只保存本轮 Worker 输入 artifacts。
2. 在当前项目中创建 `{pro_doc_destination}`，把原始 `{pro_delivery_zip}` 保存在该目录，并将 `FULL_REVIEW.md` 和 `LOCAL_WORKER_PARSE_PROMPT.md` 解压到同一目录。确认包内 prompt 与本 prompt 的 source lock 和处理要求一致。
3. 云端审阅 source lock：`{repo_url}` / branch `{branch}` / commit `{commit}`。不要因此 reset、stash、discard 或覆盖本地较新的工作；先比较本地 HEAD、实际 diff 和当前生产环境。
4. 将 Cloud Pro 结论分为：远程代码/Worker 交付包直接支持的事实、推断、未知项、必须由本地 IsaacLab/GPU/log/hardware 验证的事项。云端建议的科学 gate、阈值和资源要求不得自动升级为本地硬门槛。
5. 结合本地 `.ai/PROJECT.md` command registry、当前 memory、resolved config、真实日志和资源状态，审查可执行性；保留有价值的 insights/novelty，修正不适合生产环境的 gate、命令和验收标准。
6. 输出：本地核验结果、采用/修改/拒绝的建议及理由、最小下一步方案、所需命令与证据、仍需 Owner 决定的事项。没有本地证据时明确写 `NOT_RUN` 或 `INCONCLUSIVE`。"""


def render(
    template: str,
    *,
    repo_url: str,
    branch: str,
    commit: str,
    drive_location: str,
    zip_list: list[str],
    review_type: str | None,
    owner_request: str | None,
    pro_delivery_zip: str,
    pro_doc_destination: str,
) -> str:
    worker_prompt = local_worker_parse_prompt(
        repo_url=repo_url,
        branch=branch,
        commit=commit,
        drive_location=drive_location,
        pro_delivery_zip=pro_delivery_zip,
        pro_doc_destination=pro_doc_destination,
    )
    replacements = {
        "{{REPO_URL}}": repo_url,
        "{{BRANCH}}": branch,
        "{{COMMIT_SHA}}": commit,
        "{{DRIVE_LOCATION}}": drive_location,
        "{{ZIP_LIST}}": "\n".join(zip_list),
        "{{WORKER_ZIP_LIST}}": "\n".join(zip_list),
        "{{REVIEW_TYPE}}": review_type.strip() if review_type and review_type.strip() else OWNER_PLACEHOLDER,
        "{{OWNER_REQUEST}}": owner_request.strip() if owner_request and owner_request.strip() else REQUEST_PLACEHOLDER,
        "{{PRO_DELIVERY_ZIP}}": pro_delivery_zip,
        "{{PRO_DOC_DESTINATION}}": pro_doc_destination,
        "{{LOCAL_WORKER_PARSE_PROMPT}}": worker_prompt,
    }
    for key, value in replacements.items():
        template = template.replace(key, value)
    return template


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description=__doc__)
    root.add_argument("--repo", type=Path, default=Path.cwd())
    root.add_argument("--release-dir", type=Path, required=True)
    root.add_argument("--drive-location", required=True)
    root.add_argument("--review-type")
    root.add_argument("--owner-request")
    root.add_argument("--remote", default="origin")
    root.add_argument("--branch")
    root.add_argument("--template", type=Path)
    root.add_argument("--output", type=Path)
    root.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    root.add_argument("--pro-doc-root")
    root.add_argument("--pro-delivery-zip", default=PRO_DELIVERY_ZIP)
    return root


def main() -> int:
    args = parser().parse_args()
    root = git_root(args.repo)
    release = args.release_dir.expanduser().resolve()
    if not release.is_dir():
        raise SystemExit(f"release directory not found: {release}")
    if Path(args.pro_delivery_zip).name != args.pro_delivery_zip or not args.pro_delivery_zip.startswith(PRO_PREFIX) or not args.pro_delivery_zip.endswith('.zip'):
        raise SystemExit("--pro-delivery-zip must be a simple pro_delivery__*.zip filename")
    repo_url, branch, commit = verify_git_publish(root, args.remote, args.branch)
    local_root = load_pro_doc_root(root, args.config, args.pro_doc_root)
    pro_doc_destination = f"{local_root}/{release.name}/{commit[:12]}"
    template_path = args.template
    if template_path is None:
        installed = root / ".ai/PRO_REVIEW_PROMPT.md"
        if installed.is_file():
            template_path = installed
        else:
            template_path = Path(__file__).resolve().parents[1] / "templates/PRO_REVIEW_PROMPT.md"
    if not template_path.is_file():
        raise SystemExit(f"prompt template not found: {template_path}")
    output = args.output.expanduser().resolve() if args.output else release / "PRO_REVIEW_PROMPT.md"
    output.write_text(
        render(
            template_path.read_text(encoding="utf-8"),
            repo_url=repo_url,
            branch=branch,
            commit=commit,
            drive_location=args.drive_location,
            zip_list=zip_lines(release),
            review_type=args.review_type,
            owner_request=args.owner_request,
            pro_delivery_zip=args.pro_delivery_zip,
            pro_doc_destination=pro_doc_destination,
        ),
        encoding="utf-8",
    )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
