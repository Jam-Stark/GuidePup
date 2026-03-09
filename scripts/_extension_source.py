"""Helpers for locating the editable Isaac Lab extension source tree."""

from pathlib import Path
import sys


def append_acc_source_dir() -> Path:
    """Append the editable source directory for the ``acc`` package to ``sys.path``."""

    repo_root = Path(__file__).resolve().parent.parent
    candidates = (repo_root / "source" / "LMP", repo_root / "source" / "acc")

    for candidate in candidates:
        if (candidate / "acc").is_dir():
            candidate_str = str(candidate)
            if candidate_str not in sys.path:
                sys.path.append(candidate_str)
            return candidate

    checked_paths = ", ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"Could not locate the acc extension source directory. Checked: {checked_paths}")
