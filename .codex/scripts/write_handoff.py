#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
HANDOFF = ROOT / "HANDOFF.md"


def run(cmd: list[str]) -> str:
    try:
        return subprocess.check_output(cmd, cwd=ROOT, text=True, stderr=subprocess.STDOUT).strip()
    except Exception as exc:
        return f"unavailable: {exc}"


def main() -> int:
    status = run(["git", "status", "--short"])
    diff_files = run(["git", "diff", "--name-only"])

    text = f"""# HANDOFF.md

    ```yaml
    task: update_this
    status: partial
    changed:
    {format_list(diff_files)}
    verified: []
    failed: []
    facts:
    - git_status: |
    {indent(status or "clean", 4)}
    next:
    - update this handoff with the next concrete step
    avoid:
    - full_repo_scan
    - repeated_full_build
    - hardware_launch_without_approval

    """
    HANDOFF.write_text(text, encoding="utf-8")
    return 0

def format_list(s: str) -> str:
    items = [line.strip() for line in s.splitlines() if line.strip()]
    if not items:
        return " []"
    return "\n".join(f" - {item}" for item in items)

def indent(s: str, spaces: int) -> str:
    pad = " " * spaces
    return "\n".join(pad + line for line in s.splitlines())

if __name__ == "__main__":
    raise SystemExit(main())