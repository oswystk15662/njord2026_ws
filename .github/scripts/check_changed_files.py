#!/usr/bin/env python3

"""Validate structured files changed relative to a Git base ref."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml


def changed_files(base: str) -> list[Path]:
    result = subprocess.run(
        ["git", "diff", "--name-only", "--diff-filter=ACMR", f"{base}...HEAD"],
        check=True,
        capture_output=True,
        text=True,
    )
    return [Path(line) for line in result.stdout.splitlines() if line]


def validate(path: Path) -> None:
    suffix = path.suffix.lower()
    if suffix == ".py":
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    elif suffix in {".yaml", ".yml"}:
        yaml.safe_load(path.read_text(encoding="utf-8"))
    elif suffix == ".xml":
        ET.parse(path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", required=True)
    args = parser.parse_args()

    failures: list[str] = []
    for path in changed_files(args.base):
        if not path.is_file():
            continue
        try:
            validate(path)
        except Exception as exc:  # noqa: BLE001 - aggregate all validation errors
            failures.append(f"{path}: {exc}")

    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
