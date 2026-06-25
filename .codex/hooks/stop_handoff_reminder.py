#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
import sys


def main() -> int:
    try:
        payload = json.load(sys.stdin)
    except Exception:
        payload = {}

    cwd = Path(payload.get("cwd") or ".").resolve()
    handoff = cwd / "HANDOFF.md"

    if not handoff.exists():
        print(json.dumps({
            "continue": true,
            "systemMessage": "HANDOFF.md is missing. Before ending autonomous work, create/update HANDOFF.md."
        }))
        return 0

    text = handoff.read_text(encoding="utf-8", errors="replace")
    if "next:" not in text or "changed:" not in text:
        print(json.dumps({
            "continue": true,
            "systemMessage": "HANDOFF.md exists but looks incomplete. Include changed:, verified:, failed:, facts:, next:, avoid:."
        }))
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())