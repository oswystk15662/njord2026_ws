#!/usr/bin/env python3
from __future__ import annotations

import json
import re
import sys


BLOCK_PATTERNS = [
    (r"\bsudo\b", "sudo is blocked by repository policy."),
    (r"\brm\s+-rf\b", "rm -rf is blocked by repository policy."),
    (r"\bgit\s+clean\b", "git clean is blocked by repository policy."),
    (r"\bgit\s+push\b.*\s--force\b", "force push is blocked by repository policy."),
    (r"\bchmod\s+777\b", "chmod 777 is blocked by repository policy."),
]

WARN_PATTERNS = [
    (r"\bcolcon\s+build\s*$", "Prefer colcon build --packages-select <pkg>."),
    (r"\bcolcon\s+test\s*$", "Prefer colcon test --packages-select <pkg>."),
    (r"\bdocker\s+build\b", "Docker builds are expensive; confirm they are necessary."),
    (r"\bgrep\s+-R\b", "Prefer rg with a specific directory."),
    (r"\bfind\s+\.\s+-type\s+f\b", "Avoid broad full-repo file scans."),
    (r"\bros2\s+launch\b", "Ensure this is simulation-only; hardware launch needs approval."),
]


def main() -> int:
    try:
        payload = json.load(sys.stdin)
    except Exception:
        return 0

    tool_input = payload.get("tool_input") or {}
    command = tool_input.get("command") or ""

    if not command:
        return 0

    for pattern, reason in BLOCK_PATTERNS:
        if re.search(pattern, command):
            print(json.dumps({
                "hookSpecificOutput": {
                    "hookEventName": "PreToolUse",
                    "permissionDecision": "deny",
                    "permissionDecisionReason": reason
                }
            }))
            return 0

    warnings = []
    for pattern, message in WARN_PATTERNS:
        if re.search(pattern, command):
            warnings.append(message)

    if warnings:
        print(json.dumps({
            "hookSpecificOutput": {
                "hookEventName": "PreToolUse",
                "additionalContext": "Command policy warning: " + " ".join(warnings)
            }
        }))
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())