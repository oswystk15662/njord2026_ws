---
name: overnight-dev
description: Autonomous overnight or away-from-keyboard development workflow for njord2026_ws. Use when the user wants Codex to make bounded progress from TASK_QUEUE.md using subagents if helpful, while preserving safety and writing HANDOFF.md before stopping.
---

# overnight-dev

Use this skill when the user wants autonomous development while away.

## Inputs

- `TASK_QUEUE.md`
- `HANDOFF.md`
- `AGENTS.md`
- `CODEX_GLOSSARY.md`

## Procedure

1. Read `AGENTS.md`.
2. Read `TASK_QUEUE.md`.
3. Select the first unchecked task.
4. Decide whether subagents are useful.
5. Use at most 3 subagents:
   - research: read-only
   - implementation: editor
   - review: read-only
6. Make the smallest useful progress.
7. Run targeted validation.
8. Stop on repeated failures.
9. Update `HANDOFF.md`.

## Safety

Never:
- use sudo
- push commits
- change secrets
- run hardware launch
- delete large directories
- modify unrelated packages

## Final output

Include:
- changed files
- validation commands
- failures
- next command