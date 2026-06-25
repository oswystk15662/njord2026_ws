---
name: handoff-writer
description: Write or update HANDOFF.md before stopping autonomous work, switching tasks, or handing work to another Codex session. Use to compress state into short structured YAML with changed files, verified commands, failures, facts, next steps, and avoid-list.
---

# handoff-writer

Use this skill before stopping autonomous work or switching tasks.

## Required file

Update `HANDOFF.md`.

## Schema

```yaml
task:
status:
changed:
verified:
failed:
facts:
next:
avoid:
```

## Rules

- Keep it short.
- Prefer concrete filenames.
- Prefer exact commands.
- Include blockers.
- Include next command.
- Do not write long prose.