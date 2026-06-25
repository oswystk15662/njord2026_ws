Use the overnight autonomous workflow for `njord2026_ws`.

Read:
- AGENTS.md
- TASK_QUEUE.md
- HANDOFF.md
- CODEX_GLOSSARY.md

Work on the first unchecked task in TASK_QUEUE.md.

Rules:
- Use at most 3 subagents.
- Only one subagent may edit files.
- Research and review subagents are read-only.
- Do not use recursive subagents.
- Do not run hardware launch files.
- Do not use sudo.
- Do not push.
- Do not modify secrets or credentials.
- Prefer package-level build/test.
- If the same command fails twice, stop and update HANDOFF.md.

Before stopping:
- Update HANDOFF.md.
- Summarize changed files.
- List verified commands.
- List exact next steps.