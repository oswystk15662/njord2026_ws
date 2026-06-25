# AGENTS.md

This repository is `njord2026_ws`, a ROS 2 workspace for Njord 2026 development.

## Main goals

- Keep the robot software reliable and understandable.
- Prefer small, safe, testable changes.
- Preserve existing robot behavior unless the user explicitly asks for refactoring.
- Avoid wasting Codex credits on broad searches, repeated failed commands, or unnecessary full builds.

## Workspace assumptions

- This is a ROS 2 workspace.
- Prefer package-level operations over whole-workspace operations.
- Do not assume hardware is connected.
- Do not run hardware launch files unless explicitly approved.
- Do not modify credentials, tokens, SSH keys, billing, or global system configuration.

## Cost and scope control

- Do not scan the entire repository unless explicitly necessary.
- Prefer `rg` with specific keywords and directories.
- Before editing, identify the minimum relevant files.
- Do not run full workspace builds repeatedly.
- Prefer:
  - `colcon build --packages-select <pkg>`
  - `colcon test --packages-select <pkg>`
  - targeted Python/C++ tests
- If the same command fails twice, stop retrying and write the cause to `HANDOFF.md`.

## Subagents

Subagents are allowed for autonomous long-running work, especially overnight.

Rules:
- Use at most 3 subagents at a time.
- Only one subagent may edit files.
- Research and review subagents must be read-only.
- Do not create recursive subagents.
- Do not use subagents for small single-file fixes.
- Parent agent must merge results and update `HANDOFF.md`.

Recommended roles:
- research: inspect files and identify likely causes
- implementation: make the smallest useful change
- review: inspect diff and point out risks

## Build and test policy

Allowed without asking:
- `git status`
- `git diff`
- `rg <keyword> <specific_dir>`
- `colcon build --packages-select <pkg>`
- package-specific tests

Ask or avoid:
- full `colcon build`
- full `colcon test`
- `ros2 launch` that may operate hardware
- docker builds
- installing new packages
- network operations

Forbidden unless explicitly approved:
- `sudo`
- `rm -rf`
- `git clean`
- force push
- changing secrets or credentials
- modifying system files outside this workspace

## Overnight autonomous mode

When the user asks to work while they are away:

- Work from `TASK_QUEUE.md`.
- Start with the first unchecked task.
- Use subagents if useful, but follow the subagent rules above.
- Make bounded progress.
- Prefer small patches.
- Run targeted validation.
- Run one broader validation near the end only if reasonable.
- Always update `HANDOFF.md` before stopping.
- If blocked, write the exact blocker and next recommended command.

## Handoff format

Always keep `HANDOFF.md` short and structured.

Use this schema:

```yaml
task:
status:
changed:
verified:
failed:
facts:
next:
avoid: