---
name: github-review
description: Review local git changes before commit or PR. Use when checking diffs, PR readiness, robot safety risks, launch path mistakes, missing install rules, secrets, or unnecessary refactors. Do not use for implementation unless explicitly asked.
---

# github-review

Use this skill when reviewing local changes before commit or PR.

## Procedure

1. Check `git status`.
2. Check `git diff --stat`.
3. Review changed files only.
4. Look for:
   - robot safety issues
   - launch path errors
   - missing install rules
   - parameter mistakes
   - unnecessary refactors
   - secrets
5. Suggest small fixes.

## Do not

- push
- force push
- modify files unless asked
- review unrelated files