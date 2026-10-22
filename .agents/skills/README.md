# Agent skills

Each directory here is an [Agent Skill](https://agentskills.io): a `SKILL.md` plus
whatever files it uses. Everything under this folder is committed, scripts included
(`.gitignore` re-includes it).

Claude Code only looks in `.claude/skills/`. Give each skill a symlink there:

```bash
ln -s ../../.agents/skills/<name> .claude/skills/<name>
```
