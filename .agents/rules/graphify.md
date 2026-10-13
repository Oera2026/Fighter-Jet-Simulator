---
trigger: always_on
description: Prefer the graphify knowledge graph when exploring unfamiliar code; refresh it after code changes. Same guidance as AGENTS.md.
---

## Graphify Knowledge Graph

This project keeps a graphify knowledge graph in `graphify-out/` (generated locally, not committed). Follow the graphify section of `AGENTS.md`:

- When exploring code you don't know yet, prefer `graphify query "<keywords>"`, `graphify path "<A>" "<B>"` and `graphify explain "<concept>"` over blind grep searches.
- Skip the graph when you already know which file to open, or when the task isn't about the code. If `graphify` or `graphify-out/graph.json` is missing, carry on without it.
- After a session that changes code, run `graphify update .` once at the end.
