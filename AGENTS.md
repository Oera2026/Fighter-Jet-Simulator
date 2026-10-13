# Agent Guidelines & Rules

## Graphify Knowledge Graph

This project keeps an AST-based knowledge graph in `graphify-out/`. It is generated locally and not committed.

When you are exploring code you don't know yet, prefer the graph over blind file searches:

1. **Find relevant code**: `graphify query "<question or task keywords>"` (or `query_graph` if MCP is enabled).
2. **Trace dependencies**: `graphify path "<nodeA>" "<nodeB>"` (or `shortest_path`) for call graphs and couplings.
3. **Inspect a symbol**: `graphify explain "<concept>"` (or `get_node`) for a class, struct or function and its direct connections.
4. **Wiki**: if `graphify-out/wiki/index.md` exists, it is a navigable overview of the codebase.

Skip the graph when you already know which file to open, or when the task isn't about the code (docs, config, git, agent settings). If `graphify` isn't installed or `graphify-out/graph.json` doesn't exist, carry on without it.

**Keeping it current**: after a session that changes code, run `graphify update .` once at the end (AST-only, fast, no API cost).
