---
id: 003
title: Setting up RAG-Specific Skills and Agents
stage: general
date_iso: 2025-12-30
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-textbook-foundation
user: rehan363
command: none
labels: ["rag", "phase2", "intelligence", "setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - .claude/skills/rag-technical-standards.md
  - .claude/agents/rag-developer.md
---

# Prompt
i think that we shoud move to phase 2 give me suggession

# Response
I recommended starting Phase 2 (RAG Integration) by building out the specialized intelligence required for the task.

## Actions Taken:
1. Created `.claude/skills/rag-technical-standards.md`: Defines the technical stack (FastAPI, Qdrant, Neon) and architectural rules for RAG.
2. Created `.claude/agents/rag-developer.md`: A specialized backend engineer persona for RAG development.

## Next Steps:
Run /sp.specify for the RAG Integration feature to begin the SDD workflow.
