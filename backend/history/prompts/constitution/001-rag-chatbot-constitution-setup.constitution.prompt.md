---
id: 001
title: RAG Chatbot Constitution Setup
stage: constitution
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-rag-chatbot
user: unknown
command: /sp.constitution
labels: ["constitution", "rag", "setup", "project-initialization"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution# Constitution – RAG Chatbot for Physical AI & Humanoid Robotics Book

## Rules
- All API keys and secrets must be loaded via environment variables.
- No hard-coded credentials are allowed in code or prompts.
- The chatbot must only answer using retrieved context (RAG-based).
- Hallucinations are strictly prohibited.
- If relevant context is missing, the system must respond with "Information not found in the book."
- Code must be modular, readable, and production-ready.

## Goals
- Build a complete Retrieval-Augmented Generation (RAG) chatbot.
- Ingest book data from:
  - Sitemap: https://physical-ai-humanoid-robotics-textb-pied.vercel.app/sitemap.xml
  - Book URL: https://physical-ai-humanoid-robotics-textb-pied.vercel.app/
- Convert book content into semantic chunks.
- Generate embeddings and store them in Qdrant Vector Database.
- Enable accurate, grounded question-answering over the book.

## Standards
- Primary language: Python
- Follow industry-standard RAG architecture.
- Use overlapping chunking for semantic continuity.
- Store metadata with each chunk (URL, section, chunk index).
- Ensure scalable and cloud-compatible design.

## Response snapshot

Created comprehensive project constitution for RAG Chatbot system with six core principles:

1. **Security-First Development**: Environment-based secrets management, no hard-coded credentials
2. **RAG-Based Grounding (NON-NEGOTIABLE)**: Strict retrieval-only responses, hallucination prevention
3. **Modular & Production-Ready Code**: Python 3.10+, type hints, structured logging
4. **RAG Architecture Standards**: Overlapping chunking, metadata tracking, Qdrant vector store
5. **Data Ingestion Integrity**: Clean HTML parsing, idempotent ingestion, versioning
6. **Testing & Quality Assurance**: Unit, integration, and edge case testing

Technology stack specified: beautifulsoup4, requests, qdrant-client, openai, python-dotenv, pydantic

Development workflow documented: spec → plan → tasks → implementation → testing → documentation

Governance model established with quality gates and continuous improvement through PHR/ADR documentation.

## Outcome

- ✅ Impact: Established foundational principles and standards for entire RAG chatbot project
- 🧪 Tests: No tests at this stage (constitution definition)
- 📁 Files: Created/updated .specify/memory/constitution.md
- 🔁 Next prompts: /sp.specify (create feature spec), /sp.plan (architecture planning), implementation of data ingestion pipeline
- 🧠 Reflection: Constitution provides clear security, quality, and architectural guardrails. RAG-grounding principle is critical for preventing hallucinations. Ready for feature specification phase.

## Evaluation notes (flywheel)

- Failure modes observed: None - constitution creation successful
- Graders run and results (PASS/FAIL): N/A (meta-level artifact)
- Prompt variant (if applicable): Direct constitution command
- Next experiment (smallest change to try): Validate constitution against actual implementation constraints during spec/plan phase
