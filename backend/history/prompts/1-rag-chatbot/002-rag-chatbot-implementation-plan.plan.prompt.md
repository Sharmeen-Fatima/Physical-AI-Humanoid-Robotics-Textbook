---
id: 002
title: RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-rag-chatbot
branch: 1-rag-chatbot
user: Sharmeen Fatima
command: /sp.plan
labels: ["plan", "architecture", "rag", "design-decisions"]
links:
  spec: specs/1-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-rag-chatbot/plan.md
  - specs/1-rag-chatbot/research.md
  - specs/1-rag-chatbot/data-model.md
  - specs/1-rag-chatbot/quickstart.md
  - specs/1-rag-chatbot/contracts/rag-api.yaml
tests:
  - none
---

## Prompt

/sp.plan
# Plan – High-Level Roadmap

1. Sitemap discovery and URL extraction
2. Web scraping and HTML cleaning
3. Text normalization and preprocessing
4. Chunking with overlap
5. Embedding generation
6. Qdrant collection setup
7. Vector storage with metadata
8. Similarity-based retrieval
9. RAG prompt integration
10. Chatbot interface completion
11. Testing and validation
12. Logging and monitoring for production
13. Error handling & retries for ingestion
14. Periodic vector refresh/update (for book updates)

## Response snapshot

Created comprehensive implementation plan for RAG Chatbot System including:

**Plan.md** - Complete architectural design:
- Technical Context: Python 3.10+, Cohere (embeddings), Gemini (LLM), Qdrant (vector DB)
- Constitution Check: All 6 principles validated (Security-First, RAG-Grounding, Modular Code, RAG Standards, Ingestion Integrity, Testing)
- Project Structure: Single backend application with modular components (ingestion, embedding, storage, retrieval, generation, chatbot)
- 6 Architectural Decisions documented (Cohere embeddings, Gemini LLM, Qdrant Cloud, CLI-first interface, overlapping chunking, content hashing for idempotency)
- 5 Risks identified with mitigation strategies (API rate limiting, low retrieval relevance, hallucinations, Qdrant outages, content updates)
- Success Metrics defined (100% ingestion success, >80% retrieval relevance, zero hallucinations, <2s latency, 99% uptime)

**Research.md** (Phase 0) - Resolved 10 research tasks:
1. Cohere embed-english-v3.0 (1024 dimensions)
2. Fixed overlapping chunks (800 tokens, 200 overlap)
3. Qdrant HNSW + Cosine similarity (m=16, ef_construct=100)
4. Similarity threshold 0.70 (configurable)
5. Gemini prompt engineering (explicit grounding, temperature=0.2)
6. SHA-256 content hashing for idempotency
7. BeautifulSoup HTML cleaning (remove nav/header/footer)
8. Exponential backoff retry (5 retries, 2x multiplier)
9. Multi-layered testing (unit, integration, manual evaluation)
10. In-memory session management (UUID sessions, 30-min expiration)

**Data-Model.md** (Phase 1) - Defined 8 entities:
- BookPage, TextChunk, VectorEmbedding, ChunkMetadata, Query, RetrievalResult, ChatResponse, Session
- Complete schemas with validation rules and relationships
- Qdrant point schema for vector storage

**Contracts/rag-api.yaml** (Phase 1) - OpenAPI 3.0 specification:
- 5 endpoints: /health, /query, /ingest, /ingest/status, /session
- Request/response schemas for query-answer flow
- Error handling and status codes

**Quickstart.md** (Phase 1) - Developer setup guide:
- Prerequisites (Python 3.10+, API keys for Cohere/Gemini/Qdrant)
- Step-by-step setup (venv, dependencies, .env configuration)
- Ingestion workflow (init Qdrant, run book ingestion)
- CLI usage examples with multi-turn conversations
- Testing instructions and troubleshooting

**Agent Context Updated**: Claude Code context file updated with technology stack choices

## Outcome

- ✅ Impact: Complete architectural design with all technical unknowns resolved, ready for implementation (/sp.tasks)
- 🧪 Tests: Testing strategy defined (unit, integration, manual evaluation); no tests run yet (planning phase)
- 📁 Files: Created 5 planning artifacts (plan.md, research.md, data-model.md, quickstart.md, rag-api.yaml)
- 🔁 Next prompts: /sp.tasks (task breakdown), /sp.adr (document architectural decisions if needed)
- 🧠 Reflection: Comprehensive planning completed with clear decisions on embeddings (Cohere 1024d), chunking (800/200), retrieval (cosine similarity, 0.70 threshold), and RAG grounding (Gemini with explicit prompts). All constitution principles validated. Ready for task generation.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning phase completed successfully with all research tasks resolved
- Graders run and results (PASS/FAIL): Constitution Check - ALL GATES PASS (6/6 principles validated)
- Prompt variant (if applicable): Standard /sp.plan command with high-level roadmap provided by user
- Next experiment (smallest change to try): During implementation, validate that chunking parameters (800/200) produce high-quality retrieval; may need tuning based on empirical testing

## Architectural Decision Records Suggested

📋 Architectural decision detected: **RAG Technology Stack Selection (Cohere + Gemini + Qdrant)** — Multiple significant architectural choices made with long-term impact:
- Embedding model: Cohere embed-english-v3.0 vs OpenAI/SentenceTransformers
- LLM provider: Gemini 1.5 Pro vs GPT-4/Claude
- Vector database: Qdrant Cloud vs Pinecone/Weaviate
- Chunking strategy: Fixed-size overlapping vs semantic
- Idempotency: Content hashing vs URL-based deduplication
- Session management: In-memory vs Redis/database

Document reasoning and tradeoffs? Run `/sp.adr rag-technology-stack-selection`
