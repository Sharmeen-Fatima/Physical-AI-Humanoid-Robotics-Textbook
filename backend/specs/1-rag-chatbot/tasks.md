# Tasks: RAG Chatbot System

**Feature**: `1-rag-chatbot`
**Branch**: `1-rag-chatbot`
**Created**: 2025-12-20
**Status**: Ready for Implementation

## Overview

This document breaks down the RAG Chatbot System implementation into actionable, independently testable tasks organized by user story priority. Each phase delivers a working increment that can be validated independently.

**MVP Scope**: User Story 1 (Content Ingestion and Vector Storage)

**Total Tasks**: 63
**Estimated Phases**: 6

---

## Implementation Strategy

### Incremental Delivery
- **Phase 1**: Setup (T001-T006) - Project scaffolding and dependencies
- **Phase 2**: Foundational (T007-T011) - Core infrastructure shared across all stories
- **Phase 3**: User Story 1 - P1 (T012-T031) - Content Ingestion MVP
- **Phase 4**: User Story 2 - P2 (T032-T042) - Semantic Query and Retrieval
- **Phase 5**: User Story 3 - P3 (T043-T053) - Grounded Answer Generation
- **Phase 6**: User Story 4 - P4 (T054-T060) - Chatbot Interface and Session Management
- **Phase 7**: Polish & Cross-Cutting (T061-T063) - Final integration and documentation

### Parallel Execution Opportunities
Tasks marked with `[P]` can be executed in parallel with other tasks in the same phase (different files, no dependencies).

---

## Phase 1: Setup

**Goal**: Initialize project structure, dependencies, and configuration framework.

**Independent Test**: Run `pytest --version` and verify all dependencies install successfully.

### Tasks

- [ ] T001 Create backend project structure with src/, tests/, scripts/, specs/, logs/ directories
- [ ] T002 [P] Create requirements.txt with all dependencies (beautifulsoup4, lxml, requests, qdrant-client, cohere, google-generativeai, python-dotenv, pydantic, pytest)
- [ ] T003 [P] Create .env.example template with all required environment variables (COHERE_API_KEY, GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, BOOK_SITEMAP_URL, etc.)
- [ ] T004 [P] Create .gitignore with Python patterns (.env, __pycache__, *.pyc, venv/, logs/)
- [ ] T005 [P] Create backend/README.md with project overview and quickstart instructions
- [ ] T006 Install dependencies in virtual environment and verify imports work

---

## Phase 2: Foundational Infrastructure

**Goal**: Build core shared infrastructure (config, logging, utilities) used by all user stories.

**Independent Test**: Validate that config module loads environment variables and fails fast on missing keys.

### Tasks

- [ ] T007 Implement Pydantic settings model in src/config/settings.py with environment variable validation
- [ ] T008 [P] Implement structured logging setup in src/utils/logger.py with INFO/WARNING/ERROR levels
- [ ] T009 [P] Implement exponential backoff retry decorator in src/utils/retry.py (5 retries, 2x multiplier, jitter)
- [ ] T010 [P] Create Pydantic data schemas in src/storage/schemas.py for BookPage, TextChunk, ChunkMetadata
- [ ] T011 Create startup validation script in scripts/validate_config.py to test environment variables

---

## Phase 3: User Story 1 - Content Ingestion and Vector Storage (P1)

**User Story**: As a system administrator, I need to ingest the entire Physical AI & Humanoid Robotics book content into a searchable vector database.

**Goal**: Complete end-to-end ingestion pipeline: sitemap → scraping → chunking → embedding → Qdrant storage.

**Independent Test**: Run ingestion script on book sitemap and verify:
- All URLs are scraped successfully
- Text chunks are created with proper overlap
- Embeddings are generated for all chunks
- All chunks are stored in Qdrant with complete metadata
- Re-running ingestion creates zero duplicates (idempotency)

### Tasks

#### Sitemap Parsing (T012-T014)

- [ ] T012 [P] [US1] Implement sitemap parser in src/ingestion/sitemap_parser.py to fetch and parse sitemap.xml
- [ ] T013 [US1] Add URL validation logic to filter book-related URLs and ignore non-book pages
- [ ] T014 [US1] Add error handling for unreachable sitemap or malformed XML

#### Web Scraping & HTML Cleaning (T015-T019)

- [ ] T015 [P] [US1] Implement web scraper in src/ingestion/web_scraper.py using requests and BeautifulSoup
- [ ] T016 [US1] Implement HTML cleaning logic to remove <nav>, <header>, <footer>, <script>, <style> tags
- [ ] T017 [US1] Extract section titles from <h1> or <h2> tags for metadata
- [ ] T018 [US1] Extract main content from <main> or <article> tags with fallback to <body>
- [ ] T019 [US1] Add retry logic for network failures during scraping using exponential backoff

#### Text Chunking (T020-T024)

- [ ] T020 [P] [US1] Implement Cohere tokenizer integration in src/ingestion/chunker.py for accurate token counting
- [ ] T021 [US1] Implement overlapping chunking logic (800 tokens, 200 overlap) with configurable parameters
- [ ] T022 [US1] Generate SHA-256 content hash for each chunk to create deterministic UUIDs
- [ ] T023 [US1] Add chunk metadata creation (parent_url, section, chunk_index, char_start, char_end)
- [ ] T024 [US1] Add validation to reject chunks < 100 tokens or > 1000 tokens

#### Embedding Generation (T025-T027)

- [ ] T025 [P] [US1] Implement Cohere embedding client in src/embedding/cohere_embedder.py with API key from config
- [ ] T026 [US1] Implement batch embedding generation (batch size: 96) with rate limit handling
- [ ] T027 [US1] Add retry logic for Cohere API rate limits and transient failures

#### Qdrant Storage (T028-T031)

- [ ] T028 [P] [US1] Implement Qdrant client wrapper in src/storage/qdrant_client.py with connection validation
- [ ] T029 [US1] Implement collection initialization with vector size=1024, distance=Cosine, HNSW indexing (m=16, ef_construct=100)
- [ ] T030 [US1] Implement idempotent upsert logic using SHA-256-based point IDs
- [ ] T031 [US1] Create ingestion script in scripts/ingest_book.py orchestrating full pipeline (sitemap → scraping → chunking → embedding → storage)

---

## Phase 4: User Story 2 - Semantic Query and Retrieval (P2)

**User Story**: As a user, I want to ask natural language questions and receive relevant context chunks.

**Goal**: Implement query embedding and vector similarity search in Qdrant.

**Independent Test**: Submit test queries and verify:
- Query embeddings are generated correctly
- Top-k chunks are retrieved with similarity scores
- Results are filtered by similarity threshold
- Chunks are ranked by relevance

**Dependencies**: Requires User Story 1 (ingestion) to be completed first.

### Tasks

#### Query Embedding (T032-T034)

- [ ] T032 [P] [US2] Implement query embedding function in src/embedding/cohere_embedder.py using input_type="search_query"
- [ ] T033 [US2] Add query text validation (max 500 characters) and truncation if needed
- [ ] T034 [US2] Add retry logic for query embedding API calls

#### Vector Search (T035-T039)

- [ ] T035 [P] [US2] Implement vector similarity search in src/retrieval/vector_search.py using Qdrant client
- [ ] T036 [US2] Add top-k configuration (default k=5, configurable via settings)
- [ ] T037 [US2] Add similarity threshold filtering (default 0.70, configurable)
- [ ] T038 [US2] Implement result ranking by similarity score (highest first)
- [ ] T039 [US2] Handle empty results gracefully (return empty list, log INFO)

#### Retrieval Data Models (T040-T042)

- [ ] T040 [P] [US2] Create Query and RetrievalResult Pydantic models in src/storage/schemas.py
- [ ] T041 [US2] Add metadata extraction from Qdrant payloads (URL, section, chunk_text)
- [ ] T042 [US2] Create retrieval testing script in scripts/test_retrieval.py for manual validation

---

## Phase 5: User Story 3 - Grounded Answer Generation (P3)

**User Story**: As a user, I want to receive accurate, citation-backed answers based only on retrieved book content.

**Goal**: Implement RAG answer generation with strict grounding and hallucination prevention.

**Independent Test**: Submit test queries and verify:
- Answers are generated only from retrieved chunks
- No hallucinated information is present
- Source citations are included
- "Information not found" response for unanswerable queries

**Dependencies**: Requires User Story 2 (retrieval) to be completed first.

### Tasks

#### Prompt Engineering (T043-T046)

- [ ] T043 [P] [US3] Create RAG system prompt template in src/generation/prompts.py with explicit grounding instructions
- [ ] T044 [US3] Implement context formatting function to combine retrieved chunks with metadata
- [ ] T045 [US3] Implement fallback detection logic (if no chunks or low similarity, return "Information not found in the book")
- [ ] T046 [US3] Add citation extraction logic to parse sources from chunk metadata

#### Gemini Integration (T047-T050)

- [ ] T047 [P] [US3] Implement Gemini client wrapper in src/generation/gemini_generator.py with API key from config
- [ ] T048 [US3] Configure Gemini generation parameters (temperature=0.2, top_p=0.8, max_tokens=1024)
- [ ] T049 [US3] Implement answer generation function with system instruction and user prompt
- [ ] T050 [US3] Add retry logic for Gemini API rate limits and failures

#### Response Validation (T051-T053)

- [ ] T051 [P] [US3] Create ChatResponse Pydantic model in src/storage/schemas.py with answer, citations, confidence fields
- [ ] T052 [US3] Implement confidence scoring based on average similarity scores (high: >0.80, medium: 0.70-0.80, low: <0.70)
- [ ] T053 [US3] Create end-to-end RAG testing script in scripts/test_rag.py for manual validation

---

## Phase 6: User Story 4 - Chatbot Interface and Session Management (P4)

**User Story**: As a user, I want to interact through a conversational interface that maintains context across questions.

**Goal**: Implement CLI chatbot with session management for multi-turn conversations.

**Independent Test**: Conduct multi-turn conversation and verify:
- Session ID is created on first query
- Conversation history is maintained
- Follow-up questions are contextualized
- Sessions can be cleared

**Dependencies**: Requires User Story 3 (answer generation) to be completed first.

### Tasks

#### Session Management (T054-T056)

- [ ] T054 [P] [US4] Implement Session and SessionManager classes in src/chatbot/session_manager.py
- [ ] T055 [US4] Add session creation with UUID generation and timestamp tracking
- [ ] T056 [US4] Implement conversation history storage (last 5 messages, 30-min expiration)

#### CLI Interface (T057-T060)

- [ ] T057 [P] [US4] Implement CLI chatbot in src/chatbot/cli_interface.py with input/output loop
- [ ] T058 [US4] Integrate session manager to track conversation context
- [ ] T059 [US4] Add commands: "quit" to exit, "clear" to reset session, "help" for instructions
- [ ] T060 [US4] Create chatbot launcher script in scripts/run_chatbot.py

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Final integration, documentation, and production readiness.

**Independent Test**: Run full system end-to-end and verify all components work together.

### Tasks

- [ ] T061 Create comprehensive integration test in tests/integration/test_rag_pipeline.py covering ingestion → retrieval → generation
- [ ] T062 [P] Add health check endpoint logic for Qdrant/Cohere/Gemini status validation
- [ ] T063 [P] Update backend/README.md with complete usage instructions, troubleshooting, and architecture overview

---

## Dependency Graph

### Story Completion Order

```
Phase 1: Setup
  ↓
Phase 2: Foundational Infrastructure
  ↓
Phase 3: User Story 1 (P1) - Content Ingestion [MVP - INDEPENDENT]
  ↓
Phase 4: User Story 2 (P2) - Semantic Retrieval [DEPENDS ON: US1]
  ↓
Phase 5: User Story 3 (P3) - Answer Generation [DEPENDS ON: US2]
  ↓
Phase 6: User Story 4 (P4) - Chatbot Interface [DEPENDS ON: US3]
  ↓
Phase 7: Polish & Cross-Cutting
```

### Parallel Execution Within Phases

**Phase 1 - Setup**:
- T002, T003, T004, T005 can run in parallel (different files)

**Phase 2 - Foundational**:
- T008, T009, T010 can run in parallel (independent modules)

**Phase 3 - User Story 1**:
- T012 (sitemap) + T015 (scraper) + T020 (chunker) + T025 (embedder) + T028 (qdrant) can be developed in parallel
- Then integrate in T031 (ingestion script)

**Phase 4 - User Story 2**:
- T032 (query embedding) + T035 (vector search) + T040 (data models) can run in parallel

**Phase 5 - User Story 3**:
- T043 (prompts) + T047 (gemini client) + T051 (response model) can run in parallel

**Phase 6 - User Story 4**:
- T054 (session manager) + T057 (CLI interface) can run in parallel

**Phase 7 - Polish**:
- T062, T063 can run in parallel

---

## Testing Strategy

### Unit Tests (Optional - Not Included in Task List)
If TDD approach is requested, add unit tests for:
- Chunking logic (overlap calculation, token counting)
- Embedding generation (API mocking)
- Retrieval filtering (similarity threshold logic)
- Prompt formatting (context assembly)

### Integration Tests
- **T061**: Full RAG pipeline test (included in Phase 7)

### Manual Validation Scripts
- **T011**: Config validation
- **T031**: Ingestion pipeline
- **T042**: Retrieval testing
- **T053**: RAG answer testing
- **T060**: CLI chatbot

---

## Success Criteria Per User Story

### US1: Content Ingestion (Phase 3)
✅ All sitemap URLs are scraped successfully (0 errors)
✅ Text chunks have 800 tokens ±5% with 200-token overlap
✅ Embeddings generated for 100% of chunks
✅ Qdrant stores all chunks with complete metadata
✅ Re-ingestion creates zero duplicates (idempotency validated)

### US2: Semantic Retrieval (Phase 4)
✅ Query embeddings generated in <500ms
✅ Top-k retrieval returns results ranked by similarity
✅ Similarity threshold filters low-relevance chunks
✅ Empty results handled gracefully

### US3: Grounded Answer Generation (Phase 5)
✅ Answers generated only from retrieved context (no hallucinations)
✅ Source citations included in 100% of valid answers
✅ "Information not found" response for unanswerable queries
✅ Confidence scores correlate with retrieval quality

### US4: Chatbot Interface (Phase 6)
✅ Session ID created on first query
✅ Conversation history maintained across messages
✅ Sessions expire after 30 minutes of inactivity
✅ CLI supports "quit", "clear", "help" commands

---

## File Paths Quick Reference

### Configuration & Utils
- `src/config/settings.py` - Pydantic settings
- `src/utils/logger.py` - Logging setup
- `src/utils/retry.py` - Retry decorator

### Data Models
- `src/storage/schemas.py` - All Pydantic models

### Ingestion Pipeline
- `src/ingestion/sitemap_parser.py` - Sitemap parsing
- `src/ingestion/web_scraper.py` - HTML scraping/cleaning
- `src/ingestion/chunker.py` - Text chunking

### Embedding & Storage
- `src/embedding/cohere_embedder.py` - Cohere API client
- `src/storage/qdrant_client.py` - Qdrant wrapper

### Retrieval & Generation
- `src/retrieval/vector_search.py` - Vector similarity search
- `src/generation/prompts.py` - RAG prompts
- `src/generation/gemini_generator.py` - Gemini API client

### Chatbot Interface
- `src/chatbot/session_manager.py` - Session tracking
- `src/chatbot/cli_interface.py` - CLI implementation

### Scripts
- `scripts/validate_config.py` - Config validation
- `scripts/ingest_book.py` - Full ingestion pipeline
- `scripts/test_retrieval.py` - Manual retrieval testing
- `scripts/test_rag.py` - Manual RAG testing
- `scripts/run_chatbot.py` - Chatbot launcher

### Tests
- `tests/integration/test_rag_pipeline.py` - E2E integration test

---

## Notes

- **MVP Recommendation**: Complete Phase 1-3 (Setup + Foundational + User Story 1) to validate the ingestion pipeline before proceeding to retrieval and generation.
- **Parallel Work**: Developers can work on different modules in parallel (marked with `[P]`) to speed up development.
- **Constitution Compliance**: All tasks align with the 6 core principles (Security-First, RAG-Grounding, Modular Code, RAG Standards, Ingestion Integrity, Testing).
- **Idempotency**: Task T030 implements idempotent upsert using content hashing to enable safe re-ingestion.
- **Error Handling**: Retry logic tasks (T009, T019, T027, T034, T050) ensure resilience against API rate limits and transient failures.

---

**Next Steps**:
1. Review and approve this task breakdown
2. Run `/sp.implement` to begin automated implementation
3. Or manually implement tasks starting from Phase 1 (Setup)

**Handoff Options**:
- `/sp.analyze` - Run consistency analysis across spec, plan, and tasks
- `/sp.implement` - Start automated implementation in phases
