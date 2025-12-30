# Implementation Plan: RAG Chatbot System

**Branch**: `1-rag-chatbot` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/1-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a Retrieval-Augmented Generation (RAG) chatbot system that ingests content from the Physical AI & Humanoid Robotics book, stores it in a vector database, and provides accurate, citation-backed answers to user questions based strictly on retrieved context. The system will use Cohere for embeddings, Qdrant for vector storage, and Gemini for answer generation, with strict hallucination prevention and environment-based credential management.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:
- `beautifulsoup4`, `lxml`, `requests` (web scraping)
- `qdrant-client` (vector database)
- `cohere` (embeddings)
- `google-generativeai` (Gemini LLM)
- `python-dotenv` (environment management)
- `pydantic` (configuration validation)
- `pytest` (testing)

**Storage**: Qdrant Cloud (vector database for embeddings and metadata)
**Testing**: `pytest` with fixtures for mock data, integration tests for end-to-end RAG pipeline
**Target Platform**: Python CLI application, cloud-deployable (Docker-compatible)
**Project Type**: Single backend application with modular components
**Performance Goals**:
- Query response time <2s (p95)
- Ingestion throughput: process full book in <30 minutes
- Support 10,000 concurrent queries

**Constraints**:
- Must use environment variables for all credentials
- Zero hallucinations (strict RAG grounding)
- Idempotent re-ingestion
- Configurable chunk size, overlap, and retrieval parameters

**Scale/Scope**:
- Single book source (extensible to multiple)
- Estimated 50-200 pages, 500-2000 chunks
- Support for periodic content refresh

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Security-First Development
✅ **Pass** - All API keys (Cohere, Gemini, Qdrant) will be loaded from `.env` file using `python-dotenv`
✅ **Pass** - Configuration validation with Pydantic will enforce required env vars at startup (fail-fast)
✅ **Pass** - No hard-coded credentials in any code or configuration files

### RAG-Based Grounding (NON-NEGOTIABLE)
✅ **Pass** - Answer generation will only use retrieved chunks from Qdrant
✅ **Pass** - Prompt engineering will explicitly instruct Gemini to use only provided context
✅ **Pass** - Fallback response "Information not found in the book" when retrieval returns empty or low-relevance results
✅ **Pass** - Citation metadata (URL, section) will be included in all responses

### Modular & Production-Ready Code
✅ **Pass** - Python 3.10+ with type hints for all functions
✅ **Pass** - Single-responsibility modules: ingestion, chunking, embedding, retrieval, generation
✅ **Pass** - Structured logging with Python's `logging` module (INFO, WARNING, ERROR levels)
✅ **Pass** - Explicit exception handling with custom error types
✅ **Pass** - Docstrings for all public interfaces

### RAG Architecture Standards
✅ **Pass** - Overlapping chunking strategy (default: 800 tokens, 200 overlap)
✅ **Pass** - Metadata schema: `{url, section, chunk_index, timestamp, book_version}`
✅ **Pass** - Consistent Cohere embedding model across ingestion and retrieval
✅ **Pass** - Qdrant as primary vector store
✅ **Pass** - Configurable retrieval parameters (top-k, similarity threshold)

### Data Ingestion Integrity
✅ **Pass** - Sitemap parsing with validation for well-formed URLs
✅ **Pass** - HTML cleaning with BeautifulSoup (remove nav, headers, footers)
✅ **Pass** - Idempotent ingestion using content hashing to detect duplicates
✅ **Pass** - Content validation before chunking (min length, non-empty)
✅ **Pass** - Book version tracking in metadata

### Testing & Quality Assurance
✅ **Pass** - Unit tests for chunking, embedding, retrieval logic
✅ **Pass** - Integration tests for full RAG pipeline
✅ **Pass** - Test fixtures with sample book excerpts
✅ **Pass** - Edge case coverage (empty results, long queries, special characters)

**Overall Status**: ✅ ALL GATES PASS - Proceed to Phase 0 Research

## Project Structure

### Documentation (this feature)

```text
specs/1-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── rag-api.yaml    # API contract for query/response
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── config/
│   │   ├── __init__.py
│   │   └── settings.py          # Environment validation with Pydantic
│   ├── ingestion/
│   │   ├── __init__.py
│   │   ├── sitemap_parser.py    # Extract URLs from sitemap.xml
│   │   ├── web_scraper.py       # Scrape and clean HTML content
│   │   └── chunker.py           # Split text into overlapping chunks
│   ├── embedding/
│   │   ├── __init__.py
│   │   └── cohere_embedder.py   # Generate embeddings via Cohere API
│   ├── storage/
│   │   ├── __init__.py
│   │   ├── qdrant_client.py     # Qdrant operations (init, upsert, search)
│   │   └── schemas.py           # Data models for chunks and metadata
│   ├── retrieval/
│   │   ├── __init__.py
│   │   └── vector_search.py     # Query Qdrant for relevant chunks
│   ├── generation/
│   │   ├── __init__.py
│   │   ├── gemini_generator.py  # Generate answers via Gemini
│   │   └── prompts.py           # RAG prompt templates
│   ├── chatbot/
│   │   ├── __init__.py
│   │   ├── cli_interface.py     # CLI for user interaction
│   │   └── session_manager.py   # Session context management
│   └── utils/
│       ├── __init__.py
│       ├── logger.py            # Structured logging setup
│       └── retry.py             # Exponential backoff retry logic
│
├── tests/
│   ├── unit/
│   │   ├── test_chunker.py
│   │   ├── test_embedder.py
│   │   ├── test_retrieval.py
│   │   └── test_generator.py
│   ├── integration/
│   │   ├── test_ingestion_pipeline.py
│   │   └── test_rag_pipeline.py
│   └── fixtures/
│       ├── sample_html.py
│       └── sample_chunks.py
│
├── scripts/
│   ├── ingest_book.py           # CLI script to run ingestion
│   └── run_chatbot.py           # CLI script to start chatbot
│
├── .env.example                 # Template for environment variables
├── .gitignore
├── requirements.txt             # Python dependencies
└── README.md

```

**Structure Decision**: Single backend Python application structure chosen because:
- All components are tightly coupled in the RAG pipeline
- No separate frontend (CLI-based initially)
- Modular design allows future API endpoint addition without restructuring
- Clear separation of concerns: ingestion → embedding → storage → retrieval → generation

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected** - All constitution gates pass without requiring justification.

## Phase 0: Research & Unknowns

**Status**: In Progress

### Research Tasks

#### 1. Cohere Embedding Model Selection
**Unknown**: Which Cohere embedding model to use for optimal semantic retrieval?

**Research needed**:
- Compare `embed-english-v3.0` vs `embed-multilingual-v3.0`
- Evaluate embedding dimensions (1024 vs 768)
- Test performance on technical/academic content
- Validate token limits and batch processing capabilities

#### 2. Chunking Strategy Optimization
**Unknown**: Optimal chunk size and overlap for technical book content?

**Research needed**:
- Industry best practices for RAG chunking (200-1000 tokens)
- Evaluate semantic boundary detection (sentence vs paragraph)
- Test overlap ratios (10-30% recommended)
- Validate preservation of context across chunk boundaries

#### 3. Qdrant Collection Configuration
**Unknown**: Optimal Qdrant index settings for retrieval performance?

**Research needed**:
- Vector dimensionality matching Cohere embeddings
- Distance metric selection (cosine vs dot product vs euclidean)
- Indexing parameters (HNSW vs flat)
- Quantization options for performance/accuracy tradeoff

#### 4. Similarity Threshold Tuning
**Unknown**: What similarity score threshold ensures high relevance without false negatives?

**Research needed**:
- Baseline similarity distribution for sample queries
- Precision-recall tradeoff analysis
- Industry standards for semantic search thresholds (0.6-0.8 typical)

#### 5. Gemini Prompt Engineering
**Unknown**: How to structure prompts for strict grounding and hallucination prevention?

**Research needed**:
- Zero-shot vs few-shot prompting for RAG
- System instructions for citation enforcement
- Temperature and top-p settings for deterministic output
- Safety settings configuration

#### 6. Idempotent Ingestion Strategy
**Unknown**: How to detect and prevent duplicate chunks during re-ingestion?

**Research needed**:
- Content hashing approaches (SHA-256 of text)
- Qdrant upsert behavior vs explicit deduplication
- Versioning strategy for updated book content
- Performance impact of duplicate detection

#### 7. HTML Cleaning Best Practices
**Unknown**: How to reliably extract clean text from diverse HTML structures?

**Research needed**:
- BeautifulSoup selectors for content vs navigation
- Handling code blocks, tables, and special formatting
- Preservation of headings/sections for metadata
- Dealing with JavaScript-rendered content (if applicable)

#### 8. Error Handling and Retry Logic
**Unknown**: Optimal retry strategy for API rate limits and transient failures?

**Research needed**:
- Cohere and Gemini API rate limits and quotas
- Exponential backoff parameters (initial delay, max retries, backoff multiplier)
- Circuit breaker patterns for degraded services
- Fallback behaviors when APIs are unavailable

#### 9. Testing Strategy for RAG Quality
**Unknown**: How to systematically test and validate RAG answer quality?

**Research needed**:
- Manual evaluation frameworks (relevance, accuracy, citation correctness)
- Automated metrics (BLEU, ROUGE, semantic similarity)
- Test dataset creation (ground-truth Q&A pairs)
- Hallucination detection techniques

#### 10. Session Management Implementation
**Unknown**: How to maintain conversation context efficiently for multi-turn conversations?

**Research needed**:
- In-memory vs persistent session storage
- Context window management for Gemini (token limits)
- Session expiration and cleanup strategies
- Concurrency handling for multiple users

---

**Next Step**: Create `research.md` with findings for each research task before proceeding to Phase 1.

## Phase 1: Design & Contracts

**Status**: Pending (awaits Phase 0 completion)

Will generate:
- `data-model.md` - Entity schemas and relationships
- `contracts/rag-api.yaml` - API specifications for query/response endpoints
- `quickstart.md` - Development setup and usage guide

## Architectural Decisions

### Decision 1: Cohere for Embeddings
**Options Considered**:
- OpenAI `text-embedding-3-small` (1536 dimensions)
- Cohere `embed-english-v3.0` (1024 dimensions)
- Sentence Transformers (open-source, self-hosted)

**Chosen**: Cohere `embed-english-v3.0`

**Rationale**:
- Spec explicitly requires Cohere as embedding provider
- Strong performance on semantic search tasks
- Competitive pricing and rate limits
- Native batch processing support

**Tradeoffs**:
- Vendor lock-in (mitigated by consistent API abstraction)
- Requires API key management (already required by constitution)

---

### Decision 2: Gemini for Answer Generation
**Options Considered**:
- OpenAI GPT-4
- Anthropic Claude
- Google Gemini (multiple models)

**Chosen**: Google Gemini Pro 1.5

**Rationale**:
- Spec explicitly requires Gemini as LLM provider
- Large context window (1M tokens) supports extensive retrieved chunks
- Strong instruction-following for grounding enforcement
- Competitive pricing

**Tradeoffs**:
- Vendor lock-in (mitigated by generation module abstraction)
- Requires prompt engineering for hallucination prevention

---

### Decision 3: Qdrant Cloud for Vector Storage
**Options Considered**:
- Qdrant Cloud (managed)
- Qdrant self-hosted (Docker)
- Pinecone
- Weaviate

**Chosen**: Qdrant Cloud

**Rationale**:
- Spec explicitly requires Qdrant
- Managed service reduces operational overhead
- Strong support for metadata filtering
- HNSW indexing for fast similarity search

**Tradeoffs**:
- Cloud dependency (mitigated by Qdrant's portable data format)
- Costs scale with vector count (acceptable for book-scale dataset)

---

### Decision 4: CLI-First Interface
**Options Considered**:
- Web API (FastAPI/Flask)
- CLI (Python argparse/Click)
- Web UI (Gradio/Streamlit)

**Chosen**: CLI with optional API endpoint support

**Rationale**:
- Simplest MVP for validation
- Easier testing and debugging
- Can add FastAPI layer later without restructuring
- Aligns with modular architecture principle

**Tradeoffs**:
- Limited user experience vs web UI
- No built-in session persistence (will add in-memory for P4)

---

### Decision 5: Overlapping Chunking Strategy
**Options Considered**:
- Fixed-size chunks (no overlap)
- Semantic chunking (sentence/paragraph boundaries)
- Overlapping fixed-size chunks

**Chosen**: Overlapping fixed-size chunks (800 tokens, 200 overlap)

**Rationale**:
- Preserves context across boundaries (critical for technical content)
- Predictable chunk sizes for embedding model
- Industry-standard approach for RAG systems
- Configurable for tuning

**Tradeoffs**:
- Increased storage (25% overlap = 25% more chunks)
- Potential redundancy in retrieval (acceptable, improves recall)

---

### Decision 6: Content Hashing for Idempotency
**Options Considered**:
- URL-based deduplication
- SHA-256 content hashing
- Qdrant upsert with external IDs

**Chosen**: SHA-256 content hashing with Qdrant point IDs

**Rationale**:
- Detects identical content even if URL changes
- Efficient comparison without storing full text
- Qdrant supports deterministic point IDs for upsert behavior
- Enables versioning when content updates

**Tradeoffs**:
- Additional computation during ingestion (negligible for book-scale)
- Hash collisions theoretically possible (SHA-256 makes this extremely unlikely)

---

## Non-Functional Requirements

### Performance
- **Latency**: Query-to-answer <2s (p95), ingestion throughput >10 pages/minute
- **Scalability**: Support 10,000 concurrent queries (Qdrant Cloud handles this)
- **Resource limits**: <500MB memory for CLI process, <50ms per chunk embedding

### Reliability
- **Availability**: 99% uptime for query operations (depends on Qdrant/Cohere/Gemini SLAs)
- **Error handling**: Exponential backoff for transient API failures, max 5 retries
- **Data integrity**: 100% ingestion success rate with validation checkpoints

### Security
- **Credentials**: All API keys in `.env`, validated at startup, never logged
- **Data privacy**: No PII handling (book content only)
- **API security**: Rate limiting and quota monitoring for external APIs

### Maintainability
- **Code coverage**: >80% unit test coverage, 100% coverage for critical RAG pipeline
- **Documentation**: Inline docstrings, README with setup/usage, ADR for key decisions
- **Logging**: Structured logs with request IDs, log levels appropriate to severity

## Risks and Mitigation

### Risk 1: API Rate Limiting During Bulk Ingestion
**Impact**: High (blocks ingestion pipeline)
**Probability**: Medium (depends on book size and API quotas)

**Mitigation**:
- Implement exponential backoff retry logic
- Batch embedding requests where supported by Cohere
- Monitor API quota usage and alert on approaching limits
- Add configurable delay between requests

---

### Risk 2: Low Retrieval Relevance (Poor Chunk Quality)
**Impact**: High (degrades answer accuracy)
**Probability**: Medium (depends on chunking strategy tuning)

**Mitigation**:
- Create test dataset with ground-truth Q&A pairs
- Measure retrieval precision/recall during testing
- Make chunk size, overlap, and similarity threshold configurable
- Implement A/B testing framework for parameter tuning

---

### Risk 3: Hallucinations in Generated Answers
**Impact**: Critical (violates constitution principle)
**Probability**: Low-Medium (Gemini may extrapolate beyond context)

**Mitigation**:
- Explicit prompt engineering: "ONLY use provided context, do NOT add external knowledge"
- Post-generation validation: check answer tokens against retrieved chunks
- Manual review of sample responses during testing
- Lower temperature (0.2-0.3) for more deterministic output

---

### Risk 4: Qdrant Cloud Service Outage
**Impact**: High (blocks all queries)
**Probability**: Low (managed service SLA)

**Mitigation**:
- Implement health checks for Qdrant connection
- Graceful degradation: return error message instead of crashing
- Consider Qdrant local fallback for development/testing
- Monitor Qdrant status page and set up alerts

---

### Risk 5: Book Content Updates Invalidate Index
**Impact**: Medium (stale answers)
**Probability**: Low (book updates infrequent)

**Mitigation**:
- Track book version in metadata during ingestion
- Implement periodic re-ingestion workflow (manual or scheduled)
- Add "Last Updated" timestamp in chatbot responses
- Support incremental updates (detect changed pages only)

---

## Success Metrics

### Ingestion Success
- ✅ 100% sitemap URL extraction success rate
- ✅ 100% page scraping success rate (retry on transient failures)
- ✅ 100% chunking success (no data loss)
- ✅ 100% embedding generation success
- ✅ 100% Qdrant storage success
- ✅ Zero duplicates on re-ingestion

### Retrieval Quality
- ✅ >80% retrieval relevance (manual evaluation on 50 test queries)
- ✅ <2s p95 query latency (embedding + search + generation)
- ✅ Top-5 chunks include correct answer for >90% of answerable queries

### Answer Quality
- ✅ Zero hallucinations (100% grounding to retrieved context)
- ✅ 100% citation inclusion (URL + section) in answers
- ✅ "Information not found" response for 100% of unanswerable queries
- ✅ Coherent synthesis of multi-chunk information (subjective, manual review)

### System Reliability
- ✅ 100% startup validation (fail-fast on missing env vars)
- ✅ 100% error handling coverage (no silent failures)
- ✅ <30 minutes full book ingestion time
- ✅ 99% uptime for query operations (excluding scheduled maintenance)

---

## Next Steps

1. **Phase 0 Completion**: Generate `research.md` with findings for all 10 research tasks
2. **Phase 1 Execution**:
   - Create `data-model.md` with entity schemas
   - Generate `contracts/rag-api.yaml` with API specifications
   - Write `quickstart.md` with setup instructions
3. **Agent Context Update**: Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude`
4. **Proceed to `/sp.tasks`**: Break down plan into actionable, testable tasks
