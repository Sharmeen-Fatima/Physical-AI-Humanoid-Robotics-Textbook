# RAG Chatbot for Physical AI & Humanoid Robotics Book - Constitution

## Core Principles

### I. Security-First Development
- **All API keys and secrets MUST be loaded via environment variables** (`.env` files)
- **No hard-coded credentials** are allowed in code, prompts, or configuration files
- **Secrets management**: Use `python-dotenv` for local development, cloud secret managers for production
- **Validation**: All environment variables must be validated at startup
- **Fail-fast policy**: Application must not start if required secrets are missing

### II. RAG-Based Grounding (NON-NEGOTIABLE)
- **The chatbot MUST only answer using retrieved context** from the vector database
- **Hallucinations are strictly prohibited** - no generated information beyond retrieved chunks
- **If relevant context is missing**, the system MUST respond with: "Information not found in the book."
- **Citation transparency**: Responses should reference source sections/URLs when possible
- **Retrieval-first**: Every query requires vector search before LLM generation

### III. Modular & Production-Ready Code
- **Primary language**: Python 3.10+
- **Code organization**: Modular, single-responsibility components
- **Type hints**: Mandatory for all function signatures
- **Error handling**: Explicit exception handling, no silent failures
- **Logging**: Structured logging at appropriate levels (INFO, WARNING, ERROR)
- **Documentation**: Docstrings for all public functions and classes

### IV. RAG Architecture Standards
- **Chunking strategy**: Overlapping semantic chunks for context continuity
- **Chunk metadata**: Every chunk MUST include:
  - Source URL
  - Section/chapter identifier
  - Chunk index/position
  - Timestamp of ingestion
- **Embedding model**: Consistent model across ingestion and retrieval
- **Vector database**: Qdrant as the primary vector store
- **Retrieval parameters**: Top-k and similarity threshold must be configurable

### V. Data Ingestion Integrity
- **Source URLs**:
  - Sitemap: `https://physical-ai-humanoid-robotics-textb-pied.vercel.app/sitemap.xml`
  - Book URL: `https://physical-ai-humanoid-robotics-textb-pied.vercel.app/`
- **Content extraction**: Parse HTML cleanly, preserve structure, remove navigation/UI elements
- **Idempotency**: Re-ingestion should be safe and not create duplicates
- **Validation**: Verify extracted content quality before chunking
- **Versioning**: Track book version/update timestamp in metadata

### VI. Testing & Quality Assurance
- **Unit tests**: For chunking logic, embedding generation, retrieval functions
- **Integration tests**: End-to-end RAG pipeline testing
- **Test data**: Sample book excerpts for reproducible testing
- **Quality metrics**: Measure retrieval precision/recall where possible
- **Edge cases**: Test empty results, long queries, special characters

## Technology Stack

### Required Dependencies
- **Web scraping**: `beautifulsoup4`, `requests`, `lxml`
- **Vector database**: `qdrant-client`
- **Embeddings**: OpenAI API (`openai`) or open-source alternatives
- **Environment management**: `python-dotenv`
- **LLM integration**: OpenAI API or compatible provider
- **Async support**: `asyncio`, `aiohttp` (if needed for performance)

### Configuration Management
- **Environment files**: `.env` for local, cloud secrets for production
- **Config validation**: Pydantic models for type-safe configuration
- **Defaults**: Sensible defaults with explicit override capability
- **Documentation**: All config options documented in `.env.example`

## Development Workflow

### Feature Development
1. **Specification**: Create feature spec in `specs/<feature>/spec.md`
2. **Planning**: Document architecture in `specs/<feature>/plan.md`
3. **Task breakdown**: Define testable tasks in `specs/<feature>/tasks.md`
4. **Implementation**: Follow smallest viable change principle
5. **Testing**: Write tests, verify coverage, run integration tests
6. **Documentation**: Update README, add inline comments for complex logic

### Code Review Standards
- **Security review**: Verify no secrets in code
- **RAG compliance**: Confirm grounding and no hallucinations
- **Code quality**: Check modularity, type hints, error handling
- **Testing**: Verify test coverage and edge case handling
- **Documentation**: Ensure changes are documented

### Deployment Requirements
- **Environment validation**: All required secrets present
- **Health checks**: Startup validation for database connection, API keys
- **Logging**: Production-level logging configured
- **Monitoring**: Track query latency, retrieval quality, error rates
- **Scalability**: Design must support cloud deployment (Docker, serverless, etc.)

## Governance

### Constitution Supremacy
- This constitution supersedes all other development practices
- All code changes must comply with stated principles
- Amendments require documentation and ratification
- Non-compliance must be explicitly justified and approved

### Quality Gates
- **Pre-commit**: No secrets in code, linting passes
- **Pre-merge**: Tests pass, code review approved
- **Pre-deploy**: Integration tests pass, security scan clean

### Continuous Improvement
- **Prompt History Records (PHR)**: Document significant development sessions
- **Architecture Decision Records (ADR)**: Document architectural choices
- **Retrospectives**: Review and refine principles based on learnings

**Version**: 1.0.0 | **Ratified**: 2025-12-20 | **Last Amended**: 2025-12-20
