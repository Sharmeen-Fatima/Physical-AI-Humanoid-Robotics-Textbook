# RAG Chatbot for Physical AI & Humanoid Robotics Book

A Retrieval-Augmented Generation (RAG) chatbot system that provides accurate, citation-backed answers to questions about the Physical AI & Humanoid Robotics textbook.

## Features

- **Content Ingestion**: Automatically scrapes and indexes book content from web sitemap
- **Semantic Search**: Vector-based similarity search using Cohere embeddings and Qdrant
- **Grounded Answers**: Gemini-powered answer generation strictly based on retrieved context
- **Citation Support**: All answers include source URLs and section references
- **Hallucination Prevention**: Zero external knowledge injection - only book content is used
- **Session Management**: Multi-turn conversational interface with context tracking

## Quick Start

### Prerequisites

- Python 3.10+
- API Keys for:
  - Cohere (embeddings)
  - Google Gemini (LLM)
  - Qdrant Cloud (vector database)

### Installation

1. Clone the repository and navigate to backend:
```bash
cd backend
```

2. Create virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Configure environment variables:
```bash
cp .env.example .env
# Edit .env with your API keys
```

5. Run configuration validation:
```bash
python scripts/validate_config.py
```

### Usage

#### 1. Health Check

Before starting, verify all systems are operational:

```bash
python scripts/health_check.py
```

This validates:
- Configuration loaded successfully
- All API keys are present and valid
- Qdrant connection is working
- Cohere API is accessible
- Gemini API is accessible
- All modules can be imported

#### 2. Ingest Book Content

```bash
python scripts/ingest_book.py
```

This will:
- Fetch all pages from the book sitemap
- Clean and chunk the text content (800 tokens, 200 overlap)
- Generate embeddings using Cohere (1024 dimensions)
- Store vectors and metadata in Qdrant (HNSW indexing)

**Output**: Logs progress to console and `logs/ingestion.log`

#### 3. Test Retrieval (Optional)

Verify semantic search is working:

```bash
python scripts/test_retrieval.py
```

Tests:
- Basic search functionality
- Search with formatted context
- Threshold filtering (0.60, 0.70, 0.80)
- Top-k variation (1, 3, 5, 10)
- Edge cases (empty queries, long queries)
- Engine statistics

#### 4. Test Generation (Optional)

Verify answer generation is working:

```bash
python scripts/test_generation.py
```

Tests:
- Basic RAG pipeline (retrieve → generate)
- Multi-turn conversations with history
- No context handling
- Citation validation
- Response validation
- Prompt building

#### 5. Run the Chatbot

```bash
python scripts/chatbot.py
```

**Interactive Commands**:
- `/help` - Show help message
- `/stats` - Show session statistics
- `/clear` - Clear conversation history
- `/quit` - Exit chatbot

**Example interaction**:
```
You: What are the main components of a humanoid robot?

[Searching book content...]
[Found 5 relevant sections]
[Generating answer...]

======================================================================
ANSWER:
======================================================================
The main components of a humanoid robot include [Source 1] sensors for
perception, actuators for movement, control systems for coordination,
and power systems [Source 2]. These work together to enable autonomous
operation and human-like movement patterns.

SOURCES:
----------------------------------------------------------------------
  [1] Chapter 3: Robot Components (score: 0.873)
      https://physical-ai-humanoid-robotics-textb-pied.vercel.app/chapter-3/components
  [2] Chapter 4: Power Systems (score: 0.821)
      https://physical-ai-humanoid-robotics-textb-pied.vercel.app/chapter-4/power

METADATA:
----------------------------------------------------------------------
  Model: gemini-1.5-pro | Sources: 5 | Grounded: True | Citations: True
======================================================================

You: How do sensors work?
```

#### 6. Run Integration Tests

Test the complete end-to-end pipeline:

```bash
python tests/test_integration.py
```

Or using pytest:

```bash
pytest tests/test_integration.py -v
```

Tests:
- Configuration and component initialization
- Retrieval pipeline (embedding → search → ranking)
- Generation pipeline (context → answer → validation)
- Session management (create, update, history)
- Full RAG workflow (single-turn and multi-turn)

## Project Structure

```
backend/
├── src/
│   ├── config/          # Environment configuration
│   ├── ingestion/       # Web scraping and chunking
│   ├── embedding/       # Cohere embedding generation
│   ├── storage/         # Qdrant client and schemas
│   ├── retrieval/       # Vector similarity search
│   ├── generation/      # Gemini answer generation
│   ├── chatbot/         # CLI interface and sessions
│   └── utils/           # Logging and retry logic
├── tests/               # Unit and integration tests
├── scripts/             # Executable scripts
├── logs/                # Application logs
└── specs/               # Feature specifications and planning docs
```

## Architecture

### RAG Pipeline

```
User Query
    ↓
Query Embedding (Cohere)
    ↓
Vector Search (Qdrant)
    ↓
Top-K Retrieval + Filtering
    ↓
Context Assembly
    ↓
Answer Generation (Gemini)
    ↓
Citation Formatting
    ↓
Response to User
```

### Technology Stack

- **Web Scraping**: BeautifulSoup4, requests, lxml
- **Embeddings**: Cohere `embed-english-v3.0` (1024 dimensions)
- **Vector Database**: Qdrant Cloud (HNSW indexing, cosine similarity)
- **LLM**: Google Gemini 1.5 Pro
- **Config**: Pydantic, python-dotenv
- **Testing**: pytest

## Configuration

All configuration is managed via environment variables in `.env`:

| Variable | Description | Example |
|----------|-------------|---------|
| `COHERE_API_KEY` | Cohere API key for embeddings | `abc123...` |
| `GEMINI_API_KEY` | Google Gemini API key | `xyz789...` |
| `QDRANT_URL` | Qdrant Cloud cluster URL | `https://your-cluster.cloud.qdrant.io` |
| `QDRANT_API_KEY` | Qdrant API key | `def456...` |
| `CHUNK_SIZE` | Token count per chunk | `800` |
| `CHUNK_OVERLAP` | Overlap between chunks | `200` |
| `TOP_K` | Number of chunks to retrieve | `5` |
| `SIMILARITY_THRESHOLD` | Minimum similarity score | `0.70` |

## Available Scripts

All scripts are located in the `scripts/` directory:

| Script | Purpose | Usage |
|--------|---------|-------|
| `validate_config.py` | Validate environment configuration | `python scripts/validate_config.py` |
| `health_check.py` | Check system health (APIs, DB, imports) | `python scripts/health_check.py` |
| `ingest_book.py` | Ingest book content into vector DB | `python scripts/ingest_book.py` |
| `test_retrieval.py` | Test semantic search functionality | `python scripts/test_retrieval.py` |
| `test_generation.py` | Test answer generation pipeline | `python scripts/test_generation.py` |
| `chatbot.py` | Run interactive CLI chatbot | `python scripts/chatbot.py` |

## Development

### Running Tests

```bash
# Integration tests (recommended)
python tests/test_integration.py

# Or using pytest
pytest tests/test_integration.py -v

# All tests with coverage (when unit tests are added)
pytest tests/ --cov=src --cov-report=html
```

### Logging

All logs are written to `logs/` directory:
- `logs/ingestion.log` - Book ingestion pipeline
- `logs/retrieval_test.log` - Retrieval testing
- `logs/generation_test.log` - Generation testing
- `logs/chatbot.log` - Chatbot sessions
- `logs/integration_test.log` - Integration tests
- `logs/health_check.log` - Health check results

Log levels can be configured via `LOG_LEVEL` environment variable (DEBUG, INFO, WARNING, ERROR).

### Project Documentation

- **Specification**: `specs/1-rag-chatbot/spec.md` - Feature requirements
- **Implementation Plan**: `specs/1-rag-chatbot/plan.md` - Architecture decisions
- **Data Model**: `specs/1-rag-chatbot/data-model.md` - Entity schemas
- **API Contracts**: `specs/1-rag-chatbot/contracts/rag-api.yaml` - OpenAPI spec
- **Quickstart Guide**: `specs/1-rag-chatbot/quickstart.md` - Detailed setup
- **Tasks**: `specs/1-rag-chatbot/tasks.md` - Implementation breakdown

### Constitution

This project follows strict principles defined in `.specify/memory/constitution.md`:

1. **Security-First**: Environment-based secrets, no hard-coded credentials
2. **RAG-Based Grounding**: Zero hallucinations, citation-backed answers
3. **Modular Code**: Single-responsibility, type hints, structured logging
4. **RAG Architecture**: Overlapping chunks, metadata tracking, configurable parameters
5. **Data Integrity**: Idempotent ingestion, content validation, versioning
6. **Quality Assurance**: Unit + integration testing, edge case coverage

## Troubleshooting

See `specs/1-rag-chatbot/quickstart.md` for common issues and solutions.

## License

[Your License Here]

## Contributing

[Your Contributing Guidelines Here]
"# Backend-Physical-AI" 
