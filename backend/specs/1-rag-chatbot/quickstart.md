# Quickstart Guide: RAG Chatbot System

**Feature**: `1-rag-chatbot`
**Date**: 2025-12-20
**Audience**: Developers setting up the project for the first time

---

## Prerequisites

Before you begin, ensure you have the following:

### System Requirements
- **Python**: 3.10 or higher
- **pip**: Latest version
- **Git**: For version control
- **Internet connection**: Required for API calls (Cohere, Gemini, Qdrant Cloud)

### API Keys Required
You will need to sign up for and obtain API keys from:

1. **Cohere**: [https://dashboard.cohere.com/](https://dashboard.cohere.com/)
   - Free tier: 100 requests/minute
   - Model: `embed-english-v3.0`

2. **Google Gemini**: [https://ai.google.dev/](https://ai.google.dev/)
   - Free tier: 60 requests/minute
   - Model: `gemini-1.5-pro`

3. **Qdrant Cloud**: [https://cloud.qdrant.io/](https://cloud.qdrant.io/)
   - Free tier: 1GB cluster
   - Create a collection named `physical_ai_book`

---

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd backend
```

### 2. Create a Virtual Environment

```bash
# Create virtual environment
python3.10 -m venv venv

# Activate virtual environment
# On Linux/Mac:
source venv/bin/activate

# On Windows:
venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

**Expected `requirements.txt` contents**:
```txt
beautifulsoup4==4.12.2
lxml==4.9.3
requests==2.31.0
qdrant-client==1.7.0
cohere==4.37
google-generativeai==0.3.2
python-dotenv==1.0.0
pydantic==2.5.0
pytest==7.4.3
pytest-asyncio==0.21.1
```

### 4. Configure Environment Variables

Create a `.env` file in the `backend/` directory:

```bash
# Copy the example file
cp .env.example .env

# Edit .env with your API keys
nano .env  # or use your preferred editor
```

**`.env` file structure**:
```env
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Gemini API Configuration
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Configuration
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=physical_ai_book

# Book Configuration
BOOK_SITEMAP_URL=https://physical-ai-humanoid-robotics-textb-pied.vercel.app/sitemap.xml
BOOK_BASE_URL=https://physical-ai-humanoid-robotics-textb-pied.vercel.app

# RAG Configuration
CHUNK_SIZE=800
CHUNK_OVERLAP=200
TOP_K=5
SIMILARITY_THRESHOLD=0.70

# Logging
LOG_LEVEL=INFO
```

**Security Note**: Never commit the `.env` file to version control. It's already included in `.gitignore`.

### 5. Validate Configuration

Run the configuration validator to ensure all required environment variables are set:

```bash
python src/config/settings.py
```

**Expected output**:
```text
✓ All required environment variables are set
✓ Cohere API key validated
✓ Gemini API key validated
✓ Qdrant connection successful
✓ Configuration loaded successfully
```

If any errors occur, double-check your `.env` file for missing or incorrect values.

---

## Initial Setup: Ingest Book Content

Before you can query the chatbot, you need to ingest the book content into Qdrant.

### Step 1: Initialize Qdrant Collection

```bash
python scripts/init_qdrant.py
```

**What this does**:
- Creates a new Qdrant collection named `physical_ai_book`
- Configures vector size (1024), distance metric (cosine), and HNSW indexing
- Verifies collection creation

**Expected output**:
```text
[INFO] Connecting to Qdrant at https://your-cluster.cloud.qdrant.io
[INFO] Creating collection 'physical_ai_book'
[INFO] Collection configuration:
       - Vector size: 1024
       - Distance: Cosine
       - HNSW: m=16, ef_construct=100
[SUCCESS] Collection 'physical_ai_book' created successfully
```

### Step 2: Run Book Ingestion

```bash
python scripts/ingest_book.py
```

**What this does**:
1. Fetches sitemap from configured URL
2. Scrapes all book pages
3. Cleans HTML and extracts text
4. Splits text into overlapping chunks (800 tokens, 200 overlap)
5. Generates embeddings via Cohere
6. Stores chunks and embeddings in Qdrant

**Expected output**:
```text
[INFO] Starting book ingestion
[INFO] Fetching sitemap from https://physical-ai-humanoid-robotics-textb-pied.vercel.app/sitemap.xml
[INFO] Found 42 URLs in sitemap
[INFO] Scraping page 1/42: /chapter-1/introduction
[INFO] Scraping page 2/42: /chapter-2/fundamentals
...
[INFO] Scraped 42 pages successfully
[INFO] Created 850 text chunks
[INFO] Generating embeddings (batch size: 96)
[INFO] Batch 1/9: 96 chunks
[INFO] Batch 2/9: 96 chunks
...
[INFO] Generated 850 embeddings
[INFO] Storing chunks in Qdrant
[INFO] Stored 850/850 chunks
[SUCCESS] Book ingestion completed in 18m 32s
[SUMMARY] Pages: 42, Chunks: 850, Errors: 0
```

**Troubleshooting**:
- If ingestion fails, check logs in `logs/ingestion.log`
- For rate limit errors, reduce batch size or add delays
- Re-run ingestion (idempotent via content hashing)

---

## Running the Chatbot

### CLI Interface

Start the chatbot CLI:

```bash
python scripts/run_chatbot.py
```

**Example interaction**:
```text
=== RAG Chatbot for Physical AI & Humanoid Robotics ===
Type your question or 'quit' to exit

You: What are the main types of actuators used in humanoid robots?

Bot: The main types of actuators used in humanoid robots include:
     1. Electric motors (DC, brushless, servo)
     2. Hydraulic actuators (high force-to-weight ratio)
     3. Pneumatic actuators (compliant, safe for human interaction)

     Each type has trade-offs in power density, control precision, and safety.

     Sources:
     - https://physical-ai-humanoid-robotics-textb-pied.vercel.app/chapter-3/actuators (Chapter 3: Actuators and Power Systems)

You: Which one is best for bipedal walking?

Bot: Hydraulic actuators are often preferred for bipedal walking due to their high power-to-weight ratio, enabling robots to generate sufficient ground reaction forces for dynamic balance. However, electric motors with advanced control algorithms are also viable for lighter, energy-efficient humanoids.

     Sources:
     - https://physical-ai-humanoid-robotics-textb-pied.vercel.app/chapter-5/locomotion (Chapter 5: Bipedal Locomotion)

You: quit

Goodbye!
```

### API Server (Optional - Future Enhancement)

To run the chatbot as a REST API:

```bash
# Install FastAPI (if not already in requirements.txt)
pip install fastapi uvicorn

# Start the API server
uvicorn src.chatbot.api:app --reload --host 0.0.0.0 --port 8000
```

**API will be available at**: `http://localhost:8000`

**API Documentation (Swagger UI)**: `http://localhost:8000/docs`

---

## Testing

### Run Unit Tests

```bash
pytest tests/unit -v
```

**Expected output**:
```text
tests/unit/test_chunker.py::test_chunk_text_basic PASSED
tests/unit/test_chunker.py::test_chunk_text_with_overlap PASSED
tests/unit/test_embedder.py::test_generate_embedding PASSED
tests/unit/test_retrieval.py::test_filter_by_similarity PASSED
...
==================== 15 passed in 3.24s ====================
```

### Run Integration Tests

```bash
pytest tests/integration -v
```

**Note**: Integration tests require valid API keys and will make actual API calls.

### Run All Tests with Coverage

```bash
pytest --cov=src --cov-report=html
```

**View coverage report**: Open `htmlcov/index.html` in your browser.

---

## Project Structure

After setup, your directory structure should look like this:

```text
backend/
├── src/
│   ├── config/
│   │   ├── __init__.py
│   │   └── settings.py
│   ├── ingestion/
│   │   ├── __init__.py
│   │   ├── sitemap_parser.py
│   │   ├── web_scraper.py
│   │   └── chunker.py
│   ├── embedding/
│   │   ├── __init__.py
│   │   └── cohere_embedder.py
│   ├── storage/
│   │   ├── __init__.py
│   │   ├── qdrant_client.py
│   │   └── schemas.py
│   ├── retrieval/
│   │   ├── __init__.py
│   │   └── vector_search.py
│   ├── generation/
│   │   ├── __init__.py
│   │   ├── gemini_generator.py
│   │   └── prompts.py
│   ├── chatbot/
│   │   ├── __init__.py
│   │   ├── cli_interface.py
│   │   └── session_manager.py
│   └── utils/
│       ├── __init__.py
│       ├── logger.py
│       └── retry.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── fixtures/
├── scripts/
│   ├── init_qdrant.py
│   ├── ingest_book.py
│   └── run_chatbot.py
├── specs/
│   └── 1-rag-chatbot/
│       ├── spec.md
│       ├── plan.md
│       ├── research.md
│       ├── data-model.md
│       ├── quickstart.md  # This file
│       └── contracts/
│           └── rag-api.yaml
├── logs/
│   ├── app.log
│   └── ingestion.log
├── .env
├── .env.example
├── .gitignore
├── requirements.txt
└── README.md
```

---

## Common Issues and Troubleshooting

### Issue: "ModuleNotFoundError: No module named 'cohere'"

**Solution**: Ensure you've activated the virtual environment and installed dependencies:
```bash
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -r requirements.txt
```

### Issue: "QdrantException: Collection not found"

**Solution**: Run the Qdrant initialization script:
```bash
python scripts/init_qdrant.py
```

### Issue: "Rate limit exceeded" during ingestion

**Solution**: Reduce batch size or add delays between API calls. Edit `src/embedding/cohere_embedder.py`:
```python
BATCH_SIZE = 50  # Reduce from 96
time.sleep(1)  # Add delay between batches
```

### Issue: "Information not found in the book" for known topics

**Solutions**:
1. **Lower similarity threshold**: Edit `.env` and set `SIMILARITY_THRESHOLD=0.60`
2. **Increase top-k**: Set `TOP_K=10` to retrieve more chunks
3. **Re-ingest book**: Content may not have been ingested properly

### Issue: Chatbot generates hallucinated information

**Solutions**:
1. **Check prompt engineering**: Ensure `src/generation/prompts.py` includes explicit grounding instructions
2. **Lower temperature**: Edit Gemini configuration to use `temperature=0.1` (more deterministic)
3. **Review retrieved chunks**: Add `--debug` flag to see which chunks are being used

---

## Next Steps

After completing this quickstart, you can:

1. **Test the RAG Pipeline**: Ask various questions to validate retrieval and answer quality
2. **Tune Parameters**: Experiment with chunk size, overlap, top-k, and similarity threshold
3. **Add More Books**: Extend ingestion to support multiple book sources
4. **Deploy to Production**: Containerize with Docker and deploy to cloud (AWS, GCP, Azure)
5. **Build Web UI**: Create a web interface using React/Vue for better UX

---

## Additional Resources

- **Constitution**: `backend/.specify/memory/constitution.md` - Project principles and standards
- **Specification**: `backend/specs/1-rag-chatbot/spec.md` - Feature requirements
- **Implementation Plan**: `backend/specs/1-rag-chatbot/plan.md` - Architectural decisions
- **Data Model**: `backend/specs/1-rag-chatbot/data-model.md` - Entity schemas
- **API Contracts**: `backend/specs/1-rag-chatbot/contracts/rag-api.yaml` - API specifications

---

## Support and Feedback

If you encounter issues not covered in this guide, please:
1. Check the logs in `logs/` directory
2. Review the specification and plan documents
3. Open an issue in the project repository

Happy building! 🤖📚