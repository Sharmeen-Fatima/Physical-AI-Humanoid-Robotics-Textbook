# Research Findings: RAG Chatbot System

**Feature**: `1-rag-chatbot`
**Date**: 2025-12-20
**Phase**: 0 (Research & Unknown Resolution)

## Overview

This document consolidates research findings for all technical unknowns and dependencies identified in the implementation plan. Each research task includes the decision made, rationale, alternatives considered, and implementation guidance.

---

## 1. Cohere Embedding Model Selection

### Decision
Use **Cohere `embed-english-v3.0`** embedding model with **1024 dimensions**.

### Rationale
- Optimized for English semantic search tasks
- 1024-dimensional embeddings provide strong semantic capture with reasonable storage overhead
- Supports batch processing (up to 96 texts per request)
- Token limit: 512 tokens per text (sufficient for 800-token chunks with truncation)
- Pricing: Competitive for embedding generation at scale
- Performance: Strong benchmarks on retrieval tasks (BEIR, MTEB)

### Alternatives Considered
- **`embed-multilingual-v3.0`**: Not needed (book is English-only)
- **`embed-english-light-v3.0`** (384 dimensions): Lower quality retrieval, minimal cost savings
- **768-dimensional models**: No significant advantage over 1024 for this use case

### Implementation Guidance
```python
import cohere

co = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

response = co.embed(
    texts=["chunk text here"],
    model="embed-english-v3.0",
    input_type="search_document",  # Use "search_document" for indexing, "search_query" for queries
    embedding_types=["float"]
)

embeddings = response.embeddings
```

**Key Parameters**:
- `input_type="search_document"` for ingestion (asymmetric search optimization)
- `input_type="search_query"` for user queries
- `embedding_types=["float"]` for full precision (required by Qdrant)

---

## 2. Chunking Strategy Optimization

### Decision
Use **fixed-size overlapping chunks** with:
- **Chunk size**: 800 tokens
- **Overlap**: 200 tokens (25%)
- **Tokenization**: Cohere tokenizer for consistency with embedding model

### Rationale
- **800 tokens**: Balances context preservation and retrieval precision
  - Large enough to capture complete thoughts/paragraphs
  - Small enough to avoid diluting semantic meaning
  - Fits within Cohere's 512-token embedding limit with truncation handling
- **200-token overlap**: Ensures continuity across boundaries
  - Captures concepts spanning chunk edges
  - Industry standard: 20-30% overlap recommended
- **Fixed-size over semantic**: More predictable, easier to configure

### Alternatives Considered
- **Semantic chunking (sentence/paragraph boundaries)**: Variable chunk sizes complicate retrieval tuning
- **Larger chunks (1500+ tokens)**: Dilutes semantic specificity, exceeds embedding model limits
- **No overlap**: Risks losing context at boundaries (critical for technical content)

### Implementation Guidance
```python
import cohere

co = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

def chunk_text(text: str, chunk_size: int = 800, overlap: int = 200) -> list[str]:
    """Split text into overlapping chunks using Cohere tokenizer."""
    # Tokenize full text
    tokens = co.tokenize(text=text, model="embed-english-v3.0").tokens

    chunks = []
    start = 0
    while start < len(tokens):
        end = min(start + chunk_size, len(tokens))
        chunk_tokens = tokens[start:end]

        # Detokenize back to text
        chunk_text = co.detokenize(tokens=chunk_tokens, model="embed-english-v3.0").text
        chunks.append(chunk_text)

        start += (chunk_size - overlap)  # Advance with overlap

    return chunks
```

**Configuration Parameters** (make these configurable in settings):
- `CHUNK_SIZE = 800`
- `CHUNK_OVERLAP = 200`

---

## 3. Qdrant Collection Configuration

### Decision
Use the following Qdrant collection settings:
- **Vector size**: 1024 (matches Cohere embeddings)
- **Distance metric**: **Cosine similarity**
- **Indexing**: **HNSW** (Hierarchical Navigable Small World)
- **Quantization**: None (use full vectors for accuracy)

### Rationale
- **Cosine similarity**: Standard for semantic search, normalizes vector magnitude differences
- **HNSW**: Fast approximate nearest neighbor search with high recall
  - `m=16` (connections per node): Balanced speed/accuracy
  - `ef_construct=100`: Build-time accuracy (can be tuned)
- **No quantization**: Dataset is small enough (<10k vectors) that full precision is feasible

### Alternatives Considered
- **Dot product**: Requires normalized vectors (cosine is safer default)
- **Euclidean distance**: Less common for embeddings, no clear advantage
- **Scalar quantization**: Reduces storage/memory but impacts accuracy (not needed for book-scale dataset)

### Implementation Guidance
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, HnswConfigDiff

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

collection_name = "physical_ai_book"

client.create_collection(
    collection_name=collection_name,
    vectors_config=VectorParams(
        size=1024,
        distance=Distance.COSINE
    ),
    hnsw_config=HnswConfigDiff(
        m=16,
        ef_construct=100
    )
)
```

**Search Configuration**:
```python
# During retrieval
search_params = {"hnsw_ef": 128}  # Runtime accuracy (higher = more accurate, slower)
```

---

## 4. Similarity Threshold Tuning

### Decision
Use **0.70** as the default similarity threshold (cosine similarity score).

### Rationale
- Industry standard for semantic search: 0.6-0.8 range
- **0.70** provides:
  - High precision (few false positives)
  - Acceptable recall (captures relevant chunks)
- Will be configurable for tuning based on empirical testing

### Alternatives Considered
- **0.60**: Higher recall but more noise (lower precision)
- **0.80**: Very high precision but risks missing relevant content (lower recall)
- **Dynamic thresholding**: Complex, requires ML-based calibration

### Implementation Guidance
```python
MIN_SIMILARITY_THRESHOLD = 0.70  # Configurable in settings

def filter_results(search_results, threshold=MIN_SIMILARITY_THRESHOLD):
    """Filter search results by similarity score."""
    return [result for result in search_results if result.score >= threshold]
```

**Tuning Process**:
1. Create test dataset with ground-truth Q&A pairs
2. Measure precision/recall at different thresholds (0.60, 0.65, 0.70, 0.75, 0.80)
3. Select threshold that maximizes F1 score
4. Document final threshold in configuration

---

## 5. Gemini Prompt Engineering

### Decision
Use **Gemini 1.5 Pro** with the following prompt structure:

**System Instruction**:
```text
You are a knowledgeable assistant for the "Physical AI and Humanoid Robotics" textbook. Your ONLY source of information is the provided context from the book. Follow these rules strictly:

1. ONLY answer using information from the provided context
2. Do NOT use any external knowledge or training data
3. If the context does not contain enough information to answer the question, respond EXACTLY with: "Information not found in the book."
4. Always cite your sources by including the section/page reference from the context
5. Be concise but complete in your answers
```

**User Prompt Template**:
```text
Context from the book:
{retrieved_chunks_with_metadata}

User Question: {user_query}

Instructions: Answer the question using ONLY the context above. Include source citations.
```

### Rationale
- **Explicit grounding instruction**: Repeatedly emphasizes "only use context"
- **Fallback response**: Exact wording prevents rephrasing that might add information
- **Citation requirement**: Forces model to reference sources
- **Temperature = 0.2**: Low temperature reduces creativity/hallucination risk

### Model Configuration
```python
import google.generativeai as genai

genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

model = genai.GenerativeModel(
    model_name="gemini-1.5-pro",
    system_instruction=SYSTEM_INSTRUCTION,
    generation_config={
        "temperature": 0.2,
        "top_p": 0.8,
        "top_k": 40,
        "max_output_tokens": 1024,
    },
    safety_settings={
        "HARM_CATEGORY_HARASSMENT": "BLOCK_NONE",
        "HARM_CATEGORY_HATE_SPEECH": "BLOCK_NONE",
        "HARM_CATEGORY_SEXUALLY_EXPLICIT": "BLOCK_NONE",
        "HARM_CATEGORY_DANGEROUS_CONTENT": "BLOCK_NONE",
    }
)
```

### Implementation Guidance
```python
def format_context(chunks: list[dict]) -> str:
    """Format retrieved chunks with metadata for prompt."""
    context_parts = []
    for i, chunk in enumerate(chunks, 1):
        context_parts.append(
            f"[Source {i}] (URL: {chunk['url']}, Section: {chunk['section']})\n"
            f"{chunk['text']}\n"
        )
    return "\n---\n".join(context_parts)

def generate_answer(query: str, retrieved_chunks: list[dict]) -> str:
    context = format_context(retrieved_chunks)
    prompt = f"""Context from the book:
{context}

User Question: {query}

Instructions: Answer the question using ONLY the context above. Include source citations."""

    response = model.generate_content(prompt)
    return response.text
```

---

## 6. Idempotent Ingestion Strategy

### Decision
Use **SHA-256 content hashing** with Qdrant point IDs for idempotent ingestion.

### Rationale
- Hash the chunk text to generate a deterministic ID
- Qdrant's upsert behavior: if point ID exists, update; else insert
- Detects duplicate content even if URL or metadata changes
- Enables safe re-ingestion without manual deduplication

### Implementation Guidance
```python
import hashlib
import uuid

def generate_chunk_id(chunk_text: str) -> str:
    """Generate deterministic UUID from chunk content hash."""
    hash_digest = hashlib.sha256(chunk_text.encode('utf-8')).hexdigest()
    # Use first 32 chars of hash as UUID
    return str(uuid.UUID(hash_digest[:32]))

def upsert_chunks(client, collection_name, chunks):
    """Upsert chunks to Qdrant (idempotent operation)."""
    from qdrant_client.models import PointStruct

    points = []
    for chunk in chunks:
        point_id = generate_chunk_id(chunk["text"])
        points.append(PointStruct(
            id=point_id,
            vector=chunk["embedding"],
            payload={
                "text": chunk["text"],
                "url": chunk["url"],
                "section": chunk["section"],
                "chunk_index": chunk["chunk_index"],
                "timestamp": chunk["timestamp"],
                "book_version": chunk["book_version"]
            }
        ))

    client.upsert(collection_name=collection_name, points=points)
```

**Versioning Strategy**:
- Include `book_version` or `last_updated` timestamp in metadata
- On re-ingestion, compare versions to detect content changes
- Update chunks if book version has changed

---

## 7. HTML Cleaning Best Practices

### Decision
Use **BeautifulSoup4** with the following extraction strategy:

1. Parse HTML with `lxml` parser (fast and lenient)
2. Remove unwanted elements: `<nav>`, `<header>`, `<footer>`, `<aside>`, `<script>`, `<style>`
3. Extract main content from `<main>` or `<article>` tags (if present)
4. Extract headings (`<h1>` to `<h6>`) for section metadata
5. Preserve code blocks and formatting for technical content

### Rationale
- **lxml parser**: Fast and handles malformed HTML gracefully
- **Tag-based removal**: Targets common navigation/UI elements
- **Semantic HTML support**: Prioritizes `<main>` and `<article>` for content
- **Metadata extraction**: Headings provide section context for chunks

### Implementation Guidance
```python
from bs4 import BeautifulSoup

def clean_html(html_content: str) -> tuple[str, str]:
    """Extract clean text and section title from HTML."""
    soup = BeautifulSoup(html_content, 'lxml')

    # Remove unwanted elements
    for tag in soup(['nav', 'header', 'footer', 'aside', 'script', 'style', 'noscript']):
        tag.decompose()

    # Extract section title (first h1/h2)
    section_title = ""
    heading = soup.find(['h1', 'h2'])
    if heading:
        section_title = heading.get_text(strip=True)

    # Extract main content
    main_content = soup.find('main') or soup.find('article') or soup.find('body')

    if main_content:
        # Get text with preserved structure
        text = main_content.get_text(separator='\n', strip=True)
        # Clean up excessive whitespace
        text = '\n'.join(line.strip() for line in text.splitlines() if line.strip())
    else:
        text = ""

    return text, section_title
```

**Edge Cases**:
- **JavaScript-rendered content**: Not handled by BeautifulSoup (would require Selenium/Playwright if needed)
- **Tables and code blocks**: Preserved in text extraction (may need special handling for formatting)
- **Special characters**: UTF-8 encoding handles most cases; validate on sample pages

---

## 8. Error Handling and Retry Logic

### Decision
Implement **exponential backoff retry** with the following parameters:

- **Initial delay**: 1 second
- **Backoff multiplier**: 2x
- **Max retries**: 5
- **Max delay**: 60 seconds
- **Jitter**: ±20% randomization to avoid thundering herd

### Rationale
- **Exponential backoff**: Standard pattern for transient API failures
- **5 retries**: Covers most transient errors without excessive waiting
- **Jitter**: Prevents synchronized retries in batch operations
- **60s max delay**: Caps total wait time at reasonable limit

### API Rate Limits (as of Dec 2025)
- **Cohere Free Tier**: 100 requests/minute (upgrade for production)
- **Gemini Free Tier**: 60 requests/minute
- **Qdrant Cloud**: No explicit rate limit (depends on plan)

### Implementation Guidance
```python
import time
import random
from functools import wraps

def retry_with_exponential_backoff(
    max_retries=5,
    initial_delay=1.0,
    backoff_multiplier=2.0,
    max_delay=60.0,
    jitter=0.2
):
    """Decorator for retry logic with exponential backoff."""
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            delay = initial_delay
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except (ConnectionError, TimeoutError, Exception) as e:
                    last_exception = e

                    if attempt == max_retries:
                        raise last_exception

                    # Add jitter
                    jittered_delay = delay * (1 + random.uniform(-jitter, jitter))
                    jittered_delay = min(jittered_delay, max_delay)

                    print(f"Retry {attempt + 1}/{max_retries} after {jittered_delay:.2f}s due to: {e}")
                    time.sleep(jittered_delay)

                    delay *= backoff_multiplier

            raise last_exception
        return wrapper
    return decorator

# Usage
@retry_with_exponential_backoff()
def call_cohere_api(texts):
    return co.embed(texts=texts, model="embed-english-v3.0")
```

**Fallback Behaviors**:
- **Cohere API down**: Log error, skip chunk embedding (mark as failed for retry)
- **Gemini API down**: Return "Service temporarily unavailable" to user
- **Qdrant down**: Fail health check, prevent queries (graceful degradation)

---

## 9. Testing Strategy for RAG Quality

### Decision
Implement **multi-layered testing**:

1. **Unit tests**: Test individual components (chunking, embedding, retrieval logic)
2. **Integration tests**: Test end-to-end RAG pipeline
3. **Manual evaluation**: Sample-based quality review
4. **Automated metrics**: ROUGE-L for answer similarity to ground truth

### Rationale
- **Unit tests**: Fast, isolate component bugs
- **Integration tests**: Validate full pipeline behavior
- **Manual evaluation**: Gold standard for quality (human judgment)
- **Automated metrics**: Scale evaluation without manual review

### Test Dataset Creation
1. **Ground-truth Q&A pairs**: Create 20-50 question-answer pairs from book content
2. **Negative examples**: Questions outside book scope (should return "Information not found")
3. **Edge cases**: Long queries, ambiguous questions, multi-hop reasoning

### Evaluation Metrics
- **Retrieval Precision**: Percentage of retrieved chunks that are relevant
- **Retrieval Recall**: Percentage of relevant chunks that are retrieved
- **Answer Accuracy**: Manual assessment (correct, partially correct, incorrect, hallucinated)
- **Citation Correctness**: Percentage of answers with valid source citations

### Implementation Guidance
```python
# Sample test case structure
test_qa_pairs = [
    {
        "question": "What is the primary advantage of hydraulic actuation in humanoid robots?",
        "expected_answer_contains": ["high force-to-weight ratio", "power density"],
        "expected_sources": ["chapter-3", "actuators"],
        "category": "factual"
    },
    {
        "question": "How does the weather affect crop yields in Europe?",
        "expected_response": "Information not found in the book.",
        "category": "out_of_scope"
    }
]

def evaluate_rag_pipeline(test_cases):
    results = {"correct": 0, "incorrect": 0, "hallucinated": 0}

    for case in test_cases:
        answer = rag_pipeline(case["question"])

        if case["category"] == "out_of_scope":
            if answer == case["expected_response"]:
                results["correct"] += 1
            else:
                results["hallucinated"] += 1
        else:
            # Check if answer contains expected keywords
            if any(keyword in answer.lower() for keyword in case["expected_answer_contains"]):
                results["correct"] += 1
            else:
                results["incorrect"] += 1

    return results
```

---

## 10. Session Management Implementation

### Decision
Use **in-memory session storage** with the following design:

- **Session ID**: UUID generated on first query
- **Storage**: Python dictionary (for MVP), Redis (for production)
- **Expiration**: 30 minutes of inactivity
- **Context window**: Last 5 user messages + responses (prevent token overflow)

### Rationale
- **In-memory**: Simplest for MVP, no external dependencies
- **UUID session IDs**: Unique, secure, non-guessable
- **30-minute expiration**: Balances UX (persistent conversation) and memory usage
- **Limited context window**: Prevents exceeding Gemini token limits

### Alternatives Considered
- **Redis**: Better for production (persistence, scalability) but adds dependency
- **Database**: Overkill for ephemeral session data
- **No session management**: Simpler but poor multi-turn conversation UX

### Implementation Guidance
```python
import uuid
from datetime import datetime, timedelta
from dataclasses import dataclass
from typing import Dict, List

@dataclass
class SessionData:
    session_id: str
    messages: List[dict]  # {"role": "user"|"assistant", "content": str, "timestamp": datetime}
    last_activity: datetime
    created_at: datetime

class SessionManager:
    def __init__(self, expiration_minutes=30, max_context_messages=5):
        self.sessions: Dict[str, SessionData] = {}
        self.expiration_minutes = expiration_minutes
        self.max_context_messages = max_context_messages

    def create_session(self) -> str:
        """Create a new session and return session ID."""
        session_id = str(uuid.uuid4())
        self.sessions[session_id] = SessionData(
            session_id=session_id,
            messages=[],
            last_activity=datetime.now(),
            created_at=datetime.now()
        )
        return session_id

    def add_message(self, session_id: str, role: str, content: str):
        """Add a message to the session history."""
        if session_id not in self.sessions:
            raise ValueError(f"Session {session_id} not found")

        session = self.sessions[session_id]
        session.messages.append({
            "role": role,
            "content": content,
            "timestamp": datetime.now()
        })

        # Keep only last N messages
        if len(session.messages) > self.max_context_messages * 2:
            session.messages = session.messages[-self.max_context_messages * 2:]

        session.last_activity = datetime.now()

    def get_context(self, session_id: str) -> List[dict]:
        """Retrieve conversation context for a session."""
        if session_id not in self.sessions:
            return []

        # Clean up expired sessions
        self._cleanup_expired()

        return self.sessions.get(session_id, SessionData("", [], datetime.now(), datetime.now())).messages

    def _cleanup_expired(self):
        """Remove sessions that have exceeded expiration time."""
        expiration_threshold = datetime.now() - timedelta(minutes=self.expiration_minutes)
        expired = [sid for sid, session in self.sessions.items() if session.last_activity < expiration_threshold]

        for sid in expired:
            del self.sessions[sid]

    def clear_session(self, session_id: str):
        """Clear a specific session."""
        if session_id in self.sessions:
            del self.sessions[session_id]
```

**Usage in Chatbot**:
```python
session_manager = SessionManager()

# First query
session_id = session_manager.create_session()
session_manager.add_message(session_id, "user", "What is SLAM?")
# ... generate answer ...
session_manager.add_message(session_id, "assistant", answer)

# Follow-up query
session_manager.add_message(session_id, "user", "How is it used in robots?")
context = session_manager.get_context(session_id)
# ... use context in prompt ...
```

---

## Summary of Research Findings

| Research Task | Decision | Key Parameters |
|---------------|----------|----------------|
| Embedding Model | Cohere `embed-english-v3.0` | 1024 dimensions, batch processing |
| Chunking Strategy | Fixed-size overlapping | 800 tokens, 200 overlap |
| Qdrant Configuration | HNSW + Cosine similarity | m=16, ef_construct=100 |
| Similarity Threshold | 0.70 (configurable) | Balances precision/recall |
| Gemini Prompting | Explicit grounding + low temp | Temperature=0.2, citation enforcement |
| Idempotency | SHA-256 content hashing | Deterministic point IDs |
| HTML Cleaning | BeautifulSoup + semantic tags | Remove nav/header/footer, extract main content |
| Retry Logic | Exponential backoff | 5 retries, 1s initial delay, 2x multiplier |
| Testing Strategy | Multi-layered (unit, integration, manual) | Ground-truth Q&A dataset, ROUGE-L metrics |
| Session Management | In-memory UUID sessions | 30-min expiration, 5-message context window |

---

## Next Steps

All research tasks are resolved. Proceed to **Phase 1: Design & Contracts**.

**Phase 1 Deliverables**:
1. `data-model.md` - Entity schemas and relationships
2. `contracts/rag-api.yaml` - API specifications
3. `quickstart.md` - Development setup guide
4. Update agent context with technology choices
