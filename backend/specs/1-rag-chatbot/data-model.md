# Data Model: RAG Chatbot System

**Feature**: `1-rag-chatbot`
**Date**: 2025-12-20
**Phase**: 1 (Design & Contracts)

## Overview

This document defines the data entities, schemas, and relationships for the RAG chatbot system. All entities are designed to support the ingestion, storage, retrieval, and generation workflows.

---

## Entity Definitions

### 1. BookPage

Represents a single web page from the Physical AI & Humanoid Robotics book.

**Attributes**:

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| `url` | `str` | Full URL of the page | Required, must be valid HTTP(S) URL |
| `raw_html` | `str` | Unprocessed HTML content from scraping | Optional (may not be persisted) |
| `cleaned_text` | `str` | Extracted text after HTML cleaning | Required, min length 100 characters |
| `section_title` | `str` | Page/section heading (extracted from `<h1>` or `<h2>`) | Optional |
| `scrape_timestamp` | `datetime` | When the page was scraped | Required, ISO 8601 format |
| `content_hash` | `str` | SHA-256 hash of cleaned_text (for change detection) | Required, 64 hex characters |

**Relationships**:
- One `BookPage` generates multiple `TextChunk` entities

**Validation Rules**:
- `url` must match sitemap domain: `physical-ai-humanoid-robotics-textb-pied.vercel.app`
- `cleaned_text` must have at least 100 characters (reject empty/invalid pages)
- `scrape_timestamp` must be in UTC

**Example**:
```python
{
    "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/chapter-3/actuators",
    "cleaned_text": "Actuators are the muscles of robots...",
    "section_title": "Chapter 3: Actuators and Power Systems",
    "scrape_timestamp": "2025-12-20T10:30:00Z",
    "content_hash": "a1b2c3d4e5f6..."
}
```

---

### 2. TextChunk

Represents a semantic segment of text created from a `BookPage`.

**Attributes**:

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| `chunk_id` | `str` | Unique identifier (UUID derived from content hash) | Required, UUID format |
| `text` | `str` | Chunk content (tokenized to ~800 tokens) | Required, 100-1000 tokens |
| `chunk_index` | `int` | Position of chunk within parent page (0-indexed) | Required, >= 0 |
| `parent_url` | `str` | URL of the source `BookPage` | Required, references `BookPage.url` |
| `section_title` | `str` | Section/chapter identifier | Optional, inherited from `BookPage` |
| `char_start` | `int` | Character offset in original page text | Required, >= 0 |
| `char_end` | `int` | End character offset in original page text | Required, > char_start |
| `overlap_start` | `int` | Tokens overlapping with previous chunk | Default 0 for first chunk |
| `overlap_end` | `int` | Tokens overlapping with next chunk | Default 0 for last chunk |

**Relationships**:
- Multiple `TextChunk` entities belong to one `BookPage`
- One `TextChunk` generates one `VectorEmbedding`

**Validation Rules**:
- `text` must be 100-1000 tokens (using Cohere tokenizer)
- `chunk_index` must be unique within `parent_url`
- `char_end` must be greater than `char_start`

**Example**:
```python
{
    "chunk_id": "550e8400-e29b-41d4-a716-446655440000",
    "text": "Hydraulic actuators provide high force density...",
    "chunk_index": 0,
    "parent_url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/chapter-3/actuators",
    "section_title": "Chapter 3: Actuators and Power Systems",
    "char_start": 0,
    "char_end": 3200,
    "overlap_start": 0,
    "overlap_end": 200
}
```

---

### 3. VectorEmbedding

Represents the vector representation of a `TextChunk`.

**Attributes**:

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| `chunk_id` | `str` | References `TextChunk.chunk_id` | Required, UUID format |
| `embedding_vector` | `List[float]` | 1024-dimensional vector from Cohere | Required, length = 1024 |
| `embedding_model` | `str` | Cohere model used for generation | Required, e.g., "embed-english-v3.0" |
| `generation_timestamp` | `datetime` | When embedding was created | Required, ISO 8601 format |

**Relationships**:
- One `VectorEmbedding` belongs to one `TextChunk`
- Stored in Qdrant as a vector point

**Validation Rules**:
- `embedding_vector` must have exactly 1024 float values
- All vector values should be in range [-1.0, 1.0] (typical for normalized embeddings)
- `embedding_model` must match configured model

**Example**:
```python
{
    "chunk_id": "550e8400-e29b-41d4-a716-446655440000",
    "embedding_vector": [0.012, -0.034, 0.056, ...],  # 1024 values
    "embedding_model": "embed-english-v3.0",
    "generation_timestamp": "2025-12-20T10:32:15Z"
}
```

---

### 4. ChunkMetadata

Metadata stored alongside vectors in Qdrant for filtering and citation.

**Attributes**:

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| `text` | `str` | Full chunk text (for display/context) | Required |
| `url` | `str` | Source page URL (for citation) | Required |
| `section` | `str` | Section/chapter title (for citation) | Optional |
| `chunk_index` | `int` | Position within page | Required, >= 0 |
| `ingestion_timestamp` | `datetime` | When chunk was ingested into Qdrant | Required, ISO 8601 |
| `book_version` | `str` | Version identifier for content tracking | Optional, e.g., "2025-12-20" |

**Stored in**: Qdrant point payload

**Validation Rules**:
- All fields must be JSON-serializable (Qdrant requirement)
- `url` must be a valid HTTP(S) URL
- `ingestion_timestamp` must be in ISO 8601 format

**Example**:
```python
{
    "text": "Hydraulic actuators provide high force density...",
    "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/chapter-3/actuators",
    "section": "Chapter 3: Actuators and Power Systems",
    "chunk_index": 0,
    "ingestion_timestamp": "2025-12-20T10:35:00Z",
    "book_version": "2025-12-20"
}
```

---

### 5. Query

Represents a user question submitted to the chatbot.

**Attributes**:

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| `query_id` | `str` | Unique identifier for the query | Required, UUID format |
| `query_text` | `str` | User's question | Required, max 500 characters |
| `query_embedding` | `List[float]` | Embedding of query (for retrieval) | Required, length = 1024 |
| `session_id` | `str` | Associated session (for multi-turn) | Optional, UUID format |
| `timestamp` | `datetime` | When query was submitted | Required, ISO 8601 format |

**Relationships**:
- One `Query` may be associated with one `Session` (if multi-turn conversation)
- One `Query` generates one or more `RetrievalResult` entities

**Validation Rules**:
- `query_text` must be non-empty and <= 500 characters
- `query_embedding` must have exactly 1024 float values

**Example**:
```python
{
    "query_id": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
    "query_text": "What are the advantages of hydraulic actuators?",
    "query_embedding": [0.023, -0.045, ...],
    "session_id": "12345678-90ab-cdef-1234-567890abcdef",
    "timestamp": "2025-12-20T14:22:10Z"
}
```

---

### 6. RetrievalResult

Represents chunks retrieved from Qdrant for a given query.

**Attributes**:

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| `query_id` | `str` | References `Query.query_id` | Required, UUID format |
| `chunk_id` | `str` | Retrieved chunk ID | Required, UUID format |
| `chunk_text` | `str` | Text of the retrieved chunk | Required |
| `metadata` | `dict` | Chunk metadata (URL, section, etc.) | Required, contains `ChunkMetadata` fields |
| `similarity_score` | `float` | Cosine similarity score | Required, 0.0-1.0 |
| `rank` | `int` | Ranking position (1 = most relevant) | Required, >= 1 |

**Relationships**:
- Multiple `RetrievalResult` entities belong to one `Query`
- One `RetrievalResult` references one `TextChunk`

**Validation Rules**:
- `similarity_score` must be in range [0.0, 1.0]
- `rank` must be unique within `query_id` (no ties)

**Example**:
```python
{
    "query_id": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
    "chunk_id": "550e8400-e29b-41d4-a716-446655440000",
    "chunk_text": "Hydraulic actuators provide high force density...",
    "metadata": {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/chapter-3/actuators",
        "section": "Chapter 3: Actuators and Power Systems",
        "chunk_index": 0
    },
    "similarity_score": 0.87,
    "rank": 1
}
```

---

### 7. ChatResponse

Represents the final answer generated by Gemini.

**Attributes**:

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| `response_id` | `str` | Unique identifier for the response | Required, UUID format |
| `query_id` | `str` | References `Query.query_id` | Required, UUID format |
| `answer_text` | `str` | Generated answer or fallback message | Required |
| `citations` | `List[dict]` | Source references with URL and section | Optional, list of `{url, section}` |
| `chunks_used` | `List[str]` | List of chunk IDs used in generation | Required |
| `confidence_indicator` | `str` | High/Medium/Low (based on similarity scores) | Optional |
| `timestamp` | `datetime` | When response was generated | Required, ISO 8601 format |
| `model_used` | `str` | Gemini model identifier | Required, e.g., "gemini-1.5-pro" |

**Relationships**:
- One `ChatResponse` belongs to one `Query`
- One `ChatResponse` references multiple `RetrievalResult` entities (via `chunks_used`)

**Validation Rules**:
- `answer_text` must be non-empty
- If `answer_text` is "Information not found in the book.", `citations` must be empty
- `chunks_used` must contain valid UUIDs

**Example**:
```python
{
    "response_id": "f1e2d3c4-b5a6-4b5c-8d9e-0f1a2b3c4d5e",
    "query_id": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
    "answer_text": "Hydraulic actuators offer high force-to-weight ratios...",
    "citations": [
        {"url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/chapter-3/actuators", "section": "Chapter 3"}
    ],
    "chunks_used": ["550e8400-e29b-41d4-a716-446655440000"],
    "confidence_indicator": "High",
    "timestamp": "2025-12-20T14:22:12Z",
    "model_used": "gemini-1.5-pro"
}
```

---

### 8. Session (Optional - for P4 Multi-Turn Conversations)

Represents a conversation session for context tracking.

**Attributes**:

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| `session_id` | `str` | Unique session identifier | Required, UUID format |
| `created_at` | `datetime` | Session creation timestamp | Required, ISO 8601 format |
| `last_activity` | `datetime` | Last interaction timestamp | Required, ISO 8601 format |
| `messages` | `List[dict]` | Conversation history | Required, list of `{role, content, timestamp}` |
| `is_active` | `bool` | Whether session is still valid | Required, default True |

**Relationships**:
- One `Session` contains multiple `Query` and `ChatResponse` pairs

**Validation Rules**:
- `messages` should not exceed 10 messages (5 user + 5 assistant) to prevent token overflow
- `is_active` set to False after 30 minutes of inactivity

**Example**:
```python
{
    "session_id": "12345678-90ab-cdef-1234-567890abcdef",
    "created_at": "2025-12-20T14:00:00Z",
    "last_activity": "2025-12-20T14:22:12Z",
    "messages": [
        {"role": "user", "content": "What is SLAM?", "timestamp": "2025-12-20T14:10:00Z"},
        {"role": "assistant", "content": "SLAM stands for...", "timestamp": "2025-12-20T14:10:05Z"}
    ],
    "is_active": True
}
```

---

## Entity Relationships Diagram

```text
BookPage (1) ----< TextChunk (N)
                      |
                      | (1:1)
                      |
                  VectorEmbedding
                      |
                      | stored as
                      |
                  Qdrant Point (with ChunkMetadata)
                      |
                      | retrieved by
                      |
Query (1) ----< RetrievalResult (N) ----< TextChunk (N)
  |                                            |
  | (1:1)                                      | (N:1)
  |                                            |
ChatResponse                             ChunkMetadata
  |
  | (N:1) [optional]
  |
Session
```

---

## Storage Implementation

### Qdrant Point Schema

Each `VectorEmbedding` is stored as a Qdrant point:

```python
from qdrant_client.models import PointStruct

point = PointStruct(
    id=chunk_id,  # UUID string from TextChunk
    vector=embedding_vector,  # 1024-dimensional float list
    payload={
        "text": chunk_text,
        "url": parent_url,
        "section": section_title,
        "chunk_index": chunk_index,
        "ingestion_timestamp": timestamp_iso,
        "book_version": book_version
    }
)
```

### In-Memory Storage (for MVP)

- **Sessions**: Python dictionary `{session_id: SessionData}`
- **Recent queries**: LRU cache for query history (optional optimization)

### Future Considerations

- **Database**: PostgreSQL for persisting `BookPage`, `TextChunk`, `Query`, `ChatResponse` (if audit trail needed)
- **Cache**: Redis for session management in production
- **Versioning**: Track book content changes and trigger re-ingestion

---

## Validation and Constraints Summary

| Entity | Key Constraints |
|--------|-----------------|
| `BookPage` | URL must match book domain; cleaned_text >= 100 chars |
| `TextChunk` | Text must be 100-1000 tokens; chunk_index unique per page |
| `VectorEmbedding` | Vector length = 1024; all values in [-1, 1] |
| `ChunkMetadata` | All fields JSON-serializable; URL must be valid |
| `Query` | Query text <= 500 chars; embedding length = 1024 |
| `RetrievalResult` | Similarity score in [0, 1]; rank unique per query |
| `ChatResponse` | Answer non-empty; citations required if answer found |
| `Session` | Messages <= 10; inactive after 30 min |

---

## Next Steps

- Implement Pydantic models for all entities (type validation)
- Create database migrations (if persisting data beyond Qdrant)
- Define API contracts in `contracts/rag-api.yaml`
