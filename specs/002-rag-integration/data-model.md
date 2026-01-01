# Data Model: RAG Integration

**Feature**: RAG Integration for Physical AI Textbook
**Date**: 2025-12-30
**Status**: Phase 1 Complete

---

## 1. Text Chunks (Qdrant Vector Store)

### Purpose
Represents indexed segments of textbook content stored in Qdrant for vector similarity search.

### Schema

```python
from dataclasses import dataclass
from datetime import datetime
from typing import List

@dataclass
class TextChunk:
    id: str                          # UUID
    text: str                        # Chunk content (512-768 tokens)
    embedding: List[float]           # 768-dimensional vector
    metadata: ChunkMetadata

@dataclass
class ChunkMetadata:
    week: str                        # "Week 1", "Week 2", etc.
    module: str                      # "Module 1 - ROS2"
    section: str                     # "Foundations of Physical AI"
    file_path: str                   # "docs/module1-ros2/week1-foundations.md"
    header_hierarchy: List[str]      # ["Week 1", "What is Physical AI?"]
    chunk_index: int                 # Position within section (0-indexed)
    total_chunks_in_section: int     # Total chunks for this section
    tokens: int                      # Chunk token count
    created_at: datetime
```

### Indexes
- **Vector Index**: HNSW algorithm for similarity search (Qdrant default)
- **Payload Indexes**: `week`, `module` (for filtered search)

### Storage Estimate
- **Chunk Count**: ~130-150 chunks (13 weeks × 10 chunks/week)
- **Vector Size**: 768 floats × 4 bytes = 3.07 KB per vector
- **Total Storage**: ~150 chunks × 3.07 KB = ~460 KB (well within 1GB free tier)

### Example

```python
chunk = TextChunk(
    id="uuid-abc-123",
    text="DDS (Data Distribution Service) is a middleware protocol...",
    embedding=[0.023, -0.14, 0.087, ...],  # 768 dimensions
    metadata=ChunkMetadata(
        week="Week 3",
        module="Module 1 - ROS2",
        section="ROS 2 Architecture",
        file_path="docs/module1-ros2/week3-ros2-intro.md",
        header_hierarchy=["Week 3", "ROS 2 Architecture", "DDS Middleware"],
        chunk_index=1,                # Second chunk in this section (0-indexed)
        total_chunks_in_section=3,    # Section has 3 total chunks
        tokens=642,
        created_at=datetime.now()
    )
)
```

---

## 2. Chat Messages (Neon Postgres)

### Purpose
Represents user questions and AI responses in a conversation session. Stored in Neon Postgres for anonymous session persistence.

### Schema

```typescript
interface ChatMessage {
    id: string;                      // UUID
    role: 'user' | 'assistant';
    content: string;
    citations?: Citation[];
    timestamp: Date;
    query_type: 'full_text' | 'text_selection';
    context?: SelectedTextContext;
}

interface Citation {
    source: string;                  // "[Week 1, Section 2.3]"
    chunk_id: string;
    relevance_score: number;         // 0.0 - 1.0
}

interface SelectedTextContext {
    text: string;
    file_path: string;
    selection_range: { start: number; end: number };
}
```

### Storage
- **Location**: Neon Serverless Postgres
- **Table**: `session_messages` (see Query Sessions section for schema)
- **Persistence**: Persistent across sessions with anonymous session tokens
- **Size Limit**: No hard limit (Neon free tier supports sufficient storage)
- **Retention**: Messages retained indefinitely (Phase 4 can add cleanup policy)

### Example

```typescript
const userMessage: ChatMessage = {
    id: "msg-001",
    role: "user",
    content: "What is DDS middleware?",
    timestamp: new Date(),
    query_type: "full_text",
    context: null
};

const assistantMessage: ChatMessage = {
    id: "msg-002",
    role: "assistant",
    content: "DDS (Data Distribution Service) is a middleware protocol used in ROS 2...",
    citations: [
        {
            source: "[Week 3, ROS 2 Architecture]",
            chunk_id: "uuid-abc-123",
            relevance_score: 0.92
        }
    ],
    timestamp: new Date(),
    query_type: "full_text",
    context: null
};
```

---

## 3. Query Sessions (Optional - Future Phase)

### Purpose
Track user conversation sessions in database for persistent history. **Not used in Phase 2 MVP**.

### Schema (Neon Postgres)

```sql
CREATE TABLE query_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_token VARCHAR(255) UNIQUE NOT NULL,
    user_id UUID NULL,                    -- NULL for anonymous users
    created_at TIMESTAMP DEFAULT NOW(),
    last_activity TIMESTAMP DEFAULT NOW(),
    message_count INT DEFAULT 0
);

CREATE TABLE session_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES query_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    citations JSONB,
    query_type VARCHAR(50),
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_session_messages_session ON session_messages(session_id);
CREATE INDEX idx_query_sessions_token ON query_sessions(session_token);
CREATE INDEX idx_query_sessions_activity ON query_sessions(last_activity DESC);
```

### Usage
- **Phase 2 MVP**: Not implemented (session storage only)
- **Phase 4 Bonus**: Enable persistent history with Better-Auth integration
- **Rationale**: Reduces MVP complexity, aligns with Constitution Principle VII (core first)

---

## 4. Indexing Metadata (Application State)

### Purpose
Track the state of documentation indexing for re-indexing decisions and health checks.

### Schema

```python
from dataclasses import dataclass
from datetime import datetime

@dataclass
class IndexMetadata:
    collection_name: str = "physical-ai-textbook"
    total_chunks: int
    total_documents: int
    last_indexed: datetime
    embedding_model: str = "gemini-embedding-001"
    chunk_strategy: str = "markdown-header-split"
    chunk_size: int = 512
    chunk_overlap: int = 51
    version: str = "1.0.0"
```

### Storage
- **Location**: `backend/data/index_metadata.json`
- **Purpose**: Persist indexing state between server restarts
- **Update Trigger**: After successful indexing or re-indexing

### Example

```json
{
    "collection_name": "physical-ai-textbook",
    "total_chunks": 142,
    "total_documents": 21,
    "last_indexed": "2025-12-31T10:30:00Z",
    "embedding_model": "gemini-embedding-001",
    "chunk_strategy": "markdown-header-split",
    "chunk_size": 512,
    "chunk_overlap": 51,
    "version": "1.0.0"
}
```

---

## 5. API Request/Response Models

### Purpose
Define Pydantic models for API validation and documentation.

### Chat Query Request

```python
from pydantic import BaseModel, Field
from typing import Optional, Literal

class SelectedTextContext(BaseModel):
    text: str
    file_path: str
    selection_range: dict[str, int]  # {"start": 0, "end": 100}

class ChatQueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=500)
    query_type: Literal['full_text', 'text_selection'] = 'full_text'
    context: Optional[SelectedTextContext] = None
    session_id: Optional[str] = None      # For future use
    max_context_chunks: int = Field(default=5, ge=1, le=10)
```

### Chat Query Response

```python
from typing import List

class Citation(BaseModel):
    source: str
    chunk_id: str
    relevance_score: float = Field(ge=0.0, le=1.0)

class ChatQueryResponse(BaseModel):
    response: str
    citations: List[Citation]
    context_chunks_used: int
    processing_time_ms: int
    model_used: str = "gemini-2.5-flash"
```

### Health Check Response

```python
from datetime import datetime

class HealthCheckResponse(BaseModel):
    status: Literal['healthy', 'degraded', 'offline']
    qdrant_connected: bool
    gemini_api_available: bool
    indexed_chunks: int
    last_indexed: datetime
    warnings: Optional[List[str]] = None
```

### Error Response

```python
class ErrorResponse(BaseModel):
    error: str                           # Error code
    message: str                         # Human-readable message
    retry_after_seconds: Optional[int] = None
    fallback_suggestion: Optional[str] = None
```

---

## 6. Data Flow Diagram

```
User Query → Frontend (TypeScript)
    ↓
    | POST /api/chat/query
    ↓
Backend (FastAPI)
    ↓
    ├─→ Embedding Service (Google Gemini)
    |       ↓
    |   768-dim vector
    |       ↓
    ├─→ Vector Store (Qdrant)
    |       ↓
    |   Top 5 chunks with metadata
    |       ↓
    └─→ LLM Service (Gemini via LiteLLM)
            ↓
        Response with citations
            ↓
Frontend (Display + Session Storage)
```

---

## 7. Validation Rules

### Text Chunks
- `text`: Non-empty, 1-2000 characters
- `embedding`: Exactly 768 dimensions, all floats
- `week`: Must match pattern "Week [1-13]"
- `module`: Must match pattern "Module [1-4] - [Name]"

### Chat Messages
- `query`: 1-500 characters (enforced by rate limiting)
- `role`: Must be 'user' or 'assistant'
- `relevance_score`: 0.0 to 1.0

### API Requests
- `max_context_chunks`: 1-10 (prevents excessive context)
- `query_type`: Must be 'full_text' or 'text_selection'
- `context`: Required if query_type is 'text_selection'

---

## 8. Migration Strategy

### Phase 2 → Phase 4 Migration

When implementing persistent history in Phase 4:

1. **Database Setup**: Create Neon Postgres instance with schema from section 3
2. **Session Migration**: Offer users "Save this conversation?" prompt
3. **Backward Compatibility**: Keep sessionStorage as fallback if database unavailable
4. **Data Export**: Provide JSON export of conversation history

### Version Compatibility

```python
# backend/src/models/version.py
DATA_MODEL_VERSION = "1.0.0"

# Future versions:
# 1.1.0 - Add user_id to TextChunk metadata (Phase 4)
# 1.2.0 - Add conversation_topic classification
# 2.0.0 - Breaking: Change embedding dimensions (if model changes)
```

---

## Summary

This data model supports:
- ✅ Vector similarity search with Qdrant
- ✅ Session-based chat history (browser storage)
- ✅ Rich metadata for citations and filtering
- ✅ API validation with Pydantic
- ✅ Future extensibility for persistent storage

**Next**: Define API contracts in `contracts/chat-api.md`
