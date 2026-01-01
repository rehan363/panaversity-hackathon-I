# API Contract: Chat Endpoints

**Feature**: RAG Integration
**Version**: 1.0.0
**Base URL**:
- Development: `http://localhost:8000/api`
- Production: `https://your-backend.vercel.app/api`

---

## Authentication

**Phase 2**: No authentication required
**Phase 4**: Bearer token authentication for persistent history

---

## 1. POST /api/chat/query

### Purpose
Submit a full-text question to the RAG system.

### Request

```http
POST /api/chat/query
Content-Type: application/json

{
    "query": "What is DDS middleware?",
    "query_type": "full_text",
    "context": null,
    "max_context_chunks": 5
}
```

#### Request Schema

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `query` | string | Yes | User's question (1-500 chars) |
| `query_type` | enum | No | `"full_text"` or `"text_selection"` (default: `"full_text"`) |
| `context` | object | No | Selected text context (required if query_type is `"text_selection"`) |
| `max_context_chunks` | integer | No | Number of chunks to retrieve (1-10, default: 5) |

### Response (Success)

```http
HTTP/1.1 200 OK
Content-Type: application/json

{
    "response": "DDS (Data Distribution Service) is a middleware protocol used in ROS 2 for real-time, scalable, and reliable data distribution between nodes...",
    "citations": [
        {
            "source": "[Week 3, ROS 2 Architecture]",
            "chunk_id": "uuid-abc-123",
            "relevance_score": 0.92
        },
        {
            "source": "[Week 4, Nodes and Topics]",
            "chunk_id": "uuid-def-456",
            "relevance_score": 0.85
        }
    ],
    "context_chunks_used": 3,
    "processing_time_ms": 1847,
    "model_used": "gemini-2.5-flash"
}
```

#### Response Schema

| Field | Type | Description |
|-------|------|-------------|
| `response` | string | AI-generated answer with citations |
| `citations` | array | List of source citations |
| `context_chunks_used` | integer | Number of chunks retrieved from vector store |
| `processing_time_ms` | integer | Total processing time in milliseconds |
| `model_used` | string | LLM model identifier |

### Response (Rate Limited)

```http
HTTP/1.1 429 Too Many Requests
Content-Type: application/json
Retry-After: 60

{
    "error": "rate_limit_exceeded",
    "message": "Too many requests. Please try again in 60 seconds.",
    "retry_after_seconds": 60
}
```

### Response (Backend Error)

```http
HTTP/1.1 503 Service Unavailable
Content-Type: application/json

{
    "error": "service_unavailable",
    "message": "AI assistant temporarily offline. Browse content normally.",
    "fallback_suggestion": "Try searching the documentation directly."
}
```

### Response (Invalid Request)

```http
HTTP/1.1 400 Bad Request
Content-Type: application/json

{
    "error": "invalid_request",
    "message": "Query must be between 1 and 500 characters."
}
```

---

## 2. POST /api/chat/query-selection

### Purpose
Ask a question about selected text from the documentation.

### Request

```http
POST /api/chat/query-selection
Content-Type: application/json

{
    "query": "Explain this in simpler terms",
    "query_type": "text_selection",
    "context": {
        "text": "The DDS middleware implements a publish-subscribe pattern with Quality of Service (QoS) policies...",
        "file_path": "docs/module1-ros2/week3-ros2-intro.md",
        "selection_range": { "start": 1420, "end": 1680 }
    },
    "max_context_chunks": 3
}
```

#### Context Schema

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `text` | string | Yes | Selected text from documentation |
| `file_path` | string | Yes | Source file path |
| `selection_range` | object | Yes | Start and end character positions |

### Response

Same structure as `/api/chat/query`, but `citations` may include the selected text source:

```json
{
    "response": "In simpler terms, DDS is like a smart messaging system...",
    "citations": [
        {
            "source": "[Selected Text, Week 3]",
            "chunk_id": "selection-context",
            "relevance_score": 1.0
        },
        {
            "source": "[Week 3, ROS 2 Architecture]",
            "chunk_id": "uuid-abc-123",
            "relevance_score": 0.88
        }
    ],
    "context_chunks_used": 2,
    "processing_time_ms": 1523,
    "model_used": "gemini-2.5-flash"
}
```

---

## 3. GET /api/health

### Purpose
Check backend and dependencies status for frontend graceful degradation.

### Request

```http
GET /api/health
```

### Response (Healthy)

```http
HTTP/1.1 200 OK
Content-Type: application/json

{
    "status": "healthy",
    "qdrant_connected": true,
    "gemini_api_available": true,
    "indexed_chunks": 142,
    "last_indexed": "2025-12-31T10:30:00Z"
}
```

### Response (Degraded)

```http
HTTP/1.1 200 OK
Content-Type: application/json

{
    "status": "degraded",
    "qdrant_connected": true,
    "gemini_api_available": false,
    "indexed_chunks": 142,
    "last_indexed": "2025-12-31T10:30:00Z",
    "warnings": ["Gemini API rate limit reached", "Falling back to cached responses"]
}
```

### Response (Offline)

```http
HTTP/1.1 503 Service Unavailable
Content-Type: application/json

{
    "status": "offline",
    "qdrant_connected": false,
    "gemini_api_available": false,
    "indexed_chunks": 0,
    "last_indexed": null,
    "warnings": ["Vector database unreachable", "Service temporarily unavailable"]
}
```

#### Response Schema

| Field | Type | Description |
|-------|------|-------------|
| `status` | enum | `"healthy"`, `"degraded"`, or `"offline"` |
| `qdrant_connected` | boolean | Vector database connectivity status |
| `gemini_api_available` | boolean | LLM API availability status |
| `indexed_chunks` | integer | Total indexed chunks in vector store |
| `last_indexed` | datetime | ISO 8601 timestamp of last indexing |
| `warnings` | array | Optional list of warning messages |

---

## 4. POST /admin/index (Admin Only)

### Purpose
Trigger re-indexing of documentation content. Protected endpoint for admin use.

### Request

```http
POST /admin/index
Content-Type: application/json
Authorization: Bearer <admin-token>

{
    "force_reindex": false,
    "docs_path": "physical-ai-textbook/docs"
}
```

#### Request Schema

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `force_reindex` | boolean | No | Force re-index even if content unchanged (default: false) |
| `docs_path` | string | No | Path to documentation directory (default: configured path) |

### Response (Accepted)

```http
HTTP/1.1 202 Accepted
Content-Type: application/json

{
    "status": "indexing_started",
    "job_id": "uuid-job-789",
    "estimated_time_seconds": 120
}
```

### Response (Unauthorized)

```http
HTTP/1.1 401 Unauthorized
Content-Type: application/json

{
    "error": "unauthorized",
    "message": "Admin authentication required"
}
```

---

## Error Codes

| HTTP Status | Error Code | Description |
|-------------|-----------|-------------|
| 400 | `invalid_request` | Malformed request body or invalid parameters |
| 401 | `unauthorized` | Authentication required or invalid token |
| 429 | `rate_limit_exceeded` | Too many requests (retry after N seconds) |
| 500 | `internal_error` | Unexpected server error |
| 503 | `service_unavailable` | Backend dependencies offline (Qdrant, Gemini) |

---

## Rate Limits

| Tier | Limit | Scope |
|------|-------|-------|
| **Free (Phase 2)** | 3 requests/minute | Per IP address |
| **Authenticated (Phase 4)** | 10 requests/minute | Per user account |
| **Global** | 5 requests/minute | Gemini API constraint (shared across all users) |

### Rate Limit Headers

```http
X-RateLimit-Limit: 3
X-RateLimit-Remaining: 2
X-RateLimit-Reset: 1735651200
```

---

## CORS Configuration

### Allowed Origins

```python
allow_origins = [
    "http://localhost:3000",                        # Docusaurus dev server
    "https://rehan363.github.io",                   # GitHub Pages production
    "https://panaversity-hackathon-i.vercel.app"    # Vercel preview
]
```

### Allowed Methods

```python
allow_methods = ["GET", "POST"]
```

### Allowed Headers

```python
allow_headers = ["Content-Type", "Authorization", "X-Request-ID"]
```

---

## Request Flow Diagram

```
┌─────────────────┐
│  Frontend (JS)  │
└────────┬────────┘
         │ POST /api/chat/query
         │ { "query": "What is DDS?" }
         ↓
┌─────────────────┐
│  Rate Limiter   │ ← Check: 3 req/min limit
└────────┬────────┘
         │ (if passed)
         ↓
┌─────────────────┐
│  Embedding Svc  │ ← Generate query embedding
└────────┬────────┘
         │ [768-dim vector]
         ↓
┌─────────────────┐
│  Qdrant Search  │ ← Retrieve top 5 chunks
└────────┬────────┘
         │ [chunks with metadata]
         ↓
┌─────────────────┐
│  LLM Service    │ ← Gemini via LiteLLM
└────────┬────────┘
         │ { "response": "...", "citations": [...] }
         ↓
┌─────────────────┐
│  Frontend (JS)  │ ← Display response + citations
└─────────────────┘
```

---

## Security Considerations

1. **API Key Protection**: Never expose `GEMINI_API_KEY` or `QDRANT_API_KEY` in responses
2. **Input Validation**: Sanitize queries to prevent injection attacks
3. **Rate Limiting**: Protect against DoS and API quota exhaustion
4. **CORS**: Restrict origins to prevent unauthorized access
5. **Error Messages**: Avoid leaking implementation details in error responses

---

## Testing

### Example cURL Commands

**Health Check**:
```bash
curl http://localhost:8000/api/health
```

**Full-Text Query**:
```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?", "max_context_chunks": 5}'
```

**Text Selection Query**:
```bash
curl -X POST http://localhost:8000/api/chat/query-selection \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this in simpler terms",
    "query_type": "text_selection",
    "context": {
      "text": "The DDS middleware...",
      "file_path": "docs/module1-ros2/week3-ros2-intro.md",
      "selection_range": {"start": 100, "end": 200}
    }
  }'
```

---

## Versioning

- **Current Version**: 1.0.0
- **Versioning Strategy**: Semantic versioning (MAJOR.MINOR.PATCH)
- **Backward Compatibility**: Breaking changes require new API version (e.g., `/api/v2/chat/query`)

---

## OpenAPI Schema

Full OpenAPI 3.0 schema available at: `http://localhost:8000/docs` (FastAPI auto-generated)

Interactive API testing available at: `http://localhost:8000/redoc`

---

## Summary

This API contract defines:
- ✅ 2 core endpoints (chat query + text selection)
- ✅ 1 health check endpoint
- ✅ 1 admin endpoint (indexing)
- ✅ Comprehensive error handling
- ✅ Rate limiting strategy
- ✅ CORS configuration
- ✅ Security best practices

**Next**: Create `quickstart.md` for setup instructions.
