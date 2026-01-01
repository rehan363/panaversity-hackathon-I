# RAG Integration Backend

Backend API service for the Physical AI Textbook RAG (Retrieval-Augmented Generation) system.

## Tech Stack

- **Framework**: FastAPI 0.115+
- **Vector Database**: Qdrant Cloud (free tier)
- **LLM**: Google Gemini 2.5 Flash via LiteLLM
- **Embeddings**: Google gemini-embedding-001 (768 dimensions)
- **Database**: Neon Serverless Postgres (for session storage)
- **Rate Limiting**: slowapi
- **Package Manager**: uv

## Prerequisites

- **Python**: 3.10 or higher
- **uv**: Install from [astral.sh/uv](https://astral.sh/uv)
- **Gemini API Key**: Get free API key at [Google AI Studio](https://makersuite.google.com/app/apikey)
- **Qdrant Account**: Sign up at [Qdrant Cloud](https://cloud.qdrant.io/)
- **Neon Account**: Sign up at [Neon](https://neon.tech/) (optional for Phase 2)

## Quick Start

### 1. Install Dependencies

```bash
# Install uv package manager (if not already installed)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install project dependencies
uv pip install -e .

# Install development dependencies (optional)
uv pip install -e ".[dev]"
```

### 2. Configure Environment

```bash
# Copy environment template
cp .env.example .env

# Edit .env and add your API keys
```

**Required environment variables**:
```bash
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here
```

### 3. Initialize Qdrant Collection

```bash
python scripts/setup_qdrant.py
```

Expected output:
```
✓ Connected to Qdrant at https://your-cluster.qdrant.io:6333
✓ Created collection 'physical-ai-textbook'
✓ Vector size: 768, Distance: Cosine
✓ Payload indexes created: week, module
```

### 4. Index Documentation

```bash
python scripts/index_docs.py --docs-path ../physical-ai-textbook/docs
```

Expected output:
```
Scanning documentation directory...
Found 21 Markdown files
Generated 142 chunks from 21 documents
✓ Indexing complete in 87.3 seconds
```

### 5. Start Development Server

```bash
uvicorn rag_backend.main:app --reload --port 8000
```

Server will start at:
- **API**: http://localhost:8000
- **Docs**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

### 6. Verify Health

```bash
curl http://localhost:8000/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "gemini_api_available": true,
  "indexed_chunks": 142,
  "last_indexed": "2026-01-01T12:00:00Z"
}
```

## API Endpoints

### Core Endpoints

- `POST /api/chat/query` - Submit RAG query (full-text or text-selection)
- `GET /api/health` - Health check for frontend graceful degradation

### Admin Endpoints

- `POST /admin/index` - Trigger documentation re-indexing (requires ADMIN_TOKEN)

See [contracts/chat-api.md](../specs/002-rag-integration/contracts/chat-api.md) for full API documentation.

## Testing

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=rag_backend --cov-report=html

# Run specific test suite
pytest tests/integration/test_rag_pipeline.py -v

# Run unit tests only
pytest tests/unit/ -v
```

## Project Structure

```
backend/
├── rag_backend/
│   ├── main.py                      # FastAPI app entry point
│   ├── config.py                    # Environment configuration
│   ├── models/
│   │   ├── chat.py                  # Chat request/response models
│   │   ├── chunk.py                 # TextChunk models
│   │   └── health.py                # Health check models
│   ├── services/
│   │   ├── embedding_service.py     # Gemini embeddings
│   │   ├── vector_store.py          # Qdrant client wrapper
│   │   ├── llm_service.py           # LiteLLM + Gemini integration
│   │   ├── rag_pipeline.py          # RAG orchestration
│   │   └── database_service.py      # Neon Postgres client
│   ├── routers/
│   │   ├── chat.py                  # Chat endpoints
│   │   ├── health.py                # Health endpoint
│   │   └── admin.py                 # Admin endpoints
│   └── utils/
│       ├── chunking.py              # Markdown chunking
│       ├── rate_limiter.py          # Rate limiting config
│       └── error_handlers.py        # Exception handlers
├── scripts/
│   ├── index_docs.py                # CLI tool for indexing
│   ├── setup_qdrant.py              # Qdrant initialization
│   └── setup_database.py            # Database migrations
├── tests/
│   ├── unit/                        # Unit tests
│   ├── integration/                 # Integration tests
│   └── e2e/                         # End-to-end tests
├── data/
│   └── index_metadata.json          # Indexing state
├── pyproject.toml                   # Project configuration
├── .env.example                     # Environment template
└── README.md                        # This file
```

## Development

### Code Quality

```bash
# Format code
black rag_backend/ scripts/ tests/

# Lint code
ruff check rag_backend/ scripts/ tests/

# Type checking (if using mypy)
mypy rag_backend/
```

### Adding New Dependencies

```bash
# Add runtime dependency
uv pip install package-name

# Add to pyproject.toml dependencies list
```

## Deployment

### Vercel

```bash
# Install Vercel CLI
npm install -g vercel

# Deploy
vercel --prod

# Configure environment variables in Vercel dashboard
```

### Railway

```bash
# Install Railway CLI
npm install -g @railway/cli

# Initialize and deploy
railway login
railway init
railway up
```

### Docker

```bash
# Build image
docker build -t rag-backend .

# Run container
docker run -p 8000:8000 --env-file .env rag-backend
```

## Environment Variables

### Required

| Variable | Description | Example |
|----------|-------------|---------|
| `GEMINI_API_KEY` | Google Gemini API key | `AIza...` |
| `QDRANT_URL` | Qdrant cluster URL | `https://xyz.qdrant.io:6333` |
| `QDRANT_API_KEY` | Qdrant API key | `abc123...` |

### Optional

| Variable | Default | Description |
|----------|---------|-------------|
| `NEON_DATABASE_URL` | None | Postgres connection string (Phase 2+) |
| `ADMIN_TOKEN` | None | Admin endpoint authentication token |
| `ENVIRONMENT` | `development` | Environment name |
| `LOG_LEVEL` | `INFO` | Logging level |
| `RATE_LIMIT_PER_MINUTE` | `3` | Rate limit per IP |
| `EMBEDDING_MODEL` | `gemini-embedding-001` | Embedding model name |
| `LLM_MODEL` | `gemini-2.5-flash` | LLM model name |
| `CHUNK_SIZE` | `512` | Token count per chunk |
| `CHUNK_OVERLAP` | `51` | Token overlap between chunks |

## Troubleshooting

### 401 Unauthorized from Gemini

**Solution**: Verify `GEMINI_API_KEY` is correct. Test directly:
```bash
curl "https://generativelanguage.googleapis.com/v1beta/models?key=YOUR_API_KEY"
```

### 429 Too Many Requests

**Solution**: Gemini free tier limit (5 RPM / 100 RPD). Wait 60 seconds or implement caching.

### Qdrant Connection Timeout

**Solution**:
1. Verify `QDRANT_URL` and `QDRANT_API_KEY`
2. Check Qdrant Cloud dashboard for cluster status
3. Ensure IP allowlist includes your server IP

### Module Not Found

**Solution**: Reinstall dependencies with `uv pip install -e .`

## Performance Targets

| Metric | Target |
|--------|--------|
| Health Check (p95) | <30ms |
| Query Response (p95) | <2.3s |
| Concurrent Users | 10-15 (Gemini free tier constraint) |
| Daily Queries | ~100 (Gemini free tier: 100 RPD) |

## Contributing

See [quickstart.md](../specs/002-rag-integration/quickstart.md) for full development workflow.

## License

See root project LICENSE file.

## Support

- **Issues**: [GitHub Issues](https://github.com/rehan363/panaversity-hackathon-I/issues)
- **Docs**: See `specs/002-rag-integration/` for full documentation
