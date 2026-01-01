# RAG Integration Quickstart Guide

**Feature**: RAG Integration for Physical AI Textbook
**Last Updated**: 2025-12-30

---

## Prerequisites

### Backend Requirements
- **Python**: 3.10 or higher
- **Package Manager**: pip or uv
- **Gemini API Key**: Get free API key at [Google AI Studio](https://makersuite.google.com/app/apikey)
- **Qdrant Account**: Sign up at [Qdrant Cloud](https://cloud.qdrant.io/)

### Frontend Requirements
- **Node.js**: 20.0 or higher
- **Package Manager**: npm or yarn

---

## Part 1: Backend Setup

### Step 1: Create Qdrant Collection

```bash
# Navigate to backend directory
cd backend

# Install dependencies
pip install -r requirements.txt

# Copy environment template
cp .env.example .env
```

**Edit `.env` file**:
```bash
# Required API Keys
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here

# Optional Configuration
LOG_LEVEL=INFO
RATE_LIMIT_PER_MINUTE=3
MAX_CONTEXT_CHUNKS=5
CHUNK_SIZE=512
CHUNK_OVERLAP=51
```

**Initialize Qdrant collection**:
```bash
python scripts/setup_qdrant.py

# Expected output:
# ✓ Connected to Qdrant at https://your-cluster.qdrant.io:6333
# ✓ Created collection 'physical-ai-textbook'
# ✓ Vector size: 768, Distance: Cosine
# ✓ Payload indexes created: week, module
```

---

### Step 2: Index Documentation

```bash
# Run indexing script
python scripts/index_docs.py \
    --docs-path ../physical-ai-textbook/docs \
    --collection physical-ai-textbook

# Expected output:
# Scanning documentation directory...
# Found 21 Markdown files
#
# Chunking strategy: markdown-header-split
# Chunk size: 512-768 tokens, overlap: 10%
# Generated 142 chunks from 21 documents
#
# Generating embeddings with gemini-embedding-001...
# Progress: [████████████████████] 142/142 (100%)
#
# Uploading to Qdrant collection 'physical-ai-textbook'...
# Progress: [████████████████████] 142/142 (100%)
#
# ✓ Indexing complete in 87.3 seconds
# ✓ Metadata saved to backend/data/index_metadata.json
```

---

### Step 3: Start Backend Server

**Development Mode**:
```bash
uvicorn src.main:app --reload --port 8000

# Server starts at: http://localhost:8000
# API docs at: http://localhost:8000/docs
# ReDoc at: http://localhost:8000/redoc
```

**Production Mode**:
```bash
uvicorn src.main:app --host 0.0.0.0 --port 8000 --workers 4
```

---

### Step 4: Verify Backend Health

```bash
curl http://localhost:8000/api/health

# Expected response:
# {
#   "status": "healthy",
#   "qdrant_connected": true,
#   "gemini_api_available": true,
#   "indexed_chunks": 142,
#   "last_indexed": "2025-12-31T10:30:00Z"
# }
```

**Test Chat Query**:
```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?", "max_context_chunks": 5}'

# Expected response:
# {
#   "response": "Physical AI refers to artificial intelligence systems...",
#   "citations": [
#     {
#       "source": "[Week 1, Foundations of Physical AI]",
#       "chunk_id": "uuid-...",
#       "relevance_score": 0.94
#     }
#   ],
#   "context_chunks_used": 3,
#   "processing_time_ms": 1645,
#   "model_used": "gemini-2.5-flash"
# }
```

---

## Part 2: Frontend Setup

### Step 1: Install Dependencies

```bash
cd physical-ai-textbook

# Install packages (no new dependencies needed)
npm install

# Existing packages from Docusaurus setup:
# - react@19.0.0
# - typescript@5.6.2
# - @docusaurus/core@3.9.2
```

---

### Step 2: Configure API Proxy

Verify `docusaurus.config.ts` includes proxy configuration:

```typescript
// docusaurus.config.ts
import type {Config} from '@docusaurus/types';

const config: Config = {
  // ... existing config

  // Add API proxy for development
  proxy: {
    '/api': {
      target: 'http://localhost:8000',
      changeOrigin: true,
      pathRewrite: {'^/api': '/api'}
    }
  },

  // ... rest of config
};

export default config;
```

---

### Step 3: Start Development Server

```bash
npm start

# Docusaurus dev server starts at http://localhost:3000
# Chat widget should appear as floating button in bottom-right
```

---

### Step 4: Test Chat Widget

1. Open browser at `http://localhost:3000`
2. Navigate to any documentation page (e.g., Week 1)
3. Click the floating chat button (💬 icon in bottom-right)
4. Type a question: **"What is Physical AI?"**
5. Verify response includes citation: **"[Source: Week 1, Foundations]"**
6. Try text selection:
   - Highlight a paragraph
   - Click "Ask AI about this" button
   - Ask: **"Explain this in simpler terms"**

---

## Part 3: Testing

### Backend Tests

```bash
cd backend

# Run all tests
pytest

# Run with coverage
pytest --cov=src --cov-report=html

# Run specific test suite
pytest tests/integration/test_rag_pipeline.py -v

# Run unit tests only
pytest tests/unit/ -v
```

**Expected Output**:
```
================================ test session starts ================================
collected 42 items

tests/unit/test_chunking.py ............                                    [ 28%]
tests/unit/test_embedding.py .......                                        [ 45%]
tests/unit/test_vector_store.py .....                                       [ 57%]
tests/integration/test_rag_pipeline.py ..........                           [ 80%]
tests/integration/test_api_endpoints.py ........                            [100%]

================================ 42 passed in 12.34s ================================
```

---

### Frontend Tests

```bash
cd physical-ai-textbook

# Run component tests
npm test

# Run E2E tests (requires backend running)
npm run test:e2e
```

---

## Part 4: Deployment

### Backend Deployment (Vercel)

```bash
cd backend

# Install Vercel CLI
npm install -g vercel

# Login to Vercel
vercel login

# Deploy
vercel --prod

# Set environment variables in Vercel dashboard:
# 1. Go to https://vercel.com/dashboard
# 2. Select your project
# 3. Settings → Environment Variables
# 4. Add:
#    - GEMINI_API_KEY
#    - QDRANT_URL
#    - QDRANT_API_KEY
```

**Alternative: Railway**
```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Initialize project
railway init

# Deploy
railway up

# Add environment variables via Railway dashboard
```

---

### Frontend Deployment (GitHub Pages)

```bash
cd physical-ai-textbook

# Update API proxy to production backend
# In docusaurus.config.ts, change:
proxy: {
  '/api': {
    target: 'https://your-backend.vercel.app',  # ← Production URL
    changeOrigin: true
  }
}

# Build
npm run build

# Deploy to GitHub Pages
npm run deploy

# Site will be live at:
# https://rehan363.github.io/panaversity-hackathon-I/
```

---

## Troubleshooting

### Backend Issues

**Problem**: `ModuleNotFoundError: No module named 'qdrant_client'`

**Solution**:
```bash
pip install -r requirements.txt
```

---

**Problem**: `401 Unauthorized` when calling Gemini API

**Solution**:
1. Verify `GEMINI_API_KEY` in `.env` is correct
2. Test API key directly:
   ```bash
   curl "https://generativelanguage.googleapis.com/v1beta/models?key=YOUR_API_KEY"
   ```
3. Regenerate API key at [Google AI Studio](https://makersuite.google.com/app/apikey)

---

**Problem**: `429 Too Many Requests` from Gemini

**Solution**:
- **Cause**: Gemini free tier limit (5 RPM / 100 RPD)
- **Fix**: Wait 60 seconds before retrying
- **Long-term**: Implement response caching or upgrade to paid tier

---

**Problem**: Qdrant connection timeout

**Solution**:
1. Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
2. Check Qdrant Cloud dashboard for cluster status
3. Ensure IP allowlist includes your server IP (if configured)

---

### Frontend Issues

**Problem**: Chat widget not appearing

**Solution**:
1. Verify Root.tsx is swizzled:
   ```bash
   npm run swizzle @docusaurus/theme-classic Root -- --wrap
   ```
2. Ensure `<RAGChatbot />` is imported and rendered in Root.tsx

---

**Problem**: API calls failing with CORS error

**Solution**:
1. Check backend CORS configuration includes frontend origin
2. Verify `docusaurus.config.ts` proxy is configured correctly
3. For production, ensure Vercel backend allows GitHub Pages origin

---

**Problem**: Chat responses not showing citations

**Solution**:
1. Verify backend returns `citations` array in response
2. Check `Citation.tsx` component is rendering correctly
3. Test backend directly with cURL to isolate issue

---

## Environment Variables Reference

### Backend (`.env`)

```bash
# ===== Required =====
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here

# ===== Optional =====
LOG_LEVEL=INFO                    # DEBUG, INFO, WARNING, ERROR
RATE_LIMIT_PER_MINUTE=3           # Max requests per minute per IP
MAX_CONTEXT_CHUNKS=5              # Max chunks retrieved per query
CHUNK_SIZE=512                    # Token count per chunk
CHUNK_OVERLAP=51                  # Token overlap between chunks
EMBEDDING_MODEL=gemini-embedding-001
LLM_MODEL=gemini-2.5-flash

# ===== Admin (Optional) =====
ADMIN_API_KEY=your_secure_admin_key  # For /admin/index endpoint
```

---

### Frontend (No `.env` needed for Phase 2)

API URL is configured via proxy in `docusaurus.config.ts`:
- **Development**: Proxied to `http://localhost:8000`
- **Production**: Proxied to `https://your-backend.vercel.app`

---

## Performance Benchmarks

### Expected Latency (Development)

| Operation | p50 | p95 | p99 |
|-----------|-----|-----|-----|
| Health Check | 15ms | 30ms | 50ms |
| Query Embedding | 200ms | 350ms | 500ms |
| Vector Search | 50ms | 100ms | 150ms |
| LLM Generation | 1200ms | 1800ms | 2500ms |
| **Total Query** | **1500ms** | **2300ms** | **3200ms** |

### Expected Throughput

- **Concurrent Users**: 50 (per Constitution requirement)
- **Requests per Second**: ~1 RPS (limited by Gemini free tier)
- **Daily Queries**: ~100 (Gemini free tier: 100 RPD)

---

## Next Steps

After successful setup:

1. ✅ Backend running and healthy
2. ✅ Documentation indexed in Qdrant (142 chunks)
3. ✅ Frontend chat widget functional
4. → **Run `/sp.tasks`** to generate implementation tasks
5. → **Run `/sp.implement`** to execute tasks
6. → Test with sample queries from spec acceptance scenarios
7. → Deploy to staging environment
8. → Run Constitution compliance checklist
9. → Deploy to production
10. → Create PHR documenting deployment

---

## Support

- **GitHub Issues**: [panaversity-hackathon-I/issues](https://github.com/rehan363/panaversity-hackathon-I/issues)
- **Gemini API Docs**: https://ai.google.dev/gemini-api/docs
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Docusaurus Docs**: https://docusaurus.io/docs

---

## Appendix: Directory Structure

```
backend/
├── src/
│   ├── main.py                      # FastAPI app entry point
│   ├── config.py                    # Environment configuration
│   ├── models/                      # Pydantic models
│   ├── services/                    # Business logic
│   ├── routers/                     # API endpoints
│   └── utils/                       # Helper functions
├── scripts/
│   ├── index_docs.py                # Documentation indexing
│   └── setup_qdrant.py              # Qdrant initialization
├── tests/
├── data/
│   └── index_metadata.json
├── requirements.txt
├── .env.example
└── README.md

physical-ai-textbook/
├── src/
│   ├── components/
│   │   └── RAGChatbot/              # Chat widget
│   ├── hooks/                       # Custom React hooks
│   └── theme/
│       └── Root.tsx                 # Swizzled root component
├── docs/                            # Textbook content
├── docusaurus.config.ts
└── package.json
```

---

**Quickstart guide complete!** Proceed to implementation with `/sp.tasks`.
