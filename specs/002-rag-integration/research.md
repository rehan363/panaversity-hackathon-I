# Research Findings: RAG Integration

**Date**: 2025-12-30
**Feature**: RAG Integration for Physical AI Textbook
**Purpose**: Resolve technical unknowns and establish implementation approach

---

## 1. OpenAI Agents SDK Compatibility with Gemini API

### Decision
Use **LiteLLM as middleware** to integrate Gemini API with OpenAI Agents SDK.

### Rationale
- **Direct Integration**: Gemini provides OpenAI-compatible endpoints at `https://generativelanguage.googleapis.com/v1beta/openai/`, but with limited compatibility
- **LiteLLM Advantage**: Provides robust error handling, supports 350+ LLM providers, and allows easy switching between models
- **Implementation**: `gemini_agent = Agent(model="litellm/gemini/gemini-2.5-flash-preview-04-17")`
- **Fallback**: If LiteLLM adds complexity, can use direct AsyncOpenAI client with custom base_url

### Alternatives Considered
1. **Direct Gemini API calls** - Would require custom agent orchestration logic
2. **LangChain**: - Heavier dependency, overkill for this use case
3. **Custom Agent Framework** - Too much development time for hackathon

### Sources
- [OpenAI compatibility - Gemini API](https://ai.google.dev/gemini-api/docs/openai)
- [Use OpenAI Agents SDK with Gemini](https://blog.langdb.ai/integrate-gemini-claude-deepseek-into-agents-sdk-by-openai)
- [Build Agents with OpenAI SDK using any LLM Provider](https://medium.com/@amri369/build-agents-with-openai-sdk-using-any-llm-provider-claude-deepseek-perplexity-gemini-5c80185b3cc2)

---

## 2. Free Embedding Model Selection

### Decision
Use **Google gemini-embedding-001** (transitioning from text-embedding-004).

### Rationale
- **Cost**: Completely free with 1500 RPM rate limit
- **Quality**: Better performance than Cohere models in 2025 benchmarks
- **Dimensions**: 768 (optimal balance of quality and storage)
- **Token Limit**: Supports up to 2048 tokens per chunk (vs 1024 for text-embedding-004)
- **Longevity**: text-embedding-004 deprecating August 2025; gemini-embedding-001 is current recommended model

### Alternatives Considered

| Model | Dimensions | Cost | Performance | Rate Limits | Status |
|-------|-----------|------|-------------|-------------|--------|
| **Google gemini-embedding-001** | 768 | Free | Better | 1500 RPM | ✅ Recommended |
| Google text-embedding-004 | 768 | Free | Modest | 1500 RPM | Deprecating Aug 2025 |
| Cohere embed-v3.0 | 1024 | Free tier | Underperforming | Lower | Not recommended |
| Cohere embed-light-v3.0 | 384 | Free tier | Lower quality | Lower | Not recommended |

### Implementation
```python
from google.generativeai import embed_content

embedding = embed_content(
    model="models/gemini-embedding-001",
    content=chunk_text,
    task_type="retrieval_document"
)
```

### Sources
- [Choosing The Right Embedding Model](https://ragwalla.com/docs/guides/choosing-an-embedding-model-for-your-workload)
- [Best Embedding Models for Information Retrieval in 2025](https://dev.to/datastax/the-best-embedding-models-for-information-retrieval-in-2025-3dp5)
- [Embedding Models: OpenAI vs Gemini vs Cohere](https://research.aimultiple.com/embedding-models/)

---

## 3. Chunking Strategy for Markdown/MDX Content

### Decision
Use **Markdown-aware hierarchical chunking** with LangChain's MarkdownHeaderTextSplitter.

### Optimal Parameters
- **Chunk Size**: 512-768 tokens (optimized for educational content)
- **Overlap**: 10-20% (51-154 tokens overlap for context continuity)
- **Strategy**: Split by headers first, then recursively within sections

### Rationale
- **Structure Preservation**: Maintains document hierarchy (Week → Module → Section)
- **Context Integrity**: Keeps related content together (better than fixed-size splits)
- **Performance**: 5-10% accuracy improvement over naive splitting with minimal overhead
- **Metadata Enrichment**: Each chunk retains header hierarchy for better citations

### Implementation Approach

1. **Primary Splitter**: `MarkdownHeaderTextSplitter`
   - Split by headers (#, ##, ###)
   - Preserves semantic sections

2. **Secondary Splitter**: `RecursiveCharacterTextSplitter`
   - For content within sections
   - Hierarchical separators: `["\n\n", "\n", " ", ""]`

3. **Metadata Structure**:
   ```python
   {
       "text": "chunk_content",
       "week": "Week 1",
       "module": "Module 1 - ROS2",
       "section": "Foundations of Physical AI",
       "file_path": "docs/module1-ros2/week1-foundations.md",
       "header_hierarchy": ["Week 1", "What is Physical AI?"],
       "tokens": 512
   }
   ```

### Alternatives Considered
- **Fixed-Size Chunking**: Simpler but breaks semantic boundaries
- **Semantic Chunking**: 70% improvement but too computationally expensive for hackathon
- **Sentence-Based**: Too granular, loses context

### Sources
- [Best Document Chunking Strategy for RAG (2025)](https://langcopilot.com/posts/2025-10-11-document-chunking-for-rag-practical-guide)
- [RAG Chunking Strategies 2025](https://app.ailog.fr/en/blog/guides/chunking-strategies)
- [Best Chunking Strategies for RAG in 2025](https://www.firecrawl.dev/blog/best-chunking-strategies-rag-2025)

---

## 4. Qdrant Configuration for Educational Content

### Decision
Configure Qdrant collection with indexed payload fields and cosine similarity.

### Collection Configuration

```python
{
    "collection_name": "physical-ai-textbook",
    "vectors": {
        "size": 768,              # matches gemini-embedding-001
        "distance": "Cosine"      # standard for text similarity
    },
    "payload_schema": {
        "week": "keyword",        # indexed for filtering
        "module": "keyword",      # indexed for filtering
        "section": "text",        # full-text search
        "file_path": "keyword",
        "header_hierarchy": "keyword[]",
        "tokens": "integer"
    },
    "optimizers_config": {
        "indexing_threshold": 10000  # suitable for ~200 chunks
    }
}
```

### Rationale
- **Payload Indexes**: Fast filtering by week/module (e.g., "Find content about ROS2 in Module 1")
- **Hybrid Search**: Combines vector similarity with metadata filters
- **Storage**: Free tier (1GB) sufficient for ~13 weeks × 10 chunks/week = ~130-150 chunks
- **Performance**: Cosine similarity is standard for text embeddings

### Sources
- [Qdrant - High-performance Vector Database](https://github.com/qdrant/qdrant)
- [Developer's Guide to Qdrant](https://www.cohorte.co/blog/a-developers-friendly-guide-to-qdrant-vector-database)
- [Best Vector Databases in 2025](https://www.firecrawl.dev/blog/best-vector-databases-2025)

---

## 5. Rate Limiting Strategies

### Critical Constraint
**Gemini Free Tier**: 5 RPM / 100 RPD (as of December 2025)

### Decision
Implement **multi-layer rate limiting** with client-side debouncing, backend limits, and caching.

### Strategy Layers

1. **Client-Side (Frontend)**:
   - Debounce query input (500ms delay)
   - Disable submit button during processing
   - Local limit: 3 requests/minute per browser session

2. **Backend (FastAPI)**:
   ```python
   from slowapi import Limiter
   limiter = Limiter(key_func=get_remote_address)

   @app.post("/api/chat/query")
   @limiter.limit("3/minute")  # More restrictive than Gemini's 5 RPM
   async def chat_query(request: Request, query: ChatQuery):
       # Implementation
   ```

3. **Application-Level**:
   - Exponential backoff on 429 errors
   - Request queuing with max size (10)
   - Response caching with LRU (maxsize=100)

4. **Graceful Degradation**:
   - Display: "AI assistant temporarily at capacity. Please try again in 60 seconds."
   - Show retry countdown timer
   - Allow content browsing without chat

### Rationale
- **Prevents API Abuse**: Protects free tier quota
- **User Experience**: Clear messaging when rate limited
- **Caching**: Reduces duplicate queries for common questions
- **Fail-Safe**: System remains usable when chat unavailable (Constitution Principle IX)

### Sources
- [Gemini API Rate Limits Guide 2025](https://www.aifreeapi.com/en/posts/gemini-api-rate-limit)
- [Gemini API Free Tier Limits](https://blog.laozhang.ai/api-guides/gemini-api-free-tier/)
- [Gemini has slashed free API limits](https://www.howtogeek.com/gemini-slashed-free-api-limits-what-to-use-instead/)

---

## 6. Docusaurus Chat Component Integration

### Decision
Build **custom React component** with global state management.

### Rationale
- **Full Control**: Custom styling, behavior, and integration
- **Lightweight**: No heavyweight plugin dependencies
- **Progressive Enhancement**: Works with existing Docusaurus setup
- **Constitution Compliance**: Follows modular architecture principles

### Implementation Approach

1. **Component Structure**:
   ```
   src/components/RAGChatbot/
   ├── index.tsx           # Main chat widget
   ├── ChatModal.tsx       # Modal dialog
   ├── MessageList.tsx     # Chat history display
   ├── QueryInput.tsx      # User input field
   ├── Citation.tsx        # Source citation display
   ├── styles.module.css   # Component styles
   └── types.ts            # TypeScript interfaces
   ```

2. **Integration Points**:
   - Swizzle Root component to add global chat widget
   - Use React Context for text selection state
   - Configure API proxy in `docusaurus.config.ts`

3. **API Proxy Configuration**:
   ```typescript
   proxy: {
     '/api': {
       target: 'http://localhost:8000',
       changeOrigin: true
     }
   }
   ```

### Alternatives Considered
- **docusaurus-plugin-chat-page**: Too heavyweight, requires full page route
- **EnhanceDocs**: Open-source but requires backend rewrite
- **Third-party Widget**: Lacks customization, potential vendor lock-in

### Sources
- [Introducing docusaurus-plugin-chat-page](https://dev.to/nichnarmada/introducing-docusaurus-plugin-chat-page-an-ai-powered-chat-interface-for-your-documentation-3ed4)
- [Unlock the Power of Docusaurus with AI](https://dev.to/ahelmy/unlock-the-power-of-docusaurus-with-ai-3clk)
- [Upstash RAG Chat Component](https://github.com/upstash/rag-chat-component)

---

## Summary

All technical unknowns have been resolved with concrete implementation decisions:

1. ✅ **OpenAI Agents SDK + Gemini**: Use LiteLLM middleware
2. ✅ **Embedding Model**: Google gemini-embedding-001 (free, 768-dim)
3. ✅ **Chunking**: Markdown-aware hierarchical (512-768 tokens, 10-20% overlap)
4. ✅ **Qdrant**: Cosine similarity, indexed payloads for week/module
5. ✅ **Rate Limiting**: Multi-layer (client + backend + caching)
6. ✅ **Frontend**: Custom React component with swizzled Root

**Next Phase**: Proceed to data model design and API contract definition.
