---
name: rag-technical-standards
description: Defines the architectural and implementation standards for the textbook's RAG (Retrieval-Augmented Generation) system.
allowed-tools: Read, Grep, Glob
---

# RAG Technical Standards

This skill ensures the AI chatbot follows the "Interactive Learning Experience" principles in the Constitution (Principle II & IV).

## 1. Tech Stack
- **Backend API**: FastAPI (Python 3.10+).
- **Vector Database**: Qdrant (Cloud Free Tier).
- **Relational Database**: Neon Postgres (Serverless).
- **AI SDK**: OpenAI Agents/ChatKit.

## 2. Data Flow
- **Ingestion**: Markdown files from `/docs` -> Chunks -> OpenAI Embeddings -> Qdrant.
- **Retrieval**: User query -> Embedding -> Vector Search -> Top Context -> LLM Response.
- **Citations**: Responses MUST include references in the format: `[Source: Week X, Section Y]`.

## 3. Security
- **API Keys**: No hardcoding. Use `.env` files.
- **Rate Limiting**: Prevent bot abuse on the chat endpoint.
- **Graceful Degradation**: If the backend is down, display: "AI assistant temporarily offline. Browse content normally." (Constitution Principle IX).
