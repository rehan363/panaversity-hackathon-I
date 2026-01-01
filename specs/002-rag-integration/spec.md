# Feature Specification: RAG Integration for Physical AI Textbook

**Feature Branch**: `002-rag-integration`
**Created**: 2025-12-30
**Status**: Draft
**Input**: User description: "we will be using openai agents sdk with free gemini api key and we will also use free embedding model"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Textbook Content (Priority: P1)

A student is reading Week 3 on ROS 2 architecture and encounters an unfamiliar term like "DDS middleware." They want immediate clarification without leaving the page or searching external resources.

**Why this priority**: This is the core value proposition of the RAG system. Without this, the feature has no purpose. It directly addresses the Constitution's Principle II requirement for an "Interactive Learning Experience."

**Independent Test**: Can be fully tested by entering a question in the chat interface and verifying that the response includes relevant content from the textbook with proper citations.

**Acceptance Scenarios**:

1. **Given** a student is viewing any chapter, **When** they type a question like "What is DDS middleware?" in the chat interface, **Then** the system returns a relevant answer extracted from the textbook content with a citation (e.g., "[Source: Week 3, ROS 2 Architecture]").
2. **Given** a student asks a question about a topic not covered in the textbook, **When** the query is processed, **Then** the system responds with "I couldn't find information about that in the textbook" rather than hallucinating an answer.
3. **Given** a student asks a vague question like "Tell me about robots," **When** the query is processed, **Then** the system provides a summary of relevant chapters and suggests more specific questions.

---

### User Story 2 - Get Context-Specific Help from Selected Text (Priority: P2)

A student highlights a complex paragraph in Week 10 about Nav2 path planning algorithms and wants the AI to explain it in simpler terms or provide additional context.

**Why this priority**: This enhances the learning experience by allowing students to get targeted help on difficult passages. It's a "bonus" feature that significantly improves usability but isn't required for the MVP.

**Independent Test**: Can be tested independently by selecting text in a chapter, clicking "Ask AI about this," and verifying that the response is contextually relevant to the selected passage.

**Acceptance Scenarios**:

1. **Given** a student has selected text in a chapter, **When** they click the "Ask AI about this selection" button, **Then** the system uses the selected text as context for the query and provides a relevant explanation.
2. **Given** a student selects text and asks "Explain this in simpler terms," **When** the query is processed, **Then** the system rephrases the content at a more accessible reading level.

---

### User Story 3 - Browse Conversation History (Priority: P3)

A student has asked multiple questions during a study session and wants to review their previous questions and answers without re-asking.

**Why this priority**: This is a quality-of-life feature that improves user experience but is not essential for the core functionality. It can be added later if time permits.

**Independent Test**: Can be tested by asking multiple questions, then scrolling up in the chat interface to verify that all previous exchanges are visible.

**Acceptance Scenarios**:

1. **Given** a student has asked 5 questions, **When** they scroll up in the chat interface, **Then** all previous questions and answers are displayed in chronological order.
2. **Given** a student refreshes the page, **When** they return to the same chapter, **Then** their conversation history is restored (if using session storage).

---

### Edge Cases

- What happens when the user asks a question while the backend is offline or experiencing high latency?
- How does the system handle extremely long queries (>500 words)?
- What happens if the user asks the same question multiple times in rapid succession?
- How does the system handle queries in languages other than English?
- What happens if the embedding model fails to generate embeddings for a new chapter?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a chat interface embedded in the Docusaurus site where users can ask questions.
- **FR-002**: System MUST use a free embedding model (e.g., Google's text-embedding-004 or Cohere's free tier) to generate vector embeddings from textbook content.
- **FR-003**: System MUST store vector embeddings in Qdrant vector database for similarity search.
- **FR-004**: System MUST use OpenAI Agents SDK to orchestrate the RAG pipeline (retrieval + generation).
- **FR-005**: System MUST use Gemini API (with free API key) as the language model for generating responses.
- **FR-006**: System MUST retrieve the top 3-5 most relevant text chunks from the vector database based on query similarity.
- **FR-007**: System MUST include source citations in responses with chunk position (e.g., "[Source: Week 2, Section 2.3, Part 1 of 2]").
- **FR-008**: System MUST handle backend failures gracefully by displaying the message: "AI assistant temporarily offline. Browse content normally" (per Constitution Principle IX).
- **FR-009**: System MUST prevent hallucination by only answering questions based on retrieved textbook content.
- **FR-010**: System MUST support both full-text queries and text-selection-based queries (as specified in Constitution Principle II).
- **FR-011**: System MUST implement rate limiting to prevent API abuse (e.g., max 10 requests per minute per user).
- **FR-012**: System MUST store API keys in environment variables, never hardcoded (per Constitution Principle VI).
- **FR-013**: System MUST implement LRU caching (100 entries) with query deduplication (5-minute window) to maximize Gemini API quota efficiency.

### Key Entities

- **Chat Message**: Represents a user question or AI response, including text content, timestamp, and source citations. Stored in Neon Postgres.
- **Text Chunk**: Represents a segment of textbook content (typically 500-1000 tokens) with metadata (week number, section title, file path, chunk_index, total_chunks_in_section).
- **Embedding**: Represents the vector representation of a text chunk or query, used for similarity search.
- **Query Session**: Represents a user's conversation context, including message history and selected text context. Stored in Neon Postgres with anonymous session tokens.

## Clarifications

### Session 2025-12-30

- Q: How should the system handle the mismatch between the 50 concurrent users requirement and Gemini's 5 RPM limit? → A: Reduce concurrent user target to 10-15 users (realistic for 5 RPM) and update Constitution
- Q: Should Phase 2 include Neon Postgres for chat history storage to comply with the Constitution? → A: Add Neon Postgres to Phase 2 for anonymous session storage (Constitution compliant)
- Q: Should we merge `/api/chat/query` and `/api/chat/query-selection` into a single endpoint? → A: Merge into single `/api/chat/query` with `query_type` enum parameter
- Q: What caching strategy should be implemented to maximize the limited Gemini API quota? → A: LRU cache (100 entries) + query deduplication within 5-minute window
- Q: Should chunk metadata include positional information for precise citations? → A: Add `chunk_index` and `total_chunks_in_section` for precise citations

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can receive relevant answers to textbook questions in under 3 seconds (p95 latency).
- **SC-002**: At least 80% of queries return responses with valid source citations from the textbook.
- **SC-003**: System handles at least 10-15 concurrent users without degradation (adjusted for Gemini free tier 5 RPM limit).
- **SC-004**: Zero API keys or secrets are exposed in the codebase (verified via security scan).
- **SC-005**: When the backend is unavailable, users can still access and read textbook content without errors.
- **SC-006**: Users can successfully ask questions and receive answers on mobile devices (responsive design).

## Assumptions

- **Assumption 1**: Gemini API free tier provides sufficient quota for hackathon demo purposes (~100 requests/day expected).
- **Assumption 2**: Qdrant Cloud free tier (1GB storage) is sufficient for embedding all 13 weeks of content.
- **Assumption 3**: The free embedding model (e.g., Google text-embedding-004) provides adequate quality for educational content retrieval.
- **Assumption 4**: Users will primarily ask factual questions about content rather than requesting creative writing or off-topic discussions.
- **Assumption 5**: Session-based conversation history (stored in browser) is acceptable for MVP; persistent user accounts are not required for Phase 2.
- **Assumption 6**: The OpenAI Agents SDK is compatible with Gemini API as the LLM backend (requires verification during planning phase).

## Dependencies

- **Dependency 1**: Docusaurus site must be deployed and accessible (completed in Phase 1).
- **Dependency 2**: Textbook content for Weeks 1-2 must be finalized and structured for indexing.
- **Dependency 3**: Gemini API free tier access must be obtained and tested.
- **Dependency 4**: Free embedding model API must be accessible and tested for latency/quality.
- **Dependency 5**: Qdrant Cloud free tier account must be created and configured.

## Out of Scope

- **User authentication**: Phase 2 focuses only on the RAG system; authentication is a separate bonus feature (Phase 4).
- **Content personalization**: Responses are generic and not tailored to user background (Phase 4 feature).
- **Urdu translation**: RAG responses will be in English only for Phase 2.
- **Multi-modal queries**: Phase 2 supports text-only queries; image-based questions are out of scope.
- **Analytics and usage tracking**: No telemetry or usage metrics collection in Phase 2.
