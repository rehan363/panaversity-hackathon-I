# Professional RAG Agent Architecture with OpenAI Agents SDK

**Created**: 2026-01-02
**Status**: Design Document
**Tech Stack**: OpenAI Agents SDK + Gemini + Qdrant + FastAPI

---

## 🏗️ Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     User Query Interface                        │
│                   (FastAPI /api/chat/query)                     │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│                    INPUT GUARDRAILS (Blocking)                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐     │
│  │ Relevance    │  │ Language     │  │ Content Safety   │     │
│  │ Checker      │  │ Validator    │  │ Filter           │     │
│  └──────────────┘  └──────────────┘  └──────────────────┘     │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│                    ORCHESTRATOR AGENT                           │
│              (Main Agent - Query Router)                        │
│                                                                 │
│  Responsibilities:                                              │
│  - Analyze user query intent                                    │
│  - Route to appropriate sub-agent                              │
│  - Coordinate handoffs                                          │
│  - Format final response                                        │
│                                                                 │
│  Tools Available:                                               │
│  ✓ handoff_to_retrieval_agent()                                │
│  ✓ handoff_to_direct_answer_agent()                            │
│  ✓ handoff_to_clarification_agent()                            │
└───────────────────────────┬─────────────────────────────────────┘
                            │
            ┌───────────────┼───────────────┐
            │               │               │
            ↓               ↓               ↓
    ┌───────────┐   ┌──────────────┐  ┌──────────────┐
    │ Retrieval │   │ Direct Answer│  │ Clarification│
    │ Agent     │   │ Agent        │  │ Agent        │
    └─────┬─────┘   └──────┬───────┘  └──────┬───────┘
          │                │                  │
          ↓                │                  │
    [Use Tools]            │                  │
          │                │                  │
          └────────────────┴──────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│                  OUTPUT GUARDRAILS (Parallel)                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐     │
│  │ Citation     │  │ Hallucination│  │ Response Length  │     │
│  │ Validator    │  │ Detector     │  │ Checker          │     │
│  └──────────────┘  └──────────────┘  └──────────────────┘     │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
                    [Final Response]
```

---

## 🤖 Agent Hierarchy

### **1. Orchestrator Agent (Main)** 🎯
**Role**: Traffic controller and coordinator

**Instructions**:
```
You are the main orchestrator for a Physical AI textbook assistant.
Your job is to analyze the user's query and route it to the appropriate specialist agent.

ROUTING RULES:
1. If query is about textbook content → handoff_to_retrieval_agent
2. If query is too vague or ambiguous → handoff_to_clarification_agent
3. If query is off-topic (not about robotics/AI) → respond directly with polite decline

NEVER answer content questions yourself. Always delegate to retrieval agent.
```

**Tools**:
- `handoff_to_retrieval_agent()`
- `handoff_to_clarification_agent()`

**Model**: `gemini-1.5-flash` (fast routing)

---

### **2. Retrieval Agent (Sub-Agent)** 📚
**Role**: Search and retrieve relevant textbook content

**Instructions**:
```
You are a retrieval specialist for Physical AI textbook content.
Your job is to search the vector database and provide accurate, cited answers.

RULES:
1. ALWAYS use retrieve_context() tool FIRST
2. ONLY answer based on retrieved context
3. ALWAYS cite sources with [Week X, Section Y, Part N of M]
4. If no relevant context found, admit it clearly
5. Never make up information or use external knowledge

CITATION FORMAT:
"According to [Week 3, ROS 2 Architecture, Part 1 of 2], ..."
```

**Tools**:
- `retrieve_context(query: str, top_k: int, week_filter: Optional[int]) -> str`
- `get_chunk_details(chunk_id: str) -> dict`

**Model**: `gemini-1.5-flash`

---

### **3. Clarification Agent (Sub-Agent)** 🤔
**Role**: Handle ambiguous queries

**Instructions**:
```
You are a clarification specialist.
When a query is vague or ambiguous, help the user refine it.

PROVIDE:
1. Why the query needs clarification
2. 2-3 specific questions to narrow down intent
3. Examples of better-formed queries

Example:
User: "Tell me about robots"
You: "I'd be happy to help! The textbook covers many robot topics.
Could you specify:
- Are you interested in ROS 2 architecture?
- Physical robot components?
- Navigation systems?
Example: 'Explain ROS 2 DDS middleware'"
```

**Tools**: None (conversational only)

**Model**: `gemini-1.5-flash`

---

## 🛡️ Guardrails Architecture

### **Input Guardrails (Blocking Mode)** ⚠️
Run BEFORE agent execution to prevent wasted tokens

#### **1. Relevance Checker**
```python
@input_guardrail
async def check_relevance(ctx, agent, input: str) -> GuardrailFunctionOutput:
    """Check if query is about Physical AI/Robotics."""

    # Fast check using keyword matching + simple LLM call
    keywords = ["robot", "ros", "ai", "sensor", "actuator", "navigation",
                "perception", "control", "physical", "autonomous"]

    # Quick keyword check
    if any(kw in input.lower() for kw in keywords):
        return GuardrailFunctionOutput(
            output_info={"relevant": True, "reason": "Contains robotics keywords"},
            tripwire_triggered=False
        )

    # Deeper check with small model
    result = await relevance_checker_agent.run(input)

    return GuardrailFunctionOutput(
        output_info=result.output,
        tripwire_triggered=not result.output["is_relevant"]
    )
```

**Response if triggered**:
```
"I'm specialized in Physical AI and Robotics topics from the textbook.
Your question seems to be about [detected_topic].
Could you rephrase to focus on robotics, ROS 2, sensors, or autonomous systems?"
```

---

#### **2. Language Validator**
```python
@input_guardrail
async def check_language(ctx, agent, input: str) -> GuardrailFunctionOutput:
    """Ensure query is in English (textbook is English-only)."""

    # Fast language detection
    detected_lang = detect_language(input)

    if detected_lang != "en":
        return GuardrailFunctionOutput(
            output_info={"language": detected_lang},
            tripwire_triggered=True
        )

    return GuardrailFunctionOutput(
        output_info={"language": "en"},
        tripwire_triggered=False
    )
```

**Response if triggered**:
```
"I currently only support questions in English.
Please rephrase your question in English, and I'll be happy to help!"
```

---

#### **3. Content Safety Filter**
```python
@input_guardrail
async def check_safety(ctx, agent, input: str) -> GuardrailFunctionOutput:
    """Filter harmful, inappropriate, or homework-cheating queries."""

    # Check for academic dishonesty signals
    cheating_patterns = [
        "solve this homework", "answer key", "exam questions",
        "give me the solution", "do my assignment"
    ]

    if any(pattern in input.lower() for pattern in cheating_patterns):
        return GuardrailFunctionOutput(
            output_info={"issue": "academic_dishonesty"},
            tripwire_triggered=True
        )

    # Use Gemini's built-in safety ratings
    safety_check = await gemini_safety_check(input)

    return GuardrailFunctionOutput(
        output_info=safety_check,
        tripwire_triggered=safety_check["unsafe"]
    )
```

**Response if triggered**:
```
"I'm here to help you learn, not complete assignments.
I can explain concepts, provide examples, and clarify topics.
How can I help you understand this material better?"
```

---

### **Output Guardrails (Parallel Mode)** ✅
Run AFTER agent generates response (doesn't block)

#### **1. Citation Validator**
```python
@output_guardrail
async def validate_citations(ctx, agent, output: str) -> GuardrailFunctionOutput:
    """Ensure response has proper citations."""

    citation_pattern = r'\[Week \d+[^\]]*\]'
    citations = re.findall(citation_pattern, output)

    has_citations = len(citations) > 0

    if not has_citations and len(output) > 100:  # Long answer without cite
        return GuardrailFunctionOutput(
            output_info={"citations_found": 0, "warning": "Missing citations"},
            tripwire_triggered=True
        )

    return GuardrailFunctionOutput(
        output_info={"citations_found": len(citations)},
        tripwire_triggered=False
    )
```

**Action if triggered**: Log warning, append note to response

---

#### **2. Hallucination Detector**
```python
@output_guardrail
async def detect_hallucination(ctx, agent, output: str) -> GuardrailFunctionOutput:
    """Check if response contains information not in retrieved context."""

    # Get the context that was retrieved
    retrieved_context = ctx.get("retrieved_chunks", [])

    # Use small model to check consistency
    check_result = await hallucination_checker.run({
        "response": output,
        "source_context": retrieved_context
    })

    return GuardrailFunctionOutput(
        output_info=check_result,
        tripwire_triggered=check_result["likely_hallucination"]
    )
```

**Action if triggered**: Add warning, flag for review

---

#### **3. Response Length Checker**
```python
@output_guardrail
async def check_response_length(ctx, agent, output: str) -> GuardrailFunctionOutput:
    """Ensure response is not too long or too short."""

    word_count = len(output.split())

    too_short = word_count < 10
    too_long = word_count > 500

    return GuardrailFunctionOutput(
        output_info={"word_count": word_count},
        tripwire_triggered=(too_short or too_long)
    )
```

---

## 🔧 Tools Architecture

### **Tool 1: retrieve_context** (Core RAG)
```python
@tool
async def retrieve_context(
    query: str,
    top_k: int = 5,
    week_filter: Optional[int] = None,
    similarity_threshold: float = 0.7
) -> str:
    """
    Search vector database for relevant textbook content.

    Args:
        query: User's question
        top_k: Number of chunks to retrieve (default: 5)
        week_filter: Optional week number to filter (1-13)
        similarity_threshold: Minimum similarity score (0.0-1.0)

    Returns:
        Formatted context string with citations
    """

    # Generate embedding
    embedding = await embedding_service.generate_query_embedding(query)

    # Search Qdrant
    results = await vector_store.search(
        query_embedding=embedding,
        top_k=top_k,
        score_threshold=similarity_threshold,
        week_filter=week_filter
    )

    if not results:
        return "No relevant content found in the textbook for this query."

    # Format results with metadata
    formatted_chunks = []
    for idx, chunk in enumerate(results, 1):
        citation = f"[Week {chunk['week']}, {chunk['module']}, Part {chunk['chunk_index']+1} of {chunk['total_chunks']}]"
        formatted_chunks.append(
            f"**Source {idx}** {citation}\n"
            f"Relevance: {chunk['score']:.2%}\n\n"
            f"{chunk['content']}\n\n"
            f"---"
        )

    return "\n\n".join(formatted_chunks)
```

---

### **Tool 2: get_chunk_details** (Metadata Lookup)
```python
@tool
async def get_chunk_details(chunk_id: str) -> dict:
    """
    Get detailed metadata for a specific chunk.

    Args:
        chunk_id: Unique chunk identifier

    Returns:
        Dictionary with chunk metadata
    """

    chunk_data = await vector_store.get_chunk_by_id(chunk_id)

    if not chunk_data:
        return {"error": "Chunk not found"}

    return {
        "week": chunk_data["week"],
        "module": chunk_data["module"],
        "file_path": chunk_data["file_path"],
        "chunk_position": f"{chunk_data['chunk_index']+1}/{chunk_data['total_chunks']}",
        "heading_path": chunk_data.get("heading_path", []),
        "content_preview": chunk_data["content"][:200]
    }
```

---

### **Tool 3: search_by_week** (Filtered Search)
```python
@tool
async def search_by_week(week: int, query: str) -> str:
    """
    Search only within a specific week's content.

    Args:
        week: Week number (1-13)
        query: Search query

    Returns:
        Filtered search results
    """

    return await retrieve_context(
        query=query,
        week_filter=week,
        top_k=3
    )
```

---

## 🎯 Usage Example

```python
from openai import AsyncOpenAI
from openai_agents import Agent, Runner, handoff

# 1. Setup Gemini client
gemini_client = AsyncOpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key=settings.gemini_api_key
)

# 2. Create sub-agents
retrieval_agent = Agent(
    name="Retrieval Specialist",
    instructions=RETRIEVAL_INSTRUCTIONS,
    model=OpenAIChatCompletionsModel(
        model="gemini-1.5-flash",
        openai_client=gemini_client
    ),
    functions=[retrieve_context, get_chunk_details, search_by_week]
)

clarification_agent = Agent(
    name="Clarification Helper",
    instructions=CLARIFICATION_INSTRUCTIONS,
    model=OpenAIChatCompletionsModel(
        model="gemini-1.5-flash",
        openai_client=gemini_client
    )
)

# 3. Create orchestrator with handoffs
orchestrator = Agent(
    name="RAG Orchestrator",
    instructions=ORCHESTRATOR_INSTRUCTIONS,
    model=OpenAIChatCompletionsModel(
        model="gemini-1.5-flash",
        openai_client=gemini_client
    ),
    functions=[
        handoff(retrieval_agent),
        handoff(clarification_agent)
    ],
    input_guardrails=[
        check_relevance,
        check_language,
        check_safety
    ],
    output_guardrails=[
        validate_citations,
        detect_hallucination,
        check_response_length
    ]
)

# 4. Run agent
runner = Runner()

response = await runner.run(
    agent=orchestrator,
    messages=[{"role": "user", "content": "What is DDS middleware?"}]
)

print(response.final_output)
```

---

## 📊 Decision Matrix: When to Use What

| User Query Type | Routed To | Reasoning |
|----------------|-----------|-----------|
| "What is ROS 2?" | Retrieval Agent | Clear content question |
| "Explain sensors" | Retrieval Agent | Specific topic |
| "Tell me about robots" | Clarification Agent | Too vague |
| "Help me with homework" | Orchestrator (deny) | Academic dishonesty |
| "What's the weather?" | Orchestrator (deny) | Off-topic |
| "Compare Week 3 and Week 5" | Retrieval Agent | Complex but valid |

---

## 🔍 Tracing and Monitoring

```python
from openai_agents import set_tracing_processor

# Enable tracing
set_tracing_processor(custom_processor)

# Trace includes:
# - Which agents were invoked
# - Which tools were called
# - Which guardrails triggered
# - Full conversation flow
# - Token usage per agent
```

---

## 📈 Benefits Over Previous Implementation

| Feature | Custom (Previous) | OpenAI SDK (New) |
|---------|------------------|------------------|
| **Agent Specialization** | ❌ Single pipeline | ✅ Multiple specialized agents |
| **Tool Calling** | ❌ Manual orchestration | ✅ Automatic function calling |
| **Guardrails** | ⚠️ Basic validation | ✅ Layered, configurable |
| **Handoffs** | ❌ Not supported | ✅ Built-in agent transfer |
| **Tracing** | ⚠️ Manual logging | ✅ Built-in trace system |
| **Conversation State** | ⚠️ Frontend only | ✅ Built-in message management |
| **Error Handling** | ⚠️ Custom exceptions | ✅ SDK-managed with tripwires |
| **Scalability** | ⚠️ Monolithic | ✅ Modular sub-agents |

---

## 🚀 Implementation Priority

**Phase 1: Core (This Refactor)**
1. ✅ Setup AsyncOpenAI with Gemini
2. ✅ Create Orchestrator Agent
3. ✅ Create Retrieval Agent with tools
4. ✅ Implement input guardrails (blocking)
5. ✅ Test basic flow

**Phase 2: Advanced**
6. ✅ Add Clarification Agent
7. ✅ Implement output guardrails
8. ✅ Add handoff logic
9. ✅ Setup tracing

**Phase 3: Polish**
10. ✅ Add more specialized tools
11. ✅ Fine-tune guardrails
12. ✅ Performance optimization
13. ✅ Comprehensive testing

---

## 📚 Sources
- [OpenAI Agents SDK - Agents](https://openai.github.io/openai-agents-python/agents/)
- [OpenAI Agents SDK - Guardrails](https://openai.github.io/openai-agents-python/guardrails/)
- [Building Agents - OpenAI](https://developers.openai.com/tracks/building-agents/)
- [Practical Guide to Building Agents (PDF)](https://cdn.openai.com/business-guides-and-resources/a-practical-guide-to-building-agents.pdf)

---

**Ready to implement?** 🚀
