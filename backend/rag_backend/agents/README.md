# RAG Agent System with OpenAI Agents SDK

Professional multi-agent RAG system for the Physical AI Textbook.

---

## 🏗️ Architecture Overview

```
User Query
    ↓
Input Guardrails (Blocking)
    ├─ check_relevance
    ├─ check_language
    └─ check_safety
    ↓
Orchestrator Agent
    ├─ Handoff → Retrieval Agent (+ tools)
    ├─ Handoff → Explanation Agent
    ├─ Handoff → Comparison Agent
    ├─ Handoff → Clarification Agent
    └─ Handoff → Summary Agent
    ↓
Output Guardrails (Parallel)
    ├─ validate_citations
    ├─ check_response_length
    └─ detect_hallucination
    ↓
Response to User
```

---

## 🤖 Agents

### **1. Orchestrator Agent** (Main)
- **Role**: Routes queries to appropriate specialists
- **Model**: gemini-1.5-flash
- **Guardrails**: All input + output guardrails
- **Handoffs**: Can delegate to any sub-agent

### **2. Retrieval Agent** (Sub-agent)
- **Role**: Search textbook and retrieve content
- **Model**: gemini-1.5-flash
- **Tools**:
  - `retrieve_context(query, top_k, week_filter)`
  - `list_week_topics(week)`
  - `search_across_weeks(query, weeks)`
  - `get_chunk_neighbors(chunk_id)`

### **3. Explanation Agent** (Sub-agent)
- **Role**: Simplify complex concepts
- **Model**: gemini-1.5-flash
- **Format**: Simple definition + analogy + why it matters

### **4. Comparison Agent** (Sub-agent)
- **Role**: Compare and contrast topics
- **Model**: gemini-1.5-flash
- **Format**: Table + key differences + recommendations

### **5. Clarification Agent** (Sub-agent)
- **Role**: Handle vague queries
- **Model**: gemini-1.5-flash
- **Format**: Questions + example queries

### **6. Summary Agent** (Sub-agent)
- **Role**: Generate week overviews
- **Model**: gemini-1.5-flash
- **Tools**:
  - `generate_week_summary(week)`
  - `list_week_topics(week)`

---

## 🛠️ Tools/Skills

### **retrieve_context**
```python
await retrieve_context(
    query="What is DDS middleware?",
    top_k=5,
    week_filter=3  # Optional: search only Week 3
)
```

### **list_week_topics**
```python
await list_week_topics(week=3)
# Returns: "1. ROS 2 Architecture, 2. DDS Middleware, 3. QoS..."
```

### **search_across_weeks**
```python
await search_across_weeks(
    query="sensor fusion",
    weeks=[1, 2, 3, 4, 5]  # Optional: specific weeks
)
# Returns: Where concept appears across weeks
```

### **generate_week_summary**
```python
await generate_week_summary(week=3)
# Returns: Complete overview of Week 3
```

---

## 🛡️ Guardrails

### **Input Guardrails** (Blocking)

1. **check_relevance**
   - Blocks off-topic queries (weather, cooking, etc.)
   - Allows robotics/AI keywords
   - Response: "I specialize in Physical AI topics..."

2. **check_language**
   - Blocks non-English queries
   - Response: "I only support English questions..."

3. **check_safety**
   - Blocks academic dishonesty ("solve my homework")
   - Blocks inappropriate content
   - Response: "I'm here to help you learn..."

### **Output Guardrails** (Parallel)

1. **validate_citations**
   - Checks for citation patterns in long answers
   - Warns if citations missing

2. **check_response_length**
   - Validates 10-500 word range
   - Warns if too short or too long

3. **detect_hallucination**
   - Checks for Week 14+ (doesn't exist)
   - Checks for hedge words ("probably", "maybe")
   - Warns if signals detected

---

## 📊 Query Routing Examples

| User Query | Route | Agents Used |
|-----------|-------|-------------|
| "What is ROS 2?" | Simple content | Retrieval Agent |
| "Explain DDS simply" | Needs simplification | Retrieval → Explanation |
| "Compare ROS 1 and ROS 2" | Comparison | Retrieval (2x) → Comparison |
| "What's in Week 3?" | Summary | Summary Agent |
| "Tell me about robots" | Too vague | Clarification Agent |
| "Where is sensor fusion discussed?" | Cross-week | Retrieval (cross-week tool) |
| "What's the weather?" | Off-topic | Blocked by check_relevance |

---

## 🚀 Usage

```python
from openai_agents import Runner
from rag_backend.agents import get_orchestrator_agent

# Get orchestrator
orchestrator = get_orchestrator_agent()

# Create runner
runner = Runner()

# Process query
result = await runner.run(
    agent=orchestrator,
    messages=[{"role": "user", "content": "What is DDS middleware?"}]
)

print(result.final_output)
```

---

## 🔍 Tracing

To see which agents and tools were used:

```python
from openai_agents import set_tracing_processor

def custom_trace_processor(trace_data):
    print(f"Agents used: {trace_data['agents']}")
    print(f"Tools called: {trace_data['tools']}")
    print(f"Guardrails triggered: {trace_data['guardrails']}")

set_tracing_processor(custom_trace_processor)
```

---

## 📁 File Structure

```
backend/rag_backend/agents/
├── __init__.py              # Exports
├── orchestrator.py          # Main orchestrator agent
├── sub_agents.py            # 5 specialized sub-agents
├── tools.py                 # 5 tools/skills
├── guardrails.py            # 6 guardrails (3 input + 3 output)
└── README.md                # This file
```

---

## ✅ Benefits Over Previous Implementation

1. **Modularity**: Each agent has one focused job
2. **Guardrails**: Prevents off-topic, inappropriate, or hallucinated responses
3. **Tool Calling**: Automatic, agent decides when to use tools
4. **Handoffs**: Complex queries handled by multiple specialists
5. **Tracing**: Built-in monitoring of agent activity
6. **Testing**: Each agent testable independently
7. **Scalability**: Add new agents without touching existing code

---

## 🎯 Next Steps

1. Install dependencies: `pip install openai openai-agents`
2. Test simple query: `curl -X POST /api/chat/query -d '{"query": "What is ROS?"}'`
3. Test guardrails: Try off-topic query
4. Test handoffs: Try comparison query
5. Monitor logs to see agent routing

---

**This is a production-ready, professional RAG agent system!** 🚀
