# Sub-Agents & Skills Implementation Guide

**For**: Physical AI Textbook RAG System
**Date**: 2026-01-02

---

## 🎯 Real-World Use Cases

Let me show you **exactly** how sub-agents and skills solve real problems in our RAG system.

---

## 📚 **Scenario 1: Complex Query Handling**

### **User Query:**
```
"Compare ROS 1 and ROS 2 architecture, and explain which is better for autonomous navigation"
```

### **Problem with Single Agent:**
```python
# Without sub-agents - one agent does everything
single_agent.instructions = """
1. Search for ROS 1 info
2. Search for ROS 2 info
3. Compare them
4. Analyze navigation context
5. Make recommendation
6. Cite all sources
"""
# Issues:
# - Too complex for one agent
# - Forgets context between steps
# - Citations get mixed up
# - Hard to validate each step
```

### **Solution with Sub-Agents:**

```python
# 1. ORCHESTRATOR AGENT - Breaks down the task
orchestrator_agent = Agent(
    name="Query Decomposer",
    instructions="""
    Break complex queries into sub-tasks.
    For comparison queries:
    1. Identify topics to compare
    2. Hand off each topic to retrieval agent
    3. Hand off comparison to analysis agent
    4. Synthesize final answer
    """,
    functions=[
        handoff(retrieval_agent),
        handoff(comparison_agent)
    ]
)

# 2. RETRIEVAL AGENT - Specialized in searching
retrieval_agent = Agent(
    name="Content Retriever",
    instructions="""
    Search ONLY for requested topic.
    Return raw context with citations.
    Do NOT compare or analyze.
    """,
    functions=[retrieve_context, search_by_week]
)

# 3. COMPARISON AGENT - Specialized in analysis
comparison_agent = Agent(
    name="Comparison Analyst",
    instructions="""
    Given two contexts, compare and contrast.
    Highlight key differences.
    Make evidence-based recommendations.
    """,
    functions=[generate_comparison_table]
)

# Execution Flow:
# User: "Compare ROS 1 and ROS 2..."
#   ↓
# Orchestrator: "I need two separate searches"
#   ↓
# Handoff to Retrieval: "Get ROS 1 architecture"
#   ↓ (retrieves + cites)
# Handoff to Retrieval: "Get ROS 2 architecture"
#   ↓ (retrieves + cites)
# Handoff to Comparison: "Compare these two contexts"
#   ↓ (analyzes, compares)
# Orchestrator: "Here's the comparison with citations"
```

---

## 🛠️ **Scenario 2: Tool/Skills in Action**

### **User Query:**
```
"Show me all topics covered in Week 3"
```

### **Skills Needed:**

#### **Skill 1: list_week_topics**
```python
@tool
async def list_week_topics(week: int) -> str:
    """
    List all topics covered in a specific week.

    Args:
        week: Week number (1-13)

    Returns:
        Formatted list of topics with module names
    """

    # Query Qdrant for all chunks in this week
    results = await vector_store.client.scroll(
        collection_name=vector_store.collection_name,
        scroll_filter=Filter(
            must=[FieldCondition(key="week", match=MatchValue(value=week))]
        ),
        limit=100
    )

    # Extract unique modules
    modules = set()
    for point in results[0]:
        modules.add(point.payload.get("module"))

    # Format output
    output = f"## Week {week} Topics:\n\n"
    for idx, module in enumerate(sorted(modules), 1):
        output += f"{idx}. {module}\n"

    return output

# Agent automatically knows to use this tool
agent = Agent(
    name="Topic Lister",
    instructions="Use list_week_topics when user asks about week contents",
    functions=[list_week_topics]
)

# User: "Show me all topics covered in Week 3"
# Agent thinks: "I should use list_week_topics(week=3)"
# Tool executes → Returns formatted list
# Agent: "Here are the topics in Week 3: ..."
```

---

#### **Skill 2: search_across_weeks**
```python
@tool
async def search_across_weeks(
    query: str,
    weeks: list[int]
) -> dict[int, list[dict]]:
    """
    Search for a concept across multiple weeks.

    Args:
        query: Search query
        weeks: List of week numbers to search

    Returns:
        Dictionary mapping week number to results
    """

    results_by_week = {}

    for week in weeks:
        week_results = await vector_store.search(
            query_embedding=await embedding_service.generate_query_embedding(query),
            week_filter=week,
            top_k=3
        )

        if week_results:
            results_by_week[week] = week_results

    return results_by_week

# Usage:
# User: "Where is sensor fusion discussed?"
# Agent: "Let me search across all weeks"
#   → search_across_weeks("sensor fusion", weeks=[1,2,3,4,5...13])
# Agent: "Sensor fusion is discussed in Week 4, Week 8, and Week 11"
```

---

#### **Skill 3: get_prerequisite_topics**
```python
@tool
async def get_prerequisite_topics(week: int) -> list[str]:
    """
    Get prerequisite topics needed before studying a week.

    Args:
        week: Target week number

    Returns:
        List of prerequisite topic names
    """

    # This could read from a structured metadata file
    prerequisites = {
        1: [],
        2: ["Week 1: Introduction to Physical AI"],
        3: ["Week 1: Intro", "Week 2: ROS Basics"],
        4: ["Week 2: ROS", "Week 3: ROS 2 Architecture"],
        # ... etc
    }

    return prerequisites.get(week, [])

# Usage:
# User: "What should I learn before Week 5?"
# Agent: Uses get_prerequisite_topics(5)
# Agent: "Before Week 5, you should understand: [prerequisites]"
```

---

## 🎭 **Scenario 3: Multi-Agent Collaboration**

### **User Query:**
```
"I'm stuck on understanding DDS middleware. Can you explain it simply?"
```

### **Agent Collaboration Flow:**

```python
# 1. MAIN ORCHESTRATOR
orchestrator = Agent(
    name="Main Assistant",
    instructions="""
    Route queries to appropriate specialist.
    For "explain simply" requests, use explanation agent.
    """,
    functions=[
        handoff(retrieval_agent),
        handoff(explanation_agent)
    ]
)

# 2. RETRIEVAL AGENT (Sub-agent 1)
retrieval_agent = Agent(
    name="Content Retriever",
    instructions="Get exact textbook content about the topic",
    functions=[retrieve_context]
)

# 3. EXPLANATION AGENT (Sub-agent 2)
explanation_agent = Agent(
    name="Concept Simplifier",
    instructions="""
    You receive textbook content from retrieval agent.
    Your job: Explain in simple terms with analogies.

    Format:
    1. Simple Definition (one sentence)
    2. Real-World Analogy
    3. Why It Matters
    4. Key Takeaway
    """,
    functions=[generate_analogy, create_visual_metaphor]
)

# Execution:
# User: "I'm stuck on understanding DDS middleware. Explain simply?"
#   ↓
# Orchestrator: Detects "explain simply" → Routes to specialists
#   ↓
# Handoff to Retrieval Agent: "Get DDS middleware content"
#   ↓ (retrieves technical content)
# Handoff to Explanation Agent: "Simplify this content"
#   ↓ (generates simple explanation + analogy)
# Orchestrator: Returns simplified explanation with citations
```

---

## 🔍 **Scenario 4: Validation Chain**

### **User Query:**
```
"What does the textbook say about quantum computing?"
```

### **Validation Sub-Agents:**

```python
# 1. RELEVANCE CHECKER (Sub-agent)
relevance_checker = Agent(
    name="Relevance Validator",
    instructions="""
    Check if query is about Physical AI/Robotics topics.
    Return: {is_relevant: bool, suggested_topics: list}
    """,
    model="gemini-1.5-flash"  # Fast, cheap model
)

# 2. MAIN RETRIEVAL AGENT
retrieval_agent = Agent(
    name="Content Retriever",
    instructions="Search textbook ONLY if query is relevant",
    functions=[retrieve_context]
)

# Flow:
# User: "What does the textbook say about quantum computing?"
#   ↓
# Relevance Checker:
#   → Checks: "quantum computing" vs "robotics topics"
#   → Returns: {is_relevant: False, reason: "Not a robotics topic"}
#   ↓
# Orchestrator: "This topic isn't covered in the Physical AI textbook.
#                The textbook focuses on robotics, ROS, sensors, etc."
#   ↓
# Retrieval Agent: NEVER CALLED (saves tokens + API calls!)
```

---

## 🧩 **Complete Architecture Example**

### **Full System with All Sub-Agents:**

```python
from openai import AsyncOpenAI
from openai_agents import Agent, Runner, OpenAIChatCompletionsModel, handoff, tool

# Setup Gemini client
gemini_client = AsyncOpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key=settings.gemini_api_key
)

# ===== TOOLS/SKILLS =====

@tool
async def retrieve_context(query: str, top_k: int = 5) -> str:
    """Search vector database for relevant content."""
    embedding = await embedding_service.generate_query_embedding(query)
    chunks = await vector_store.search(embedding, top_k=top_k)
    return format_chunks_with_citations(chunks)

@tool
async def list_week_topics(week: int) -> str:
    """List all topics in a specific week."""
    # Implementation shown above
    ...

@tool
async def search_across_weeks(query: str, weeks: list[int]) -> dict:
    """Search multiple weeks for a concept."""
    # Implementation shown above
    ...

@tool
async def get_chunk_neighbors(chunk_id: str, before: int = 1, after: int = 1) -> str:
    """Get surrounding chunks for more context."""
    # Useful when user needs more detail
    ...

@tool
async def generate_summary(week: int) -> str:
    """Generate a summary of an entire week's content."""
    ...

# ===== SUB-AGENTS =====

# 1. Retrieval Specialist
retrieval_agent = Agent(
    name="Retrieval Specialist",
    instructions="""
    You are an expert at searching the Physical AI textbook.

    Rules:
    - Use retrieve_context() for topic searches
    - Use list_week_topics() when asked about week contents
    - Use search_across_weeks() for concept tracking
    - Always include citations
    - Return raw context, don't analyze
    """,
    model=OpenAIChatCompletionsModel(
        model="gemini-1.5-flash",
        openai_client=gemini_client
    ),
    functions=[
        retrieve_context,
        list_week_topics,
        search_across_weeks,
        get_chunk_neighbors
    ]
)

# 2. Explanation Specialist
explanation_agent = Agent(
    name="Explanation Specialist",
    instructions="""
    You simplify complex concepts.

    Given textbook content, create:
    1. Simple definition (one sentence)
    2. Real-world analogy
    3. Why it matters
    4. Key takeaway

    Use everyday language, avoid jargon.
    """,
    model=OpenAIChatCompletionsModel(
        model="gemini-1.5-flash",
        openai_client=gemini_client
    )
)

# 3. Comparison Specialist
comparison_agent = Agent(
    name="Comparison Analyst",
    instructions="""
    You compare and contrast concepts.

    Format:
    | Feature | Option A | Option B |
    |---------|----------|----------|
    | ...     | ...      | ...      |

    Then: Recommendation based on use case.
    """,
    model=OpenAIChatCompletionsModel(
        model="gemini-1.5-flash",
        openai_client=gemini_client
    )
)

# 4. Clarification Helper
clarification_agent = Agent(
    name="Clarification Helper",
    instructions="""
    You help users refine vague queries.

    Provide:
    - Why the query needs clarification
    - 2-3 specific questions
    - Example of a better query
    """,
    model=OpenAIChatCompletionsModel(
        model="gemini-1.5-flash",
        openai_client=gemini_client
    )
)

# 5. Summary Generator
summary_agent = Agent(
    name="Summary Generator",
    instructions="""
    You create concise summaries.

    Given content, provide:
    - Key concepts (bullet points)
    - Main takeaways
    - Prerequisites (if any)
    - Next topics to explore
    """,
    model=OpenAIChatCompletionsModel(
        model="gemini-1.5-flash",
        openai_client=gemini_client
    ),
    functions=[generate_summary]
)

# ===== MAIN ORCHESTRATOR =====

orchestrator = Agent(
    name="RAG Orchestrator",
    instructions="""
    You are the main coordinator for a Physical AI textbook assistant.

    ROUTING RULES:

    1. Simple Content Queries → retrieval_agent
       Examples: "What is ROS?", "Explain sensors"

    2. "Explain Simply" Requests → retrieval_agent THEN explanation_agent
       Examples: "I don't understand X", "Explain Y simply"

    3. Comparison Queries → retrieval_agent (twice) THEN comparison_agent
       Examples: "Compare A and B", "What's the difference between X and Y"

    4. Vague Queries → clarification_agent
       Examples: "Tell me about robots", "Help me with AI"

    5. Week Overview → summary_agent
       Examples: "Summarize Week 3", "What's in Week 5"

    6. Off-Topic Queries → Handle yourself
       Respond: "I specialize in Physical AI and Robotics topics from the textbook."

    NEVER answer content questions yourself. Always delegate.
    """,
    model=OpenAIChatCompletionsModel(
        model="gemini-1.5-flash",
        openai_client=gemini_client
    ),
    functions=[
        handoff(retrieval_agent),
        handoff(explanation_agent),
        handoff(comparison_agent),
        handoff(clarification_agent),
        handoff(summary_agent)
    ]
)

# ===== USAGE =====

async def process_user_query(user_query: str):
    """Process a user query through the agent system."""

    runner = Runner()

    result = await runner.run(
        agent=orchestrator,
        messages=[{"role": "user", "content": user_query}]
    )

    return result.final_output
```

---

## 📊 **Query Routing Examples**

| User Query | Orchestrator Decision | Sub-Agents Used | Tools Called |
|------------|----------------------|-----------------|--------------|
| "What is DDS middleware?" | Simple content query | retrieval_agent | retrieve_context("DDS middleware") |
| "Explain DDS simply" | Needs simplification | retrieval_agent → explanation_agent | retrieve_context() then simplify |
| "Compare ROS 1 and ROS 2" | Comparison needed | retrieval_agent (2x) → comparison_agent | retrieve_context("ROS 1"), retrieve_context("ROS 2") |
| "What's in Week 3?" | Week overview | summary_agent | list_week_topics(3), generate_summary(3) |
| "Tell me about robots" | Too vague | clarification_agent | None (conversational) |
| "Where is sensor fusion discussed?" | Cross-week search | retrieval_agent | search_across_weeks("sensor fusion", [1-13]) |
| "What's the weather?" | Off-topic | orchestrator handles | None (polite decline) |

---

## 🎯 **Benefits Summary**

### **Why Sub-Agents?**

1. **Specialization** → Each agent is expert at one thing
2. **Clarity** → Instructions are focused and clear
3. **Testing** → Test each agent independently
4. **Scalability** → Add new agents without touching existing ones
5. **Maintainability** → Change one agent without affecting others

### **Why Tools/Skills?**

1. **Automatic Invocation** → Agent decides when to call tools
2. **Reusability** → Same tool used by multiple agents
3. **Type Safety** → Pydantic validation on tool inputs
4. **Documentation** → Docstrings help agent understand tool purpose
5. **Error Handling** → SDK handles tool failures gracefully

---

## 🔄 **Execution Flow Diagram**

```
User: "Compare ROS 1 and ROS 2 for navigation"
  ↓
┌────────────────────────────────────────┐
│      Orchestrator Agent                │
│  "This is a comparison query"          │
│  "I need retrieval → comparison"       │
└──────────────┬─────────────────────────┘
               │
               ↓ handoff()
┌────────────────────────────────────────┐
│      Retrieval Agent (Call 1)          │
│  Tool: retrieve_context("ROS 1")       │
│  Returns: [Week 2 chunks with ROS 1]   │
└──────────────┬─────────────────────────┘
               │
               ↓ handoff()
┌────────────────────────────────────────┐
│      Retrieval Agent (Call 2)          │
│  Tool: retrieve_context("ROS 2")       │
│  Returns: [Week 3 chunks with ROS 2]   │
└──────────────┬─────────────────────────┘
               │
               ↓ handoff()
┌────────────────────────────────────────┐
│      Comparison Agent                  │
│  Input: ROS 1 context + ROS 2 context  │
│  Output: Comparison table + analysis   │
└──────────────┬─────────────────────────┘
               │
               ↓
        Final Response with Citations
```

---

## 🚀 **Next Steps**

Would you like me to:

1. ✅ **Implement this full architecture** with all sub-agents and tools
2. ✅ **Show you how to test** each agent independently
3. ✅ **Add more specialized agents** (e.g., Code Example Agent, Prerequisite Checker)
4. ✅ **Implement tracing** to see which agents/tools were used

This architecture is **production-ready** and follows OpenAI's best practices! 🎯
