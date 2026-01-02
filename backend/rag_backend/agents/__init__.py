"""
RAG Agent System with OpenAI Agents SDK

This module contains the complete agent system:
- Orchestrator: Main routing agent
- Sub-agents: Specialized agents for different tasks
- Tools: Skills that agents can use
- Guardrails: Input/output validation
"""

from rag_backend.agents.orchestrator import get_orchestrator_agent
from rag_backend.agents.sub_agents import get_all_sub_agents
from rag_backend.agents.tools import (
    retrieve_context,
    list_week_topics,
    search_across_weeks,
    get_chunk_neighbors,
    generate_week_summary
)
from rag_backend.agents.guardrails import (
    check_relevance,
    check_language,
    check_safety,
    validate_citations,
    check_response_length,
    detect_hallucination,
    get_guardrail_response
)

__all__ = [
    "get_orchestrator_agent",
    "get_all_sub_agents",
    "retrieve_context",
    "list_week_topics",
    "search_across_weeks",
    "get_chunk_neighbors",
    "generate_week_summary",
    "check_relevance",
    "check_language",
    "check_safety",
    "validate_citations",
    "check_response_length",
    "detect_hallucination",
    "get_guardrail_response",
]
