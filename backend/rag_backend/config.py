"""
Configuration management for RAG backend.

This module handles all environment variables and application settings
using Pydantic Settings for validation and type safety.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )

    # Gemini API Configuration
    gemini_api_key: str
    gemini_model: str = "gemini-1.5-flash"
    gemini_embedding_model: str = "models/text-embedding-004"
    gemini_temperature: float = 0.7
    gemini_max_tokens: int = 1024

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "physical_ai_textbook"
    qdrant_vector_size: int = 768  # Google text-embedding-004 dimensions

    # Neon Postgres Configuration
    neon_database_url: str
    neon_pool_size: int = 10
    neon_max_overflow: int = 5

    # Rate Limiting Configuration
    rate_limit_per_minute: int = 3
    rate_limit_strategy: str = "fixed-window"

    # Caching Configuration
    cache_max_entries: int = 100
    cache_ttl_seconds: int = 300  # 5 minutes

    # RAG Pipeline Configuration
    top_k_results: int = 5
    similarity_threshold: float = 0.7
    chunk_size: int = 768
    chunk_overlap: int = 100

    # Admin Configuration
    admin_token: Optional[str] = None

    # Application Configuration
    app_name: str = "Physical AI Textbook RAG Backend"
    app_version: str = "1.0.0"
    debug_mode: bool = False
    log_level: str = "INFO"

    # CORS Configuration
    cors_origins: list[str] = [
        "http://localhost:3000",
        "https://rehan363.github.io",
        "https://panaversity-hackathon-i.vercel.app"
    ]

    # Documentation Path
    docs_base_path: str = "../physical-ai-textbook/docs"


# Global settings instance
settings = Settings()
