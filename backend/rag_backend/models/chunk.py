"""
Pydantic models for text chunk and metadata.
"""

from pydantic import BaseModel, Field
from typing import Optional


class ChunkMetadata(BaseModel):
    """Metadata for a text chunk."""

    week: int = Field(..., ge=1, le=13, description="Week number (1-13)")
    module: str = Field(..., description="Module or section title")
    file_path: str = Field(..., description="Path to source file")
    chunk_index: int = Field(..., ge=0, description="Position of this chunk in the section")
    total_chunks: int = Field(..., ge=1, description="Total number of chunks in the section")
    heading_path: Optional[list[str]] = Field(None, description="Hierarchical heading path (e.g., ['Week 3', 'ROS 2 Architecture', 'DDS Middleware'])")


class TextChunk(BaseModel):
    """Represents a chunk of text with embeddings and metadata."""

    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    content: str = Field(..., min_length=1, description="Text content of the chunk")
    embedding: Optional[list[float]] = Field(None, description="Vector embedding (768 dimensions)")
    metadata: ChunkMetadata = Field(..., description="Chunk metadata")
    token_count: Optional[int] = Field(None, description="Number of tokens in the chunk")
