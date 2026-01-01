"""
Pydantic models for health check endpoint.
"""

from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class HealthCheckResponse(BaseModel):
    """Response model for health check endpoint."""

    status: str = Field(..., description="Health status: 'healthy' or 'unhealthy'")
    version: str = Field(..., description="Application version")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Health check timestamp")
    services: dict[str, str] = Field(
        default_factory=dict,
        description="Status of individual services (e.g., {'qdrant': 'ok', 'database': 'ok'})"
    )
    details: Optional[dict] = Field(None, description="Additional health details")
