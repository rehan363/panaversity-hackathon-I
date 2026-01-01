"""
Rate limiting configuration using slowapi.
"""

from slowapi import Limiter
from slowapi.util import get_remote_address
from rag_backend.config import settings


def get_limiter() -> Limiter:
    """
    Create and configure rate limiter instance.

    Returns:
        Limiter: Configured slowapi Limiter instance
    """
    limiter = Limiter(
        key_func=get_remote_address,
        default_limits=[f"{settings.rate_limit_per_minute}/minute"],
        strategy=settings.rate_limit_strategy,
        storage_uri="memory://",
        headers_enabled=True,
    )
    return limiter


# Global limiter instance
limiter = get_limiter()
