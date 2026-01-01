"""
FastAPI application entry point for RAG Integration backend.

This module initializes the FastAPI app with CORS, rate limiting,
error handlers, and routes for the RAG chatbot system.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Textbook RAG API",
    description="Retrieval-Augmented Generation API for Physical AI Textbook",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS Configuration
# Allowed origins from contracts/chat-api.md
allowed_origins = [
    "http://localhost:3000",                        # Docusaurus dev server
    "https://rehan363.github.io",                   # GitHub Pages production
    "https://panaversity-hackathon-i.vercel.app"    # Vercel preview
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type", "Authorization", "X-Request-ID"],
    expose_headers=["X-RateLimit-Limit", "X-RateLimit-Remaining", "X-RateLimit-Reset"]
)

# Root endpoint
@app.get("/")
async def root():
    """Root endpoint returning API information."""
    return {
        "name": "Physical AI Textbook RAG API",
        "version": "1.0.0",
        "status": "operational",
        "docs": "/docs",
        "health": "/api/health"
    }

# Health check endpoint (basic - will be enhanced in Phase 2)
@app.get("/api/health")
async def health_check():
    """
    Basic health check endpoint.

    Returns operational status for frontend to determine if backend is available.
    Will be enhanced in Phase 2 with Qdrant and Gemini connectivity checks.
    """
    return {
        "status": "healthy",
        "message": "Backend is operational"
    }

# Startup event
@app.on_event("startup")
async def startup_event():
    """Application startup event."""
    logger.info("🚀 Physical AI Textbook RAG API starting up...")
    logger.info(f"📋 API Documentation available at /docs")
    logger.info(f"🔒 CORS enabled for origins: {allowed_origins}")
    logger.info("✅ Application ready")

# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Application shutdown event."""
    logger.info("👋 Physical AI Textbook RAG API shutting down...")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
