"""Vercel serverless function entry point for FastAPI backend."""

import sys
import os

# Add backend directory to Python path for imports
backend_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'backend')
sys.path.insert(0, backend_path)

from mangum import Mangum
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.config import get_settings
from app.api.routes import chat_router, health_router

settings = get_settings()

# Create FastAPI application (without lifespan for serverless)
app = FastAPI(
    title="RAG Chatbot API",
    description="RAG-powered chatbot for the Physical AI and Humanoid Robotics Book",
    version="1.0.0",
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health_router, prefix="/api")
app.include_router(chat_router, prefix="/api")


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "name": "RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/api/health",
    }


@app.get("/api")
async def api_root():
    """API root endpoint."""
    return {
        "name": "RAG Chatbot API",
        "version": "1.0.0",
        "endpoints": {
            "chat": "/api/chat",
            "health": "/api/health",
        },
    }


# Mangum handler for Vercel serverless
handler = Mangum(app, lifespan="off")
