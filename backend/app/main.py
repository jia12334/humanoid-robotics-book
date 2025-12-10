"""FastAPI application entry point."""

from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.config import get_settings
from app.db.postgres import init_db
from app.db.qdrant import init_qdrant_collection
from app.api.routes import chat_router, health_router

settings = get_settings()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan handler for startup and shutdown events."""
    # Startup
    print("Starting up RAG Chatbot API...")

    # Initialize PostgreSQL tables
    try:
        await init_db()
        print("PostgreSQL database initialized")
    except Exception as e:
        print(f"Warning: Could not initialize PostgreSQL: {e}")

    # Initialize Qdrant collection
    try:
        await init_qdrant_collection()
        print("Qdrant collection initialized")
    except Exception as e:
        print(f"Warning: Could not initialize Qdrant: {e}")

    yield

    # Shutdown
    print("Shutting down RAG Chatbot API...")


# Create FastAPI application
app = FastAPI(
    title="RAG Chatbot API",
    description="RAG-powered chatbot for the Physical AI and Humanoid Robotics Book",
    version="1.0.0",
    lifespan=lifespan,
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


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug,
    )
