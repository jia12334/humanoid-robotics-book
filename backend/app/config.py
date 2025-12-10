"""Application configuration using Pydantic Settings."""

from typing import List
from pydantic_settings import BaseSettings
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # OpenAI
    openai_api_key: str = ""  # Optional if using Gemini

    # Gemini (free embeddings)
    gemini_api_key: str = ""
    embedding_provider: str = "gemini"  # "openai" or "gemini"
    vector_dimensions: int = 768  # 768 for Gemini, 1536 for OpenAI

    # Qdrant
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "book_chunks"

    # PostgreSQL (Neon)
    database_url: str

    # Application
    environment: str = "development"
    debug: bool = False

    # CORS
    cors_origins: str = "http://localhost:3000"

    # Models
    embedding_model: str = "text-embedding-3-small"
    chat_model: str = "gemini-1.5-flash"

    # RAG Configuration
    top_k_results: int = 5
    similarity_threshold: float = 0.7
    max_context_length: int = 4000

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
