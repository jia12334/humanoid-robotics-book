"""Qdrant vector database client and collection management."""

from typing import Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.exceptions import UnexpectedResponse

from app.config import get_settings

settings = get_settings()

# Global client instance
_qdrant_client: Optional[QdrantClient] = None


def get_qdrant_client() -> QdrantClient:
    """Get or create Qdrant client instance."""
    global _qdrant_client

    if _qdrant_client is None:
        _qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=120,  # Increased timeout for slow connections
        )

    return _qdrant_client


async def init_qdrant_collection() -> None:
    """Initialize Qdrant collection if it doesn't exist."""
    client = get_qdrant_client()
    collection_name = settings.qdrant_collection_name

    try:
        # Check if collection exists
        collections = client.get_collections()
        collection_names = [c.name for c in collections.collections]

        if collection_name not in collection_names:
            # Create collection with configurable dimensions (768 for Gemini, 1536 for OpenAI)
            client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=settings.vector_dimensions,
                    distance=models.Distance.COSINE,
                ),
                # Optimize for search performance
                optimizers_config=models.OptimizersConfigDiff(
                    indexing_threshold=10000,
                ),
            )
            print(f"Created Qdrant collection: {collection_name}")
        else:
            print(f"Qdrant collection '{collection_name}' already exists")

    except UnexpectedResponse as e:
        print(f"Error initializing Qdrant collection: {e}")
        raise


async def check_qdrant_health() -> bool:
    """Check if Qdrant is accessible."""
    try:
        client = get_qdrant_client()
        client.get_collections()
        return True
    except Exception:
        return False
