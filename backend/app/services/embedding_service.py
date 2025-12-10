"""Embedding service with support for multiple providers (OpenAI, Gemini)."""

from abc import ABC, abstractmethod
from typing import List
import google.generativeai as genai
from openai import AsyncOpenAI

from app.config import get_settings

settings = get_settings()


class BaseEmbeddingService(ABC):
    """Abstract base class for embedding services."""

    @abstractmethod
    async def embed_text(self, text: str) -> List[float]:
        """Generate embedding for a single text."""
        pass

    @abstractmethod
    async def embed_batch(self, texts: List[str], batch_size: int = 100) -> List[List[float]]:
        """Generate embeddings for multiple texts."""
        pass

    async def embed_query(self, query: str) -> List[float]:
        """Generate embedding for a search query."""
        return await self.embed_text(query)


class GeminiEmbeddingService(BaseEmbeddingService):
    """Embedding service using Google Gemini API (FREE)."""

    def __init__(self):
        """Initialize with Gemini client."""
        genai.configure(api_key=settings.gemini_api_key)
        self.model = "models/text-embedding-004"

    async def embed_text(self, text: str) -> List[float]:
        """Generate embedding for a single text."""
        text = text.replace("\n", " ").strip()

        result = genai.embed_content(
            model=self.model,
            content=text,
            task_type="retrieval_document",
        )

        return result['embedding']

    async def embed_batch(self, texts: List[str], batch_size: int = 100) -> List[List[float]]:
        """Generate embeddings for multiple texts."""
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]
            cleaned_batch = [t.replace("\n", " ").strip() for t in batch]

            # Gemini supports batch embedding
            result = genai.embed_content(
                model=self.model,
                content=cleaned_batch,
                task_type="retrieval_document",
            )

            all_embeddings.extend(result['embedding'])

        return all_embeddings

    async def embed_query(self, query: str) -> List[float]:
        """Generate embedding for a search query."""
        query = query.replace("\n", " ").strip()

        result = genai.embed_content(
            model=self.model,
            content=query,
            task_type="retrieval_query",
        )

        return result['embedding']


class OpenAIEmbeddingService(BaseEmbeddingService):
    """Embedding service using OpenAI API."""

    def __init__(self):
        """Initialize with OpenAI client."""
        self.client = AsyncOpenAI(api_key=settings.openai_api_key)
        self.model = settings.embedding_model

    async def embed_text(self, text: str) -> List[float]:
        """Generate embedding for a single text."""
        text = text.replace("\n", " ").strip()

        response = await self.client.embeddings.create(
            model=self.model,
            input=text,
        )

        return response.data[0].embedding

    async def embed_batch(self, texts: List[str], batch_size: int = 100) -> List[List[float]]:
        """Generate embeddings for multiple texts."""
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]
            cleaned_batch = [t.replace("\n", " ").strip() for t in batch]

            response = await self.client.embeddings.create(
                model=self.model,
                input=cleaned_batch,
            )

            batch_embeddings = [item.embedding for item in response.data]
            all_embeddings.extend(batch_embeddings)

        return all_embeddings


class EmbeddingService(BaseEmbeddingService):
    """Factory class that delegates to the configured provider."""

    def __init__(self):
        """Initialize with the configured provider."""
        if settings.embedding_provider == "gemini":
            self._service = GeminiEmbeddingService()
        else:
            self._service = OpenAIEmbeddingService()

    async def embed_text(self, text: str) -> List[float]:
        return await self._service.embed_text(text)

    async def embed_batch(self, texts: List[str], batch_size: int = 100) -> List[List[float]]:
        return await self._service.embed_batch(texts, batch_size)

    async def embed_query(self, query: str) -> List[float]:
        return await self._service.embed_query(query)
