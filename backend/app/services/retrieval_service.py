"""Retrieval service for vector search in Qdrant."""

from typing import List, Dict, Optional
from qdrant_client.http import models

from app.config import get_settings
from app.db.qdrant import get_qdrant_client

settings = get_settings()


class RetrievalService:
    """Service for retrieving relevant document chunks from Qdrant."""

    def __init__(self):
        """Initialize with Qdrant client."""
        self.client = get_qdrant_client()
        self.collection_name = settings.qdrant_collection_name

    async def search(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        filters: Optional[Dict] = None,
        score_threshold: float = 0.7,
    ) -> List[Dict]:
        """
        Search for relevant chunks using vector similarity.

        Args:
            query_embedding: Query vector
            top_k: Number of results to return
            filters: Optional filters (e.g., module, section)
            score_threshold: Minimum similarity score

        Returns:
            List of chunk dictionaries with scores
        """
        # Build filter if provided
        query_filter = None
        if filters:
            conditions = []

            if filters.get("module"):
                conditions.append(
                    models.FieldCondition(
                        key="module",
                        match=models.MatchValue(value=filters["module"]),
                    )
                )

            if conditions:
                query_filter = models.Filter(should=conditions)

        # Perform search using query_points (newer qdrant-client API)
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            query_filter=query_filter,
            limit=top_k,
            score_threshold=score_threshold,
            with_payload=True,
        ).points

        # Format results
        chunks = []
        for hit in results:
            chunk = {
                "content": hit.payload.get("content", ""),
                "title": hit.payload.get("title", ""),
                "section": hit.payload.get("section", ""),
                "document_path": hit.payload.get("document_path", ""),
                "module": hit.payload.get("module", ""),
                "headings": hit.payload.get("headings", []),
                "chunk_id": hit.payload.get("chunk_id", ""),
                "score": hit.score,
            }
            chunks.append(chunk)

        return chunks

    async def search_with_reranking(
        self,
        query_embedding: List[float],
        query_text: str,
        top_k: int = 5,
    ) -> List[Dict]:
        """
        Two-stage retrieval with simple keyword-based reranking.

        Args:
            query_embedding: Query vector
            query_text: Original query text for reranking
            top_k: Number of final results

        Returns:
            Reranked list of chunks
        """
        # Stage 1: Get more candidates with lower threshold
        candidates = await self.search(
            query_embedding=query_embedding,
            top_k=top_k * 3,
            score_threshold=0.5,
        )

        if not candidates:
            return []

        # Stage 2: Rerank using keyword matching
        query_terms = set(query_text.lower().split())

        for candidate in candidates:
            content_lower = candidate["content"].lower()

            # Calculate keyword overlap score
            keyword_matches = sum(1 for term in query_terms if term in content_lower)
            keyword_score = keyword_matches / len(query_terms) if query_terms else 0

            # Combine vector score with keyword score
            candidate["final_score"] = candidate["score"] * 0.7 + keyword_score * 0.3

        # Sort by combined score
        candidates.sort(key=lambda x: x["final_score"], reverse=True)

        return candidates[:top_k]

    async def get_chunk_count(self) -> int:
        """Get total number of chunks in collection."""
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return collection_info.points_count
        except Exception:
            return 0
