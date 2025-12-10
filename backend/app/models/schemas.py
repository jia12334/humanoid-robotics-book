"""Pydantic schemas for API request/response validation."""

from typing import List, Optional
from pydantic import BaseModel, Field
from datetime import datetime
import uuid


class Source(BaseModel):
    """Source citation for a RAG response."""

    title: str = Field(..., description="Document title")
    section: str = Field(..., description="Section heading")
    url: str = Field(..., description="URL path to the source")
    relevance_score: float = Field(..., ge=0, le=1, description="Relevance score")


class ChatRequest(BaseModel):
    """Request schema for chat endpoint."""

    message: str = Field(..., min_length=1, max_length=2000, description="User message")
    conversation_id: Optional[str] = Field(
        None, description="Optional conversation ID for context"
    )


class ChatResponse(BaseModel):
    """Response schema for chat endpoint."""

    answer: str = Field(..., description="Generated response")
    sources: List[Source] = Field(default_factory=list, description="Source citations")
    conversation_id: str = Field(..., description="Conversation ID for follow-ups")


class Message(BaseModel):
    """Individual message in a conversation."""

    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    role: str = Field(..., pattern="^(user|assistant)$")
    content: str
    sources: List[Source] = Field(default_factory=list)
    created_at: datetime = Field(default_factory=datetime.utcnow)


class HealthResponse(BaseModel):
    """Health check response."""

    status: str = Field(..., description="Overall health status")
    services: dict = Field(..., description="Individual service statuses")
    timestamp: datetime = Field(default_factory=datetime.utcnow)


class ChunkPayload(BaseModel):
    """Schema for document chunk stored in Qdrant."""

    chunk_id: str
    document_path: str
    title: str
    section: str
    module: str
    content: str
    headings: List[str] = Field(default_factory=list)
    chunk_index: int
    content_length: int


class IngestRequest(BaseModel):
    """Request to trigger document ingestion."""

    docs_path: str = Field(
        default="humanoid-robotics-book/docs",
        description="Path to docs folder relative to project root",
    )
    force_reingest: bool = Field(
        default=False, description="Force re-ingestion of all documents"
    )


class IngestResponse(BaseModel):
    """Response from ingestion endpoint."""

    status: str
    documents_processed: int
    chunks_created: int
    duration_seconds: float
