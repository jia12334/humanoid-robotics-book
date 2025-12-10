"""SQLAlchemy database models for Neon Postgres."""

from datetime import datetime
from typing import List
import uuid

from sqlalchemy import (
    Column,
    String,
    Text,
    DateTime,
    ForeignKey,
    Index,
    JSON,
)
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import DeclarativeBase, relationship


class Base(DeclarativeBase):
    """Base class for SQLAlchemy models."""

    pass


class Conversation(Base):
    """Conversation model to group related messages."""

    __tablename__ = "conversations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow)
    updated_at = Column(
        DateTime(timezone=True), default=datetime.utcnow, onupdate=datetime.utcnow
    )
    metadata_ = Column("metadata", JSON, default=dict)

    # Relationship to messages
    messages = relationship(
        "MessageDB", back_populates="conversation", cascade="all, delete-orphan"
    )

    def __repr__(self):
        return f"<Conversation(id={self.id})>"


class MessageDB(Base):
    """Message model for storing chat history."""

    __tablename__ = "messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    conversation_id = Column(
        UUID(as_uuid=True), ForeignKey("conversations.id", ondelete="CASCADE")
    )
    role = Column(String(20), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    sources = Column(JSON, default=list)  # Store source citations as JSON
    selected_text = Column(Text, nullable=True)  # For selection-based queries
    page_url = Column(String(500), nullable=True)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow)

    # Relationship to conversation
    conversation = relationship("Conversation", back_populates="messages")

    __table_args__ = (Index("idx_messages_conversation", "conversation_id"),)

    def __repr__(self):
        return f"<Message(id={self.id}, role={self.role})>"


class ChunkMetadata(Base):
    """Metadata for document chunks (mirrors Qdrant payload for joins)."""

    __tablename__ = "chunks_metadata"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chunk_id = Column(String(100), unique=True, nullable=False)
    document_path = Column(String(500), nullable=False)
    title = Column(String(500))
    section = Column(String(500))
    module = Column(String(100))
    content_hash = Column(String(64))  # SHA256 hash for change detection
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow)
    updated_at = Column(
        DateTime(timezone=True), default=datetime.utcnow, onupdate=datetime.utcnow
    )

    __table_args__ = (
        Index("idx_chunks_document_path", "document_path"),
        Index("idx_chunks_module", "module"),
    )

    def __repr__(self):
        return f"<ChunkMetadata(chunk_id={self.chunk_id})>"
