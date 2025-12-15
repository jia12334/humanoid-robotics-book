"""RAG service for orchestrating the retrieval and generation pipeline."""

import uuid
from typing import List, Dict, Optional
import google.generativeai as genai
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.config import get_settings
from app.models.database import Conversation, MessageDB
from app.services.embedding_service import EmbeddingService
from app.services.retrieval_service import RetrievalService

settings = get_settings()

# System prompt for the chatbot
SYSTEM_PROMPT = """You are a helpful assistant for the "Physical AI and Humanoid Robotics" textbook.

Your role is to:
1. Answer questions about robotics, ROS 2, digital twin simulation, NVIDIA Isaac, and humanoid systems
2. Explain concepts from the book clearly and accurately
3. Provide code examples when relevant (the book uses Python and ROS 2)
4. Reference specific chapters or sections when applicable
5. If asked about something not covered in the provided context, clearly state that you don't have information about it in the book

Guidelines:
- Be concise but thorough
- Use technical terms appropriately but explain them when needed
- When showing code, use proper markdown formatting
- Always cite which section/chapter your information comes from
- If the context doesn't contain relevant information, say so honestly

The user is learning about Physical AI and robotics - be encouraging and educational."""


class RAGService:
    """Service for RAG-powered question answering."""

    def __init__(self, db: AsyncSession):
        """
        Initialize RAG service.

        Args:
            db: Database session for conversation persistence
        """
        self.db = db
        self.embedding_service = EmbeddingService()
        self.retrieval_service = RetrievalService()
        genai.configure(api_key=settings.gemini_api_key)
        self.gemini_model = genai.GenerativeModel('models/gemini-2.5-flash')

    async def answer_question(
        self,
        question: str,
        conversation_id: Optional[str] = None,
        selected_text: Optional[str] = None,
    ) -> Dict:
        """
        Answer a question using RAG pipeline.

        Args:
            question: User's question
            conversation_id: Optional ID for conversation context
            selected_text: Optional text the user selected for context

        Returns:
            Dictionary with answer and sources
        """
        # Step 1: Generate query embedding
        query_embedding = await self.embedding_service.embed_query(question)

        # Step 2: Retrieve relevant chunks
        chunks = await self.retrieval_service.search_with_reranking(
            query_embedding=query_embedding,
            query_text=question,
            top_k=settings.top_k_results,
        )

        # Step 3: Build context from retrieved chunks
        context = self._build_context(chunks, selected_text)

        # Step 4: Get conversation history if available
        conversation_history = []
        if conversation_id:
            conversation_history = await self._get_conversation_history(conversation_id)

        # Step 5: Generate response using OpenAI
        answer = await self._generate_response(
            question=question,
            context=context,
            conversation_history=conversation_history,
            selected_text=selected_text,
        )

        # Step 6: Save to conversation history
        await self._save_message(
            conversation_id=conversation_id,
            role="user",
            content=question,
            selected_text=selected_text,
        )
        await self._save_message(
            conversation_id=conversation_id,
            role="assistant",
            content=answer,
            sources=[self._chunk_to_source(c) for c in chunks[:3]],
        )

        # Step 7: Format sources for response
        sources = self._format_sources(chunks)

        return {
            "answer": answer,
            "sources": sources,
        }

    def _build_context(
        self, chunks: List[Dict], selected_text: Optional[str] = None
    ) -> str:
        """Build context string from retrieved chunks."""
        context_parts = []

        # Add selected text if provided
        if selected_text:
            context_parts.append(
                f"The user has selected the following text from the book:\n"
                f'"""{selected_text}"""\n'
            )

        # Add retrieved chunks
        for i, chunk in enumerate(chunks, 1):
            source_info = f"[Source {i}: {chunk['title']}"
            if chunk.get("section"):
                source_info += f" - {chunk['section']}"
            source_info += "]"

            context_parts.append(f"{source_info}\n{chunk['content']}")

        return "\n\n---\n\n".join(context_parts)

    def _format_sources(self, chunks: List[Dict]) -> List[Dict]:
        """Format source citations for response."""
        seen = set()
        sources = []

        for chunk in chunks:
            # Deduplicate by document + section
            key = (chunk["document_path"], chunk.get("section", ""))
            if key in seen:
                continue
            seen.add(key)

            # Build URL path
            url_path = chunk["document_path"]
            if url_path.endswith(".md") or url_path.endswith(".mdx"):
                url_path = url_path.rsplit(".", 1)[0]
            url = f"/docs/{url_path}"

            sources.append(
                {
                    "title": chunk["title"],
                    "section": chunk.get("section", ""),
                    "url": url,
                    "relevance_score": round(chunk.get("final_score", chunk["score"]), 3),
                }
            )

        return sources[:3]  # Return top 3 unique sources

    def _chunk_to_source(self, chunk: Dict) -> Dict:
        """Convert chunk to source format for database storage."""
        return {
            "title": chunk["title"],
            "section": chunk.get("section", ""),
            "url": f"/docs/{chunk['document_path'].rsplit('.', 1)[0]}",
            "score": chunk.get("score", 0),
        }

    async def _generate_response(
        self,
        question: str,
        context: str,
        conversation_history: List[Dict],
        selected_text: Optional[str] = None,
    ) -> str:
        """Generate response using Gemini."""
        # Build conversation history for Gemini
        history = []
        for msg in conversation_history[-8:]:
            role = "user" if msg["role"] == "user" else "model"
            history.append({"role": role, "parts": [msg["content"]]})

        # Build user message with context
        user_message = f"""Based on the following context from the Physical AI and Humanoid Robotics textbook, please answer the question.

Context:
{context}

Question: {question}

Please provide a clear, helpful answer based on the context provided. If the context doesn't contain relevant information to answer the question, please say so."""

        # Start chat with history
        chat = self.gemini_model.start_chat(history=history)

        # Combine system prompt with user message
        full_prompt = f"{SYSTEM_PROMPT}\n\n{user_message}"

        # Generate response
        response = await chat.send_message_async(full_prompt)

        return response.text

    async def _get_conversation_history(self, conversation_id: str) -> List[Dict]:
        """Retrieve conversation history from database."""
        try:
            conv_uuid = uuid.UUID(conversation_id)
        except ValueError:
            return []

        result = await self.db.execute(
            select(MessageDB)
            .where(MessageDB.conversation_id == conv_uuid)
            .order_by(MessageDB.created_at)
        )
        messages = result.scalars().all()

        return [{"role": msg.role, "content": msg.content} for msg in messages]

    async def _save_message(
        self,
        conversation_id: Optional[str],
        role: str,
        content: str,
        selected_text: Optional[str] = None,
        sources: Optional[List[Dict]] = None,
    ) -> None:
        """Save message to conversation history."""
        if not conversation_id:
            return

        try:
            conv_uuid = uuid.UUID(conversation_id)
        except ValueError:
            return

        # Check if conversation exists, create if not
        result = await self.db.execute(
            select(Conversation).where(Conversation.id == conv_uuid)
        )
        conversation = result.scalar_one_or_none()

        if not conversation:
            conversation = Conversation(id=conv_uuid)
            self.db.add(conversation)
            await self.db.flush()

        # Create message
        message = MessageDB(
            conversation_id=conv_uuid,
            role=role,
            content=content,
            selected_text=selected_text,
            sources=sources or [],
        )
        self.db.add(message)
        await self.db.flush()
