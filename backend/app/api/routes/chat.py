"""Chat endpoint for RAG Q&A."""

import uuid
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession

from app.models.schemas import ChatRequest, ChatResponse, Source
from app.db.postgres import get_db
from app.services.rag_service import RAGService

router = APIRouter(prefix="/chat", tags=["chat"])


@router.post("", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    db: AsyncSession = Depends(get_db),
) -> ChatResponse:
    """
    Process a chat message and return a RAG-powered response.

    Args:
        request: Chat request with user message and optional conversation ID

    Returns:
        ChatResponse with answer, sources, and conversation ID
    """
    try:
        # Initialize RAG service
        rag_service = RAGService(db)

        # Generate conversation ID if not provided
        conversation_id = request.conversation_id or str(uuid.uuid4())

        # Process the query through RAG pipeline
        result = await rag_service.answer_question(
            question=request.message,
            conversation_id=conversation_id,
        )

        return ChatResponse(
            answer=result["answer"],
            sources=[
                Source(
                    title=s["title"],
                    section=s["section"],
                    url=s["url"],
                    relevance_score=s["relevance_score"],
                )
                for s in result["sources"]
            ],
            conversation_id=conversation_id,
        )

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error processing chat request: {str(e)}",
        )
