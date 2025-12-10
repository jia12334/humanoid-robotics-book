"""Health check endpoint."""

from datetime import datetime
from fastapi import APIRouter

from app.models.schemas import HealthResponse
from app.db.qdrant import check_qdrant_health
from app.db.postgres import engine

router = APIRouter(tags=["health"])


@router.get("/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """
    Check the health status of all services.

    Returns:
        HealthResponse with status of each service
    """
    services = {}

    # Check Qdrant
    try:
        qdrant_healthy = await check_qdrant_health()
        services["qdrant"] = "connected" if qdrant_healthy else "disconnected"
    except Exception as e:
        services["qdrant"] = f"error: {str(e)}"

    # Check PostgreSQL
    try:
        async with engine.connect() as conn:
            await conn.execute("SELECT 1")
        services["postgres"] = "connected"
    except Exception as e:
        services["postgres"] = f"error: {str(e)}"

    # Check OpenAI (basic check - API key presence)
    from app.config import get_settings

    settings = get_settings()
    services["openai"] = (
        "configured" if settings.openai_api_key.startswith("sk-") else "not configured"
    )

    # Determine overall status
    all_healthy = all(
        status in ["connected", "configured"] for status in services.values()
    )
    overall_status = "healthy" if all_healthy else "degraded"

    return HealthResponse(
        status=overall_status,
        services=services,
        timestamp=datetime.utcnow(),
    )
