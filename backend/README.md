# RAG Chatbot Backend

FastAPI backend for the Physical AI and Humanoid Robotics Book RAG chatbot.

## Tech Stack

- **FastAPI** - Async Python web framework
- **OpenAI** - GPT-4o-mini for chat, text-embedding-3-small for vectors
- **Qdrant Cloud** - Vector database (Free Tier)
- **Neon Postgres** - Serverless PostgreSQL for metadata & conversations

## Prerequisites

Before running, you need to set up these services:

1. **OpenAI API Key**: https://platform.openai.com/api-keys
2. **Qdrant Cloud Account**: https://cloud.qdrant.io (create a free cluster)
3. **Neon Postgres Account**: https://neon.tech (create a free database)

## Setup

### 1. Install Dependencies

```bash
cd backend
python -m venv venv

# Windows
venv\Scripts\activate

# macOS/Linux
source venv/bin/activate

pip install -r requirements.txt
```

### 2. Configure Environment

Copy `.env.example` to `.env` and fill in your credentials:

```bash
cp .env.example .env
```

Edit `.env`:
```env
# OpenAI
OPENAI_API_KEY=sk-your-key-here

# Qdrant Cloud
QDRANT_URL=https://your-cluster-id.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres
DATABASE_URL=postgresql://user:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require

# App Config
CORS_ORIGINS=http://localhost:3000,http://localhost:3001
```

### 3. Ingest Documents

Run the ingestion script to process all book content:

```bash
cd backend
python -m scripts.ingest_docs --docs-path ../humanoid-robotics-book/docs
```

Options:
- `--docs-path`: Path to the docs folder (default: `../humanoid-robotics-book/docs`)
- `--force`: Force re-ingestion (deletes existing collection first)

### 4. Run the Server

```bash
uvicorn app.main:app --reload --port 8000
```

The API will be available at: http://localhost:8000

- Swagger docs: http://localhost:8000/docs
- Health check: http://localhost:8000/api/health

## API Endpoints

### POST /api/chat
Send a chat message and get a RAG-powered response.

```json
{
  "message": "What is ROS 2?",
  "conversation_id": "optional-uuid"
}
```

Response:
```json
{
  "answer": "ROS 2 is...",
  "sources": [
    {
      "title": "ROS 2 Architecture",
      "section": "Core Concepts",
      "url": "/docs/module-1-ros2/ros2-architecture",
      "relevance_score": 0.89
    }
  ],
  "conversation_id": "uuid"
}
```

### GET /api/health
Check service health status.

## Deployment to Railway

1. Create a new project on [Railway](https://railway.app)
2. Connect your GitHub repository
3. Add environment variables from `.env`
4. Railway will auto-detect the Dockerfile and deploy

Set the production CORS origins:
```
CORS_ORIGINS=https://your-book.vercel.app
```

## Project Structure

```
backend/
├── app/
│   ├── main.py              # FastAPI entry point
│   ├── config.py            # Configuration
│   ├── api/routes/          # API endpoints
│   ├── db/                  # Database connections
│   ├── models/              # Pydantic & SQLAlchemy models
│   ├── services/            # Business logic
│   └── utils/               # Utilities
├── scripts/
│   └── ingest_docs.py       # Document ingestion
├── requirements.txt
├── Dockerfile
└── .env.example
```

## Development

### Running Tests

```bash
pytest
```

### Code Formatting

```bash
pip install black isort
black app/
isort app/
```
