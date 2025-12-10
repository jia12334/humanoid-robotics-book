"""
Document Ingestion Script

This script processes all markdown files from the Docusaurus docs folder,
chunks them, generates embeddings, and stores them in Qdrant.

Usage:
    cd backend
    python -m scripts.ingest_docs

Or with custom path:
    python -m scripts.ingest_docs --docs-path ../humanoid-robotics-book/docs
"""

import asyncio
import argparse
import sys
import uuid
from pathlib import Path
from datetime import datetime
from typing import List

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv

load_dotenv()

from qdrant_client.http import models as qdrant_models

from app.config import get_settings
from app.utils.markdown_parser import MarkdownParser, ParsedDocument
from app.utils.chunking import SemanticChunker, Chunk
from app.services.embedding_service import EmbeddingService
from app.db.qdrant import get_qdrant_client, init_qdrant_collection

settings = get_settings()


async def ingest_documents(docs_path: Path, force_reingest: bool = False) -> dict:
    """
    Main ingestion pipeline.

    Args:
        docs_path: Path to the docs folder
        force_reingest: If True, delete existing collection and reingest

    Returns:
        Statistics about the ingestion
    """
    start_time = datetime.now()
    stats = {
        "documents_processed": 0,
        "chunks_created": 0,
        "errors": [],
    }

    print(f"Starting document ingestion from: {docs_path}")
    print(f"Target Qdrant collection: {settings.qdrant_collection_name}")

    # Initialize Qdrant collection
    client = get_qdrant_client()

    if force_reingest:
        print("Force reingest: deleting existing collection...")
        try:
            client.delete_collection(settings.qdrant_collection_name)
        except Exception:
            pass  # Collection might not exist

    await init_qdrant_collection()

    # Initialize services
    parser = MarkdownParser(docs_path)
    chunker = SemanticChunker(
        min_chunk_size=200,
        max_chunk_size=800,
        overlap=50,
    )
    embedding_service = EmbeddingService()

    # Scan and parse documents
    print("\nScanning for markdown files...")
    documents = parser.parse_all()
    print(f"Found {len(documents)} documents")

    # Process each document
    all_chunks: List[Chunk] = []

    for doc in documents:
        try:
            chunks = chunker.chunk_document(doc)
            all_chunks.extend(chunks)
            stats["documents_processed"] += 1
            print(f"  Processed: {doc.path} ({len(chunks)} chunks)")
        except Exception as e:
            error_msg = f"Error processing {doc.path}: {str(e)}"
            stats["errors"].append(error_msg)
            print(f"  ERROR: {error_msg}")

    stats["chunks_created"] = len(all_chunks)
    print(f"\nTotal chunks created: {len(all_chunks)}")

    if not all_chunks:
        print("No chunks to process. Exiting.")
        return stats

    # Generate embeddings in batches
    print("\nGenerating embeddings...")
    batch_size = 50
    all_embeddings = []

    for i in range(0, len(all_chunks), batch_size):
        batch = all_chunks[i : i + batch_size]
        texts = [c.content for c in batch]

        print(f"  Embedding batch {i // batch_size + 1}/{(len(all_chunks) + batch_size - 1) // batch_size}...")
        embeddings = await embedding_service.embed_batch(texts)
        all_embeddings.extend(embeddings)

    # Upload to Qdrant
    print("\nUploading to Qdrant...")

    points = []
    for chunk, embedding in zip(all_chunks, all_embeddings):
        # Convert string chunk_id to UUID (Qdrant requires UUID or integer IDs)
        point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, chunk.chunk_id))
        point = qdrant_models.PointStruct(
            id=point_id,
            vector=embedding,
            payload={
                "chunk_id": chunk.chunk_id,
                "document_path": chunk.document_path,
                "title": chunk.title,
                "section": chunk.section,
                "module": chunk.module,
                "content": chunk.content,
                "headings": chunk.headings,
                "chunk_index": chunk.chunk_index,
                "content_length": chunk.content_length,
            },
        )
        points.append(point)

    # Upload in batches (smaller batches for slow connections)
    upload_batch_size = 20
    for i in range(0, len(points), upload_batch_size):
        batch = points[i : i + upload_batch_size]
        try:
            client.upsert(
                collection_name=settings.qdrant_collection_name,
                points=batch,
            )
            print(f"  Uploaded {min(i + upload_batch_size, len(points))}/{len(points)} points")
        except Exception as e:
            print(f"  Retry batch {i // upload_batch_size + 1}...")
            import time
            time.sleep(2)
            client.upsert(
                collection_name=settings.qdrant_collection_name,
                points=batch,
            )
            print(f"  Uploaded {min(i + upload_batch_size, len(points))}/{len(points)} points (retry succeeded)")

    # Final statistics
    end_time = datetime.now()
    duration = (end_time - start_time).total_seconds()

    print("\n" + "=" * 50)
    print("INGESTION COMPLETE")
    print("=" * 50)
    print(f"Documents processed: {stats['documents_processed']}")
    print(f"Chunks created: {stats['chunks_created']}")
    print(f"Duration: {duration:.2f} seconds")

    if stats["errors"]:
        print(f"\nErrors ({len(stats['errors'])}):")
        for error in stats["errors"]:
            print(f"  - {error}")

    # Verify upload
    collection_info = client.get_collection(settings.qdrant_collection_name)
    print(f"\nQdrant collection '{settings.qdrant_collection_name}' now has {collection_info.points_count} points")

    return stats


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Ingest documentation into Qdrant vector database"
    )
    parser.add_argument(
        "--docs-path",
        type=str,
        default="../humanoid-robotics-book/docs",
        help="Path to the docs folder (default: ../humanoid-robotics-book/docs)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force reingest (delete existing collection first)",
    )

    args = parser.parse_args()
    docs_path = Path(args.docs_path).resolve()

    if not docs_path.exists():
        print(f"Error: Docs path does not exist: {docs_path}")
        sys.exit(1)

    if not docs_path.is_dir():
        print(f"Error: Docs path is not a directory: {docs_path}")
        sys.exit(1)

    # Run ingestion
    asyncio.run(ingest_documents(docs_path, force_reingest=args.force))


if __name__ == "__main__":
    main()
