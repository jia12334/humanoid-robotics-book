"""Semantic chunking for document content."""

import re
import hashlib
from typing import Dict, List, Optional
from dataclasses import dataclass

import tiktoken

from .markdown_parser import ParsedDocument


@dataclass
class Chunk:
    """Represents a document chunk for embedding."""

    chunk_id: str
    document_path: str
    title: str
    section: str
    module: str
    content: str
    headings: List[str]
    chunk_index: int
    content_length: int
    content_hash: str


class SemanticChunker:
    """
    Semantic chunker that respects document structure.

    Features:
    - Respects heading boundaries
    - Preserves code blocks as single units
    - Maintains paragraph integrity
    - Configurable size with overlap
    """

    # Patterns for splitting
    HEADING_PATTERN = re.compile(r"^(#{2,4})\s+(.+)$", re.MULTILINE)
    CODE_BLOCK_PATTERN = re.compile(r"```[\s\S]*?```")
    PARAGRAPH_SEPARATOR = "\n\n"

    def __init__(
        self,
        min_chunk_size: int = 200,
        max_chunk_size: int = 1000,
        overlap: int = 100,
        encoding_name: str = "cl100k_base",
    ):
        """
        Initialize chunker with size constraints.

        Args:
            min_chunk_size: Minimum chunk size in tokens
            max_chunk_size: Maximum chunk size in tokens
            overlap: Number of overlapping tokens between chunks
            encoding_name: Tiktoken encoding for token counting
        """
        self.min_chunk_size = min_chunk_size
        self.max_chunk_size = max_chunk_size
        self.overlap = overlap
        self.encoding = tiktoken.get_encoding(encoding_name)

    def count_tokens(self, text: str) -> int:
        """Count tokens in text using tiktoken."""
        return len(self.encoding.encode(text))

    def chunk_document(self, document: ParsedDocument) -> List[Chunk]:
        """
        Chunk a parsed document into semantic chunks.

        Args:
            document: ParsedDocument to chunk

        Returns:
            List of Chunk objects
        """
        chunks = []
        sections = self._split_by_headings(document.content)

        chunk_index = 0
        for section in sections:
            section_chunks = self._chunk_section(section)

            for chunk_content in section_chunks:
                # Generate unique chunk ID
                chunk_id = self._generate_chunk_id(document.path, chunk_index)

                # Calculate content hash
                content_hash = hashlib.sha256(chunk_content.encode()).hexdigest()[:16]

                chunk = Chunk(
                    chunk_id=chunk_id,
                    document_path=document.path,
                    title=document.title,
                    section=section["heading"],
                    module=document.module,
                    content=chunk_content,
                    headings=section["heading_path"],
                    chunk_index=chunk_index,
                    content_length=self.count_tokens(chunk_content),
                    content_hash=content_hash,
                )
                chunks.append(chunk)
                chunk_index += 1

        return chunks

    def _generate_chunk_id(self, document_path: str, chunk_index: int) -> str:
        """Generate a unique chunk ID."""
        # Clean path for ID
        clean_path = document_path.replace("/", "-").replace("\\", "-")
        clean_path = re.sub(r"\.mdx?$", "", clean_path)
        return f"{clean_path}-chunk-{chunk_index}"

    def _split_by_headings(self, content: str) -> List[Dict]:
        """
        Split content by markdown headings (##, ###, ####).

        Returns list of sections with heading info.
        """
        sections = []
        current_section = {
            "heading": "Introduction",
            "content": "",
            "heading_path": [],
            "level": 1,
        }
        heading_stack = []

        lines = content.split("\n")
        i = 0

        while i < len(lines):
            line = lines[i]
            match = self.HEADING_PATTERN.match(line)

            if match:
                # Save previous section if it has content
                if current_section["content"].strip():
                    sections.append(current_section)

                level = len(match.group(1))
                heading_text = match.group(2).strip()

                # Update heading stack
                heading_stack = self._update_heading_stack(
                    heading_stack, level, heading_text
                )

                current_section = {
                    "heading": heading_text,
                    "content": "",
                    "heading_path": [h[1] for h in heading_stack],
                    "level": level,
                }
            else:
                current_section["content"] += line + "\n"

            i += 1

        # Don't forget the last section
        if current_section["content"].strip():
            sections.append(current_section)

        return sections

    def _update_heading_stack(
        self, stack: List[tuple], level: int, heading: str
    ) -> List[tuple]:
        """Update heading stack for nested heading tracking."""
        # Remove headings at same or lower level
        while stack and stack[-1][0] >= level:
            stack.pop()

        stack.append((level, heading))
        return stack

    def _chunk_section(self, section: Dict) -> List[str]:
        """
        Chunk a section's content while preserving structure.

        Handles:
        - Code blocks (kept as single units)
        - Paragraph boundaries
        - Size constraints
        """
        content = section["content"]
        chunks = []

        # Protect code blocks by replacing with placeholders
        code_blocks = []
        protected_content = content

        def replace_code_block(match):
            idx = len(code_blocks)
            code_blocks.append(match.group(0))
            return f"__CODE_BLOCK_{idx}__"

        protected_content = self.CODE_BLOCK_PATTERN.sub(
            replace_code_block, protected_content
        )

        # Split into paragraphs
        paragraphs = protected_content.split(self.PARAGRAPH_SEPARATOR)

        current_chunk = ""
        current_tokens = 0

        for para in paragraphs:
            para = para.strip()
            if not para:
                continue

            # Restore code blocks in paragraph
            for idx, code_block in enumerate(code_blocks):
                para = para.replace(f"__CODE_BLOCK_{idx}__", code_block)

            para_tokens = self.count_tokens(para)

            # If paragraph alone exceeds max size, split it
            if para_tokens > self.max_chunk_size:
                # Save current chunk if exists
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())
                    current_chunk = ""
                    current_tokens = 0

                # Split large paragraph
                split_chunks = self._split_large_paragraph(para)
                chunks.extend(split_chunks)
                continue

            # Check if adding this paragraph exceeds max size
            if current_tokens + para_tokens > self.max_chunk_size:
                # Save current chunk if it meets minimum size
                if current_tokens >= self.min_chunk_size:
                    chunks.append(current_chunk.strip())

                    # Start new chunk with overlap from previous
                    overlap_text = self._get_overlap_text(current_chunk)
                    current_chunk = overlap_text + "\n\n" + para if overlap_text else para
                    current_tokens = self.count_tokens(current_chunk)
                else:
                    # Current chunk too small, just add paragraph
                    current_chunk += "\n\n" + para
                    current_tokens += para_tokens
            else:
                # Add paragraph to current chunk
                if current_chunk:
                    current_chunk += "\n\n" + para
                else:
                    current_chunk = para
                current_tokens += para_tokens

        # Don't forget the last chunk
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        # Filter out empty chunks and very small ones
        chunks = [c for c in chunks if c.strip() and self.count_tokens(c) >= 50]

        return chunks

    def _split_large_paragraph(self, text: str) -> List[str]:
        """Split a large paragraph that exceeds max chunk size."""
        chunks = []
        sentences = re.split(r"(?<=[.!?])\s+", text)

        current_chunk = ""
        current_tokens = 0

        for sentence in sentences:
            sentence_tokens = self.count_tokens(sentence)

            if current_tokens + sentence_tokens > self.max_chunk_size:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = sentence
                current_tokens = sentence_tokens
            else:
                current_chunk += " " + sentence if current_chunk else sentence
                current_tokens += sentence_tokens

        if current_chunk:
            chunks.append(current_chunk.strip())

        return chunks

    def _get_overlap_text(self, text: str) -> str:
        """Get overlap text from the end of a chunk."""
        if self.overlap <= 0:
            return ""

        tokens = self.encoding.encode(text)
        if len(tokens) <= self.overlap:
            return text

        overlap_tokens = tokens[-self.overlap :]
        return self.encoding.decode(overlap_tokens)
