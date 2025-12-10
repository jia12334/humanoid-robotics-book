"""Markdown parser for extracting content and metadata from documentation files."""

import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass


@dataclass
class ParsedDocument:
    """Represents a parsed markdown document."""

    path: str
    title: str
    module: str
    content: str
    frontmatter: Dict[str, str]
    headings: List[Tuple[int, str]]  # List of (level, heading_text)


class MarkdownParser:
    """Parser for markdown documentation files."""

    # Regex patterns
    FRONTMATTER_PATTERN = re.compile(r"^---\s*\n(.*?)\n---\s*\n", re.DOTALL)
    HEADING_PATTERN = re.compile(r"^(#{1,6})\s+(.+)$", re.MULTILINE)
    CODE_BLOCK_PATTERN = re.compile(r"```[\s\S]*?```", re.MULTILINE)

    def __init__(self, docs_root: Path):
        """
        Initialize parser with docs root directory.

        Args:
            docs_root: Path to the docs/ directory
        """
        self.docs_root = docs_root

    def parse_file(self, file_path: Path) -> ParsedDocument:
        """
        Parse a single markdown file.

        Args:
            file_path: Path to the markdown file

        Returns:
            ParsedDocument with extracted content and metadata
        """
        content = file_path.read_text(encoding="utf-8")

        # Extract frontmatter
        frontmatter = self._extract_frontmatter(content)

        # Remove frontmatter from content
        content_without_frontmatter = self.FRONTMATTER_PATTERN.sub("", content)

        # Extract title
        title = self._extract_title(content_without_frontmatter, frontmatter, file_path)

        # Get module from path
        module = self._get_module(file_path)

        # Extract all headings
        headings = self._extract_headings(content_without_frontmatter)

        # Get relative path
        try:
            relative_path = file_path.relative_to(self.docs_root)
        except ValueError:
            relative_path = file_path

        return ParsedDocument(
            path=str(relative_path),
            title=title,
            module=module,
            content=content_without_frontmatter,
            frontmatter=frontmatter,
            headings=headings,
        )

    def _extract_frontmatter(self, content: str) -> Dict[str, str]:
        """Extract YAML frontmatter from markdown content."""
        frontmatter = {}
        match = self.FRONTMATTER_PATTERN.match(content)

        if match:
            fm_content = match.group(1)
            for line in fm_content.split("\n"):
                if ":" in line:
                    key, value = line.split(":", 1)
                    frontmatter[key.strip()] = value.strip().strip('"').strip("'")

        return frontmatter

    def _extract_title(
        self, content: str, frontmatter: Dict[str, str], file_path: Path
    ) -> str:
        """
        Extract document title from frontmatter, first heading, or filename.

        Priority:
        1. Frontmatter 'title' field
        2. First H1 heading
        3. Filename (converted from slug)
        """
        # Check frontmatter
        if "title" in frontmatter:
            return frontmatter["title"]

        # Check for first H1
        h1_match = re.search(r"^#\s+(.+)$", content, re.MULTILINE)
        if h1_match:
            return h1_match.group(1).strip()

        # Fall back to filename
        return self._slug_to_title(file_path.stem)

    def _slug_to_title(self, slug: str) -> str:
        """Convert a filename slug to a human-readable title."""
        # Remove common prefixes like numbers
        slug = re.sub(r"^\d+[-_]", "", slug)

        # Replace hyphens and underscores with spaces
        title = slug.replace("-", " ").replace("_", " ")

        # Capitalize words
        return title.title()

    def _get_module(self, file_path: Path) -> str:
        """
        Get module name from file path.

        Examples:
            docs/module-1-ros2/file.md -> module-1-ros2
            docs/introduction/file.md -> introduction
            docs/glossary.md -> general
        """
        try:
            relative_path = file_path.relative_to(self.docs_root)
            parts = relative_path.parts

            if len(parts) > 1:
                return parts[0]
            else:
                return "general"
        except ValueError:
            return "general"

    def _extract_headings(self, content: str) -> List[Tuple[int, str]]:
        """Extract all headings with their levels."""
        headings = []
        for match in self.HEADING_PATTERN.finditer(content):
            level = len(match.group(1))
            text = match.group(2).strip()
            headings.append((level, text))
        return headings

    def scan_documents(self) -> List[Path]:
        """
        Scan docs directory for all markdown files.

        Returns:
            List of paths to markdown files
        """
        md_files = list(self.docs_root.rglob("*.md"))
        mdx_files = list(self.docs_root.rglob("*.mdx"))
        return sorted(md_files + mdx_files)

    def parse_all(self) -> List[ParsedDocument]:
        """
        Parse all markdown files in the docs directory.

        Returns:
            List of ParsedDocument objects
        """
        documents = []
        for file_path in self.scan_documents():
            try:
                doc = self.parse_file(file_path)
                documents.append(doc)
            except Exception as e:
                print(f"Warning: Could not parse {file_path}: {e}")
        return documents
