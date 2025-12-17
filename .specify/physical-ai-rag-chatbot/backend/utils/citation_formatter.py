"""Utility for formatting citations from retrieved document chunks"""

from typing import List, Dict, Any
import re

class CitationFormatter:
    """Utility class for formatting citations from retrieved document chunks"""

    @staticmethod
    def format_citations(chunks: List[Dict[str, Any]]) -> List[str]:
        """Format citations from retrieved chunks"""
        citations = []
        seen_sources = set()

        for chunk in chunks:
            source_file = chunk.get("source_file", "")
            if source_file and source_file not in seen_sources:
                # Format the citation - for now we'll just use the source file name
                # In a more advanced implementation, we might include line numbers or other metadata
                formatted_citation = f"docs/{source_file}"
                citations.append(formatted_citation)
                seen_sources.add(source_file)

        return citations

    @staticmethod
    def format_detailed_citations(chunks: List[Dict[str, Any]]) -> List[Dict[str, str]]:
        """Format detailed citations with additional metadata"""
        citations = []
        seen_sources = set()

        for chunk in chunks:
            source_file = chunk.get("source_file", "")
            if source_file and source_file not in seen_sources:
                citation = {
                    "source": f"docs/{source_file}",
                    "relevance_score": chunk.get("score", 0.0),
                    "content_preview": chunk.get("content", "")[:100] + "..." if len(chunk.get("content", "")) > 100 else chunk.get("content", "")
                }
                citations.append(citation)
                seen_sources.add(source_file)

        return citations

    @staticmethod
    def extract_line_numbers(content: str, search_term: str) -> List[int]:
        """Extract line numbers where a search term appears in the content"""
        lines = content.split('\n')
        line_numbers = []
        for i, line in enumerate(lines, 1):
            if search_term.lower() in line.lower():
                line_numbers.append(i)
        return line_numbers

    @staticmethod
    def create_citation_string(citations: List[str]) -> str:
        """Create a formatted string of citations"""
        if not citations:
            return ""

        citation_text = "\n\nSources cited:\n"
        for i, citation in enumerate(citations, 1):
            citation_text += f"{i}. {citation}\n"

        return citation_text