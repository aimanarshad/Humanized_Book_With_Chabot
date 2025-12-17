"""Configuration for the Physical AI RAG chatbot ingestion pipeline"""

import os
from typing import Dict, Any

# Document chunking configuration
CHUNK_CONFIG = {
    "chunk_size": 600,  # tokens (within the 500-800 range specified in the spec)
    "chunk_overlap": 100,  # tokens
    "separators": ["\n\n", "\n", " ", ""],
    "encoding_name": "gpt-3.5-turbo"  # For token counting
}

# Embedding configuration
EMBEDDING_CONFIG = {
    "model": "gemini-embedding-001",
    "vector_size": 768  # Size of Gemini embeddings
}

# Qdrant configuration
QDRANT_CONFIG = {
    "collection_name": "physical_ai_docs",
    "distance_metric": "Cosine",
    "vector_size": EMBEDDING_CONFIG["vector_size"]
}

# Database configuration
DATABASE_CONFIG = {
    "neon_url": os.getenv("NEON_DATABASE_URL", ""),
    "qdrant_url": os.getenv("QDRANT_URL", ""),
    "qdrant_api_key": os.getenv("QDRANT_API_KEY", "")
}

# Paths configuration
PATHS_CONFIG = {
    "docs_directory": "../../../docs",  # Relative to scripts directory
    "source_file_pattern": "**/*.md"
}

def get_config() -> Dict[str, Any]:
    """Get the complete configuration"""
    return {
        "chunk": CHUNK_CONFIG,
        "embedding": EMBEDDING_CONFIG,
        "qdrant": QDRANT_CONFIG,
        "database": DATABASE_CONFIG,
        "paths": PATHS_CONFIG
    }