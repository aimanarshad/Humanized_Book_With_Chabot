"""Embedding service for the Physical AI RAG chatbot"""

import os
from typing import List
from langchain_google_genai import GoogleGenerativeAIEmbeddings
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class EmbeddingService:
    """Service for generating embeddings using Google Generative AI API"""

    def __init__(self):
        """Initialize the embedding service"""
        self.api_key = os.getenv("GEMINI_API_KEY", os.getenv("OPENAI_API_KEY"))  # Use GEMINI_API_KEY if available, fallback to OPENAI_API_KEY
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY or OPENAI_API_KEY environment variable is not set")

        self.model = os.getenv("EMBEDDING_MODEL", "text-embedding-004")
        self.embeddings = GoogleGenerativeAIEmbeddings(model=self.model, api_key=self.api_key)

    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a single text"""
        try:
            embedding = self.embeddings.embed_query(text)
            return embedding
        except Exception as e:
            raise Exception(f"Error generating embedding: {str(e)}")

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a list of texts"""
        try:
            embeddings = self.embeddings.embed_documents(texts)
            return embeddings
        except Exception as e:
            raise Exception(f"Error generating embeddings: {str(e)}")

    def get_embedding_dimension(self) -> int:
        """Get the dimension of the embeddings"""
        # text-embedding-004 has 768 dimensions
        # text-embedding-005 (when available) will have 768 dimensions as well
        return 768