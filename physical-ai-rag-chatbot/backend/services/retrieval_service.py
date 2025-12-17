"""Retrieval service for the Physical AI RAG chatbot"""

import os
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class RetrievalService:
    """Service for retrieving relevant documents using vector similarity search"""

    def __init__(self):
        """Initialize the retrieval service"""
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not self.qdrant_url:
            raise ValueError("QDRANT_URL environment variable is not set")

        # Initialize Qdrant client
        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            timeout=10
        )

        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_docs")
        self.top_k = int(os.getenv("RETRIEVAL_TOP_K", "5"))  # Number of documents to retrieve

    async def retrieve_relevant_chunks(self, query_embedding: List[float], top_k: int = None) -> List[Dict[str, Any]]:
        """Retrieve the most relevant document chunks for a query embedding"""
        if top_k is None:
            top_k = self.top_k

        try:
            # Perform vector similarity search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True
            )

            # Format results
            retrieved_chunks = []
            for result in search_results:
                chunk = {
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "source_file": result.payload.get("source_file", ""),
                    "chunk_id": result.payload.get("chunk_id", ""),
                    "score": result.score,
                    "metadata": {k: v for k, v in result.payload.items()
                               if k not in ["content", "source_file", "chunk_id"]}
                }
                retrieved_chunks.append(chunk)

            return retrieved_chunks

        except Exception as e:
            raise Exception(f"Error retrieving relevant chunks: {str(e)}")

    async def retrieve_relevant_chunks_with_selected_text(
        self,
        query_embedding: List[float],
        selected_text: str,
        top_k: int = None
    ) -> List[Dict[str, Any]]:
        """Retrieve relevant chunks, with special handling for selected text queries"""
        if top_k is None:
            top_k = self.top_k

        try:
            # First, try to find chunks that contain or are highly related to the selected text
            # For now, we'll use the query embedding as before, but in a more targeted way
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True,
                # Add a filter to prioritize chunks that might be related to the selected text
                # This could be enhanced with more sophisticated filtering
            )

            # Format results
            retrieved_chunks = []
            for result in search_results:
                chunk = {
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "source_file": result.payload.get("source_file", ""),
                    "chunk_id": result.payload.get("chunk_id", ""),
                    "score": result.score,
                    "metadata": {k: v for k, v in result.payload.items()
                               if k not in ["content", "source_file", "chunk_id"]}
                }
                retrieved_chunks.append(chunk)

            return retrieved_chunks

        except Exception as e:
            raise Exception(f"Error retrieving relevant chunks with selected text: {str(e)}")

    def collection_exists(self) -> bool:
        """Check if the collection exists in Qdrant"""
        try:
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]
            return self.collection_name in collection_names
        except Exception:
            return False