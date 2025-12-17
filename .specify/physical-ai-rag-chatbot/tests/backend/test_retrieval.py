"""Unit tests for the retrieval service"""

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
from backend.services.retrieval_service import RetrievalService


class TestRetrievalService:
    """Test cases for the RetrievalService class"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        with patch('backend.services.retrieval_service.QdrantClient'):
            with patch('os.getenv', return_value='test_url'):
                self.retrieval_service = RetrievalService()

    @pytest.mark.asyncio
    async def test_retrieve_relevant_chunks(self):
        """Test retrieving relevant chunks for a query embedding"""
        # Mock the Qdrant client search method
        mock_result = Mock()
        mock_result.id = 1
        mock_result.payload = {
            "content": "Test content",
            "source_file": "test.md",
            "chunk_id": "test_0"
        }
        mock_result.score = 0.9

        self.retrieval_service.client.search = Mock(return_value=[mock_result])

        query_embedding = [0.1, 0.2, 0.3]
        chunks = await self.retrieval_service.retrieve_relevant_chunks(query_embedding)

        assert len(chunks) == 1
        assert chunks[0]["id"] == 1
        assert chunks[0]["content"] == "Test content"
        assert chunks[0]["source_file"] == "test.md"
        assert chunks[0]["score"] == 0.9

    @pytest.mark.asyncio
    async def test_retrieve_relevant_chunks_with_selected_text(self):
        """Test retrieving relevant chunks with selected text"""
        # Mock the Qdrant client search method
        mock_result = Mock()
        mock_result.id = 1
        mock_result.payload = {
            "content": "Test content",
            "source_file": "test.md",
            "chunk_id": "test_0"
        }
        mock_result.score = 0.9

        self.retrieval_service.client.search = Mock(return_value=[mock_result])

        query_embedding = [0.1, 0.2, 0.3]
        chunks = await self.retrieval_service.retrieve_relevant_chunks_with_selected_text(
            query_embedding, "selected text"
        )

        assert len(chunks) == 1
        assert chunks[0]["id"] == 1
        assert chunks[0]["content"] == "Test content"
        assert chunks[0]["source_file"] == "test.md"
        assert chunks[0]["score"] == 0.9

    def test_collection_exists(self):
        """Test checking if collection exists"""
        # Mock the get_collections method
        mock_collection = Mock()
        mock_collection.name = "physical_ai_docs"
        self.retrieval_service.client.get_collections = Mock(return_value=Mock(collections=[mock_collection]))

        result = self.retrieval_service.collection_exists()
        assert result is True

    @pytest.mark.asyncio
    async def test_retrieve_relevant_chunks_error_handling(self):
        """Test error handling in retrieve_relevant_chunks"""
        # Mock the client to raise an exception
        self.retrieval_service.client.search = Mock(side_effect=Exception("Connection error"))

        query_embedding = [0.1, 0.2, 0.3]
        with pytest.raises(Exception, match="Error retrieving relevant chunks"):
            await self.retrieval_service.retrieve_relevant_chunks(query_embedding)

    @pytest.mark.asyncio
    async def test_retrieve_relevant_chunks_with_selected_text_error_handling(self):
        """Test error handling in retrieve_relevant_chunks_with_selected_text"""
        # Mock the client to raise an exception
        self.retrieval_service.client.search = Mock(side_effect=Exception("Connection error"))

        query_embedding = [0.1, 0.2, 0.3]
        with pytest.raises(Exception, match="Error retrieving relevant chunks with selected text"):
            await self.retrieval_service.retrieve_relevant_chunks_with_selected_text(
                query_embedding, "selected text"
            )


# Run tests if this file is executed directly
if __name__ == "__main__":
    pytest.main([__file__])