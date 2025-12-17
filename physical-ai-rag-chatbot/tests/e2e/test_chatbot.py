"""End-to-end tests for the Physical AI RAG chatbot"""

import pytest
import asyncio
from unittest.mock import Mock, patch, MagicMock
from backend.services.rag_service import RAGService
from backend.services.embedding_service import EmbeddingService
from backend.services.retrieval_service import RetrievalService
from backend.utils.citation_formatter import CitationFormatter


class TestChatbotE2E:
    """End-to-end test cases for the chatbot functionality"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        # Mock the services to avoid external dependencies during testing
        self.mock_embedding_service = Mock(spec=EmbeddingService)
        self.mock_retrieval_service = Mock(spec=RetrievalService)

        # Set up mock return values
        self.mock_embedding_service.generate_embedding = Mock(return_value=[0.1, 0.2, 0.3])
        self.mock_retrieval_service.retrieve_relevant_chunks = Mock(return_value=[
            {
                "id": 1,
                "content": "ROS 2 is a flexible framework for writing robot software.",
                "source_file": "module1-ros2.md",
                "chunk_id": "ros2_0",
                "score": 0.9
            }
        ])
        self.mock_retrieval_service.retrieve_relevant_chunks_with_selected_text = Mock(return_value=[
            {
                "id": 1,
                "content": "Selected text content for testing.",
                "source_file": "test.md",
                "chunk_id": "test_0",
                "score": 0.8
            }
        ])

        # Create the RAG service with mocked dependencies
        self.rag_service = RAGService(self.mock_embedding_service, self.mock_retrieval_service)

    @pytest.mark.asyncio
    async def test_basic_query_response(self):
        """Test the complete flow of a basic query"""
        question = "What is ROS 2?"
        conversation_id = "test_conversation_1"

        # Mock the LLM response
        with patch.object(self.rag_service, 'basic_rag_chain') as mock_chain:
            mock_chain.ainvoke = AsyncMock(return_value="ROS 2 is a flexible framework for writing robot software.")

            response = await self.rag_service.generate_response(question, conversation_id)

            # Assertions
            assert "answer" in response
            assert "sources" in response
            assert "conversation_id" in response
            assert "ROS 2" in response["answer"]
            assert response["conversation_id"] == conversation_id
            assert len(response["sources"]) > 0

            # Verify that the embedding and retrieval services were called
            self.mock_embedding_service.generate_embedding.assert_called_once_with(question)
            self.mock_retrieval_service.retrieve_relevant_chunks.assert_called_once()

    @pytest.mark.asyncio
    async def test_selection_based_query_response(self):
        """Test the complete flow of a selection-based query"""
        question = "Explain this concept"
        selected_text = "ROS 2 is a flexible framework for writing robot software."
        conversation_id = "test_conversation_2"

        # Mock the LLM response
        with patch.object(self.rag_service, 'selection_rag_chain') as mock_chain:
            mock_chain.ainvoke = AsyncMock(return_value="This concept refers to ROS 2 as a flexible framework.")

            response = await self.rag_service.generate_response_with_selection(
                question, selected_text, conversation_id
            )

            # Assertions
            assert "answer" in response
            assert "sources" in response
            assert "conversation_id" in response
            assert "selected_text" in response
            assert "ROS 2" in response["answer"]
            assert response["conversation_id"] == conversation_id
            assert response["selected_text"] == selected_text
            assert len(response["sources"]) > 0

    @pytest.mark.asyncio
    async def test_query_with_no_relevant_results(self):
        """Test response when no relevant chunks are found"""
        question = "What is quantum computing?"
        conversation_id = "test_conversation_3"

        # Mock empty retrieval results
        self.mock_retrieval_service.retrieve_relevant_chunks = Mock(return_value=[])

        # Mock the LLM response for when there's no context
        with patch.object(self.rag_service, 'basic_rag_chain') as mock_chain:
            mock_chain.ainvoke = AsyncMock(return_value="I don't have information about quantum computing in the Physical AI book.")

            response = await self.rag_service.generate_response(question, conversation_id)

            # Assertions
            assert "answer" in response
            assert "sources" in response
            assert "conversation_id" in response
            assert response["conversation_id"] == conversation_id
            assert len(response["sources"]) == 0

    def test_citation_formatting(self):
        """Test the citation formatting utility"""
        chunks = [
            {
                "id": 1,
                "content": "ROS 2 is a flexible framework for writing robot software.",
                "source_file": "module1-ros2.md",
                "chunk_id": "ros2_0",
                "score": 0.9,
                "metadata": {}
            },
            {
                "id": 2,
                "content": "Gazebo is a robotics simulator.",
                "source_file": "module2-gazebo.md",
                "chunk_id": "gazebo_0",
                "score": 0.8,
                "metadata": {}
            }
        ]

        citations = CitationFormatter.format_citations(chunks)
        assert len(citations) == 2
        assert "docs/module1-ros2.md" in citations
        assert "docs/module2-gazebo.md" in citations

    def test_detailed_citation_formatting(self):
        """Test the detailed citation formatting utility"""
        chunks = [
            {
                "id": 1,
                "content": "ROS 2 is a flexible framework for writing robot software.",
                "source_file": "module1-ros2.md",
                "chunk_id": "ros2_0",
                "score": 0.9,
                "metadata": {}
            }
        ]

        detailed_citations = CitationFormatter.format_detailed_citations(chunks)
        assert len(detailed_citations) == 1
        citation = detailed_citations[0]
        assert citation["source"] == "docs/module1-ros2.md"
        assert citation["relevance_score"] == 0.9
        assert "ROS 2" in citation["content_preview"]

    def test_citation_string_creation(self):
        """Test creating a formatted citation string"""
        citations = ["docs/module1-ros2.md", "docs/module2-gazebo.md"]
        citation_string = CitationFormatter.create_citation_string(citations)

        assert "Sources cited:" in citation_string
        assert "docs/module1-ros2.md" in citation_string
        assert "docs/module2-gazebo.md" in citation_string

    @pytest.mark.asyncio
    async def test_knowledge_base_validation(self):
        """Test the knowledge base validation functionality"""
        # Mock retrieval service to return some chunks
        self.mock_retrieval_service.retrieve_relevant_chunks = Mock(return_value=[
            {"id": 1, "content": "test", "source_file": "test.md", "chunk_id": "test_0", "score": 0.5}
        ])

        is_valid = await self.rag_service.validate_knowledge_base()
        assert is_valid is True

        # Mock retrieval service to return no chunks
        self.mock_retrieval_service.retrieve_relevant_chunks = Mock(return_value=[])

        is_valid = await self.rag_service.validate_knowledge_base()
        assert is_valid is False

    @pytest.mark.asyncio
    async def test_error_handling_in_generate_response(self):
        """Test error handling in the generate_response method"""
        # Mock embedding service to raise an exception
        self.mock_embedding_service.generate_embedding.side_effect = Exception("API Error")

        with pytest.raises(Exception, match="Error generating response"):
            await self.rag_service.generate_response("test question", "test_conv")

    @pytest.mark.asyncio
    async def test_error_handling_in_generate_response_with_selection(self):
        """Test error handling in the generate_response_with_selection method"""
        # Mock embedding service to raise an exception
        self.mock_embedding_service.generate_embedding.side_effect = Exception("API Error")

        with pytest.raises(Exception, match="Error generating response with selection"):
            await self.rag_service.generate_response_with_selection(
                "test question", "selected text", "test_conv"
            )


# Run tests if this file is executed directly
if __name__ == "__main__":
    pytest.main([__file__])