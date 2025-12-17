"""Integration tests for the API endpoints"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, Mock
from backend.main import app


class TestAPIEndpoints:
    """Test cases for the API endpoints"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        self.client = TestClient(app)

    @patch('backend.main.rag_service')
    @patch('backend.main.api_key_auth')
    def test_query_endpoint(self, mock_api_key_auth, mock_rag_service):
        """Test the /v1/query endpoint"""
        # Mock authentication to pass
        mock_api_key_auth.return_value = True

        # Mock RAG service response
        mock_rag_service.generate_response = Mock(return_value={
            "answer": "Test answer",
            "sources": ["docs/test.md"],
            "conversation_id": "test_conversation_id"
        })

        response = self.client.post(
            "/v1/query",
            json={"question": "What is ROS 2?", "conversation_id": "test_conv"},
            headers={"X-API-Key": "test_key"}
        )

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert data["answer"] == "Test answer"

    @patch('backend.main.rag_service')
    @patch('backend.main.api_key_auth')
    def test_query_with_selection_endpoint(self, mock_api_key_auth, mock_rag_service):
        """Test the /v1/query-with-selection endpoint"""
        # Mock authentication to pass
        mock_api_key_auth.return_value = True

        # Mock RAG service response
        mock_rag_service.generate_response_with_selection = Mock(return_value={
            "answer": "Test answer from selection",
            "sources": ["docs/test.md"],
            "conversation_id": "test_conversation_id",
            "selected_text": "selected text"
        })

        response = self.client.post(
            "/v1/query-with-selection",
            json={
                "question": "Explain this",
                "selected_text": "selected text",
                "conversation_id": "test_conv"
            },
            headers={"X-API-Key": "test_key"}
        )

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert data["answer"] == "Test answer from selection"

    @patch('backend.main.get_conversation_history')
    @patch('backend.main.get_conversation')
    @patch('backend.main.api_key_auth')
    def test_get_conversation_endpoint(self, mock_api_key_auth, mock_get_conversation, mock_get_conversation_history):
        """Test the /v1/conversation/{id} endpoint"""
        # Mock authentication to pass
        mock_api_key_auth.return_value = True

        # Mock conversation exists
        mock_get_conversation.return_value = Mock()

        # Mock conversation history
        mock_get_conversation_history.return_value = [
            {"message_id": "msg1", "sender": "user", "text_content": "Hello", "timestamp": "2023-01-01T00:00:00"}
        ]

        response = self.client.get("/v1/conversation/test_conv", headers={"X-API-Key": "test_key"})

        assert response.status_code == 200
        data = response.json()
        assert "conversation_id" in data
        assert "history" in data
        assert len(data["history"]) == 1

    @patch('backend.main.create_feedback')
    @patch('backend.main.api_key_auth')
    def test_feedback_endpoint(self, mock_api_key_auth, mock_create_feedback):
        """Test the /v1/feedback endpoint"""
        # Mock authentication to pass
        mock_api_key_auth.return_value = True

        # Mock feedback creation
        mock_feedback = Mock()
        mock_feedback.feedback_id = "test_feedback_id"
        mock_create_feedback.return_value = mock_feedback

        response = self.client.post(
            "/v1/feedback",
            json={
                "message_id": "test_msg",
                "rating": "positive",
                "comment": "Great response!"
            },
            headers={"X-API-Key": "test_key"}
        )

        assert response.status_code == 200
        data = response.json()
        assert "feedback_id" in data
        assert data["status"] == "success"

    @patch('backend.main.api_key_auth')
    def test_feedback_endpoint_invalid_rating(self, mock_api_key_auth):
        """Test the /v1/feedback endpoint with invalid rating"""
        # Mock authentication to pass
        mock_api_key_auth.return_value = True

        response = self.client.post(
            "/v1/feedback",
            json={
                "message_id": "test_msg",
                "rating": "invalid_rating",
                "comment": "Test comment"
            },
            headers={"X-API-Key": "test_key"}
        )

        assert response.status_code == 400

    def test_health_check_endpoint(self):
        """Test the health check endpoint"""
        response = self.client.get("/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert data["service"] == "Physical AI RAG Chatbot API"

    @patch('backend.main.rag_service')
    @patch('backend.main.api_key_auth')
    def test_query_endpoint_missing_api_key(self, mock_api_key_auth, mock_rag_service):
        """Test the /v1/query endpoint without API key"""
        # Mock authentication to fail
        mock_api_key_auth.side_effect = Exception("API key is missing")

        response = self.client.post(
            "/v1/query",
            json={"question": "What is ROS 2?", "conversation_id": "test_conv"}
        )

        assert response.status_code == 401

    @patch('backend.main.rag_service')
    @patch('backend.main.api_key_auth')
    def test_query_endpoint_error_handling(self, mock_api_key_auth, mock_rag_service):
        """Test error handling in /v1/query endpoint"""
        # Mock authentication to pass
        mock_api_key_auth.return_value = True

        # Mock RAG service to raise an exception
        mock_rag_service.generate_response.side_effect = Exception("Test error")

        response = self.client.post(
            "/v1/query",
            json={"question": "What is ROS 2?", "conversation_id": "test_conv"},
            headers={"X-API-Key": "test_key"}
        )

        assert response.status_code == 500


# Run tests if this file is executed directly
if __name__ == "__main__":
    pytest.main([__file__])