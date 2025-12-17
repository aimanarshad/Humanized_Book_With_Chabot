"""Unit tests for database operations"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from backend.db.models import Base, Conversation, Message, Feedback
from backend.db.crud import (
    create_conversation, get_conversation, update_conversation_last_message,
    create_message, get_messages_by_conversation, get_conversation_history,
    create_feedback, get_feedback_by_message, conversation_exists, message_exists,
    get_all_conversations
)
from datetime import datetime


class TestDatabaseOperations:
    """Test cases for database operations"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        # Create an in-memory SQLite database for testing
        self.engine = create_engine("sqlite:///:memory:")
        Base.metadata.create_all(self.engine)
        SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=self.engine)
        self.db = SessionLocal()

    def teardown_method(self):
        """Clean up after each test method"""
        self.db.close()

    def test_create_conversation(self):
        """Test creating a conversation"""
        conversation = create_conversation(self.db, user_id="test_user")

        assert conversation is not None
        assert conversation.user_id == "test_user"
        assert conversation.conversation_id is not None
        assert conversation.timestamp_started is not None

    def test_get_conversation(self):
        """Test getting a conversation by ID"""
        # Create a conversation first
        created_conversation = create_conversation(self.db, user_id="test_user")

        # Get the conversation
        retrieved_conversation = get_conversation(self.db, created_conversation.conversation_id)

        assert retrieved_conversation is not None
        assert retrieved_conversation.conversation_id == created_conversation.conversation_id
        assert retrieved_conversation.user_id == "test_user"

    def test_create_message(self):
        """Test creating a message in a conversation"""
        # Create a conversation first
        conversation = create_conversation(self.db)

        # Create a message
        message = create_message(
            self.db,
            conversation_id=conversation.conversation_id,
            sender="user",
            text_content="Hello, world!",
            selected_text="selected text"
        )

        assert message is not None
        assert message.sender == "user"
        assert message.text_content == "Hello, world!"
        assert message.selected_text == "selected text"

    def test_get_messages_by_conversation(self):
        """Test getting all messages for a conversation"""
        # Create a conversation
        conversation = create_conversation(self.db)

        # Create some messages
        create_message(self.db, conversation_id=conversation.conversation_id, sender="user", text_content="Message 1")
        create_message(self.db, conversation_id=conversation.conversation_id, sender="bot", text_content="Message 2")

        # Get messages
        messages = get_messages_by_conversation(self.db, conversation.conversation_id)

        assert len(messages) == 2
        assert messages[0].sender == "user"
        assert messages[1].sender == "bot"

    def test_get_conversation_history(self):
        """Test getting formatted conversation history"""
        # Create a conversation
        conversation = create_conversation(self.db)

        # Create some messages
        create_message(self.db, conversation_id=conversation.conversation_id, sender="user", text_content="Message 1")
        create_message(self.db, conversation_id=conversation.conversation_id, sender="bot", text_content="Message 2")

        # Get conversation history
        history = get_conversation_history(self.db, conversation.conversation_id)

        assert len(history) == 2
        assert history[0]["sender"] == "user"
        assert history[0]["text_content"] == "Message 1"
        assert history[1]["sender"] == "bot"
        assert history[1]["text_content"] == "Message 2"

    def test_create_feedback(self):
        """Test creating feedback for a message"""
        # Create a conversation and message first
        conversation = create_conversation(self.db)
        message = create_message(self.db, conversation_id=conversation.conversation_id, sender="user", text_content="Test message")

        # Create feedback
        feedback = create_feedback(self.db, message_id=message.message_id, rating="positive", user_id="test_user", comment="Great response!")

        assert feedback is not None
        assert feedback.rating == "positive"
        assert feedback.user_id == "test_user"
        assert feedback.comment == "Great response!"

    def test_get_feedback_by_message(self):
        """Test getting feedback for a specific message"""
        # Create a conversation and message first
        conversation = create_conversation(self.db)
        message = create_message(self.db, conversation_id=conversation.conversation_id, sender="user", text_content="Test message")

        # Create feedback
        create_feedback(self.db, message_id=message.message_id, rating="positive", comment="Good")

        # Get feedback
        retrieved_feedback = get_feedback_by_message(self.db, message.message_id)

        assert retrieved_feedback is not None
        assert retrieved_feedback.rating == "positive"
        assert retrieved_feedback.comment == "Good"

    def test_conversation_exists(self):
        """Test checking if a conversation exists"""
        # Create a conversation
        conversation = create_conversation(self.db)

        # Check if it exists
        exists = conversation_exists(self.db, conversation.conversation_id)
        assert exists is True

        # Check for non-existent conversation
        exists = conversation_exists(self.db, "non_existent_id")
        assert exists is False

    def test_message_exists(self):
        """Test checking if a message exists"""
        # Create a conversation and message
        conversation = create_conversation(self.db)
        message = create_message(self.db, conversation_id=conversation.conversation_id, sender="user", text_content="Test message")

        # Check if it exists
        exists = message_exists(self.db, message.message_id)
        assert exists is True

        # Check for non-existent message
        exists = message_exists(self.db, "non_existent_id")
        assert exists is False

    def test_get_all_conversations(self):
        """Test getting all conversations"""
        # Create a few conversations
        conv1 = create_conversation(self.db, user_id="user1")
        conv2 = create_conversation(self.db, user_id="user2")
        conv3 = create_conversation(self.db)  # No user_id

        # Get all conversations
        all_convs = get_all_conversations(self.db)
        assert len(all_convs) == 3

        # Get conversations for specific user
        user1_convs = get_all_conversations(self.db, user_id="user1")
        assert len(user1_convs) == 1
        assert user1_convs[0].conversation_id == conv1.conversation_id

    def test_update_conversation_last_message(self):
        """Test updating the last message timestamp for a conversation"""
        # Create a conversation
        conversation = create_conversation(self.db)
        original_timestamp = conversation.timestamp_last_message

        # Update the last message timestamp
        update_conversation_last_message(self.db, conversation.conversation_id)

        # Get the conversation again to check the updated timestamp
        updated_conversation = get_conversation(self.db, conversation.conversation_id)
        assert updated_conversation.timestamp_last_message > original_timestamp


# Run tests if this file is executed directly
if __name__ == "__main__":
    pytest.main([__file__])