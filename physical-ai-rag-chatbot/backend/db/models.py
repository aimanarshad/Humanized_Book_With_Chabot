"""Database models for the Physical AI RAG chatbot"""

from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, ForeignKey, Boolean
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from datetime import datetime
import os

Base = declarative_base()

class Conversation(Base):
    """Represents a user's conversation session with the chatbot"""
    __tablename__ = "conversations"

    id = Column(Integer, primary_key=True, index=True)
    conversation_id = Column(String, unique=True, index=True, nullable=False)
    user_id = Column(String, index=True)  # Optional user identifier
    timestamp_started = Column(DateTime, default=datetime.utcnow)
    timestamp_last_message = Column(DateTime, default=datetime.utcnow)
    is_active = Column(Boolean, default=True)

    # Relationship to messages
    messages = relationship("Message", back_populates="conversation", cascade="all, delete-orphan")

class Message(Base):
    """Represents a single query or response within a conversation"""
    __tablename__ = "messages"

    id = Column(Integer, primary_key=True, index=True)
    message_id = Column(String, unique=True, index=True, nullable=False)
    conversation_id = Column(String, ForeignKey("conversations.conversation_id"), nullable=False)
    sender = Column(String, nullable=False)  # 'user' or 'bot'
    text_content = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow)
    selected_text = Column(Text)  # For messages that include selected text context

    # Relationship back to conversation
    conversation = relationship("Conversation", back_populates="messages")

    # Relationship to feedback
    feedback = relationship("Feedback", back_populates="message", cascade="all, delete-orphan")

class Feedback(Base):
    """Represents user feedback on a bot's message"""
    __tablename__ = "feedback"

    id = Column(Integer, primary_key=True, index=True)
    feedback_id = Column(String, unique=True, index=True, nullable=False)
    message_id = Column(String, ForeignKey("messages.message_id"), nullable=False)
    user_id = Column(String, index=True)
    rating = Column(String, nullable=False)  # 'positive', 'negative', or 'neutral'
    comment = Column(Text)
    timestamp = Column(DateTime, default=datetime.utcnow)

    # Relationship back to message
    message = relationship("Message", back_populates="feedback")

# Example of how to create the tables
# This would typically be called in your database initialization code
def create_tables(engine):
    """Create all tables in the database"""
    Base.metadata.create_all(bind=engine)