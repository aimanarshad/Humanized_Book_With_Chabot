"""CRUD operations for the Physical AI RAG chatbot database"""

from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
from typing import Optional, List
from datetime import datetime
import uuid
from .models import Conversation, Message, Feedback

# Conversation CRUD operations
def create_conversation(db: Session, user_id: Optional[str] = None) -> Conversation:
    """Create a new conversation"""
    conversation_id = str(uuid.uuid4())
    conversation = Conversation(
        conversation_id=conversation_id,
        user_id=user_id,
        timestamp_started=datetime.utcnow(),
        timestamp_last_message=datetime.utcnow()
    )
    db.add(conversation)
    db.commit()
    db.refresh(conversation)
    return conversation

def get_conversation(db: Session, conversation_id: str) -> Optional[Conversation]:
    """Get a conversation by ID"""
    return db.query(Conversation).filter(Conversation.conversation_id == conversation_id).first()

def update_conversation_last_message(db: Session, conversation_id: str):
    """Update the last message timestamp for a conversation"""
    conversation = db.query(Conversation).filter(Conversation.conversation_id == conversation_id).first()
    if conversation:
        conversation.timestamp_last_message = datetime.utcnow()
        db.commit()
        db.refresh(conversation)

# Message CRUD operations
def create_message(
    db: Session,
    conversation_id: str,
    sender: str,
    text_content: str,
    selected_text: Optional[str] = None
) -> Message:
    """Create a new message in a conversation"""
    message_id = str(uuid.uuid4())
    message = Message(
        message_id=message_id,
        conversation_id=conversation_id,
        sender=sender,
        text_content=text_content,
        selected_text=selected_text
    )
    db.add(message)
    db.commit()
    db.refresh(message)

    # Update the conversation's last message timestamp
    update_conversation_last_message(db, conversation_id)

    return message

def get_messages_by_conversation(db: Session, conversation_id: str) -> List[Message]:
    """Get all messages for a conversation"""
    return db.query(Message).filter(Message.conversation_id == conversation_id).order_by(Message.timestamp).all()

def get_conversation_history(db: Session, conversation_id: str) -> List[dict]:
    """Get formatted conversation history for a conversation"""
    messages = get_messages_by_conversation(db, conversation_id)
    return [
        {
            "message_id": msg.message_id,
            "sender": msg.sender,
            "text_content": msg.text_content,
            "timestamp": msg.timestamp.isoformat(),
            "selected_text": msg.selected_text
        }
        for msg in messages
    ]

# Feedback CRUD operations
def create_feedback(
    db: Session,
    message_id: str,
    rating: str,
    user_id: Optional[str] = None,
    comment: Optional[str] = None
) -> Feedback:
    """Create feedback for a message"""
    feedback_id = str(uuid.uuid4())
    feedback = Feedback(
        feedback_id=feedback_id,
        message_id=message_id,
        user_id=user_id,
        rating=rating,
        comment=comment
    )
    db.add(feedback)
    db.commit()
    db.refresh(feedback)
    return feedback

def get_feedback_by_message(db: Session, message_id: str) -> Optional[Feedback]:
    """Get feedback for a specific message"""
    return db.query(Feedback).filter(Feedback.message_id == message_id).first()

# Utility functions
def conversation_exists(db: Session, conversation_id: str) -> bool:
    """Check if a conversation exists"""
    return db.query(Conversation).filter(Conversation.conversation_id == conversation_id).first() is not None

def message_exists(db: Session, message_id: str) -> bool:
    """Check if a message exists"""
    return db.query(Message).filter(Message.message_id == message_id).first() is not None

def get_all_conversations(db: Session, user_id: Optional[str] = None) -> List[Conversation]:
    """Get all conversations, optionally filtered by user_id"""
    query = db.query(Conversation)
    if user_id:
        query = query.filter(Conversation.user_id == user_id)
    return query.all()