"""Main API backend for the Physical AI RAG chatbot"""

from contextlib import asynccontextmanager  # ← Added this import
from fastapi import FastAPI, Depends, HTTPException, status, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from typing import Optional
import logging
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import local modules
from .db.database import get_db, create_tables
from .security import api_key_auth
from .services.rag_service import RAGService
from .services.embedding_service import EmbeddingService
from .services.retrieval_service import RetrievalService
from .db.crud import (
    get_conversation_history, create_conversation, create_message,
    create_feedback, get_conversation
)

# Initialize services
embedding_service = EmbeddingService()
retrieval_service = RetrievalService()
rag_service = RAGService(embedding_service, retrieval_service)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# ← NEW: Modern lifespan handler (replaces @app.on_event("startup"))
@asynccontextmanager
async def lifespan(app: FastAPI):
    """Handle startup and shutdown events"""
    logger.info("Initializing database tables...")
    create_tables()
    logger.info("Database tables initialized successfully")
    yield
    # Add shutdown logic here if needed in the future

app = FastAPI()
# Initialize FastAPI app with lifespan
app = FastAPI(
    title="Physical AI RAG Chatbot API",
    description="API for the Physical AI RAG Chatbot that answers questions about Physical AI and Humanoid Robotics",
    version="1.0.0",
    lifespan=lifespan  # ← Pass the lifespan here
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("ALLOWED_ORIGINS", "*").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    
    # Allow custom headers like X-API-Key
    allow_headers=["X-API-Key", "Authorization", "Content-Type"],
)

# ← REMOVED the old @app.on_event("startup") block entirely


@app.post("/v1/query")
async def query_endpoint(
    request: Request,
    question: str,
    conversation_id: Optional[str] = None,
    api_key: bool = Depends(api_key_auth)
):
    """Endpoint to query the RAG system with a natural language question"""
    try:
        # If no conversation_id provided, create a new one
        if not conversation_id:
            conversation = create_conversation(next(get_db()))
            conversation_id = conversation.conversation_id
        else:
            # Verify conversation exists
            db = next(get_db())
            if not get_conversation(db, conversation_id):
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail="Conversation not found"
                )
            db.close()

        # Generate response using RAG service
        response_data = await rag_service.generate_response(question, conversation_id)

        # Store the user query and bot response in the database
        db = next(get_db())
        try:
            # Store user query
            create_message(db, conversation_id, "user", question)

            # Store bot response
            create_message(db, conversation_id, "bot", response_data["answer"],
                          selected_text=response_data.get("selected_text"))
        finally:
            db.close()

        return JSONResponse(content=response_data)

    except Exception as e:
        logger.error(f"Error in query endpoint: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing query: {str(e)}"
        )

@app.post("/v1/query-with-selection")
async def query_with_selection_endpoint(
    request: Request,
    question: str,
    selected_text: str,
    conversation_id: Optional[str] = None,
    api_key: bool = Depends(api_key_auth)
):
    """Endpoint to query the RAG system with a question and selected text context"""
    try:
        # If no conversation_id provided, create a new one
        if not conversation_id:
            conversation = create_conversation(next(get_db()))
            conversation_id = conversation.conversation_id
        else:
            # Verify conversation exists
            db = next(get_db())
            if not get_conversation(db, conversation_id):
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail="Conversation not found"
                )
            db.close()

        # Generate response using RAG service with selected text
        response_data = await rag_service.generate_response_with_selection(
            question, selected_text, conversation_id
        )

        # Store the user query and bot response in the database
        db = next(get_db())
        try:
            # Store user query with selected text context
            create_message(db, conversation_id, "user", question, selected_text=selected_text)

            # Store bot response
            create_message(db, conversation_id, "bot", response_data["answer"],
                          selected_text=selected_text)
        finally:
            db.close()

        return JSONResponse(content=response_data)

    except Exception as e:
        logger.error(f"Error in query-with-selection endpoint: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing query with selection: {str(e)}"
        )

@app.get("/v1/conversation/{conversation_id}")
async def get_conversation_endpoint(
    conversation_id: str,
    api_key: bool = Depends(api_key_auth)
):
    """Endpoint to retrieve conversation history"""
    try:
        db = next(get_db())
        try:
            # Verify conversation exists
            conversation = get_conversation(db, conversation_id)
            if not conversation:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail="Conversation not found"
                )

            # Get conversation history
            history = get_conversation_history(db, conversation_id)
            return JSONResponse(content={"conversation_id": conversation_id, "history": history})
        finally:
            db.close()

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in get conversation endpoint: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving conversation: {str(e)}"
        )

@app.post("/v1/feedback")
async def feedback_endpoint(
    request: Request,
    message_id: str,
    rating: str,
    comment: Optional[str] = None,
    user_id: Optional[str] = None,
    api_key: bool = Depends(api_key_auth)
):
    """Endpoint to submit feedback on a message"""
    try:
        # Validate rating
        if rating not in ["positive", "negative", "neutral"]:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Rating must be 'positive', 'negative', or 'neutral'"
            )

        # Create feedback
        db = next(get_db())
        try:
            feedback = create_feedback(db, message_id, rating, user_id, comment)
            return JSONResponse(content={
                "feedback_id": feedback.feedback_id,
                "status": "success"
            })
        finally:
            db.close()

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in feedback endpoint: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error submitting feedback: {str(e)}"
        )

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "Physical AI RAG Chatbot API"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",  # ← This is fine when running python backend/main.py directly
        host=os.getenv("HOST", "0.0.0.0"),
        port=int(os.getenv("PORT", 8000)),
        reload=os.getenv("RELOAD", "false").lower() == "true"
    )