# Physical AI RAG Chatbot

An intelligent assistant integrated into the Physical AI & Humanoid Robotics book that answers questions using Retrieval-Augmented Generation (RAG) based on the book's content.

## Features

- **Question Answering**: Ask questions about Physical AI, ROS 2, Gazebo, NVIDIA Isaac, Vision-Language-Action systems, and more
- **Source Citations**: All answers include citations to the original book content
- **Selection-Based Queries**: Ask questions with specific text selected for focused answers
- **Conversation History**: Persistent conversation tracking
- **Feedback System**: Rate responses to improve the system
- **Web Interface**: Integrated chat widget for seamless user experience

## Architecture

The system consists of three main components:

### 1. Ingestion Pipeline
- Processes markdown files from `/docs` directory
- Chunks content into 500-800 token passages with overlap
- Generates embeddings using Google Generative AI
- Stores embeddings in Qdrant vector database

### 2. Backend API
- FastAPI-based REST API
- Vector similarity search for relevant content retrieval
- LLM-powered response generation
- Conversation and feedback storage in Neon Postgres
- API key authentication

### 3. Web Interface
- React-based chat widget
- Text selection integration
- Real-time messaging with typing indicators
- Source citation display
- Feedback collection

## API Endpoints

### Query Endpoints
- `POST /v1/query` - Basic question answering
  ```json
  {
    "question": "What is ROS 2?",
    "conversation_id": "optional_conversation_id"
  }
  ```

- `POST /v1/query-with-selection` - Question answering with selected text
  ```json
  {
    "question": "Explain this concept",
    "selected_text": "The selected text content...",
    "conversation_id": "optional_conversation_id"
  }
  ```

### Conversation Endpoints
- `GET /v1/conversation/{id}` - Retrieve conversation history
- `POST /v1/feedback` - Submit feedback
  ```json
  {
    "message_id": "message_id",
    "rating": "positive|negative|neutral",
    "comment": "optional comment"
  }
  ```

## Getting Started

### Prerequisites
- Python 3.8+
- Node.js (for web interface)
- Google Generative AI API key
- Qdrant vector database
- Neon Postgres database

### Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd Physical-AI-
   ```

2. **Install backend dependencies**
   ```bash
   cd physical-ai-rag-chatbot/backend
   pip install -r requirements.txt
   ```

3. **Install web dependencies**
   ```bash
   cd ../web
   npm install
   ```

4. **Set up environment variables**
   Create a `.env` file in the root directory with:
   ```env
   GEMINI_API_KEY=your_gemini_api_key  # Or use OPENAI_API_KEY as fallback
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   NEON_DATABASE_URL=your_neon_connection_string
   PHYSICAL_AI_API_KEY=your_generated_api_key
   ```

5. **Run the ingestion script**
   ```bash
   cd ..
   python scripts/ingestion.py
   ```

6. **Start the backend**
   ```bash
   cd backend
   python -m main
   ```

### Running with Docker

Build and run the backend service:
```bash
cd physical-ai-rag-chatbot
chmod +x infra/deploy_backend.sh
./infra/deploy_backend.sh
```

## Development

### Project Structure
```
physical-ai-rag-chatbot/
├── backend/              # FastAPI backend
│   ├── db/              # Database models and operations
│   ├── services/        # Core services (embedding, retrieval, RAG)
│   ├── utils/           # Utility functions
│   └── main.py          # Main application
├── scripts/             # Ingestion and utility scripts
├── web/                 # Web interface
├── infra/               # Infrastructure files
├── docs/                # Documentation
└── tests/               # Test files
```

### Running Tests
```bash
# Backend tests
cd backend
python -m pytest tests/

# Or run all tests
cd ..
python -m pytest tests/
```

## Environment Variables

- `GEMINI_API_KEY`: Google Generative AI API key for embeddings and LLM (fallback to OPENAI_API_KEY)
- `QDRANT_URL`: Qdrant vector database URL
- `QDRANT_API_KEY`: Qdrant API key
- `NEON_DATABASE_URL`: Neon Postgres connection string
- `PHYSICAL_AI_API_KEY`: API key for authenticating requests
- `EMBEDDING_MODEL`: Google Generative AI embedding model (default: gemini-embedding-001)
- `LLM_MODEL`: Google Generative AI LLM model (default: gemini-1.5-flash)
- `QDRANT_COLLECTION_NAME`: Collection name for document embeddings
- `RETRIEVAL_TOP_K`: Number of documents to retrieve (default: 5)

## Deployment

See [Deployment Guide](docs/deployment.md) for detailed deployment instructions.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For support, please open an issue in the repository or contact the development team.

---

Built with ❤️ for the Physical AI & Humanoid Robotics community.