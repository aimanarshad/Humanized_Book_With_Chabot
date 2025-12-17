# Physical AI RAG Chatbot - Deployment Guide

## Overview

This document provides instructions for deploying the Physical AI RAG Chatbot, which consists of:
- Backend API service (FastAPI)
- Vector database (Qdrant) for document embeddings
- Relational database (Neon Postgres) for conversation history
- Web-based chat interface

## Prerequisites

### Required Services
- **Google Generative AI API Key**: For embeddings and LLM responses
- **Qdrant Vector Database**: Cloud or self-hosted
- **Neon Postgres Database**: For conversation storage
- **Docker**: For containerized deployment (optional but recommended)

### Environment Variables

Create a `.env` file in the root directory with the following variables:

```env
# API Configuration
GEMINI_API_KEY=your_gemini_api_key_here  # Or use OPENAI_API_KEY as fallback
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
NEON_DATABASE_URL=your_neon_database_connection_string
PHYSICAL_AI_API_KEY=your_generated_api_key_here

# Application Configuration
EMBEDDING_MODEL=gemini-embedding-001
LLM_MODEL=gemini-1.5-flash
QDRANT_COLLECTION_NAME=physical_ai_docs
RETRIEVAL_TOP_K=5
HOST=0.0.0.0
PORT=8000
RELOAD=false
JWT_SECRET_KEY=your_jwt_secret_key_here
ALLOWED_ORIGINS=*
```

## Backend Deployment

### Option 1: Docker Deployment (Recommended)

1. Build and deploy using the provided script:
```bash
cd physical-ai-rag-chatbot
chmod +x infra/deploy_backend.sh
./infra/deploy_backend.sh
```

2. Verify the service is running:
```bash
curl http://localhost:8000/health
```

### Option 2: Direct Python Deployment

1. Install dependencies:
```bash
cd physical-ai-rag-chatbot/backend
pip install -r requirements.txt
```

2. Run the application:
```bash
cd physical-ai-rag-chatbot
python -m backend.main
```

## Data Ingestion

Before the chatbot can answer questions, you need to populate the knowledge base:

1. Ensure your markdown files are in the `docs/` directory
2. Run the ingestion script:
```bash
cd physical-ai-rag-chatbot
python scripts/ingestion.py
```

This will:
- Load all markdown files from the docs directory
- Chunk them into 500-800 token passages
- Generate embeddings using Google Generative AI
- Store them in Qdrant vector database

## Web Interface Integration

The web interface can be integrated into your existing Docusaurus site by:

1. Building the React app:
```bash
cd physical-ai-rag-chatbot/web
npm install
npm run build
```

2. The built files will be in the `dist/` directory and can be served statically alongside your Docusaurus site.

## API Endpoints

Once deployed, the following endpoints will be available:

### Query Endpoints
- `POST /v1/query` - Basic question answering
- `POST /v1/query-with-selection` - Question answering with selected text context

### Conversation Endpoints
- `GET /v1/conversation/{id}` - Retrieve conversation history
- `POST /v1/feedback` - Submit feedback on responses

### Health Check
- `GET /health` - Service health status

## Environment Setup

### Development
```bash
# Clone the repository
git clone <your-repo-url>
cd Physical-AI-

# Install backend dependencies
cd physical-ai-rag-chatbot/backend
pip install -r requirements.txt

# Install web dependencies
cd ../web
npm install
```

### Production
- Use the Docker deployment method for production environments
- Ensure all environment variables are properly secured
- Set `RELOAD=false` in production

## Troubleshooting

### Common Issues

1. **API Keys Not Working**: Verify all API keys in `.env` file are correct and have proper permissions
2. **Database Connection Issues**: Check database URLs and credentials
3. **Vector Database Issues**: Ensure Qdrant is accessible and collection exists
4. **CORS Issues**: Check `ALLOWED_ORIGINS` setting

### Logs
- Docker: `docker logs physical-ai-rag-chatbot-backend-container`
- Direct: Check console output or configured log directory

## Scaling Considerations

- The service can be scaled horizontally behind a load balancer
- Qdrant and Neon can handle scaling independently
- Consider connection pooling for database connections
- Monitor embedding generation costs with Google Generative AI

## Security Best Practices

- Never commit API keys to version control
- Use environment variables for all secrets
- Implement proper authentication for production use
- Regularly rotate API keys
- Monitor API usage and set appropriate limits