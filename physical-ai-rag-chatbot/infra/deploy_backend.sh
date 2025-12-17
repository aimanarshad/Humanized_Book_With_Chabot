#!/bin/bash

# Deployment script for Physical AI RAG Chatbot Backend
# This script builds and deploys the backend service

set -e  # Exit on any error

# Configuration
SERVICE_NAME="physical-ai-rag-chatbot-backend"
IMAGE_NAME="physical-ai-rag-chatbot-backend"
CONTAINER_NAME="physical-ai-rag-chatbot-backend-container"
PORT=8000
ENV_FILE=".env"

echo "Starting deployment of $SERVICE_NAME..."

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed or not in PATH. Please install Docker and try again."
    exit 1
fi

# Check if .env file exists
if [ ! -f "$ENV_FILE" ]; then
    echo "Environment file $ENV_FILE not found. Please create it with required environment variables:"
    echo "GEMINI_API_KEY="  # Or use OPENAI_API_KEY as fallback
    echo "QDRANT_URL="
    echo "QDRANT_API_KEY="
    echo "NEON_DATABASE_URL="
    echo "PHYSICAL_AI_API_KEY="
    exit 1
fi

# Build the Docker image
echo "Building Docker image: $IMAGE_NAME"
docker build -f infra/backend.Dockerfile -t $IMAGE_NAME .

# Stop and remove existing container if it exists
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Stopping existing container: $CONTAINER_NAME"
    docker stop $CONTAINER_NAME
fi

if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Removing existing container: $CONTAINER_NAME"
    docker rm $CONTAINER_NAME
fi

# Run the new container
echo "Starting container: $CONTAINER_NAME"
docker run -d \
    --name $CONTAINER_NAME \
    --env-file $ENV_FILE \
    -p $PORT:$PORT \
    -v $(pwd)/logs:/app/logs \
    $IMAGE_NAME

echo "Waiting for service to start..."
sleep 10

# Check if the service is running
if docker ps | grep -q $CONTAINER_NAME; then
    echo "✅ $SERVICE_NAME is running successfully!"
    echo "Service is available at: http://localhost:$PORT"

    # Test the health endpoint
    echo "Testing health endpoint..."
    if curl -f http://localhost:$PORT/health > /dev/null 2>&1; then
        echo "✅ Health check passed"
    else
        echo "⚠️  Health check failed, but container is running"
    fi
else
    echo "❌ Failed to start $SERVICE_NAME"
    echo "Check container logs with: docker logs $CONTAINER_NAME"
    exit 1
fi

echo "Deployment completed successfully!"
echo ""
echo "Next steps:"
echo "- Verify the service at http://localhost:$PORT/health"
echo "- Run the ingestion script to populate the knowledge base: python scripts/ingestion.py"
echo "- Test the API endpoints as documented in the specification"