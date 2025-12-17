# import os
# import logging
# from pathlib import Path
# from typing import List, Dict, Any
# import tiktoken
# from langchain_text_splitters import RecursiveCharacterTextSplitter
# from langchain_community.document_loaders import TextLoader
# from qdrant_client import QdrantClient
# from qdrant_client.http import models
# from qdrant_client.http.models import Distance, VectorParams
# import google.generativeai as genai

# from dotenv import load_dotenv

# # Load environment variables
# load_dotenv()

# # Configure logging
# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)

# # Configuration
# GEMINI_API_KEY = os.getenv("OPENAI_API_KEY")
# QDRANT_URL = os.getenv("QDRANT_URL")
# QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
# EMBEDDING_MODEL = "text-embedding-3-small"
# VECTOR_SIZE = 1536  # Size of OpenAI embeddings
# COLLECTION_NAME = "physical_ai_docs"

# class Config:
#     """Configuration for document chunking and ingestion"""
#     CHUNK_SIZE = 600  # tokens (within the 500-800 range specified)
#     CHUNK_OVERLAP = 100  # tokens
#     SEPARATORS = ["\n\n", "\n", " ", ""]
#     EMBEDDING_MODEL = EMBEDDING_MODEL

# def get_embedding(text: str) -> List[float]:
#     """Get embedding for a text using Gemini API"""
#     if not GEMINI_API_KEY:
#         raise ValueError("GEMINI_API_KEY environment variable is not set")

#     genai.configure(api_key=GEMINI_API_KEY)

#     response = genai.embeddings.create(
#         model="textembedding-gecko-001",  # Gemini embedding model
#         input=text
#     )

#     # Gemini returns embedding in response.data[0].embedding
#     return response.data[0].embedding
# def load_and_chunk_documents(docs_path: str) -> List[Dict[str, Any]]:
#     """Load markdown documents and chunk them"""
#     text_splitter = RecursiveCharacterTextSplitter(
#         chunk_size=Config.CHUNK_SIZE,
#         chunk_overlap=Config.CHUNK_OVERLAP,
#         separators=Config.SEPARATORS,
#         length_function=lambda x: len(tiktoken.encoding_for_model("gpt-3.5-turbo").encode(x))
#     )

#     documents = []
#     docs_dir = Path(docs_path)

#     # Process all markdown files in the docs directory
#     for md_file in docs_dir.glob("**/*.md"):
#         if md_file.name != "main.py":  # Skip the main.py file in docs
#             try:
#                 # Use TextLoader to load the markdown file
#                 loader = TextLoader(str(md_file), encoding='utf-8')
#                 doc_chunks = loader.load_and_split(text_splitter)

#                 for i, chunk in enumerate(doc_chunks):
#                     documents.append({
#                         "content": chunk.page_content,
#                         "source_file": str(md_file.relative_to(docs_dir)),
#                         "chunk_id": f"{md_file.name}_{i}",
#                         "metadata": {
#                             "source_file": str(md_file.relative_to(docs_dir)),
#                             "chunk_id": f"{md_file.name}_{i}",
#                             "file_path": str(md_file)
#                         }
#                     })
#                 logger.info(f"Processed {len(doc_chunks)} chunks from {md_file}")
#             except Exception as e:
#                 logger.error(f"Error processing {md_file}: {e}")

#     logger.info(f"Total chunks created: {len(documents)}")
#     return documents

# def initialize_qdrant_collection():
#     """Initialize the Qdrant collection for storing document embeddings"""
#     if not QDRANT_URL:
#         raise ValueError("QDRANT_URL environment variable is not set")

#     # Connect to Qdrant
#     client = QdrantClient(
#         url=QDRANT_URL,
#         api_key=QDRANT_API_KEY,
#         timeout=10
#     )

#     # Check if collection exists, if not create it
#     collections = client.get_collections().collections
#     collection_names = [c.name for c in collections]

#     if COLLECTION_NAME not in collection_names:
#         client.create_collection(
#             collection_name=COLLECTION_NAME,
#             vectors_config=VectorParams(
#                 size=VECTOR_SIZE,
#                 distance=Distance.COSINE
#             )
#         )
#         logger.info(f"Created Qdrant collection: {COLLECTION_NAME}")
#     else:
#         logger.info(f"Qdrant collection {COLLECTION_NAME} already exists")

#     return client

# def upsert_documents_to_qdrant(documents: List[Dict[str, Any]], client: QdrantClient):
#     """Upsert document chunks and their embeddings to Qdrant"""
#     points = []

#     for i, doc in enumerate(documents):
#         # Generate embedding for the content
#         embedding = get_embedding(doc["content"])

#         # Create a Qdrant point
#         point = models.PointStruct(
#             id=i,
#             vector=embedding,
#             payload={
#                 "content": doc["content"],
#                 "source_file": doc["source_file"],
#                 "chunk_id": doc["chunk_id"],
#                 **doc["metadata"]
#             }
#         )
#         points.append(point)

#         if len(points) >= 100:  # Batch insert every 100 points
#             client.upsert(
#                 collection_name=COLLECTION_NAME,
#                 points=points
#             )
#             logger.info(f"Upserted batch of {len(points)} points to Qdrant")
#             points = []

#     # Insert remaining points
#     if points:
#         client.upsert(
#             collection_name=COLLECTION_NAME,
#             points=points
#         )
#         logger.info(f"Upserted final batch of {len(points)} points to Qdrant")

#     logger.info(f"Successfully upserted {len(documents)} documents to Qdrant")

# def main():
#     """Main ingestion function"""
#     logger.info("Starting document ingestion process...")

#     # Initialize Qdrant collection
#     qdrant_client = initialize_qdrant_collection()

#     # Load and chunk documents from the docs directory
#     docs_path = "../../../docs"  # Relative to this script's location
#     documents = load_and_chunk_documents(docs_path)

#     # Upsert documents to Qdrant
#     upsert_documents_to_qdrant(documents, qdrant_client)

#     logger.info("Document ingestion completed successfully!")

# if __name__ == "__main__":
#     main()

















import os
import logging
from pathlib import Path
from typing import List, Dict, Any
import tiktoken
import requests
import json
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import TextLoader
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", os.getenv("OPENAI_API_KEY"))  # Use GEMINI_API_KEY if available, fallback to OPENAI_API_KEY
EMBEDDING_MODEL = "gemini-embedding-001"  # Best current Gemini embedding model
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

FULL_MODEL_NAME = f"models/{EMBEDDING_MODEL}"
VECTOR_SIZE = 3072  # Dimension for gemini-embedding-001
COLLECTION_NAME = "physical_ai_docs"

class Config:
    """Configuration for document chunking and ingestion"""
    CHUNK_SIZE = 600  # tokens
    CHUNK_OVERLAP = 100  # tokens
    SEPARATORS = ["\n\n", "\n", " ", ""]
    EMBEDDING_MODEL = EMBEDDING_MODEL

def get_embedding(text: str, title: str = "") -> List[float]:
    """Get embedding for a text using Gemini REST API (no genai package needed)"""
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY or OPENAI_API_KEY environment variable is not set")

    url = f"https://generativelanguage.googleapis.com/v1/{FULL_MODEL_NAME}:embedContent?key={GEMINI_API_KEY}"

    payload = {
        "content": {"parts": [{"text": text}]},
        "task_type": "RETRIEVAL_DOCUMENT",  # Optimized for document retrieval in RAG
    }
    if title:
        payload["title"] = title  # Helps with retrieval quality when available

    headers = {
        "Content-Type": "application/json"
    }

    response = requests.post(url, headers=headers, data=json.dumps(payload))

    if response.status_code != 200:
        raise Exception(f"Error from Gemini API: {response.status_code} - {response.text}")

    data = response.json()
    return data["embedding"]["values"]

def load_and_chunk_documents(docs_path: str) -> List[Dict[str, Any]]:
    """Load markdown documents and chunk them"""
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=Config.CHUNK_SIZE,
        chunk_overlap=Config.CHUNK_OVERLAP,
        separators=Config.SEPARATORS,
        length_function=lambda x: len(tiktoken.encoding_for_model("gpt-3.5-turbo").encode(x))
    )

    documents = []
    docs_dir = Path(docs_path)

    for md_file in docs_dir.glob("**/*.md"):
        if md_file.name != "main.py":
            try:
                loader = TextLoader(str(md_file), encoding='utf-8')
                doc_chunks = loader.load_and_split(text_splitter)

                source_title = str(md_file.relative_to(docs_dir))  # Use relative path as title

                for i, chunk in enumerate(doc_chunks):
                    documents.append({
                        "content": chunk.page_content,
                        "source_file": str(md_file.relative_to(docs_dir)),
                        "chunk_id": f"{md_file.name}_{i}",
                        "title": source_title,  # For better embeddings
                        "metadata": {
                            "source_file": str(md_file.relative_to(docs_dir)),
                            "chunk_id": f"{md_file.name}_{i}",
                            "file_path": str(md_file)
                        }
                    })
                logger.info(f"Processed {len(doc_chunks)} chunks from {md_file}")
            except Exception as e:
                logger.error(f"Error processing {md_file}: {e}")

    logger.info(f"Total chunks created: {len(documents)}")
    return documents

def initialize_qdrant_collection():
    """Initialize the Qdrant collection for storing document embeddings"""
    if not QDRANT_URL:
        raise ValueError("QDRANT_URL environment variable is not set")

    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        timeout=10
    )

    collections = client.get_collections().collections
    collection_names = [c.name for c in collections]

    if COLLECTION_NAME not in collection_names:
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=VECTOR_SIZE,
                distance=Distance.COSINE
            )
        )
        logger.info(f"Created Qdrant collection: {COLLECTION_NAME}")
    else:
        logger.info(f"Qdrant collection {COLLECTION_NAME} already exists")

    return client

def upsert_documents_to_qdrant(documents: List[Dict[str, Any]], client: QdrantClient):
    """Upsert document chunks and their embeddings to Qdrant"""
    points = []

    for i, doc in enumerate(documents):
        embedding = get_embedding(doc["content"], title=doc.get("title", ""))

        point = models.PointStruct(
            id=i,
            vector=embedding,
            payload={
                "content": doc["content"],
                "source_file": doc["source_file"],
                "chunk_id": doc["chunk_id"],
                **doc["metadata"]
            }
        )
        points.append(point)

        if len(points) >= 100:
            client.upsert(
                collection_name=COLLECTION_NAME,
                points=points
            )
            logger.info(f"Upserted batch of {len(points)} points to Qdrant")
            points = []

    if points:
        client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        logger.info(f"Upserted final batch of {len(points)} points to Qdrant")

    logger.info(f"Successfully upserted {len(documents)} documents to Qdrant")

def main():
    """Main ingestion function"""
    logger.info("Starting document ingestion process...")

    qdrant_client = initialize_qdrant_collection()

    docs_path = "../../../docs"
    documents = load_and_chunk_documents(docs_path)

    upsert_documents_to_qdrant(documents, qdrant_client)

    logger.info("Document ingestion completed successfully!")

if __name__ == "__main__":
    main()