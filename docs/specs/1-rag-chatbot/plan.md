# Phased Implementation Plan: RAG Chatbot for Physical AI Book

**Feature Branch**: `1-rag-chatbot`
**Created**: 2025-12-06
**Status**: Draft
**Related Spec**: [specs/1-rag-chatbot/spec.md](specs/1-rag-chatbot/spec.md)

## Overview
This plan outlines the implementation of the RAG Chatbot for the Physical AI Book in distinct phases, detailing the scope, estimated effort, key dependencies, and potential risks for each. The goal is to deliver value incrementally and manage complexity effectively.

## Phase 0: Project Setup & Repository Structure

**Scope**: Establish the foundational project structure, version control, and initial environment configuration.

**Estimated Effort**: 1 day

**Dependencies**: None

**Risks**:
-   Incorrect repository setup leading to future conflicts.
-   Missing critical development tools or configurations.

**Tasks**:
-   Create project directories: `physical-ai-rag-chatbot/scripts`, `physical-ai-rag-chatbot/backend`, `physical-ai-rag-chatbot/web`, `physical-ai-rag-chatbot/infra`, `physical-ai-rag-chatbot/tests`.
-   Initialize Python virtual environments for backend and scripts.
-   Initialize Node.js environment for the web interface.
-   Configure `.gitignore` and other version control settings.
-   Set up basic README for the RAG chatbot project.

## Phase 1: Backend + Database Connections

**Scope**: Implement the core API backend structure, establish connections to the relational database for conversation history, and set up API key authentication.

**Estimated Effort**: 3 days

**Dependencies**: Phase 0 (Project Setup).

**Risks**:
-   Difficulty in configuring relational database (e.g., Neon Serverless Postgres) connection strings and credentials.
-   Security vulnerabilities in API key management if not handled securely.
-   Performance bottlenecks if database queries are not optimized.

**Tasks**:
-   Initialize API backend application (e.g., FastAPI) with CORS and environment variable loading: `physical-ai-rag-chatbot/backend/main.py`.
-   Implement API key authentication middleware: `physical-ai-rag-chatbot/backend/security.py`.
-   Define database schemas for `Conversation`, `Message`, `Feedback` entities: `physical-ai-rag-chatbot/backend/db/models.py`.
-   Implement relational database connection and session management: `physical-ai-rag-chatbot/backend/db/database.py`.
-   Develop CRUD operations for conversation history and feedback: `physical-ai-rag-chatbot/backend/db/crud.py`.
-   Implement `GET /v1/conversation/{id}` endpoint to retrieve chat history: `physical-ai-rag-chatbot/backend/main.py`.
-   Implement `POST /v1/feedback` endpoint to store user feedback: `physical-ai-rag-chatbot/backend/main.py`.

## Phase 2: Book Ingestion Pipeline

**Scope**: Develop a script to ingest Markdown content from the online book, chunk it, generate embeddings, and store them in the vector database.

**Estimated Effort**: 4 days

**Dependencies**: Phase 0 (Project Setup), Phase 1 (Database connections for metadata if applicable).

**Risks**:
-   Inaccurate text chunking leading to poor retrieval quality.
-   High cost or slow performance of the embedding service.
-   Issues with vector database connectivity or schema definition.
-   Out-of-memory errors for large documents during embedding generation.

**Tasks**:
-   Design document chunking strategy (e.g., recursive character text splitter) and define parameters: `physical-ai-rag-chatbot/scripts/config.py`.
-   Implement document loading and chunking script for Markdown files from `/web/docs`: `physical-ai-rag-chatbot/scripts/ingestion.py`.
-   Implement embedding generation logic using an external embedding service: `physical-ai-rag-chatbot/scripts/ingestion.py`.
-   Configure vector database client and connection: `physical-ai-rag-chatbot/scripts/ingestion.py`.
-   Define vector collection schema (vector dimension, distance metric, payload fields): `physical-ai-rag-chatbot/scripts/ingestion.py`.
-   Implement upsert logic for document chunks and their metadata into the vector database: `physical-ai-rag-chatbot/scripts/ingestion.py`.
-   Run initial ingestion pipeline to populate the vector database.

## Phase 3: RAG Query Endpoint

**Scope**: Implement the core Retrieval-Augmented Generation (RAG) logic, including query embedding, vector search, and LLM orchestration, and expose it via an API endpoint.

**Estimated Effort**: 5 days

**Dependencies**: Phase 1 (Backend API setup), Phase 2 (Populated Vector Database).

**Risks**:
-   Poor retrieval quality leading to irrelevant context for the LLM.
-   LLM hallucinations or inability to strictly adhere to provided context.
-   High latency from LLM calls or vector database queries.
-   Ineffective prompt engineering leading to suboptimal answers.

**Tasks**:
-   Implement query embedding generation for incoming user questions: `physical-ai-rag-chatbot/backend/services/embedding_service.py`.
-   Develop vector search logic to retrieve relevant document chunks from the vector database: `physical-ai-rag-chatbot/backend/services/retrieval_service.py`.
-   Implement RAG orchestration using an AI agent framework to generate answers from retrieved context: `physical-ai-rag-chatbot/backend/services/rag_service.py`.
-   Develop logic to extract and format source citations from retrieved chunks: `physical-ai-rag-chatbot/backend/utils/citation_formatter.py`.
-   Implement `POST /v1/query` endpoint in the API backend: `physical-ai-rag-chatbot/backend/main.py`.
-   Integrate logic to store user queries and chatbot responses in the relational database: `physical-ai-rag-chatbot/backend/db/crud.py`.

## Phase 4: Web-based Chat Interface Component

**Scope**: Build and integrate the web-based chat interface into the online book, enabling users to interact with the chatbot.

**Estimated Effort**: 3 days

**Dependencies**: Phase 0 (Web environment setup), Phase 3 (Functional RAG Query Endpoint).

**Risks**:
-   UI/UX issues leading to a poor user experience.
-   Frontend performance bottlenecks or slow loading times.
-   Integration challenges with the existing content management system.

**Tasks**:
-   Create basic web-based chat interface component with input field, message display, and typing indicator: `physical-ai-rag-chatbot/web/src/components/ChatInterface.js`.
-   Integrate chat interface component into the online book's main page (e.g., `physical-ai-rag-chatbot/web/src/pages/index.js`).
-   Implement API calls from the chat interface to `POST /v1/query` endpoint.
-   Display chatbot responses including formatted source citations.
-   Ensure responsive design and basic styling.

## Phase 5: Selected-Text Integration

**Scope**: Enhance the chatbot to handle queries with user-selected text as additional context, providing more precise answers.

**Estimated Effort**: 3 days

**Dependencies**: Phase 3 (RAG Query Endpoint), Phase 4 (Web-based Chat Interface).

**Risks**:
-   Difficulty in accurately capturing user-selected text across different browsers/devices.
-   Challenges in strictly grounding LLM responses to only the selected text, even if other relevant content exists.
-   Complexity in updating existing RAG pipeline to incorporate selection-based context effectively.

**Tasks**:
-   Implement client-side logic for detecting and capturing selected text on content pages: `physical-ai-rag-chatbot/web/src/utils/selection_detector.js`.
-   Add UI element to the chat interface to enable selection-based questioning mode.
-   Modify chat interface to send selected text along with the query to `POST /v1/query-with-selection` endpoint.
-   Extend backend query embedding logic to handle selected text in addition to natural language query: `physical-ai-rag-chatbot/backend/services/embedding_service.py`.
-   Enhance vector search logic to prioritize or strictly filter based on selected text context: `physical-ai-rag-chatbot/backend/services/retrieval_service.py`.
-   Implement RAG orchestration for selection-based queries, ensuring strict grounding in selected text: `physical-ai-rag-chatbot/backend/services/rag_service.py`.
-   Implement `POST /v1/query-with-selection` endpoint in the API backend: `physical-ai-rag-chatbot/backend/main.py`.

## Phase 6: Testing & Deployment

**Scope**: Comprehensive testing, deployment of all components, and final documentation.

**Estimated Effort**: 4 days

**Dependencies**: All previous phases completed.

**Risks**:
-   Uncovered bugs or integration issues found late in the cycle.
-   Deployment complexities or environment configuration issues.
-   Performance regressions under load.
-   Security vulnerabilities identified during final audits.

**Tasks**:
-   Write comprehensive unit tests for all backend services and utility functions.
-   Develop integration tests to verify end-to-end RAG pipeline functionality.
-   Create end-to-end tests for all user stories (basic query, selection-based query, history, feedback).
-   Implement performance tests for API latency and throughput.
-   Conduct security audits (SAST/DAST) on backend and frontend code.
-   Create Dockerfiles for the API backend: `physical-ai-rag-chatbot/infra/backend.Dockerfile`.
-   Write deployment scripts/configuration for API backend to a cloud platform.
-   Document deployment steps for the web-based chat interface (static site deployment).
-   Update project README with setup, usage, and deployment instructions.
-   Code cleanup and final refactoring.

## Estimated Total Effort
Approximately 20 person-days (excluding external service setup/management time). This estimate can vary significantly based on developer experience, complexity of external integrations, and specific framework choices.