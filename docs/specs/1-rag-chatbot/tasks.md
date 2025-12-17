# Tasks: RAG Chatbot for Physical AI Book

**Input**: Design documents from `/specs/1-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: All tasks will include testing aspects as defined in the plan.md.

**Organization**: Tasks are grouped by logical components and then by user story to enable independent implementation and testing where applicable.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the RAG chatbot components.

- [X] T001 Create project directories: `physical-ai-rag-chatbot/scripts`, `physical-ai-rag-chatbot/backend`, `physical-ai-rag-chatbot/web`, `physical-ai-rag-chatbot/infra`, `physical-ai-rag-chatbot/tests`
- [X] T002 Initialize Python environment for backend and scripts: `physical-ai-rag-chatbot/backend/requirements.txt`, `physical-ai-rag-chatbot/scripts/requirements.txt`
- [X] T003 [P] Initialize Node.js environment for web interface: `physical-ai-rag-chatbot/web/package.json`

---

## Phase 2: Foundational Components (Ingestion & Core Services)

**Purpose**: Establish the core infrastructure that MUST be complete before user story implementation can begin. This includes document ingestion, embedding generation, vector storage setup, and relational database schema.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

### Ingestion Pipeline Tasks
- [X] T004 [P] [Ingestion] Design document chunking strategy (e.g., recursive character text splitter) and define parameters (chunk size, overlap): `physical-ai-rag-chatbot/scripts/config.py`
- [X] T005 [P] [Ingestion] Implement document loading and chunking script for Markdown files from `/web/docs`: `physical-ai-rag-chatbot/scripts/ingestion.py`
- [X] T006 [P] [Ingestion] Implement embedding generation logic using an embedding service (e.g., OpenAI embeddings API): `physical-ai-rag-chatbot/scripts/ingestion.py`
- [X] T007 [P] [Ingestion] Develop script to store chunks and embeddings in the vector database: `physical-ai-rag-chatbot/scripts/ingestion.py`

### Vector Database Integration Tasks
- [X] T008 [P] [Vector DB] Configure vector database client and connection in the ingestion script: `physical-ai-rag-chatbot/scripts/ingestion.py`
- [X] T009 [P] [Vector DB] Define vector collection schema (vector dimension, distance metric, payload fields for metadata) in the ingestion script: `physical-ai-rag-chatbot/scripts/ingestion.py`
- [X] T010 [P] [Vector DB] Implement upsert logic for document chunks and their metadata into the vector database: `physical-ai-rag-chatbot/scripts/ingestion.py`

### Relational Database Tasks
- [X] T011 [P] [Relational DB] Define database schemas for `Conversation`, `Message`, `Feedback` entities: `physical-ai-rag-chatbot/backend/db/models.py`
- [X] T012 [P] [Relational DB] Implement database connection and session management for the relational database: `physical-ai-rag-chatbot/backend/db/database.py`
- [X] T013 [P] [Relational DB] Develop functions for creating database tables: `physical-ai-rag-chatbot/backend/db/database.py`

### Backend Core Setup Tasks
- [X] T014 [P] [Backend] Initialize API backend application, configure CORS and environment variables: `physical-ai-rag-chatbot/backend/main.py`
- [X] T015 [P] [Backend] Implement API key authentication middleware for the backend: `physical-ai-rag-chatbot/backend/security.py`
- [X] T016 [P] [Backend] Configure logging and error handling middleware for the API backend: `physical-ai-rag-chatbot/backend/main.py`

**Checkpoint**: Foundational components ready - user story implementation can now begin in parallel.

---

## Phase 3: User Story 1 - Ask Questions and Get Cited Answers (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about the online book and receive accurate, cited answers.

**Independent Test**: Ask a question related to the book's content (e.g., "What is ROS2?") and verify that the chatbot returns an accurate answer with correct citations.

### Testing Tasks (User Story 1)
- [X] T017 [P] [US1] Write unit tests for embedding generation and vector search logic: `physical-ai-rag-chatbot/tests/backend/test_retrieval.py`
- [X] T018 [P] [US1] Write integration tests for `POST /v1/query` endpoint: `physical-ai-rag-chatbot/tests/backend/test_api.py`
- [X] T019 [P] [US1] Write end-to-end test for basic chatbot interaction (query and response with citations): `physical-ai-rag-chatbot/tests/e2e/test_chatbot.py`

### Backend API & Retrieval Logic Tasks (User Story 1)
- [X] T020 [US1] Implement query embedding generation for incoming user questions: `physical-ai-rag-chatbot/backend/services/embedding_service.py`
- [X] T021 [US1] Develop vector search logic to retrieve relevant document chunks from the vector database: `physical-ai-rag-chatbot/backend/services/retrieval_service.py`
- [X] T022 [US1] Implement RAG orchestration using AI agent framework to generate answers from retrieved context: `physical-ai-rag-chatbot/backend/services/rag_service.py`
- [X] T023 [US1] Implement `POST /v1/query` endpoint in the API backend: `physical-ai-rag-chatbot/backend/main.py`
- [X] T024 [US1] Develop logic to extract and format source citations from retrieved chunks: `physical-ai-rag-chatbot/backend/utils/citation_formatter.py`
- [X] T025 [US1] Implement logic to store user queries and chatbot responses in the relational database: `physical-ai-rag-chatbot/backend/db/crud.py`

### Web-based Chat Interface Tasks (User Story 1)
- [X] T026 [P] [US1] Create basic web-based chat interface component with input field and message display: `physical-ai-rag-chatbot/web/src/components/ChatInterface.js`
- [X] T027 [P] [US1] Integrate chat interface component into the online book's main page: `physical-ai-rag-chatbot/web/src/pages/index.js`
- [X] T028 [P] [US1] Implement API calls from the chat interface to `POST /v1/query` endpoint: `physical-ai-rag-chatbot/web/src/components/ChatInterface.js`
- [X] T029 [P] [US1] Display chatbot responses including formatted source citations: `physical-ai-rag-chatbot/web/src/components/ChatInterface.js`
- [X] T030 [P] [US1] Implement visual typing indicator in the chat interface: `physical-ai-rag-chatbot/web/src/components/ChatInterface.js`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Query with Selected Text Context (Priority: P2)

**Goal**: Allow users to ask questions with specific text selected from the online book as primary context.

**Independent Test**: Select a paragraph from a content page, ask a question directly related to that paragraph, and verify the chatbot's answer is based *only* on the selected text and cites it.

### Testing Tasks (User Story 2)
- [X] T031 [P] [US2] Write unit tests for selection-based retrieval logic: `physical-ai-rag-chatbot/tests/backend/test_retrieval.py`
- [X] T032 [P] [US2] Write integration tests for `POST /v1/query-with-selection` endpoint: `physical-ai-rag-chatbot/tests/backend/test_api.py`
- [X] T033 [P] [US2] Write end-to-end test for selection-based query and response: `physical-ai-rag-chatbot/tests/e2e/test_chatbot.py`

### Backend API & Retrieval Logic Tasks (User Story 2)
- [X] T034 [US2] Extend query embedding logic to handle selected text in addition to natural language query: `physical-ai-rag-chatbot/backend/services/embedding_service.py`
- [X] T035 [US2] Enhance vector search logic to prioritize or strictly filter based on selected text context: `physical-ai-rag-chatbot/backend/services/retrieval_service.py`
- [X] T036 [US2] Implement RAG orchestration for selection-based queries, ensuring strict grounding in selected text: `physical-ai-rag-chatbot/backend/services/rag_service.py`
- [X] T037 [US2] Implement `POST /v1/query-with-selection` endpoint in the API backend: `physical-ai-rag-chatbot/backend/main.py`

### Web-based Chat Interface & Selection-Detection Tasks (User Story 2)
- [X] T038 [P] [US2] Implement client-side logic for detecting selected text on the content page: `physical-ai-rag-chatbot/web/src/utils/selection_detector.js`
- [X] T039 [P] [US2] Add UI element to the chat interface to enable selection-based questioning mode: `physical-ai-rag-chatbot/web/src/components/ChatInterface.js`
- [X] T040 [P] [US2] Modify chat interface to send selected text along with the query to `POST /v1/query-with-selection`: `physical-ai-rag-chatbot/web/src/components/ChatInterface.js`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - Review Conversation History (Priority: P3)

**Goal**: Allow users to review their previous interactions with the chatbot.

**Independent Test**: Interact with the chatbot, close and reopen the browser/widget, and verify that previous messages are still visible or retrievable.

### Testing Tasks (User Story 3)
- [X] T041 [P] [US3] Write unit tests for conversation history database operations (insert, retrieve): `physical-ai-rag-chatbot/tests/backend/test_db.py`
- [X] T042 [P] [US3] Write integration tests for `GET /v1/conversation/{id}` endpoint: `physical-ai-rag-chatbot/tests/backend/test_api.py`
- [X] T043 [P] [US3] Write end-to-end test for displaying persistent conversation history: `physical-ai-rag-chatbot/tests/e2e/test_chatbot.py`

### Backend API & Relational DB Tasks (User Story 3)
- [X] T044 [US3] Implement logic to retrieve full conversation history from the relational database: `physical-ai-rag-chatbot/backend/db/crud.py`
- [X] T045 [US3] Implement `GET /v1/conversation/{id}` endpoint in the API backend: `physical-ai-rag-chatbot/backend/main.py`
- [X] T046 [US3] Implement `POST /v1/feedback` endpoint to store user feedback on responses: `physical-ai-rag-chatbot/backend/main.py`, `physical-ai-rag-chatbot/backend/db/crud.py`

### Web-based Chat Interface Tasks (User Story 3)
- [X] T047 [P] [US3] Modify chat interface to fetch and display historical messages upon loading: `physical-ai-rag-chatbot/web/src/components/ChatInterface.js`
- [X] T048 [P] [US3] Implement UI for users to provide feedback on bot responses: `physical-ai-rag-chatbot/web/src/components/ChatInterface.js`

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Finalize implementation, deploy, and ensure quality and maintainability.

### Deployment Tasks
- [X] T049 [Deployment] Create Dockerfiles for the API backend: `physical-ai-rag-chatbot/infra/backend.Dockerfile`
- [X] T050 [Deployment] Write deployment scripts/configuration for API backend to a cloud platform: `physical-ai-rag-chatbot/infra/deploy_backend.sh`
- [X] T051 [Deployment] Document deployment steps for the web-based chat interface (static site deployment): `physical-ai-rag-chatbot/docs/deployment.md`

### Testing & Quality Assurance Tasks
- [X] T052 [P] [QA] Review and refine all existing tests (unit, integration, e2e) for comprehensive coverage.
- [ ] T053 [P] [QA] Implement performance tests for overall system latency and throughput.
- [ ] T054 [P] [QA] Conduct security audits (SAST/DAST) on the backend and frontend code.
- [ ] T055 [P] [QA] Conduct user acceptance testing (UAT) for all user stories.

### Documentation & Maintenance Tasks
- [X] T056 [P] [Docs] Update project README with setup, usage, and deployment instructions: `physical-ai-rag-chatbot/README.md`
- [ ] T057 [P] [Docs] Add inline code comments and docstrings for maintainability.
- [ ] T058 [P] [Refactor] Code cleanup and refactoring across all modules for consistency and best practices.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion.
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 components (e.g., existing chat interface) but should be independently testable.
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 components but should be independently testable.

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation.
- Models before services.
- Services before endpoints.
- Core implementation before integration.
- Story complete before moving to next priority.

### Parallel Opportunities

- All tasks marked [P] can run in parallel within their respective phases/stories.
- Once Foundational phase completes, User Stories 1, 2, and 3 can start in parallel (if team capacity allows).
- Within each user story, testing tasks and some implementation tasks (e.g., UI component creation, API endpoint definitions) can be parallelized.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories).
3.  Complete Phase 3: User Story 1.
4.  **STOP and VALIDATE**: Test User Story 1 independently.
5.  Deploy/demo if ready.

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready.
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!).
3.  Add User Story 2 → Test independently → Deploy/Demo.
4.  Add User Story 3 → Test independently → Deploy/Demo.
5.  Each story adds value without breaking previous stories.

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together.
2.  Once Foundational is done:
    -   Developer A: User Story 1.
    -   Developer B: User Story 2.
    -   Developer C: User Story 3.
3.  Stories complete and integrate independently.

---

## Notes

-   [P] tasks = different files, no dependencies.
-   [Story] label maps task to specific user story for traceability.
-   Each user story should be independently completable and testable.
-   Verify tests fail before implementing.
-   Commit after each task or logical group.
-   Stop at any checkpoint to validate story independently.
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence.
