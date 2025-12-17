"""RAG (Retrieval-Augmented Generation) service for the Physical AI RAG chatbot"""

import os
from typing import List, Dict, Any
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.prompts import PromptTemplate
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnablePassthrough
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class RAGService:
    """Service for orchestrating the RAG pipeline and generating answers"""

    def __init__(self, embedding_service, retrieval_service):
        """Initialize the RAG service"""
        self.embedding_service = embedding_service
        self.retrieval_service = retrieval_service

        # Initialize LLM
        self.api_key = os.getenv("GEMINI_API_KEY", os.getenv("OPENAI_API_KEY"))  # Use GEMINI_API_KEY if available, fallback to OPENAI_API_KEY
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY or OPENAI_API_KEY environment variable is not set")

        self.llm_model = os.getenv("LLM_MODEL", "gemini-1.5-flash")
        self.llm = ChatGoogleGenerativeAI(model=self.llm_model, api_key=self.api_key)

        # Create prompt templates
        self.basic_rag_template = PromptTemplate(
            input_variables=["context", "question"],
            template="""
            You are an expert assistant for the Physical AI & Humanoid Robotics course.
            Use the following context to answer the question.
            If the answer is not in the context, clearly state that you don't know and suggest checking the Physical AI book directly.
            Provide specific citations from the context when possible.

            Context: {context}

            Question: {question}

            Answer:
            """
        )

        self.selection_rag_template = PromptTemplate(
            input_variables=["context", "question", "selected_text"],
            template="""
            You are an expert assistant for the Physical AI & Humanoid Robotics course.
            Answer the question using ONLY the information provided in the selected text context.
            Do not use any other information from outside the selected text.
            If the answer cannot be found in the selected text, clearly state that it's not in the selected text.

            Selected Text: {selected_text}

            Question: {question}

            Answer:
            """
        )

        # Create chains
        self.basic_rag_chain = (
            {
                "context": self._get_context_from_question,
                "question": RunnablePassthrough()
            }
            | self.basic_rag_template
            | self.llm
            | StrOutputParser()
        )

        self.selection_rag_chain = (
            {
                "context": self._get_context_from_selected_text,
                "question": lambda x: x["question"],
                "selected_text": lambda x: x["selected_text"]
            }
            | self.selection_rag_template
            | self.llm
            | StrOutputParser()
        )

    def _format_context(self, chunks: List[Dict[str, Any]]) -> str:
        """Format retrieved chunks into a context string"""
        formatted_chunks = []
        for chunk in chunks:
            source = chunk.get("source_file", "unknown")
            content = chunk.get("content", "")
            formatted_chunk = f"[Source: {source}]\n{content}\n"
            formatted_chunks.append(formatted_chunk)
        return "\n".join(formatted_chunks)

    async def _get_context_from_question(self, question: str) -> str:
        """Get context from the question by retrieving relevant chunks"""
        # Generate embedding for the question
        query_embedding = await self.embedding_service.generate_embedding(question)

        # Retrieve relevant chunks
        relevant_chunks = await self.retrieval_service.retrieve_relevant_chunks(query_embedding)

        # Format context
        return self._format_context(relevant_chunks)

    async def _get_context_from_selected_text(self, data: Dict[str, str]) -> str:
        """Get context when using selected text (returns the selected text itself)"""
        return data["selected_text"]

    async def generate_response(self, question: str, conversation_id: str) -> Dict[str, Any]:
        """Generate a response to a question using the RAG pipeline"""
        try:
            # Get context from the question
            context = await self._get_context_from_question(question)

            # Generate response using the chain
            answer = await self.basic_rag_chain.ainvoke(question)

            # Extract sources from the retrieved chunks
            query_embedding = await self.embedding_service.generate_embedding(question)
            relevant_chunks = await self.retrieval_service.retrieve_relevant_chunks(query_embedding, top_k=3)
            sources = [chunk.get("source_file", "") for chunk in relevant_chunks if chunk.get("source_file")]

            return {
                "answer": answer,
                "sources": sources,
                "conversation_id": conversation_id
            }
        except Exception as e:
            raise Exception(f"Error generating response: {str(e)}")

    async def generate_response_with_selection(
        self,
        question: str,
        selected_text: str,
        conversation_id: str
    ) -> Dict[str, Any]:
        """Generate a response to a question using selected text as primary context"""
        try:
            # Prepare input data
            input_data = {
                "question": question,
                "selected_text": selected_text
            }

            # Generate response using the selection-based chain
            answer = await self.selection_rag_chain.ainvoke(input_data)

            # For selection-based queries, the source is the selected text
            # We'll extract the source from the retrieval if possible
            query_embedding = await self.embedding_service.generate_embedding(question)
            relevant_chunks = await self.retrieval_service.retrieve_relevant_chunks_with_selected_text(
                query_embedding, selected_text, top_k=1
            )
            sources = [chunk.get("source_file", "") for chunk in relevant_chunks if chunk.get("source_file")]

            return {
                "answer": answer,
                "sources": sources,
                "conversation_id": conversation_id,
                "selected_text": selected_text
            }
        except Exception as e:
            raise Exception(f"Error generating response with selection: {str(e)}")

    async def validate_knowledge_base(self) -> bool:
        """Validate that the knowledge base has content"""
        try:
            # Try to retrieve a few chunks to verify the knowledge base is populated
            test_embedding = await self.embedding_service.generate_embedding("test")
            chunks = await self.retrieval_service.retrieve_relevant_chunks(test_embedding, top_k=1)
            return len(chunks) > 0
        except Exception:
            return False