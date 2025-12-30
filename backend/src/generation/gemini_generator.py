"""
Gemini-based answer generator with RAG grounding.

Generates answers using Google Gemini 1.5 Pro with strict grounding
to retrieved context. Implements citation formatting and hallucination prevention.
"""

from typing import List, Dict, Any, Optional
import re

import google.generativeai as genai

from src.config import get_settings
from src.storage.schemas import RetrievalResult, ChatResponse
from src.utils.logger import get_logger
from src.utils.retry import retry_with_exponential_backoff

logger = get_logger(__name__)


class GeminiAnswerGenerator:
    """
    Answer generator using Gemini 1.5 Pro with RAG grounding.

    Key features:
    - Strict grounding to retrieved context (no hallucinations)
    - Temperature=0.2 for deterministic output
    - Automatic citation formatting
    - Response validation
    """

    # RAG system prompt with grounding instructions
    SYSTEM_PROMPT = """You are a knowledgeable assistant for the "Physical AI & Humanoid Robotics" book.

Your role is to answer questions STRICTLY based on the provided context from the book.

CRITICAL RULES:
1. ONLY use information from the provided context - do not use external knowledge
2. If the context doesn't contain enough information to answer, say so explicitly
3. Always cite your sources using [Source N] notation when referencing information
4. Be precise and concise - focus on answering the specific question asked
5. If asked about topics not in the context, respond: "I cannot answer this question based on the available book content."
6. Do not make assumptions or inferences beyond what's explicitly stated in the context
7. If context is contradictory, acknowledge the contradiction

Remember: It's better to say "I don't know" than to provide information not in the context."""

    def __init__(self):
        """Initialize Gemini generator with API key and configuration."""
        settings = get_settings()

        # Configure Gemini
        genai.configure(api_key=settings.gemini_api_key)

        # Initialize model with safety settings
        self.model = genai.GenerativeModel(
            model_name=settings.gemini_model,
            generation_config={
                "temperature": settings.gemini_temperature,
                "top_p": settings.gemini_top_p,
                "top_k": 40,
                "max_output_tokens": settings.gemini_max_output_tokens,
            },
        )

        self.max_tokens = settings.gemini_max_output_tokens
        self.temperature = settings.gemini_temperature

        logger.info(
            f"Initialized GeminiAnswerGenerator: model={settings.gemini_model}, "
            f"temp={settings.gemini_temperature}, max_tokens={settings.gemini_max_output_tokens}"
        )

    def build_prompt(
        self,
        query: str,
        context: str,
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> str:
        """
        Build RAG prompt with system instructions, context, and query.

        Args:
            query: User's question
            context: Retrieved context from book chunks
            conversation_history: Optional list of {"role": "user/assistant", "content": "..."}

        Returns:
            str: Complete prompt for Gemini
        """
        prompt_parts = [self.SYSTEM_PROMPT]

        # Add conversation history if provided
        if conversation_history:
            prompt_parts.append("\n--- CONVERSATION HISTORY ---")
            for turn in conversation_history[-3:]:  # Last 3 turns for context
                role = turn["role"].upper()
                content = turn["content"]
                prompt_parts.append(f"{role}: {content}")

        # Add context
        prompt_parts.append("\n--- BOOK CONTEXT ---")
        if context and context.strip():
            prompt_parts.append(context)
        else:
            prompt_parts.append("[No relevant context found in the book]")

        # Add user query
        prompt_parts.append("\n--- USER QUESTION ---")
        prompt_parts.append(query)

        prompt_parts.append("\n--- YOUR ANSWER ---")
        prompt_parts.append(
            "Provide a clear, grounded answer using ONLY the context above. "
            "Include [Source N] citations."
        )

        return "\n".join(prompt_parts)

    @retry_with_exponential_backoff(
        max_retries=3,
        exceptions=(Exception,),
    )
    def generate_answer(
        self,
        query: str,
        context: str,
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> str:
        """
        Generate answer using Gemini with RAG grounding.

        Args:
            query: User's question
            context: Retrieved context from book
            conversation_history: Optional conversation history

        Returns:
            str: Generated answer with citations

        Raises:
            Exception: If generation fails after retries
        """
        prompt = self.build_prompt(query, context, conversation_history)

        logger.info(f"Generating answer for query: {query[:100]}...")
        logger.debug(f"Prompt length: {len(prompt)} characters")

        try:
            response = self.model.generate_content(prompt)

            if not response.text:
                logger.warning("Gemini returned empty response")
                return "I apologize, but I couldn't generate an answer. Please try rephrasing your question."

            answer = response.text.strip()
            logger.info(f"Generated answer: {len(answer)} characters")
            logger.debug(f"Answer preview: {answer[:200]}...")

            return answer

        except Exception as e:
            logger.error(f"Answer generation failed: {e}")
            raise

    def validate_response(self, answer: str, context: str) -> Dict[str, Any]:
        """
        Validate generated answer for quality and grounding.

        Args:
            answer: Generated answer
            context: Original context used

        Returns:
            Dict with validation results
        """
        validation = {
            "is_valid": True,
            "warnings": [],
            "has_citations": False,
            "length": len(answer),
        }

        # Check for citations
        citations = re.findall(r'\[Source \d+\]', answer)
        validation["has_citations"] = len(citations) > 0
        validation["citation_count"] = len(citations)

        if not validation["has_citations"] and context.strip():
            validation["warnings"].append(
                "Answer lacks source citations despite having context"
            )

        # Check for common hallucination phrases
        hallucination_phrases = [
            "based on my knowledge",
            "in general",
            "typically",
            "usually",
            "it is known that",
            "studies show",
            "research indicates",
        ]

        for phrase in hallucination_phrases:
            if phrase.lower() in answer.lower():
                validation["warnings"].append(
                    f"Potential hallucination detected: '{phrase}'"
                )
                validation["is_valid"] = False

        # Check minimum length
        if validation["length"] < 20:
            validation["warnings"].append("Answer is very short")

        # Check if answer indicates lack of context
        no_answer_phrases = [
            "cannot answer",
            "don't know",
            "no information",
            "not in the context",
        ]
        validation["indicates_no_context"] = any(
            phrase in answer.lower() for phrase in no_answer_phrases
        )

        logger.debug(f"Validation result: {validation}")
        return validation

    def generate_chat_response(
        self,
        query: str,
        retrieval_results: List[RetrievalResult],
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> ChatResponse:
        """
        Generate complete chat response with answer and metadata.

        Args:
            query: User's question
            retrieval_results: List of retrieved chunks
            conversation_history: Optional conversation history

        Returns:
            ChatResponse: Complete response with answer, sources, and metadata
        """
        logger.info(f"Generating chat response for: {query[:100]}...")

        # Format context from retrieval results
        context_parts = []
        sources = []

        for result in retrieval_results:
            context_parts.append(
                f"[Source {result.rank}: {result.section} - {result.url}]\n"
                f"{result.text}"
            )
            sources.append(
                {
                    "rank": result.rank,
                    "url": result.url,
                    "section": result.section,
                    "chunk_index": result.chunk_index,
                    "similarity_score": result.similarity_score,
                }
            )

        context = "\n\n".join(context_parts)

        # Generate answer
        try:
            answer = self.generate_answer(query, context, conversation_history)
        except Exception as e:
            logger.error(f"Failed to generate answer: {e}")
            answer = (
                "I apologize, but I encountered an error while generating an answer. "
                "Please try again."
            )

        # Validate response
        validation = self.validate_response(answer, context)

        # Build ChatResponse
        response = ChatResponse(
            query=query,
            answer=answer,
            sources=sources,
            num_sources=len(sources),
            model=self.model.model_name,
            temperature=self.temperature,
            is_grounded=validation["is_valid"],
            has_citations=validation["has_citations"],
        )

        if validation["warnings"]:
            logger.warning(f"Response validation warnings: {validation['warnings']}")

        logger.info(
            f"Chat response generated: {len(answer)} chars, "
            f"{len(sources)} sources, grounded={response.is_grounded}"
        )

        return response

    def format_response_for_display(self, response: ChatResponse) -> str:
        """
        Format ChatResponse for CLI display.

        Args:
            response: ChatResponse object

        Returns:
            str: Formatted response for terminal output
        """
        lines = []

        # Answer
        lines.append("ANSWER:")
        lines.append("-" * 60)
        lines.append(response.answer)
        lines.append("")

        # Sources
        if response.sources:
            lines.append("SOURCES:")
            lines.append("-" * 60)
            for source in response.sources:
                lines.append(
                    f"[{source['rank']}] {source['section']} "
                    f"(score: {source['similarity_score']:.3f})"
                )
                lines.append(f"    {source['url']}")
            lines.append("")

        # Metadata
        lines.append("METADATA:")
        lines.append("-" * 60)
        lines.append(f"Model: {response.model}")
        lines.append(f"Sources: {response.num_sources}")
        lines.append(f"Grounded: {response.is_grounded}")
        lines.append(f"Has Citations: {response.has_citations}")

        return "\n".join(lines)
