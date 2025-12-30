#!/usr/bin/env python3
"""
Answer Generation Testing Script

Tests the complete RAG pipeline:
1. Query embedding and retrieval
2. Context formatting
3. Answer generation with Gemini
4. Response validation
5. Citation formatting

Usage:
    python scripts/test_generation.py
"""

import sys
from pathlib import Path
from typing import Dict, Any

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from config import get_settings
from retrieval import VectorSearchEngine
from generation import GeminiAnswerGenerator
from storage.schemas import ChatResponse
from utils.logger import setup_logging, get_logger

logger = get_logger(__name__)


def print_separator(title: str = "") -> None:
    """Print a visual separator."""
    if title:
        print(f"\n{'=' * 70}")
        print(f"  {title}")
        print(f"{'=' * 70}\n")
    else:
        print(f"{'=' * 70}\n")


def print_response(response: ChatResponse) -> None:
    """Pretty print a chat response."""
    print(f"QUERY: {response.query}\n")

    print("ANSWER:")
    print("-" * 70)
    print(response.answer)
    print()

    if response.sources:
        print(f"SOURCES ({response.num_sources}):")
        print("-" * 70)
        for source in response.sources:
            print(
                f"  [{source['rank']}] {source['section']} "
                f"(score: {source['similarity_score']:.3f})"
            )
            print(f"      {source['url']}")
        print()

    print("METADATA:")
    print("-" * 70)
    print(f"  Model: {response.model}")
    print(f"  Temperature: {response.temperature}")
    print(f"  Grounded: {response.is_grounded}")
    print(f"  Has Citations: {response.has_citations}")
    print()


def test_basic_rag(search_engine: VectorSearchEngine, generator: GeminiAnswerGenerator) -> None:
    """Test basic RAG pipeline."""
    print_separator("Test 1: Basic RAG Pipeline")

    query = "What are the key components of a humanoid robot?"
    print(f"Query: {query}\n")

    try:
        # Step 1: Retrieve relevant chunks
        logger.info("Step 1: Retrieving relevant chunks")
        print("Retrieving relevant chunks...")
        results = search_engine.search(query, top_k=5)
        print(f"  Retrieved {len(results)} chunks\n")

        # Step 2: Generate answer
        logger.info("Step 2: Generating answer")
        print("Generating answer with Gemini...")
        response = generator.generate_chat_response(query, results)
        print("  Answer generated\n")

        # Step 3: Display response
        print_response(response)

    except Exception as e:
        logger.error(f"Basic RAG test failed: {e}", exc_info=True)
        print(f"ERROR: {e}\n")


def test_multi_turn_conversation(
    search_engine: VectorSearchEngine, generator: GeminiAnswerGenerator
) -> None:
    """Test multi-turn conversation with history."""
    print_separator("Test 2: Multi-Turn Conversation")

    conversation = [
        "What are sensors used in robotics?",
        "How do they integrate with actuators?",
        "What are the main challenges?",
    ]

    conversation_history = []

    for i, query in enumerate(conversation, 1):
        print(f"\n--- Turn {i} ---")
        print(f"User: {query}\n")

        try:
            # Retrieve and generate
            results = search_engine.search(query, top_k=3)
            response = generator.generate_chat_response(
                query, results, conversation_history
            )

            print(f"Assistant: {response.answer}\n")
            print(f"Sources: {response.num_sources}, Grounded: {response.is_grounded}")

            # Update history
            conversation_history.append({"role": "user", "content": query})
            conversation_history.append({"role": "assistant", "content": response.answer})

        except Exception as e:
            logger.error(f"Turn {i} failed: {e}", exc_info=True)
            print(f"ERROR: {e}\n")
            break

    print()


def test_no_context_handling(
    search_engine: VectorSearchEngine, generator: GeminiAnswerGenerator
) -> None:
    """Test handling of queries with no relevant context."""
    print_separator("Test 3: No Context Handling")

    # Query unlikely to match book content
    query = "What is the weather like today?"
    print(f"Query: {query}\n")

    try:
        # Try with high threshold (likely no results)
        results = search_engine.search(query, top_k=5, similarity_threshold=0.80)
        print(f"Retrieved {len(results)} results\n")

        response = generator.generate_chat_response(query, results)
        print_response(response)

        # Check if response correctly indicates lack of context
        if any(
            phrase in response.answer.lower()
            for phrase in ["cannot answer", "don't know", "no information"]
        ):
            print("✓ Correctly indicated lack of context\n")
        else:
            print("✗ WARNING: May have hallucinated without context\n")

    except Exception as e:
        logger.error(f"No context test failed: {e}", exc_info=True)
        print(f"ERROR: {e}\n")


def test_citation_validation(
    search_engine: VectorSearchEngine, generator: GeminiAnswerGenerator
) -> None:
    """Test citation presence and formatting."""
    print_separator("Test 4: Citation Validation")

    queries = [
        "What is physical AI?",
        "Describe the role of machine learning in robotics",
        "What are the ethical considerations in AI?",
    ]

    for query in queries:
        print(f"Query: {query}")

        try:
            results = search_engine.search(query, top_k=3)
            response = generator.generate_chat_response(query, results)

            has_citations = response.has_citations
            print(f"  Has Citations: {has_citations}")

            if not has_citations and len(results) > 0:
                print(f"  ✗ WARNING: No citations despite {len(results)} sources")
            elif has_citations:
                print(f"  ✓ Citations present")

        except Exception as e:
            print(f"  ERROR: {e}")

        print()


def test_response_validation(generator: GeminiAnswerGenerator) -> None:
    """Test response validation logic."""
    print_separator("Test 5: Response Validation")

    test_cases = [
        (
            "This is a good answer [Source 1] with citations [Source 2].",
            "some context",
            "Valid answer with citations",
        ),
        (
            "This is an answer without citations.",
            "some context",
            "Answer missing citations",
        ),
        (
            "Based on my knowledge, this is true.",
            "some context",
            "Hallucination phrase detected",
        ),
        (
            "Short",
            "some context",
            "Very short answer",
        ),
        (
            "I cannot answer this based on the context.",
            "",
            "Correctly indicates no context",
        ),
    ]

    for answer, context, description in test_cases:
        print(f"Testing: {description}")
        validation = generator.validate_response(answer, context)

        print(f"  Valid: {validation['is_valid']}")
        print(f"  Has Citations: {validation['has_citations']}")
        if validation['warnings']:
            print(f"  Warnings: {', '.join(validation['warnings'])}")
        print()


def test_prompt_building(generator: GeminiAnswerGenerator) -> None:
    """Test prompt building with different inputs."""
    print_separator("Test 6: Prompt Building")

    query = "What is physical AI?"
    context = "[Source 1: Introduction]\nPhysical AI refers to AI systems in robots."

    # Test 1: Basic prompt
    print("Test 6a: Basic prompt (no history)")
    prompt1 = generator.build_prompt(query, context)
    print(f"  Prompt length: {len(prompt1)} characters")
    print(f"  Contains system prompt: {generator.SYSTEM_PROMPT[:50] in prompt1}")
    print()

    # Test 2: Prompt with history
    print("Test 6b: Prompt with conversation history")
    history = [
        {"role": "user", "content": "What are robots?"},
        {"role": "assistant", "content": "Robots are machines."},
    ]
    prompt2 = generator.build_prompt(query, context, history)
    print(f"  Prompt length: {len(prompt2)} characters")
    print(f"  Contains history: {'CONVERSATION HISTORY' in prompt2}")
    print()

    # Test 3: Prompt with empty context
    print("Test 6c: Prompt with empty context")
    prompt3 = generator.build_prompt(query, "")
    print(f"  Prompt length: {len(prompt3)} characters")
    print(f"  Contains 'No relevant context': {'No relevant context' in prompt3}")
    print()


def main() -> int:
    """Main entry point."""
    # Setup logging
    settings = get_settings()
    setup_logging(
        log_level=settings.log_level,
        log_file="logs/generation_test.log"
    )

    logger.info("Starting answer generation testing")
    print_separator("ANSWER GENERATION TESTING")

    # Initialize components
    try:
        print("Initializing components...")
        search_engine = VectorSearchEngine(top_k=5, similarity_threshold=0.70)
        generator = GeminiAnswerGenerator()
        print("  ✓ Components initialized\n")

    except Exception as e:
        logger.error(f"Failed to initialize: {e}", exc_info=True)
        print(f"FATAL ERROR: {e}")
        return 1

    # Run tests
    try:
        test_basic_rag(search_engine, generator)
        test_multi_turn_conversation(search_engine, generator)
        test_no_context_handling(search_engine, generator)
        test_citation_validation(search_engine, generator)
        test_response_validation(generator)
        test_prompt_building(generator)

        print_separator("ALL TESTS COMPLETED")
        logger.info("All generation tests completed successfully")
        return 0

    except KeyboardInterrupt:
        logger.warning("Testing interrupted by user")
        print("\nTesting interrupted by user")
        return 1

    except Exception as e:
        logger.error(f"Testing failed: {e}", exc_info=True)
        print(f"\nFATAL ERROR: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
