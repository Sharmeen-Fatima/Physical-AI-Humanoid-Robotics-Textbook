#!/usr/bin/env python3
"""
Simple chatbot test - validates the end-to-end RAG pipeline without interactive CLI.

This script:
1. Initializes all components (search engine, generator, session manager)
2. Performs a few sample queries
3. Validates grounding, citations, and multi-turn conversation

Usage:
    python test_chatbot_simple.py
"""

import sys
from pathlib import Path

# Add backend to path for src imports
backend_path = Path(__file__).parent
sys.path.insert(0, str(backend_path))

from src.config import get_settings
from src.retrieval import VectorSearchEngine
from src.generation import GeminiAnswerGenerator
from src.chatbot.session_manager import SessionManager
from src.utils.logger import setup_logging, get_logger

logger = get_logger(__name__)


def print_separator(title=""):
    """Print a visual separator."""
    if title:
        print(f"\n{'='*70}")
        print(f"  {title}")
        print(f"{'='*70}\n")
    else:
        print(f"{'='*70}\n")


def test_initialization():
    """Test 1: Initialize all components."""
    print_separator("TEST 1: Component Initialization")

    try:
        # Initialize components
        print("Initializing VectorSearchEngine...")
        search_engine = VectorSearchEngine(top_k=5, similarity_threshold=0.50)
        print("[OK] VectorSearchEngine initialized\n")

        print("Initializing GeminiAnswerGenerator...")
        generator = GeminiAnswerGenerator()
        print("[OK] GeminiAnswerGenerator initialized\n")

        print("Initializing SessionManager...")
        session_manager = SessionManager(session_timeout_minutes=30)
        session_id = session_manager.create_session()
        print(f"[OK] SessionManager initialized (session: {session_id})\n")

        return search_engine, generator, session_manager, session_id

    except Exception as e:
        print(f"[FAIL] Initialization failed: {e}")
        logger.error(f"Initialization error: {e}", exc_info=True)
        return None, None, None, None


def test_single_query(search_engine, generator, query):
    """Test 2: Single query with retrieval and generation."""
    print_separator(f"TEST 2: Single Query - {query[:50]}...")

    try:
        # Step 1: Retrieve
        print("[1/3] Retrieving relevant chunks...")
        results = search_engine.search(query, top_k=5)
        print(f"  [OK] Retrieved {len(results)} chunks")

        if results:
            print(f"  Top result score: {results[0].similarity_score:.3f}")

        # Step 2: Generate answer
        print("\n[2/3] Generating answer with Gemini...")
        response = generator.generate_chat_response(query, results)
        print(f"  [OK] Answer generated ({len(response.answer)} chars)")

        # Step 3: Display response
        print("\n[3/3] Response:")
        print("-" * 70)
        print(f"QUERY: {response.query}\n")
        print(f"ANSWER:\n{response.answer}\n")

        if response.sources:
            print(f"SOURCES ({response.num_sources}):")
            for source in response.sources[:3]:  # Show top 3
                print(f"  [{source['rank']}] {source['section']}")
                print(f"      Score: {source['similarity_score']:.3f}")
                print(f"      {source['url']}")

        print(f"\nMETADATA:")
        print(f"  Model: {response.model}")
        print(f"  Grounded: {response.is_grounded}")
        print(f"  Has Citations: {response.has_citations}")
        print(f"  Temperature: {response.temperature}")

        return response

    except Exception as e:
        print(f"[FAIL] Query test failed: {e}")
        logger.error(f"Query error: {e}", exc_info=True)
        return None


def test_multi_turn(search_engine, generator, session_manager, session_id):
    """Test 3: Multi-turn conversation."""
    print_separator("TEST 3: Multi-Turn Conversation")

    queries = [
        "What is Physical AI?",
        "How does it relate to robotics?",
    ]

    try:
        for i, query in enumerate(queries, 1):
            print(f"\n--- Turn {i} ---")
            print(f"User: {query}\n")

            # Get conversation history
            history = session_manager.get_conversation_history(session_id)

            # Retrieve and generate
            results = search_engine.search(query, top_k=3)
            print(f"  Retrieved {len(results)} chunks")

            response = generator.generate_chat_response(query, results, history)
            print(f"  Generated {len(response.answer)} char answer")

            # Update session
            session_manager.update_session(
                session_id,
                user_message=query,
                assistant_message=response.answer
            )

            print(f"\nAssistant: {response.answer[:200]}...")
            print(f"\n  Grounded: {response.is_grounded} | Citations: {response.has_citations}")

        # Check final history
        final_history = session_manager.get_conversation_history(session_id)
        print(f"\n[OK] Conversation history: {len(final_history)} messages ({len(final_history)//2} turns)")

        return True

    except Exception as e:
        print(f"[FAIL] Multi-turn test failed: {e}")
        logger.error(f"Multi-turn error: {e}", exc_info=True)
        return False


def test_grounding_and_citations(response):
    """Test 4: Validate grounding and citations."""
    print_separator("TEST 4: Grounding & Citation Validation")

    if not response:
        print("[FAIL] No response to validate")
        return False

    print(f"Answer length: {len(response.answer)} chars")
    print(f"Grounded: {response.is_grounded}")
    print(f"Has citations: {response.has_citations}")
    print(f"Number of sources: {response.num_sources}")

    # Check for citation markers
    import re
    citations = re.findall(r'\[Source \d+\]', response.answer)
    print(f"Citation markers found: {len(citations)}")

    # Validation checks
    checks = []
    checks.append(("Has sources", response.num_sources > 0))
    checks.append(("Is grounded", response.is_grounded))
    checks.append(("Has citations", response.has_citations))
    checks.append(("Citation markers present", len(citations) > 0))

    print("\nValidation Checks:")
    for check_name, passed in checks:
        status = "[OK]" if passed else "[FAIL]"
        print(f"  {status} {check_name}")

    all_passed = all(check[1] for check in checks)

    if all_passed:
        print("\n[OK] All grounding and citation checks passed!")
    else:
        print("\n[WARN] Some checks failed - review answer generation")

    return all_passed


def main():
    """Main test runner."""
    # Setup logging
    settings = get_settings()
    setup_logging(log_level=settings.log_level, log_file="logs/chatbot_test.log")

    logger.info("Starting chatbot end-to-end test")

    print("\n" + "="*70)
    print("  RAG CHATBOT END-TO-END TEST")
    print("="*70)

    # Test 1: Initialization
    search_engine, generator, session_manager, session_id = test_initialization()
    if not search_engine:
        print("\n[FAIL] TEST SUITE FAILED: Initialization error")
        return 1

    # Test 2: Single query
    test_query = "What are the main components of a humanoid robot?"
    response = test_single_query(search_engine, generator, test_query)

    # Test 3: Multi-turn conversation
    multi_turn_success = test_multi_turn(search_engine, generator, session_manager, session_id)

    # Test 4: Grounding and citations
    grounding_success = test_grounding_and_citations(response)

    # Summary
    print_separator("TEST SUMMARY")

    tests = [
        ("Component Initialization", search_engine is not None),
        ("Single Query Pipeline", response is not None),
        ("Multi-Turn Conversation", multi_turn_success),
        ("Grounding & Citations", grounding_success)
    ]

    for test_name, passed in tests:
        status = "[OK] PASS" if passed else "[FAIL] FAIL"
        print(f"  {status}: {test_name}")

    all_passed = all(test[1] for test in tests)

    print()
    if all_passed:
        print("="*70)
        print("  [OK] ALL TESTS PASSED - RAG CHATBOT IS OPERATIONAL")
        print("="*70)
        print("\nYou can now run: python scripts/chatbot.py")
        return 0
    else:
        print("="*70)
        print("  [FAIL] SOME TESTS FAILED - REVIEW ERRORS ABOVE")
        print("="*70)
        return 1


if __name__ == "__main__":
    sys.exit(main())
