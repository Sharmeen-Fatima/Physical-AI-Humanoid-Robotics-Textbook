#!/usr/bin/env python3
"""
Retrieval Testing Script

Tests the VectorSearchEngine with sample queries to validate:
1. Query embedding generation
2. Vector similarity search
3. Result ranking and formatting
4. Context generation for LLM

Usage:
    python scripts/test_retrieval.py
"""

import sys
from pathlib import Path
from typing import List

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from config import get_settings
from retrieval import VectorSearchEngine
from storage.schemas import RetrievalResult
from utils.logger import setup_logging, get_logger

logger = get_logger(__name__)


def print_separator(title: str = "") -> None:
    """Print a visual separator."""
    if title:
        print(f"\n{'=' * 60}")
        print(f"  {title}")
        print(f"{'=' * 60}\n")
    else:
        print(f"{'=' * 60}\n")


def print_results(results: List[RetrievalResult]) -> None:
    """Pretty print retrieval results."""
    if not results:
        print("  No results found.\n")
        return

    for result in results:
        print(f"  [{result.rank}] Score: {result.similarity_score:.4f}")
        print(f"      Section: {result.section}")
        print(f"      URL: {result.url}")
        print(f"      Chunk: {result.chunk_index}")
        print(f"      Text: {result.text[:200]}...")
        print()


def test_basic_search(engine: VectorSearchEngine) -> None:
    """Test basic search functionality."""
    print_separator("Test 1: Basic Search")

    query = "What are the key components of a humanoid robot?"
    logger.info(f"Query: {query}")
    print(f"Query: {query}\n")

    try:
        results = engine.search(query)
        print(f"Found {len(results)} results:\n")
        print_results(results)

    except Exception as e:
        logger.error(f"Search failed: {e}", exc_info=True)
        print(f"ERROR: {e}\n")


def test_search_with_context(engine: VectorSearchEngine) -> None:
    """Test search with formatted context."""
    print_separator("Test 2: Search with Context")

    query = "How do sensors work in physical AI systems?"
    logger.info(f"Query: {query}")
    print(f"Query: {query}\n")

    try:
        response = engine.search_with_context(query)

        print(f"Query: {response['query']}")
        print(f"Number of results: {response['num_results']}\n")

        print("Formatted Context:")
        print("-" * 60)
        print(response['context'][:1000])  # Print first 1000 chars
        if len(response['context']) > 1000:
            print(f"\n... (truncated, total {len(response['context'])} chars)")
        print()

    except Exception as e:
        logger.error(f"Search with context failed: {e}", exc_info=True)
        print(f"ERROR: {e}\n")


def test_threshold_filtering(engine: VectorSearchEngine) -> None:
    """Test similarity threshold filtering."""
    print_separator("Test 3: Threshold Filtering")

    query = "machine learning algorithms"
    thresholds = [0.60, 0.70, 0.80]

    print(f"Query: {query}\n")

    for threshold in thresholds:
        try:
            results = engine.search(query, similarity_threshold=threshold)
            print(f"Threshold {threshold:.2f}: {len(results)} results")
            if results:
                scores = [f"{r.similarity_score:.3f}" for r in results]
                print(f"  Scores: {', '.join(scores)}")
        except Exception as e:
            print(f"  ERROR at threshold {threshold}: {e}")
        print()


def test_top_k_variation(engine: VectorSearchEngine) -> None:
    """Test different top_k values."""
    print_separator("Test 4: Top-K Variation")

    query = "actuators and motors in robotics"
    k_values = [1, 3, 5, 10]

    print(f"Query: {query}\n")

    for k in k_values:
        try:
            results = engine.search(query, top_k=k)
            print(f"top_k={k}: Retrieved {len(results)} results")
            if results:
                avg_score = sum(r.similarity_score for r in results) / len(results)
                print(f"  Average score: {avg_score:.4f}")
        except Exception as e:
            print(f"  ERROR with top_k={k}: {e}")
        print()


def test_edge_cases(engine: VectorSearchEngine) -> None:
    """Test edge cases and error handling."""
    print_separator("Test 5: Edge Cases")

    test_cases = [
        ("", "Empty query"),
        ("   ", "Whitespace query"),
        ("x", "Single character"),
        ("a" * 1000, "Very long query (1000 chars)"),
    ]

    for query, description in test_cases:
        print(f"Testing: {description}")
        try:
            results = engine.search(query)
            print(f"  SUCCESS: {len(results)} results\n")
        except ValueError as e:
            print(f"  EXPECTED ERROR: {e}\n")
        except Exception as e:
            print(f"  UNEXPECTED ERROR: {e}\n")


def test_engine_stats(engine: VectorSearchEngine) -> None:
    """Test engine statistics."""
    print_separator("Test 6: Engine Statistics")

    try:
        stats = engine.get_stats()

        print("Configuration:")
        for key, value in stats['config'].items():
            print(f"  {key}: {value}")

        print("\nCollection:")
        for key, value in stats['collection'].items():
            print(f"  {key}: {value}")
        print()

    except Exception as e:
        logger.error(f"Failed to get stats: {e}", exc_info=True)
        print(f"ERROR: {e}\n")


def main() -> int:
    """Main entry point."""
    # Setup logging
    settings = get_settings()
    setup_logging(
        log_level=settings.log_level,
        log_file="logs/retrieval_test.log"
    )

    logger.info("Starting retrieval testing")
    print_separator("RETRIEVAL ENGINE TESTING")

    # Initialize search engine
    try:
        engine = VectorSearchEngine(
            top_k=5,
            similarity_threshold=0.70,
        )
        logger.info("VectorSearchEngine initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize search engine: {e}", exc_info=True)
        print(f"FATAL ERROR: Could not initialize search engine: {e}")
        return 1

    # Run tests
    try:
        test_basic_search(engine)
        test_search_with_context(engine)
        test_threshold_filtering(engine)
        test_top_k_variation(engine)
        test_edge_cases(engine)
        test_engine_stats(engine)

        print_separator("ALL TESTS COMPLETED")
        logger.info("All retrieval tests completed successfully")
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
