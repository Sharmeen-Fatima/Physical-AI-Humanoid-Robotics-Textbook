#!/usr/bin/env python3
"""
Health Check Script

Validates that all system components are operational:
1. Configuration loaded successfully
2. API keys are valid and accessible
3. Qdrant collection exists and is accessible
4. Cohere API is reachable
5. Gemini API is reachable
6. All modules can be imported

Usage:
    python scripts/health_check.py
"""

import sys
from pathlib import Path
from typing import Dict, Any

# Add backend to path for src imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.config import get_settings
from src.storage import qdrant_client
from src.embedding import CohereEmbedder
from src.generation import GeminiAnswerGenerator
from src.utils.logger import setup_logging, get_logger

logger = get_logger(__name__)


def print_header(title: str) -> None:
    """Print section header."""
    print(f"\n{'=' * 70}")
    print(f"  {title}")
    print(f"{'=' * 70}\n")


def print_result(check: str, passed: bool, details: str = "") -> None:
    """Print check result."""
    status = "[OK] PASS" if passed else "[X] FAIL"
    print(f"  {status}: {check}")
    if details:
        print(f"         {details}")


def check_configuration() -> bool:
    """Check configuration loading."""
    print_header("1. Configuration Check")

    try:
        settings = get_settings()
        print_result("Settings loaded", True, f"Log level: {settings.log_level}")

        # Check required keys
        required_keys = [
            ("Cohere API key", settings.cohere_api_key),
            ("Gemini API key", settings.gemini_api_key),
            ("Qdrant URL", settings.qdrant_url),
            ("Qdrant API key", settings.qdrant_api_key),
        ]

        all_passed = True
        for name, value in required_keys:
            if value:
                print_result(name, True, f"{value[:10]}...")
            else:
                print_result(name, False, "Missing!")
                all_passed = False

        # Check configuration values
        chunk_valid = 100 <= settings.chunk_size <= 2000
        print_result(
            "Chunk size valid", chunk_valid, f"{settings.chunk_size} tokens"
        )

        overlap_valid = 0 <= settings.chunk_overlap < settings.chunk_size
        print_result(
            "Chunk overlap valid", overlap_valid, f"{settings.chunk_overlap} tokens"
        )

        temp_valid = 0.0 <= settings.gemini_temperature <= 2.0
        print_result(
            "Gemini temperature valid", temp_valid, f"{settings.gemini_temperature}"
        )

        return all_passed and chunk_valid and overlap_valid and temp_valid

    except Exception as e:
        print_result("Configuration", False, f"Error: {e}")
        return False


def check_qdrant() -> bool:
    """Check Qdrant connectivity and collection."""
    print_header("2. Qdrant Vector Database Check")

    try:
        client = qdrant_client.QdrantClient()
        print_result("Qdrant client initialized", True)

        # Check collection exists
        exists = client.collection_exists()
        print_result("Collection exists", exists, client.collection_name)

        if exists:
            # Get collection info
            info = client.get_collection_info()
            points_count = info.get("points_count", 0)
            vectors_count = info.get("vectors_count", 0)

            print_result(
                "Collection accessible",
                True,
                f"{points_count} points, {vectors_count} vectors",
            )

            return True
        else:
            print_result(
                "Collection accessible",
                False,
                "Collection does not exist. Run ingestion first.",
            )
            return False

    except Exception as e:
        print_result("Qdrant connection", False, f"Error: {e}")
        return False


def check_cohere() -> bool:
    """Check Cohere API connectivity."""
    print_header("3. Cohere Embeddings API Check")

    try:
        embedder = CohereEmbedder()
        print_result("Cohere client initialized", True, f"Model: {embedder.model}")

        # Test embedding generation
        test_texts = ["This is a test sentence for embedding."]
        embeddings = embedder.generate_embeddings(
            texts=test_texts, input_type="search_query"
        )

        if embeddings and len(embeddings) == 1:
            dimension = len(embeddings[0])
            print_result(
                "Embedding generation", True, f"Dimension: {dimension}"
            )
            return True
        else:
            print_result("Embedding generation", False, "No embeddings returned")
            return False

    except Exception as e:
        print_result("Cohere API", False, f"Error: {e}")
        return False


def check_gemini() -> bool:
    """Check Gemini API connectivity."""
    print_header("4. Gemini LLM API Check")

    try:
        generator = GeminiAnswerGenerator()
        print_result("Gemini client initialized", True, f"Model: {generator.model.model_name}")

        # Test answer generation with minimal prompt
        test_query = "What is AI?"
        test_context = "AI stands for Artificial Intelligence."

        answer = generator.generate_answer(test_query, test_context)

        if answer and len(answer) > 0:
            print_result(
                "Answer generation", True, f"{len(answer)} characters"
            )
            return True
        else:
            print_result("Answer generation", False, "Empty response")
            return False

    except Exception as e:
        print_result("Gemini API", False, f"Error: {e}")
        return False


def check_imports() -> bool:
    """Check that all modules can be imported."""
    print_header("5. Module Import Check")

    modules = [
        ("src.config", "Configuration module"),
        ("src.storage.qdrant_client", "Qdrant client"),
        ("src.storage.schemas", "Data schemas"),
        ("src.ingestion.sitemap_parser", "Sitemap parser"),
        ("src.ingestion.web_scraper", "Web scraper"),
        ("src.ingestion.chunker", "Text chunker"),
        ("src.embedding.cohere_embedder", "Cohere embedder"),
        ("src.retrieval.vector_search", "Vector search"),
        ("src.generation.gemini_generator", "Gemini generator"),
        ("src.chatbot.session_manager", "Session manager"),
        ("src.chatbot.chat_interface", "Chat interface"),
        ("src.utils.logger", "Logger utilities"),
        ("src.utils.retry", "Retry utilities"),
    ]

    import importlib

    all_passed = True
    for module_name, description in modules:
        try:
            importlib.import_module(module_name)
            print_result(description, True, module_name)
        except ImportError as e:
            print_result(description, False, f"ImportError: {e}")
            all_passed = False

    return all_passed


def check_directories() -> bool:
    """Check that required directories exist."""
    print_header("6. Directory Structure Check")

    base_path = Path(__file__).parent.parent
    required_dirs = [
        "src",
        "src/config",
        "src/storage",
        "src/ingestion",
        "src/embedding",
        "src/retrieval",
        "src/generation",
        "src/chatbot",
        "src/utils",
        "scripts",
        "tests",
        "logs",
    ]

    all_passed = True
    for dir_name in required_dirs:
        dir_path = base_path / dir_name
        exists = dir_path.exists() and dir_path.is_dir()
        print_result(f"Directory: {dir_name}", exists)
        if not exists:
            all_passed = False

    return all_passed


def main() -> int:
    """Main entry point."""
    # Setup logging
    setup_logging(log_level="INFO", log_file="logs/health_check.log")

    print("\n" + "=" * 70)
    print("  RAG CHATBOT SYSTEM - HEALTH CHECK")
    print("=" * 70)

    logger.info("Starting health check")

    # Run all checks
    results = {}
    results["configuration"] = check_configuration()
    results["qdrant"] = check_qdrant()
    results["cohere"] = check_cohere()
    results["gemini"] = check_gemini()
    results["imports"] = check_imports()
    results["directories"] = check_directories()

    # Summary
    print_header("HEALTH CHECK SUMMARY")

    total_checks = len(results)
    passed_checks = sum(1 for passed in results.values() if passed)
    failed_checks = total_checks - passed_checks

    print(f"  Total Checks:  {total_checks}")
    print(f"  Passed:        {passed_checks} [OK]")
    print(f"  Failed:        {failed_checks} [X]")
    print()

    if failed_checks == 0:
        print("  [SUCCESS] ALL SYSTEMS OPERATIONAL")
        print("\n" + "=" * 70 + "\n")
        logger.info("Health check passed: all systems operational")
        return 0
    else:
        print("  [WARNING] SOME SYSTEMS FAILED")
        print("\n  Failed checks:")
        for check, passed in results.items():
            if not passed:
                print(f"    - {check}")
        print("\n" + "=" * 70 + "\n")
        logger.warning(f"Health check failed: {failed_checks} checks failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
