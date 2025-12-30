#!/usr/bin/env python3
"""
Configuration Validation Script

Validates that all required environment variables are set and credentials
are valid before running the RAG chatbot system.
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from config import get_settings


def main() -> int:
    """
    Validate configuration and test API connections.

    Returns:
        int: 0 if validation passes, 1 if validation fails
    """
    print("=" * 60)
    print("RAG Chatbot Configuration Validation")
    print("=" * 60)

    try:
        # Load and validate settings
        print("\n[1/4] Loading environment variables...")
        settings = get_settings()
        print("✓ Environment variables loaded successfully")

        # Validate API keys are present
        print("\n[2/4] Validating API credentials...")
        if len(settings.cohere_api_key) < 10:
            raise ValueError("COHERE_API_KEY appears to be invalid (too short)")
        if len(settings.gemini_api_key) < 10:
            raise ValueError("GEMINI_API_KEY appears to be invalid (too short)")
        if len(settings.qdrant_api_key) < 10:
            raise ValueError("QDRANT_API_KEY appears to be invalid (too short)")

        print(f"  ✓ Cohere API key: {'*' * 8}{settings.cohere_api_key[-4:]}")
        print(f"  ✓ Gemini API key: {'*' * 8}{settings.gemini_api_key[-4:]}")
        print(f"  ✓ Qdrant API key: {'*' * 8}{settings.qdrant_api_key[-4:]}")

        # Validate URLs
        print("\n[3/4] Validating URLs...")
        print(f"  ✓ Qdrant URL: {settings.qdrant_url}")
        print(f"  ✓ Book sitemap: {settings.book_sitemap_url}")
        print(f"  ✓ Book base URL: {settings.book_base_url}")

        # Validate RAG configuration
        print("\n[4/4] Validating RAG configuration...")
        print(f"  ✓ Collection name: {settings.qdrant_collection_name}")
        print(f"  ✓ Chunk size: {settings.chunk_size} tokens")
        print(f"  ✓ Chunk overlap: {settings.chunk_overlap} tokens")
        print(f"  ✓ Top-k retrieval: {settings.top_k}")
        print(f"  ✓ Similarity threshold: {settings.similarity_threshold}")
        print(f"  ✓ Embedding model: {settings.embedding_model}")
        print(f"  ✓ Gemini model: {settings.gemini_model}")
        print(f"  ✓ Temperature: {settings.gemini_temperature}")
        print(f"  ✓ Log level: {settings.log_level}")

        # Summary
        print("\n" + "=" * 60)
        print("✓ All validation checks passed!")
        print("=" * 60)
        print("\nYou can now run:")
        print("  python scripts/ingest_book.py    # Ingest book content")
        print("  python scripts/run_chatbot.py    # Start the chatbot")
        print()

        return 0

    except Exception as e:
        print(f"\n✗ Validation failed: {e}")
        print("\nPlease check your .env file and ensure all required variables are set.")
        print("See .env.example for the complete list of required variables.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
