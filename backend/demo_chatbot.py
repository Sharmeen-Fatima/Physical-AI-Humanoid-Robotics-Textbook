#!/usr/bin/env python3
"""
RAG Chatbot Demo - Non-Interactive Demonstration

Demonstrates the chatbot functionality with sample queries.
"""

import sys
from pathlib import Path

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent))

from src.config import get_settings
from src.retrieval import VectorSearchEngine
from src.generation import GeminiAnswerGenerator
from src.chatbot.session_manager import SessionManager
from src.utils.logger import setup_logging, get_logger

logger = get_logger(__name__)


def print_separator(title=""):
    """Print visual separator."""
    if title:
        print(f"\n{'='*70}")
        print(f"  {title}")
        print(f"{'='*70}\n")
    else:
        print(f"{'='*70}\n")


def print_welcome():
    """Print welcome message."""
    print("\n" + "=" * 70)
    print("  RAG CHATBOT DEMONSTRATION")
    print("  Physical AI & Humanoid Robotics Book")
    print("=" * 70)
    print("\nThis demo showcases the complete RAG pipeline:")
    print("  1. Semantic retrieval from vector database")
    print("  2. Grounded answer generation with citations")
    print("  3. Multi-turn conversation with context")
    print("\n" + "-" * 70)


def demo_query(search_engine, generator, session_manager, session_id, query, turn_num):
    """Demonstrate a single query-response interaction."""
    print(f"\n--- Turn {turn_num} ---")
    print(f"You: {query}\n")

    # Step 1: Retrieve
    print("  [Searching book content...]")
    results = search_engine.search(query, top_k=5, similarity_threshold=0.60)
    print(f"  [Found {len(results)} relevant sections]")

    if not results:
        print("\n[!] No relevant content found for this query.\n")
        return False

    # Step 2: Get conversation history
    history = session_manager.get_conversation_history(session_id)

    # Step 3: Generate answer
    print("  [Generating answer...]\n")
    response = generator.generate_chat_response(query, results, history)

    # Step 4: Display response
    print("=" * 70)
    print("ANSWER:")
    print("=" * 70)
    print(response.answer)
    print()

    # Display sources
    if response.sources:
        print("SOURCES:")
        print("-" * 70)
        for source in response.sources[:3]:  # Show top 3
            print(f"  [{source['rank']}] {source['section']} (score: {source['similarity_score']:.3f})")
            print(f"      {source['url']}")
        print()

    # Display metadata
    print("METADATA:")
    print("-" * 70)
    print(f"  Model: {response.model}")
    print(f"  Sources: {response.num_sources}")
    print(f"  Grounded: {response.is_grounded}")
    print(f"  Citations: {response.has_citations}")
    print("=" * 70)
    print()

    # Step 5: Update session
    session_manager.update_session(
        session_id,
        user_message=query,
        assistant_message=response.answer
    )

    return True


def main():
    """Main demo runner."""
    # Setup logging
    settings = get_settings()
    setup_logging(log_level="WARNING", log_file="logs/chatbot_demo.log")

    print_welcome()

    # Initialize components
    print_separator("Initializing Components")

    print("  [1/3] Initializing VectorSearchEngine...")
    search_engine = VectorSearchEngine(top_k=5, similarity_threshold=0.60)
    print("  [OK] Search engine ready")

    print("  [2/3] Initializing GeminiAnswerGenerator...")
    generator = GeminiAnswerGenerator()
    print("  [OK] Generator ready")

    print("  [3/3] Initializing SessionManager...")
    session_manager = SessionManager(session_timeout_minutes=30)
    session_id = session_manager.create_session()
    print(f"  [OK] Session created: {session_id}")

    # Demo queries
    print_separator("Demo Conversation")

    queries = [
        "What is a humanoid robot?",
        "What are their main components?",
        "How do sensors work in robotics?",
    ]

    for i, query in enumerate(queries, 1):
        success = demo_query(search_engine, generator, session_manager, session_id, query, i)
        if not success:
            continue

    # Session statistics
    print_separator("Session Statistics")

    stats = session_manager.get_session_stats(session_id)
    print(f"  Session ID: {stats['session_id']}")
    print(f"  Duration: {stats['duration_seconds']} seconds")
    print(f"  Conversation Turns: {stats['num_turns']}")
    print(f"  Total Messages: {len(session_manager.get_conversation_history(session_id))}")
    print()

    # Cleanup
    session_manager.delete_session(session_id)

    print_separator("Demo Complete")
    print("  [SUCCESS] RAG chatbot demonstration completed successfully!")
    print()
    print("  To run the interactive chatbot:")
    print("    python scripts/chatbot.py")
    print()
    print("  Available commands:")
    print("    /help   - Show help message")
    print("    /stats  - Show session statistics")
    print("    /clear  - Clear conversation history")
    print("    /quit   - Exit chatbot")
    print()
    print("=" * 70 + "\n")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user.\n")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Demo failed: {e}\n")
        logger.error(f"Demo error: {e}", exc_info=True)
        sys.exit(1)
