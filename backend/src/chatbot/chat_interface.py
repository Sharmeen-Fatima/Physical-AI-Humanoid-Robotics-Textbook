"""
CLI chat interface for the RAG chatbot.

Integrates retrieval, answer generation, and session management
into an interactive command-line interface.
"""

from typing import Optional

from src.retrieval import VectorSearchEngine
from src.generation import GeminiAnswerGenerator
from src.chatbot.session_manager import SessionManager
from src.config import get_settings
from src.utils.logger import get_logger

logger = get_logger(__name__)


class ChatInterface:
    """
    Interactive CLI chat interface for the RAG chatbot.

    Features:
    - Session-based conversations
    - Context-aware multi-turn dialogue
    - Automatic session cleanup
    - Formatted output with sources
    - Command handling (/help, /stats, /clear, /quit)
    """

    def __init__(
        self,
        top_k: int = 5,
        similarity_threshold: float = 0.70,
        session_timeout_minutes: int = 30,
    ):
        """
        Initialize chat interface.

        Args:
            top_k: Number of chunks to retrieve (default: 5)
            similarity_threshold: Minimum similarity score (default: 0.70)
            session_timeout_minutes: Session expiration time (default: 30)
        """
        logger.info("Initializing ChatInterface")

        # Initialize components
        self.search_engine = VectorSearchEngine(
            top_k=top_k, similarity_threshold=similarity_threshold
        )
        self.generator = GeminiAnswerGenerator()
        self.session_manager = SessionManager(
            session_timeout_minutes=session_timeout_minutes
        )

        # Create session
        self.session_id = self.session_manager.create_session()

        # Settings
        self.top_k = top_k
        self.similarity_threshold = similarity_threshold

        logger.info(
            f"ChatInterface initialized: session={self.session_id}, "
            f"top_k={top_k}, threshold={similarity_threshold}"
        )

    def print_welcome(self) -> None:
        """Print welcome message."""
        print("\n" + "=" * 70)
        print("  Welcome to the Physical AI & Humanoid Robotics Chatbot!")
        print("=" * 70)
        print("\nI can answer questions about the book using RAG (Retrieval-Augmented")
        print("Generation). My responses are grounded in the book content.\n")
        print("Commands:")
        print("  /help   - Show help message")
        print("  /stats  - Show session statistics")
        print("  /clear  - Clear conversation history")
        print("  /quit   - Exit chatbot")
        print("\nType your question and press Enter.\n")
        print("-" * 70)

    def print_help(self) -> None:
        """Print help message."""
        print("\n" + "=" * 70)
        print("  HELP")
        print("=" * 70)
        print("\nAvailable Commands:")
        print("  /help   - Show this help message")
        print("  /stats  - Show session statistics")
        print("  /clear  - Clear conversation history (start fresh)")
        print("  /quit   - Exit the chatbot")
        print("\nHow to use:")
        print("  - Ask questions about the Physical AI & Humanoid Robotics book")
        print("  - I'll retrieve relevant sections and provide grounded answers")
        print("  - All responses include source citations [Source N]")
        print("  - Your conversation history is preserved within the session")
        print("\nExamples:")
        print("  'What are the key components of a humanoid robot?'")
        print("  'How do sensors and actuators work together?'")
        print("  'Explain the role of machine learning in physical AI'")
        print("\n" + "-" * 70)

    def print_stats(self) -> None:
        """Print session statistics."""
        stats = self.session_manager.get_session_stats(self.session_id)
        if not stats:
            print("\n✗ Session statistics unavailable\n")
            return

        print("\n" + "=" * 70)
        print("  SESSION STATISTICS")
        print("=" * 70)
        print(f"\nSession ID: {stats['session_id']}")
        print(f"Created: {stats['created_at']}")
        print(f"Last Activity: {stats['last_activity']}")
        print(f"Duration: {stats['duration_seconds']} seconds")
        print(f"Conversation Turns: {stats['num_turns']}")
        print(f"Expired: {stats['is_expired']}")
        print("\n" + "-" * 70)

    def clear_history(self) -> None:
        """Clear conversation history and start a new session."""
        old_session_id = self.session_id
        self.session_manager.delete_session(old_session_id)
        self.session_id = self.session_manager.create_session()

        print(f"\n✓ Conversation history cleared. New session: {self.session_id}\n")
        logger.info(
            f"Cleared history: deleted {old_session_id}, created {self.session_id}"
        )

    def process_query(self, query: str) -> None:
        """
        Process user query and generate response.

        Args:
            query: User's question
        """
        logger.info(f"Processing query: {query[:100]}...")

        try:
            # Step 1: Retrieve relevant chunks
            print("  [Searching book content...]")
            results = self.search_engine.search(
                query, top_k=self.top_k, similarity_threshold=self.similarity_threshold
            )

            if not results:
                print("\n✗ No relevant content found in the book for this query.\n")
                logger.warning(f"No results found for query: {query}")
                return

            print(f"  [Found {len(results)} relevant sections]")

            # Step 2: Get conversation history
            conversation_history = self.session_manager.get_conversation_history(
                self.session_id
            )

            # Step 3: Generate answer
            print("  [Generating answer...]\n")
            response = self.generator.generate_chat_response(
                query, results, conversation_history
            )

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
                for source in response.sources:
                    print(
                        f"  [{source['rank']}] {source['section']} "
                        f"(score: {source['similarity_score']:.3f})"
                    )
                    print(f"      {source['url']}")
                print()

            # Display metadata
            print("METADATA:")
            print("-" * 70)
            print(
                f"  Model: {response.model} | Sources: {response.num_sources} | "
                f"Grounded: {response.is_grounded} | Citations: {response.has_citations}"
            )
            print("=" * 70)
            print()

            # Step 5: Update session
            self.session_manager.update_session(
                self.session_id,
                user_message=query,
                assistant_message=response.answer,
                metadata={
                    "num_sources": response.num_sources,
                    "is_grounded": response.is_grounded,
                },
            )

            logger.info(f"Query processed successfully: {len(response.answer)} chars")

        except Exception as e:
            logger.error(f"Failed to process query: {e}", exc_info=True)
            print(f"\n✗ ERROR: {e}\n")

    def handle_command(self, command: str) -> bool:
        """
        Handle special commands.

        Args:
            command: Command string (e.g., "/help")

        Returns:
            bool: True to continue, False to quit
        """
        command = command.lower().strip()

        if command == "/help":
            self.print_help()
            return True

        elif command == "/stats":
            self.print_stats()
            return True

        elif command == "/clear":
            self.clear_history()
            return True

        elif command in ["/quit", "/exit", "/q"]:
            print("\nThank you for using the chatbot. Goodbye!\n")
            return False

        else:
            print(f"\n✗ Unknown command: {command}")
            print("  Type /help for available commands.\n")
            return True

    def run(self) -> None:
        """
        Run the interactive chat loop.

        Main entry point for the CLI interface.
        """
        self.print_welcome()

        try:
            while True:
                # Cleanup expired sessions periodically
                self.session_manager.cleanup_expired_sessions()

                # Get user input
                try:
                    user_input = input("You: ").strip()
                except EOFError:
                    print("\nGoodbye!")
                    break

                # Skip empty input
                if not user_input:
                    continue

                # Handle commands
                if user_input.startswith("/"):
                    should_continue = self.handle_command(user_input)
                    if not should_continue:
                        break
                    continue

                # Process query
                print()
                self.process_query(user_input)

        except KeyboardInterrupt:
            print("\n\nInterrupted by user. Goodbye!\n")
            logger.info("Chat session interrupted by user")

        except Exception as e:
            logger.error(f"Chat interface error: {e}", exc_info=True)
            print(f"\n✗ FATAL ERROR: {e}\n")

        finally:
            # Cleanup session
            self.session_manager.delete_session(self.session_id)
            logger.info(f"Chat session ended: {self.session_id}")


def main() -> int:
    """Main entry point for CLI chatbot."""
    from src.utils.logger import setup_logging

    # Setup logging
    settings = get_settings()
    setup_logging(log_level=settings.log_level, log_file="logs/chatbot.log")

    logger.info("Starting CLI chatbot")

    try:
        # Create and run chat interface
        chat = ChatInterface(
            top_k=5,
            similarity_threshold=0.70,
            session_timeout_minutes=30,
        )
        chat.run()

        return 0

    except Exception as e:
        logger.error(f"Chatbot failed to start: {e}", exc_info=True)
        print(f"\n✗ FATAL ERROR: {e}\n")
        return 1


if __name__ == "__main__":
    import sys

    sys.exit(main())
