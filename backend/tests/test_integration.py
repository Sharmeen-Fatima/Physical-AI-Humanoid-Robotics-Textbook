#!/usr/bin/env python3
"""
Integration Tests for RAG Chatbot System

Tests the complete end-to-end pipeline:
1. Configuration and component initialization
2. Retrieval pipeline (query → embedding → search)
3. Generation pipeline (context → answer → validation)
4. Session management
5. Full RAG workflow (query → retrieval → generation → response)
"""

import sys
from pathlib import Path
import pytest

# Add backend to path for src imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.config import get_settings
from src.retrieval import VectorSearchEngine
from src.generation import GeminiAnswerGenerator
from src.chatbot import SessionManager, ChatInterface
from src.storage import qdrant_client
from src.utils.logger import setup_logging, get_logger

logger = get_logger(__name__)


class TestConfiguration:
    """Test configuration loading and validation."""

    def test_settings_load(self):
        """Test that settings load successfully."""
        settings = get_settings()
        assert settings is not None
        logger.info("✓ Settings loaded successfully")

    def test_required_keys(self):
        """Test that all required API keys are present."""
        settings = get_settings()
        assert settings.cohere_api_key is not None
        assert settings.gemini_api_key is not None
        assert settings.qdrant_url is not None
        assert settings.qdrant_api_key is not None
        logger.info("✓ All required API keys present")

    def test_configuration_values(self):
        """Test that configuration values are in valid ranges."""
        settings = get_settings()
        assert 100 <= settings.chunk_size <= 2000
        assert 0 <= settings.chunk_overlap <= 500
        assert settings.chunk_overlap < settings.chunk_size
        assert 0.0 <= settings.gemini_temperature <= 2.0
        logger.info("✓ Configuration values are valid")


class TestComponentInitialization:
    """Test that all components initialize correctly."""

    def test_vector_search_init(self):
        """Test VectorSearchEngine initialization."""
        engine = VectorSearchEngine(top_k=5, similarity_threshold=0.70)
        assert engine is not None
        assert engine.top_k == 5
        assert engine.similarity_threshold == 0.70
        logger.info("✓ VectorSearchEngine initialized")

    def test_answer_generator_init(self):
        """Test GeminiAnswerGenerator initialization."""
        generator = GeminiAnswerGenerator()
        assert generator is not None
        assert generator.model is not None
        logger.info("✓ GeminiAnswerGenerator initialized")

    def test_session_manager_init(self):
        """Test SessionManager initialization."""
        manager = SessionManager(session_timeout_minutes=30)
        assert manager is not None
        assert manager.session_timeout.total_seconds() == 1800
        logger.info("✓ SessionManager initialized")

    def test_qdrant_client_init(self):
        """Test QdrantClient initialization."""
        client = qdrant_client.QdrantClient()
        assert client is not None
        assert client.collection_name is not None
        logger.info("✓ QdrantClient initialized")


class TestRetrievalPipeline:
    """Test the retrieval pipeline."""

    @pytest.fixture
    def search_engine(self):
        """Create search engine fixture."""
        return VectorSearchEngine(top_k=5, similarity_threshold=0.70)

    def test_basic_search(self, search_engine):
        """Test basic semantic search."""
        query = "What are the key components of a humanoid robot?"
        results = search_engine.search(query)

        assert isinstance(results, list)
        # Results may be empty if collection is empty
        logger.info(f"✓ Basic search completed: {len(results)} results")

    def test_search_with_threshold(self, search_engine):
        """Test search with different thresholds."""
        query = "machine learning in robotics"

        results_low = search_engine.search(query, similarity_threshold=0.60)
        results_high = search_engine.search(query, similarity_threshold=0.80)

        # Higher threshold should return fewer or equal results
        assert len(results_high) <= len(results_low)
        logger.info("✓ Threshold filtering works correctly")

    def test_search_with_context(self, search_engine):
        """Test search_with_context method."""
        query = "sensors in robotics"
        response = search_engine.search_with_context(query)

        assert "query" in response
        assert "results" in response
        assert "context" in response
        assert "num_results" in response
        assert response["query"] == query
        logger.info("✓ Search with context completed")

    def test_format_context(self, search_engine):
        """Test context formatting."""
        query = "actuators"
        results = search_engine.search(query, top_k=3)

        context = search_engine.format_context(results)
        assert isinstance(context, str)

        if results:
            assert "[Source 1:" in context
        logger.info("✓ Context formatting works")

    def test_empty_query_handling(self, search_engine):
        """Test that empty queries are rejected."""
        with pytest.raises(ValueError):
            search_engine.search("")

        with pytest.raises(ValueError):
            search_engine.search("   ")

        logger.info("✓ Empty query validation works")


class TestGenerationPipeline:
    """Test the answer generation pipeline."""

    @pytest.fixture
    def generator(self):
        """Create generator fixture."""
        return GeminiAnswerGenerator()

    @pytest.fixture
    def search_engine(self):
        """Create search engine fixture."""
        return VectorSearchEngine(top_k=3, similarity_threshold=0.70)

    def test_prompt_building(self, generator):
        """Test RAG prompt construction."""
        query = "What is physical AI?"
        context = "[Source 1]\nPhysical AI refers to AI in robots."

        prompt = generator.build_prompt(query, context)

        assert generator.SYSTEM_PROMPT in prompt
        assert query in prompt
        assert context in prompt
        logger.info("✓ Prompt building works")

    def test_prompt_with_history(self, generator):
        """Test prompt building with conversation history."""
        query = "Tell me more"
        context = "Some context"
        history = [
            {"role": "user", "content": "What is AI?"},
            {"role": "assistant", "content": "AI is artificial intelligence."},
        ]

        prompt = generator.build_prompt(query, context, history)

        assert "CONVERSATION HISTORY" in prompt
        assert "What is AI?" in prompt
        logger.info("✓ Prompt with history works")

    def test_response_validation(self, generator):
        """Test response validation logic."""
        # Valid response with citations
        answer1 = "This is true [Source 1] and verified [Source 2]."
        context1 = "Some context"
        validation1 = generator.validate_response(answer1, context1)
        assert validation1["has_citations"] is True

        # Response without citations
        answer2 = "This has no citations."
        validation2 = generator.validate_response(answer2, context1)
        assert validation2["has_citations"] is False

        # Response with hallucination phrases
        answer3 = "Based on my knowledge, this is true."
        validation3 = generator.validate_response(answer3, context1)
        assert validation3["is_valid"] is False

        logger.info("✓ Response validation works")

    def test_generate_chat_response(self, generator, search_engine):
        """Test full chat response generation."""
        query = "What are robots?"
        results = search_engine.search(query, top_k=2)

        response = generator.generate_chat_response(query, results)

        assert response.query == query
        assert response.answer is not None
        assert isinstance(response.sources, list)
        assert response.model is not None
        logger.info("✓ Chat response generation works")


class TestSessionManagement:
    """Test session management."""

    @pytest.fixture
    def session_manager(self):
        """Create session manager fixture."""
        return SessionManager(session_timeout_minutes=30, max_history_turns=10)

    def test_create_session(self, session_manager):
        """Test session creation."""
        session_id = session_manager.create_session()
        assert session_id is not None
        assert len(session_id) == 36  # UUID length with hyphens
        logger.info("✓ Session creation works")

    def test_get_session(self, session_manager):
        """Test session retrieval."""
        session_id = session_manager.create_session()
        session = session_manager.get_session(session_id)

        assert session is not None
        assert session.session_id == session_id
        logger.info("✓ Session retrieval works")

    def test_update_session(self, session_manager):
        """Test session updates."""
        session_id = session_manager.create_session()

        updated = session_manager.update_session(
            session_id,
            user_message="Hello",
            assistant_message="Hi there!",
        )

        assert updated is True

        session = session_manager.get_session(session_id)
        assert len(session.conversation_history) == 2
        logger.info("✓ Session update works")

    def test_conversation_history(self, session_manager):
        """Test conversation history tracking."""
        session_id = session_manager.create_session()

        # Add multiple turns
        for i in range(5):
            session_manager.update_session(
                session_id,
                user_message=f"Question {i}",
                assistant_message=f"Answer {i}",
            )

        history = session_manager.get_conversation_history(session_id)
        assert len(history) == 10  # 5 turns * 2 messages
        logger.info("✓ Conversation history tracking works")

    def test_session_stats(self, session_manager):
        """Test session statistics."""
        session_id = session_manager.create_session()
        session_manager.update_session(
            session_id, user_message="Test", assistant_message="Response"
        )

        stats = session_manager.get_session_stats(session_id)

        assert stats is not None
        assert stats["num_turns"] == 1
        assert stats["is_expired"] is False
        logger.info("✓ Session statistics work")

    def test_delete_session(self, session_manager):
        """Test session deletion."""
        session_id = session_manager.create_session()
        deleted = session_manager.delete_session(session_id)

        assert deleted is True

        session = session_manager.get_session(session_id)
        assert session is None
        logger.info("✓ Session deletion works")


class TestFullRAGWorkflow:
    """Test the complete end-to-end RAG workflow."""

    @pytest.fixture
    def components(self):
        """Create all components."""
        return {
            "search_engine": VectorSearchEngine(top_k=5, similarity_threshold=0.70),
            "generator": GeminiAnswerGenerator(),
            "session_manager": SessionManager(session_timeout_minutes=30),
        }

    def test_single_turn_rag(self, components):
        """Test single-turn RAG workflow."""
        search_engine = components["search_engine"]
        generator = components["generator"]

        query = "What is a humanoid robot?"

        # Step 1: Retrieve
        results = search_engine.search(query, top_k=3)

        # Step 2: Generate
        response = generator.generate_chat_response(query, results)

        # Assertions
        assert response.query == query
        assert response.answer is not None
        assert len(response.answer) > 0

        logger.info("✓ Single-turn RAG workflow works")

    def test_multi_turn_rag(self, components):
        """Test multi-turn RAG workflow with history."""
        search_engine = components["search_engine"]
        generator = components["generator"]
        session_manager = components["session_manager"]

        session_id = session_manager.create_session()

        queries = [
            "What are sensors in robotics?",
            "How do they work with actuators?",
        ]

        for query in queries:
            # Retrieve
            results = search_engine.search(query, top_k=3)

            # Get history
            history = session_manager.get_conversation_history(session_id)

            # Generate
            response = generator.generate_chat_response(query, results, history)

            # Update session
            session_manager.update_session(
                session_id, user_message=query, assistant_message=response.answer
            )

            assert response.answer is not None

        # Check history
        final_history = session_manager.get_conversation_history(session_id)
        assert len(final_history) == 4  # 2 turns * 2 messages

        logger.info("✓ Multi-turn RAG workflow works")

    def test_rag_with_no_results(self, components):
        """Test RAG workflow when no results are found."""
        generator = components["generator"]

        query = "Completely unrelated query about weather"
        results = []  # Empty results

        response = generator.generate_chat_response(query, results)

        # Should still generate a response indicating lack of context
        assert response.answer is not None
        assert response.num_sources == 0

        logger.info("✓ RAG with no results works")


def run_all_tests():
    """Run all integration tests."""
    print("\n" + "=" * 70)
    print("  RUNNING INTEGRATION TESTS")
    print("=" * 70 + "\n")

    # Setup logging
    settings = get_settings()
    setup_logging(log_level="INFO", log_file="logs/integration_test.log")

    # Run pytest
    pytest_args = [
        __file__,
        "-v",
        "--tb=short",
        "--color=yes",
    ]

    exit_code = pytest.main(pytest_args)

    print("\n" + "=" * 70)
    if exit_code == 0:
        print("  [OK] ALL INTEGRATION TESTS PASSED")
    else:
        print("  [X] SOME TESTS FAILED")
    print("=" * 70 + "\n")

    return exit_code


if __name__ == "__main__":
    sys.exit(run_all_tests())
