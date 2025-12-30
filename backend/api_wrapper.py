#!/usr/bin/env python3
"""
API Wrapper for RAG Chatbot

This script provides a JSON-based CLI interface for the RAG chatbot system.
It accepts JSON input from stdin and outputs JSON responses to stdout, making
it easy to integrate with Node.js/Express APIs.

Usage:
    echo '{"query": "What is a humanoid robot?", "sessionId": "abc123"}' | python api_wrapper.py

Input JSON Schema:
    {
        "action": "query" | "create_session" | "get_session" | "delete_session",
        "query": str (required for action=query),
        "sessionId": str (optional for action=query),
        "topK": int (optional, default=5),
        "threshold": float (optional, default=0.70)
    }

Output JSON Schema:
    {
        "success": bool,
        "sessionId": str,
        "query": str,
        "answer": str,
        "sources": [...],
        "metadata": {...},
        "error": str (only if success=false)
    }
"""

import sys
import json
from pathlib import Path
from typing import Dict, Any, Optional

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src.config import get_settings
from src.retrieval import VectorSearchEngine
from src.generation import GeminiAnswerGenerator
from src.chatbot.session_manager import SessionManager
from src.utils.logger import setup_logging, get_logger

# Global instances (singleton pattern for performance)
_search_engine: Optional[VectorSearchEngine] = None
_generator: Optional[GeminiAnswerGenerator] = None
_session_manager: Optional[SessionManager] = None


def get_search_engine() -> VectorSearchEngine:
    """Get or create VectorSearchEngine instance."""
    global _search_engine
    if _search_engine is None:
        _search_engine = VectorSearchEngine(top_k=5, similarity_threshold=0.70)
    return _search_engine


def get_generator() -> GeminiAnswerGenerator:
    """Get or create GeminiAnswerGenerator instance."""
    global _generator
    if _generator is None:
        _generator = GeminiAnswerGenerator()
    return _generator


def get_session_manager() -> SessionManager:
    """Get or create SessionManager instance."""
    global _session_manager
    if _session_manager is None:
        _session_manager = SessionManager(session_timeout_minutes=30)
    return _session_manager


def handle_query(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Handle a user query request.

    Args:
        data: Input data containing query, sessionId, topK, threshold

    Returns:
        Response dictionary with answer and sources
    """
    logger = get_logger(__name__)

    # Validate input
    query = data.get("query", "").strip()
    if not query:
        return {
            "success": False,
            "error": "Query is required and must be a non-empty string",
            "code": "INVALID_QUERY"
        }

    if len(query) > 500:
        return {
            "success": False,
            "error": "Query exceeds maximum length of 500 characters",
            "code": "QUERY_TOO_LONG"
        }

    # Get or create session
    session_id = data.get("sessionId")
    session_manager = get_session_manager()

    if not session_id:
        session_id = session_manager.create_session()
        logger.info(f"Created new session: {session_id}")
    else:
        # Verify session exists
        session = session_manager.get_session(session_id)
        if not session:
            session_id = session_manager.create_session()
            logger.info(f"Session not found, created new: {session_id}")

    try:
        # Step 1: Retrieve relevant chunks
        search_engine = get_search_engine()
        top_k = data.get("topK", 5)
        threshold = data.get("threshold", 0.70)

        logger.info(f"Searching for query: '{query[:50]}...' (top_k={top_k}, threshold={threshold})")
        results = search_engine.search(query, top_k=top_k, similarity_threshold=threshold)
        logger.info(f"Retrieved {len(results)} results")

        # Step 2: Get conversation history
        history = session_manager.get_conversation_history(session_id)

        # Step 3: Generate answer
        generator = get_generator()
        logger.info("Generating answer with Gemini...")
        response = generator.generate_chat_response(query, results, history)

        # Step 4: Update session
        session_manager.update_session(
            session_id,
            user_message=query,
            assistant_message=response.answer
        )

        # Format response
        return {
            "success": True,
            "sessionId": session_id,
            "query": response.query,
            "answer": response.answer,
            "sources": response.sources,
            "metadata": {
                "numSources": response.num_sources,
                "isGrounded": response.is_grounded,
                "hasCitations": response.has_citations,
                "model": response.model,
                "temperature": response.temperature
            }
        }

    except Exception as e:
        logger.error(f"Query processing failed: {e}", exc_info=True)
        return {
            "success": False,
            "error": f"Failed to process query: {str(e)}",
            "code": "RAG_PIPELINE_ERROR"
        }


def handle_create_session(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Handle session creation request.

    Returns:
        Response dictionary with new sessionId
    """
    logger = get_logger(__name__)

    try:
        session_manager = get_session_manager()
        session_id = session_manager.create_session()

        session = session_manager.get_session(session_id)

        logger.info(f"Created session: {session_id}")

        return {
            "success": True,
            "sessionId": session_id,
            "createdAt": session.created_at if session else None
        }

    except Exception as e:
        logger.error(f"Session creation failed: {e}", exc_info=True)
        return {
            "success": False,
            "error": f"Failed to create session: {str(e)}",
            "code": "SESSION_CREATE_ERROR"
        }


def handle_get_session(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Handle get session request.

    Args:
        data: Input data containing sessionId

    Returns:
        Response dictionary with session history
    """
    logger = get_logger(__name__)

    session_id = data.get("sessionId")
    if not session_id:
        return {
            "success": False,
            "error": "Session ID is required",
            "code": "MISSING_SESSION_ID"
        }

    try:
        session_manager = get_session_manager()
        session = session_manager.get_session(session_id)

        if not session:
            return {
                "success": False,
                "error": "Session not found or expired",
                "code": "SESSION_NOT_FOUND"
            }

        history = session_manager.get_conversation_history(session_id)
        stats = session_manager.get_session_stats(session_id)

        logger.info(f"Retrieved session: {session_id}")

        return {
            "success": True,
            "sessionId": session_id,
            "messages": history,
            "createdAt": session.created_at,
            "lastActivity": session.last_activity,
            "numTurns": stats["num_turns"] if stats else 0
        }

    except Exception as e:
        logger.error(f"Session retrieval failed: {e}", exc_info=True)
        return {
            "success": False,
            "error": f"Failed to retrieve session: {str(e)}",
            "code": "SESSION_GET_ERROR"
        }


def handle_delete_session(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Handle session deletion request.

    Args:
        data: Input data containing sessionId

    Returns:
        Response dictionary confirming deletion
    """
    logger = get_logger(__name__)

    session_id = data.get("sessionId")
    if not session_id:
        return {
            "success": False,
            "error": "Session ID is required",
            "code": "MISSING_SESSION_ID"
        }

    try:
        session_manager = get_session_manager()
        deleted = session_manager.delete_session(session_id)

        if not deleted:
            logger.warning(f"Session not found for deletion: {session_id}")
        else:
            logger.info(f"Deleted session: {session_id}")

        return {
            "success": True,
            "sessionId": session_id,
            "deleted": deleted
        }

    except Exception as e:
        logger.error(f"Session deletion failed: {e}", exc_info=True)
        return {
            "success": False,
            "error": f"Failed to delete session: {str(e)}",
            "code": "SESSION_DELETE_ERROR"
        }


def main() -> int:
    """
    Main entry point.

    Reads JSON from stdin, processes request, outputs JSON to stdout.
    """
    # Setup logging (WARNING level to avoid polluting stdout)
    settings = get_settings()
    setup_logging(log_level="WARNING", log_file="logs/api_wrapper.log")
    logger = get_logger(__name__)

    try:
        # Read JSON from stdin
        input_data = sys.stdin.read()

        if not input_data:
            response = {
                "success": False,
                "error": "No input data provided",
                "code": "NO_INPUT"
            }
            print(json.dumps(response), flush=True)
            return 1

        # Parse JSON
        try:
            data = json.loads(input_data)
        except json.JSONDecodeError as e:
            response = {
                "success": False,
                "error": f"Invalid JSON: {str(e)}",
                "code": "INVALID_JSON"
            }
            print(json.dumps(response), flush=True)
            return 1

        # Route to appropriate handler
        action = data.get("action", "query")

        if action == "query":
            response = handle_query(data)
        elif action == "create_session":
            response = handle_create_session(data)
        elif action == "get_session":
            response = handle_get_session(data)
        elif action == "delete_session":
            response = handle_delete_session(data)
        else:
            response = {
                "success": False,
                "error": f"Unknown action: {action}",
                "code": "UNKNOWN_ACTION"
            }

        # Output JSON response
        print(json.dumps(response), flush=True)

        return 0 if response.get("success") else 1

    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        response = {
            "success": False,
            "error": f"Internal error: {str(e)}",
            "code": "INTERNAL_ERROR"
        }
        print(json.dumps(response), flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
