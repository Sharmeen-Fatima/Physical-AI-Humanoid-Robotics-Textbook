"""
Session manager for chatbot conversations.

Manages conversation sessions with UUID identification,
30-minute expiration, and conversation history tracking.
"""

from datetime import datetime, timedelta
from typing import Dict, List, Optional
from uuid import uuid4
import json

from src.storage.schemas import SessionData
from src.utils.logger import get_logger

logger = get_logger(__name__)


class SessionManager:
    """
    Manager for chatbot conversation sessions.

    Features:
    - UUID-based session identification
    - 30-minute session expiration
    - Conversation history tracking (last 10 turns per session)
    - Automatic cleanup of expired sessions
    """

    def __init__(self, session_timeout_minutes: int = 30, max_history_turns: int = 10):
        """
        Initialize session manager.

        Args:
            session_timeout_minutes: Session expiration time (default: 30)
            max_history_turns: Maximum conversation turns to keep (default: 10)
        """
        self.session_timeout = timedelta(minutes=session_timeout_minutes)
        self.max_history_turns = max_history_turns
        self.sessions: Dict[str, SessionData] = {}

        logger.info(
            f"Initialized SessionManager: timeout={session_timeout_minutes}min, "
            f"max_history={max_history_turns}"
        )

    def create_session(self) -> str:
        """
        Create a new session.

        Returns:
            str: Session UUID
        """
        session_id = str(uuid4())
        now = datetime.utcnow()

        session_data = SessionData(
            session_id=session_id,
            created_at=now.isoformat(),
            last_activity=now.isoformat(),
            conversation_history=[],
            metadata={},
        )

        self.sessions[session_id] = session_data
        logger.info(f"Created session: {session_id}")

        return session_id

    def get_session(self, session_id: str) -> Optional[SessionData]:
        """
        Get session data by ID.

        Args:
            session_id: Session UUID

        Returns:
            Optional[SessionData]: Session data or None if not found/expired
        """
        if session_id not in self.sessions:
            logger.warning(f"Session not found: {session_id}")
            return None

        session = self.sessions[session_id]

        # Check expiration
        if self._is_expired(session):
            logger.info(f"Session expired: {session_id}")
            self.delete_session(session_id)
            return None

        return session

    def update_session(
        self,
        session_id: str,
        user_message: str,
        assistant_message: str,
        metadata: Optional[Dict] = None,
    ) -> bool:
        """
        Update session with new conversation turn.

        Args:
            session_id: Session UUID
            user_message: User's message
            assistant_message: Assistant's response
            metadata: Optional metadata to merge

        Returns:
            bool: True if updated, False if session not found
        """
        session = self.get_session(session_id)
        if not session:
            return False

        # Add conversation turn
        session.conversation_history.append({"role": "user", "content": user_message})
        session.conversation_history.append(
            {"role": "assistant", "content": assistant_message}
        )

        # Trim history to max turns
        if len(session.conversation_history) > self.max_history_turns * 2:
            session.conversation_history = session.conversation_history[
                -self.max_history_turns * 2 :
            ]

        # Update timestamp
        session.last_activity = datetime.utcnow().isoformat()

        # Merge metadata
        if metadata:
            session.metadata.update(metadata)

        self.sessions[session_id] = session
        logger.debug(
            f"Updated session {session_id}: "
            f"{len(session.conversation_history)} messages in history"
        )

        return True

    def delete_session(self, session_id: str) -> bool:
        """
        Delete a session.

        Args:
            session_id: Session UUID

        Returns:
            bool: True if deleted, False if not found
        """
        if session_id in self.sessions:
            del self.sessions[session_id]
            logger.info(f"Deleted session: {session_id}")
            return True

        logger.warning(f"Cannot delete - session not found: {session_id}")
        return False

    def cleanup_expired_sessions(self) -> int:
        """
        Remove all expired sessions.

        Returns:
            int: Number of sessions cleaned up
        """
        expired_ids = [
            sid for sid, session in self.sessions.items() if self._is_expired(session)
        ]

        for session_id in expired_ids:
            self.delete_session(session_id)

        if expired_ids:
            logger.info(f"Cleaned up {len(expired_ids)} expired sessions")

        return len(expired_ids)

    def get_conversation_history(self, session_id: str) -> List[Dict[str, str]]:
        """
        Get conversation history for a session.

        Args:
            session_id: Session UUID

        Returns:
            List[Dict[str, str]]: Conversation history or empty list
        """
        session = self.get_session(session_id)
        if not session:
            return []

        return session.conversation_history

    def get_session_stats(self, session_id: str) -> Optional[Dict]:
        """
        Get statistics for a session.

        Args:
            session_id: Session UUID

        Returns:
            Optional[Dict]: Session statistics or None
        """
        session = self.get_session(session_id)
        if not session:
            return None

        created_at = datetime.fromisoformat(session.created_at)
        last_activity = datetime.fromisoformat(session.last_activity)
        duration = last_activity - created_at

        return {
            "session_id": session.session_id,
            "created_at": session.created_at,
            "last_activity": session.last_activity,
            "duration_seconds": int(duration.total_seconds()),
            "num_turns": len(session.conversation_history) // 2,
            "is_expired": self._is_expired(session),
        }

    def get_all_stats(self) -> Dict:
        """
        Get statistics for all sessions.

        Returns:
            Dict: Aggregate session statistics
        """
        total_sessions = len(self.sessions)
        active_sessions = sum(
            1 for s in self.sessions.values() if not self._is_expired(s)
        )
        expired_sessions = total_sessions - active_sessions

        total_turns = sum(
            len(s.conversation_history) // 2 for s in self.sessions.values()
        )

        return {
            "total_sessions": total_sessions,
            "active_sessions": active_sessions,
            "expired_sessions": expired_sessions,
            "total_conversation_turns": total_turns,
            "avg_turns_per_session": (
                total_turns / total_sessions if total_sessions > 0 else 0
            ),
        }

    def _is_expired(self, session: SessionData) -> bool:
        """
        Check if a session is expired.

        Args:
            session: SessionData object

        Returns:
            bool: True if expired
        """
        last_activity = datetime.fromisoformat(session.last_activity)
        now = datetime.utcnow()
        return (now - last_activity) > self.session_timeout

    def export_session(self, session_id: str, filepath: str) -> bool:
        """
        Export session to JSON file.

        Args:
            session_id: Session UUID
            filepath: Output file path

        Returns:
            bool: True if exported successfully
        """
        session = self.get_session(session_id)
        if not session:
            return False

        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(session.model_dump(), f, indent=2, ensure_ascii=False)

            logger.info(f"Exported session {session_id} to {filepath}")
            return True

        except Exception as e:
            logger.error(f"Failed to export session: {e}")
            return False

    def import_session(self, filepath: str) -> Optional[str]:
        """
        Import session from JSON file.

        Args:
            filepath: Input file path

        Returns:
            Optional[str]: Session ID if imported successfully
        """
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)

            session = SessionData(**data)
            self.sessions[session.session_id] = session

            logger.info(f"Imported session {session.session_id} from {filepath}")
            return session.session_id

        except Exception as e:
            logger.error(f"Failed to import session: {e}")
            return None
