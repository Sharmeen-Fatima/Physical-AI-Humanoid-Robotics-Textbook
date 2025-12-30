#!/usr/bin/env python3
"""
RAG Chatbot - Interactive CLI Interface

Main entry point for the Physical AI & Humanoid Robotics chatbot.

Usage:
    python scripts/chatbot.py
"""

import sys
from pathlib import Path

# Add backend to path for src imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.chatbot.chat_interface import main

if __name__ == "__main__":
    sys.exit(main())
