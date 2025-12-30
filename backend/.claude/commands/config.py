"""
Central configuration file for RAG Chatbot
Loads all API keys and service URLs securely from environment variables.
"""

import os
from dotenv import load_dotenv

# Load variables from .env file
load_dotenv()

# =========================
# QDRANT CONFIGURATION
# =========================
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not QDRANT_URL or not QDRANT_API_KEY:
    raise ValueError("Qdrant configuration is missing in .env file")

# =========================
# COHERE CONFIGURATION
# =========================
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

if not COHERE_API_KEY:
    raise ValueError("Cohere API key is missing in .env file")

# =========================
# GEMINI CONFIGURATION
# =========================
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

if not GEMINI_API_KEY:
    raise ValueError("Gemini API key is missing in .env file")

# =========================
# RAG SETTINGS
# =========================
CHUNK_SIZE = 700
CHUNK_OVERLAP = 120
TOP_K_RESULTS = 5

# =========================
# PROJECT METADATA
# =========================
PROJECT_NAME = "Physical AI & Humanoid Robotics RAG Chatbot"
DATA_SOURCE = "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/"
SITEMAP_URL = "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/sitemap.xml"
