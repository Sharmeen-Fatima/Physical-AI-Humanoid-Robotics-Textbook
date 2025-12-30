"""
Settings and configuration management using Pydantic.

This module handles environment variable loading and validation
with fail-fast behavior if required credentials are missing.
"""

from functools import lru_cache
from typing import Optional

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # Cohere API Configuration
    cohere_api_key: str = Field(..., description="Cohere API key for embeddings")

    # Gemini API Configuration
    gemini_api_key: str = Field(..., description="Google Gemini API key for LLM")

    # Qdrant Configuration
    qdrant_url: str = Field(..., description="Qdrant Cloud cluster URL")
    qdrant_api_key: str = Field(..., description="Qdrant API key")
    qdrant_collection_name: str = Field(
        default="physical_ai_book", description="Qdrant collection name"
    )

    # Book Configuration
    book_sitemap_url: str = Field(
        default="https://physical-ai-humanoid-robotics-textb-pied.vercel.app/sitemap.xml",
        description="URL of the book sitemap.xml",
    )
    book_base_url: str = Field(
        default="https://physical-ai-humanoid-robotics-textb-pied.vercel.app",
        description="Base URL of the book website",
    )

    # RAG Configuration
    chunk_size: int = Field(default=800, description="Number of tokens per chunk", ge=100, le=2000)
    chunk_overlap: int = Field(default=200, description="Number of overlapping tokens", ge=0, le=500)
    top_k: int = Field(default=5, description="Number of top chunks to retrieve", ge=1, le=20)
    similarity_threshold: float = Field(
        default=0.70, description="Minimum similarity score for relevance", ge=0.0, le=1.0
    )

    # Embedding Configuration
    embedding_model: str = Field(default="embed-english-v3.0", description="Cohere embedding model")
    embedding_batch_size: int = Field(default=96, description="Batch size for embedding generation", ge=1, le=96)

    # Generation Configuration
    gemini_model: str = Field(default="gemini-1.5-pro", description="Gemini model for answer generation")
    gemini_temperature: float = Field(default=0.2, description="Gemini temperature parameter", ge=0.0, le=2.0)
    gemini_top_p: float = Field(default=0.8, description="Gemini top-p parameter", ge=0.0, le=1.0)
    gemini_max_output_tokens: int = Field(default=1024, description="Max tokens in generated answer", ge=1, le=8192)

    # Logging
    log_level: str = Field(default="INFO", description="Logging level")

    # Session Management
    session_expiration_minutes: int = Field(
        default=30, description="Session expiration time in minutes", ge=1, le=1440
    )
    max_context_messages: int = Field(
        default=5, description="Maximum number of messages to keep in session context", ge=1, le=20
    )

    @field_validator("log_level")
    @classmethod
    def validate_log_level(cls, v: str) -> str:
        """Validate log level is one of the standard Python logging levels."""
        valid_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        if v.upper() not in valid_levels:
            raise ValueError(f"log_level must be one of {valid_levels}")
        return v.upper()

    @field_validator("qdrant_url")
    @classmethod
    def validate_qdrant_url(cls, v: str) -> str:
        """Validate Qdrant URL format."""
        if not v.startswith("https://"):
            raise ValueError("qdrant_url must start with https://")
        return v

    @field_validator("book_sitemap_url", "book_base_url")
    @classmethod
    def validate_urls(cls, v: str) -> str:
        """Validate URLs start with http:// or https://."""
        if not v.startswith(("http://", "https://")):
            raise ValueError(f"URL must start with http:// or https://: {v}")
        return v


@lru_cache()
def get_settings() -> Settings:
    """
    Get cached settings instance.

    Returns:
        Settings: Validated settings loaded from environment variables

    Raises:
        ValidationError: If required environment variables are missing or invalid
    """
    return Settings()


if __name__ == "__main__":
    """Validate configuration when run directly."""
    try:
        settings = get_settings()
        print("✓ All required environment variables are set")
        print("✓ Configuration validation passed")
        print(f"\n✓ Cohere API key: {'*' * 8}{settings.cohere_api_key[-4:]}")
        print(f"✓ Gemini API key: {'*' * 8}{settings.gemini_api_key[-4:]}")
        print(f"✓ Qdrant URL: {settings.qdrant_url}")
        print(f"✓ Collection name: {settings.qdrant_collection_name}")
        print(f"✓ Chunk size: {settings.chunk_size} tokens")
        print(f"✓ Chunk overlap: {settings.chunk_overlap} tokens")
        print(f"✓ Top-k retrieval: {settings.top_k}")
        print(f"✓ Similarity threshold: {settings.similarity_threshold}")
        print("\n✓ Configuration loaded successfully")
    except Exception as e:
        print(f"✗ Configuration validation failed: {e}")
        exit(1)
