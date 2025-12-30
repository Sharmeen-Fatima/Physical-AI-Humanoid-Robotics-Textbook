"""
Text chunker for splitting content into overlapping semantic segments.

Uses Cohere tokenizer for accurate token counting and creates chunks
with configurable size and overlap.
"""

import hashlib
from typing import List
from uuid import uuid4

import cohere

from src.config import get_settings
from src.storage.schemas import TextChunk, BookPage
from src.utils.logger import get_logger

logger = get_logger(__name__)


class TextChunker:
    """Chunker for splitting text into overlapping token-based segments."""

    def __init__(self, chunk_size: int = 800, chunk_overlap: int = 200):
        """
        Initialize text chunker.

        Args:
            chunk_size: Number of tokens per chunk (default: 800)
            chunk_overlap: Number of overlapping tokens (default: 200)
        """
        settings = get_settings()
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.cohere_client = cohere.Client(settings.cohere_api_key)
        self.embedding_model = settings.embedding_model

        logger.info(
            f"Initialized chunker: size={chunk_size}, overlap={chunk_overlap}"
        )

    def tokenize(self, text: str) -> List[int]:
        """
        Tokenize text using Cohere tokenizer.

        Args:
            text: Text to tokenize

        Returns:
            List[int]: List of token IDs
        """
        response = self.cohere_client.tokenize(
            text=text,
            model=self.embedding_model
        )
        return response.tokens

    def detokenize(self, tokens: List[int]) -> str:
        """
        Convert tokens back to text using Cohere detokenizer.

        Args:
            tokens: List of token IDs

        Returns:
            str: Detokenized text
        """
        response = self.cohere_client.detokenize(
            tokens=tokens,
            model=self.embedding_model
        )
        return response.text

    def generate_chunk_id(self, chunk_text: str) -> str:
        """
        Generate deterministic UUID from chunk content hash.

        Args:
            chunk_text: Text content of the chunk

        Returns:
            str: UUID string derived from SHA-256 hash
        """
        hash_digest = hashlib.sha256(chunk_text.encode("utf-8")).hexdigest()
        # Use first 32 characters of hash to create UUID
        uuid_str = f"{hash_digest[:8]}-{hash_digest[8:12]}-{hash_digest[12:16]}-{hash_digest[16:20]}-{hash_digest[20:32]}"
        return uuid_str

    def chunk_text(self, book_page: BookPage) -> List[TextChunk]:
        """
        Split book page text into overlapping chunks.

        Args:
            book_page: BookPage object with cleaned text

        Returns:
            List[TextChunk]: List of text chunks with metadata
        """
        text = book_page.cleaned_text
        url = book_page.url
        section_title = book_page.section_title

        logger.info(f"Chunking text from {url} ({len(text)} chars)")

        # Tokenize full text
        try:
            tokens = self.tokenize(text)
            logger.debug(f"Tokenized to {len(tokens)} tokens")
        except Exception as e:
            logger.error(f"Tokenization failed for {url}: {e}")
            raise

        chunks: List[TextChunk] = []
        chunk_index = 0
        start_idx = 0

        while start_idx < len(tokens):
            # Calculate end index for this chunk
            end_idx = min(start_idx + self.chunk_size, len(tokens))

            # Extract chunk tokens
            chunk_tokens = tokens[start_idx:end_idx]

            # Detokenize to get chunk text
            try:
                chunk_text = self.detokenize(chunk_tokens)
            except Exception as e:
                logger.error(f"Detokenization failed: {e}")
                raise

            # Calculate character positions (approximate)
            char_start = len(self.detokenize(tokens[:start_idx])) if start_idx > 0 else 0
            char_end = len(self.detokenize(tokens[:end_idx]))

            # Calculate overlap info
            overlap_start = self.chunk_overlap if chunk_index > 0 else 0
            is_last_chunk = end_idx >= len(tokens)
            overlap_end = 0 if is_last_chunk else self.chunk_overlap

            # Generate deterministic chunk ID
            chunk_id = self.generate_chunk_id(chunk_text)

            # Create TextChunk object
            chunk = TextChunk(
                chunk_id=chunk_id,
                text=chunk_text,
                chunk_index=chunk_index,
                parent_url=url,
                section_title=section_title,
                char_start=char_start,
                char_end=char_end,
                overlap_start=overlap_start,
                overlap_end=overlap_end,
            )

            # Validate chunk
            if self.validate_chunk(chunk):
                chunks.append(chunk)
                chunk_index += 1
            else:
                logger.warning(f"Skipping invalid chunk at index {chunk_index}")

            # Move start index forward (with overlap)
            start_idx += (self.chunk_size - self.chunk_overlap)

        logger.info(f"Created {len(chunks)} chunks from {url}")
        return chunks

    def validate_chunk(self, chunk: TextChunk) -> bool:
        """
        Validate chunk meets quality criteria.

        Args:
            chunk: TextChunk to validate

        Returns:
            bool: True if chunk is valid, False otherwise
        """
        # Check minimum length (100 characters)
        if len(chunk.text) < 100:
            logger.warning(f"Chunk too short: {len(chunk.text)} chars")
            return False

        # Check token count (rough estimate: should be close to chunk_size)
        estimated_tokens = len(chunk.text.split())  # Rough estimate
        if estimated_tokens > self.chunk_size * 2:  # Allow 2x for safety
            logger.warning(f"Chunk potentially too long: ~{estimated_tokens} tokens")
            return False

        return True
