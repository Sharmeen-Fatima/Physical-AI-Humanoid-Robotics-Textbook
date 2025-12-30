"""
Cohere embedding generation client.

Generates vector embeddings using Cohere's embed-english-v3.0 model
with batch processing and retry logic.
"""

from typing import List

import cohere

from src.config import get_settings
from src.utils.logger import get_logger
from src.utils.retry import retry_with_exponential_backoff

logger = get_logger(__name__)


class CohereEmbedder:
    """Client for generating embeddings using Cohere API."""

    def __init__(self):
        """Initialize Cohere embedder with API key from settings."""
        settings = get_settings()
        self.client = cohere.Client(settings.cohere_api_key)
        self.model = settings.embedding_model
        self.batch_size = settings.embedding_batch_size

        logger.info(
            f"Initialized Cohere embedder: model={self.model}, "
            f"batch_size={self.batch_size}"
        )

    @retry_with_exponential_backoff(
        max_retries=5,
        exceptions=(Exception,),  # Catch all Cohere API exceptions
    )
    def generate_embeddings(
        self, texts: List[str], input_type: str = "search_document"
    ) -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of text strings to embed
            input_type: Type of input - "search_document" for indexing,
                       "search_query" for queries (default: "search_document")

        Returns:
            List[List[float]]: List of embedding vectors (1024 dimensions each)

        Raises:
            CohereError: If API call fails after retries
        """
        if not texts:
            logger.warning("Empty text list provided for embedding")
            return []

        logger.info(f"Generating embeddings for {len(texts)} texts (type={input_type})")

        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type=input_type,
                embedding_types=["float"],
            )

            embeddings = response.embeddings.float

            logger.info(
                f"Generated {len(embeddings)} embeddings, "
                f"dimension={len(embeddings[0]) if embeddings else 0}"
            )

            return embeddings

        except Exception as e:
            logger.error(f"Failed to generate embeddings: {e}")
            raise

    def generate_embeddings_batch(
        self, texts: List[str], input_type: str = "search_document"
    ) -> List[List[float]]:
        """
        Generate embeddings in batches for large lists.

        Args:
            texts: List of text strings to embed
            input_type: Type of input - "search_document" or "search_query"

        Returns:
            List[List[float]]: List of embedding vectors for all texts

        Raises:
            CohereError: If any batch fails after retries
        """
        if not texts:
            return []

        all_embeddings = []
        total_batches = (len(texts) + self.batch_size - 1) // self.batch_size

        logger.info(
            f"Processing {len(texts)} texts in {total_batches} batches "
            f"of size {self.batch_size}"
        )

        for batch_idx in range(0, len(texts), self.batch_size):
            batch_texts = texts[batch_idx : batch_idx + self.batch_size]
            batch_num = (batch_idx // self.batch_size) + 1

            logger.info(
                f"Processing batch {batch_num}/{total_batches} "
                f"({len(batch_texts)} texts)"
            )

            batch_embeddings = self.generate_embeddings(batch_texts, input_type)
            all_embeddings.extend(batch_embeddings)

        logger.info(f"Completed batch processing: {len(all_embeddings)} embeddings generated")
        return all_embeddings

    def generate_query_embedding(self, query_text: str) -> List[float]:
        """
        Generate embedding for a single query.

        Args:
            query_text: Query text to embed

        Returns:
            List[float]: Embedding vector (1024 dimensions)

        Raises:
            CohereError: If API call fails after retries
        """
        logger.debug(f"Generating query embedding for: {query_text[:100]}...")

        embeddings = self.generate_embeddings(
            texts=[query_text],
            input_type="search_query"
        )

        return embeddings[0] if embeddings else []
