"""
Qdrant client wrapper for vector storage and retrieval.

Handles collection initialization, idempotent upsert, and vector similarity search.
"""

from datetime import datetime
from typing import List, Dict, Any

from qdrant_client import QdrantClient as QdrantBaseClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    SearchParams,
    Filter,
    HnswConfigDiff,
)

from src.config import get_settings
from src.storage.schemas import TextChunk, ChunkMetadata
from src.utils.logger import get_logger

logger = get_logger(__name__)


class QdrantClient:
    """Wrapper for Qdrant vector database operations."""

    def __init__(self):
        """Initialize Qdrant client with settings from environment."""
        settings = get_settings()
        self.client = QdrantBaseClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection_name
        self.vector_size = 1024  # Cohere embed-english-v3.0 dimension

        logger.info(
            f"Initialized Qdrant client: url={settings.qdrant_url}, "
            f"collection={self.collection_name}"
        )

    def create_collection(self, recreate: bool = False) -> None:
        """
        Create Qdrant collection with vector configuration.

        Args:
            recreate: If True, delete existing collection and recreate

        Raises:
            Exception: If collection creation fails
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            exists = any(c.name == self.collection_name for c in collections)

            if exists and recreate:
                logger.warning(f"Deleting existing collection: {self.collection_name}")
                self.client.delete_collection(self.collection_name)
                exists = False

            if not exists:
                logger.info(f"Creating collection: {self.collection_name}")

                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=Distance.COSINE,
                    ),
                    hnsw_config=HnswConfigDiff(
                        m=16,  # Number of connections per node
                        ef_construct=100,  # Build-time accuracy
                    ),
                )

                logger.info(
                    f"Collection created: {self.collection_name} "
                    f"(size={self.vector_size}, distance=COSINE, HNSW=16/100)"
                )
            else:
                logger.info(f"Collection already exists: {self.collection_name}")

        except Exception as e:
            logger.error(f"Failed to create collection: {e}")
            raise

    def upsert_chunks(
        self, chunks: List[TextChunk], embeddings: List[List[float]]
    ) -> int:
        """
        Upsert chunks with embeddings to Qdrant (idempotent).

        Uses chunk_id as point ID for idempotent upserts.

        Args:
            chunks: List of TextChunk objects with metadata
            embeddings: List of embedding vectors corresponding to chunks

        Returns:
            int: Number of chunks successfully upserted

        Raises:
            ValueError: If chunks and embeddings lists have different lengths
            Exception: If upsert operation fails
        """
        if len(chunks) != len(embeddings):
            raise ValueError(
                f"Chunks ({len(chunks)}) and embeddings ({len(embeddings)}) "
                "must have the same length"
            )

        if not chunks:
            logger.warning("No chunks to upsert")
            return 0

        logger.info(f"Upserting {len(chunks)} chunks to Qdrant")

        # Create points for upsert
        points = []
        for chunk, embedding in zip(chunks, embeddings):
            # Create metadata payload
            metadata = ChunkMetadata(
                text=chunk.text,
                url=chunk.parent_url,
                section=chunk.section_title,
                chunk_index=chunk.chunk_index,
                ingestion_timestamp=datetime.utcnow().isoformat(),
                book_version=None,  # TODO: Add versioning support
            )

            # Create point with chunk_id as deterministic ID
            point = PointStruct(
                id=chunk.chunk_id,  # UUID string from content hash
                vector=embedding,
                payload=metadata.model_dump(),
            )
            points.append(point)

        # Batch upsert
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )

            logger.info(f"Successfully upserted {len(points)} chunks")
            return len(points)

        except Exception as e:
            logger.error(f"Failed to upsert chunks: {e}")
            raise

    def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        similarity_threshold: float = 0.70,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar chunks using vector similarity.

        Args:
            query_vector: Query embedding vector (1024 dimensions)
            top_k: Number of top results to return (default: 5)
            similarity_threshold: Minimum similarity score (default: 0.70)

        Returns:
            List[Dict[str, Any]]: List of search results with metadata and scores

        Raises:
            Exception: If search operation fails
        """
        logger.info(
            f"Searching Qdrant: top_k={top_k}, threshold={similarity_threshold}"
        )

        try:
            search_result = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                score_threshold=similarity_threshold,
                search_params=SearchParams(
                    hnsw_ef=128,  # Runtime accuracy (higher = more accurate, slower)
                ),
            )

            results = []
            for hit in search_result.points:
                result = {
                    "chunk_id": hit.id,
                    "similarity_score": hit.score,
                    "metadata": hit.payload,
                }
                results.append(result)

            logger.info(f"Found {len(results)} results above threshold")
            return results

        except Exception as e:
            logger.error(f"Search failed: {e}")
            raise

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.

        Returns:
            Dict[str, Any]: Collection information (count, config, etc.)
        """
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "name": info.config.params.vectors.size if hasattr(info.config.params, 'vectors') else None,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status,
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            return {}

    def collection_exists(self) -> bool:
        """
        Check if collection exists.

        Returns:
            bool: True if collection exists, False otherwise
        """
        try:
            collections = self.client.get_collections().collections
            return any(c.name == self.collection_name for c in collections)
        except Exception as e:
            logger.error(f"Failed to check collection existence: {e}")
            return False
