"""
Vector search engine for semantic retrieval.

Combines query embedding generation and vector similarity search
to retrieve the most relevant chunks from Qdrant.
"""

from typing import List, Dict, Any, Optional

from src.embedding import CohereEmbedder
from src.storage import qdrant_client
from src.storage.schemas import Query, RetrievalResult
from src.utils.logger import get_logger

logger = get_logger(__name__)


class VectorSearchEngine:
    """
    Semantic search engine using Cohere embeddings and Qdrant vector search.

    Workflow:
    1. Generate query embedding using Cohere (input_type="search_query")
    2. Perform vector similarity search in Qdrant
    3. Filter by similarity threshold
    4. Rank and format results with metadata
    """

    def __init__(
        self,
        top_k: int = 5,
        similarity_threshold: float = 0.70,
    ):
        """
        Initialize vector search engine.

        Args:
            top_k: Number of top results to retrieve (default: 5)
            similarity_threshold: Minimum similarity score (0-1, default: 0.70)
        """
        self.top_k = top_k
        self.similarity_threshold = similarity_threshold

        # Initialize components
        self.embedder = CohereEmbedder()
        self.qdrant = qdrant_client.QdrantClient()

        logger.info(
            f"Initialized VectorSearchEngine: top_k={top_k}, "
            f"threshold={similarity_threshold}"
        )

    def search(
        self,
        query_text: str,
        top_k: Optional[int] = None,
        similarity_threshold: Optional[float] = None,
    ) -> List[RetrievalResult]:
        """
        Perform semantic search for a query.

        Args:
            query_text: The user's query text
            top_k: Override default top_k (optional)
            similarity_threshold: Override default threshold (optional)

        Returns:
            List[RetrievalResult]: Ranked list of relevant chunks with metadata

        Raises:
            ValueError: If query_text is empty
            Exception: If embedding or search fails
        """
        if not query_text or not query_text.strip():
            raise ValueError("Query text cannot be empty")

        query_text = query_text.strip()
        k = top_k if top_k is not None else self.top_k
        threshold = (
            similarity_threshold
            if similarity_threshold is not None
            else self.similarity_threshold
        )

        logger.info(
            f"Searching for query: '{query_text[:100]}...' "
            f"(top_k={k}, threshold={threshold})"
        )

        # Step 1: Generate query embedding
        try:
            query_embedding = self.embedder.generate_query_embedding(query_text)
            logger.debug(
                f"Generated query embedding: dimension={len(query_embedding)}"
            )
        except Exception as e:
            logger.error(f"Failed to generate query embedding: {e}")
            raise

        # Step 2: Perform vector similarity search
        try:
            search_results = self.qdrant.search(
                query_vector=query_embedding,
                top_k=k,
                similarity_threshold=threshold,
            )
        except Exception as e:
            logger.error(f"Vector search failed: {e}")
            raise

        # Step 3: Format results as RetrievalResult objects
        retrieval_results = []
        for idx, result in enumerate(search_results):
            metadata = result["metadata"]
            retrieval_result = RetrievalResult(
                chunk_id=str(result["chunk_id"]),
                text=metadata.get("text", ""),
                url=metadata.get("url", ""),
                section=metadata.get("section", "Unknown Section"),
                chunk_index=metadata.get("chunk_index", 0),
                similarity_score=result["similarity_score"],
                rank=idx + 1,
            )
            retrieval_results.append(retrieval_result)

        logger.info(
            f"Retrieved {len(retrieval_results)} results "
            f"(scores: {[f'{r.similarity_score:.3f}' for r in retrieval_results]})"
        )

        return retrieval_results

    def format_context(self, results: List[RetrievalResult]) -> str:
        """
        Format retrieval results into a context string for LLM.

        Args:
            results: List of RetrievalResult objects

        Returns:
            str: Formatted context with source citations
        """
        if not results:
            return "No relevant information found."

        context_parts = []
        for result in results:
            context_parts.append(
                f"[Source {result.rank}: {result.section} - {result.url}]\n"
                f"{result.text}\n"
            )

        context = "\n".join(context_parts)
        logger.debug(f"Formatted context: {len(context)} characters")
        return context

    def search_with_context(
        self,
        query_text: str,
        top_k: Optional[int] = None,
        similarity_threshold: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Perform search and return both results and formatted context.

        Args:
            query_text: The user's query text
            top_k: Override default top_k (optional)
            similarity_threshold: Override default threshold (optional)

        Returns:
            Dict with 'results' (List[RetrievalResult]) and 'context' (str)
        """
        results = self.search(
            query_text=query_text,
            top_k=top_k,
            similarity_threshold=similarity_threshold,
        )

        context = self.format_context(results)

        return {
            "query": query_text,
            "results": results,
            "context": context,
            "num_results": len(results),
        }

    def get_stats(self) -> Dict[str, Any]:
        """
        Get search engine statistics.

        Returns:
            Dict with configuration and collection stats
        """
        collection_info = self.qdrant.get_collection_info()

        return {
            "config": {
                "top_k": self.top_k,
                "similarity_threshold": self.similarity_threshold,
                "embedding_model": self.embedder.model,
            },
            "collection": {
                "name": self.qdrant.collection_name,
                "points_count": collection_info.get("points_count", 0),
                "vectors_count": collection_info.get("vectors_count", 0),
            },
        }
