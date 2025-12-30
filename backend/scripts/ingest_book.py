#!/usr/bin/env python3
"""
Book Ingestion Script

Orchestrates the complete ingestion pipeline:
1. Parse sitemap to get all book URLs
2. Scrape and clean HTML content from each page
3. Split content into overlapping chunks
4. Generate embeddings using Cohere
5. Store chunks and embeddings in Qdrant

This script is idempotent - re-running will not create duplicates.
"""

import sys
import time
from pathlib import Path
from typing import List

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from config import get_settings
from ingestion import SitemapParser, WebScraper, TextChunker
from embedding import CohereEmbedder
from storage import qdrant_client
from storage.schemas import BookPage, TextChunk
from utils.logger import setup_logging, get_logger

logger = get_logger(__name__)


class BookIngestionPipeline:
    """Orchestrates the complete book ingestion pipeline."""

    def __init__(self):
        """Initialize pipeline components."""
        settings = get_settings()

        logger.info("Initializing ingestion pipeline")

        # Initialize components
        self.sitemap_parser = SitemapParser(
            sitemap_url=settings.book_sitemap_url,
            base_url=settings.book_base_url,
        )
        self.scraper = WebScraper()
        self.chunker = TextChunker(
            chunk_size=settings.chunk_size,
            chunk_overlap=settings.chunk_overlap,
        )
        self.embedder = CohereEmbedder()
        self.qdrant = qdrant_client.QdrantClient()

        # Stats tracking
        self.stats = {
            "urls_found": 0,
            "pages_scraped": 0,
            "pages_failed": 0,
            "chunks_created": 0,
            "embeddings_generated": 0,
            "chunks_stored": 0,
            "start_time": time.time(),
        }

    def initialize_qdrant(self) -> None:
        """Initialize Qdrant collection."""
        logger.info("Initializing Qdrant collection")

        if not self.qdrant.collection_exists():
            self.qdrant.create_collection()
            logger.info("Qdrant collection created")
        else:
            logger.info("Qdrant collection already exists")
            info = self.qdrant.get_collection_info()
            logger.info(
                f"Current collection stats: "
                f"{info.get('points_count', 'unknown')} points"
            )

    def fetch_urls(self) -> List[str]:
        """Fetch and filter book URLs from sitemap."""
        logger.info("=" * 60)
        logger.info("Step 1: Fetching book URLs from sitemap")
        logger.info("=" * 60)

        urls = self.sitemap_parser.get_book_urls()
        self.stats["urls_found"] = len(urls)

        logger.info(f"Found {len(urls)} book URLs")
        return urls

    def scrape_pages(self, urls: List[str]) -> List[BookPage]:
        """Scrape and clean content from all URLs."""
        logger.info("=" * 60)
        logger.info("Step 2: Scraping and cleaning pages")
        logger.info("=" * 60)

        pages = []
        total = len(urls)

        for idx, url in enumerate(urls, 1):
            try:
                logger.info(f"[{idx}/{total}] Scraping {url}")
                page = self.scraper.scrape_page(url)
                pages.append(page)
                self.stats["pages_scraped"] += 1

            except Exception as e:
                logger.error(f"Failed to scrape {url}: {e}")
                self.stats["pages_failed"] += 1
                continue

        logger.info(
            f"Scraped {len(pages)}/{total} pages successfully "
            f"({self.stats['pages_failed']} failed)"
        )
        return pages

    def chunk_pages(self, pages: List[BookPage]) -> List[TextChunk]:
        """Chunk all pages into overlapping text segments."""
        logger.info("=" * 60)
        logger.info("Step 3: Chunking pages")
        logger.info("=" * 60)

        all_chunks = []

        for idx, page in enumerate(pages, 1):
            try:
                logger.info(
                    f"[{idx}/{len(pages)}] Chunking {page.url} "
                    f"({len(page.cleaned_text)} chars)"
                )
                chunks = self.chunker.chunk_text(page)
                all_chunks.extend(chunks)
                self.stats["chunks_created"] += len(chunks)

            except Exception as e:
                logger.error(f"Failed to chunk {page.url}: {e}")
                continue

        logger.info(f"Created {len(all_chunks)} chunks from {len(pages)} pages")
        return all_chunks

    def generate_embeddings(self, chunks: List[TextChunk]) -> List[List[float]]:
        """Generate embeddings for all chunks."""
        logger.info("=" * 60)
        logger.info("Step 4: Generating embeddings")
        logger.info("=" * 60)

        # Extract chunk texts
        chunk_texts = [chunk.text for chunk in chunks]

        logger.info(f"Generating embeddings for {len(chunk_texts)} chunks")

        # Generate in batches
        embeddings = self.embedder.generate_embeddings_batch(
            texts=chunk_texts,
            input_type="search_document"
        )

        self.stats["embeddings_generated"] = len(embeddings)
        logger.info(f"Generated {len(embeddings)} embeddings")

        return embeddings

    def store_in_qdrant(
        self, chunks: List[TextChunk], embeddings: List[List[float]]
    ) -> None:
        """Store chunks and embeddings in Qdrant."""
        logger.info("=" * 60)
        logger.info("Step 5: Storing in Qdrant")
        logger.info("=" * 60)

        logger.info(f"Upserting {len(chunks)} chunks to Qdrant")

        chunks_stored = self.qdrant.upsert_chunks(chunks, embeddings)
        self.stats["chunks_stored"] = chunks_stored

        logger.info(f"Successfully stored {chunks_stored} chunks")

    def print_summary(self) -> None:
        """Print ingestion summary."""
        elapsed = time.time() - self.stats["start_time"]
        minutes = int(elapsed // 60)
        seconds = int(elapsed % 60)

        logger.info("=" * 60)
        logger.info("INGESTION COMPLETE")
        logger.info("=" * 60)
        logger.info(f"URLs found:           {self.stats['urls_found']}")
        logger.info(f"Pages scraped:        {self.stats['pages_scraped']}")
        logger.info(f"Pages failed:         {self.stats['pages_failed']}")
        logger.info(f"Chunks created:       {self.stats['chunks_created']}")
        logger.info(f"Embeddings generated: {self.stats['embeddings_generated']}")
        logger.info(f"Chunks stored:        {self.stats['chunks_stored']}")
        logger.info(f"Total time:           {minutes}m {seconds}s")
        logger.info("=" * 60)

        # Get final collection stats
        info = self.qdrant.get_collection_info()
        logger.info(f"Collection: {info.get('points_count', 'unknown')} total points")

    def run(self) -> int:
        """
        Run the complete ingestion pipeline.

        Returns:
            int: 0 if successful, 1 if failed
        """
        try:
            logger.info("Starting book ingestion pipeline")

            # Initialize Qdrant
            self.initialize_qdrant()

            # Step 1: Fetch URLs
            urls = self.fetch_urls()
            if not urls:
                logger.error("No URLs found in sitemap!")
                return 1

            # Step 2: Scrape pages
            pages = self.scrape_pages(urls)
            if not pages:
                logger.error("No pages scraped successfully!")
                return 1

            # Step 3: Chunk pages
            chunks = self.chunk_pages(pages)
            if not chunks:
                logger.error("No chunks created!")
                return 1

            # Step 4: Generate embeddings
            embeddings = self.generate_embeddings(chunks)
            if not embeddings:
                logger.error("No embeddings generated!")
                return 1

            # Step 5: Store in Qdrant
            self.store_in_qdrant(chunks, embeddings)

            # Print summary
            self.print_summary()

            return 0

        except KeyboardInterrupt:
            logger.warning("Ingestion interrupted by user")
            return 1

        except Exception as e:
            logger.error(f"Ingestion failed: {e}", exc_info=True)
            return 1

        finally:
            self.scraper.close()


def main() -> int:
    """Main entry point."""
    # Setup logging
    settings = get_settings()
    setup_logging(
        log_level=settings.log_level,
        log_file="logs/ingestion.log"
    )

    # Run pipeline
    pipeline = BookIngestionPipeline()
    return pipeline.run()


if __name__ == "__main__":
    sys.exit(main())
