#!/usr/bin/env python3
"""
Book Ingestion Runner - Fixed import paths

Ingests the Physical AI & Humanoid Robotics book into Qdrant vector database.

Workflow:
1. Parse sitemap to get all book URLs
2. Scrape and clean HTML content
3. Chunk text with 800 tokens, 200 overlap
4. Generate embeddings with Cohere
5. Store in Qdrant with metadata

Usage:
    python run_ingestion.py
"""

import sys
from pathlib import Path

# Add backend to path for src imports
backend_path = Path(__file__).parent
sys.path.insert(0, str(backend_path))

from src.config import get_settings
from src.ingestion import SitemapParser, WebScraper, TextChunker
from src.embedding import CohereEmbedder
from src.storage import qdrant_client
from src.utils.logger import setup_logging, get_logger

logger = get_logger(__name__)


def main():
    """Main ingestion pipeline."""
    # Setup
    settings = get_settings()
    setup_logging(log_level=settings.log_level, log_file="logs/ingestion.log")

    logger.info("Starting book ingestion pipeline")
    print("\n" + "="*70)
    print("  BOOK INGESTION PIPELINE")
    print("="*70)
    print(f"\nTarget: {settings.book_sitemap_url}")
    print(f"Collection: {settings.qdrant_collection_name}")
    print(f"Chunk size: {settings.chunk_size} tokens, overlap: {settings.chunk_overlap}\n")

    # Initialize components
    print("[1/6] Initializing components...")
    sitemap_parser = SitemapParser(
        sitemap_url=settings.book_sitemap_url,
        base_url=settings.book_base_url
    )
    scraper = WebScraper()
    chunker = TextChunker(
        chunk_size=settings.chunk_size,
        chunk_overlap=settings.chunk_overlap
    )
    embedder = CohereEmbedder()
    qdrant = qdrant_client.QdrantClient()
    print("  [OK] All components initialized\n")

    # Step 1: Parse sitemap
    print("[2/6] Parsing sitemap...")
    try:
        urls = sitemap_parser.get_book_urls()
        print(f"  [OK] Found {len(urls)} book URLs in sitemap")
        if urls:
            print(f"  Sample URLs:")
            for url in urls[:3]:
                print(f"    - {url}")
        print()
        logger.info(f"Found {len(urls)} URLs")
    except Exception as e:
        print(f"  [FAIL] Sitemap parsing failed: {e}")
        logger.error(f"Sitemap parsing failed: {e}", exc_info=True)
        return 1

    # Step 2: Scrape pages
    print("[3/6] Scraping and cleaning pages...")
    pages = []
    failed_urls = []

    for i, url in enumerate(urls, 1):
        try:
            print(f"  [{i}/{len(urls)}] Scraping: {url}")
            page = scraper.scrape_page(url)
            pages.append(page)
            logger.info(f"Scraped {url}: {len(page.cleaned_text)} chars")
        except Exception as e:
            failed_urls.append(url)
            logger.error(f"Failed to scrape {url}: {e}")
            print(f"      [WARN] Failed: {e}")

    print(f"\n  [OK] Scraped {len(pages)}/{len(urls)} pages ({len(failed_urls)} failed)\n")

    if not pages:
        print("[FAIL] No pages scraped successfully")
        return 1

    # Step 3: Chunk text
    print("[4/6] Chunking text...")
    all_chunks = []

    for i, page in enumerate(pages, 1):
        print(f"  [{i}/{len(pages)}] Chunking: {page.section_title or page.url[:50]}...")
        try:
            # chunk_text expects a BookPage object
            chunks = chunker.chunk_text(page)
            all_chunks.extend(chunks)
            logger.info(f"Created {len(chunks)} chunks from {page.url}")
        except Exception as e:
            logger.error(f"Failed to chunk {page.url}: {e}")
            print(f"      [WARN] Chunking failed: {e}")

    print(f"\n  [OK] Created {len(all_chunks)} total chunks\n")

    if not all_chunks:
        print("[FAIL] No chunks created")
        return 1

    # Step 4: Generate embeddings
    print("[5/6] Generating embeddings...")
    print(f"  Processing {len(all_chunks)} chunks in batches of {embedder.batch_size}...")

    try:
        chunk_texts = [chunk.text for chunk in all_chunks]
        embeddings = embedder.generate_embeddings_batch(
            texts=chunk_texts,
            input_type="search_document"
        )
        print(f"  [OK] Generated {len(embeddings)} embeddings (dim={len(embeddings[0])})\n")
        logger.info(f"Generated {len(embeddings)} embeddings")
    except Exception as e:
        print(f"  [FAIL] Embedding generation failed: {e}")
        logger.error(f"Embedding failed: {e}", exc_info=True)
        return 1

    # Step 5: Store in Qdrant
    print("[6/6] Storing in Qdrant...")
    print(f"  Upserting {len(all_chunks)} points to collection '{qdrant.collection_name}'...")

    try:
        # Upsert chunks with embeddings (method expects chunks and embeddings separately)
        count = qdrant.upsert_chunks(all_chunks, embeddings)

        print(f"  [OK] Successfully stored {count} chunks\n")
        logger.info(f"Stored {count} chunks in Qdrant")

    except Exception as e:
        print(f"  [FAIL] Qdrant storage failed: {e}")
        logger.error(f"Qdrant upsert failed: {e}", exc_info=True)
        return 1

    # Summary
    print("="*70)
    print("  INGESTION COMPLETE")
    print("="*70)
    print(f"\nStatistics:")
    print(f"  URLs processed: {len(pages)}/{len(urls)}")
    print(f"  Chunks created: {len(all_chunks)}")
    print(f"  Embeddings generated: {len(embeddings)}")
    print(f"  Qdrant collection: {qdrant.collection_name}")
    print(f"  Vector dimension: {len(embeddings[0]) if embeddings else 0}")

    if failed_urls:
        print(f"\n  Failed URLs ({len(failed_urls)}):")
        for url in failed_urls[:5]:
            print(f"    - {url}")
        if len(failed_urls) > 5:
            print(f"    ... and {len(failed_urls) - 5} more")

    print("\n" + "="*70)
    print("\nNext: Run 'python test_chatbot_simple.py' to test retrieval\n")

    logger.info("Ingestion pipeline completed successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())
