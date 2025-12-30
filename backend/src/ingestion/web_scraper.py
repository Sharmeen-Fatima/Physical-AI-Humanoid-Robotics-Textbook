"""
Web scraper for extracting clean text from book pages.

Scrapes HTML content and removes navigation, headers, footers, and UI elements.
"""

import hashlib
from datetime import datetime
from typing import Tuple

import requests
from bs4 import BeautifulSoup

from src.storage.schemas import BookPage
from src.utils.logger import get_logger
from src.utils.retry import retry_with_exponential_backoff

logger = get_logger(__name__)


class WebScraper:
    """Scraper for extracting and cleaning book page content."""

    def __init__(self, timeout: int = 30):
        """
        Initialize web scraper.

        Args:
            timeout: Request timeout in seconds (default: 30)
        """
        self.timeout = timeout
        self.session = requests.Session()
        self.session.headers.update({
            "User-Agent": "RAG-Chatbot-BookScraper/1.0"
        })

    @retry_with_exponential_backoff(max_retries=3, exceptions=(requests.RequestException,))
    def fetch_page(self, url: str) -> str:
        """
        Fetch HTML content from a URL.

        Args:
            url: URL of the page to fetch

        Returns:
            str: Raw HTML content

        Raises:
            requests.RequestException: If fetching fails after retries
        """
        logger.info(f"Fetching page: {url}")
        response = self.session.get(url, timeout=self.timeout)
        response.raise_for_status()
        logger.debug(f"Fetched {len(response.content)} bytes from {url}")
        return response.text

    def clean_html(self, html_content: str) -> Tuple[str, str]:
        """
        Extract clean text and section title from HTML.

        Args:
            html_content: Raw HTML content

        Returns:
            Tuple[str, str]: (cleaned_text, section_title)
                - cleaned_text: Main content without navigation/UI elements
                - section_title: First h1/h2 heading found
        """
        soup = BeautifulSoup(html_content, "lxml")

        # Remove unwanted elements
        for tag in soup(["nav", "header", "footer", "aside", "script", "style", "noscript", "iframe"]):
            tag.decompose()

        # Extract section title (first h1 or h2)
        section_title = ""
        heading = soup.find(["h1", "h2"])
        if heading:
            section_title = heading.get_text(strip=True)
            logger.debug(f"Extracted section title: {section_title}")

        # Extract main content
        main_content = soup.find("main") or soup.find("article") or soup.find("body")

        if main_content:
            # Get text with preserved structure
            text = main_content.get_text(separator="\n", strip=True)

            # Clean up excessive whitespace
            lines = [line.strip() for line in text.splitlines() if line.strip()]
            cleaned_text = "\n".join(lines)

            logger.debug(f"Extracted {len(cleaned_text)} characters of clean text")
        else:
            cleaned_text = ""
            logger.warning("No main content found in HTML")

        return cleaned_text, section_title

    def validate_content(self, text: str, min_length: int = 100) -> bool:
        """
        Validate that extracted content meets quality criteria.

        Args:
            text: Cleaned text content
            min_length: Minimum required length in characters

        Returns:
            bool: True if content is valid, False otherwise
        """
        if not text or len(text) < min_length:
            logger.warning(f"Content too short: {len(text)} chars (min: {min_length})")
            return False
        return True

    def generate_content_hash(self, text: str) -> str:
        """
        Generate SHA-256 hash of content for change detection.

        Args:
            text: Content to hash

        Returns:
            str: SHA-256 hash (hexadecimal)
        """
        return hashlib.sha256(text.encode("utf-8")).hexdigest()

    def scrape_page(self, url: str) -> BookPage:
        """
        Scrape a page and return a BookPage object.

        Args:
            url: URL of the page to scrape

        Returns:
            BookPage: Structured page data with cleaned content

        Raises:
            ValueError: If content validation fails
            requests.RequestException: If fetching fails
        """
        # Fetch HTML
        html_content = self.fetch_page(url)

        # Clean and extract content
        cleaned_text, section_title = self.clean_html(html_content)

        # Validate content
        if not self.validate_content(cleaned_text):
            raise ValueError(f"Invalid content extracted from {url}")

        # Generate content hash
        content_hash = self.generate_content_hash(cleaned_text)

        # Create BookPage object
        book_page = BookPage(
            url=url,
            cleaned_text=cleaned_text,
            section_title=section_title if section_title else None,
            scrape_timestamp=datetime.utcnow(),
            content_hash=content_hash,
        )

        logger.info(
            f"Successfully scraped {url}: "
            f"{len(cleaned_text)} chars, "
            f"section='{section_title or 'N/A'}'"
        )

        return book_page

    def close(self):
        """Close the requests session."""
        self.session.close()
        logger.debug("Closed scraper session")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
