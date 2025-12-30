"""
Sitemap parser for extracting book URLs.

Fetches and parses sitemap.xml to extract all valid book page URLs.
"""

import xml.etree.ElementTree as ET
from typing import List
from urllib.parse import urlparse

import requests

from src.utils.logger import get_logger
from src.utils.retry import retry_with_exponential_backoff

logger = get_logger(__name__)


class SitemapParser:
    """Parser for extracting URLs from sitemap.xml."""

    def __init__(self, sitemap_url: str, base_url: str):
        """
        Initialize sitemap parser.

        Args:
            sitemap_url: URL of the sitemap.xml file
            base_url: Base URL of the book website for filtering
        """
        self.sitemap_url = sitemap_url
        self.base_url = base_url
        self._base_domain = urlparse(base_url).netloc

    @retry_with_exponential_backoff(max_retries=3)
    def fetch_sitemap(self) -> str:
        """
        Fetch sitemap XML content.

        Returns:
            str: Raw XML content of the sitemap

        Raises:
            requests.RequestException: If fetching fails after retries
        """
        logger.info(f"Fetching sitemap from {self.sitemap_url}")
        response = requests.get(self.sitemap_url, timeout=30)
        response.raise_for_status()
        logger.info(f"Successfully fetched sitemap ({len(response.content)} bytes)")
        return response.text

    def parse_sitemap(self, xml_content: str) -> List[str]:
        """
        Parse sitemap XML and extract URLs.

        Args:
            xml_content: Raw XML content of the sitemap

        Returns:
            List[str]: List of URLs found in the sitemap

        Raises:
            ET.ParseError: If XML is malformed
        """
        logger.info("Parsing sitemap XML")

        try:
            root = ET.fromstring(xml_content)

            # Handle namespace in sitemap URLs
            namespaces = {"ns": "http://www.sitemaps.org/schemas/sitemap/0.9"}
            urls = []

            # Extract all <loc> elements
            for url_element in root.findall(".//ns:loc", namespaces):
                url = url_element.text
                if url:
                    urls.append(url.strip())

            # Fallback: try without namespace if none found
            if not urls:
                for url_element in root.findall(".//loc"):
                    url = url_element.text
                    if url:
                        urls.append(url.strip())

            logger.info(f"Extracted {len(urls)} URLs from sitemap")
            return urls

        except ET.ParseError as e:
            logger.error(f"Failed to parse sitemap XML: {e}")
            raise

    def filter_book_urls(self, urls: List[str]) -> List[str]:
        """
        Filter URLs to include only book-related pages.

        Handles placeholder domains (e.g., example.com) by replacing with base_url.

        Args:
            urls: List of all URLs from sitemap

        Returns:
            List[str]: Filtered list of book page URLs
        """
        logger.info(f"Filtering {len(urls)} URLs for book domain")
        logger.info(f"Base domain: {self._base_domain}")

        valid_urls = []
        placeholder_domains = ["example.com", "your-docusaurus-site.example.com", "localhost"]

        for url in urls:
            parsed = urlparse(url)

            # Replace placeholder domains with actual base_url
            if any(placeholder in parsed.netloc for placeholder in placeholder_domains):
                logger.info(f"Replacing placeholder domain in: {url}")
                url = url.replace(parsed.scheme + "://" + parsed.netloc, self.base_url.rstrip('/'))
                parsed = urlparse(url)
                logger.info(f"  -> Corrected to: {url}")

            # Check if URL belongs to the book domain
            if parsed.netloc == self._base_domain:
                # Exclude non-content pages (optional: customize these patterns)
                exclude_patterns = ["/api/", "/admin/", "/auth/", "/static/"]
                if not any(pattern in url for pattern in exclude_patterns):
                    valid_urls.append(url)
                    logger.debug(f"  -> INCLUDED: {url}")
                else:
                    logger.debug(f"  -> EXCLUDED (non-content pattern): {url}")
            else:
                logger.debug(f"  -> EXCLUDED (domain mismatch): {url}")

        logger.info(f"Filtered to {len(valid_urls)} valid book URLs")
        if valid_urls:
            logger.info(f"Sample valid URLs: {valid_urls[:3]}")
        return valid_urls

    def get_book_urls(self) -> List[str]:
        """
        Fetch, parse, and filter sitemap to get all book URLs.

        Returns:
            List[str]: List of valid book page URLs

        Raises:
            Exception: If fetching or parsing fails
        """
        try:
            # Fetch sitemap
            xml_content = self.fetch_sitemap()

            # Parse URLs
            all_urls = self.parse_sitemap(xml_content)

            # Filter to book pages only
            book_urls = self.filter_book_urls(all_urls)

            if not book_urls:
                logger.warning("No book URLs found in sitemap!")

            return book_urls

        except Exception as e:
            logger.error(f"Failed to get book URLs: {e}")
            raise
