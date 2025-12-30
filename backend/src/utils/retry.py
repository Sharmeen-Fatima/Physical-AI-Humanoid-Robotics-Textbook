"""
Retry logic with exponential backoff for handling transient failures.

Provides a decorator for retrying operations that may fail due to
network issues, API rate limits, or other temporary problems.
"""

import random
import time
from functools import wraps
from typing import Callable, Type, TypeVar, Tuple

from .logger import get_logger

logger = get_logger(__name__)

T = TypeVar("T")


def retry_with_exponential_backoff(
    max_retries: int = 5,
    initial_delay: float = 1.0,
    backoff_multiplier: float = 2.0,
    max_delay: float = 60.0,
    jitter: float = 0.2,
    exceptions: Tuple[Type[Exception], ...] = (Exception,),
) -> Callable[[Callable[..., T]], Callable[..., T]]:
    """
    Decorator for retry logic with exponential backoff.

    Args:
        max_retries: Maximum number of retry attempts (default: 5)
        initial_delay: Initial delay in seconds (default: 1.0)
        backoff_multiplier: Multiplier for exponential backoff (default: 2.0)
        max_delay: Maximum delay cap in seconds (default: 60.0)
        jitter: Random jitter factor ±percentage (default: 0.2 = ±20%)
        exceptions: Tuple of exception types to catch and retry

    Returns:
        Decorated function with retry logic

    Example:
        @retry_with_exponential_backoff(max_retries=3)
        def call_api():
            return requests.get("https://api.example.com")
    """

    def decorator(func: Callable[..., T]) -> Callable[..., T]:
        @wraps(func)
        def wrapper(*args, **kwargs) -> T:
            delay = initial_delay
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e

                    if attempt == max_retries:
                        logger.error(
                            f"Max retries ({max_retries}) exceeded for {func.__name__}. "
                            f"Last error: {e}"
                        )
                        raise last_exception

                    # Calculate jittered delay
                    jittered_delay = delay * (1 + random.uniform(-jitter, jitter))
                    jittered_delay = min(jittered_delay, max_delay)

                    logger.warning(
                        f"Retry {attempt + 1}/{max_retries} for {func.__name__} "
                        f"after {jittered_delay:.2f}s due to: {e}"
                    )

                    time.sleep(jittered_delay)
                    delay *= backoff_multiplier

            # This should never be reached, but type checker needs it
            raise last_exception

        return wrapper

    return decorator
