/**
 * Rate Limiting Middleware
 *
 * Implements rate limiting to prevent API abuse and ensure fair usage.
 * Limits requests per IP address.
 */

import rateLimit from 'express-rate-limit';
import logger from '../utils/logger';

/**
 * Rate limiter for chat API endpoints
 *
 * Configuration:
 * - 60 requests per minute per IP
 * - Responds with 429 Too Many Requests when exceeded
 * - Includes Retry-After header
 */
export const chatRateLimiter = rateLimit({
  windowMs: 60 * 1000, // 1 minute window
  max: 60, // 60 requests per window
  message: {
    error: 'Too Many Requests',
    message: 'You have exceeded the rate limit of 60 requests per minute. Please try again later.',
    code: 'RATE_LIMIT_EXCEEDED',
  },
  standardHeaders: true, // Return rate limit info in `RateLimit-*` headers
  legacyHeaders: false, // Disable `X-RateLimit-*` headers
  handler: (req, res) => {
    logger.warn(`Rate limit exceeded for IP: ${req.ip}`);
    res.status(429).json({
      error: 'Too Many Requests',
      message: 'You have exceeded the rate limit of 60 requests per minute. Please try again later.',
      code: 'RATE_LIMIT_EXCEEDED',
      retryAfter: Math.ceil(60 - ((Date.now() - (req as any).rateLimit?.resetTime?.getTime() || 0) / 1000)),
    });
  },
  skip: (req) => {
    // Skip rate limiting for health check endpoints
    return req.path === '/health' || req.path === '/api/health';
  },
  keyGenerator: (req) => {
    // Use IP address as the key for rate limiting
    return req.ip || 'unknown';
  },
});

/**
 * Stricter rate limiter for session creation
 *
 * Configuration:
 * - 10 session creations per minute per IP
 * - Prevents abuse of session creation endpoint
 */
export const sessionCreationRateLimiter = rateLimit({
  windowMs: 60 * 1000, // 1 minute window
  max: 10, // 10 session creations per minute
  message: {
    error: 'Too Many Requests',
    message: 'You have exceeded the session creation limit of 10 per minute. Please try again later.',
    code: 'SESSION_CREATION_RATE_LIMIT_EXCEEDED',
  },
  standardHeaders: true,
  legacyHeaders: false,
  handler: (req, res) => {
    logger.warn(`Session creation rate limit exceeded for IP: ${req.ip}`);
    res.status(429).json({
      error: 'Too Many Requests',
      message: 'You have exceeded the session creation limit of 10 per minute. Please try again later.',
      code: 'SESSION_CREATION_RATE_LIMIT_EXCEEDED',
    });
  },
});

export default chatRateLimiter;
