/**
 * HTTP Request/Response Logging Middleware
 *
 * Logs all HTTP requests and responses with detailed metadata.
 * Includes request ID for distributed tracing.
 */

import { Request, Response, NextFunction } from 'express';
import { v4 as uuidv4 } from 'uuid';
import logger from '../utils/logger';

/**
 * Request logging middleware
 *
 * Features:
 * - Logs HTTP method, path, status code, response time
 * - Generates unique request ID for tracing
 * - Logs request body for POST/PUT/PATCH (excluding sensitive data)
 * - Logs response status and duration
 */
export const requestLogger = (req: Request, res: Response, next: NextFunction) => {
  // Generate unique request ID
  const requestId = uuidv4();
  (req as any).id = requestId;

  // Record request start time
  const startTime = Date.now();

  // Log incoming request
  logger.http('Incoming request', {
    requestId,
    method: req.method,
    path: req.path,
    query: req.query,
    ip: req.ip,
    userAgent: req.get('user-agent'),
  });

  // Log request body for mutation methods (excluding sensitive fields)
  if (['POST', 'PUT', 'PATCH'].includes(req.method)) {
    const sanitizedBody = sanitizeBody(req.body);
    logger.debug('Request body', {
      requestId,
      body: sanitizedBody,
    });
  }

  // Override res.json to log response
  const originalJson = res.json.bind(res);
  res.json = function (body: any) {
    // Calculate response time
    const duration = Date.now() - startTime;

    // Log response
    logger.http('Outgoing response', {
      requestId,
      method: req.method,
      path: req.path,
      statusCode: res.statusCode,
      duration: `${duration}ms`,
    });

    // Log response body in debug mode
    if (process.env['NODE_ENV'] === 'development') {
      logger.debug('Response body', {
        requestId,
        statusCode: res.statusCode,
        body: sanitizeBody(body),
      });
    }

    // Call original json method
    return originalJson(body);
  };

  // Override res.send to log non-JSON responses
  const originalSend = res.send.bind(res);
  res.send = function (body: any) {
    // Calculate response time
    const duration = Date.now() - startTime;

    // Log response
    logger.http('Outgoing response', {
      requestId,
      method: req.method,
      path: req.path,
      statusCode: res.statusCode,
      duration: `${duration}ms`,
    });

    // Call original send method
    return originalSend(body);
  };

  // Continue to next middleware
  next();
};

/**
 * Sanitize request/response body
 *
 * Removes sensitive fields from logs (passwords, tokens, API keys)
 */
function sanitizeBody(body: any): any {
  if (!body || typeof body !== 'object') {
    return body;
  }

  const sensitiveFields = [
    'password',
    'token',
    'apiKey',
    'api_key',
    'secret',
    'authorization',
    'auth',
    'sessionId', // Partially redact session IDs
  ];

  const sanitized = { ...body };

  for (const field of sensitiveFields) {
    if (field in sanitized) {
      if (field === 'sessionId' && typeof sanitized[field] === 'string') {
        // Show first 8 chars of session ID for debugging
        sanitized[field] = `${sanitized[field].substring(0, 8)}...`;
      } else {
        sanitized[field] = '[REDACTED]';
      }
    }
  }

  return sanitized;
}

/**
 * Performance monitoring middleware
 *
 * Logs warnings for slow requests (>2 seconds)
 */
export const performanceMonitor = (req: Request, res: Response, next: NextFunction) => {
  const startTime = Date.now();

  res.on('finish', () => {
    const duration = Date.now() - startTime;
    const requestId = (req as any).id || 'unknown';

    // Log warning for slow requests
    if (duration > 2000) {
      logger.warn('Slow request detected', {
        requestId,
        method: req.method,
        path: req.path,
        duration: `${duration}ms`,
        statusCode: res.statusCode,
      });
    }
  });

  next();
};

/**
 * Request ID middleware
 *
 * Adds request ID to response headers for client-side tracing
 */
export const addRequestId = (req: Request, res: Response, next: NextFunction) => {
  const requestId = (req as any).id || uuidv4();
  (req as any).id = requestId;

  // Add request ID to response headers
  res.setHeader('X-Request-ID', requestId);

  next();
};

export default requestLogger;
