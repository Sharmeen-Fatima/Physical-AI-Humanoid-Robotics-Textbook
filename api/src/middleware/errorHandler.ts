/**
 * Error Handling Middleware
 *
 * Centralized error handling for the Express API.
 * Catches errors from routes and formats user-friendly responses.
 */

import { Request, Response, NextFunction } from 'express';
import logger from '../utils/logger';

/**
 * Custom error class with additional properties
 */
export class ApiError extends Error {
  statusCode: number;
  code: string;
  isOperational: boolean;

  constructor(message: string, statusCode: number = 500, code: string = 'INTERNAL_ERROR') {
    super(message);
    this.statusCode = statusCode;
    this.code = code;
    this.isOperational = true; // Distinguishes operational errors from programmer errors
    Error.captureStackTrace(this, this.constructor);
  }
}

/**
 * Error response interface
 */
interface ErrorResponse {
  error: string;
  message: string;
  code: string;
  requestId?: string;
  stack?: string;
}

/**
 * Main error handling middleware
 *
 * Features:
 * - Catches async errors from routes
 * - Formats user-friendly error messages
 * - Logs errors with Winston
 * - Returns appropriate HTTP status codes
 * - Includes stack traces in development mode
 */
export const errorHandler = (
  err: Error | ApiError,
  req: Request,
  res: Response,
  _next: NextFunction
) => {
  // Default to 500 Internal Server Error
  let statusCode = 500;
  let code = 'INTERNAL_ERROR';
  let message = 'An unexpected error occurred. Please try again later.';

  // Extract error details from ApiError instances
  if (err instanceof ApiError) {
    statusCode = err.statusCode;
    code = err.code;
    message = err.message;
  }

  // Log the error
  const requestId = (req as any).id || 'unknown';
  logger.error('Error in request processing', {
    requestId,
    method: req.method,
    path: req.path,
    statusCode,
    code,
    message: err.message,
    stack: err.stack,
    ip: req.ip,
  });

  // Prepare error response
  const errorResponse: ErrorResponse = {
    error: getErrorType(statusCode),
    message,
    code,
    requestId,
  };

  // Include stack trace in development mode
  if (process.env['NODE_ENV'] === 'development') {
    errorResponse.stack = err.stack;
  }

  // Send error response
  res.status(statusCode).json(errorResponse);
};

/**
 * Not Found (404) handler
 *
 * Catches requests to undefined routes
 */
export const notFoundHandler = (req: Request, _res: Response, next: NextFunction) => {
  const error = new ApiError(
    `Route not found: ${req.method} ${req.path}`,
    404,
    'ROUTE_NOT_FOUND'
  );

  logger.warn(`404 - Route not found: ${req.method} ${req.path}`, {
    method: req.method,
    path: req.path,
    ip: req.ip,
  });

  next(error);
};

/**
 * Async route handler wrapper
 *
 * Wraps async route handlers to catch promise rejections
 * and pass them to the error handling middleware.
 *
 * Usage:
 *   router.get('/path', asyncHandler(async (req, res) => {
 *     const data = await someAsyncOperation();
 *     res.json(data);
 *   }));
 */
export const asyncHandler = (
  fn: (req: Request, res: Response, next: NextFunction) => Promise<any>
) => {
  return (req: Request, res: Response, next: NextFunction) => {
    Promise.resolve(fn(req, res, next)).catch(next);
  };
};

/**
 * Get error type from status code
 */
function getErrorType(statusCode: number): string {
  if (statusCode >= 400 && statusCode < 500) {
    return 'Client Error';
  } else if (statusCode >= 500) {
    return 'Server Error';
  }
  return 'Error';
}

/**
 * Validation error handler
 *
 * Formats validation errors from request validation libraries
 */
export const validationErrorHandler = (errors: any[]): ApiError => {
  const message = errors.map((err) => err.msg || err.message).join(', ');
  return new ApiError(message, 400, 'VALIDATION_ERROR');
};

export default errorHandler;
