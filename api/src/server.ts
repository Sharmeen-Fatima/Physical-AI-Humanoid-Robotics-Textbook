/**
 * Express API Server
 *
 * Main entry point for the RAG Chatbot API.
 * Integrates all routes, middleware, and services.
 */

import express, { Application } from 'express';
import cors from 'cors';
import helmet from 'helmet';
import dotenv from 'dotenv';
import path from 'path';

// Import middleware
import { requestLogger, performanceMonitor, addRequestId } from './middleware/requestLogger';
import { chatRateLimiter, sessionCreationRateLimiter } from './middleware/rateLimit';
import { errorHandler, notFoundHandler } from './middleware/errorHandler';
import logger from './utils/logger';

// Import routes
import chatRoutes from './routes/chat';

// Load environment variables
dotenv.config({ path: path.join(__dirname, '..', '.env') });

// Initialize Express app
const app: Application = express();
const PORT = process.env['PORT'] || 3001;
const NODE_ENV = process.env['NODE_ENV'] || 'development';

// ===========================
// Global Middleware
// ===========================

// Security middleware
app.use(helmet());

// CORS configuration
const corsOrigin = process.env['CORS_ORIGIN'] || 'http://localhost:3000';
const corsOptions = {
  origin: corsOrigin.includes(',')
    ? corsOrigin.split(',').map(origin => origin.trim())
    : corsOrigin,
  credentials: true,
  optionsSuccessStatus: 200,
};
app.use(cors(corsOptions));

// Body parsing middleware
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true, limit: '10mb' }));

// Request ID and logging middleware
app.use(addRequestId);
app.use(requestLogger);
app.use(performanceMonitor);

// ===========================
// Health Check Endpoint
// ===========================

app.get('/health', (_req, res) => {
  res.status(200).json({
    status: 'healthy',
    environment: NODE_ENV,
    timestamp: new Date().toISOString(),
    uptime: process.uptime(),
  });
});

app.get('/api/health', (_req, res) => {
  res.status(200).json({
    status: 'healthy',
    service: 'RAG Chatbot API',
    environment: NODE_ENV,
    timestamp: new Date().toISOString(),
    uptime: process.uptime(),
  });
});

// ===========================
// API Routes
// ===========================

// Apply rate limiting to chat routes
app.use('/api/chat/query', chatRateLimiter);
app.use('/api/chat/sessions', sessionCreationRateLimiter);

// Mount chat routes
app.use('/api/chat', chatRoutes);

// ===========================
// Error Handling
// ===========================

// 404 handler (must come after all routes)
app.use(notFoundHandler);

// Global error handler (must be last)
app.use(errorHandler);

// ===========================
// Server Startup
// ===========================

/**
 * Start the Express server
 */
const startServer = () => {
  try {
    app.listen(PORT, () => {
      logger.info(`🚀 Server started successfully`);
      logger.info(`Environment: ${NODE_ENV}`);
      logger.info(`Port: ${PORT}`);
      logger.info(`Health check: http://localhost:${PORT}/health`);
      logger.info(`API endpoint: http://localhost:${PORT}/api/chat`);

      // Log environment configuration
      if (NODE_ENV === 'development') {
        logger.debug(`CORS Origins: ${JSON.stringify(corsOptions.origin)}`);
        logger.debug('Rate Limit: 60 requests/minute');
      }
    });
  } catch (error) {
    logger.error('Failed to start server:', error);
    process.exit(1);
  }
};

// ===========================
// Graceful Shutdown
// ===========================

/**
 * Handle graceful shutdown on SIGTERM and SIGINT
 */
const gracefulShutdown = (signal: string) => {
  logger.info(`${signal} received. Starting graceful shutdown...`);

  // Close server
  process.exit(0);
};

process.on('SIGTERM', () => gracefulShutdown('SIGTERM'));
process.on('SIGINT', () => gracefulShutdown('SIGINT'));

// ===========================
// Error Handling
// ===========================

/**
 * Handle unhandled promise rejections
 */
process.on('unhandledRejection', (reason: any, promise: Promise<any>) => {
  logger.error('Unhandled Rejection at:', promise, 'reason:', reason);
  // Optionally exit the process
  // process.exit(1);
});

/**
 * Handle uncaught exceptions
 */
process.on('uncaughtException', (error: Error) => {
  logger.error('Uncaught Exception:', error);
  // Exit the process for uncaught exceptions
  process.exit(1);
});

// Start server if not in test mode
if (process.env['NODE_ENV'] !== 'test') {
  startServer();
}

// Export app for testing
export default app;
