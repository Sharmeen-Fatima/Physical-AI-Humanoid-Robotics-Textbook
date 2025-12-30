/**
 * Chat API Routes
 *
 * RESTful endpoints for the RAG chatbot system.
 */

import { Router, Request, Response, NextFunction } from 'express';
import pythonBridge from '../services/pythonBridge';
import logger from '../utils/logger';

const router = Router();

/**
 * POST /api/chat/query
 *
 * Submit a user query and receive a grounded answer from the RAG chatbot.
 *
 * Request Body:
 *   - query (string, required): User's question
 *   - sessionId (string, optional): Existing session ID
 *   - topK (number, optional): Number of chunks to retrieve (default: 5)
 *   - threshold (number, optional): Similarity threshold (default: 0.70)
 *
 * Response:
 *   - 200: Successful response with answer and sources
 *   - 400: Invalid request (missing or invalid query)
 *   - 500: Server error (RAG pipeline failure)
 */
router.post('/query', async (req: Request, res: Response, next: NextFunction) => {
  const { query, sessionId, topK, threshold } = req.body;

  // Validation
  if (!query || typeof query !== 'string' || query.trim().length === 0) {
    logger.warn('Invalid query received');
    return res.status(400).json({
      error: 'Invalid request',
      message: 'Query is required and must be a non-empty string',
      code: 'INVALID_QUERY',
    });
  }

  if (query.length > 500) {
    logger.warn(`Query too long: ${query.length} characters`);
    return res.status(400).json({
      error: 'Invalid request',
      message: 'Query exceeds maximum length of 500 characters',
      code: 'QUERY_TOO_LONG',
    });
  }

  try {
    logger.info(`Processing query: "${query.substring(0, 50)}..." (session: ${sessionId || 'new'})`);

    // Call Python backend
    const response = await pythonBridge.sendQuery(query, sessionId, { topK, threshold });

    // Check if Python returned an error
    if (!response.success) {
      logger.error(`Python error: ${response.error} (code: ${response.code})`);
      return res.status(500).json({
        error: 'Server error',
        message: response.error || 'Failed to generate response',
        code: response.code || 'RAG_PIPELINE_ERROR',
      });
    }

    // Return successful response
    logger.info(`Query successful: session=${response.sessionId}, sources=${response.metadata?.numSources}`);

    return res.status(200).json({
      sessionId: response.sessionId,
      query: response.query,
      answer: response.answer,
      sources: response.sources || [],
      metadata: {
        numSources: response.metadata?.numSources || 0,
        isGrounded: response.metadata?.isGrounded || false,
        hasCitations: response.metadata?.hasCitations || false,
        model: response.metadata?.model || 'unknown',
        timestamp: new Date().toISOString(),
      },
    });
  } catch (error) {
    logger.error(`Query processing error: ${(error as Error).message}`);
    return next(error); // Pass to error handler middleware
  }
});

/**
 * POST /api/chat/sessions
 *
 * Create a new chat session.
 *
 * Response:
 *   - 201: Session created successfully
 *   - 500: Server error
 */
router.post('/sessions', async (_req: Request, res: Response, next: NextFunction) => {
  try {
    logger.info('Creating new session');

    const response = await pythonBridge.createSession();

    if (!response.success) {
      logger.error(`Session creation error: ${response.error}`);
      return res.status(500).json({
        error: 'Server error',
        message: response.error || 'Failed to create session',
        code: response.code || 'SESSION_CREATE_ERROR',
      });
    }

    logger.info(`Session created: ${response.sessionId}`);

    return res.status(201).json({
      sessionId: response.sessionId,
      createdAt: response.createdAt || new Date().toISOString(),
    });
  } catch (error) {
    logger.error(`Session creation error: ${(error as Error).message}`);
    return next(error);
  }
});

/**
 * GET /api/chat/sessions/:id
 *
 * Get session history and metadata.
 *
 * Parameters:
 *   - id (string): Session ID
 *
 * Response:
 *   - 200: Session data retrieved successfully
 *   - 404: Session not found or expired
 *   - 500: Server error
 */
router.get('/sessions/:id', async (req: Request, res: Response, next: NextFunction) => {
  const { id } = req.params;

  if (!id || typeof id !== 'string') {
    return res.status(400).json({
      error: 'Invalid request',
      message: 'Session ID is required',
      code: 'MISSING_SESSION_ID',
    });
  }

  try {
    logger.info(`Retrieving session: ${id}`);

    const response = await pythonBridge.getSession(id);

    if (!response.success) {
      if (response.code === 'SESSION_NOT_FOUND') {
        logger.warn(`Session not found: ${id}`);
        return res.status(404).json({
          error: 'Not found',
          message: 'Session not found or expired',
          code: 'SESSION_NOT_FOUND',
        });
      }

      logger.error(`Session retrieval error: ${response.error}`);
      return res.status(500).json({
        error: 'Server error',
        message: response.error || 'Failed to retrieve session',
        code: response.code || 'SESSION_GET_ERROR',
      });
    }

    logger.info(`Session retrieved: ${id}, turns=${response.numTurns}`);

    return res.status(200).json({
      sessionId: response.sessionId,
      messages: response.messages || [],
      createdAt: response.createdAt,
      lastActivity: response.lastActivity,
      numTurns: response.numTurns || 0,
    });
  } catch (error) {
    logger.error(`Session retrieval error: ${(error as Error).message}`);
    return next(error);
  }
});

/**
 * DELETE /api/chat/sessions/:id
 *
 * Clear session history.
 *
 * Parameters:
 *   - id (string): Session ID
 *
 * Response:
 *   - 204: Session deleted successfully (no content)
 *   - 500: Server error
 */
router.delete('/sessions/:id', async (req: Request, res: Response, next: NextFunction) => {
  const { id } = req.params;

  if (!id || typeof id !== 'string') {
    return res.status(400).json({
      error: 'Invalid request',
      message: 'Session ID is required',
      code: 'MISSING_SESSION_ID',
    });
  }

  try {
    logger.info(`Deleting session: ${id}`);

    const response = await pythonBridge.deleteSession(id);

    if (!response.success) {
      logger.error(`Session deletion error: ${response.error}`);
      return res.status(500).json({
        error: 'Server error',
        message: response.error || 'Failed to delete session',
        code: response.code || 'SESSION_DELETE_ERROR',
      });
    }

    logger.info(`Session deleted: ${id}`);

    // Return 204 No Content on successful deletion
    return res.status(204).send();
  } catch (error) {
    logger.error(`Session deletion error: ${(error as Error).message}`);
    return next(error);
  }
});

export default router;
