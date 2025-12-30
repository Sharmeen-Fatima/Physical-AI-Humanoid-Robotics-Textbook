/**
 * TypeScript Type Definitions for RAG Chatbot
 *
 * Defines interfaces for chat messages, sessions, API responses, and component props.
 */

/**
 * Source citation from RAG retrieval
 */
export interface ChatSource {
  rank: number;
  url: string;
  section: string;
  chunk_index: number;
  similarity_score: number;
}

/**
 * Metadata about the chat response
 */
export interface ChatMetadata {
  numSources: number;
  isGrounded: boolean;
  hasCitations: boolean;
  model: string;
  timestamp?: string;
  temperature?: number;
}

/**
 * Chat message (user or assistant)
 */
export interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  sources?: ChatSource[];
  metadata?: ChatMetadata;
  timestamp?: string;
}

/**
 * Chat session
 */
export interface ChatSession {
  sessionId: string;
  messages: ChatMessage[];
  createdAt: string;
  lastActivity?: string;
  numTurns?: number;
}

/**
 * API Response for /api/chat/query
 */
export interface ChatQueryResponse {
  sessionId: string;
  query: string;
  answer: string;
  sources: ChatSource[];
  metadata: ChatMetadata;
}

/**
 * API Response for /api/chat/sessions (POST)
 */
export interface CreateSessionResponse {
  sessionId: string;
  createdAt: string;
}

/**
 * API Response for /api/chat/sessions/:id (GET)
 */
export interface GetSessionResponse {
  sessionId: string;
  messages: ChatMessage[];
  createdAt: string;
  lastActivity: string;
  numTurns: number;
}

/**
 * API Error Response
 */
export interface ApiError {
  error: string;
  message: string;
  code: string;
  requestId?: string;
}

/**
 * Chatbot configuration
 */
export interface ChatbotConfig {
  apiBaseUrl?: string;
  topK?: number;
  threshold?: number;
  maxQueryLength?: number;
  position?: 'bottom-right' | 'bottom-left';
  theme?: 'light' | 'dark' | 'auto';
  welcomeMessage?: string;
}

/**
 * Chatbot state
 */
export interface ChatbotState {
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;
  messages: ChatMessage[];
  sessionId: string | null;
}
