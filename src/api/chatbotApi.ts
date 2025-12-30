/**
 * Chatbot API Client
 *
 * HTTP client for communicating with the Express API backend.
 * Handles all chatbot-related API requests.
 */

import type {
  ChatQueryResponse,
  CreateSessionResponse,
  GetSessionResponse,
} from '../types/chatbot';

/**
 * API configuration
 */
const API_BASE_URL = typeof window !== 'undefined'
  ? (window as any)._env_?.REACT_APP_API_URL || 'http://localhost:3001'
  : 'http://localhost:3001';

/**
 * API client class
 */
export class ChatbotApiClient {
  private baseUrl: string;

  constructor(baseUrl: string = API_BASE_URL) {
    this.baseUrl = baseUrl;
  }

  /**
   * Send a chat query
   *
   * @param query - User's question
   * @param sessionId - Optional session ID for conversation continuity
   * @param options - Optional search parameters (topK, threshold)
   * @returns Promise resolving to chat response
   */
  async sendQuery(
    query: string,
    sessionId?: string,
    options?: { topK?: number; threshold?: number }
  ): Promise<ChatQueryResponse> {
    const response = await fetch(`${this.baseUrl}/api/chat/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query,
        sessionId,
        ...options,
      }),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || 'Failed to send query');
    }

    return response.json();
  }

  /**
   * Create a new chat session
   *
   * @returns Promise resolving to new session data
   */
  async createSession(): Promise<CreateSessionResponse> {
    const response = await fetch(`${this.baseUrl}/api/chat/sessions`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || 'Failed to create session');
    }

    return response.json();
  }

  /**
   * Get session history
   *
   * @param sessionId - Session ID
   * @returns Promise resolving to session data
   */
  async getSession(sessionId: string): Promise<GetSessionResponse> {
    const response = await fetch(`${this.baseUrl}/api/chat/sessions/${sessionId}`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || 'Failed to get session');
    }

    return response.json();
  }

  /**
   * Delete a chat session
   *
   * @param sessionId - Session ID
   * @returns Promise resolving when session is deleted
   */
  async deleteSession(sessionId: string): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/chat/sessions/${sessionId}`, {
      method: 'DELETE',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || 'Failed to delete session');
    }
  }

  /**
   * Check API health
   *
   * @returns Promise resolving to health status
   */
  async healthCheck(): Promise<{ status: string; service: string }> {
    const response = await fetch(`${this.baseUrl}/api/health`);

    if (!response.ok) {
      throw new Error('API health check failed');
    }

    return response.json();
  }
}

// Export singleton instance
export const chatbotApi = new ChatbotApiClient();

export default chatbotApi;
