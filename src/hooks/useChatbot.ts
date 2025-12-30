/**
 * Custom React Hook for Chatbot State Management
 *
 * Manages chatbot state, messages, and API interactions.
 */

import { useState, useCallback, useEffect } from 'react';
import { chatbotApi } from '../api/chatbotApi';
import type { ChatMessage, ChatbotConfig } from '../types/chatbot';

/**
 * Chatbot hook return type
 */
interface UseChatbotReturn {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;
  sessionId: string | null;
  toggleOpen: () => void;
  sendMessage: (query: string) => Promise<void>;
  clearChat: () => void;
  closeError: () => void;
}

/**
 * Custom hook for chatbot functionality
 *
 * @param config - Chatbot configuration
 * @returns Chatbot state and actions
 */
export function useChatbot(config: ChatbotConfig = {}): UseChatbotReturn {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState<string | null>(null);

  /**
   * Load session history from API
   */
  const loadSessionHistory = useCallback(async (sid: string) => {
    try {
      const session = await chatbotApi.getSession(sid);
      setMessages(session.messages);
    } catch (err) {
      console.error('Failed to load session history:', err);
      // If session not found, clear it
      if (typeof window !== 'undefined') {
        localStorage.removeItem('chatbot_session_id');
      }
      setSessionId(null);
    }
  }, []);

  /**
   * Load session ID from localStorage on mount
   */
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const savedSessionId = localStorage.getItem('chatbot_session_id');
    if (savedSessionId) {
      setSessionId(savedSessionId);
      // Optionally load session history
      loadSessionHistory(savedSessionId);
    }
  }, [loadSessionHistory]);

  /**
   * Save session ID to localStorage
   */
  useEffect(() => {
    if (typeof window === 'undefined') return;

    if (sessionId) {
      localStorage.setItem('chatbot_session_id', sessionId);
    }
  }, [sessionId]);

  /**
   * Toggle chatbot open/closed
   */
  const toggleOpen = useCallback(() => {
    setIsOpen((prev) => !prev);
  }, []);

  /**
   * Send a message to the chatbot
   */
  const sendMessage = useCallback(
    async (query: string) => {
      if (!query.trim() || isLoading) return;

      setIsLoading(true);
      setError(null);

      // Add user message immediately
      const userMessage: ChatMessage = {
        role: 'user',
        content: query,
        timestamp: new Date().toISOString(),
      };
      setMessages((prev) => [...prev, userMessage]);

      try {
        // Send query to API
        const response = await chatbotApi.sendQuery(
          query,
          sessionId || undefined,
          {
            topK: config.topK || 5,
            threshold: config.threshold || 0.70,
          }
        );

        // Update session ID if new
        if (response.sessionId && response.sessionId !== sessionId) {
          setSessionId(response.sessionId);
        }

        // Add assistant message
        const assistantMessage: ChatMessage = {
          role: 'assistant',
          content: response.answer,
          sources: response.sources,
          metadata: response.metadata,
          timestamp: response.metadata.timestamp || new Date().toISOString(),
        };
        setMessages((prev) => [...prev, assistantMessage]);
      } catch (err) {
        const errorMessage =
          err instanceof Error ? err.message : 'Failed to send message';
        setError(errorMessage);

        // Remove user message on error
        setMessages((prev) => prev.slice(0, -1));
      } finally {
        setIsLoading(false);
      }
    },
    [sessionId, isLoading, config.topK, config.threshold]
  );

  /**
   * Clear chat history
   */
  const clearChat = useCallback(async () => {
    if (sessionId) {
      try {
        await chatbotApi.deleteSession(sessionId);
      } catch (err) {
        console.error('Failed to delete session:', err);
      }
    }
    setMessages([]);
    setSessionId(null);
    if (typeof window !== 'undefined') {
      localStorage.removeItem('chatbot_session_id');
    }
  }, [sessionId]);

  /**
   * Close error message
   */
  const closeError = useCallback(() => {
    setError(null);
  }, []);

  return {
    messages,
    isOpen,
    isLoading,
    error,
    sessionId,
    toggleOpen,
    sendMessage,
    clearChat,
    closeError,
  };
}

export default useChatbot;
