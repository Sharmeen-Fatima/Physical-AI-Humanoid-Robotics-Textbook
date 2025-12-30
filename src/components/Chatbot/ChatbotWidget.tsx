/**
 * ChatbotWidget Component
 *
 * Main chatbot UI component with floating toggle button and chat window.
 */

import React from 'react';
import { useChatbot } from '../../hooks/useChatbot';
import { MessageList } from './MessageList';
import { InputArea } from './InputArea';
import type { ChatbotConfig } from '../../types/chatbot';
import styles from './Chatbot.module.css';

interface ChatbotWidgetProps {
  config?: ChatbotConfig;
}

export function ChatbotWidget({ config = {} }: ChatbotWidgetProps) {
  const {
    messages,
    isOpen,
    isLoading,
    error,
    sessionId,
    toggleOpen,
    sendMessage,
    clearChat,
    closeError,
  } = useChatbot(config);

  return (
    <>
      {/* Floating toggle button */}
      <button
        className={`${styles.toggleButton} ${isOpen ? styles.toggleButtonOpen : ''}`}
        onClick={toggleOpen}
        aria-label={isOpen ? 'Close chatbot' : 'Open chatbot'}
      >
        {isOpen ? (
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
          >
            <line x1="18" y1="6" x2="6" y2="18" />
            <line x1="6" y1="6" x2="18" y2="18" />
          </svg>
        ) : (
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        )}
      </button>

      {/* Chat window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.chatTitle}>
              <svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
              >
                <circle cx="12" cy="12" r="10" />
                <path d="M8 14s1.5 2 4 2 4-2 4-2" />
                <line x1="9" y1="9" x2="9.01" y2="9" />
                <line x1="15" y1="9" x2="15.01" y2="9" />
              </svg>
              <span>Physical AI Assistant</span>
            </div>
            <div className={styles.chatActions}>
              {messages.length > 0 && (
                <button
                  className={styles.clearButton}
                  onClick={clearChat}
                  title="Clear chat history"
                  aria-label="Clear chat"
                >
                  <svg
                    width="18"
                    height="18"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    strokeWidth="2"
                  >
                    <polyline points="3 6 5 6 21 6" />
                    <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2" />
                  </svg>
                </button>
              )}
              <button
                className={styles.closeButton}
                onClick={toggleOpen}
                aria-label="Close chatbot"
              >
                <svg
                  width="18"
                  height="18"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                >
                  <line x1="18" y1="6" x2="6" y2="18" />
                  <line x1="6" y1="6" x2="18" y2="18" />
                </svg>
              </button>
            </div>
          </div>

          {/* Error banner */}
          {error && (
            <div className={styles.errorBanner}>
              <span>{error}</span>
              <button onClick={closeError} aria-label="Close error">
                ×
              </button>
            </div>
          )}

          {/* Messages */}
          <MessageList messages={messages} isLoading={isLoading} />

          {/* Input */}
          <InputArea onSend={sendMessage} disabled={isLoading} />

          {/* Footer */}
          <div className={styles.chatFooter}>
            {sessionId && (
              <span className={styles.sessionInfo} title={`Session ID: ${sessionId}`}>
                Session active
              </span>
            )}
            <span className={styles.poweredBy}>
              Powered by Gemini 2.5 Flash
            </span>
          </div>
        </div>
      )}
    </>
  );
}

export default ChatbotWidget;
