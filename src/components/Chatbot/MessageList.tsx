/**
 * MessageList Component
 *
 * Renders list of chat messages with auto-scroll to latest message.
 */

import React, { useEffect, useRef } from 'react';
import type { ChatMessage } from '../../types/chatbot';
import { UserMessage } from './UserMessage';
import { AssistantMessage } from './AssistantMessage';
import styles from './Chatbot.module.css';

interface MessageListProps {
  messages: ChatMessage[];
  isLoading?: boolean;
}

export function MessageList({ messages, isLoading = false }: MessageListProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  if (messages.length === 0 && !isLoading) {
    return (
      <div className={styles.emptyState}>
        <svg
          width="48"
          height="48"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
        <p>Ask a question about humanoid robotics!</p>
        <p className={styles.emptyStateHint}>
          I can help you understand concepts from the Physical AI textbook.
        </p>
      </div>
    );
  }

  return (
    <div className={styles.messageList}>
      {messages.map((message, index) => (
        <div key={index}>
          {message.role === 'user' ? (
            <UserMessage content={message.content} timestamp={message.timestamp} />
          ) : (
            <AssistantMessage
              content={message.content}
              sources={message.sources}
              metadata={message.metadata}
              timestamp={message.timestamp}
            />
          )}
        </div>
      ))}

      {isLoading && (
        <div className={styles.messageWrapper}>
          <div className={`${styles.message} ${styles.assistantMessage}`}>
            <div className={styles.typingIndicator}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        </div>
      )}

      <div ref={messagesEndRef} />
    </div>
  );
}

export default MessageList;
