/**
 * UserMessage Component
 *
 * Displays a user's message in the chat.
 */

import React from 'react';
import styles from './Chatbot.module.css';

interface UserMessageProps {
  content: string;
  timestamp?: string;
}

export function UserMessage({ content, timestamp }: UserMessageProps) {
  const formatTime = (isoString?: string) => {
    if (!isoString) return '';
    const date = new Date(isoString);
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className={styles.messageWrapper}>
      <div className={`${styles.message} ${styles.userMessage}`}>
        <div className={styles.messageContent}>{content}</div>
        {timestamp && <div className={styles.timestamp}>{formatTime(timestamp)}</div>}
      </div>
    </div>
  );
}

export default UserMessage;
