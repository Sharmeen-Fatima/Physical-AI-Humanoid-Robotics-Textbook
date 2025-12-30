/**
 * AssistantMessage Component
 *
 * Displays assistant's response with source citations.
 */

import React from 'react';
import type { ChatSource, ChatMetadata } from '../../types/chatbot';
import { SourcesList } from './SourcesList';
import styles from './Chatbot.module.css';

interface AssistantMessageProps {
  content: string;
  sources?: ChatSource[];
  metadata?: ChatMetadata;
  timestamp?: string;
}

export function AssistantMessage({
  content,
  sources,
  metadata,
  timestamp,
}: AssistantMessageProps) {
  const formatTime = (isoString?: string) => {
    if (!isoString) return '';
    const date = new Date(isoString);
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className={styles.messageWrapper}>
      <div className={`${styles.message} ${styles.assistantMessage}`}>
        <div className={styles.messageContent}>{content}</div>

        {sources && sources.length > 0 && <SourcesList sources={sources} />}

        <div className={styles.messageFooter}>
          {timestamp && <span className={styles.timestamp}>{formatTime(timestamp)}</span>}
          {metadata?.isGrounded && (
            <span className={styles.groundedBadge} title="Answer is grounded in sources">
              ✓ Grounded
            </span>
          )}
        </div>
      </div>
    </div>
  );
}

export default AssistantMessage;
