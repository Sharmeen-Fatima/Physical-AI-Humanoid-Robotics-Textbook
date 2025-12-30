/**
 * SourcesList Component
 *
 * Displays source citations from RAG retrieval with expandable details.
 */

import React, { useState } from 'react';
import type { ChatSource } from '../../types/chatbot';
import styles from './Chatbot.module.css';

interface SourcesListProps {
  sources: ChatSource[];
}

export function SourcesList({ sources }: SourcesListProps) {
  const [isExpanded, setIsExpanded] = useState(false);

  if (!sources || sources.length === 0) {
    return null;
  }

  return (
    <div className={styles.sourcesContainer}>
      <button
        className={styles.sourcesToggle}
        onClick={() => setIsExpanded(!isExpanded)}
        aria-expanded={isExpanded}
      >
        <svg
          width="16"
          height="16"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          className={isExpanded ? styles.iconRotated : ''}
        >
          <polyline points="9 18 15 12 9 6" />
        </svg>
        <span>
          {sources.length} source{sources.length !== 1 ? 's' : ''}
        </span>
      </button>

      {isExpanded && (
        <div className={styles.sourcesList}>
          {sources.map((source, index) => (
            <div key={index} className={styles.sourceItem}>
              <div className={styles.sourceRank}>Source {source.rank}</div>
              <a
                href={source.url}
                target="_blank"
                rel="noopener noreferrer"
                className={styles.sourceLink}
              >
                {source.section}
              </a>
              <div className={styles.sourceScore}>
                Relevance: {(source.similarity_score * 100).toFixed(1)}%
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

export default SourcesList;
