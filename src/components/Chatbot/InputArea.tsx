/**
 * InputArea Component
 *
 * Text input field for user queries with send button and character counter.
 */

import React, { useState, KeyboardEvent, ChangeEvent } from 'react';
import styles from './Chatbot.module.css';

interface InputAreaProps {
  onSend: (message: string) => void;
  disabled?: boolean;
  maxLength?: number;
  placeholder?: string;
}

export function InputArea({
  onSend,
  disabled = false,
  maxLength = 500,
  placeholder = 'Ask a question about humanoid robotics...',
}: InputAreaProps) {
  const [input, setInput] = useState('');

  const handleSubmit = () => {
    const trimmed = input.trim();
    if (trimmed && !disabled) {
      onSend(trimmed);
      setInput('');
    }
  };

  const handleKeyPress = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  const handleChange = (e: ChangeEvent<HTMLTextAreaElement>) => {
    const value = e.target.value;
    if (value.length <= maxLength) {
      setInput(value);
    }
  };

  const remainingChars = maxLength - input.length;
  const isNearLimit = remainingChars < 50;

  return (
    <div className={styles.inputArea}>
      <textarea
        className={styles.input}
        value={input}
        onChange={handleChange}
        onKeyPress={handleKeyPress}
        placeholder={placeholder}
        disabled={disabled}
        rows={1}
        aria-label="Chat message input"
      />
      <div className={styles.inputActions}>
        <span
          className={`${styles.charCounter} ${isNearLimit ? styles.charCounterWarning : ''}`}
        >
          {remainingChars}
        </span>
        <button
          className={styles.sendButton}
          onClick={handleSubmit}
          disabled={disabled || !input.trim()}
          aria-label="Send message"
        >
          <svg
            width="20"
            height="20"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
          >
            <path d="M22 2L11 13M22 2l-7 20-4-9-9-4 20-7z" />
          </svg>
        </button>
      </div>
    </div>
  );
}

export default InputArea;
