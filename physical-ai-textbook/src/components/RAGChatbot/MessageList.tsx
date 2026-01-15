/**
 * Message list component for displaying chat history
 */

import React, { useEffect, useRef, useState } from 'react';
import type { ChatMessage } from './types';
import { Citation } from './Citation';
import styles from './styles.module.css';

interface MessageListProps {
  messages: ChatMessage[];
  isLoading: boolean;
}

export const MessageList: React.FC<MessageListProps> = ({ messages, isLoading }) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const scrollContainerRef = useRef<HTMLDivElement>(null);
  const [isScrolledToBottom, setIsScrolledToBottom] = useState(true);

  // Auto-scroll to bottom when new messages arrive, but only if user is already at the bottom
  useEffect(() => {
    if (isScrolledToBottom) {
      messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, isLoading, isScrolledToBottom]);

  // Check scroll position to determine if user is at the bottom
  const handleScroll = () => {
    if (scrollContainerRef.current) {
      const { scrollTop, clientHeight, scrollHeight } = scrollContainerRef.current;
      const threshold = 10; // Pixels from bottom to consider "at bottom"
      setIsScrolledToBottom(scrollTop + clientHeight >= scrollHeight - threshold);
    }
  };

  useEffect(() => {
    const scrollContainer = scrollContainerRef.current;
    if (scrollContainer) {
      scrollContainer.addEventListener('scroll', handleScroll);
      // Initial check
      handleScroll();
    }
    return () => {
      if (scrollContainer) {
        scrollContainer.removeEventListener('scroll', handleScroll);
      }
    };
  }, []);

  const formatTime = (date: Date) => {
    return new Date(date).toLocaleTimeString('en-US', {
      hour: '2-digit',
      minute: '2-digit',
    });
  };

  if (messages.length === 0 && !isLoading) {
    return (
      <div className={styles.messageList} ref={scrollContainerRef}>
        <div className={styles.emptyState}>
          <div className={styles.emptyStateIcon}>💬</div>
          <div className={styles.emptyStateTitle}>Welcome to AI Assistant!</div>
          <div className={styles.emptyStateText}>
            Ask me anything about the Physical AI textbook.
            <br />
            I'll provide answers with source citations.
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.messageList} ref={scrollContainerRef}>
      {messages.map((message) => (
        <div
          key={message.id}
          className={`${styles.message} ${
            message.role === 'user' ? styles.userMessage : styles.assistantMessage
          }`}
        >
          <div
            className={`${styles.messageBubble} ${
              message.role === 'user' ? styles.userBubble : styles.assistantBubble
            }`}
          >
            <p className={styles.messageContent}>{message.content}</p>

            {message.citations && message.citations.length > 0 && (
              <div className={styles.citations}>
                <div className={styles.citationsTitle}>Sources:</div>
                {message.citations.map((citation, index) => (
                  <Citation key={index} citation={citation} />
                ))}
              </div>
            )}

            <div className={styles.messageTime}>
              {formatTime(message.timestamp)}
              {message.processing_time_ms && ` • ${message.processing_time_ms}ms`}
            </div>
          </div>
        </div>
      ))}

      {isLoading && (
        <div className={`${styles.message} ${styles.assistantMessage}`}>
          <div className={`${styles.messageBubble} ${styles.assistantBubble}`}>
            <div className={styles.loadingDots}>
              <div className={styles.dot} />
              <div className={styles.dot} />
              <div className={styles.dot} />
            </div>
          </div>
        </div>
      )}

      <div ref={messagesEndRef} />
    </div>
  );
};

