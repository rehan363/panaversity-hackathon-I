/**
 * Main RAG Chatbot component with floating action button and text selection handling.
 */

import React, { useState, useEffect, useCallback } from 'react';
import { ChatModal } from './ChatModal';
import { useChatAPI } from '../../hooks/useChatAPI';
import useSessionStorage from '../../hooks/useSessionStorage'; // Correct import for default export
import { useTextSelection } from '../../hooks/useTextSelection';
import type { ChatMessage } from './types'; // Import ChatMessage type
import styles from './styles.module.css';

export const RAGChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [chatHistory, setChatHistory] = useSessionStorage<ChatMessage[]>('rag-chat-history', []);

  const { isLoading, error, sendQuery, clearError, rateLimitSeconds } = useChatAPI(chatHistory, setChatHistory);
  const { selection, clearSelection } = useTextSelection();

  const handleOpenChat = () => {
    setIsOpen(true);
    clearError();
  };

  const handleCloseChat = () => {
    setIsOpen(false);
  };

  const handleSubmitQuery = async (query: string) => {
    await sendQuery(query);
  };

  const handleClearHistory = () => {
    setChatHistory([]);
    clearError(); // Also clear any errors
  };

  const handleAskAboutSelection = () => {
    if (!selection) return;

    // Open the chat modal
    handleOpenChat();

    // Clear the on-screen selection button
    clearSelection();

    // The logic to pre-fill the input will be handled in QueryInput.tsx (T051)
    // For now, we can pass the selection context to the modal.
    // This part of the implementation anticipates the next steps.
  };

  return (
    <>
      {/* Floating Action Button */}
      <button
        className={styles.chatButton}
        onClick={handleOpenChat}
        aria-label="Open AI chat assistant"
        title="Ask AI Assistant"
      >
        💬
      </button>

      {/* "Ask AI about this" button on text selection */}
      {selection && (
        <button
          className={styles.selectionButton}
          style={{
            top: `${selection.position.y}px`,
            left: `${selection.position.x}px`,
          }}
          onClick={handleAskAboutSelection}
        >
          Ask AI about this
        </button>
      )}

      {/* Chat Modal */}
      <ChatModal
        isOpen={isOpen}
        onClose={handleCloseChat}
        messages={chatHistory} // Pass chatHistory to ChatModal
        isLoading={isLoading}
        error={error}
        onSubmitQuery={handleSubmitQuery}
        rateLimitSeconds={rateLimitSeconds}
        selection={selection}
        onClearHistory={handleClearHistory} // Pass the clear history handler
      />
    </>
  );
};

export default RAGChatbot;
