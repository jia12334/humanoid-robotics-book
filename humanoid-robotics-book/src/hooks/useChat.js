/**
 * Custom hook for managing chat state and interactions.
 */

import { useState, useCallback, useRef } from 'react';
import { sendChatMessage } from '../services/chatApi';

/**
 * Generate a unique ID for messages.
 */
function generateId() {
  return `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

/**
 * Custom hook for chat functionality.
 *
 * @returns {Object} Chat state and functions
 */
export function useChat() {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [conversationId, setConversationId] = useState(null);
  const abortControllerRef = useRef(null);

  /**
   * Send a message and get a response.
   */
  const sendMessage = useCallback(async (userMessage) => {
    if (!userMessage.trim() || isLoading) return;

    // Clear any previous errors
    setError(null);

    // Add user message to the list
    const userMsg = {
      id: generateId(),
      role: 'user',
      content: userMessage,
      timestamp: new Date().toISOString(),
    };

    setMessages((prev) => [...prev, userMsg]);
    setIsLoading(true);

    try {
      // Create abort controller for potential cancellation
      abortControllerRef.current = new AbortController();

      // Send to backend
      const response = await sendChatMessage({
        message: userMessage,
        conversationId,
      });

      // Update conversation ID if not set
      if (!conversationId && response.conversation_id) {
        setConversationId(response.conversation_id);
      }

      // Add assistant response
      const assistantMsg = {
        id: generateId(),
        role: 'assistant',
        content: response.answer,
        sources: response.sources || [],
        timestamp: new Date().toISOString(),
      };

      setMessages((prev) => [...prev, assistantMsg]);
    } catch (err) {
      // Don't show error if request was aborted
      if (err.name === 'AbortError') return;

      setError(err.message || 'Failed to get response. Please try again.');

      // Add error message to chat
      const errorMsg = {
        id: generateId(),
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        isError: true,
        timestamp: new Date().toISOString(),
      };
      setMessages((prev) => [...prev, errorMsg]);
    } finally {
      setIsLoading(false);
      abortControllerRef.current = null;
    }
  }, [conversationId, isLoading]);

  /**
   * Clear the conversation and start fresh.
   */
  const clearChat = useCallback(() => {
    // Cancel any pending request
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
    }

    setMessages([]);
    setConversationId(null);
    setError(null);
    setIsLoading(false);
  }, []);

  /**
   * Retry the last message if there was an error.
   */
  const retryLastMessage = useCallback(() => {
    // Find the last user message
    const lastUserMsg = [...messages].reverse().find((m) => m.role === 'user');

    if (lastUserMsg) {
      // Remove the error message
      setMessages((prev) => prev.filter((m) => !m.isError));
      // Resend
      sendMessage(lastUserMsg.content);
    }
  }, [messages, sendMessage]);

  return {
    messages,
    isLoading,
    error,
    conversationId,
    sendMessage,
    clearChat,
    retryLastMessage,
  };
}

export default useChat;
