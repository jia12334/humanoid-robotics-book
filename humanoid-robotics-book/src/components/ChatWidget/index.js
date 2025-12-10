/**
 * ChatWidget - Main chat widget component for Docusaurus
 *
 * A floating chat interface that allows users to ask questions
 * about the Physical AI and Humanoid Robotics book content.
 */

import React, { useState, useRef, useEffect } from 'react';
import { useChat } from '../../hooks/useChat';
import styles from './styles.module.css';

// Icons as inline SVGs for simplicity
const ChatIcon = () => (
  <svg className={styles.fabIcon} fill="none" viewBox="0 0 24 24" stroke="currentColor">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2}
      d="M8 12h.01M12 12h.01M16 12h.01M21 12c0 4.418-4.03 8-9 8a9.863 9.863 0 01-4.255-.949L3 20l1.395-3.72C3.512 15.042 3 13.574 3 12c0-4.418 4.03-8 9-8s9 3.582 9 8z" />
  </svg>
);

const CloseIcon = () => (
  <svg className={styles.headerButtonIcon} fill="none" viewBox="0 0 24 24" stroke="currentColor">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
  </svg>
);

const ClearIcon = () => (
  <svg className={styles.headerButtonIcon} fill="none" viewBox="0 0 24 24" stroke="currentColor">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2}
      d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16" />
  </svg>
);

const SendIcon = () => (
  <svg className={styles.sendButtonIcon} fill="none" viewBox="0 0 24 24" stroke="currentColor">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2}
      d="M12 19l9 2-9-18-9 18 9-2zm0 0v-8" />
  </svg>
);

const RobotIcon = () => (
  <svg className={styles.headerIcon} fill="none" viewBox="0 0 24 24" stroke="currentColor">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2}
      d="M9.75 17L9 20l-1 1h8l-1-1-.75-3M3 13h18M5 17h14a2 2 0 002-2V5a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z" />
  </svg>
);

const BookIcon = () => (
  <svg className={styles.welcomeIcon} fill="none" viewBox="0 0 24 24" stroke="currentColor">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2}
      d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.747 0 3.332.477 4.5 1.253v13C19.832 18.477 18.247 18 16.5 18c-1.746 0-3.332.477-4.5 1.253" />
  </svg>
);

/**
 * Message component for displaying individual chat messages
 */
function Message({ message }) {
  const isUser = message.role === 'user';
  const isError = message.isError;

  return (
    <div className={`${styles.message} ${isUser ? styles.messageUser : styles.messageAssistant} ${isError ? styles.messageError : ''}`}>
      <div className={styles.messageContent}>
        {message.content}
      </div>

      {/* Show sources for assistant messages */}
      {!isUser && message.sources && message.sources.length > 0 && (
        <div className={styles.sources}>
          <div className={styles.sourcesTitle}>Sources:</div>
          {message.sources.map((source, idx) => (
            <a
              key={idx}
              href={source.url}
              className={styles.sourceLink}
              target="_blank"
              rel="noopener noreferrer"
            >
              {source.title} {source.section && `- ${source.section}`}
            </a>
          ))}
        </div>
      )}
    </div>
  );
}

/**
 * Loading indicator component
 */
function LoadingIndicator() {
  return (
    <div className={styles.loadingIndicator}>
      <div className={styles.loadingDots}>
        <span className={styles.loadingDot}></span>
        <span className={styles.loadingDot}></span>
        <span className={styles.loadingDot}></span>
      </div>
      <span>Thinking...</span>
    </div>
  );
}

/**
 * Welcome message when chat is empty
 */
function WelcomeMessage() {
  return (
    <div className={styles.welcome}>
      <BookIcon />
      <div className={styles.welcomeTitle}>
        Ask about the book!
      </div>
      <div className={styles.welcomeText}>
        I can help you understand concepts about Physical AI, ROS 2,
        humanoid robotics, digital twins, and more from this textbook.
      </div>
    </div>
  );
}

/**
 * Main ChatWidget component
 */
export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const {
    messages,
    isLoading,
    sendMessage,
    clearChat,
  } = useChat();

  // Scroll to bottom when messages change
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, isLoading]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Handle form submission
  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      sendMessage(inputValue);
      setInputValue('');
    }
  };

  // Handle keyboard shortcuts
  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      handleSubmit(e);
    }
  };

  return (
    <>
      {/* Floating Action Button */}
      <button
        className={styles.fab}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat assistant'}
      >
        {isOpen ? <CloseIcon /> : <ChatIcon />}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.header}>
            <div className={styles.headerTitle}>
              <RobotIcon />
              <span>Book Assistant</span>
            </div>
            <div className={styles.headerActions}>
              <button
                className={styles.headerButton}
                onClick={clearChat}
                aria-label="Clear chat"
                title="Clear chat"
              >
                <ClearIcon />
              </button>
              <button
                className={styles.headerButton}
                onClick={() => setIsOpen(false)}
                aria-label="Close chat"
              >
                <CloseIcon />
              </button>
            </div>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 ? (
              <WelcomeMessage />
            ) : (
              <>
                {messages.map((msg) => (
                  <Message key={msg.id} message={msg} />
                ))}
                {isLoading && <LoadingIndicator />}
                <div ref={messagesEndRef} />
              </>
            )}
          </div>

          {/* Input Area */}
          <form className={styles.inputArea} onSubmit={handleSubmit}>
            <input
              ref={inputRef}
              type="text"
              className={styles.input}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask about robotics, ROS 2, AI..."
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !inputValue.trim()}
              aria-label="Send message"
            >
              <SendIcon />
            </button>
          </form>
        </div>
      )}
    </>
  );
}
