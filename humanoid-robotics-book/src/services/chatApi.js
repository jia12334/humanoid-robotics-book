/**
 * Chat API client for communicating with the RAG backend.
 */

// API base URL - configure via environment or use default
const API_BASE_URL = typeof window !== 'undefined'
  ? (window.__DOCUSAURUS_SITE_CONFIG__?.customFields?.apiUrl || 'http://localhost:8000')
  : 'http://localhost:8000';

/**
 * Send a chat message to the RAG backend.
 *
 * @param {Object} params - Chat parameters
 * @param {string} params.message - User's message
 * @param {string} [params.conversationId] - Optional conversation ID for context
 * @returns {Promise<Object>} Response with answer and sources
 */
export async function sendChatMessage({ message, conversationId }) {
  const response = await fetch(`${API_BASE_URL}/api/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      message,
      conversation_id: conversationId,
    }),
  });

  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    throw new Error(errorData.detail || `Chat request failed: ${response.status}`);
  }

  return response.json();
}

/**
 * Check backend health status.
 *
 * @returns {Promise<Object>} Health status response
 */
export async function checkHealth() {
  const response = await fetch(`${API_BASE_URL}/api/health`);

  if (!response.ok) {
    throw new Error('Health check failed');
  }

  return response.json();
}

export default {
  sendChatMessage,
  checkHealth,
};
