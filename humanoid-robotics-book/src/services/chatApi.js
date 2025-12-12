/**
 * Chat API client for communicating with the RAG backend.
 */

// API base URL - configure via environment or use same domain (empty string)
// For Vercel deployment: empty string means /api/* on same domain
// For local dev: set REACT_APP_API_URL=http://localhost:8000
const API_BASE_URL = typeof window !== 'undefined'
  ? (window.__DOCUSAURUS_SITE_CONFIG__?.customFields?.apiUrl ?? '')
  : '';

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
