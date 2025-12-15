/**
 * Chat API client for communicating with the RAG backend.
 */

// API base URL - use localhost for development, Railway for production
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
  ? 'http://localhost:8000'
  : 'https://humanoid-robotics-book-production.up.railway.app';

// Debug logging
if (typeof window !== 'undefined') {
  console.log('[ChatAPI] API_BASE_URL:', API_BASE_URL);
}

/**
 * Send a chat message to the RAG backend.
 *
 * @param {Object} params - Chat parameters
 * @param {string} params.message - User's message
 * @param {string} [params.conversationId] - Optional conversation ID for context
 * @returns {Promise<Object>} Response with answer and sources
 */
export async function sendChatMessage({ message, conversationId }) {
  const url = `${API_BASE_URL}/api/chat`;
  console.log('[ChatAPI] Sending request to:', url);

  try {
    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        message,
        conversation_id: conversationId,
      }),
    });

    console.log('[ChatAPI] Response status:', response.status);

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      console.error('[ChatAPI] Error response:', errorData);
      throw new Error(errorData.detail || `Chat request failed: ${response.status}`);
    }

    const data = await response.json();
    console.log('[ChatAPI] Success response received');
    return data;
  } catch (err) {
    console.error('[ChatAPI] Request failed:', err.message);
    throw err;
  }
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
