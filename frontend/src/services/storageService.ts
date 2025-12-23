/**
 * Storage Service
 *
 * Provides conversation persistence with session storage and in-memory fallback.
 * Implements graceful degradation for private browsing and quota exceeded scenarios.
 *
 * Feature: 007-rag-chatbot-ui
 * Based on: research.md (Task 4), data-model.md
 * Requirements: FR-025, FR-026, FR-027
 */

import type { ConversationState, StorageType } from '../types/chatState';
import type { Message } from '../types/message';

const STORAGE_KEY = 'chatbot_conversation';

/**
 * Storage service interface for conversation persistence
 */
export interface IStorageService {
  getConversation(): ConversationState | null;
  saveConversation(state: ConversationState): void;
  clearConversation(): void;
  getStorageType(): StorageType;
}

/**
 * In-memory storage adapter for fallback scenarios
 */
class InMemoryStorageAdapter implements IStorageService {
  private store: Map<string, ConversationState> = new Map();

  getConversation(): ConversationState | null {
    return this.store.get(STORAGE_KEY) || null;
  }

  saveConversation(state: ConversationState): void {
    this.store.set(STORAGE_KEY, state);
  }

  clearConversation(): void {
    this.store.delete(STORAGE_KEY);
  }

  getStorageType(): StorageType {
    return 'memory';
  }
}

/**
 * Session storage adapter with quota handling
 */
class SessionStorageAdapter implements IStorageService {
  private fallbackAdapter: InMemoryStorageAdapter | null = null;
  private currentStorageType: StorageType = 'session';

  getConversation(): ConversationState | null {
    if (this.currentStorageType === 'memory' && this.fallbackAdapter) {
      return this.fallbackAdapter.getConversation();
    }

    try {
      const stored = sessionStorage.getItem(STORAGE_KEY);
      if (!stored) return null;

      const parsed = JSON.parse(stored);

      // Reconstruct Date objects (JSON.parse converts to strings)
      if (parsed.messages && Array.isArray(parsed.messages)) {
        parsed.messages = parsed.messages.map((msg: Message & { timestamp: string | Date }) => ({
          ...msg,
          timestamp: new Date(msg.timestamp),
        }));
      }

      return parsed as ConversationState;
    } catch (e) {
      console.error('[Storage] Failed to retrieve conversation', e);
      return null;
    }
  }

  saveConversation(state: ConversationState): void {
    if (this.currentStorageType === 'memory' && this.fallbackAdapter) {
      this.fallbackAdapter.saveConversation(state);
      return;
    }

    try {
      sessionStorage.setItem(STORAGE_KEY, JSON.stringify(state));
    } catch (e) {
      // Quota exceeded mid-session, switch to memory (FR-026)
      console.warn('[Storage] Switching to in-memory storage due to quota exceeded');
      this.currentStorageType = 'memory';
      this.fallbackAdapter = new InMemoryStorageAdapter();
      this.fallbackAdapter.saveConversation(state);
    }
  }

  clearConversation(): void {
    if (this.currentStorageType === 'memory' && this.fallbackAdapter) {
      this.fallbackAdapter.clearConversation();
      return;
    }

    try {
      sessionStorage.removeItem(STORAGE_KEY);
    } catch (e) {
      console.error('[Storage] Failed to clear conversation', e);
    }
  }

  getStorageType(): StorageType {
    return this.currentStorageType;
  }
}

/**
 * Detect session storage availability
 * Tests write capability to handle private browsing and quota scenarios
 */
function detectStorageAvailability(): boolean {
  if (typeof window === 'undefined' || !window.sessionStorage) {
    return false; // SSR or very old browser
  }

  try {
    const testKey = '__chatbot_storage_test__';
    sessionStorage.setItem(testKey, 'test');
    sessionStorage.removeItem(testKey);
    return true;
  } catch (e) {
    console.warn('[Storage] Session storage unavailable:', (e as Error).name);
    return false; // Private browsing or quota exceeded
  }
}

/**
 * Create appropriate storage service based on availability
 */
function createStorageService(): IStorageService {
  if (detectStorageAvailability()) {
    return new SessionStorageAdapter();
  } else {
    console.warn('[Storage] Using in-memory fallback (private browsing detected)');
    return new InMemoryStorageAdapter();
  }
}

/**
 * Singleton storage service instance
 *
 * Usage:
 * ```typescript
 * import { storageService } from '../services/storageService';
 *
 * const state = storageService.getConversation();
 * storageService.saveConversation(newState);
 * ```
 */
export const storageService = createStorageService();

/**
 * Re-export for testing and dependency injection
 */
export { createStorageService, detectStorageAvailability };
