/**
 * Mock Chat API Contract
 *
 * This file defines the interface contract for the RAG chatbot backend API.
 * During UI development, MockChatAPI provides canned responses.
 * In production, a real implementation will replace this with actual RAG backend calls.
 *
 * Feature: 007-rag-chatbot-ui
 * Purpose: Define clear integration boundary between UI and future backend
 */

// ============================================================================
// Type Definitions
// ============================================================================

/**
 * Represents a chat message in the conversation history
 */
export interface ChatMessage {
  /** Role of the message sender */
  role: 'user' | 'assistant';

  /** Message content (supports markdown) */
  content: string;

  /** ISO 8601 timestamp when message was created */
  timestamp: string;
}

/**
 * Request payload for sending a message to the chatbot
 */
export interface SendMessageRequest {
  /** The user's current message */
  message: string;

  /** Full conversation history for context-aware responses */
  conversationHistory: ChatMessage[];

  /**
   * Optional context for page-aware or selection-aware responses
   * (Future enhancement: select-text-to-ask feature)
   */
  context?: {
    /** Current documentation page URL or path */
    currentPage?: string;

    /** User-selected text for contextual queries (future) */
    selectedText?: string;
  };
}

/**
 * Response payload from chatbot after processing a message
 */
export interface SendMessageResponse {
  /** Assistant's reply (markdown-formatted) */
  reply: string;

  /** Unique identifier for this message */
  messageId: string;

  /** ISO 8601 timestamp when response was generated */
  timestamp: string;
}

/**
 * Chat API interface contract
 *
 * Future RAG backend implementation must conform to this interface.
 */
export interface ChatAPI {
  /**
   * Send a message to the chatbot and receive a response
   *
   * @param request - Message and conversation context
   * @returns Promise resolving to assistant's response
   * @throws Error if API call fails
   */
  sendMessage(request: SendMessageRequest): Promise<SendMessageResponse>;
}

// ============================================================================
// Mock Implementation (for UI Development)
// ============================================================================

/**
 * Mock chatbot API for UI development and testing
 *
 * Returns canned responses based on keyword detection.
 * Simulates 500-1500ms network delay for realistic UX testing.
 *
 * **Replace this with real RAG backend implementation in production.**
 */
export class MockChatAPI implements ChatAPI {
  /**
   * Simulated network delay range (milliseconds)
   */
  private readonly MIN_DELAY_MS = 500;
  private readonly MAX_DELAY_MS = 1500;

  /**
   * Send a message and receive a mock response
   */
  async sendMessage(request: SendMessageRequest): Promise<SendMessageResponse> {
    // Simulate network latency
    const delay = Math.random() * (this.MAX_DELAY_MS - this.MIN_DELAY_MS) + this.MIN_DELAY_MS;
    await new Promise(resolve => setTimeout(resolve, delay));

    // Generate response based on keywords
    const reply = this.generateResponse(request.message);

    return {
      reply,
      messageId: `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date().toISOString(),
    };
  }

  /**
   * Generate canned response based on message content
   *
   * @param message - User's message
   * @returns Markdown-formatted response
   */
  private generateResponse(message: string): string {
    const lowerMessage = message.toLowerCase();

    // VLA (Vision-Language-Action) keyword
    if (lowerMessage.includes('vla')) {
      return `**Vision-Language-Action (VLA)** systems combine computer vision, natural language understanding, and robotic action planning.

They enable robots to:
- **Perceive** their environment using cameras and sensors
- **Interpret** human instructions in natural language
- **Execute** physical tasks through coordinated motor control

VLA represents the convergence of AI perception, language models, and embodied intelligence.`;
    }

    // SLAM keyword
    if (lowerMessage.includes('slam')) {
      return `**SLAM (Simultaneous Localization and Mapping)** is a fundamental technique in mobile robotics.

It allows robots to:
1. Build a map of an unknown environment
2. Track their own location within that map
3. Navigate autonomously without prior knowledge

SLAM is essential for applications like autonomous vehicles, robotic vacuum cleaners, and warehouse automation.`;
    }

    // ROS 2 keyword
    if (lowerMessage.includes('ros') || lowerMessage.includes('robot operating system')) {
      return `**ROS 2 (Robot Operating System 2)** is a flexible framework for writing robot software.

Key features:
- **Nodes**: Modular processes that perform specific tasks
- **Topics**: Publish-subscribe messaging for sensor data and commands
- **Services**: Request-response patterns for synchronous operations
- **Actions**: Long-running tasks with feedback and cancellation support

ROS 2 powers everything from hobby robots to industrial automation systems.`;
    }

    // Digital Twin keyword
    if (lowerMessage.includes('digital twin') || lowerMessage.includes('simulation')) {
      return `**Digital Twins** are virtual replicas of physical robots used for simulation and testing.

Benefits:
- **Safe Testing**: Experiment with behaviors without risking hardware damage
- **Rapid Iteration**: Test changes in simulation before deploying to real robots
- **Physics Accuracy**: Realistic collision detection, gravity, and dynamics

Tools like **Gazebo** and **NVIDIA Isaac Sim** enable high-fidelity digital twin environments.`;
    }

    // NVIDIA Isaac keyword
    if (lowerMessage.includes('isaac') || lowerMessage.includes('nvidia')) {
      return `**NVIDIA Isaac Platform** provides tools for building AI-powered robots.

Components:
- **Isaac Sim**: High-fidelity physics simulator powered by Omniverse
- **Isaac SDK**: Libraries for perception, navigation, and manipulation
- **Isaac ROS**: ROS 2 packages optimized for NVIDIA hardware

Isaac accelerates development with GPU-accelerated compute and realistic simulation.`;
    }

    // Humanoid keyword
    if (lowerMessage.includes('humanoid')) {
      return `**Humanoid robots** are designed to mimic human form and capabilities.

Challenges:
- **Balance**: Maintaining stability on two legs (bipedal locomotion)
- **Dexterity**: Fine motor control for hand manipulation
- **Human-Robot Interaction**: Natural communication and collaboration

Examples: Boston Dynamics Atlas, Tesla Optimus, Agility Robotics Digit.`;
    }

    // Default fallback response
    return `I'm a mock chatbot for UI development. In production, I'll be powered by a RAG (Retrieval-Augmented Generation) system that retrieves answers from the **Physical AI & Humanoid Robotics** book content.

Try asking me about:
- **VLA** (Vision-Language-Action systems)
- **SLAM** (Simultaneous Localization and Mapping)
- **ROS 2** (Robot Operating System)
- **Digital Twins** and simulation
- **NVIDIA Isaac** platform
- **Humanoid robots**

Your question: "${message}"

(In production, this would search the book's vector database and generate a contextual answer based on relevant chapters.)`;
  }
}

// ============================================================================
// Singleton Instance (for convenience)
// ============================================================================

/**
 * Default mock API instance for use throughout the application
 *
 * Usage:
 * ```typescript
 * import { mockChatAPI } from './contracts/mock-api';
 *
 * const response = await mockChatAPI.sendMessage({
 *   message: 'What is VLA?',
 *   conversationHistory: [],
 * });
 * ```
 */
export const mockChatAPI = new MockChatAPI();

// ============================================================================
// Production Integration Notes
// ============================================================================

/**
 * PRODUCTION REPLACEMENT GUIDE
 *
 * To integrate with a real RAG backend:
 *
 * 1. Create a new file: `services/ragChatService.ts`
 *
 * 2. Implement ChatAPI interface:
 *    ```typescript
 *    import { ChatAPI, SendMessageRequest, SendMessageResponse } from './contracts/mock-api';
 *
 *    class RAGChatAPI implements ChatAPI {
 *      private readonly apiBaseUrl: string;
 *
 *      constructor(baseUrl: string) {
 *        this.apiBaseUrl = baseUrl;
 *      }
 *
 *      async sendMessage(request: SendMessageRequest): Promise<SendMessageResponse> {
 *        const response = await fetch(`${this.apiBaseUrl}/chat`, {
 *          method: 'POST',
 *          headers: { 'Content-Type': 'application/json' },
 *          body: JSON.stringify(request),
 *        });
 *
 *        if (!response.ok) {
 *          throw new Error(`API error: ${response.statusText}`);
 *        }
 *
 *        return response.json();
 *      }
 *    }
 *
 *    export const ragChatAPI = new RAGChatAPI(process.env.REACT_APP_RAG_API_URL);
 *    ```
 *
 * 3. Update imports in `hooks/useChatState.ts`:
 *    - Replace: `import { mockChatAPI } from '../services/mockChatService';`
 *    - With: `import { ragChatAPI as chatAPI } from '../services/ragChatService';`
 *
 * 4. Test integration:
 *    - Verify conversation history is passed correctly
 *    - Check markdown rendering of real responses
 *    - Validate error handling for network failures
 *
 * 5. Environment variables (.env):
 *    ```
 *    REACT_APP_RAG_API_URL=https://api.example.com/rag
 *    ```
 */
