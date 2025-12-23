/**
 * Mock Chat Service
 *
 * Provides canned responses for UI development and testing.
 * Based on contracts/mock-api.ts with keyword-based response generation.
 *
 * Feature: 007-rag-chatbot-ui
 * Based on: contracts/mock-api.ts
 *
 * PRODUCTION NOTE: Replace this service with a real RAG backend implementation.
 * See contracts/mock-api.ts for integration guide.
 */

/**
 * Chat message in conversation history
 */
export interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
}

/**
 * Request payload for sending a message
 */
export interface SendMessageRequest {
  message: string;
  conversationHistory: ChatMessage[];
  context?: {
    currentPage?: string;
    selectedText?: string;
  };
}

/**
 * Response payload from chatbot
 */
export interface SendMessageResponse {
  reply: string;
  messageId: string;
  timestamp: string;
}

/**
 * Chat API interface contract
 */
export interface ChatAPI {
  sendMessage(request: SendMessageRequest): Promise<SendMessageResponse>;
}

/**
 * Mock chatbot API for UI development
 *
 * Simulates 500-1500ms network delay and returns keyword-based responses.
 */
class MockChatAPI implements ChatAPI {
  private readonly MIN_DELAY_MS = 500;
  private readonly MAX_DELAY_MS = 1500;

  async sendMessage(request: SendMessageRequest): Promise<SendMessageResponse> {
    // Simulate network latency
    const delay = Math.random() * (this.MAX_DELAY_MS - this.MIN_DELAY_MS) + this.MIN_DELAY_MS;
    await new Promise(resolve => setTimeout(resolve, delay));

    // Generate response based on keywords
    const reply = this.generateResponse(request.message);

    return {
      reply,
      messageId: `msg-${Date.now()}-${Math.random().toString(36).substring(2, 11)}`,
      timestamp: new Date().toISOString(),
    };
  }

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

    // Navigation keyword
    if (lowerMessage.includes('navigation') || lowerMessage.includes('path planning')) {
      return `**Robot Navigation** involves planning and executing paths through an environment.

Key components:
- **Global Planning**: Finding optimal routes using maps (A*, Dijkstra)
- **Local Planning**: Avoiding dynamic obstacles in real-time
- **Localization**: Determining the robot's position (GPS, SLAM, sensors)

Navigation stacks like Nav2 (ROS 2) provide ready-to-use solutions.`;
    }

    // Perception keyword
    if (lowerMessage.includes('perception') || lowerMessage.includes('sensor')) {
      return `**Robot Perception** enables machines to understand their environment.

Common sensors:
- **LiDAR**: 3D point clouds for mapping and obstacle detection
- **Cameras**: Visual recognition, depth estimation (RGB-D)
- **IMU**: Acceleration and orientation tracking
- **Force/Torque**: Contact and manipulation sensing

Modern perception uses deep learning for object detection and scene understanding.`;
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
- **Navigation** and path planning
- **Perception** and sensors

Your question: "${message}"

*(In production, this would search the book's vector database and generate a contextual answer based on relevant chapters.)*`;
  }
}

/**
 * Singleton mock API instance
 *
 * Usage:
 * ```typescript
 * import { mockChatAPI } from '../services/mockChatService';
 *
 * const response = await mockChatAPI.sendMessage({
 *   message: 'What is VLA?',
 *   conversationHistory: [],
 * });
 * ```
 */
export const mockChatAPI = new MockChatAPI();
