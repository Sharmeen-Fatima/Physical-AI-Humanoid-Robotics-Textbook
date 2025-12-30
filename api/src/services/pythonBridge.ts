/**
 * Python Bridge Service
 *
 * Spawns Python subprocess to interact with the RAG chatbot backend.
 * Handles JSON-based communication via stdin/stdout.
 */

import { spawn, ChildProcess } from 'child_process';
import { join } from 'path';
import logger from '../utils/logger';

/**
 * Input data for Python API wrapper
 */
export interface PythonInput {
  action: 'query' | 'create_session' | 'get_session' | 'delete_session';
  query?: string;
  sessionId?: string;
  topK?: number;
  threshold?: number;
}

/**
 * Response from Python API wrapper
 */
export interface PythonResponse {
  success: boolean;
  sessionId?: string;
  query?: string;
  answer?: string;
  sources?: Array<{
    rank: number;
    url: string;
    section: string;
    chunk_index: number;
    similarity_score: number;
  }>;
  metadata?: {
    numSources: number;
    isGrounded: boolean;
    hasCitations: boolean;
    model: string;
    temperature: number;
  };
  messages?: Array<{
    role: 'user' | 'assistant';
    content: string;
  }>;
  createdAt?: string;
  lastActivity?: string;
  numTurns?: number;
  deleted?: boolean;
  error?: string;
  code?: string;
}

/**
 * Python Bridge Service Class
 */
export class PythonBridge {
  private pythonPath: string;
  private wrapperPath: string;
  private timeout: number;

  constructor() {
    // Path to Python wrapper script
    this.pythonPath = 'python'; // Use system Python or configure virtualenv
    this.wrapperPath = join(__dirname, '..', '..', '..', 'backend', 'api_wrapper.py');
    this.timeout = 30000; // 30 seconds timeout
  }

  /**
   * Get the backend directory path (where .env and Python code live)
   */
  private getBackendDir(): string {
    return join(__dirname, '..', '..', '..', 'backend');
  }

  /**
   * Execute Python wrapper with JSON input
   *
   * @param input - Input data for Python wrapper
   * @returns Promise resolving to Python response
   */
  async execute(input: PythonInput): Promise<PythonResponse> {
    logger.info(`Executing Python wrapper: action=${input.action}`);

    return new Promise((resolve, reject) => {
      let stdout = '';
      let stderr = '';
      let timeoutId: NodeJS.Timeout;

      // Spawn Python process
      const pythonProcess: ChildProcess = spawn(this.pythonPath, [this.wrapperPath], {
        stdio: ['pipe', 'pipe', 'pipe'],
        env: process.env, // Inherit environment variables
        cwd: this.getBackendDir(), // Run from backend directory to access .env
      });

      // Setup timeout
      timeoutId = setTimeout(() => {
        pythonProcess.kill('SIGKILL');
        logger.error('Python process timeout');
        reject(new Error('Python process timeout (30s)'));
      }, this.timeout);

      // Collect stdout
      pythonProcess.stdout?.on('data', (data: Buffer) => {
        stdout += data.toString();
      });

      // Collect stderr
      pythonProcess.stderr?.on('data', (data: Buffer) => {
        stderr += data.toString();
      });

      // Handle process exit
      pythonProcess.on('close', (code: number | null) => {
        clearTimeout(timeoutId);

        // Log stderr (warnings, deprecation notices)
        if (stderr) {
          logger.debug(`Python stderr: ${stderr}`);
        }

        if (code !== 0) {
          logger.error(`Python process exited with code ${code}`);
          logger.error(`Stderr: ${stderr}`);
          logger.error(`Stdout: ${stdout}`);
          reject(new Error(`Python process failed with exit code ${code}`));
          return;
        }

        // Parse JSON output
        try {
          const response: PythonResponse = JSON.parse(stdout);
          logger.info(`Python response: success=${response.success}`);

          if (!response.success) {
            logger.warn(`Python returned error: ${response.error} (code: ${response.code})`);
          }

          resolve(response);
        } catch (err) {
          logger.error('Failed to parse Python JSON output');
          logger.error(`Stdout: ${stdout}`);
          reject(new Error(`Failed to parse Python JSON output: ${(err as Error).message}`));
        }
      });

      // Handle process error
      pythonProcess.on('error', (err: Error) => {
        clearTimeout(timeoutId);
        logger.error(`Python process error: ${err.message}`);
        reject(new Error(`Failed to spawn Python process: ${err.message}`));
      });

      // Send input JSON to stdin
      try {
        const inputJson = JSON.stringify(input);
        logger.debug(`Sending to Python: ${inputJson}`);
        pythonProcess.stdin?.write(inputJson);
        pythonProcess.stdin?.end();
      } catch (err) {
        clearTimeout(timeoutId);
        pythonProcess.kill();
        reject(new Error(`Failed to write to Python stdin: ${(err as Error).message}`));
      }
    });
  }

  /**
   * Send a query to the RAG chatbot
   *
   * @param query - User query string
   * @param sessionId - Optional session ID
   * @param options - Optional search parameters
   * @returns Promise resolving to chat response
   */
  async sendQuery(
    query: string,
    sessionId?: string,
    options?: { topK?: number; threshold?: number }
  ): Promise<PythonResponse> {
    const input: PythonInput = {
      action: 'query',
      query,
      sessionId,
      topK: options?.topK,
      threshold: options?.threshold,
    };

    return this.execute(input);
  }

  /**
   * Create a new chat session
   *
   * @returns Promise resolving to new session data
   */
  async createSession(): Promise<PythonResponse> {
    const input: PythonInput = {
      action: 'create_session',
    };

    return this.execute(input);
  }

  /**
   * Get session history
   *
   * @param sessionId - Session ID
   * @returns Promise resolving to session data
   */
  async getSession(sessionId: string): Promise<PythonResponse> {
    const input: PythonInput = {
      action: 'get_session',
      sessionId,
    };

    return this.execute(input);
  }

  /**
   * Delete a chat session
   *
   * @param sessionId - Session ID
   * @returns Promise resolving to deletion confirmation
   */
  async deleteSession(sessionId: string): Promise<PythonResponse> {
    const input: PythonInput = {
      action: 'delete_session',
      sessionId,
    };

    return this.execute(input);
  }
}

// Export singleton instance
export default new PythonBridge();
