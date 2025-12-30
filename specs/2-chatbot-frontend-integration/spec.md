# Feature Specification: Chatbot Frontend Integration

**Feature ID**: `2-chatbot-frontend-integration`
**Priority**: P1 (High)
**Status**: Planning
**Created**: 2025-12-27
**Owner**: Development Team

---

## 1. Overview

### 1.1 Feature Summary

Integrate the RAG chatbot system into the Docusaurus-based Physical AI & Humanoid Robotics book website. This feature provides users with an interactive, context-aware chatbot embedded directly in the documentation site, enabling them to ask questions about the book content and receive grounded, citation-backed answers.

### 1.2 User Stories

**US1: As a reader**, I want to access a chatbot widget on any book page, so I can quickly ask questions about the content I'm reading without leaving the page.

**US2: As a reader**, I want the chatbot to remember my conversation history, so I can ask follow-up questions without repeating context.

**US3: As a reader**, I want to see source citations in chatbot responses, so I can verify the information and read more in the referenced sections.

**US4: As a reader**, I want the chatbot to be mobile-responsive, so I can use it effectively on my phone or tablet.

**US5: As a developer**, I want a REST API layer between the frontend and Python backend, so the system is scalable and maintainable.

### 1.3 Success Criteria

- ✓ Chatbot widget accessible from all documentation pages
- ✓ Sub-2 second response time for typical queries
- ✓ Session persistence across page navigation
- ✓ Mobile-responsive UI (works on screens ≥320px width)
- ✓ Accessibility compliance (WCAG 2.1 AA)
- ✓ 95% uptime for API endpoints
- ✓ Proper error handling with user-friendly messages

---

## 2. Technical Architecture

### 2.1 System Components

```
┌─────────────────────────────────────────────────────────────┐
│                     BROWSER (React/Docusaurus)              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Chatbot Widget Component                             │  │
│  │  - Message List                                       │  │
│  │  - Input Field                                        │  │
│  │  - Typing Indicator                                   │  │
│  │  - Source Citations                                   │  │
│  └──────────────────────────────────────────────────────┘  │
│                          ↓ ↑                                │
│              (Axios + React Query)                          │
└─────────────────────────────────────────────────────────────┘
                           ↓ ↑
                    HTTP/JSON API
                           ↓ ↑
┌─────────────────────────────────────────────────────────────┐
│              API LAYER (Node.js/Express/TypeScript)         │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  REST Endpoints                                       │  │
│  │  - POST /api/chat/query                              │  │
│  │  - GET  /api/chat/sessions/:id                       │  │
│  │  - POST /api/chat/sessions                           │  │
│  │  - DELETE /api/chat/sessions/:id                     │  │
│  └──────────────────────────────────────────────────────┘  │
│                          ↓ ↑                                │
│                  (Child Process Spawn)                      │
└─────────────────────────────────────────────────────────────┘
                           ↓ ↑
                    Python Subprocess
                           ↓ ↑
┌─────────────────────────────────────────────────────────────┐
│              BACKEND (Python RAG System)                    │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  RAG Pipeline                                         │  │
│  │  - VectorSearchEngine (Retrieval)                    │  │
│  │  - GeminiAnswerGenerator (Generation)                │  │
│  │  - SessionManager (Context Tracking)                 │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Technology Stack

**Frontend**:
- React 19 (already in package.json)
- TypeScript
- @tanstack/react-query (data fetching/caching)
- Axios (HTTP client)
- CSS Modules or Tailwind CSS (styling)

**API Layer**:
- Node.js 20+
- Express.js (REST API)
- TypeScript
- Prisma (optional for session persistence)
- Winston (logging)
- express-rate-limit (DOS protection)

**Backend** (already complete):
- Python 3.10+
- VectorSearchEngine, GeminiAnswerGenerator, SessionManager

### 2.3 Data Flow

1. **User Query**:
   ```
   User types question → Frontend validates input → POST /api/chat/query
   ```

2. **API Processing**:
   ```
   API receives query → Spawns Python process → Calls RAG pipeline
   ```

3. **Backend Processing**:
   ```
   Retrieve relevant chunks → Generate answer → Return ChatResponse
   ```

4. **Response Display**:
   ```
   API formats response → Returns JSON → Frontend renders message + sources
   ```

---

## 3. API Specification

### 3.1 Endpoints

#### POST /api/chat/query

**Purpose**: Submit a user query and receive a grounded answer

**Request**:
```json
{
  "query": "What are the main components of a humanoid robot?",
  "sessionId": "uuid-string" // optional, creates new if not provided
}
```

**Response** (200 OK):
```json
{
  "sessionId": "0cdb9765-412b-4b29-a883-be4c91bd3a58",
  "query": "What are the main components of a humanoid robot?",
  "answer": "Humanoid robots consist of...",
  "sources": [
    {
      "rank": 1,
      "section": "Chapter 2: Humanoid Robot Architecture",
      "url": "https://example.com/book/chapter2",
      "similarity_score": 0.708
    }
  ],
  "metadata": {
    "numSources": 2,
    "isGrounded": true,
    "hasCitations": true,
    "model": "gemini-2.5-flash",
    "timestamp": "2025-12-27T10:30:00Z"
  }
}
```

**Error Response** (400 Bad Request):
```json
{
  "error": "Invalid request",
  "message": "Query is required and must be a non-empty string",
  "code": "INVALID_QUERY"
}
```

**Error Response** (500 Internal Server Error):
```json
{
  "error": "Server error",
  "message": "Failed to generate response",
  "code": "RAG_PIPELINE_ERROR"
}
```

#### POST /api/chat/sessions

**Purpose**: Create a new chat session

**Response** (201 Created):
```json
{
  "sessionId": "uuid-string",
  "createdAt": "2025-12-27T10:30:00Z"
}
```

#### GET /api/chat/sessions/:id

**Purpose**: Get session history

**Response** (200 OK):
```json
{
  "sessionId": "uuid-string",
  "messages": [
    {
      "role": "user",
      "content": "What is Physical AI?",
      "timestamp": "2025-12-27T10:30:00Z"
    },
    {
      "role": "assistant",
      "content": "Physical AI represents...",
      "timestamp": "2025-12-27T10:30:05Z"
    }
  ],
  "createdAt": "2025-12-27T10:30:00Z",
  "lastActivity": "2025-12-27T10:30:05Z"
}
```

#### DELETE /api/chat/sessions/:id

**Purpose**: Clear session history

**Response** (204 No Content)

### 3.2 Rate Limiting

- 60 requests per minute per IP
- 429 status code when exceeded
- Retry-After header with cooldown time

### 3.3 Authentication

- Optional for MVP (public chatbot)
- Future: JWT-based auth for personalized features

---

## 4. Frontend Components

### 4.1 Component Hierarchy

```
<ChatbotWidget>
  ├── <ChatbotToggle />              // Floating button to open/close
  ├── <ChatbotWindow>
  │   ├── <ChatbotHeader />          // Title, minimize, close buttons
  │   ├── <MessageList>
  │   │   ├── <UserMessage />        // User's question
  │   │   ├── <AssistantMessage>    // Bot's response
  │   │   │   ├── <MessageText />   // Main answer text
  │   │   │   └── <SourcesList />   // Citations
  │   │   └── <TypingIndicator />   // Loading state
  │   ├── <InputArea>
  │   │   ├── <TextInput />         // Query input field
  │   │   └── <SendButton />        // Submit button
  │   └── <ChatbotFooter />         // Powered by info, clear chat
  └── <ErrorBoundary />             // Graceful error handling
```

### 4.2 Component Specifications

#### ChatbotWidget

**Props**:
- `position`: 'bottom-right' | 'bottom-left' (default: 'bottom-right')
- `theme`: 'light' | 'dark' | 'auto' (default: 'auto')
- `initialOpen`: boolean (default: false)

**State**:
- `isOpen`: boolean
- `sessionId`: string | null
- `messages`: Message[]
- `isLoading`: boolean

**Behavior**:
- Persists session ID in localStorage
- Auto-scrolls to latest message
- Handles keyboard shortcuts (Esc to close)

#### MessageList

**Features**:
- Auto-scroll to bottom on new message
- Timestamp display
- Markdown support for formatting
- Code syntax highlighting

#### SourcesList

**Display**:
- Expandable/collapsible citations
- Clickable links to source pages
- Similarity score indicators

### 4.3 Styling Requirements

- Mobile-first responsive design
- Minimum tap target size: 44x44px
- Color contrast ratio ≥ 4.5:1 (WCAG AA)
- Smooth animations (< 200ms)
- Dark mode support
- RTL language support

---

## 5. Implementation Phases

### Phase 1: Setup & Planning (Current)
- ✓ Create spec document
- ✓ Define API contracts
- ✓ Plan component hierarchy

### Phase 2: API Layer Development
- Create Express routes for chatbot endpoints
- Implement Python subprocess spawning
- Add error handling and logging
- Write API tests

### Phase 3: Frontend Components
- Build ChatbotWidget component
- Implement message rendering
- Add input handling and validation
- Style for mobile and desktop

### Phase 4: Integration & Testing
- Connect frontend to API
- Add React Query data fetching
- Implement session persistence
- E2E testing with Playwright

### Phase 5: Polish & Deployment
- Performance optimization
- Accessibility audit
- Documentation
- Deploy to production

---

## 6. Non-Functional Requirements

### 6.1 Performance

- Initial load time: < 2 seconds
- Query response time: < 3 seconds (p95)
- Widget open/close animation: < 200ms
- Message rendering: < 100ms per message

### 6.2 Security

- Input sanitization (XSS prevention)
- Rate limiting (60 req/min per IP)
- CORS configuration (whitelist domains)
- Content Security Policy headers

### 6.3 Accessibility

- ARIA labels on all interactive elements
- Keyboard navigation support
- Screen reader compatibility
- Focus management

### 6.4 Scalability

- API supports 100 concurrent users
- Horizontal scaling via load balancer
- Session state in Redis (future)
- CDN for static assets

---

## 7. Error Handling

### 7.1 Error Categories

**User Errors**:
- Empty query
- Query too long (> 500 chars)
- Rate limit exceeded

**System Errors**:
- API unavailable
- Python backend timeout
- Qdrant connection failed
- Gemini API error

### 7.2 Error Messages

- User-friendly language
- No technical jargon
- Actionable suggestions
- Retry buttons where appropriate

---

## 8. Testing Strategy

### 8.1 Unit Tests

- React components with Jest + React Testing Library
- API routes with Supertest
- Python integration (already done)

### 8.2 Integration Tests

- Frontend → API → Backend flow
- Session persistence
- Error propagation

### 8.3 E2E Tests

- User query flow (Playwright)
- Multi-turn conversations
- Source citation clicks
- Mobile viewport testing

---

## 9. Monitoring & Analytics

### 9.1 Metrics to Track

- Query volume (per hour/day)
- Average response time
- Error rate
- User engagement (avg messages per session)
- Most common queries

### 9.2 Logging

- API request/response logs (Winston)
- Python subprocess logs
- Frontend error logs (Sentry optional)

---

## 10. Future Enhancements

- User authentication for personalized history
- Multi-language support (i18n)
- Voice input/output
- Conversation export (PDF, markdown)
- Admin dashboard for analytics
- Fine-tune model on user feedback

---

## 11. Dependencies

**Hard Dependencies**:
- Backend RAG system (✓ Complete)
- Node.js 20+ runtime
- React 19+ in Docusaurus
- PostgreSQL for session persistence (optional)

**Soft Dependencies**:
- Redis for session caching (future)
- Sentry for error tracking (future)

---

## 12. Risks & Mitigation

| Risk | Impact | Probability | Mitigation |
|------|--------|------------|------------|
| Python subprocess crashes | High | Low | Implement retry logic, health checks |
| Slow API response | Medium | Medium | Add caching, optimize queries |
| Mobile UI issues | Medium | Low | Thorough mobile testing |
| Rate limit too strict | Low | Medium | Monitor usage, adjust limits |

---

## 13. Acceptance Criteria

### Definition of Done

- [ ] API endpoints implemented and tested
- [ ] Chatbot widget functional on all pages
- [ ] Mobile-responsive (tested on 3 devices)
- [ ] Accessibility audit passed
- [ ] Unit tests >80% coverage
- [ ] E2E tests for critical flows
- [ ] Documentation complete
- [ ] Deployed to staging environment

---

**Status**: Ready for Implementation
**Next Step**: Begin Phase 2 - API Layer Development
