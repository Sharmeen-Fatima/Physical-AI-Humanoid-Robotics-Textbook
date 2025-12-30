# Tasks: Chatbot Frontend Integration

**Feature**: `2-chatbot-frontend-integration`
**Branch**: `2-chatbot-frontend-integration`
**Status**: Planning → Implementation
**Created**: 2025-12-27

---

## Overview

This document breaks down the Chatbot Frontend Integration into actionable, independently testable tasks organized by phase. Each phase delivers a working increment that can be validated independently.

**Total Tasks**: 35 tasks across 5 phases
**Estimated Timeline**: 2-3 weeks (based on 1-2 developers)

---

## Task Organization

- **Phase 1**: Planning & Setup (T001-T005) - CURRENT
- **Phase 2**: API Layer Development (T006-T015)
- **Phase 3**: Frontend Components (T016-T025)
- **Phase 4**: Integration & State Management (T026-T030)
- **Phase 5**: Testing, Polish & Deployment (T031-T035)

---

## Phase 1: Planning & Setup (COMPLETED)

**Goal**: Define requirements, create spec, and set up project structure.

**Independent Test**: Review spec document and confirm all stakeholders aligned.

### Tasks

- [x] T001 Create feature specification document (specs/2-chatbot-frontend-integration/spec.md)
- [x] T002 Define API contracts and endpoints
- [x] T003 Create component hierarchy diagram
- [x] T004 Create task breakdown document (this file)
- [ ] T005 Set up development branch and initial PR

---

## Phase 2: API Layer Development

**Goal**: Create REST API endpoints that bridge frontend React app to Python backend.

**Independent Test**: Use Postman/curl to test all endpoints return correct responses.

**Dependencies**: Python RAG backend (✓ Complete)

### Tasks

#### API Routes (T006-T009)

- [ ] T006 [P] Create Express route: POST /api/chat/query
  - Accept query and optional sessionId
  - Spawn Python subprocess with RAG pipeline
  - Return ChatResponse JSON
  - Test: curl POST with sample query

- [ ] T007 Create Express route: POST /api/chat/sessions
  - Generate new session ID (uuid)
  - Return session metadata
  - Test: curl POST to create session

- [ ] T008 Create Express route: GET /api/chat/sessions/:id
  - Fetch session history from Python SessionManager
  - Return message array
  - Test: curl GET with valid session ID

- [ ] T009 Create Express route: DELETE /api/chat/sessions/:id
  - Clear session in Python SessionManager
  - Return 204 No Content
  - Test: curl DELETE with valid session ID

#### Python Integration (T010-T012)

- [ ] T010 [P] Create Python bridge service (api/src/services/pythonBridge.ts)
  - Implement spawn() to call Python RAG pipeline
  - Handle stdout/stderr parsing
  - Implement timeout and error handling
  - Test: Call RAG pipeline with sample query

- [ ] T011 Create Python CLI wrapper script (backend/api_wrapper.py)
  - Accept JSON input from stdin
  - Call RAG components (VectorSearchEngine, GeminiAnswerGenerator)
  - Output JSON response to stdout
  - Test: echo query JSON | python api_wrapper.py

- [ ] T012 Add retry logic for Python subprocess failures
  - 3 retries with exponential backoff
  - Log failures with Winston
  - Test: Simulate Python crash, verify retry

#### Middleware & Error Handling (T013-T015)

- [ ] T013 [P] Add express-rate-limit middleware
  - 60 requests per minute per IP
  - Return 429 with Retry-After header
  - Test: Send 61 requests in 60s, verify 429

- [ ] T014 Create error handling middleware (api/src/middleware/errorHandler.ts)
  - Catch async errors
  - Format user-friendly error messages
  - Log errors with Winston
  - Test: Trigger error, verify response format

- [ ] T015 Add request/response logging middleware
  - Log method, path, status code, duration
  - Include request ID for tracing
  - Test: Make request, verify log entry

---

## Phase 3: Frontend Components

**Goal**: Build React components for chatbot UI.

**Independent Test**: Render components in Storybook/dev mode with mock data.

**Dependencies**: API endpoints (Phase 2)

### Tasks

#### Core Components (T016-T020)

- [ ] T016 [P] Create ChatbotWidget component (src/components/Chatbot/ChatbotWidget.tsx)
  - Floating toggle button (bottom-right)
  - Open/close state management
  - Position props (bottom-right, bottom-left)
  - Theme props (light, dark, auto)
  - Test: Render with props, verify toggle works

- [ ] T017 Create ChatbotWindow component (src/components/Chatbot/ChatbotWindow.tsx)
  - Header with title and close button
  - Main message area
  - Input area at bottom
  - Footer with clear chat button
  - Test: Render with mock messages

- [ ] T018 [P] Create MessageList component (src/components/Chatbot/MessageList.tsx)
  - Auto-scroll to latest message
  - Render UserMessage and AssistantMessage
  - Timestamp display
  - Loading indicator
  - Test: Render 10 messages, verify scroll

- [ ] T019 Create UserMessage component (src/components/Chatbot/UserMessage.tsx)
  - Display user query text
  - Timestamp
  - Avatar/icon
  - Test: Render with sample text

- [ ] T020 Create AssistantMessage component (src/components/Chatbot/AssistantMessage.tsx)
  - Display answer text with markdown support
  - Render SourcesList
  - Timestamp
  - Avatar/icon
  - Test: Render with mock ChatResponse

#### UI Elements (T021-T025)

- [ ] T021 Create SourcesList component (src/components/Chatbot/SourcesList.tsx)
  - Expandable/collapsible citations
  - Display source rank, section, URL, score
  - Clickable links
  - Test: Render 3 sources, verify expand/collapse

- [ ] T022 Create InputArea component (src/components/Chatbot/InputArea.tsx)
  - Text input field (auto-resize)
  - Send button
  - Character counter (max 500)
  - Enter key to send
  - Disabled state during loading
  - Test: Type message, verify send

- [ ] T023 Create TypingIndicator component (src/components/Chatbot/TypingIndicator.tsx)
  - Animated dots
  - "Assistant is typing..." text
  - Test: Render, verify animation

- [ ] T024 [P] Create ChatbotStyles.module.css
  - Mobile-first responsive styles
  - Dark mode support
  - Smooth animations
  - Accessibility (focus states)
  - Test: Visual regression testing

- [ ] T025 Create ErrorMessage component (src/components/Chatbot/ErrorMessage.tsx)
  - Display error with icon
  - Retry button
  - Dismiss button
  - Test: Render with sample error

---

## Phase 4: Integration & State Management

**Goal**: Connect frontend components to API, manage state with React Query.

**Independent Test**: Full user flow from query to response with real API.

**Dependencies**: API Layer (Phase 2), Frontend Components (Phase 3)

### Tasks

#### API Integration (T026-T028)

- [ ] T026 [P] Create API client (src/api/chatbotApi.ts)
  - Use Axios for HTTP requests
  - Implement sendQuery(query, sessionId)
  - Implement createSession()
  - Implement getSession(id)
  - Implement deleteSession(id)
  - Test: Call each method with mock server

- [ ] T027 [P] Set up React Query hooks (src/hooks/useChatbot.ts)
  - useSendQuery mutation
  - useSession query
  - useCreateSession mutation
  - Optimistic updates
  - Error handling
  - Test: Render hook in test component

- [ ] T028 Integrate API client with ChatbotWidget
  - Call sendQuery on form submit
  - Update messages array on response
  - Handle loading and error states
  - Test: Send query, verify response rendered

#### Session Management (T029-T030)

- [ ] T029 Implement session persistence (localStorage)
  - Save sessionId to localStorage on create
  - Load sessionId on component mount
  - Clear sessionId on session delete
  - Test: Refresh page, verify session restored

- [ ] T030 Add conversation history loading
  - Fetch messages on session load
  - Display historical messages
  - Auto-scroll to bottom
  - Test: Reload with existing session, verify history

---

## Phase 5: Testing, Polish & Deployment

**Goal**: Comprehensive testing, performance optimization, and production deployment.

**Independent Test**: All tests passing, accessibility audit passed, deployed to staging.

**Dependencies**: Phases 2-4 complete

### Tasks

#### Testing (T031-T032)

- [ ] T031 [P] Write unit tests for components
  - Jest + React Testing Library
  - Test user interactions (click, type, submit)
  - Test loading and error states
  - Target: >80% coverage
  - Test: npm run test

- [ ] T032 Write E2E tests (e2e/chatbot.spec.ts)
  - Playwright tests
  - User query flow
  - Multi-turn conversation
  - Source citation clicks
  - Mobile viewport
  - Test: npm run e2e

#### Polish (T033-T034)

- [ ] T033 Accessibility audit
  - Run axe DevTools
  - Test keyboard navigation
  - Test screen reader (NVDA/JAWS)
  - Fix ARIA labels
  - Target: WCAG 2.1 AA compliance
  - Test: npm run a11y-audit

- [ ] T034 Performance optimization
  - Lazy load chatbot widget
  - Code splitting
  - Optimize bundle size
  - Add service worker caching
  - Target: <2s initial load
  - Test: Lighthouse audit >90 score

#### Deployment (T035)

- [ ] T035 Deploy to production
  - Build frontend (npm run build)
  - Build API (npm run build in api/)
  - Deploy to hosting (Vercel/Netlify)
  - Set up environment variables
  - Configure CORS for production domain
  - Test: Smoke test on production URL

---

## Dependency Graph

```
Phase 1 (Planning) → COMPLETE
    ↓
Phase 2 (API Layer)
  ├─ T006-T009 (Routes) can run in parallel
  ├─ T010-T012 (Python Bridge) can run in parallel
  └─ T013-T015 (Middleware) depends on T006
    ↓
Phase 3 (Frontend Components)
  ├─ T016-T020 (Core Components) can run in parallel
  └─ T021-T025 (UI Elements) depends on T016-T020
    ↓
Phase 4 (Integration)
  ├─ T026-T027 (API Client) can run in parallel
  ├─ T028 depends on T026-T027
  └─ T029-T030 (Session) depends on T028
    ↓
Phase 5 (Testing & Deployment)
  ├─ T031-T034 can run in parallel
  └─ T035 depends on T031-T034
```

---

## Parallel Work Opportunities

Tasks marked with `[P]` can be worked on in parallel:
- T006-T009: API routes (4 developers)
- T010-T012: Python integration (1 developer)
- T016-T020: Core components (3 developers)
- T026-T027: API client (2 developers)
- T031-T034: Testing and polish (4 developers)

---

## Testing Checkpoints

### Checkpoint 1: Phase 2 Complete
- ✓ All API endpoints respond correctly
- ✓ Python subprocess spawns successfully
- ✓ Rate limiting works
- ✓ Errors logged properly

### Checkpoint 2: Phase 3 Complete
- ✓ All components render without errors
- ✓ Responsive on mobile (320px+)
- ✓ Dark mode works
- ✓ Accessibility basics (ARIA labels)

### Checkpoint 3: Phase 4 Complete
- ✓ User can send query and receive response
- ✓ Multi-turn conversation works
- ✓ Session persists across page refresh
- ✓ Sources are clickable

### Checkpoint 4: Phase 5 Complete
- ✓ All tests passing (unit + E2E)
- ✓ Lighthouse score >90
- ✓ Accessibility audit passed
- ✓ Deployed to production

---

## File Structure

```
Physical_AI_Humanoid_Robotics_Book/
├── api/
│   ├── src/
│   │   ├── routes/
│   │   │   └── chat.ts                  # T006-T009
│   │   ├── services/
│   │   │   └── pythonBridge.ts          # T010-T012
│   │   ├── middleware/
│   │   │   ├── rateLimit.ts             # T013
│   │   │   ├── errorHandler.ts          # T014
│   │   │   └── logger.ts                # T015
│   │   └── server.ts                    # Main Express app
│   └── tests/
│       └── chat.test.ts                 # API tests
├── backend/
│   └── api_wrapper.py                   # T011 (Python CLI wrapper)
├── src/
│   ├── components/
│   │   └── Chatbot/
│   │       ├── ChatbotWidget.tsx        # T016
│   │       ├── ChatbotWindow.tsx        # T017
│   │       ├── MessageList.tsx          # T018
│   │       ├── UserMessage.tsx          # T019
│   │       ├── AssistantMessage.tsx     # T020
│   │       ├── SourcesList.tsx          # T021
│   │       ├── InputArea.tsx            # T022
│   │       ├── TypingIndicator.tsx      # T023
│   │       ├── ErrorMessage.tsx         # T025
│   │       └── ChatbotStyles.module.css # T024
│   ├── api/
│   │   └── chatbotApi.ts                # T026
│   ├── hooks/
│   │   └── useChatbot.ts                # T027
│   └── types/
│       └── chatbot.ts                   # TypeScript interfaces
├── e2e/
│   └── chatbot.spec.ts                  # T032
└── specs/
    └── 2-chatbot-frontend-integration/
        ├── spec.md                      # T001
        └── tasks.md                     # T004 (this file)
```

---

## Success Metrics

| Phase | Metric | Target |
|-------|--------|--------|
| Phase 2 | API response time | <500ms p95 |
| Phase 2 | Error rate | <1% |
| Phase 3 | Component render time | <100ms |
| Phase 3 | Lighthouse Accessibility | >90 |
| Phase 4 | E2E query success rate | >95% |
| Phase 4 | Session persistence | 100% |
| Phase 5 | Test coverage | >80% |
| Phase 5 | Bundle size | <200KB gzipped |

---

## Next Steps

**Current Phase**: Phase 1 (Planning) - COMPLETE
**Next Phase**: Phase 2 (API Layer Development)

**Recommended Start**: Begin with T006 (POST /api/chat/query route) and T011 (Python wrapper) in parallel.

---

**Status**: Ready for Phase 2 Implementation
**Total Progress**: 4/35 tasks complete (11%)
