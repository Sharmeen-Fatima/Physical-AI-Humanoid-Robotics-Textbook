# Tasks: RAG Chatbot Frontend Integration

**Input**: Design documents from `/specs/002-chatbot-frontend-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are NOT explicitly requested in spec.md, so test tasks are omitted per template instructions. However, test infrastructure setup is included for future use.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend API**: `api/src/` (Node.js/Express)
- **Frontend**: `src/` (Docusaurus + React components)
- **Python RAG**: `backend/` (existing, minimal changes)
- **Database**: `api/prisma/` (Prisma schema + migrations)
- **Tests**: `e2e/tests/` (Playwright E2E tests)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create API backend directory structure per plan.md: api/src/{routes,middleware,services,models,db,utils}, api/tests/{integration,unit}
- [X] T002 Initialize Node.js API project with TypeScript in api/package.json (express, jsonwebtoken, bcrypt, @prisma/client, axios, dotenv, winston, express-rate-limit, cookie-parser, cors)
- [X] T003 [P] Initialize TypeScript configuration in api/tsconfig.json with strict mode enabled
- [X] T004 [P] Create .env.example in api/ with DATABASE_URL, JWT_SECRET, PYTHON_RAG_URL, GOOGLE_TRANSLATE_API_KEY, NODE_ENV, PORT
- [X] T005 [P] Add .env to .gitignore and create api/.gitignore
- [X] T006 [P] Setup Docker Compose for PostgreSQL development database in docker-compose.yml at repository root
- [X] T007 [P] Create Prisma schema file in api/prisma/schema.prisma based on data-model.md (User, ChatSession, ChatMessage, TranslationCache)
- [X] T008 Install frontend dependencies: axios, react-query, i18next, react-i18next in package.json at repository root
- [X] T009 [P] Setup Playwright for E2E testing in e2e/playwright.config.ts

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T010 Generate Prisma Client and create initial migration in api/prisma/migrations/ based on schema.prisma
- [ ] T011 [P] Create base error classes in api/src/utils/errors.ts (ValidationError, UnauthorizedError, NotFoundError, ServiceUnavailableError)
- [ ] T012 [P] Setup Winston logger configuration in api/src/utils/logger.ts with console and file transports
- [ ] T013 [P] Create JWT authentication middleware in api/src/middleware/auth.ts for verifying HTTP-only cookies
- [ ] T014 [P] Create input validation middleware factory in api/src/middleware/validation.ts using Joi or Zod schemas
- [ ] T015 [P] Create rate limiting middleware in api/src/middleware/rateLimit.ts (30 requests/hour for chat)
- [ ] T016 Create Express server setup in api/src/server.ts with CORS, cookie-parser, JSON body parser, and global error handler
- [ ] T017 [P] Create minimal FastAPI wrapper in backend/api/server.py that exposes POST /query endpoint using existing RAG components
- [ ] T018 [P] Add CORS configuration to FastAPI server in backend/api/server.py to allow localhost:3000 and localhost:3001
- [ ] T019 [P] Setup i18next configuration in src/i18n/config.ts with English and Urdu locales
- [ ] T020 [P] Create AuthContext for global authentication state in src/contexts/AuthContext.tsx
- [ ] T021 [P] Create base API client with Axios in src/services/api.ts with withCredentials: true for cookie support
- [ ] T022 [P] Configure Docusaurus i18n for RTL support in docusaurus.config.ts: locales ['en', 'ur'], localeConfigs with ur: direction 'rtl'

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 2 - Create Account and Login (Priority: P2)

**Note**: Implementing P2 (authentication) BEFORE P1 (chatbot) to establish security foundation first, as recommended in plan.md

**Goal**: Enable users to create accounts, log in, and maintain authenticated sessions to access the chatbot feature

**Independent Test**: Can be tested by creating a new account through signup form, logging in with credentials, verifying session persistence, and logging out

### Implementation for User Story 2

- [ ] T023 [P] [US2] Create User model entity in api/src/models/User.ts with Prisma Client methods (create, findByEmail, findById)
- [ ] T024 [P] [US2] Create authentication service in api/src/services/authService.ts with signup, login, and verifyToken functions
- [ ] T025 [US2] Implement POST /api/auth/signup endpoint in api/src/routes/auth.ts per contracts/auth-api.yaml (email validation, password hashing with bcrypt, JWT issuance)
- [ ] T026 [US2] Implement POST /api/auth/login endpoint in api/src/routes/auth.ts per contracts/auth-api.yaml (credential verification, JWT in HTTP-only cookie)
- [ ] T027 [US2] Implement POST /api/auth/logout endpoint in api/src/routes/auth.ts per contracts/auth-api.yaml (clear auth cookie)
- [ ] T028 [US2] Implement GET /api/auth/me endpoint in api/src/routes/auth.ts per contracts/auth-api.yaml (return current user profile)
- [ ] T029 [P] [US2] Create SignupForm component in src/components/Auth/SignupForm.tsx with email and password fields, validation, and API integration
- [ ] T030 [P] [US2] Create LoginForm component in src/components/Auth/LoginForm.tsx with email and password fields and API integration
- [ ] T031 [US2] Create AuthModal component in src/components/Auth/AuthModal.tsx to switch between SignupForm and LoginForm
- [ ] T032 [US2] Implement useAuth custom hook in src/hooks/useAuth.ts for signup, login, logout, and current user state management
- [ ] T033 [US2] Create authApi service in src/services/authApi.ts with API calls for signup, login, logout, getCurrentUser
- [ ] T034 [US2] Add logout button to navigation bar (modify Docusaurus navbar config in docusaurus.config.ts or create custom navbar component)
- [ ] T035 [US2] Add session persistence logic to AuthContext using localStorage for token presence check and auto-login on page load

**Checkpoint**: At this point, User Story 2 (authentication) should be fully functional - users can signup, login, maintain sessions, and logout independently

---

## Phase 4: User Story 1 - Ask Questions About Book Content (Priority: P1) 🎯 MVP

**Goal**: Enable authenticated students to ask questions about the Physical AI textbook and receive accurate answers with clickable citations to relevant book sections

**Independent Test**: Can be fully tested by asking questions through the chatbot interface and verifying that responses include accurate citations from the book with clickable links. Delivers immediate value for learning and comprehension.

### Implementation for User Story 1

- [ ] T036 [P] [US1] Create ChatSession model entity in api/src/models/ChatSession.ts with Prisma Client methods (create, findByUserId, findById, updateTimestamp)
- [ ] T037 [P] [US1] Create ChatMessage model entity in api/src/models/ChatMessage.ts with Prisma Client methods (create, findBySessionId)
- [ ] T038 [US1] Create RAG proxy service in api/src/services/ragProxy.ts to forward questions to Python backend (POST to PYTHON_RAG_URL/query) with retry logic and timeout (30s)
- [ ] T039 [US1] Implement POST /api/chat/message endpoint in api/src/routes/chat.ts per contracts/chat-api.yaml (create session if needed, save user message, call RAG proxy, save assistant message with citations)
- [ ] T040 [US1] Implement GET /api/chat/history endpoint in api/src/routes/chat.ts per contracts/chat-api.yaml (retrieve user's sessions with pagination)
- [ ] T041 [US1] Implement GET /api/chat/session/:sessionId endpoint in api/src/routes/chat.ts per contracts/chat-api.yaml (retrieve specific session with messages)
- [ ] T042 [P] [US1] Create ChatButton component in src/components/ChatBot/ChatButton.tsx (floating button bottom-right, click to open chat panel)
- [ ] T043 [P] [US1] Create ChatPanel component in src/components/ChatBot/ChatPanel.tsx (expandable panel with header, message list, input area, close button)
- [ ] T044 [P] [US1] Create MessageList component in src/components/ChatBot/MessageList.tsx (display messages with role distinction, timestamps, scroll to bottom on new message)
- [ ] T045 [P] [US1] Create MessageInput component in src/components/ChatBot/MessageInput.tsx (multiline textarea, Shift+Enter for newline, Enter to send, 1000 char limit)
- [ ] T046 [P] [US1] Create TypingIndicator component in src/components/ChatBot/TypingIndicator.tsx (animated dots while waiting for response)
- [ ] T047 [P] [US1] Create CitationLink component in src/components/ChatBot/CitationLink.tsx (clickable link with section title, score display, navigation to book section)
- [ ] T048 [US1] Create main ChatBot wrapper component in src/components/ChatBot/index.tsx that manages state (isOpen, messages) and renders ChatButton or ChatPanel
- [ ] T049 [US1] Create useChat custom hook in src/hooks/useChat.ts for sending messages, managing chat state, and fetching history using React Query
- [ ] T050 [US1] Create chatApi service in src/services/chatApi.ts with API calls for sendMessage, getChatHistory, getSession
- [ ] T051 [US1] Swizzle Docusaurus Root component: create src/theme/Root.tsx and inject ChatBot component to appear on all pages
- [ ] T052 [P] [US1] Create chatbot CSS styles in src/components/ChatBot/styles.css (floating button position, panel layout, message styling, responsive design)
- [ ] T053 [US1] Add authentication gate to ChatBot component: show AuthModal if user not logged in when clicking chat button
- [ ] T054 [US1] Implement error handling in ChatBot: display user-friendly messages for RAG backend unavailable, rate limit exceeded, network errors
- [ ] T055 [US1] Add citation URL mapping logic in CitationLink to convert RAG backend section identifiers to actual Docusaurus URLs

**Checkpoint**: At this point, User Story 1 (chatbot Q&A) should be fully functional - authenticated users can ask questions and receive cited answers with clickable links

---

## Phase 5: User Story 3 - Translate Content to Urdu (Priority: P3)

**Goal**: Enable Urdu-speaking students to read the textbook and interact with the chatbot in their native language with proper RTL layout

**Independent Test**: Can be tested by toggling language to Urdu and verifying that UI elements, book content, and chatbot responses are translated with correct RTL formatting

### Implementation for User Story 3

- [ ] T056 [P] [US3] Create TranslationCache model entity in api/src/models/TranslationCache.ts with Prisma Client methods (findByHash, create, incrementHitCount)
- [ ] T057 [US3] Create translation service in api/src/services/translateService.ts with Google Translate API integration, SHA-256 hashing for cache lookup, and cache hit/miss logic
- [ ] T058 [US3] Implement POST /api/translate endpoint in api/src/routes/translate.ts per contracts/translation-api.yaml (single text translation with caching)
- [ ] T059 [US3] Implement POST /api/translate/batch endpoint in api/src/routes/translate.ts per contracts/translation-api.yaml (batch translation for multiple texts)
- [ ] T060 [P] [US3] Create LanguageToggle component in src/components/LanguageToggle/LanguageToggle.tsx (English/Urdu button in navbar)
- [ ] T061 [US3] Create useTranslation custom hook wrapper in src/hooks/useTranslation.ts that wraps i18next with additional API translation logic
- [ ] T062 [US3] Create translationApi service in src/services/translationApi.ts with API calls for translate and translateBatch
- [ ] T063 [P] [US3] Create English locale translation file in src/i18n/locales/en/translation.json with all UI strings (button labels, form fields, error messages)
- [ ] T064 [P] [US3] Create Urdu locale translation file in src/i18n/locales/ur/translation.json with pre-translated UI strings (static translations to avoid API costs)
- [ ] T065 [US3] Integrate LanguageToggle into Docusaurus navbar: modify docusaurus.config.ts to add custom navbar item or use Docusaurus localeDropdown
- [ ] T066 [US3] Add RTL layout CSS in src/styles/rtl.css using CSS logical properties (margin-inline-start, padding-inline-end, text-align: start)
- [ ] T067 [US3] Update ChatBot components to use i18next translations for all UI strings (button labels, placeholders, error messages)
- [ ] T068 [US3] Add dynamic translation for chatbot responses: when language is 'ur', call POST /api/translate before displaying assistant messages
- [ ] T069 [US3] Add language preference persistence: update user's languagePreference in database when toggling language via PUT /api/auth/profile endpoint (new endpoint)
- [ ] T070 [US3] Implement PUT /api/auth/profile endpoint in api/src/routes/auth.ts to update user languagePreference field
- [ ] T071 [US3] Test RTL layout with actual Urdu text: verify chat panel, message bubbles, citations, and input field display correctly with dir="rtl"

**Checkpoint**: All user stories should now be independently functional - users can signup/login (US2), ask questions with citations (US1), and toggle to Urdu with RTL layout (US3)

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and production readiness

- [ ] T072 [P] Create comprehensive README.md in api/ with setup instructions, environment variables, API endpoints, and deployment guide
- [ ] T073 [P] Create comprehensive README.md in repository root with quickstart instructions referencing quickstart.md
- [ ] T074 [P] Add error boundary component in src/components/ErrorBoundary.tsx to catch React errors and display fallback UI
- [ ] T075 [P] Add structured logging to all API endpoints: log request/response, errors, RAG backend calls, translation API calls
- [ ] T076 [P] Add ARIA labels to all interactive ChatBot elements for screen reader accessibility (chatbot button, panel, message list, input)
- [ ] T077 [P] Add keyboard navigation support: Tab to navigate, Esc to close chat panel, Enter to send message
- [ ] T078 [P] Implement high contrast mode support in chatbot CSS for accessibility compliance (WCAG 2.1 Level AA)
- [ ] T079 [P] Add loading states to all async operations: signup, login, sending message, fetching history
- [ ] T080 [P] Add optimistic UI updates in ChatBot: immediately show user message before API response
- [ ] T081 Add performance monitoring: measure and log chatbot response time (target <5s p95)
- [ ] T082 [P] Add frontend bundle size analysis and optimize: ensure chatbot components add <500KB gzipped
- [ ] T083 [P] Create Prisma seed script in api/prisma/seed.ts for development test users and sample chat sessions
- [ ] T084 Create Vercel configuration in vercel.json for serverless function routing from /api/* to api/src/*
- [ ] T085 [P] Add security headers middleware in api/src/middleware/security.ts (helmet.js: CSP, X-Frame-Options, HSTS)
- [ ] T086 [P] Add CORS whitelist configuration in api/src/server.ts for production domain
- [ ] T087 [P] Add input sanitization for chat messages: prevent XSS by sanitizing user input before storing and displaying
- [ ] T088 [P] Add SQL injection prevention verification: ensure all Prisma queries use parameterized inputs (verify in code review)
- [ ] T089 Setup Vercel Postgres database for production deployment and add DATABASE_URL to Vercel environment variables
- [ ] T090 Add Google Translate API key to Vercel environment variables and verify billing is enabled on GCP project
- [ ] T091 [P] Deploy Python RAG backend (backend/api/server.py) to separate hosting (Railway, Render, or Vercel Python runtime) and update PYTHON_RAG_URL
- [ ] T092 Run full quickstart.md validation: follow all steps from scratch to ensure documentation is accurate
- [ ] T093 [P] Add mobile responsive design testing: verify chatbot works on 320px minimum width devices
- [ ] T094 [P] Add browser compatibility testing: Chrome 90+, Firefox 88+, Safari 14+, Edge 90+
- [ ] T095 Create basic E2E test in e2e/tests/auth.spec.ts for signup → login → logout flow
- [ ] T096 Create basic E2E test in e2e/tests/chatbot.spec.ts for login → ask question → view citation → click citation link
- [ ] T097 Create basic E2E test in e2e/tests/translation.spec.ts for login → toggle to Urdu → verify UI translation → ask question in Urdu
- [ ] T098 [P] Code cleanup: remove console.logs, unused imports, commented code across all files
- [ ] T099 [P] TypeScript strict mode verification: ensure no 'any' types, all functions have return types, no type assertions without justification
- [ ] T100 Final security review: verify JWT_SECRET is strong, passwords never logged, all secrets in .env, no sensitive data in client-side code

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 2 - Auth (Phase 3)**: Depends on Foundational phase completion - Recommended to implement first for security foundation
- **User Story 1 - Chatbot (Phase 4)**: Depends on Foundational + User Story 2 (requires authentication)
- **User Story 3 - Translation (Phase 5)**: Depends on Foundational + User Story 2 (requires authentication) - Can be developed in parallel with User Story 1
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 2 (P2 - Auth)**: Can start after Foundational (Phase 2) - No dependencies on other stories (but must complete before US1 and US3 can be tested)
- **User Story 1 (P1 - Chatbot)**: Depends on US2 completion for authentication gate
- **User Story 3 (P3 - Translation)**: Depends on US2 completion for authenticated translation API access - Can be developed in parallel with US1 (different files)

### Within Each User Story

**User Story 2 (Auth)**:
1. T023-T024 (Models + Service) can run in parallel
2. T025-T028 (API endpoints) depend on T023-T024
3. T029-T031 (Frontend forms) can run in parallel
4. T032-T033 (Hooks + API client) depend on T029-T031
5. T034-T035 (UI integration) depend on T032-T033

**User Story 1 (Chatbot)**:
1. T036-T037 (Models) can run in parallel
2. T038 (RAG proxy service) can run in parallel with T036-T037
3. T039-T041 (API endpoints) depend on T036-T038
4. T042-T047 (UI components) can run in parallel
5. T048 (ChatBot wrapper) depends on T042-T047
6. T049-T050 (Hooks + API client) can run in parallel with T042-T047
7. T051-T055 (Integration) depend on T048-T050

**User Story 3 (Translation)**:
1. T056-T057 (Model + Service) can run in parallel
2. T058-T059 (API endpoints) depend on T056-T057
3. T060-T062 (UI components + hooks) can run in parallel
4. T063-T064 (Translation files) can run in parallel
5. T065-T071 (Integration) depend on T060-T064

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T003-T009)
- All Foundational tasks marked [P] can run in parallel within Phase 2 (T011-T015, T017-T022)
- User Story 1 and User Story 3 can be developed in parallel by different developers after US2 completes (different components and files)
- All Polish tasks marked [P] can run in parallel (T072-T080, T082-T088, T093-T094, T098-T099)

---

## Parallel Example: User Story 1 (Chatbot)

```bash
# Launch all models for User Story 1 together:
Task T036: "Create ChatSession model entity in api/src/models/ChatSession.ts"
Task T037: "Create ChatMessage model entity in api/src/models/ChatMessage.ts"
Task T038: "Create RAG proxy service in api/src/services/ragProxy.ts"

# Launch all UI components for User Story 1 together:
Task T042: "Create ChatButton component in src/components/ChatBot/ChatButton.tsx"
Task T043: "Create ChatPanel component in src/components/ChatBot/ChatPanel.tsx"
Task T044: "Create MessageList component in src/components/ChatBot/MessageList.tsx"
Task T045: "Create MessageInput component in src/components/ChatBot/MessageInput.tsx"
Task T046: "Create TypingIndicator component in src/components/ChatBot/TypingIndicator.tsx"
Task T047: "Create CitationLink component in src/components/ChatBot/CitationLink.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 2 + User Story 1)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 2 (Auth) - Required for access control
4. Complete Phase 4: User Story 1 (Chatbot) - Core value proposition
5. **STOP and VALIDATE**: Test authenticated users can ask questions and get cited answers
6. Deploy/demo if ready (functional chatbot with auth, no translation yet)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 2 (Auth) → Test independently → Deploy/Demo (users can signup/login)
3. Add User Story 1 (Chatbot) → Test with auth → Deploy/Demo (MVP! - authenticated Q&A with citations)
4. Add User Story 3 (Translation) → Test independently → Deploy/Demo (Full feature set with Urdu support)
5. Add Polish (Phase 6) → Production-ready deployment
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (critical path)
2. Once Foundational is done:
   - Developer A: User Story 2 (Auth) - MUST complete first
3. Once User Story 2 is done:
   - Developer A: User Story 1 (Chatbot backend + frontend)
   - Developer B: User Story 3 (Translation) - can work in parallel
4. Stories complete and integrate independently
5. Team completes Polish tasks in parallel

---

## Notes

- [P] tasks = different files, no dependencies within the phase
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Tests are NOT included in this task list as they were not explicitly requested in spec.md
- However, test infrastructure (Playwright, test directories) is created for future use
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- MVP = User Story 2 (Auth) + User Story 1 (Chatbot) = ~65 tasks
- Full feature = All 3 User Stories = ~71 tasks
- Production-ready = All phases including Polish = 100 tasks
- Estimated effort from plan.md: 3-4 weeks total (1 week auth, 1.5 weeks chatbot, 1 week translation, 0.5 weeks polish)
