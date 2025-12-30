# Implementation Plan: RAG Chatbot Frontend Integration

**Branch**: `002-chatbot-frontend-integration` | **Date**: 2025-12-25 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-chatbot-frontend-integration/spec.md`

## Summary

Integrate the completed Python RAG chatbot backend into the existing Docusaurus-based Physical AI textbook website by creating: (1) a React-based chatbot UI component with floating button and expandable panel, (2) a Node.js/Express API backend for user authentication and RAG backend proxying, (3) user signup/login with JWT session management, and (4) English-to-Urdu translation with RTL layout support. The frontend will communicate with the existing Python RAG backend via the new Node.js proxy layer, maintaining separation of concerns while enabling authenticated, multilingual interactions.

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.6+ with React 19 (Docusaurus 3.9.2)
- Backend API: Node.js 20+ with Express.js 4.x or Fastify 4.x
- Database: SQLite 3.x (development) / PostgreSQL 15+ (production)

**Primary Dependencies**:
- Frontend: `@docusaurus/core`, `react`, `react-dom`, `axios`, `react-query`, `i18next` (translation)
- Backend API: `express`/`fastify`, `jsonwebtoken`, `bcrypt`, `prisma`/`typeorm` (ORM), `axios` (RAG proxy)
- Translation: Google Translate API client or `@google-cloud/translate`
- Testing: `jest`, `@testing-library/react`, `supertest` (API tests)

**Storage**:
- SQLite for local development
- PostgreSQL for production deployment
- Schema: Users (id, email, password_hash, created_at, language_pref), Sessions (id, user_id, token, expires_at), ChatHistory (id, user_id, session_id, messages_json, created_at)

**Testing**:
- Frontend: Jest + React Testing Library for component tests
- Backend API: Jest + Supertest for endpoint integration tests
- E2E: Playwright or Cypress for full user flows

**Target Platform**:
- Vercel deployment (Next.js API routes or serverless functions for backend)
- Browser targets: Chrome 90+, Firefox 88+, Safari 14+, Edge 90+

**Project Type**: Web application (Docusaurus frontend + Node.js API backend)

**Performance Goals**:
- Chatbot response time: <5s (p95) including RAG backend latency
- Translation latency: <1s for UI elements, <2s for chatbot responses
- Page load impact: <200ms additional overhead for chat component
- Support 50+ concurrent authenticated users
- JWT validation: <50ms

**Constraints**:
- Must not modify existing Docusaurus site structure or break navigation
- Must not require changes to Python RAG backend codebase
- Frontend bundle size increase: <500KB gzipped
- All user data encrypted at rest and in transit (HTTPS/TLS)
- WCAG 2.1 Level AA accessibility compliance
- Google Translate API costs must be monitored (cache translations where possible)

**Scale/Scope**:
- Initial launch: 50-100 active users
- Database: ~1000 users, ~10,000 chat messages initially
- Translation cache: ~5000 phrase pairs (English-Urdu)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Security-First Development
✅ **Pass** - User passwords will be hashed with bcrypt (min 10 salt rounds)
✅ **Pass** - JWT tokens will use strong secrets from environment variables (`.env`)
✅ **Pass** - Database credentials stored in `.env`, never committed to git
✅ **Pass** - Google Translate API keys loaded from environment
✅ **Pass** - Input validation on all API endpoints (email format, password strength, message length)
✅ **Pass** - HTTPS enforced in production (Vercel provides TLS)
✅ **Pass** - SQL injection prevention via ORM (Prisma/TypeORM parameterized queries)
✅ **Pass** - CORS configured to allow only the Docusaurus origin

### Modular & Production-Ready Code
✅ **Pass** - TypeScript with strict mode enabled for type safety
✅ **Pass** - Frontend components follow single-responsibility (ChatButton, ChatPanel, MessageList, Auth forms)
✅ **Pass** - Backend routes separated by concern (auth, chat, translation, RAG proxy)
✅ **Pass** - Structured logging (Winston or Pino) for API requests and errors
✅ **Pass** - Error boundaries in React for graceful failure handling
✅ **Pass** - API error responses follow consistent format (status, message, data)

### Integration Architecture
✅ **Pass** - Node.js API acts as proxy to Python RAG backend (keeps systems decoupled)
✅ **Pass** - RAG backend endpoint discovery via environment variable (PYTHON_RAG_URL)
✅ **Pass** - Retry logic for RAG backend calls (3 attempts with exponential backoff)
✅ **Pass** - Timeout handling (30s max for RAG queries)
✅ **Pass** - Fallback UI when RAG backend unavailable ("Service temporarily unavailable")

### Accessibility & Internationalization
✅ **Pass** - All interactive elements keyboard-navigable (Tab, Enter, Esc)
✅ **Pass** - ARIA labels for chatbot button, chat panel, message list
✅ **Pass** - RTL layout support via CSS logical properties (`inline-start`, `inline-end`)
✅ **Pass** - i18next for language management with persistent locale preference
✅ **Pass** - High contrast mode support for chat UI
✅ **Pass** - Screen reader announcements for new chat messages

### Testing & Quality Assurance
✅ **Pass** - Unit tests for React components (ChatButton, MessageList, AuthForms)
✅ **Pass** - Integration tests for API endpoints (signup, login, logout, chat, translate)
✅ **Pass** - E2E tests for critical flows (signup → login → ask question → view citation)
✅ **Pass** - Mock RAG backend for frontend development and testing
✅ **Pass** - Test coverage target: 80% for business logic

**Overall Status**: ✅ ALL GATES PASS - Proceed to Phase 0 Research

## Project Structure

### Documentation (this feature)

```text
specs/002-chatbot-frontend-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   ├── auth-api.yaml        # Authentication endpoints (signup, login, logout)
│   ├── chat-api.yaml        # Chat endpoints (send message, get history)
│   └── translation-api.yaml # Translation endpoints
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Backend API (NEW - Node.js authentication + RAG proxy)
api/
├── src/
│   ├── routes/
│   │   ├── auth.ts          # POST /api/auth/signup, /login, /logout
│   │   ├── chat.ts          # POST /api/chat/message, GET /api/chat/history
│   │   └── translate.ts     # POST /api/translate
│   ├── middleware/
│   │   ├── auth.ts          # JWT verification middleware
│   │   ├── rateLimit.ts     # Rate limiting (30 questions/hour)
│   │   └── validation.ts    # Input validation schemas
│   ├── services/
│   │   ├── authService.ts   # User signup, login, token generation
│   │   ├── ragProxy.ts      # Proxy to Python RAG backend
│   │   └── translateService.ts # Google Translate API integration
│   ├── models/
│   │   ├── User.ts          # User entity (Prisma/TypeORM)
│   │   └── ChatSession.ts   # Chat session entity
│   ├── db/
│   │   ├── schema.prisma    # Prisma schema (if using Prisma)
│   │   └── migrations/      # Database migrations
│   ├── utils/
│   │   ├── logger.ts        # Winston/Pino logger
│   │   └── errors.ts        # Custom error classes
│   └── server.ts            # Express/Fastify app entry point
├── tests/
│   ├── integration/
│   │   ├── auth.test.ts     # Test auth endpoints
│   │   └── chat.test.ts     # Test chat endpoints
│   └── unit/
│       └── authService.test.ts
├── .env.example             # Environment variables template
├── package.json
└── tsconfig.json

# Frontend (MODIFIED - Docusaurus + new React components)
src/
├── components/
│   ├── ChatBot/
│   │   ├── ChatButton.tsx       # Floating chat button (bottom-right)
│   │   ├── ChatPanel.tsx        # Expandable chat panel
│   │   ├── MessageList.tsx      # Display messages with citations
│   │   ├── MessageInput.tsx     # Multiline input with Send button
│   │   ├── TypingIndicator.tsx  # Animated typing dots
│   │   └── CitationLink.tsx     # Clickable citation component
│   ├── Auth/
│   │   ├── LoginForm.tsx        # Email + password login
│   │   ├── SignupForm.tsx       # Email + password signup
│   │   └── AuthModal.tsx        # Modal wrapper for auth forms
│   └── LanguageToggle/
│       └── LanguageToggle.tsx   # English/Urdu toggle button
├── hooks/
│   ├── useAuth.ts               # Authentication state management
│   ├── useChat.ts               # Chat message state + API calls
│   └── useTranslation.ts        # i18next wrapper hook
├── services/
│   ├── api.ts                   # Axios instance with auth interceptor
│   ├── authApi.ts               # API calls: signup, login, logout
│   ├── chatApi.ts               # API calls: sendMessage, getHistory
│   └── translationApi.ts        # API calls: translate text
├── contexts/
│   └── AuthContext.tsx          # Global auth state (React Context)
├── styles/
│   ├── chatbot.css              # Chat UI styles
│   └── rtl.css                  # RTL layout overrides
└── theme/
    └── Root.tsx                 # Docusaurus theme wrapper (inject ChatBot)

# Backend RAG (EXISTING - No changes)
backend/
└── [existing Python RAG backend - unchanged]

# Tests (NEW - E2E tests for frontend integration)
e2e/
├── tests/
│   ├── auth.spec.ts             # E2E: signup → login → logout
│   ├── chatbot.spec.ts          # E2E: ask question → view citation
│   └── translation.spec.ts      # E2E: toggle language → verify UI
└── playwright.config.ts

# Database (NEW)
prisma/
├── schema.prisma                # Prisma schema file
└── migrations/                  # Auto-generated migrations
```

**Structure Decision**: Hybrid web application structure with separated concerns:
1. **API Backend** (`api/`): New Node.js service for authentication, RAG proxying, and translation
2. **Frontend** (`src/`): Modified Docusaurus site with new React components for chatbot and auth
3. **Backend RAG** (`backend/`): Existing Python RAG system (unchanged)
4. **E2E Tests** (`e2e/`): Playwright tests for full user journeys

This structure maintains the existing Docusaurus site, adds the API layer without modifying the Python RAG backend, and keeps testing separate.

## Complexity Tracking

> **No violations detected** - All constitution gates pass. This section is empty.

## Phase 0: Research & Technology Selection

### Research Questions

1. **Docusaurus Custom Component Integration**
   - How to add persistent components (ChatButton) to all pages without modifying each page?
   - Best practice: Use Docusaurus `theme/Root.tsx` swizzling or global layout wrapper

2. **Node.js API Deployment on Vercel**
   - Does Vercel support Express/Fastify as serverless functions?
   - Alternative: Use Vercel's native API routes (`/api` directory with Next.js-style handlers)

3. **Python RAG Backend Communication**
   - What HTTP endpoint does the Python RAG backend expose?
   - Expected request/response format for chat queries?
   - Authentication required for RAG backend (if any)?

4. **Google Translate API Integration**
   - Translation API v2 (simpler) vs. v3 (advanced features) - which to use?
   - Cost estimation: ~$20/million characters - feasible for expected volume?
   - Caching strategy to reduce API calls?

5. **Database Choice for Vercel Deployment**
   - SQLite: Works locally, not compatible with Vercel serverless (ephemeral filesystem)
   - PostgreSQL: Recommended for Vercel (via Vercel Postgres or external provider like Supabase/Neon)

6. **JWT Session Management**
   - Token expiration: 7 days (spec assumption) - stored in HTTP-only cookie or localStorage?
   - Refresh token pattern needed, or single long-lived token acceptable for MVP?

7. **RTL Layout Best Practices**
   - CSS logical properties (`margin-inline-start` vs. `margin-left`) for automatic RTL flipping?
   - Docusaurus built-in RTL support or custom implementation?

### Research Outputs

(To be filled in research.md during Phase 0 execution)

## Phase 1: Detailed Design

### Data Model

(To be created in `data-model.md`)

**Entities**:
- User (id, email, password_hash, created_at, language_preference)
- Chat Session (id, user_id, created_at, messages)
- Chat Message (id, session_id, role, content, citations, timestamp, translated)

### API Contracts

(To be created in `contracts/` directory)

**Endpoints**:
- `POST /api/auth/signup` - Create new user account
- `POST /api/auth/login` - Authenticate user and issue JWT
- `POST /api/auth/logout` - Invalidate session token
- `POST /api/chat/message` - Send question to RAG backend
- `GET /api/chat/history` - Retrieve user's chat history
- `POST /api/translate` - Translate text English ↔ Urdu

### Quickstart Guide

(To be created in `quickstart.md`)

**Topics to cover**:
1. Local development setup (Node.js, Python RAG backend, database)
2. Environment variables configuration
3. Database initialization and migrations
4. Running frontend and backend concurrently
5. Testing auth flow and chatbot interaction
6. Deploying to Vercel (frontend + API routes)
7. Connecting to production PostgreSQL

## Architecture Decision Records

### ADR-001: Use Node.js API Layer Instead of Direct Frontend-to-Python Communication

**Context**: The frontend needs to authenticate users and communicate with the Python RAG backend.

**Decision**: Create a Node.js/Express API layer that handles authentication and proxies requests to the Python RAG backend, rather than calling Python directly from the React frontend.

**Rationale**:
- Enables user authentication and session management in a JavaScript ecosystem (JWT, bcrypt)
- Keeps Python RAG backend stateless and focused on RAG functionality
- Provides a single API surface for the frontend (auth + chat + translation)
- Allows rate limiting and request validation before hitting RAG backend
- Facilitates future horizontal scaling (Node.js API can be scaled independently)

**Alternatives Considered**:
1. **Direct Python Flask/FastAPI auth**: Would require extending Python backend with auth logic, mixing concerns
2. **Firebase Auth + Cloud Functions**: Vendor lock-in and additional cost
3. **Supabase Auth**: External dependency, learning curve for team

**Consequences**:
- Additional operational complexity (two backend services)
- Slight latency increase (Node.js proxy overhead ~10-50ms)
- Benefit: Separation of concerns, easier to replace RAG backend in future

---

### ADR-002: Use Vercel Postgres Instead of SQLite for Production

**Context**: The spec suggests SQLite (dev) or PostgreSQL (production). Vercel's serverless environment has ephemeral filesystems.

**Decision**: Use Vercel Postgres (or external Supabase/Neon) for both development and production, with option for local PostgreSQL via Docker.

**Rationale**:
- SQLite incompatible with Vercel's serverless functions (no persistent filesystem)
- Vercel Postgres offers managed PostgreSQL with zero-config deployment
- Consistent database behavior across dev and prod
- Prisma supports PostgreSQL with excellent migration tools

**Alternatives Considered**:
1. **SQLite + migrate to Postgres**: Extra migration step, different behavior in dev/prod
2. **MongoDB Atlas**: NoSQL not ideal for relational user/session data
3. **Self-hosted PostgreSQL**: Requires infrastructure management

**Consequences**:
- Requires PostgreSQL running locally (Docker Compose) or cloud instance for development
- Benefit: Production parity, no migration needed when deploying

---

### ADR-003: Use i18next for Translation Management with Google Translate API Integration

**Context**: Need to translate UI and chatbot responses from English to Urdu.

**Decision**: Use i18next for frontend translation keys and integrate Google Translate API for dynamic chatbot response translation.

**Rationale**:
- i18next is battle-tested for React localization
- UI translations can be pre-defined (no API cost)
- Google Translate API used only for dynamic chatbot responses (user-generated content)
- Caching translated responses reduces API costs
- Supports RTL via i18next language detection

**Alternatives Considered**:
1. **Manual translation files only**: Doesn't handle dynamic chatbot responses
2. **Microsoft Translator**: Similar pricing, less common in React ecosystem
3. **DeepL API**: Higher quality, but higher cost (~5x Google Translate)

**Consequences**:
- Google Translate API costs (~$20/million characters)
- Translation quality may vary for technical terms (requires monitoring and manual overrides)
- Benefit: Fast implementation, dynamic translation support

## Risks and Mitigation

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Python RAG backend unavailable during chat | High | Medium | Implement retry logic (3 attempts), display user-friendly error message, log failures for monitoring |
| Google Translate API rate limits exceeded | Medium | Low | Implement aggressive caching (Redis or in-memory), pre-translate common UI elements, display original English if translation fails |
| JWT token theft/session hijacking | High | Low | Use HTTP-only cookies (not localStorage), short expiration (7 days), implement token refresh pattern, require re-auth for sensitive actions |
| Docusaurus build breaks after adding custom components | Medium | Medium | Thoroughly test in development, use Docusaurus's official swizzling mechanism, maintain fallback to default theme |
| Database connection pooling issues on Vercel serverless | Medium | Medium | Use Prisma Data Proxy or Vercel Postgres (built-in connection pooling), set appropriate connection limits |
| RTL layout breaks citation links or chat UI | Low | Medium | Extensive testing with Urdu language, use CSS logical properties, manual RTL testing with native Urdu speakers |

## Next Steps After Planning

1. **Execute `/sp.tasks`** to generate detailed task breakdown from this plan
2. Review tasks with team for estimation and prioritization
3. Set up development environment (Node.js, PostgreSQL, Python RAG backend)
4. Begin implementation starting with authentication (P2) before chatbot integration (P1) for security foundation

---

**Plan Status**: ✅ Complete - Ready for task generation
**Approval Required**: Technical lead review of architecture decisions (ADRs)
**Estimated Effort**: 3-4 weeks (1 week auth, 1.5 weeks chatbot integration, 1 week translation, 0.5 weeks testing/polish)
