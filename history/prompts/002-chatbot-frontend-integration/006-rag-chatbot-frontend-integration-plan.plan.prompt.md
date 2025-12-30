---
id: 006
title: RAG Chatbot Frontend Integration Plan
stage: plan
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-chatbot-frontend-integration
branch: 002-chatbot-frontend-integration
user: Computer Technology
command: /sp.plan
labels: ["planning", "architecture", "frontend-integration", "api-design", "documentation"]
links:
  spec: specs/002-chatbot-frontend-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-chatbot-frontend-integration/plan.md
 - specs/002-chatbot-frontend-integration/research.md
 - specs/002-chatbot-frontend-integration/data-model.md
 - specs/002-chatbot-frontend-integration/quickstart.md
 - specs/002-chatbot-frontend-integration/contracts/auth-api.yaml
 - specs/002-chatbot-frontend-integration/contracts/chat-api.yaml
 - specs/002-chatbot-frontend-integration/contracts/translation-api.yaml
tests:
 - None (planning phase)
---

## Prompt

/sp.plan

(No additional arguments - command invoked to create implementation plan from existing specification)

## Response snapshot

Created complete implementation plan for RAG Chatbot Frontend Integration with comprehensive architecture, research, data modeling, and API contracts.

**Planning Artifacts Created**:

1. **Implementation Plan** (`plan.md`):
   - Technical Context: TypeScript/React frontend, Node.js API backend, PostgreSQL database
   - Constitution Check: All 5 gates passed (Security, Modular Code, Integration, Accessibility, Testing)
   - Project Structure: Hybrid web app with separated concerns (API, Frontend, Backend RAG, E2E tests)
   - 3 Architecture Decision Records (ADRs): Node.js proxy layer, PostgreSQL over SQLite, i18next+Google Translate
   - Risk matrix with 6 identified risks and mitigation strategies
   - Estimated effort: 3-4 weeks

2. **Research Document** (`research.md`):
   - 7 research questions investigated and answered
   - Docusaurus component integration: Use `theme/Root.tsx` swizzling
   - Vercel deployment: Serverless functions with individual endpoint files
   - Python RAG communication: Add minimal FastAPI server wrapper
   - Translation: Google Translate API v2 with aggressive caching (~$15-30/month)
   - Database: Vercel Postgres (prod), Docker PostgreSQL (dev)
   - JWT storage: HTTP-only cookies with SameSite=Strict
   - RTL layout: CSS logical properties + Docusaurus i18n

3. **Data Model** (`data-model.md`):
   - 4 entities: User, ChatSession, ChatMessage, TranslationCache
   - Complete Prisma schema with PostgreSQL types
   - Indexes on foreign keys and frequently queried fields
   - Initial database migration SQL
   - Common query patterns and performance considerations

4. **API Contracts** (`contracts/`):
   - `auth-api.yaml`: 4 endpoints (signup, login, logout, me) with OpenAPI 3.0 spec
   - `chat-api.yaml`: 3 endpoints (sendMessage, getHistory, getSession) with citation support
   - `translation-api.yaml`: 2 endpoints (translate, translateBatch) with caching
   - All contracts include request/response schemas, error codes, security schemes

5. **Quickstart Guide** (`quickstart.md`):
   - Complete local development setup (4 parts, ~45 min)
   - Docker Compose for PostgreSQL
   - Environment variable configuration
   - Database initialization with Prisma
   - Python FastAPI server creation
   - Node.js API backend setup
   - Frontend integration steps
   - Deployment to Vercel instructions
   - Troubleshooting section

**Key Architectural Decisions**:

1. **Node.js API Proxy Layer**: Decided to create a separate Node.js/Express backend instead of direct frontend-to-Python communication. This provides authentication, rate limiting, and clean separation of concerns. Python RAG backend remains stateless and focused on RAG functionality.

2. **PostgreSQL for All Environments**: Chose Vercel Postgres over SQLite due to serverless incompatibility. Development uses Docker PostgreSQL for production parity, eliminating migration issues.

3. **i18next + Google Translate Hybrid**: Static UI translations via i18next (free), dynamic chatbot responses via Google Translate API with aggressive caching (estimated $15-30/month actual cost vs. $150 without caching).

**Research Outcomes**:

All 7 research questions resolved:
- Docusaurus: Swizzle Root component for global chat button
- Vercel: Use serverless functions (not Express monolith)
- Python RAG: Add minimal FastAPI wrapper (`backend/api/server.py`)
- Translation: Google Translate API v2 with cache (80-90% cost reduction)
- Database: PostgreSQL everywhere (Docker locally, Vercel Postgres in prod)
- JWT: HTTP-only cookies (XSS protection, auto CSRF via SameSite)
- RTL: CSS logical properties (future-proof, automatic flipping)

**Technical Stack**:

Frontend:
- TypeScript 5.6+ with React 19 (Docusaurus 3.9.2)
- axios, react-query, i18next
- Jest + React Testing Library

Backend API:
- Node.js 20+ with Express.js 4.x
- Prisma ORM with PostgreSQL
- jsonwebtoken, bcrypt, Google Translate client

Database:
- PostgreSQL 15+ (Vercel Postgres for prod, Docker for dev)
- 4 tables: users, chat_sessions, chat_messages, translation_cache

Testing:
- Frontend: Jest + React Testing Library
- Backend: Jest + Supertest
- E2E: Playwright

**Performance Goals**:
- Chatbot response < 5s (p95)
- Translation < 1s (UI), < 2s (chatbot responses)
- Page load impact < 200ms
- Support 50+ concurrent users

**Next Steps**:
1. Execute `/sp.tasks` to generate detailed task breakdown
2. Review ADRs with technical lead
3. Set up development environment (Node.js, PostgreSQL, Python)
4. Begin implementation: Auth → Chatbot Integration → Translation

## Outcome

- ✅ Impact: Created production-ready architecture plan with detailed design artifacts, API contracts, and deployment guide. All research questions resolved with concrete technical decisions.
- 🧪 Tests: No tests at planning stage. Testing strategy defined in plan (80% coverage target, unit + integration + E2E).
- 📁 Files: 7 planning artifacts created (plan, research, data model, 3 API contracts, quickstart guide). Agent context updated.
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation tasks from plan. Consider `/sp.adr` to document the 3 significant architectural decisions if formal ADR tracking is required.
- 🧠 Reflection: Successfully balanced technical depth with actionability. Research phase prevented costly mistakes (SQLite on Vercel, direct Python integration). Prisma schema and API contracts provide clear implementation contracts. Quickstart guide enables rapid developer onboarding.

## Evaluation notes (flywheel)

- Failure modes observed: None. Planning completed without iteration.
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all 5 gates), Research - COMPLETE (7/7 questions answered), Contracts - PASS (OpenAPI valid, all endpoints documented)
- Prompt variant (if applicable): Standard /sp.plan workflow with comprehensive Phase 0 (research) and Phase 1 (data model + contracts + quickstart)
- Next experiment (smallest change to try): Proceed to `/sp.tasks` to validate that plan provides sufficient detail for task generation
