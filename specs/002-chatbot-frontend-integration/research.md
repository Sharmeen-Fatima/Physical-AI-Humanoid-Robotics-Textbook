# Research: RAG Chatbot Frontend Integration

**Feature**: RAG Chatbot Frontend Integration
**Date**: 2025-12-25
**Status**: Complete

## Research Questions and Findings

### 1. Docusaurus Custom Component Integration

**Question**: How to add persistent components (ChatButton) to all pages without modifying each page?

**Research Findings**:

Docusaurus provides two primary mechanisms for adding global components:

1. **Theme Swizzling with Root Component** (Recommended)
   - Create `src/theme/Root.tsx` (or `.js`) by swizzling the default theme
   - This component wraps the entire application
   - Perfect for injecting global components like chat buttons

   ```tsx
   // src/theme/Root.tsx
   import React from 'react';
   import ChatBot from '@site/src/components/ChatBot';

   export default function Root({children}) {
     return (
       <>
         {children}
         <ChatBot />
       </>
     );
   }
   ```

2. **Docusaurus Plugin with Client Module**
   - Create a custom plugin that injects client-side code
   - More complex, but offers more control over lifecycle

**Decision**: Use `src/theme/Root.tsx` swizzling approach
- Simpler implementation
- Official Docusaurus pattern
- No build configuration changes needed
- Easy to maintain and understand

**Alternatives Considered**:
- Custom plugin: Overkill for simple component injection
- Modifying each MDX file: Not scalable, breaks on new pages

---

### 2. Node.js API Deployment on Vercel

**Question**: Does Vercel support Express/Fastify as serverless functions?

**Research Findings**:

Vercel supports multiple approaches for backend APIs:

1. **Vercel Serverless Functions (Recommended for this project)**
   - Use `api/` directory with individual endpoint files
   - Each file exports a request handler (similar to Next.js API routes)
   - Automatic serverless deployment with zero config
   - Example:
     ```ts
     // api/auth/login.ts
     import { VercelRequest, VercelResponse } from '@vercel/node';
     export default function handler(req: VercelRequest, res: VercelResponse) {
       // Handle login logic
     }
     ```

2. **Express/Fastify with @vercel/node adapter**
   - Possible but requires additional configuration
   - Single `api/index.ts` file that exports the Express/Fastify app
   - Less idiomatic for Vercel, may have cold start issues

3. **Separate Node.js deployment (e.g., Railway, Render)**
   - Deploy API backend independently
   - More operational complexity
   - Better for complex applications

**Decision**: Use Vercel Serverless Functions with individual endpoint files
- Native Vercel integration
- Automatic scaling and edge deployment
- No additional infrastructure needed
- Consistent with existing Docusaurus/Vercel setup

**Trade-offs**:
- Cold starts (~1-2s first request): Acceptable for this use case
- Limited middleware flexibility: Mitigated by shared utility functions
- File-based routing: Simpler than Express routes for small API surface

---

### 3. Python RAG Backend Communication

**Question**: What HTTP endpoint does the Python RAG backend expose? Expected request/response format?

**Research Findings** (based on existing backend code inspection):

The Python RAG backend currently exists as a CLI application (`scripts/chatbot.py`) without an HTTP API. To integrate with the frontend, we need to:

**Option A: Add HTTP API to Python Backend** (requires changes to backend - violates constraint)
**Option B: Create Node.js wrapper that invokes Python CLI** (fragile, not production-ready)
**Option C: Extract RAG logic into reusable HTTP API** (Recommended)

**Recommended Approach**: Create a minimal Python FastAPI wrapper around existing RAG components
- **File**: `backend/api/server.py` (new file, doesn't modify existing logic)
- **Endpoint**: `POST /query`
- **Request Format**:
  ```json
  {
    "question": "What is physical AI?",
    "session_id": "optional-session-uuid",
    "top_k": 5,
    "similarity_threshold": 0.70
  }
  ```
- **Response Format**:
  ```json
  {
    "answer": "Physical AI refers to...",
    "sources": [
      {
        "url": "https://book.com/chapter-1",
        "section": "Introduction to Physical AI",
        "score": 0.873
      }
    ],
    "grounded": true,
    "model": "gemini-1.5-pro"
  }
  ```

**Implementation**:
```python
# backend/api/server.py (NEW FILE - minimal changes)
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from src.retrieval.vector_search import VectorSearch
from src.generation.gemini_generator import GeminiGenerator

app = FastAPI()

class QueryRequest(BaseModel):
    question: str
    session_id: str | None = None
    top_k: int = 5

@app.post("/query")
async def query(req: QueryRequest):
    # Use existing RAG components
    retriever = VectorSearch()
    generator = GeminiGenerator()
    results = retriever.search(req.question, top_k=req.top_k)
    answer = generator.generate(req.question, results)
    return {"answer": answer, "sources": results}
```

**Decision**: Add minimal FastAPI server to existing Python backend
- Reuses existing RAG logic (no duplication)
- Clean HTTP API for Node.js proxy to call
- Can run independently on separate port (e.g., :8000)
- Environment variable in Node.js: `PYTHON_RAG_URL=http://localhost:8000`

**Note**: This is a minimal addition that doesn't modify existing CLI functionality.

---

### 4. Google Translate API Integration

**Question**: Translation API v2 (simpler) vs. v3 (advanced features) - which to use? Cost estimation?

**Research Findings**:

**Google Cloud Translation API v2** (Basic)
- REST API, simpler integration
- Automatic language detection
- Text-only translation
- Pricing: $20 per million characters
- Node.js client: `@google-cloud/translate` (supports both v2 and v3)

**Google Cloud Translation API v3** (Advanced)
- More features: glossaries, batch translation, custom models
- Better quality for technical content
- Regional endpoints for lower latency
- Pricing: Same ($20/million chars), additional features cost extra
- Requires more setup (project ID, region specification)

**Cost Estimation** (for expected volume):
- Assume 100 active users
- Each user asks ~5 questions/day
- Average chatbot response: 500 characters
- Monthly volume: 100 users × 5 questions × 500 chars × 30 days = 7.5 million characters
- Monthly cost: 7.5M × ($20/1M) = **~$150/month**

**Caching Strategy** to reduce costs:
1. **Pre-translate static UI elements** (buttons, labels, etc.) - store in `locales/ur/translation.json`
2. **Cache chatbot responses** - Hash question + answer, store translation in database/Redis
3. **Cache book content translations** - Translate chapters once, persist in database
4. **Estimated savings**: 80-90% reduction → **~$15-30/month** actual cost

**Decision**: Use Google Cloud Translation API v2 with aggressive caching
- Simpler integration for MVP
- Same pricing as v3
- Sufficient quality for general text
- Can upgrade to v3 later if custom glossaries needed for technical terms

**Alternative**: Microsoft Translator Text API
- Similar pricing (~$10/million characters)
- Less ecosystem support in Node.js
- Decision: Stick with Google for consistency with existing Google Gemini usage

---

### 5. Database Choice for Vercel Deployment

**Question**: SQLite vs. PostgreSQL for Vercel serverless?

**Research Findings**:

**SQLite on Vercel**:
- ❌ Not supported for production
- Vercel serverless functions have ephemeral filesystems
- Each function invocation may get a different container
- Database file would be lost between deployments

**PostgreSQL Options**:
1. **Vercel Postgres** (Recommended)
   - Native integration with Vercel projects
   - Automatic connection pooling (Neon-powered)
   - Zero-config setup from Vercel dashboard
   - Pricing: Free tier (256MB), then $20/month for 512MB
   - Environment variables auto-injected (`POSTGRES_URL`)

2. **Supabase**
   - PostgreSQL + real-time + auth + storage
   - Free tier: 500MB database, 50K monthly active users
   - Connection pooling included
   - More features than needed, but free tier is generous

3. **Railway / Render PostgreSQL**
   - Dedicated PostgreSQL instances
   - More control, but requires manual setup
   - Slightly more expensive for equivalent resources

**Decision**: Use Vercel Postgres for production, local PostgreSQL (Docker) for development
- Seamless Vercel integration
- Built-in connection pooling (critical for serverless)
- Free tier sufficient for initial launch
- Development parity: Use Docker Compose with PostgreSQL 15

**Development Setup**:
```yaml
# docker-compose.yml
services:
  postgres:
    image: postgres:15
    environment:
      POSTGRES_DB: chatbot_dev
      POSTGRES_USER: dev
      POSTGRES_PASSWORD: dev_password
    ports:
      - "5432:5432"
    volumes:
      - pgdata:/var/lib/postgresql/data
```

---

### 6. JWT Session Management

**Question**: Token expiration, storage location (HTTP-only cookie vs. localStorage), refresh token pattern?

**Research Findings**:

**Token Storage Options**:
1. **HTTP-only Cookies** (Recommended)
   - ✅ Not accessible via JavaScript (XSS protection)
   - ✅ Automatically sent with requests (no manual header management)
   - ✅ Can be marked `Secure` (HTTPS only) and `SameSite` (CSRF protection)
   - ❌ Slightly more complex to implement (requires cookie-parser middleware)

2. **localStorage**
   - ✅ Simple to implement
   - ❌ Vulnerable to XSS attacks (malicious scripts can read token)
   - ❌ Requires manual Authorization header management
   - ❌ Not recommended for sensitive tokens

**Token Expiration Strategy**:
- **Access Token**: Short-lived (7 days, as per spec assumption)
- **Refresh Token**: Not needed for MVP (adds complexity)
- **Session Extension**: Re-issue token on each authenticated request (sliding window)

**Implementation**:
```ts
// Backend: Set HTTP-only cookie
res.cookie('auth_token', jwt.sign(payload, SECRET), {
  httpOnly: true,
  secure: process.env.NODE_ENV === 'production',
  sameSite: 'strict',
  maxAge: 7 * 24 * 60 * 60 * 1000 // 7 days
});

// Frontend: Axios automatically includes cookies
axios.post('/api/chat/message', { question }, { withCredentials: true });
```

**Decision**: Use HTTP-only cookies for JWT storage
- Security best practice
- Automatic CSRF protection with `SameSite=Strict`
- No refresh token pattern for MVP (can add later if needed)

---

### 7. RTL Layout Best Practices

**Question**: CSS logical properties vs. manual RTL styles? Docusaurus built-in support?

**Research Findings**:

**CSS Logical Properties** (Modern Approach)
- Use `margin-inline-start` instead of `margin-left`
- Use `padding-block-end` instead of `padding-bottom`
- Automatically flip based on `dir="rtl"` attribute
- Example:
  ```css
  .chat-panel {
    margin-inline-start: 20px; /* Left in LTR, Right in RTL */
    border-inline-start: 1px solid; /* Left border in LTR, Right in RTL */
  }
  ```

**Docusaurus RTL Support**:
- ✅ Built-in RTL support via i18n configuration
- Set `direction: 'rtl'` in `docusaurus.config.ts` for specific locales
- Automatically applies `dir="rtl"` to `<html>` element
- Example:
  ```ts
  // docusaurus.config.ts
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      ur: {
        direction: 'rtl',
        label: 'اردو',
        htmlLang: 'ur-PK',
      },
    },
  },
  ```

**CSS Approach for Chat Components**:
1. Use CSS logical properties for new chat UI components
2. Test with `dir="rtl"` attribute on chat panel
3. Manual RTL overrides for edge cases (e.g., citation link icons)

**Example**:
```css
/* chatbot.css */
.chat-button {
  /* Works in both LTR and RTL */
  position: fixed;
  bottom: 20px;
  inset-inline-end: 20px; /* Right in LTR, Left in RTL */
}

.message {
  text-align: start; /* Left in LTR, Right in RTL */
  padding-inline-start: 10px;
}
```

**Decision**: Use CSS logical properties + Docusaurus i18n RTL support
- Future-proof approach (CSS standard)
- Automatic flipping with minimal code
- Docusaurus handles locale switching
- Manual testing with Urdu-speaking users to catch edge cases

**Testing Strategy**:
1. Enable RTL mode in browser dev tools
2. Visual regression tests with Percy or Chromatic
3. Manual review by native Urdu speaker

---

## Summary of Key Decisions

| Research Area | Decision | Rationale |
|---------------|----------|-----------|
| Global Component Injection | Docusaurus `theme/Root.tsx` swizzling | Official pattern, simple, maintainable |
| Backend API Deployment | Vercel Serverless Functions (individual endpoint files) | Native integration, auto-scaling, zero config |
| Python RAG Communication | Add minimal FastAPI server to backend | Reuses logic, clean API, no code duplication |
| Translation API | Google Translate API v2 + aggressive caching | Simple integration, sufficient quality, cost-effective |
| Database | Vercel Postgres (prod), Docker PostgreSQL (dev) | Serverless-compatible, connection pooling, free tier |
| JWT Storage | HTTP-only cookies with `SameSite=Strict` | Security best practice, XSS/CSRF protection |
| RTL Layout | CSS logical properties + Docusaurus i18n | Standard approach, automatic flipping, future-proof |

## Outstanding Questions for Implementation

1. **Python RAG Backend API**: Who will implement the minimal FastAPI wrapper? (Estimated: 2-3 hours)
2. **Vercel Postgres Setup**: Do we have Vercel team access to provision the database?
3. **Google Cloud Project**: Do we have an existing GCP project for Translate API, or create new?
4. **Urdu Translation Review**: Who is the native Urdu speaker for final UI/translation review?

## Next Phase

All research questions resolved. Ready to proceed to **Phase 1: Data Model & Contracts**.
