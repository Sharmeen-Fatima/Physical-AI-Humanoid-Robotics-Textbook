# Quickstart Guide: RAG Chatbot Frontend Integration

**Feature**: RAG Chatbot Frontend Integration
**Date**: 2025-12-25
**Estimated Setup Time**: 30-45 minutes

## Prerequisites

- **Node.js**: 20.x or higher ([Download](https://nodejs.org/))
- **Docker**: Latest version ([Download](https://www.docker.com/get-started))
- **Git**: Latest version
- **Python**: 3.10+ (for RAG backend)
- **Code Editor**: VS Code recommended with Prisma extension

## Part 1: Local Development Setup

### Step 1: Clone and Install Dependencies

```bash
# Navigate to project root
cd Physical_AI_Humanoid_Robotics_Book

# Install frontend dependencies
npm install

# Install additional dependencies for chatbot features
npm install axios react-query i18next react-i18next jwt-simple bcrypt

# Install API backend dependencies
cd api
npm init -y
npm install express jsonwebtoken bcrypt @prisma/client axios dotenv winston express-rate-limit cookie-parser cors
npm install -D typescript @types/node @types/express @types/jsonwebtoken @types/bcrypt ts-node nodemon prisma
npx tsc --init
cd ..
```

### Step 2: Start PostgreSQL Database

```bash
# Create docker-compose.yml in project root
cat > docker-compose.yml <<'EOF'
version: '3.8'
services:
  postgres:
    image: postgres:15
    container_name: chatbot_postgres
    environment:
      POSTGRES_DB: chatbot_dev
      POSTGRES_USER: dev
      POSTGRES_PASSWORD: dev_password
    ports:
      - "5432:5432"
    volumes:
      - pgdata:/var/lib/postgresql/data

volumes:
  pgdata:
EOF

# Start PostgreSQL
docker-compose up -d

# Verify it's running
docker ps | grep postgres
```

### Step 3: Configure Environment Variables

```bash
# Create .env file in /api directory
cat > api/.env <<'EOF'
# Database
DATABASE_URL="postgresql://dev:dev_password@localhost:5432/chatbot_dev?schema=public"

# JWT Secret (generate with: openssl rand -base64 32)
JWT_SECRET="your-super-secret-jwt-key-change-in-production"

# Python RAG Backend URL
PYTHON_RAG_URL="http://localhost:8000"

# Google Translate API (get from Google Cloud Console)
GOOGLE_TRANSLATE_API_KEY="your-google-translate-api-key"

# Environment
NODE_ENV="development"

# Server
PORT=3001
EOF

# IMPORTANT: Add .env to .gitignore
echo "api/.env" >> .gitignore
```

### Step 4: Initialize Database with Prisma

```bash
cd api

# Initialize Prisma (skip if schema.prisma already exists)
npx prisma init

# Copy the Prisma schema from data-model.md to prisma/schema.prisma
# (Or create it manually based on the schema in data-model.md)

# Generate Prisma Client
npx prisma generate

# Create and apply initial migration
npx prisma migrate dev --name init

# (Optional) Seed database with test user
npx prisma db seed

cd ..
```

**Prisma Seed Script** (optional - create `api/prisma/seed.ts`):

```typescript
import { PrismaClient } from '@prisma/client';
import bcrypt from 'bcrypt';

const prisma = new PrismaClient();

async function main() {
  const passwordHash = await bcrypt.hash('testpass123', 10);

  await prisma.user.create({
    data: {
      email: 'test@example.com',
      passwordHash,
      languagePreference: 'en',
    },
  });

  console.log('✅ Seed data created');
}

main()
  .catch((e) => console.error(e))
  .finally(() => prisma.$disconnect());
```

### Step 5: Start Python RAG Backend

```bash
# Navigate to Python backend directory
cd backend

# Activate virtual environment (if exists)
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install FastAPI and uvicorn (if not already installed)
pip install fastapi uvicorn

# Create minimal FastAPI server (backend/api/server.py)
# (Based on research.md findings - reuses existing RAG components)

# Start the server on port 8000
uvicorn api.server:app --reload --port 8000

# Verify it's running
curl http://localhost:8000/query
```

**Minimal FastAPI Server** (create `backend/api/server.py`):

```python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from src.retrieval.vector_search import VectorSearch
from src.generation.gemini_generator import GeminiGenerator

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3001"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class QueryRequest(BaseModel):
    question: str
    session_id: str | None = None
    top_k: int = 5

@app.post("/query")
async def query(req: QueryRequest):
    try:
        # Use existing RAG components
        retriever = VectorSearch()
        generator = GeminiGenerator()

        results = retriever.search(req.question, top_k=req.top_k)
        answer = generator.generate(req.question, results)

        return {
            "answer": answer["text"],
            "sources": answer["sources"],
            "grounded": True,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health():
    return {"status": "ok"}
```

### Step 6: Start Node.js API Backend

```bash
# Navigate to API directory
cd api

# Create basic server.ts (see below)

# Start development server with nodemon
npx nodemon --exec ts-node src/server.ts

# Or add to package.json scripts:
# "dev": "nodemon --exec ts-node src/server.ts"
# Then run: npm run dev
```

**Basic API Server** (create `api/src/server.ts`):

```typescript
import express from 'express';
import cookieParser from 'cookie-parser';
import cors from 'cors';
import dotenv from 'dotenv';

dotenv.config();

const app = express();
const PORT = process.env.PORT || 3001;

// Middleware
app.use(cors({ origin: 'http://localhost:3000', credentials: true }));
app.use(express.json());
app.use(cookieParser());

// Routes (import from separate files)
import authRoutes from './routes/auth';
import chatRoutes from './routes/chat';
import translateRoutes from './routes/translate';

app.use('/api/auth', authRoutes);
app.use('/api/chat', chatRoutes);
app.use('/api/translate', translateRoutes);

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'ok' });
});

app.listen(PORT, () => {
  console.log(`✅ API server running on http://localhost:${PORT}`);
});
```

### Step 7: Start Docusaurus Frontend

```bash
# Navigate to project root
cd ..

# Start Docusaurus development server
npm start

# Open browser to http://localhost:3000
```

## Part 2: Testing the Integration

### Test 1: User Signup

```bash
curl -X POST http://localhost:3001/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test2@example.com","password":"SecurePass123"}'
```

**Expected Response**:
```json
{
  "success": true,
  "message": "Account created successfully",
  "user": {
    "id": "uuid-here",
    "email": "test2@example.com",
    "createdAt": "2025-12-25T...",
    "languagePreference": "en"
  }
}
```

### Test 2: User Login

```bash
curl -X POST http://localhost:3001/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test2@example.com","password":"SecurePass123"}' \
  -c cookies.txt
```

### Test 3: Send Chat Message (authenticated)

```bash
curl -X POST http://localhost:3001/api/chat/message \
  -H "Content-Type: application/json" \
  -b cookies.txt \
  -d '{"content":"What is physical AI?"}'
```

**Expected Response**:
```json
{
  "success": true,
  "sessionId": "session-uuid",
  "userMessage": {...},
  "assistantMessage": {
    "id": "message-uuid",
    "role": "assistant",
    "content": "Physical AI refers to...",
    "citations": [
      {"url": "https://...", "section": "Chapter 1", "score": 0.85}
    ]
  }
}
```

### Test 4: Translation

```bash
curl -X POST http://localhost:3001/api/translate \
  -H "Content-Type: application/json" \
  -b cookies.txt \
  -d '{"text":"Hello","sourceLanguage":"en","targetLanguage":"ur"}'
```

## Part 3: Frontend Component Integration

### Add ChatBot to Docusaurus

1. **Swizzle Root Component**:

```bash
npm run swizzle @docusaurus/theme-classic Root -- --eject
```

2. **Modify `src/theme/Root.tsx`**:

```tsx
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

3. **Create ChatBot Component** (`src/components/ChatBot/index.tsx`):

```tsx
import React, { useState } from 'react';
import './styles.css';

export default function ChatBot() {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <>
      {!isOpen && (
        <button
          className="chat-button"
          onClick={() => setIsOpen(true)}
          aria-label="Open chatbot"
        >
          💬
        </button>
      )}
      {isOpen && (
        <div className="chat-panel">
          <div className="chat-header">
            <h3>Ask the Book</h3>
            <button onClick={() => setIsOpen(false)}>✕</button>
          </div>
          <div className="chat-messages">
            {/* Message list */}
          </div>
          <div className="chat-input">
            {/* Input form */}
          </div>
        </div>
      )}
    </>
  );
}
```

## Part 4: Deployment to Vercel

### Prepare for Production

1. **Set Production Environment Variables** in Vercel Dashboard:
   - `DATABASE_URL`: Vercel Postgres connection string
   - `JWT_SECRET`: Strong random secret (generate with `openssl rand -base64 32`)
   - `PYTHON_RAG_URL`: URL of deployed Python backend
   - `GOOGLE_TRANSLATE_API_KEY`: Google Cloud API key

2. **Configure Vercel Serverless Functions**:

Create `vercel.json`:

```json
{
  "builds": [
    {
      "src": "api/src/**/*.ts",
      "use": "@vercel/node"
    }
  ],
  "routes": [
    {
      "src": "/api/(.*)",
      "dest": "/api/src/$1"
    }
  ]
}
```

3. **Deploy**:

```bash
# Install Vercel CLI
npm i -g vercel

# Login
vercel login

# Deploy
vercel --prod
```

## Troubleshooting

### Issue: Database connection fails

**Solution**: Ensure PostgreSQL is running and connection string is correct.

```bash
docker ps  # Check if postgres container is running
psql -h localhost -U dev -d chatbot_dev  # Test connection
```

### Issue: Python RAG backend not responding

**Solution**: Check if the backend is running and accessible:

```bash
curl http://localhost:8000/health
```

### Issue: JWT authentication not working

**Solution**: Ensure `JWT_SECRET` is set and cookies are being sent:

```bash
# Check if cookie is set
curl -v http://localhost:3001/api/auth/login ...
# Look for Set-Cookie header
```

### Issue: Translation API returns 503

**Solution**: Verify Google Translate API key is valid and project has billing enabled.

## Next Steps

1. **Implement Frontend Components**: Build React components for chat UI (see `plan.md` for component structure)
2. **Add E2E Tests**: Set up Playwright for end-to-end testing
3. **Configure i18next**: Set up Urdu translation files
4. **Review Security**: Ensure all secrets are in environment variables

## Useful Commands

```bash
# Reset database
npx prisma migrate reset

# View database in Prisma Studio
npx prisma studio

# Run migrations
npx prisma migrate dev

# Generate Prisma Client after schema changes
npx prisma generate

# Check TypeScript compilation
npx tsc --noEmit

# Build frontend for production
npm run build
```

## Resources

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [Prisma Documentation](https://www.prisma.io/docs)
- [Vercel Deployment Guide](https://vercel.com/docs)
- [Google Translate API Docs](https://cloud.google.com/translate/docs)
- [React Query Documentation](https://tanstack.com/query/latest)

---

**Setup Complete!** You should now have:
- ✅ PostgreSQL database running
- ✅ Python RAG backend on port 8000
- ✅ Node.js API backend on port 3001
- ✅ Docusaurus frontend on port 3000
- ✅ Basic authentication and chat endpoints working

Ready to start building the chatbot UI components! 🚀
