# Deployment Guide

This guide explains how to deploy your Physical AI Humanoid Robotics Book with the chatbot to production.

## Architecture Overview

Your application consists of three components:

1. **Frontend (Docusaurus)** - The book website
2. **API Server (Express/Node.js)** - Handles chatbot requests
3. **Python Backend** - RAG system with vector search and AI generation

## Deployment Options

### Option 1: Vercel + Railway (Recommended)

#### 1. Deploy Frontend to Vercel

1. Push your code to GitHub
2. Go to [Vercel Dashboard](https://vercel.com)
3. Click "New Project"
4. Import your GitHub repository
5. Vercel will auto-detect Docusaurus
6. **Important**: Add environment variable:
   - `REACT_APP_API_URL` = `https://your-api.vercel.app` (or Railway URL)
7. Deploy

#### 2. Deploy API Server to Vercel

**Option A: Deploy as separate Vercel project**

1. Create a new Vercel project
2. Set Root Directory to `api/`
3. Add environment variables:
   ```
   CORS_ORIGIN=https://your-book-site.vercel.app
   JWT_SECRET=your-jwt-secret-here
   PYTHON_RAG_URL=https://your-python-backend.railway.app
   NODE_ENV=production
   PORT=3001
   ```
4. Deploy

**Option B: Deploy as Vercel Serverless Function**

Create `api/vercel.json`:
```json
{
  "version": 2,
  "builds": [
    {
      "src": "src/server.ts",
      "use": "@vercel/node"
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "src/server.ts"
    }
  ]
}
```

#### 3. Deploy Python Backend to Railway

1. Go to [Railway.app](https://railway.app)
2. Create new project from GitHub repo
3. Set Root Directory to `backend/`
4. Add environment variables from `backend/.env`:
   ```
   COHERE_API_KEY=your-key
   GEMINI_API_KEY=your-key
   QDRANT_URL=your-url
   QDRANT_API_KEY=your-key
   QDRANT_COLLECTION_NAME=physical_ai_book
   BOOK_SITEMAP_URL=your-production-url/sitemap.xml
   BOOK_BASE_URL=your-production-url
   ```
5. Deploy

### Option 2: All-in-One Docker Deployment

Use the provided `docker-compose.yml` to deploy to:
- DigitalOcean App Platform
- AWS ECS
- Google Cloud Run
- Azure Container Apps

## Configuration Steps

### 1. Update Frontend API URL

The frontend needs to know your production API URL. You have two options:

**Option A: Environment Variable (Recommended)**

In Vercel, set:
```
REACT_APP_API_URL=https://your-api-url.com
```

**Option B: Update chatbotApi.ts directly**

Edit `src/api/chatbotApi.ts`:
```typescript
const API_BASE_URL = typeof window !== 'undefined'
  ? (window as any)._env_?.REACT_APP_API_URL || 'https://your-api-url.com'
  : 'https://your-api-url.com';
```

### 2. Update CORS Configuration

In `api/src/server.ts` or `api/.env`, update CORS_ORIGIN:
```
CORS_ORIGIN=https://your-book-site.vercel.app,https://physical-ai-humanoid-robotics-textb-pied.vercel.app
```

### 3. Update Python Backend URL

In `api/.env`:
```
PYTHON_RAG_URL=https://your-python-backend.railway.app
```

## Post-Deployment Checklist

- [ ] Frontend is accessible at Vercel URL
- [ ] API health check works: `https://your-api.vercel.app/api/health`
- [ ] CORS headers include your frontend URL
- [ ] Chatbot widget appears on the page
- [ ] Chatbot can send queries and receive responses
- [ ] Environment variables are set correctly
- [ ] API keys are secured (not committed to Git)

## Troubleshooting

### Chatbot shows "Failed to fetch"

1. Check browser console for CORS errors
2. Verify `REACT_APP_API_URL` is set correctly
3. Test API endpoint directly: `curl https://your-api.vercel.app/api/health`
4. Check CORS configuration includes your frontend URL

### API returns 500 errors

1. Check Vercel/Railway logs
2. Verify Python backend is accessible
3. Check all environment variables are set
4. Test Python backend health endpoint

### Python backend not responding

1. Check Railway deployment logs
2. Verify Python dependencies are installed
3. Test with: `curl https://your-python-backend.railway.app/health`

## Environment Variables Summary

### Frontend (Vercel)
```
REACT_APP_API_URL=https://your-api-url.com
```

### API Server (Vercel/Railway)
```
CORS_ORIGIN=https://your-frontend.vercel.app
JWT_SECRET=your-secret
PYTHON_RAG_URL=https://your-python-backend.railway.app
NODE_ENV=production
PORT=3001
```

### Python Backend (Railway)
```
COHERE_API_KEY=your-key
GEMINI_API_KEY=your-key
QDRANT_URL=your-url
QDRANT_API_KEY=your-key
QDRANT_COLLECTION_NAME=physical_ai_book
BOOK_SITEMAP_URL=your-url/sitemap.xml
BOOK_BASE_URL=your-url
```

## Quick Start (Vercel Only)

If you want the simplest deployment:

1. **Push to GitHub**
2. **Deploy Frontend to Vercel** - Auto-detects Docusaurus
3. **Deploy API to Vercel** - Set root directory to `api/`
4. **Set Environment Variables** in both projects
5. **Test** - Open your site and try the chatbot

Note: Vercel has limited support for Python processes. You may need to deploy the Python backend separately to Railway or switch to a Python-to-HTTP service.

## Alternative: Simplify Architecture

If deployment is complex, consider:

1. **Convert Python backend to HTTP API** - Use FastAPI
2. **Deploy Python as separate service** - Railway, Render, Fly.io
3. **Update API to call HTTP endpoint** - Instead of spawning Python process

This makes deployment much simpler across platforms.
