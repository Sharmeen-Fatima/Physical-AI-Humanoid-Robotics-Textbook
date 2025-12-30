# Data Model: RAG Chatbot Frontend Integration

**Feature**: RAG Chatbot Frontend Integration
**Date**: 2025-12-25
**Status**: Complete

## Overview

This document defines the database schema and entity relationships for user authentication, chat session management, and translation caching. The data model supports PostgreSQL (production) with Prisma ORM for type-safe database access.

## Entity Relationship Diagram

```
┌──────────────┐
│    User      │
└──────┬───────┘
       │ 1
       │
       │ *
┌──────▼───────────┐
│  ChatSession     │
└──────┬───────────┘
       │ 1
       │
       │ *
┌──────▼───────────┐
│  ChatMessage     │
└──────────────────┘

┌───────────────────┐
│ TranslationCache  │  (independent)
└───────────────────┘
```

## Entities

### 1. User

Represents an authenticated user of the chatbot system.

**Attributes**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY | Unique identifier |
| email | VARCHAR(255) | UNIQUE, NOT NULL | User email (used for login) |
| password_hash | VARCHAR(255) | NOT NULL | Bcrypt-hashed password (10 salt rounds) |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Account creation timestamp |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update timestamp |
| language_preference | VARCHAR(10) | DEFAULT 'en' | Preferred language code ('en' or 'ur') |
| is_active | BOOLEAN | DEFAULT true | Account active status (for soft deletes) |

**Indexes**:
- `idx_users_email` (UNIQUE) on `email` for fast login lookups
- `idx_users_created_at` on `created_at` for analytics

**Validation Rules**:
- `email`: Must match regex `/^[^\s@]+@[^\s@]+\.[^\s@]+$/`
- `password_hash`: Bcrypt format (60 characters, starts with `$2a$` or `$2b$`)
- `language_preference`: Must be one of `['en', 'ur']`

**Example**:
```sql
INSERT INTO users (id, email, password_hash, language_preference)
VALUES (
  '550e8400-e29b-41d4-a716-446655440000',
  'student@example.com',
  '$2b$10$N9qo8uLOickgx2ZMRZoMyeIjZAgcfl7p92ldGxad68LJZdL17lhWy',
  'en'
);
```

---

### 2. ChatSession

Represents a conversation session between a user and the chatbot. Sessions group related messages together.

**Attributes**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY | Unique session identifier |
| user_id | UUID | FOREIGN KEY (users.id), NOT NULL | Owner of this session |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Session start time |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last message timestamp |
| title | VARCHAR(255) | NULL | Optional session title (e.g., first question) |
| is_active | BOOLEAN | DEFAULT true | Whether session is still active |
| message_count | INTEGER | DEFAULT 0 | Cached count of messages in session |

**Relationships**:
- `user_id` → `users.id` (Many-to-One: each session belongs to one user)
- One ChatSession has many ChatMessages (One-to-Many)

**Indexes**:
- `idx_sessions_user_id` on `user_id` for fetching user's sessions
- `idx_sessions_updated_at` on `updated_at` for sorting recent sessions

**Constraints**:
- ON DELETE CASCADE: If user is deleted, all their sessions are deleted

**Example**:
```sql
INSERT INTO chat_sessions (id, user_id, title)
VALUES (
  '7c9e6679-7425-40de-944b-e07fc1f90ae7',
  '550e8400-e29b-41d4-a716-446655440000',
  'What is physical AI?'
);
```

---

### 3. ChatMessage

Represents a single message within a chat session (either user question or bot response).

**Attributes**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY | Unique message identifier |
| session_id | UUID | FOREIGN KEY (chat_sessions.id), NOT NULL | Parent session |
| role | VARCHAR(10) | NOT NULL | Message sender: 'user' or 'assistant' |
| content | TEXT | NOT NULL | Message text content |
| citations | JSONB | NULL | Array of citation objects (bot messages only) |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Message timestamp |
| translated_content | TEXT | NULL | Translated version of content (if language != 'en') |
| language | VARCHAR(10) | DEFAULT 'en' | Language of the message ('en' or 'ur') |

**Relationships**:
- `session_id` → `chat_sessions.id` (Many-to-One: each message belongs to one session)

**Indexes**:
- `idx_messages_session_id` on `session_id` for fetching session history
- `idx_messages_created_at` on `created_at` for ordering messages chronologically

**Constraints**:
- ON DELETE CASCADE: If session is deleted, all messages are deleted
- `role` ENUM CHECK: Must be 'user' or 'assistant'

**Citations Schema** (JSONB):
```json
[
  {
    "url": "https://book.com/chapter-1",
    "section": "Introduction to Physical AI",
    "score": 0.873
  },
  {
    "url": "https://book.com/chapter-2",
    "section": "Components of Humanoid Robots",
    "score": 0.821
  }
]
```

**Example**:
```sql
INSERT INTO chat_messages (id, session_id, role, content, citations, language)
VALUES (
  '9b1deb4d-3b7d-4bad-9bdd-2b0d7b3dcb6d',
  '7c9e6679-7425-40de-944b-e07fc1f90ae7',
  'user',
  'What are the main components of a humanoid robot?',
  NULL,
  'en'
),
(
  'a3bb189e-8bf9-3888-9912-ace4e6543002',
  '7c9e6679-7425-40de-944b-e07fc1f90ae7',
  'assistant',
  'The main components include sensors, actuators, control systems, and power systems...',
  '[{"url":"https://book.com/chapter-3","section":"Robot Components","score":0.873}]'::jsonb,
  'en'
);
```

---

### 4. TranslationCache

Caches translated text to reduce Google Translate API costs. Stores source text hash and target language translation.

**Attributes**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY | Unique cache entry identifier |
| source_text_hash | VARCHAR(64) | NOT NULL | SHA-256 hash of source text |
| source_language | VARCHAR(10) | NOT NULL | Source language code ('en') |
| target_language | VARCHAR(10) | NOT NULL | Target language code ('ur') |
| source_text | TEXT | NOT NULL | Original text (for reference) |
| translated_text | TEXT | NOT NULL | Translated text |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Cache entry creation time |
| hit_count | INTEGER | DEFAULT 0 | Number of times this cache was used |
| last_used_at | TIMESTAMP | NULL | Last time this translation was retrieved |

**Indexes**:
- `idx_translation_cache_hash` (UNIQUE) on `(source_text_hash, source_language, target_language)` for fast lookups
- `idx_translation_cache_created_at` on `created_at` for cache eviction policies

**Cache Strategy**:
- Store indefinitely (or until manual purge)
- LRU eviction if cache grows > 100,000 entries
- Hash function: SHA-256 of normalized source text (trimmed, lowercased)

**Example**:
```sql
INSERT INTO translation_cache (id, source_text_hash, source_language, target_language, source_text, translated_text)
VALUES (
  '1e4d2d84-73b1-4e7a-9c2f-5f8e3d9a6b12',
  'a1b2c3d4e5f6...',  -- SHA-256 of "What is physical AI?"
  'en',
  'ur',
  'What is physical AI?',
  'فزیکل AI کیا ہے؟'
);
```

---

## Prisma Schema

```prisma
// prisma/schema.prisma

generator client {
  provider = "prisma-client-js"
}

datasource db {
  provider = "postgresql"
  url      = env("DATABASE_URL")
}

model User {
  id                  String        @id @default(uuid())
  email               String        @unique @db.VarChar(255)
  passwordHash        String        @map("password_hash") @db.VarChar(255)
  createdAt           DateTime      @default(now()) @map("created_at")
  updatedAt           DateTime      @updatedAt @map("updated_at")
  languagePreference  String        @default("en") @map("language_preference") @db.VarChar(10)
  isActive            Boolean       @default(true) @map("is_active")

  chatSessions        ChatSession[]

  @@index([email], name: "idx_users_email")
  @@index([createdAt], name: "idx_users_created_at")
  @@map("users")
}

model ChatSession {
  id            String        @id @default(uuid())
  userId        String        @map("user_id")
  createdAt     DateTime      @default(now()) @map("created_at")
  updatedAt     DateTime      @updatedAt @map("updated_at")
  title         String?       @db.VarChar(255)
  isActive      Boolean       @default(true) @map("is_active")
  messageCount  Int           @default(0) @map("message_count")

  user          User          @relation(fields: [userId], references: [id], onDelete: Cascade)
  messages      ChatMessage[]

  @@index([userId], name: "idx_sessions_user_id")
  @@index([updatedAt], name: "idx_sessions_updated_at")
  @@map("chat_sessions")
}

model ChatMessage {
  id                  String      @id @default(uuid())
  sessionId           String      @map("session_id")
  role                String      @db.VarChar(10)  // 'user' or 'assistant'
  content             String      @db.Text
  citations           Json?       // JSONB array of citation objects
  createdAt           DateTime    @default(now()) @map("created_at")
  translatedContent   String?     @map("translated_content") @db.Text
  language            String      @default("en") @db.VarChar(10)

  session             ChatSession @relation(fields: [sessionId], references: [id], onDelete: Cascade)

  @@index([sessionId], name: "idx_messages_session_id")
  @@index([createdAt], name: "idx_messages_created_at")
  @@map("chat_messages")
}

model TranslationCache {
  id                String    @id @default(uuid())
  sourceTextHash    String    @map("source_text_hash") @db.VarChar(64)
  sourceLanguage    String    @map("source_language") @db.VarChar(10)
  targetLanguage    String    @map("target_language") @db.VarChar(10)
  sourceText        String    @map("source_text") @db.Text
  translatedText    String    @map("translated_text") @db.Text
  createdAt         DateTime  @default(now()) @map("created_at")
  hitCount          Int       @default(0) @map("hit_count")
  lastUsedAt        DateTime? @map("last_used_at")

  @@unique([sourceTextHash, sourceLanguage, targetLanguage], name: "idx_translation_cache_hash")
  @@index([createdAt], name: "idx_translation_cache_created_at")
  @@map("translation_cache")
}
```

## Database Migrations

### Initial Migration

```sql
-- CreateTable
CREATE TABLE "users" (
    "id" UUID NOT NULL DEFAULT gen_random_uuid(),
    "email" VARCHAR(255) NOT NULL,
    "password_hash" VARCHAR(255) NOT NULL,
    "created_at" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updated_at" TIMESTAMP(3) NOT NULL,
    "language_preference" VARCHAR(10) NOT NULL DEFAULT 'en',
    "is_active" BOOLEAN NOT NULL DEFAULT true,

    CONSTRAINT "users_pkey" PRIMARY KEY ("id")
);

CREATE TABLE "chat_sessions" (
    "id" UUID NOT NULL DEFAULT gen_random_uuid(),
    "user_id" UUID NOT NULL,
    "created_at" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updated_at" TIMESTAMP(3) NOT NULL,
    "title" VARCHAR(255),
    "is_active" BOOLEAN NOT NULL DEFAULT true,
    "message_count" INTEGER NOT NULL DEFAULT 0,

    CONSTRAINT "chat_sessions_pkey" PRIMARY KEY ("id")
);

CREATE TABLE "chat_messages" (
    "id" UUID NOT NULL DEFAULT gen_random_uuid(),
    "session_id" UUID NOT NULL,
    "role" VARCHAR(10) NOT NULL,
    "content" TEXT NOT NULL,
    "citations" JSONB,
    "created_at" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "translated_content" TEXT,
    "language" VARCHAR(10) NOT NULL DEFAULT 'en',

    CONSTRAINT "chat_messages_pkey" PRIMARY KEY ("id")
);

CREATE TABLE "translation_cache" (
    "id" UUID NOT NULL DEFAULT gen_random_uuid(),
    "source_text_hash" VARCHAR(64) NOT NULL,
    "source_language" VARCHAR(10) NOT NULL,
    "target_language" VARCHAR(10) NOT NULL,
    "source_text" TEXT NOT NULL,
    "translated_text" TEXT NOT NULL,
    "created_at" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "hit_count" INTEGER NOT NULL DEFAULT 0,
    "last_used_at" TIMESTAMP(3),

    CONSTRAINT "translation_cache_pkey" PRIMARY KEY ("id")
);

-- CreateIndex
CREATE UNIQUE INDEX "users_email_key" ON "users"("email");
CREATE INDEX "idx_users_email" ON "users"("email");
CREATE INDEX "idx_users_created_at" ON "users"("created_at");

CREATE INDEX "idx_sessions_user_id" ON "chat_sessions"("user_id");
CREATE INDEX "idx_sessions_updated_at" ON "chat_sessions"("updated_at");

CREATE INDEX "idx_messages_session_id" ON "chat_messages"("session_id");
CREATE INDEX "idx_messages_created_at" ON "chat_messages"("created_at");

CREATE UNIQUE INDEX "idx_translation_cache_hash" ON "translation_cache"("source_text_hash", "source_language", "target_language");
CREATE INDEX "idx_translation_cache_created_at" ON "translation_cache"("created_at");

-- AddForeignKey
ALTER TABLE "chat_sessions" ADD CONSTRAINT "chat_sessions_user_id_fkey" FOREIGN KEY ("user_id") REFERENCES "users"("id") ON DELETE CASCADE ON UPDATE CASCADE;

ALTER TABLE "chat_messages" ADD CONSTRAINT "chat_messages_session_id_fkey" FOREIGN KEY ("session_id") REFERENCES "chat_sessions"("id") ON DELETE CASCADE ON UPDATE CASCADE;
```

## Data Access Patterns

### Common Queries

1. **Create New User**
   ```ts
   const user = await prisma.user.create({
     data: {
       email: 'student@example.com',
       passwordHash: bcrypt.hashSync('password', 10),
       languagePreference: 'en',
     },
   });
   ```

2. **Find User by Email**
   ```ts
   const user = await prisma.user.findUnique({
     where: { email: 'student@example.com' },
   });
   ```

3. **Create Chat Session**
   ```ts
   const session = await prisma.chatSession.create({
     data: {
       userId: user.id,
       title: 'New Conversation',
     },
   });
   ```

4. **Add Message to Session**
   ```ts
   const message = await prisma.chatMessage.create({
     data: {
       sessionId: session.id,
       role: 'user',
       content: 'What is physical AI?',
       language: 'en',
     },
   });
   ```

5. **Get Chat History**
   ```ts
   const messages = await prisma.chatMessage.findMany({
     where: { sessionId: session.id },
     orderBy: { createdAt: 'asc' },
   });
   ```

6. **Lookup Translation Cache**
   ```ts
   const hash = crypto.createHash('sha256').update('What is physical AI?').digest('hex');
   const cached = await prisma.translationCache.findUnique({
     where: {
       idx_translation_cache_hash: {
         sourceTextHash: hash,
         sourceLanguage: 'en',
         targetLanguage: 'ur',
       },
     },
   });
   ```

## Performance Considerations

1. **Connection Pooling**: Use Prisma Data Proxy or PgBouncer for Vercel serverless (limit: 5-10 connections per function)
2. **Query Optimization**: Indexes on foreign keys, frequently queried fields (email, session_id)
3. **JSONB Queries**: For citations, use `@>` operator for containment queries if needed
4. **Cache Expiration**: Consider TTL for translation cache (e.g., 90 days) to limit growth
5. **Pagination**: Implement cursor-based pagination for chat sessions and message history

## Security Notes

- **Password Hashing**: Always use bcrypt with at least 10 salt rounds
- **SQL Injection**: Prisma uses parameterized queries (safe by default)
- **Data Encryption**: PostgreSQL supports column-level encryption if needed (e.g., for PII)
- **Soft Deletes**: Use `isActive` flag instead of hard deletes for audit trail
