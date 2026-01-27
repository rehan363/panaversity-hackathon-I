# Plan: 003-authentication

## Architecture Overview
The authentication system will be built using a decoupled architecture where Better-Auth manages the session lifecycle and Drizzle ORM interfaces with Neon Postgres.

## Components

### 1. Backend Service (Next.js or Express/Node.js)
- **Decision**: Better-Auth requires a Node.js runtime. We will implement it either in a dedicated Next.js API route or an Express server.
- **Rationale**: Better-Auth's server-side logic handles cookie management and validation securely.

### 2. Database Layer
- **Neon Postgres**: Serverless database for user data.
- **Drizzle DMM**: schema-first approach for type-safe database access.

### 3. Frontend Integration (Docusaurus)
- **Better-Auth Client SDK**: Used in the browser to interact with the auth server.
- **React Context**: To provide auth state (`session`, `isLoading`) to all Docusaurus components.

## Implementation Steps

### Phase 1: Infrastructure
- [ ] Initialize Neon Postgres project.
- [ ] Set up Drizzle config and schema.
- [ ] Run first migration to create auth tables.

### Phase 2: Auth Server
- [ ] Install Better-Auth and dependencies.
- [ ] Configure `auth.ts` with Drizzle adapter and Email provider.
- [ ] Implement API routes for auth (signup, login, logout, session).

### Phase 3: Frontend Setup
- [ ] Install `@better-auth/react` in the Docusaurus project.
- [ ] Swizzle the `Root` component to add `AuthProvider`.
- [ ] Swizzle the `Navbar` to add the `AuthButton`.

### Phase 4: Personalization Onboarding
- [ ] Update `signUp` to include `background` fields.
- [ ] Implement a post-signup onboarding modal or page if fields are missing.

## Security Decisions
- **ADR 001: Session Management**: Use HTTP-only cookies for session storage to prevent XSS.
- **ADR 002: Secure Passwords**: Use Better-Auth's default Argon2/Bcrypt implementation.
- **ADR 003: CORS**: Explicitly define `trustedOrigins` to prevent CSRF.

## Risks & Mitigations
- **Neon Cold Starts**: Implement retry logic in the Drizzle client.
- **SSR Mismatch**: Ensure the Docusaurus build doesn't depend on client-side cookies for static rendering.
