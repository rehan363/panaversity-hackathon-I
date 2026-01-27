# Tasks: 003-authentication

## Task List

### T1: Setup Database & Drizzle (100% Done Criteria: Migrations ran successfully)
- [ ] Install `drizzle-orm @neondatabase/serverless drizzle-kit`.
- [ ] Create `src/lib/db/schema.ts` with auth tables.
- [ ] Create `drizzle.config.ts`.
- [ ] Run `npx drizzle-kit generate` and `npx drizzle-kit migrate`.

### T2: Auth Server Implementation (100% Done Criteria: API endpoints return 200)
- [ ] Install `better-auth`.
- [ ] Create `src/lib/auth.ts` configuration.
- [ ] Set up server route (e.g., `api/auth/[...better-auth]`).
- [ ] Test signup endpoint with Postman/Curl.

### T3: Frontend Context & Hooks (100% Done Criteria: useSession works in components)
- [ ] Install `better-auth/react`.
- [ ] Create `src/theme/Root.tsx` (Swizzled) and wrap with AuthProvider.
- [ ] Implement `useAuth` hook for easy access to user data.

### T4: Navbar Auth UI (100% Done Criteria: Login button appears/changes to profile)
- [ ] Swizzle Navbar and add `AuthButton` component.
- [ ] Implement conditional rendering: `Sign In` vs `User Profile`.

### T5: Signup & Onboarding Flow (100% Done Criteria: Background data saved in DB)
- [ ] Create `/signup` page with background profile sliders/options.
- [ ] Implement `signUp.email` call with custom user data.
- [ ] Verify data persistence in Neon.

## Testing Tasks
- [ ] **Test T1**: Verify tables `user`, `session`, `account` exist in Neon dashboard.
- [ ] **Test T2**: Sign up with a test email, verify record in `user` table.
- [ ] **Test T3**: Log in, verify `better-auth.session-token` cookie in DevTools.
- [ ] **Test T4**: Click logout, verify redirect and cookie deletion.
