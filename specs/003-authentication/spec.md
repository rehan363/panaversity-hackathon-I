# Specification: 003-authentication

## Overview
Implement a robust authentication and onboarding system for the Physical AI & Robotics Textbook platform using Better-Auth, Drizzle ORM, and Neon Postgres.

## Goals
- Secure user signup and signin.
- Collect user background profile (hardware/software knowledge) during signup.
- Manage user sessions persistently.
- Provide a responsive UI for auth in the Docusaurus navbar.

## User Stories
- **US1**: As a new student, I want to sign up with my email and name so that I can track my laboratory progress.
- **US2**: As a returning student, I want to log in to my account so that I can access my personalized textbook view.
- **US3**: As a student during signup, I want to specify my background (e.g., Python, ROS 2, Hardware experience) so the content can be personalized for me.
- **US4**: As an authenticated user, I want to see my profile icon in the navbar and be able to log out.

## Requirements

### R1: Authentication Engine
- Use **Better-Auth** as the core authentication library.
- Support **Email & Password** provider.
- Store sensitive data (passwords) using secure hashing (Argon2/Bcrypt) via Better-Auth defaults.

### R2: Database
- Use **Neon Serverless Postgres** as the primary relational database.
- Use **Drizzle ORM** for schema definition and migrations.
- Primary tables: `user`, `session`, `account`, `verification`.

### R3: Custom User Profile Fields
- `role`: Default to 'student'.
- `background`: JSON or separate fields for software/hardware experience level.
- `onboarding_complete`: Boolean to track if profile data was collected.

### R4: Frontend Integration
- Integrate with **Docusaurus v3**.
- Swizzle the `Navbar` to include an `AuthButton` component.
- Swizzle the `Root` component to wrap the app in an `AuthProvider`.

## Constraints
- Must not block static page generation (SSG).
- Must handle Neon database cold starts gracefully.
- Must follow the **Security & Privacy** principles defined in the Constitution.

## Success Criteria
- [ ] User can successfully sign up and their data is visible in Neon Postgres.
- [ ] User can log in and a valid session is created in the browser (HTTP-only cookies).
- [ ] Profile data is editable and persistent.
- [ ] Navbar updates dynamically based on auth state.
- [ ] "Logout" clears the session and redirects to home.
