import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import { neon } from "@neondatabase/serverless";
import { drizzle } from "drizzle-orm/neon-http";
import * as dotenv from "dotenv";

dotenv.config();

const sql = neon(process.env.NEON_DATABASE_URL!);
const db = drizzle(sql);

export const auth = betterAuth({
    database: drizzleAdapter(db, {
        provider: "pg",
    }),
    emailAndPassword: {
        enabled: true,
    },
    user: {
        additionalFields: {
            python_experience: {
                type: "number",
                defaultValue: 0,
            },
            hardware_experience: {
                type: "number",
                defaultValue: 0,
            },
            onboarding_complete: {
                type: "boolean",
                defaultValue: false,
            }
        }
    },
    // We can add social providers (GitHub, Google) here if needed later
});
