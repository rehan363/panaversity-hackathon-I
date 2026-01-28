import React from 'react';
import { useSession, signOut } from '../../lib/auth-client';
import Link from '@docusaurus/Link';

export default function AuthButton() {
    const { data: session, isPending } = useSession();

    if (isPending) {
        return <div style={{ width: '40px' }} />;
    }

    if (session) {
        return (
            <div style={{ display: 'flex', alignItems: 'center', gap: '1rem' }}>
                <span style={{ fontSize: '0.85rem', fontWeight: 600 }}>
                    {session.user.name}
                </span>
                <button
                    onClick={() => signOut()}
                    className="button button--secondary button--sm"
                >
                    Logout
                </button>
            </div>
        );
    }

    return (
        <div style={{ display: 'flex', gap: '0.5rem' }}>
            <Link className="button button--outline button--primary button--sm" to="/login">
                Login
            </Link>
            <Link className="button button--primary button--sm" to="/signup">
                Sign Up
            </Link>
        </div>
    );
}
