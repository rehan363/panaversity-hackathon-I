import React from 'react';
import Layout from '@theme/Layout';
import LoginForm from '../components/Auth/LoginForm';

export default function LoginPage() {
    return (
        <Layout title="Login" description="Login to your Physical AI Student account">
            <main style={{ padding: '2rem 0' }}>
                <LoginForm />
            </main>
        </Layout>
    );
}
