import React from 'react';
import Layout from '@theme/Layout';
import RegisterForm from '../components/Auth/RegisterForm';

export default function SignupPage() {
    return (
        <Layout title="Sign Up" description="Create your Physical AI Student account">
            <main style={{ padding: '2rem 0' }}>
                <RegisterForm />
            </main>
        </Layout>
    );
}
