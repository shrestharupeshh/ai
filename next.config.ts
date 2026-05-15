import type { NextConfig } from 'next';
const nextConfig: NextConfig = { experimental: { ppr: false }, images: { remotePatterns: [{ protocol: 'https', hostname: '**' }] } };
export default nextConfig;
