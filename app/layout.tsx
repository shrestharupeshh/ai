import type { Metadata } from 'next';
import './globals.css';
import { Providers } from './providers';
export const metadata: Metadata = { title: 'ENSO Intelligence — Advanced El Niño / La Niña Tracker', description: 'Live ENSO tracking, forecasts, historical analogs, and Nepal impact intelligence.' };
export default function RootLayout({children}:{children:React.ReactNode}){return <html lang="en" suppressHydrationWarning><body><Providers>{children}</Providers></body></html>}
