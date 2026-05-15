'use client';
import { QueryClient, QueryClientProvider } from '@tanstack/react-query';
import { ThemeProvider } from 'next-themes';
import { useState } from 'react';
export function Providers({children}:{children:React.ReactNode}){const [client]=useState(()=>new QueryClient({defaultOptions:{queries:{staleTime:5*60_000,refetchInterval:15*60_000,retry:1}}}));return <ThemeProvider attribute="class" defaultTheme="dark"><QueryClientProvider client={client}>{children}</QueryClientProvider></ThemeProvider>}
