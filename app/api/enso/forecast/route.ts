import { NextResponse } from 'next/server';
import { getForecast } from '@/app/lib/climate';
export const revalidate=3600;
export async function GET(){return NextResponse.json(await getForecast());}
