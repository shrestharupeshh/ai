import { NextResponse } from 'next/server';
import { getEnsoStatus } from '@/app/lib/climate';
export const revalidate=1800;
export async function GET(){try{return NextResponse.json(await getEnsoStatus());}catch(error){return NextResponse.json({error:'ENSO status unavailable',detail:error instanceof Error?error.message:'unknown'},{status:503});}}
