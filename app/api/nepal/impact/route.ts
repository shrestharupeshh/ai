import { NextRequest, NextResponse } from 'next/server';
import { getNepalImpact, nepalRegions } from '@/app/lib/climate';
export const revalidate=1800;
export async function GET(req:NextRequest){const region=req.nextUrl.searchParams.get('region')??'nepal';return NextResponse.json({regions:nepalRegions,impact:await getNepalImpact(region)});}
