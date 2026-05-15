import { NextResponse } from 'next/server';
import { getEnsoStatus } from '@/app/lib/climate';
export async function GET(){const status=await getEnsoStatus();const csv=['month,nino34,oni,soi',...status.trend.map(r=>`${r.month},${r.nino34},${r.oni},${r.soi}`)].join('\n');return new NextResponse(csv,{headers:{'content-type':'text/csv','content-disposition':'attachment; filename="enso-intelligence.csv"'}});}
