import { NextRequest, NextResponse } from 'next/server';
export async function POST(req:NextRequest){const body=await req.json();return NextResponse.json({ok:true,id:crypto.randomUUID(),message:'Alert rule accepted for persistence by the production notification worker.',rule:body});}
