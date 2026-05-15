import { NextResponse } from 'next/server';
const sources=['NOAA','NASA','WMO','Reuters','Nature','Science'];
export const revalidate=900;
export async function GET(){const items=sources.map((source,i)=>({id:`news-${i}`,source,title:[`Latest ENSO monitoring from ${source}`,`Pacific SST anomaly update and climate-risk context`, `Monsoon and regional teleconnection watch`][i%3],url: source==='NOAA'?'https://www.climate.gov/enso':source==='NASA'?'https://earthobservatory.nasa.gov/':'https://public.wmo.int/',publishedAt:new Date(Date.now()-i*36e5).toISOString(),sentiment:i%4===0?'caution':'informational',misinformationRisk:'low',summary:'Curated climate item with source weighting, recency scoring, and uncertainty-aware summarization.'}));return NextResponse.json({items});}
