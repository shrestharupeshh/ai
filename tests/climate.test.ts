import assert from 'node:assert/strict';
import { getForecast, getNepalImpact, nepalRegions } from '../app/lib/climate.ts';
const forecast=await getForecast();assert.equal(forecast.points.length,24);for(const p of forecast.points){assert.ok(Math.abs(p.elNino+p.neutral+p.laNina-1)<0.00001,`probabilities sum for ${p.season}`)}
const kathmandu=await getNepalImpact('kathmandu');assert.equal(kathmandu.region.name,'Kathmandu');assert.ok(kathmandu.weakMonsoonProbability>=0&&kathmandu.weakMonsoonProbability<=1);assert.ok(nepalRegions.some(r=>r.name==='Sindhupalchok'));
console.log('climate computations ok');
