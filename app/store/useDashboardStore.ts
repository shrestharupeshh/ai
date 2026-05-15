'use client';
import { create } from 'zustand';
type Layer='sst'|'precipitation'|'wind'|'pressure'|'cloud';
type Store={layer:Layer;expert:boolean;region:string;setLayer:(layer:Layer)=>void;setExpert:(expert:boolean)=>void;setRegion:(region:string)=>void};
export const useDashboardStore=create<Store>((set)=>({layer:'sst',expert:false,region:'nepal',setLayer:(layer)=>set({layer}),setExpert:(expert)=>set({expert}),setRegion:(region)=>set({region})}));
