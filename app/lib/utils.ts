import { clsx, type ClassValue } from 'clsx';
import { twMerge } from 'tailwind-merge';
export function cn(...inputs: ClassValue[]){return twMerge(clsx(inputs));}
export function clamp(value:number,min:number,max:number){return Math.min(max,Math.max(min,value));}
export function pct(value:number){return `${Math.round(value*100)}%`;}
