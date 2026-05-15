import { cn } from '@/app/lib/utils';
export function Card({className,...props}:React.HTMLAttributes<HTMLDivElement>){return <section className={cn('glass rounded-3xl p-5',className)} {...props}/>}
export function Badge({className,...props}:React.HTMLAttributes<HTMLSpanElement>){return <span className={cn('inline-flex items-center rounded-full border border-sky-300/20 bg-sky-300/10 px-3 py-1 text-xs font-semibold uppercase tracking-[.2em] text-sky-100',className)} {...props}/>}
