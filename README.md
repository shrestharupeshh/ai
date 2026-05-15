# ENSO Intelligence — Advanced El Niño / La Niña Tracker

A production-grade Next.js 15 climate intelligence dashboard for live ENSO monitoring, model forecasts, historical analogs, regional teleconnections, alerting, and a dedicated Nepal impact engine.

## Stack
- Next.js App Router + TypeScript + Tailwind CSS
- shadcn-style reusable components, Framer Motion, Recharts, Zustand, TanStack Query, Lucide icons
- API routes for ENSO status, forecasts, Nepal impacts, AI explanations, alerts, news, and CSV export
- PostgreSQL schema via Prisma, Redis-ready cache design, Docker and CI files

## Live data architecture
Collectors target NOAA CPC ERSSTv5/ONI text feeds first, with revalidated server fetches and deterministic scientific fallbacks when upstream climate endpoints are unavailable. The architecture is prepared for NOAA Climate.gov, IRI plume, ECMWF/Copernicus, BOM, NASA, DHM Nepal, ICIMOD, GPM IMERG, ERA5, and monsoon-index ingestion.

## Development
```bash
npm install
cp .env.example .env
npm run dev
```

## Quality checks
```bash
npm run typecheck
npm run test
npm run build
```

## API routes
- `GET /api/enso/status`
- `GET /api/enso/forecast`
- `GET /api/nepal/impact?region=kathmandu`
- `POST /api/ai/explain`
- `POST /api/alerts`
- `GET /api/news`
- `GET /api/export`
