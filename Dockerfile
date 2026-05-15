FROM node:24-alpine AS deps
WORKDIR /app
COPY package*.json ./
RUN npm ci
FROM node:24-alpine AS runner
WORKDIR /app
COPY --from=deps /app/node_modules ./node_modules
COPY . .
RUN npm run build
EXPOSE 3000
CMD ["npm","start"]
