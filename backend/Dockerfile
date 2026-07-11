# DashBuddy backend — small Express API + dashboard.
# Alpine keeps the image ~60 MB; the app has one dependency (express).
FROM node:20-alpine

# Install deps first so this layer caches unless package files change.
WORKDIR /app
COPY package.json package-lock.json ./
RUN npm ci --omit=dev

# App source.
COPY server.js db.js ./
COPY public ./public

# Drive summaries live on a mounted volume, NOT in the image. Create the mount
# point owned by the unprivileged `node` user (baked into the base image) so the
# container can write to it without running as root.
RUN mkdir -p /data && chown -R node:node /data
USER node

ENV NODE_ENV=production \
    PORT=3000 \
    DATA_FILE=/data/drives.json

EXPOSE 3000

# Liveness check: hit the API and fail if it doesn't answer 2xx.
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
  CMD node -e "fetch('http://localhost:'+process.env.PORT+'/api/drives').then(r=>process.exit(r.ok?0:1)).catch(()=>process.exit(1))"

CMD ["node", "server.js"]
