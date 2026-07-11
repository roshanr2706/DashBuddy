// Dash Companion backend: receives per-drive summaries from the device and
// serves a small trends dashboard.
//
//   POST /api/drives   — device uploads one summary (X-Device-Key required)
//   GET  /api/drives   — dashboard data (newest first)
//   GET  /             — the dashboard page
//
// Config via env: PORT (default 3000), DEVICE_KEY (default "change-me"),
// DATA_FILE (default ./data/drives.json).

import express from 'express';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { allDrives, addDrive } from './db.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const PORT = process.env.PORT || 3000;
const DEVICE_KEY = process.env.DEVICE_KEY || 'change-me';

const app = express();
app.use(express.json({ limit: '64kb' }));
app.use(express.static(join(__dirname, 'public')));

// Minimal validation of the fields the device sends (see DriveLogger.cpp).
function validDrive(b) {
  return (
    b &&
    typeof b.device_id === 'string' &&
    Number.isFinite(b.session_id) &&
    Number.isFinite(b.duration_s) &&
    Number.isFinite(b.max_brake_g)
  );
}

app.post('/api/drives', (req, res) => {
  if (req.get('X-Device-Key') !== DEVICE_KEY) {
    return res.status(401).json({ error: 'bad device key' });
  }
  if (!validDrive(req.body)) {
    return res.status(400).json({ error: 'malformed drive summary' });
  }
  const { inserted, drive } = addDrive(req.body);
  res.status(inserted ? 201 : 200).json({ inserted, session_id: drive.session_id });
  console.log(
    `${inserted ? 'stored' : 'dup   '} drive ${drive.device_id}#${drive.session_id} ` +
      `(${drive.duration_s}s, smooth=${drive.smoothness})`
  );
});

app.get('/api/drives', (_req, res) => {
  const drives = allDrives().sort(
    (a, b) => (b.received_at || 0) - (a.received_at || 0)
  );
  res.json(drives);
});

app.listen(PORT, () => {
  console.log(`DashBuddy backend on http://localhost:${PORT}`);
  console.log(`Device key: "${DEVICE_KEY}" (set DEVICE_KEY to change)`);
});
