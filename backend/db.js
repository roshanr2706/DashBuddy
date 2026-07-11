// Tiny append-only JSON store for drive summaries. No external DB — the volume
// is one small record per drive, so a single JSON file is plenty and keeps the
// backend trivial to run. Swap for SQLite/Postgres if you outgrow it.

import { readFileSync, writeFileSync, existsSync, mkdirSync } from 'node:fs';
import { dirname } from 'node:path';

const DATA_FILE = process.env.DATA_FILE || './data/drives.json';

function ensureFile() {
  const dir = dirname(DATA_FILE);
  if (!existsSync(dir)) mkdirSync(dir, { recursive: true });
  if (!existsSync(DATA_FILE)) writeFileSync(DATA_FILE, '[]');
}

export function allDrives() {
  ensureFile();
  try {
    return JSON.parse(readFileSync(DATA_FILE, 'utf8'));
  } catch {
    return [];
  }
}

// Insert a drive; de-dupes on (device_id, session_id) so device retries after a
// flaky upload don't create duplicate rows.
export function addDrive(record) {
  const drives = allDrives();
  const dup = drives.find(
    (d) => d.device_id === record.device_id && d.session_id === record.session_id
  );
  if (dup) return { inserted: false, drive: dup };

  const stored = {
    ...record,
    received_at: Math.floor(Date.now() / 1000),
  };
  drives.push(stored);
  writeFileSync(DATA_FILE, JSON.stringify(drives, null, 2));
  return { inserted: true, drive: stored };
}
