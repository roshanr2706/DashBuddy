# Dash Companion (DashBuddy)

An animated-face dashboard device on an ESP32-S3. A color screen shows an
expressive face that reacts to driving (hard braking, cornering, bumps),
auto-dims to ambient light, and logs each drive's stats to onboard flash for
upload over WiFi. Powered from the car's USB-C port — no battery.

Full design rationale lives in [dash-companion-build-plan.md](dash-companion-build-plan.md).
This README covers the two things that ship in this repo:

- **`firmware/`** — ESP32-S3 firmware (PlatformIO, TFT_eSPI).
- **`backend/`** — Node/Express upload endpoint + trends dashboard.

```
Car → [MPU6050 + VEML7700] → ESP32-S3 → ST7735 face
                                   │
                                WiFi POST
                                   ▼
                        backend (Express) → dashboard
```

---

## 1. Wiring

Two buses plus one PWM pin (build plan §5). Confirm against your board's
silkscreen and avoid the hazard pins in §8 (GPIO26–37 flash/PSRAM, GPIO0/3/45/46
strapping, GPIO19/20 USB).

| Signal | ESP32-S3 GPIO | Connects to |
|---|---|---|
| SPI SCK | 12 | LCD SCL |
| SPI MOSI | 11 | LCD SDA |
| Display DC | 13 | LCD DC |
| Display CS | 10 | LCD CS |
| Display RST | 14 | LCD RES |
| Backlight (PWM) | 15 | LCD BLK |
| I2C SDA | 8 | MPU6050 SDA + VEML7700 SDA |
| I2C SCL | 9 | MPU6050 SCL + VEML7700 SCL |
| Buzzer (optional) | 2 | passive piezo + |

Power: LCD **VCC → 3.3V**, all grounds common. Feed the board 5V via either
USB-C port. I2C addresses don't collide (MPU6050 `0x68`, VEML7700 `0x10`).

All pins live in one place — [`firmware/src/config.h`](firmware/src/config.h).
Change them there if your wiring differs; the TFT SPI pins are also mirrored as
build flags in [`firmware/platformio.ini`](firmware/platformio.ini) (keep the two
in sync).

---

## 2. Firmware — build & flash

Needs [PlatformIO](https://platformio.org/install) (`pip install platformio` or
the VS Code extension). Board: **ESP32-S3-N16R8** (16 MB flash, 8 MB PSRAM).

```bash
cd firmware

# Build (downloads the pinned libraries on first run)
pio run

# Flash over USB (auto-detects the port; --target upload)
pio run --target upload

# Watch serial output
pio device monitor
```

Before flashing for real driving, put your **secrets** in a gitignored `.env`
(never in `config.h`):

```bash
cd firmware
cp .env.example .env      # then edit .env
```

```ini
WIFI_SSID=Your 2.4GHz WiFi     # leave blank to run fully offline
WIFI_PASSWORD=yourpassword
UPLOAD_URL=http://<backend-host>:3000/api/drives
DEVICE_KEY=must-match-backend
OTA_PASSWORD=something
```

`scripts/load_env.py` injects these at build time as `CFG_*` compiler defines;
`config.h` reads them with empty fallbacks, so the build works even with no
`.env`. **`.env` is gitignored — keep your WiFi password out of git.**

Non-secret tuning still lives in `config.h`:
- `imu::*` thresholds — **tune from real in-car data**, not a desk (plan §2/§11).
- `face_anim::*` — face colours, spring response, reaction durations, spin and
  dizzy thresholds, lean strength, braking intensity, and idle timing.

### Face animation and variations

The face uses a **RoboEyes-inspired vector renderer** on the existing full-screen
`TFT_eSprite` framebuffer. Two solid, pupil-free rounded eyes form every
expression through animated width, height, corner radius, asymmetry, eyelid
angles, position, and rotation. These parameters move with damped springs. No
mouth or bitmap face frames are used, and no
extra driving detector exists: `FaceEngine` consumes the signed G-force and
turn-rate values already produced by `ImuProcessor`.

The motion input accepts both genuine MPU6050 (`WHO_AM_I=0x68`) and
MPU6500-compatible (`WHO_AM_I=0x70`) devices at I2C address `0x68` or `0x69`.
Both feed the same orientation learning and event pipeline through the small
`MotionSensor` register adapter. The `displaytest` PlatformIO environment is a
standalone I2C/IMU diagnostic that reports the detected identity and live raw
acceleration/gyro values.

Three layers are composed every frame, so a reaction never permanently erases
the character's underlying mood:

| Layer | Variations | Behaviour |
|---|---|---|
| Persistent mood | calm, alert, tense | Driving tension slowly shifts the background from deep blue to dark red, the eyes from cyan-white to amber, and narrows and angles their silhouettes. Tension decays back to calm over roughly 30 seconds. |
| Live motion | left/right lean | Continuous lateral G slides and rotates the entire face through a corner, even below the discrete event threshold. |
| Idle | blink, left glance, right glance | Organic randomized blinks always run while calm. Glances run while parked and use the same spring motion as expressions. |
| Hard braking | red eye flash, sharp squint | A fast red eye/border flash and aggressive opposing lids create a short tense reaction, then spring back to the current mood. Stronger braking produces a stronger tension increase. |
| Acceleration | pleased squint, lifted eyes | The eyes widen, compress vertically, and rise into a clear happy expression. |
| Normal corner | directional lean, tilt, side-eye attitude | Direction and intensity come from the existing lateral-G and gyro measurements. The overlay settles back into the persistent mood. |
| Very hard corner | full face spin | Crossing either the visual spin G threshold or turn-rate threshold rotates the complete vector face once in the turn direction. |
| Post-spin dizzy | mismatched eyes, wobble | The spin flows into animated eye asymmetry and a decaying head wobble before restoring the previous mood. |
| Bump | vertical rattle, tall surprised eyes | Vertical-G intensity scales the jiggle and eye height. Spring damping removes the motion instead of snapping it off. |
| Strong deceleration | X eyes | Forward acceleration below -1.389 m/s² replaces both eyes with bold crosses for 30 seconds. |

The eyes carry the whole expression. Removing pupils and the mouth keeps the
face graphic and emoticon-like rather than cartoon-like, while unequal eye
heights, angled lids, whole-eye glances, and spring motion preserve personality.

Useful `face_anim::*` controls in `firmware/src/config.h` include:

- `SPRING_STIFFNESS` / `SPRING_DAMPING` — responsiveness and bounce.
- `CALM_*`, `TENSE_*`, `BRAKE_EYES` — the full face palette.
- `LIVE_LEAN_PX_PER_G`, `TURN_ROTATION_DEG` — continuous corner motion.
- `SPIN_TRIGGER_G`, `SPIN_TRIGGER_DPS`, `SPIN_DURATION_MS` — hard-turn spin.
- `DIZZY_DURATION_MS`, `DIZZY_WOBBLE_DEG` — post-spin recovery.
- `BRAKE_DURATION_MS`, `BRAKE_NARROW_LID_PX`, `BRAKE_FLASH_DECAY` — braking.
- `*_DURATION_MS`, `BUMP_JIGGLE_PX`, blink and glance ranges — other overlays.

Tune the IMU thresholds first from real driving data, then tune the visual
thresholds. In particular, keep `SPIN_TRIGGER_G` at or above
`imu::HARD_CORNER_G` so ordinary corners do not constantly trigger spins.

**Orientation is auto-detected — no mounting config needed.** The board figures
out its own axes at runtime: "up" from gravity, "forward" learned from
straight-line braking/accelerating (gyro confirms you're going straight), and
turning measured against both. The learned frame is saved to NVS, so it
calibrates once during your first few minutes of driving and survives reboots.
If auto-detection picks the wrong front/back or left/right, flip
`imu::FORWARD_SIGN_OVERRIDE` / `imu::LATERAL_SIGN` in `config.h`; bump
`imu::CAL_VERSION` to force a fresh re-learn after re-mounting.

Sign convention everywhere: forward + = speeding up, lateral/turn-rate + =
right turn. The gyro's zero-rate bias is learned while parked and subtracted,
and the gravity filter freezes during maneuvers so sustained braking can't
tilt the "up" estimate.

**Uploads and OTA only run while parked** — an HTTP retry or a flash write can
block for seconds, and the face must never stutter mid-drive.

### Bring-up follows the plan's phases (§10)

| Phase | What to check | Module |
|---|---|---|
| 1 Display | Clean, aligned face, right colors | `Display`, `FaceEngine` |
| 2 IMU | Set `debug::STREAM_IMU = true`; drive straight then turn — watch `cal%` climb and `turnRate` respond | `ImuProcessor` |
| 3 Face | Idle face flashes red on a real hard brake | `FaceEngine` |
| 4 Auto-dim | Screen dims at night / in a tunnel, recovers smoothly | `AutoDim` |
| 5 Logging | One summary record per completed drive in flash | `DriveLogger` |
| 6 Connectivity | Drive appears on the dashboard; OTA works | `Connectivity` |

> **1.44" panel gotcha (§3):** colored border or a few-pixel shift = tab/offset.
> The build uses TFT_eSPI's `ST7735_GREENTAB3` 128x128 offset profile.

### OTA updates
Once the device is on WiFi, PlatformIO can flash it wirelessly:
```bash
pio run --target upload --upload-port dashbuddy.local
```
(OTA password is `net::OTA_PASSWORD` in config.h.)

---

## 3. Backend — run the dashboard

### Option A — Docker (recommended for an always-on Ubuntu box)

Needs Docker Engine + the Compose plugin
([install docs](https://docs.docker.com/engine/install/ubuntu/)).

```bash
cd backend
docker compose up -d          # build image + start in the background
# → dashboard on http://<this-machine-ip>:3000
```

That's the whole setup. Useful follow-ups:

```bash
docker compose logs -f        # watch drive uploads arrive live
docker compose down           # stop (drive history is kept)
docker compose up -d --build  # rebuild after pulling new code
```

**Set your device key** (so it isn't the default `change-me`): create a `.env`
next to `docker-compose.yml` — `cp .env.docker.example .env` — and edit it:

```ini
DEVICE_KEY=your-secret        # MUST equal firmware net::DEVICE_KEY
HOST_PORT=3000                # host port; change if 3000 is taken
```

Then `docker compose up -d` again to apply. Drive summaries persist in the
named Docker volume `dashbuddy-data`, so they survive `down`, rebuilds, and
reboots (`restart: unless-stopped` brings the container back after a reboot).
To wipe history: `docker compose down -v`.

> **Firmware side:** point `net::UPLOAD_URL` at this machine's LAN IP and the
> host port, e.g. `http://192.168.1.50:3000/api/drives` (find it with
> `hostname -I`). Give the box a static IP / DHCP reservation so the URL
> doesn't drift.

### Option B — bare Node (quick local run)

Needs Node 18+.

```bash
cd backend
npm install

# Set the shared secret (must equal firmware net::DEVICE_KEY) and run
DEVICE_KEY=change-me npm start
# → http://localhost:3000
```

Open `http://localhost:3000` for the trends dashboard (auto-refreshes every 5 s).

| Route | Purpose |
|---|---|
| `POST /api/drives` | Device uploads one summary (`X-Device-Key` header required) |
| `GET /api/drives` | JSON of all drives, newest first (dashboard data) |
| `GET /` | The dashboard page |

Config via env (see `.env.example`): `PORT`, `DEVICE_KEY`, `DATA_FILE`.
Drives are stored as JSON in `backend/data/drives.json`. Uploads are de-duped on
`(device_id, session_id)` so device retries never double-count.

### Quick smoke test without hardware
```bash
curl -X POST http://localhost:3000/api/drives \
  -H "Content-Type: application/json" -H "X-Device-Key: change-me" \
  -d '{"device_id":"dashbuddy-test","session_id":1,"duration_s":930,
       "max_brake_g":0.62,"max_corner_g":0.48,"max_accel_g":0.35,
       "hard_brake_count":3,"hard_corner_count":5,"smoothness":82,
       "started_epoch":1720000000,"boot_start_ms":0}'
```
Then refresh the dashboard.

---

## 4. Drive summary schema

The device POSTs one record per drive (built in `DriveLogger.cpp`, consumed by
`server.js`):

```json
{
  "device_id": "dashbuddy-a1b2c3",
  "session_id": 42,
  "duration_s": 930,
  "max_brake_g": 0.62,
  "max_corner_g": 0.48,
  "max_accel_g": 0.35,
  "max_turn_dps": 34.0,
  "hard_brake_count": 3,
  "hard_corner_count": 5,
  "smoothness": 82,
  "started_epoch": 1720000000,
  "boot_start_ms": 12345
}
```

`started_epoch` is `0` until NTP has synced (no GPS clock); the backend falls
back to upload time. Per plan §6 there is intentionally **no speed/distance/
route** — accel integration drifts, so it isn't faked.

---

## 5. Layout

```
DashBuddy/
├─ dash-companion-build-plan.md   design + rationale (source of truth)
├─ firmware/
│  ├─ platformio.ini              board, pinned libs, TFT_eSPI flags, OTA
│  └─ src/
│     ├─ config.h                 pins, thresholds, wifi — tune here
│     ├─ main.cpp                 non-blocking millis() loop
│     ├─ Backlight.*  Display.*   LEDC PWM + TFT_eSPI PSRAM framebuffer
│     ├─ FaceEngine.*             3-layer parametric face (mood/event/idle)
│     ├─ ImuProcessor.*           MPU6050 → events + per-drive stats
│     ├─ AutoDim.*                VEML7700 lux → backlight
│     ├─ DriveLogger.*            LittleFS per-drive summaries
│     ├─ Connectivity.*           WiFi + NTP + upload + OTA
│     └─ Buzzer.*                 optional brake-alert tone
└─ backend/
   ├─ server.js  db.js            Express API + JSON store
   ├─ package.json
   ├─ public/index.html           trends dashboard
   ├─ Dockerfile                  container image (node:20-alpine)
   ├─ docker-compose.yml          one-command deploy + data volume
   └─ .env.docker.example         copy to .env to set DEVICE_KEY / HOST_PORT
```
