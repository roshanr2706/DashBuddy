"""PlatformIO pre-build hook: inject secrets from firmware/.env as -D defines.

A microcontroller can't read a .env at runtime, so we do the next best thing:
parse a gitignored `.env` at build time and hand each KEY=value to the compiler
as `CFG_KEY`. config.h reads those macros with empty fallbacks, so the build
still works (offline) even when .env is absent.

.env format (see .env.example):
    # comments and blank lines ignored
    WIFI_SSID=My Network        # value may contain spaces; no quotes needed
    WIFI_PASSWORD=hunter2

Keys become macros prefixed with CFG_ (WIFI_SSID -> CFG_WIFI_SSID) so they
never collide with the constexpr identifiers in config.h.
"""
Import("env")  # noqa: F821  (injected by PlatformIO/SCons)
import os

ENV_PATH = os.path.join(env.subst("$PROJECT_DIR"), ".env")  # noqa: F821

# Only these keys are accepted, so a typo in .env fails loudly instead of
# silently defining a macro config.h never reads.
ALLOWED = {
    "WIFI_SSID", "WIFI_PASSWORD", "UPLOAD_URL",
    "DEVICE_KEY", "OTA_PASSWORD",
}


def parse_env(path):
    values = {}
    with open(path, "r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, val = line.partition("=")
            key = key.strip()
            # Strip one layer of optional surrounding quotes; keep inner spaces.
            val = val.strip()
            if len(val) >= 2 and val[0] == val[-1] and val[0] in "\"'":
                val = val[1:-1]
            if key not in ALLOWED:
                print("[load_env] WARNING: ignoring unknown key '%s'" % key)
                continue
            values[key] = val
    return values


if not os.path.isfile(ENV_PATH):
    print("[load_env] no firmware/.env found — building with empty secrets "
          "(offline). Copy .env.example to .env to set WiFi/keys.")
else:
    values = parse_env(ENV_PATH)
    defines = [("CFG_" + k, env.StringifyMacro(v)) for k, v in values.items()]  # noqa: F821
    env.Append(CPPDEFINES=defines)  # noqa: F821
    print("[load_env] injected %d secret(s) from .env: %s"
          % (len(defines), ", ".join(sorted(values.keys()))))
