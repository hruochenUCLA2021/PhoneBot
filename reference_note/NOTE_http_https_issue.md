# NOTE: HTTP vs HTTPS on Android (cleartext not permitted)

## The problem you saw

Android app error:

- `Cleartext HTTP traffic to 192.168.20.15 not permitted`

Meaning:

- your app tried to call `http://192.168.20.15:8088/...`
- Android blocked it because **plain HTTP is “cleartext”** (not encrypted)


## Why Android blocks HTTP by default

With **HTTP**:

- traffic is **not encrypted**
- anyone on the same Wi‑Fi (or a compromised router/AP) can potentially:
  - read transcripts/replies
  - modify responses (tamper)
  - redirect requests

So modern Android defaults often block cleartext for security.


## HTTPS vs HTTP (practical)

- **HTTP**:
  - easiest for local prototypes
  - no certificates
  - not encrypted

- **HTTPS**:
  - encrypted + harder to tamper
  - requires TLS certificates
  - for LAN prototypes, cert setup is usually the annoying part (self‑signed/trust)


## What we do right now (prototype)

For simplicity on a trusted LAN:

- **Allow cleartext HTTP** in the Android app.
- Backend continues to run on:
  - `http://192.168.20.15:8088`


## How to allow HTTP (quick method)

Add this to the Android app manifest `<application>`:

- `android:usesCleartextTraffic="true"`

This allows `http://...` URLs (not only the AI backend).


## More secure later (recommended upgrade path)

When you want better security:

1. Serve the backend over HTTPS (TLS)
2. Use a certificate that Android trusts (or install trust for dev)
3. Optionally add:
   - authentication between phone and backend
   - request signing / tokens


## Quick setup reminder (your current LAN)

- PC backend IP: `192.168.20.15`
- Backend port: `8088`
- Android “AI backend IP/Host”: `192.168.20.15`
- Android “Port”: `8088`

Backend should bind with:

- `AI_BACKEND_HOST=0.0.0.0` (listen on all interfaces)

