# PhoneBot AI Backend (STT + GPT over HTTP)

## Goal

Run a small HTTP backend on your PC/laptop that:

- accepts **audio** (from Android)
- runs **local STT** using `whisper.cpp`
- calls **OpenAI GPT** using `OPENAI_API_KEY`
- returns `{ transcript, reply }` JSON back to Android

This keeps the API key off the Android APK and matches `reference_note/NOTE_llm_architecture.md`.


## Prerequisites

- Python 3
- `whisper.cpp` already built at:
  - `AI_backend/whisper.cpp/build/bin/whisper-cli`
- Whisper models present at:
  - `AI_backend/whisper.cpp/models/ggml-*.bin`
- OpenAI key in environment:

```bash
export OPENAI_API_KEY="sk-..."
```


## Install Python deps

From `AI_backend/`:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```


## Run server

```bash
cd AI_backend
export OPENAI_API_KEY="sk-..."
python3 server.py
```

Defaults:

- bind: `0.0.0.0`
- port: `8088`
- whisper model: `base` (uses `ggml-base.bin`)
- whisper language: `en`
- OpenAI model: `gpt-4o-mini`

Override examples:

```bash
export AI_BACKEND_HOST=0.0.0.0
export AI_BACKEND_PORT=8088
export WHISPER_MODEL=base
export WHISPER_LANGUAGE=en
export OPENAI_MODEL=gpt-4o-mini
python3 server.py
```


## Endpoints

- `GET /health`
- `POST /api/llm/text`
  - JSON: `{ "text": "...", "openai_model": "...", "system_prompt": "..." }`
- `POST /api/llm/audio`
  - multipart form:
    - `audio` (file)
    - `whisper_model` (e.g. `base`, `small.en`, `large-v3`)
    - `whisper_language` (e.g. `en`, `auto`)
    - `openai_model`
    - `system_prompt`



