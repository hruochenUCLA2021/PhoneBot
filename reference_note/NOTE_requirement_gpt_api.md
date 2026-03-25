# NOTE: Backend requirements + OpenAI GPT API (no special SDK needed)

## Key takeaway

**OpenAI GPT is accessed via normal HTTPS requests.**  
You do **not** need a special “OpenAI library” to use GPT.

Any HTTP client works:

- Python: `requests`, `httpx`, `urllib`
- Terminal: `curl`
- Android: `HttpURLConnection`, OkHttp

OpenAI “SDKs” are just convenience wrappers around the same HTTP calls.


## Why we use `fastapi` in `AI_backend`

In our architecture:

- **Phone → PC backend**: HTTP on local Wi‑Fi (your own server)
- **PC backend → OpenAI**: HTTPS over the internet (OpenAI server)

So the backend needs **two roles**:

1. **HTTP server** (to receive text/audio from Android)
2. **HTTP client** (to call OpenAI)


## What each line in `AI_backend/requirements.txt` is for

Current file:

```text
fastapi
uvicorn[standard]
python-multipart
requests
```

- **`fastapi`**: framework for building the PC HTTP API.
  - defines endpoints like `GET /health`, `POST /api/llm/text`, `POST /api/llm/audio`
- **`uvicorn[standard]`**: the actual server that listens on a port and serves the FastAPI app.
- **`python-multipart`**: enables `multipart/form-data` parsing for file uploads.
  - needed for `POST /api/llm/audio` where Android uploads a WAV file
- **`requests`**: HTTP client library used by the backend to call OpenAI HTTPS endpoints.


## How OpenAI auth works (API key)

The OpenAI API key is a secret token used like a password.

We keep it out of the Android APK and out of git by storing it in an environment variable:

```bash
export OPENAI_API_KEY="sk-..."
```

In Python code we read it by:

```python
api_key = os.getenv("OPENAI_API_KEY")
```


## Minimal “GPT via HTTP” example (concept)

This is the core idea (pseudo‑Python):

```python
import requests, os, json

api_key = os.getenv("OPENAI_API_KEY")
url = "https://api.openai.com/v1/chat/completions"

headers = {
  "Authorization": f"Bearer {api_key}",
  "Content-Type": "application/json",
}

body = {
  "model": "gpt-4o-mini",
  "messages": [
    {"role": "system", "content": "Be concise."},
    {"role": "user", "content": "Hello"},
  ],
}

r = requests.post(url, headers=headers, data=json.dumps(body))
print(r.json()["choices"][0]["message"]["content"])
```

That’s it: **HTTPS + JSON + Authorization header**.


## How this maps to our repo

### Server side (PC): FastAPI endpoints

Implemented in:

- `AI_backend/server.py`

Key endpoints:

- `POST /api/llm/text`
  - Android sends JSON `{ text: "..." }`
  - backend calls OpenAI and returns `{ transcript, reply }`
- `POST /api/llm/audio`
  - Android uploads WAV file (multipart)
  - backend runs `whisper.cpp` STT locally
  - backend calls OpenAI with the transcript
  - backend returns `{ transcript, reply }`

### Client side (PC → OpenAI): requests

Also in:

- `AI_backend/server.py`

It uses `requests.post(...)` to call OpenAI over the internet.


## Notes / common pitfalls

- **Timeouts**: always set reasonable HTTP timeouts (STT + GPT can take seconds).
- **Never hardcode the key** in repo or Android code.
- **Network routing**:
  - Phone must reach backend IP/port over Wi‑Fi.
  - Backend machine must have internet access for OpenAI.

