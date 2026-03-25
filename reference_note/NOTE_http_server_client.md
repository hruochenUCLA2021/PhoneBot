# NOTE: HTTP server/client + FastAPI routing + async (PhoneBot AI backend)

This note explains the “HTTP mental model” using our backend:

- `AI_backend/server.py`

---

## 1) What is HTTP in this project?

We use **HTTP over TCP** as a request/response protocol.

Two separate HTTP links exist:

1. **Android → PC backend (local Wi‑Fi)**
   - Android sends a request like: `POST http://<pc-ip>:8088/api/llm/audio`
   - PC returns JSON: `{ "transcript": "...", "reply": "..." }`

2. **PC backend → OpenAI (internet)**
   - backend sends: `POST https://api.openai.com/v1/chat/completions`
   - OpenAI returns JSON with the model output

---

## 2) What does an HTTP request contain?

An HTTP request is basically:

- **Method**: `GET`, `POST`, ...
- **URL path**: `/health`, `/api/llm/text`, ...
- **Headers**: metadata like content type and authorization
- **Body**: the payload (JSON or file upload)

Examples:

### A “GET /health”

- method: `GET`
- path: `/health`
- body: none

### A “POST /api/llm/text”

- method: `POST`
- path: `/api/llm/text`
- header: `Content-Type: application/json`
- body: JSON like `{ "text": "hello" }`

### A “POST /api/llm/audio”

- method: `POST`
- path: `/api/llm/audio`
- header: `Content-Type: multipart/form-data; boundary=...`
- body: “form parts”, including the uploaded audio file bytes

---

## 3) What is a server vs client?

- **Server**: listens on an IP/port, waits for requests, sends responses.
  - our backend is the server: FastAPI + Uvicorn
- **Client**: actively sends a request, waits for the response.
  - Android is a client to the backend
  - backend is a client to OpenAI

---

## 4) FastAPI: how does “registering endpoints” work?

In `server.py` we create a FastAPI app:

```130:137:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/AI_backend/server.py
app = FastAPI(title="PhoneBot AI Backend", version="0.1")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

Then we attach Python functions to URL routes using decorators:

```140:163:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/AI_backend/server.py
@app.get("/health")
def health():
    return {...}

@app.post("/api/llm/text")
def llm_text(req: TextReq):
    ...
```

Meaning:

- When the server receives `GET /health`, it runs `health()`.
- When the server receives `POST /api/llm/text`, it runs `llm_text(...)`.

The *decorator string* (`"/health"`, `"/api/llm/text"`) is the real “handle”.
The Python function name is not what the network uses.

---

## 5) Does the function name matter? (e.g., `llm_audio`)

Not for routing.

This is what matters:

- `@app.post("/api/llm/audio")`

The function name `llm_audio` is mainly for:

- readability
- logs / stack traces
- debugging

You could rename `llm_audio` to `handle_audio_upload` and it would still work as long as the decorator stays.

---

## 6) What does `async def` mean?

In `server.py`:

```166:173:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/AI_backend/server.py
@app.post("/api/llm/audio")
async def llm_audio(
    audio: UploadFile = File(...),
    ...
):
```

`async def` means this handler is defined as a Python “coroutine”, which can:

- `await` other async operations without blocking the server event loop.

However:

- Our current `llm_audio` implementation **does not `await` much** and it calls blocking code:
  - running `whisper-cli` via `subprocess.run`
  - calling OpenAI via `requests.post`

So in practice:

- `async def` here is *mostly harmless*, but not giving full async benefits.
- A more scalable version would use:
  - `httpx` (async HTTP client) instead of `requests`
  - `asyncio.to_thread(...)` or a thread pool for `whisper-cli`

For your robotics prototype, this is fine (simple and reliable).

---

## 7) How does FastAPI parse inputs?

### JSON body → Pydantic model

For `/api/llm/text`, we declare:

```124:128:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/AI_backend/server.py
class TextReq(BaseModel):
    text: str
    openai_model: Optional[str] = None
    system_prompt: Optional[str] = None
```

FastAPI automatically parses JSON into `TextReq`.

### multipart form → UploadFile + Form fields

For `/api/llm/audio`, we declare:

- `audio: UploadFile = File(...)`  (the uploaded bytes)
- other settings via `Form(...)`

FastAPI parses the multipart body and gives us:

- a file handle (`audio.file`)
- strings for form fields

---

## 8) What is CORS middleware?

This block:

```130:137:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/AI_backend/server.py
app.add_middleware(CORSMiddleware, allow_origins=["*"], ...)
```

Mainly matters for **browser** clients (web pages).

Android native apps are usually not restricted by browser CORS rules,
but leaving CORS open is convenient for quick testing from a browser.

---

## 9) One sentence summary

**FastAPI registers Python functions to URL paths; clients send HTTP requests; the server matches the method+path and runs the handler; function names don’t matter for routing; `async def` is about coroutine-style concurrency, but blocking calls still block unless moved to threads or async libraries.**





## 10) GET vs POST (very practical view)

`GET` and `POST` are **HTTP methods** (verbs). The method + path tells the server “what do you want me to do?”

### GET

- **Meaning**: “give me something / read-only query”
- **Typical usage**:
  - usually no request body
  - optional parameters go in the URL, e.g. `?verbose=1`
  - should be “safe” (should not change server state)
  - typically “idempotent” (calling it multiple times is fine)

**Example (our backend):**

- `GET /health`
  - returns server status JSON (whether whisper exists, whether key is set, etc.)

### POST

- **Meaning**: “here is data, please process it / perform an action”
- **Typical usage**:
  - includes a request body:
    - JSON (text)
    - multipart/form-data (file upload)
  - often triggers “work” on the server (STT, GPT)
  - not necessarily idempotent (may create logs, consume API tokens, etc.)

**Examples (our backend):**

- `POST /api/llm/text`
  - body: JSON like `{"text":"hello"}`
- `POST /api/llm/audio`
  - body: multipart upload (WAV file bytes + form fields like `whisper_model`)

### Simple mental model

- **GET**: “server → give me info”
- **POST**: “client → send data to server, server → do work, return result”


