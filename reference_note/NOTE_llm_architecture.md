# LLM Voice Assistant Architecture Notes

## Goal
Build a personal Android voice assistant for robotics research.

Main idea:
- Android phone handles UI, recording, display, and optionally local playback.
- PC handles API key storage and most of the intelligence.
- Phone talks to PC over local Wi-Fi using HTTP.

---

## Recommended First Architecture

```text
Android phone
  - record audio
  - send audio/text to PC over HTTP
  - show transcript and answer text
  - optionally do local TTS playback

PC backend
  - receive audio/text
  - do STT
  - call GPT / Responses API
  - optionally do TTS
  - return text and optionally audio
```

Best first version:

```text
Phone records audio
-> PC does STT + GPT
-> PC sends text back
-> Phone shows text
-> Phone uses Android local TTS to speak
```

Alternative simple backend-heavy version:

```text
Phone records audio
-> PC does STT + GPT + TTS
-> PC sends transcript + answer text + audio
-> Phone shows text and plays audio
```

---

## Why Use a PC Backend

A backend is just a separate program/service running outside the Android UI app.
For this project, the backend can simply be a Python server running on a PC.

Typical setup:

```text
Phone -> Wi-Fi -> PC backend -> OpenAI API
```

Reasons this is good:
- keeps the OpenAI API key off the Android app
- easier to debug and modify
- easier to add tools later
- easier to log requests and responses
- easier to swap models or speech systems

Important nuance:
- If someone physically steals the PC, the key can still be exposed.
- The main extra safety benefit is that backend code is not packaged into an APK that can be copied and reverse engineered.

---

## Android vs PC Responsibilities

### Android side
- UI
- push-to-talk button
- microphone recording
- HTTP request to PC
- show transcript and answer text
- optionally local TTS playback

### PC side
- FastAPI or Flask server
- stores OpenAI API key
- speech-to-text
- GPT / Responses API calls
- optional text-to-speech
- optional web/tool calls

---

## Networking Choice

Recommended: **HTTP over TCP**

Why:
- simple request/response model
- easy to debug
- good for uploading audio and receiving text or audio
- much easier than UDP
- easier than WebSocket for a first version

### HTTP vs WebSocket

Both use TCP.

- **HTTP**: discrete request/response
- **WebSocket**: persistent two-way connection

For this project, start with HTTP.
Use WebSocket only later if real-time streaming voice is needed.

---

## OpenAI API Pieces

Three possible cloud components:
- speech-to-text (STT)
- text generation (GPT / Responses API)
- text-to-speech (TTS)

Important:
- OpenAI STT is cloud-based, not local.
- OpenAI TTS is cloud-based, not local.
- OpenAI text generation is cloud-based.
- All three use the same OpenAI API key.
- All three are billed usage.

---

## API Key Notes

OpenAI API key:
- is just a secret string/token
- not like an SSH public/private key pair
- works like a password sent with requests

Typical format:
- starts with `sk-`
- long random string

Important facts:
- does not expire automatically
- remains valid until manually deleted/revoked
- should be rotated manually if leaked

Where to get it:
- OpenAI platform API keys page

Best storage on PC:
- environment variable
- `.env` file

Avoid:
- hardcoding into app source
- committing to GitHub
- embedding into APK

---

## Tool / Agent Behavior

Default GPT API behavior:
- just an LLM
- no internet by default
- no web browsing by default
- no tool use by default

If tool support is enabled in the Responses API:
- model can use tools such as web search
- model can act more like an agent

Important distinction:
- **Responses API** = cloud model interface
- **Agents SDK** = helper framework for backend-side orchestration
- neither means the agent runs locally on the phone

Recommended mental model:

```text
Android app -> backend -> Responses API (+ tools)
```

---

## Can the Model Check Live Information?

Yes, but only if tools are enabled.

Examples:
- current date/time
- UCLA dining hall menu today
- web search

Without tools:
- model behaves like a normal LLM and does not browse live websites

For robust app-specific tasks, a custom backend tool is often better than generic web search.

---

## STT and TTS: Local vs Cloud

### OpenAI cloud STT/TTS
Pros:
- very high quality
- easiest unified backend design
- strong multilingual and noisy-speech performance

Cons:
- costs money
- requires internet
- added network latency

### Local STT/TTS on PC
Pros:
- free after setup
- private
- no cloud cost for speech
- PC can run stronger local models than phone

Cons:
- more setup
- quality may still be below top cloud models

### Local STT/TTS on phone
Pros:
- can be low latency
- offline possible for some pieces

Cons:
- weaker compute
- harder to run strong STT models
- battery and performance constraints

---

## Good Local Speech Options on PC

### Local STT on PC
Good choices:
- Whisper / whisper.cpp
- Vosk

General guidance:
- Whisper / whisper.cpp: better quality
- Vosk: lighter and simpler

### Local TTS on PC
Good choices:
- Piper
- Coqui TTS

General guidance:
- Piper: very good practical local choice
- Coqui TTS: flexible and powerful

---

## Android Local Speech Capabilities

### Android local TTS
Yes, Android supports local TTS well.

Why it is attractive:
- built-in support
- free
- lightweight CPU use
- low latency
- easy integration

### Android local STT
Not really strong built-in local STT by default.

Built-in speech recognition often depends on cloud services.
For truly local STT on Android, usually need external models like:
- Whisper
- Vosk

---

## Why Android Has Built-in TTS More Than STT

TTS is easier than STT.

### TTS
- text -> audio
- input is clean and structured
- less ambiguity
- lower compute

### STT
- audio -> text
- input is noisy and ambiguous
- accents, background noise, speech rate differences
- much heavier compute

So historically:
- local TTS became standard on devices
- strong STT often stayed server/cloud-based

---

## Latency Notes

### TTS on phone vs TTS on PC
Usually lower latency:
- **phone local TTS**

Why:
- PC sends only text
- text is tiny
- phone generates speech locally
- avoids sending audio files over the network

PC TTS + sending audio back can have better voice quality, but typically higher latency.

### Practical rule
- lowest latency: phone local TTS
- best voice quality: PC/OpenAI TTS

---

## Recommended TTS Choice for First Version

Recommended first version:
- PC does STT + GPT
- phone does local TTS

Why:
- lower latency
- free
- simple
- good enough for a prototype

If voice quality becomes important later:
- move TTS to PC or OpenAI cloud

---

## Is Phone TTS Heavy?

Usually no.

Android local TTS is typically:
- low to moderate CPU
- much lighter than local STT
- acceptable for real-time interaction

So using phone TTS is practical.

---

## Quality Ranking Summary

### STT quality (general rough ranking)
- OpenAI cloud STT: best overall quality
- PC local Whisper: strong and often close
- phone local STT: usually weaker due to smaller models / weaker hardware

### TTS quality (general rough ranking)
- OpenAI cloud TTS: best naturalness
- good PC local neural TTS: decent to strong
- Android built-in TTS: practical and light, but usually less natural

---

## Cost Notes

OpenAI pipeline cost happens at three possible points:
- STT
- GPT text generation
- TTS

Rough takeaway:
- STT is usually cheap
- TTS is also relatively cheap
- GPT cost depends strongly on model and token usage

To save money:
- use smaller GPT model
- limit response length
- use local STT/TTS if needed

---

## Model Selection Notes

Important:
- API offers more models than the ChatGPT browser UI
- smaller / older API models can be cheaper
- cheap “mini” or “nano” models are often better choices than very old legacy models

General strategy:
- start with a reasonably cheap modern model
- upgrade only if quality is insufficient

---

## Final Recommended Architecture

### Best first practical build

```text
Android
  - push-to-talk
  - record audio
  - HTTP upload to PC
  - display transcript + answer
  - local Android TTS playback

PC backend
  - FastAPI
  - API key stored securely
  - OpenAI STT or local STT
  - OpenAI GPT / Responses API
  - return text to phone
```

### Later upgrade options
- switch STT to local Whisper on PC
- enable tool use / web search in Responses API
- add custom robotics tools on backend
- move TTS to PC/OpenAI if higher voice quality is needed
- use WebSocket later for realtime streaming voice

---

## One-Sentence Summary

For a personal Android robotics voice assistant, a strong first design is:

**Phone handles UI/recording/display/local speech playback; PC backend handles API key, STT, GPT/tool calls, and optional higher-end speech processing.**
