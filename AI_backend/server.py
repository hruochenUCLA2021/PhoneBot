import json
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import requests
from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel


ROOT = Path(__file__).resolve().parent
WHISPER_DIR = ROOT / "whisper.cpp"
WHISPER_BIN = WHISPER_DIR / "build" / "bin" / "whisper-cli"
WHISPER_MODELS_DIR = WHISPER_DIR / "models"
PIPER_VOICES_DIR = ROOT / "voices"


DEFAULT_WHISPER_MODEL_NAME = os.getenv("WHISPER_MODEL", "base")
DEFAULT_WHISPER_LANG = os.getenv("WHISPER_LANGUAGE", "en")
DEFAULT_OPENAI_MODEL = os.getenv("OPENAI_MODEL", "gpt-4o-mini")
DEFAULT_OPENAI_SYSTEM = os.getenv(
    "OPENAI_SYSTEM_PROMPT",
    "You are a helpful voice assistant for a robotics project. Be concise and practical.",
)


def _resolve_whisper_model(model_name: str) -> Path:
    name = (model_name or "").strip().lower()
    mapping = {
        "tiny": WHISPER_MODELS_DIR / "ggml-tiny.bin",
        "tiny.en": WHISPER_MODELS_DIR / "ggml-tiny.en.bin",
        "base": WHISPER_MODELS_DIR / "ggml-base.bin",
        "base.en": WHISPER_MODELS_DIR / "ggml-base.en.bin",
        "small": WHISPER_MODELS_DIR / "ggml-small.bin",
        "small.en": WHISPER_MODELS_DIR / "ggml-small.en.bin",
        "medium": WHISPER_MODELS_DIR / "ggml-medium.bin",
        "medium.en": WHISPER_MODELS_DIR / "ggml-medium.en.bin",
        "large-v1": WHISPER_MODELS_DIR / "ggml-large-v1.bin",
        "large-v2": WHISPER_MODELS_DIR / "ggml-large-v2.bin",
        "large-v3": WHISPER_MODELS_DIR / "ggml-large-v3.bin",
    }
    if name in mapping:
        return mapping[name]
    # allow direct file path
    p = Path(model_name)
    if p.is_file():
        return p
    raise FileNotFoundError(f"Unknown model '{model_name}'. Expected one of: {sorted(mapping.keys())}")


@dataclass(frozen=True)
class WhisperResult:
    text: str
    raw_stdout: str


def run_whisper_stt(audio_path: Path, model_name: str, language: str) -> WhisperResult:
    if not WHISPER_BIN.exists():
        raise FileNotFoundError(f"whisper-cli not found at {WHISPER_BIN}")
    model_path = _resolve_whisper_model(model_name)
    if not model_path.exists():
        raise FileNotFoundError(f"Whisper model not found: {model_path}")
    if not audio_path.exists():
        raise FileNotFoundError(f"Audio file not found: {audio_path}")

    # Print only results, no timestamps, and keep stdout clean for parsing.
    cmd = [
        str(WHISPER_BIN),
        "-m",
        str(model_path),
        "-f",
        str(audio_path),
        "-l",
        language,
        "-nt",
        "-np",
    ]
    proc = subprocess.run(
        cmd,
        cwd=str(WHISPER_DIR),
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"whisper-cli failed rc={proc.returncode} stderr={proc.stderr.strip()[:500]}")

    # With -np, stdout is primarily the transcription text (possibly multi-line).
    text = (proc.stdout or "").strip()
    return WhisperResult(text=text, raw_stdout=proc.stdout or "")


def openai_chat(prompt: str, model: str, system_prompt: str) -> str:
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY env var is not set")

    # Use the Chat Completions endpoint for simplicity/compatibility.
    # If you prefer the Responses API later, we can switch.
    url = "https://api.openai.com/v1/chat/completions"
    headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
    body = {
        "model": model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": prompt},
        ],
    }
    r = requests.post(url, headers=headers, data=json.dumps(body), timeout=60)
    if r.status_code >= 400:
        raise RuntimeError(f"OpenAI HTTP {r.status_code}: {r.text[:500]}")
    data = r.json()
    try:
        return data["choices"][0]["message"]["content"].strip()
    except Exception as e:
        raise RuntimeError(f"OpenAI response parse failed: {e} body={str(data)[:500]}")


class TextReq(BaseModel):
    text: str
    openai_model: Optional[str] = None
    system_prompt: Optional[str] = None


class TtsReq(BaseModel):
    text: str
    voice_id: Optional[str] = None  # relative to voices/ (points to a .onnx)
    lang_tag: Optional[str] = None  # e.g. en-US, zh-CN; used if voice_id is omitted
    length_scale: Optional[float] = None  # >1 slower, <1 faster (piper)


app = FastAPI(title="PhoneBot AI Backend", version="0.1")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health")
def health():
    piper_bin = os.getenv("PIPER_BIN") or shutil.which("piper")
    voices = list(PIPER_VOICES_DIR.rglob("*.onnx")) if PIPER_VOICES_DIR.exists() else []
    return {
        "ok": True,
        "whisper_bin": str(WHISPER_BIN),
        "whisper_bin_exists": WHISPER_BIN.exists(),
        "default_whisper_model": DEFAULT_WHISPER_MODEL_NAME,
        "default_openai_model": DEFAULT_OPENAI_MODEL,
        "openai_key_set": bool(os.getenv("OPENAI_API_KEY")),
        "piper_bin": piper_bin,
        "voices_dir": str(PIPER_VOICES_DIR),
        "voices_count": len(voices),
    }


def _piper_bin() -> str:
    p = os.getenv("PIPER_BIN") or shutil.which("piper")
    if not p:
        raise FileNotFoundError("piper binary not found. Install piper or set PIPER_BIN=/path/to/piper")
    return p


def _voice_root_for_lang_tag(lang_tag: str) -> Optional[Path]:
    tag = (lang_tag or "").strip()
    if not tag:
        return None
    # piper-voices layout: voices/<lang>/<locale>/...
    # Examples:
    # - en-US -> voices/en/en_US
    # - en-GB -> voices/en/en_GB
    # - zh-CN -> voices/zh/zh_CN
    parts = tag.replace("_", "-").split("-")
    if len(parts) < 2:
        return None
    lang = parts[0].lower()
    region = parts[1].upper()
    if not PIPER_VOICES_DIR.exists():
        return None
    return PIPER_VOICES_DIR / lang / f"{lang}_{region}"


def _safe_resolve_voice_id(voice_id: str) -> Path:
    if not voice_id:
        raise FileNotFoundError("voice_id is empty")
    p = (PIPER_VOICES_DIR / voice_id).resolve()
    base = PIPER_VOICES_DIR.resolve()
    if base not in p.parents and p != base:
        raise FileNotFoundError("voice_id escapes voices dir")
    if p.suffix.lower() != ".onnx":
        raise FileNotFoundError("voice_id must point to a .onnx model file")
    if not p.exists():
        raise FileNotFoundError(f"voice model not found: {p}")
    return p


def list_piper_voices(lang_tag: Optional[str] = None) -> list[dict]:
    if not PIPER_VOICES_DIR.exists():
        return []
    root = _voice_root_for_lang_tag(lang_tag or "") if lang_tag else None
    search_dir = root if root and root.exists() else PIPER_VOICES_DIR
    out: list[dict] = []
    for model_path in sorted(search_dir.rglob("*.onnx")):
        rel = model_path.relative_to(PIPER_VOICES_DIR).as_posix()
        label = model_path.stem  # e.g. en_US-ryan-medium
        config_path = model_path.with_suffix(model_path.suffix + ".json")  # *.onnx.json
        out.append(
            {
                "voice_id": rel,
                "label": label,
                "has_config": config_path.exists(),
                "lang_folder": str(model_path.parent.relative_to(PIPER_VOICES_DIR).parts[0]) if model_path.parent != PIPER_VOICES_DIR else "",
            }
        )
    return out


def preprocess_for_tts(text: str) -> str:
    # Conservative replacements so symbols/numbers read better in TTS engines.
    # Keeps meaning but avoids weird punctuation reading.
    s = (text or "").replace("\r\n", "\n").replace("\n", ". ")
    s = s.replace("*", " star ").replace("#", " hash ").replace("_", " underscore ")
    s = s.replace("/", " slash ").replace("\\", " backslash ")
    s = " ".join(s.split())
    return s.strip()


def run_piper_tts(text: str, voice_model_path: Path, length_scale: Optional[float]) -> bytes:
    piper = _piper_bin()
    cfg = voice_model_path.with_suffix(voice_model_path.suffix + ".json")
    tmpdir = Path(tempfile.mkdtemp(prefix="phonebot_piper_"))
    try:
        out_wav = tmpdir / "out.wav"
        cmd = [piper, "--model", str(voice_model_path), "--output_file", str(out_wav)]
        if cfg.exists():
            cmd += ["--config", str(cfg)]
        if length_scale is not None:
            cmd += ["--length_scale", str(float(length_scale))]
        proc = subprocess.run(
            cmd,
            input=preprocess_for_tts(text).encode("utf-8"),
            capture_output=True,
            check=False,
        )
        if proc.returncode != 0:
            stderr = (proc.stderr or b"").decode("utf-8", errors="ignore")
            raise RuntimeError(f"piper failed rc={proc.returncode} stderr={stderr.strip()[:500]}")
        if not out_wav.exists():
            raise RuntimeError("piper did not produce output wav")
        return out_wav.read_bytes()
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


@app.get("/api/tts/voices")
def tts_voices(lang_tag: str = ""):
    return {"voices": list_piper_voices(lang_tag=lang_tag.strip() or None)}


@app.post("/api/tts/speak")
def tts_speak(req: TtsReq):
    text = (req.text or "").strip()
    if not text:
        raise HTTPException(status_code=400, detail="text is empty")
    try:
        if req.voice_id:
            model_path = _safe_resolve_voice_id(req.voice_id)
        else:
            voices = list_piper_voices(lang_tag=(req.lang_tag or "").strip() or None)
            if not voices:
                raise FileNotFoundError(
                    "No voices found. Check AI_backend/voices and/or lang_tag. "
                    "Example lang_tag: en-US, zh-CN"
                )
            model_path = _safe_resolve_voice_id(voices[0]["voice_id"])
        wav_bytes = run_piper_tts(text=text, voice_model_path=model_path, length_scale=req.length_scale)
        return Response(content=wav_bytes, media_type="audio/wav")
    except FileNotFoundError as e:
        raise HTTPException(status_code=500, detail=f"tts misconfigured: {e}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/llm/text")
def llm_text(req: TextReq):
    prompt = (req.text or "").strip()
    if not prompt:
        raise HTTPException(status_code=400, detail="text is empty")
    model = (req.openai_model or DEFAULT_OPENAI_MODEL).strip()
    system_prompt = (req.system_prompt or DEFAULT_OPENAI_SYSTEM).strip()
    try:
        reply = openai_chat(prompt=prompt, model=model, system_prompt=system_prompt)
        return {"transcript": prompt, "reply": reply}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/llm/audio")
async def llm_audio(
    audio: UploadFile = File(...),
    whisper_model: str = Form(DEFAULT_WHISPER_MODEL_NAME),
    whisper_language: str = Form(DEFAULT_WHISPER_LANG),
    openai_model: str = Form(DEFAULT_OPENAI_MODEL),
    system_prompt: str = Form(DEFAULT_OPENAI_SYSTEM),
):
    suffix = Path(audio.filename or "audio.wav").suffix
    if suffix.lower() not in [".wav", ".mp3", ".flac", ".ogg"]:
        # whisper-cli supports more, but keep guardrails.
        suffix = ".wav"

    tmpdir = Path(tempfile.mkdtemp(prefix="phonebot_ai_"))
    try:
        in_path = tmpdir / f"input{suffix}"
        with in_path.open("wb") as f:
            shutil.copyfileobj(audio.file, f)

        stt = run_whisper_stt(audio_path=in_path, model_name=whisper_model, language=whisper_language)
        transcript = stt.text.strip()
        if not transcript:
            # If no speech, still return something useful.
            return JSONResponse({"transcript": "", "reply": "(no speech detected)"})

        reply = openai_chat(prompt=transcript, model=openai_model, system_prompt=system_prompt)
        return {"transcript": transcript, "reply": reply}
    except FileNotFoundError as e:
        raise HTTPException(status_code=500, detail=f"backend misconfigured: {e}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        try:
            shutil.rmtree(tmpdir, ignore_errors=True)
        except Exception:
            pass


if __name__ == "__main__":
    import uvicorn

    host = os.getenv("AI_BACKEND_HOST", "0.0.0.0")
    port = int(os.getenv("AI_BACKEND_PORT", "8088"))
    uvicorn.run("server:app", host=host, port=port, reload=False)

