import asyncio
import json
import subprocess
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from pathlib import Path
import base64

from .tinydb.db import create_session, save_consent, list_sessions, list_consents, complete_session
from backend.ai_image.sd_pipeline.run_gemini_only import OUT_DIR, PROJECT_ROOT, run_gemini_only
from backend.ai_image.agent.caricature_agent import run_caricature_agent

app = FastAPI()

# Allow  Vue dev server to talk to this API
origins = [
    "https://www.drawmemaybe.local",
    "http://localhost:5173",
    "http://127.0.0.1:5173",
]

app.add_middleware(
    CORSMiddleware,
    #allow_origins=["*"],
    allow_origins=origins,
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

BASE_DIR = Path(__file__).resolve().parent  # Ordner backend/
ASSETS_DIR = BASE_DIR / "ai_image" / "sd_pipeline" / "assets"
AGENT_JSON = PROJECT_ROOT / "backend/ai_image/sd_pipeline/agent_output.json"


class ConsentPayload(BaseModel):
    session_id: str
    consent_given: bool
    name: str | None = None

class SessionCompletePayload(BaseModel):
    session_id: str

class ChatPayload(BaseModel):
    user_text: str
    session_id: str
    
class PhotoPayload(BaseModel):
    session_id: str
    data_url: str

class RunGeminiPayload(BaseModel):
    session_id: str

class GenerateImagePayload(BaseModel):
    session_id: str

@app.get("/health")
def health_check():
    return {"status": "ok"}


@app.post("/session")
def create_session_endpoint():
    session_id = create_session()
    return {"session_id": session_id}


@app.post("/consent")
def consent_endpoint(payload: ConsentPayload):
    save_consent(
        session_id=payload.session_id,
        consent_given=payload.consent_given,
        name=payload.name,
    )
    return {"ok": True}

@app.post("/session/complete")
def complete_session_endpoint(payload: SessionCompletePayload):
    """
    Markiert eine Session als abgeschlossen.
    Wird normalerweise vom letzten Screen (z.B. Danke-Screen) aufgerufen.
    """
    success = complete_session(payload.session_id)
    if not success:
        # Session nicht gefunden
        raise HTTPException(status_code=404, detail="Session not found")
    return {"ok": True}


@app.get("/debug/sessions")
def debug_sessions():
    """
    Debug endpoint: returns all sessions and consents from TinyDB.
    """
    sessions = list_sessions()
    consents = list_consents()
    return {
        "sessions": sessions,
        "consents": consents,
    }

@app.post("/api/chat")
async def chat_endpoint(payload: ChatPayload):
    """
    Main chat endpoint:
    - receives user text
    - runs Ollama caricature agent
    - returns structured result
    """

    if not payload.user_text or len(payload.user_text.strip()) < 3:
        return {"success": False, "error": "Kein oder zu kurzer Text"}
    result = run_caricature_agent(payload.user_text)


    # überschreibt immer die JSON-Datei
    AGENT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with open(AGENT_JSON, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    
    image_path = f"backend/ai_image/sd_pipeline/assets/{payload.session_id}.png"
    timeout = 300
    elapsed = 0
    while not Path(image_path).exists() and elapsed < timeout:
        await asyncio.sleep(1)
        elapsed += 1

    
    if not Path(image_path).exists():
        return {"success": False, "error": "Timeout: kein Bild vom User."}
    
    subprocess.run([
        "python3.12",
        "backend/ai_image/sd_pipeline/run_gemini_only.py",
        payload.session_id
    ])

    return {
        "success": True,
        "session_id": payload.session_id,
        **result
    }

@app.post("/photo")
async def upload_photo(payload: PhotoPayload):
    if not payload.data_url or "," not in payload.data_url:
        return {"ok": False, "error": "No valid data URL provided"}
    
    header, _, b64data = payload.data_url.partition(",")
    if not b64data.strip():
        return {"ok": False, "error": "Data URL contains no image data"}

    img_bytes = base64.b64decode(b64data)
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    img_path = ASSETS_DIR / f"{payload.session_id}.png"
    img_path.write_bytes(img_bytes)

    return {"ok": True, "path": str(img_path)}

@app.post("/run_gemini")
async def run_gemini_endpoint(payload: RunGeminiPayload):
    result = run_gemini_only(payload.session_id)
    return {"ok": True, "path": str(result)}

import asyncio
OUT_DIR = Path("backend/ai_image/sd_pipeline/assets")

async def wait_for_image(session_id: str, timeout: int = 30):
    image_path = OUT_DIR / f"{session_id}.png"
    waited = 0
    while not image_path.exists():
        await asyncio.sleep(1)
        waited += 1
        if waited >= timeout:
            raise TimeoutError(f"⚠️ Input image for session {session_id} not found after {timeout}s")
    return image_path

async def wait_for_image(session_id: str, timeout: int = 30) -> Path:

    image_path = OUT_DIR / f"{session_id}.png"
    waited = 0
    while not image_path.exists():
        await asyncio.sleep(1)
        waited += 1
        if waited >= timeout:
            raise TimeoutError(f"⚠️ Input image for session {session_id} not found after {timeout}s")
    return image_path

import asyncio

def run_gemini_sync(session_id: str):
    return run_gemini_only(session_id)

@app.post("/generate-image")
async def generate_image(payload: RunGeminiPayload):
    session_id = payload.session_id
    loop = asyncio.get_event_loop()
    try:
        # run_gemini_only in einem Thread ausführen, ohne future.result()
        out_path = await loop.run_in_executor(None, run_gemini_sync, session_id)
        return {"ok": True, "path": str(out_path)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


    