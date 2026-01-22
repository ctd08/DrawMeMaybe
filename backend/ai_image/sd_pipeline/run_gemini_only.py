from google import genai
from google.genai import types
from PIL import Image
import json
from pathlib import Path
import os
import threading

PROJECT_ROOT = Path(__file__).resolve().parents[3]
AGENT_JSON = PROJECT_ROOT / "backend" / "ai_image" / "sd_pipeline" / "agent_output.json"
OUT_DIR = PROJECT_ROOT / "backend" / "ai_image" / "sd_pipeline" / "assets"

def get_gemini_client() -> genai.Client:
    api_key = os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")
    if not api_key:
        raise RuntimeError("⚠️ GOOGLE_API_KEY oder GEMINI_API_KEY fehlt")
    return genai.Client(api_key=api_key)

def run_gemini_only(session_id: str, timeout: int = 30) -> Path:
    if not session_id:
        raise ValueError("⚠️ Session ID ist leer! Kann Bild nicht finden.")

    client = get_gemini_client()
    print(f"⏳ Running Gemini with session_id={session_id}")

    # Load agent JSON
    if not AGENT_JSON.exists():
        raise FileNotFoundError(f"⚠️ Agent JSON nicht gefunden: {AGENT_JSON}")
    with open(AGENT_JSON, "r", encoding="utf-8") as f:
        agent_result = json.load(f)
    prompt = agent_result.get("caricature_prompt", "")
    if not prompt:
        raise RuntimeError("⚠️ Kein caricature_prompt im Agent JSON gefunden")
    print(f"Prompt length: {len(prompt)}")

    # Input image
    image_path = OUT_DIR / f"{session_id}.png"
    if not image_path.exists():
        raise FileNotFoundError(f"⚠️ Input image für Gemini nicht gefunden: {image_path}")
    input_image = Image.open(image_path)
    print(f"Input image exists? {image_path.exists()} size: {input_image.size}")

    # Gemini call in Thread mit Timeout
    response_holder = [None]
    exception_holder = [None]

    def call_gemini():
        try:
            response_holder[0] = client.models.generate_content(
                model="gemini-2.5-flash-image",
                contents=[prompt, input_image],
                config=types.GenerateContentConfig(
                    response_modalities=["IMAGE"],
                ),
            )
        except Exception as e:
            exception_holder[0] = e

    thread = threading.Thread(target=call_gemini)
    thread.start()
    thread.join(timeout=timeout)
    if thread.is_alive():
        raise TimeoutError(f"⚠️ Gemini call timed out nach {timeout}s")
    if exception_holder[0]:
        raise exception_holder[0]

    response = response_holder[0]
    if not response.candidates:
        raise RuntimeError("⚠️ Gemini returned no candidates")
    cand = response.candidates[0]
    content = getattr(cand, "content", None)
    if not content or not getattr(content, "parts", None):
        raise RuntimeError("⚠️ Gemini returned kein Bild")

    # Output image
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_image = OUT_DIR / f"{session_id}_caricature.png"

    saved = False
    for part in content.parts:
        img = part.as_image()
        if img:
            img.save(out_image)
            saved = True
            break
    if not saved:
        raise RuntimeError("⚠️ Kein Bild gespeichert – Gemini returned parts empty")

    print(f"✓ Gemini caricature saved at {out_image}")
    return out_image


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        raise RuntimeError("Usage: run_gemini_only.py <session_id>")
    session_id = sys.argv[1]
    run_gemini_only(session_id)
