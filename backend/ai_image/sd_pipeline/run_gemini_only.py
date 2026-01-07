from google import genai
from google.genai import types
from PIL import Image
import json
from pathlib import Path
import os

PROJECT_ROOT = Path(__file__).resolve().parents[3]
AGENT_JSON = PROJECT_ROOT / "backend" / "ai_image" / "sd_pipeline" / "agent_output.json"
OUT_DIR = PROJECT_ROOT / "backend" / "ai_image" / "sd_pipeline" / "assets"

def get_gemini_client() -> genai.Client:
    api_key = os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")
    if not api_key:
        raise RuntimeError("GOOGLE_API_KEY oder GEMINI_API_KEY fehlt")
    return genai.Client(api_key=api_key)

def run_gemini_only(session_id: str) -> Path:
    client = get_gemini_client()

    with open(AGENT_JSON, "r", encoding="utf-8") as f:
        agent_result = json.load(f)
    prompt = agent_result["caricature_prompt"]

    image_path = OUT_DIR / f"{session_id}.png"
    input_image = Image.open(image_path)

    response = client.models.generate_content(
        model="gemini-2.5-flash-image",
        contents=[prompt, input_image],
        config=types.GenerateContentConfig(
            response_modalities=["IMAGE"],
        ),
    )

    cand = response.candidates[0]
    content = getattr(cand, "content", None)
    if not content:
        raise RuntimeError("Gemini returned no content")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_image = OUT_DIR / f"{session_id}_caricature.png"

    for part in content.parts:
        img = part.as_image()
        if img:
            img.save(out_image)
            break

    return out_image

if __name__ == "__main__":
    # Nur zum lokalen Testen:
    test_session = "test123"
    print(run_gemini_only(test_session))
