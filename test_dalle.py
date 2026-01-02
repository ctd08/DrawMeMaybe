from pathlib import Path
from io import BytesIO
import os

from openai import OpenAI
from PIL import Image
import requests

# Basisordner = Ordner dieser Datei
BASE_DIR = Path(__file__).resolve().parent
ASSETS_DIR = BASE_DIR / "test_assets"
ASSETS_DIR.mkdir(exist_ok=True)

input_path = ASSETS_DIR / "red_head.png"
output_png_path = ASSETS_DIR / "red_head_out.png"
edited_path = ASSETS_DIR / "edited_image.png"

# Bild nach PNG konvertieren
img = Image.open(input_path).convert("RGBA")
img.save(output_png_path, format="PNG")

# OpenAI-Client (Key aus Umgebung)
#client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])

client = OpenAI(api_key="")

prompt = (
    "caricature of the person, exaggerated facial features, "
    "add a small guitar in their hands naturally, "
    "stylized, readable, simple drawing, "
    "clean background"
)

with open(output_png_path, "rb") as f:
    response = client.images.edit(
        model="dall-e-2",
        image=f,
        prompt=prompt,
        size="512x512",
    )

image_url = response.data[0].url

res = requests.get(image_url)
img = Image.open(BytesIO(res.content))
img.save(edited_path)

print(f"Fertig! Bild gespeichert unter: {edited_path}")
