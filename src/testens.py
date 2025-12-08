import torch
from diffusers import StableDiffusionImg2ImgPipeline
from PIL import Image
import random
from huggingface_hub import hf_hub_download

# LoRA-Datei herunterladen
#path = hf_hub_download(
#    repo_id="Margaritahse1/cartoon_style_LoRA", 
#   filename="cartoon_style_LoRA.safetensors"
#)

#print("LoRA gespeichert unter:", path)

print(torch.__version__)
print(torch.cuda.is_available())

# Bild laden
init_image = Image.open("src/assets/whiteguy2.png").convert("RGB")
init_image = init_image.resize((512, 512))

device = "cuda" if torch.cuda.is_available() else "cpu"

# Pipeline laden
pipe = StableDiffusionImg2ImgPipeline.from_pretrained(
    "Linaqruf/anything-v3.0",
    torch_dtype=torch.float16 if device == "cuda" else torch.float32,
).to("cuda")


# Besserer Karikatur-Prompt 
prompt = (
    "caricature portrait, clean cartoon style, smooth lines, exaggerated but attractive "
    "features, big head, small body, vibrant colors, cute funny style"
)

# Werte tun gut für Karikaturen
strength = 0.7            # weniger zerstören, mehr stylisieren
guidance_scale = 7.5       # etwas weicher
seed = random.randint(0, 1_000_000_000)
generator = torch.Generator(device).manual_seed(seed)

# Bild generieren
result = pipe(
    prompt=prompt,
    image=init_image,
    strength=strength,
    guidance_scale=guidance_scale,
    generator=generator
).images[0]

# Speichern
result.save("src/assets/caricature_result.png")
print("Fertig! Seed:", seed)
