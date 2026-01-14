#!/bin/bash
echo ==== 1. Server Starten ====
./server_start.sh

echo ==== 2. Frontend Build ====
cd /home/drawmemaybe/drawmemaybe/DrawMeMaybe/frontend
npm run build
cd /home/drawmemaybe/drawmemaybe/DrawMeMaybe

echo ==== 3. Backend Starten ====
./start_backend.sh

echo ==== 4. Starting Ollama  ===
sudo systemctl start ollama

echo ==== 5. Running Agent ====
python3.12 ai_image/sd_pipeline/run_agent_only.py

echo ==== 6. Stopping Ollama service ====
sudo systemctl stop ollama

echo ==== 7. Start Gemini ====
python backend/ai_image/sd_pipeline/run_gemini_only.py





