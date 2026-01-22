#!/bin/bash

SESSION_ID="§{1:-default_session}"

echo "Session ID: $SESSION_ID"

echo ==== 1. Frontend Build ====
cd /home/drawmemaybe/drawmemaybe/DrawMeMaybe/frontend
npm run build
cd /home/drawmemaybe/drawmemaybe/DrawMeMaybe

echo ==== 2. Backend Starten ====
./start_backend.sh &

echo ==== 3. Starting Ollama  ===
sudo systemctl start ollama

echo "✅ Warten auf User: Text + Bild…"
# NICHTS automatisch ausführen!
