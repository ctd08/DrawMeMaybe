#!/bin/bash


echo ==== 1. Frontend Build ====
cd /home/drawmemaybe/drawmemaybe/DrawMeMaybe/frontend
npm run build
cd /home/drawmemaybe/drawmemaybe/DrawMeMaybe

echo ==== 2. Backend Starten ====
./start_backend.sh &

echo ==== 3. Starting Ollama  ===
sudo systemctl start ollama

echo ==== 4. Running Agent ====
python3.12 backend/ai_image/sd_pipeline/run_agent_only.py


echo ==== 5. Stopping Ollama service ====
sudo systemctl stop ollama

echo ==== 6. Start Gemini ====
python3.12 backend/ai_image/sd_pipeline/run_gemini_only.py

echo ==== 7. To SVG starten ====
python3.12 backend/image_to_svg/to_svg.py

#echo ==== 8. Roboter ====
#cd ~/Drawmemaybe/ros2_ws
#source ./install/setup.bash
#ros2 run my_sample_pkg_cpp move_moveit





