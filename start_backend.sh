#!/bin/bash
#cd "$(dirname "$0")/.."  # Go to project root
export PYTHONPATH="${PWD}"
echo "PYTHONPATH: $PYTHONPATH"
python3 -m uvicorn backend.main:app  --host 127.0.0.1 --port 8000
.