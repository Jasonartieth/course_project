#!/usr/bin/env bash
# Create the `qarm` conda env on the 4090 machine: Genesis + PyTorch (CUDA 12.8 wheels) + analysis libs.
# - Never touches other conda envs; safe to re-run (env creation is skipped if it already exists).
# - Uses conda-forge only (avoids the Anaconda default-channel ToS prompt in non-interactive shells).
# - Quanser's Python API is NOT installed here: it comes from apt (python3-quanser-apis), see install_quanser_sdk.sh.
set -euo pipefail
ENV_NAME=qarm
HERE="$(cd "$(dirname "$0")" && pwd)"

source ~/miniconda3/etc/profile.d/conda.sh
if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  conda create -y -n "$ENV_NAME" --override-channels -c conda-forge python=3.12
fi
conda activate "$ENV_NAME"

python -m pip install --upgrade pip
python -m pip install torch --index-url https://download.pytorch.org/whl/cu128
python -m pip install -r "$HERE/requirements-4090.txt"

# Make our package importable without installing it (src layout).
SITE=$(python -c "import site; print(site.getsitepackages()[0])")
echo "$HOME/course_project/src" > "$SITE/qarm_lab_src.pth"

python -m pip freeze > "$HERE/requirements-4090.lock.txt"
python - <<'EOF'
import torch, genesis, qarm_lab
print("torch", torch.__version__, "| cuda available:", torch.cuda.is_available())
print("genesis", genesis.__version__, "| qarm_lab", qarm_lab.__version__)
EOF
