#!/usr/bin/env bash
# Install the Quanser SDK + Python APIs on the 4090 (Ubuntu 24.04) and make them importable from the `qarm`
# conda env. Needs sudo -> run it yourself in a terminal on the 4090 (it will ask for your password):
#     bash ~/course_project/env/4090/install_quanser_sdk.sh
# Safe to re-run. What it does:
#   1. adds Quanser's apt repository with Quanser's own script (repo.quanser.com, from github.com/quanser/quanser_sdk_linux)
#   2. apt installs `quanser-sdk` and `python3-quanser-apis` (system python 3.12)
#   3. exposes ONLY the `quanser` package (not the rest of the system site-packages) to the conda env through a
#      folder of symlinks + a .pth file, and puts Quanser's `pal`/`hal` (~/Quanser/Quanser_Academic_Resources) on its path
#   4. runs an import check
set -euo pipefail

if [ ! -f /etc/apt/sources.list.d/quanser.sources ]; then
  tmp=$(mktemp -d)
  ( cd "$tmp" && wget -q --no-cache https://repo.quanser.com/debian/release/config/configure_repo.sh \
      && chmod u+x configure_repo.sh && ./configure_repo.sh )
  rm -rf "$tmp"
fi
sudo apt update
sudo apt-get install -y quanser-sdk python3-quanser-apis

# --- expose `quanser` to the conda env --------------------------------------------------------------------
ENV_PY=~/miniconda3/envs/qarm/bin/python
SITE=$("$ENV_PY" -c "import site; print(site.getsitepackages()[0])")
LINKDIR=~/.local/share/qarm_quanser_site
rm -rf "$LINKDIR" && mkdir -p "$LINKDIR/quanser"
for p in $(/usr/bin/python3 -c "import quanser; print(' '.join(quanser.__path__))"); do
  for child in "$p"/*; do ln -sfn "$child" "$LINKDIR/quanser/$(basename "$child")"; done
done
echo "$LINKDIR" > "$SITE/quanser_system.pth"
echo "$HOME/Quanser/Quanser_Academic_Resources/0_libraries/python" > "$SITE/quanser_pal.pth"

"$ENV_PY" - <<'EOF'
import quanser.hardware, quanser.common
from pal.products.qarm import QArm
print("OK: quanser.hardware + pal.products.qarm import in the qarm env")
EOF
echo "groups: $(id -nG)  (dialout/plugdev needed for USB devices)"
