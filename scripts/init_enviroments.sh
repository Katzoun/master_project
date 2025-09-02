#!/bin/bash

# Nastaveni workspace
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"
mkdir venvs
echo "Workspace directory: $WORKSPACE_DIR"

echo "=== Creating Camera Package Venv ==="
python3 -m venv venvs/camera_pkg_venv --system-site-packages
touch venvs/camera_pkg_venv/COLCON_IGNORE
source venvs/camera_pkg_venv/bin/activate
pip install -r scripts/requirements/requirements_camera_pkg.txt
deactivate
echo "=== CAMERA PKG VENV COMPLETED ==="

echo "=== Creating ABB RWS Package Venv ==="
python3 -m venv venvs/abb_rws_pkg_venv --system-site-packages
touch venvs/abb_rws_pkg_venv/COLCON_IGNORE
source venvs/abb_rws_pkg_venv/bin/activate
pip install -r scripts/requirements/requirements_abb_rws_pkg.txt
deactivate
echo "=== ABB RWS PKG VENV COMPLETED ==="

echo "=== Creating GUI Package Venv ==="
python3 -m venv venvs/gui_pkg_venv --system-site-packages
touch venvs/gui_pkg_venv/COLCON_IGNORE
source venvs/gui_pkg_venv/bin/activate
pip install -r scripts/requirements/requirements_gui_pkg.txt
deactivate
echo "=== GUI PKG VENV COMPLETED ==="


echo "=== All virtual environments created ==="