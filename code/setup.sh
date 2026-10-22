#!/usr/bin/env bash
# Install the pinned Julia packages and create .venv for the figure scripts.
set -e; source "$(dirname "$0")/experiments/env.sh"
"$JULIA" -e 'using Pkg; Pkg.instantiate(); Pkg.precompile(); Pkg.status()'
"${PYTHON:-python3}" -m venv .venv
.venv/bin/pip install -q -r requirements.txt
.venv/bin/python -c "import matplotlib, numpy, scipy; print('python packages ok')"
