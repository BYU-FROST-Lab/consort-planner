# Sourced by the scripts here: moves to the repository root and sets up the
# environment. Override JULIA or PY if they are not on PATH / in .venv.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export JULIA_PROJECT="$PWD"   # planner subprocesses of the sweep inherit the project through this
export GKSwstype=100          # headless plotting
JULIA="${JULIA:-julia}"
PY="${PY:-$PWD/.venv/bin/python}"; [ -x "$PY" ] || PY=python3
mkdir -p results/new figures/new

FAILED=0
check() {  # check <label> <expected> <got>
  if [ "$2" = "$3" ]; then printf '  ok    %-34s %s\n' "$1" "$3"
  else printf '  FAIL  %-34s expected %s, got %s\n' "$1" "$2" "$3"; FAILED=1; fi
}
finish() { [ $FAILED -eq 0 ] && echo "ALL CHECKS PASSED" || { echo "SOME CHECKS FAILED"; exit 1; }; }
num() { awk -v k="$2:" '$1==k {print $2; exit}' "$1" | xargs printf "%.${3}f"; }   # num <results.yaml> <key> <decimals>
