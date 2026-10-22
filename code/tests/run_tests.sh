#!/usr/bin/env bash
# The planner's tests (about 1 minute).
set -u; source "$(dirname "$0")/../experiments/env.sh"
for t in tests/test_*.jl; do
  "$JULIA" "$t" > results/new/$(basename $t .jl).log 2>&1 && check "$(basename $t)" pass pass || check "$(basename $t)" pass fail
done
finish
