#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 INPUT.onnx OUTPUT.engine" >&2
  exit 2
fi
onnx=$1
engine=$2
if [[ ! -r "$onnx" ]]; then
  echo "input ONNX does not exist or is not readable: $onnx" >&2
  exit 1
fi
command -v trtexec >/dev/null || { echo "trtexec was not found in PATH" >&2; exit 1; }
mkdir -p "$(dirname "$engine")"
log="${engine}.build.log"
manifest="${engine}.manifest.json"
input_name="$(python3 - "$onnx" <<'PY'
import sys
import onnx
model = onnx.load(sys.argv[1])
if len(model.graph.input) != 1:
    raise SystemExit("ONNX must have exactly one input")
print(model.graph.input[0].name)
PY
)"
trtexec --onnx="$onnx" --saveEngine="$engine" --fp16 \
  "--minShapes=${input_name}:1x3x640x640" "--optShapes=${input_name}:1x3x640x640" \
  "--maxShapes=${input_name}:1x3x640x640" 2>&1 | tee "$log"
"$(dirname "$0")/inspect_tensorrt_engine.py" "$engine" --manifest "$manifest" --onnx "$onnx"
