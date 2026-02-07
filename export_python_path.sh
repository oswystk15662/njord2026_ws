# export_python_path.sh
if [ -z "$VIRTUAL_ENV" ]; then
  echo "VIRTUAL_ENV is empty. Activate venv first."
  return 1 2>/dev/null || exit 1
fi

PYVER=$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
export PYTHONPATH="$VIRTUAL_ENV/lib/python${PYVER}/site-packages:$PYTHONPATH"
