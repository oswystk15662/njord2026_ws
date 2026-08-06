#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
generated_file="$(mktemp)"

{
  printf '%s\n' '"use strict";' '' 'const L = (() => {' \
    '  const leafletModule = {exports: {}};' \
    '  const module = leafletModule;' \
    '  const exports = leafletModule.exports;'
  cat "${package_dir}/vendor/leaflet.js"
  printf '%s\n' '  return leafletModule.exports;' '})();' ''
  cat "${package_dir}/src/extension.js"
} > "${generated_file}"

mv "${generated_file}" "${package_dir}/dist/extension.js"
cd "${package_dir}"
version="$(sed -n 's/^[[:space:]]*"version":[[:space:]]*"\([^"]*\)".*/\1/p' package.json)"
archive="gnss-map-telemetry-${version}.foxe"
if command -v zip >/dev/null 2>&1; then
  zip -r -FS "${archive}" package.json dist
elif command -v jar >/dev/null 2>&1; then
  rm -f "${archive}"
  jar --create --no-manifest --file "${archive}" package.json dist
else
  printf 'Packaging requires either zip or jar.\n' >&2
  exit 1
fi
