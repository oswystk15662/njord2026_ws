#!/bin/bash

set -u

# 引数取得
MAC_ADDR=$1
RFCOMM_ID=$2
CHANNEL=$3

DEV_NAME="/dev/rfcomm${RFCOMM_ID}"

echo "[Drogger Setup] Setting up Bluetooth connection..."
echo "  Target MAC: ${MAC_ADDR}"
echo "  Device: ${DEV_NAME}"

is_bound_to_target() {
    rfcomm 2>/dev/null | grep -Ei "^rfcomm${RFCOMM_ID}:.*${MAC_ADDR}" >/dev/null
}

# 1. 既存の接続があれば解放する (再起動時などのため)
if [ -e "${DEV_NAME}" ] || is_bound_to_target; then
    echo "[Drogger Setup] Releasing existing ${DEV_NAME}..."
    rfcomm release ${RFCOMM_ID} || true
    sleep 1
fi

# 2. rfcomm bindを実行
# これにより /dev/rfcommX が作成され、アクセス時に自動接続する待機状態になる
echo "[Drogger Setup] Binding ${DEV_NAME} to ${MAC_ADDR} channel ${CHANNEL}..."

bind_ok=0
for attempt in 1 2 3; do
    bind_output="$(rfcomm bind ${RFCOMM_ID} ${MAC_ADDR} ${CHANNEL} 2>&1)"
    bind_rc=$?

    if [ ${bind_rc} -eq 0 ] || is_bound_to_target; then
        bind_ok=1
        break
    fi

    if echo "${bind_output}" | grep -q "Address already in use"; then
        echo "[Drogger Setup] Bind attempt ${attempt}: address already in use, releasing and retrying..."
        rfcomm release ${RFCOMM_ID} || true
        sleep 1
        continue
    fi

    echo "[Drogger Setup] Error: Failed to bind rfcomm (attempt ${attempt})."
    echo "[Drogger Setup] rfcomm output: ${bind_output}"
    sleep 1
done

if [ ${bind_ok} -ne 1 ]; then
    echo "[Drogger Setup] Error: Could not establish rfcomm bind after retries."
    exit 1
fi

if [ ! -e "${DEV_NAME}" ]; then
    echo "[Drogger Setup] Error: ${DEV_NAME} was not created."
    exit 1
fi

if ! chgrp dialout ${DEV_NAME} 2>/dev/null; then
    echo "[Drogger Setup] Warning: could not change group for ${DEV_NAME} to dialout."
fi

if ! chmod 660 ${DEV_NAME} 2>/dev/null; then
    echo "[Drogger Setup] Warning: could not change permissions for ${DEV_NAME}."
fi

echo "[Drogger Setup] Success! ${DEV_NAME} is ready."