#!/bin/bash

# 引数取得
MAC_ADDR=$1
RFCOMM_ID=$2
CHANNEL=$3

DEV_NAME="/dev/rfcomm${RFCOMM_ID}"

echo "[Drogger Setup] Setting up Bluetooth connection..."
echo "  Target MAC: ${MAC_ADDR}"
echo "  Device: ${DEV_NAME}"

# 1. 既存の接続があれば解放する (再起動時などのため)
if [ -e "${DEV_NAME}" ]; then
    echo "[Drogger Setup] Releasing existing ${DEV_NAME}..."
    # sudoが必要な場合があります
    rfcomm release ${RFCOMM_ID}
    sleep 1
fi

# 2. rfcomm bindを実行
# これにより /dev/rfcommX が作成され、アクセス時に自動接続する待機状態になる
echo "[Drogger Setup] Binding ${DEV_NAME} to ${MAC_ADDR} channel ${CHANNEL}..."

# コマンド実行 (エラーなら終了)
rfcomm bind ${RFCOMM_ID} ${MAC_ADDR} ${CHANNEL}
if [ $? -ne 0 ]; then
    echo "[Drogger Setup] Error: Failed to bind rfcomm."
    exit 1
fi

echo "[Drogger Setup] Success! ${DEV_NAME} is ready."
# 権限を緩める（必要に応じて）
chmod 666 ${DEV_NAME} 2>/dev/null