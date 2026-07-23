# シリアル通信仕様

## PC → ESP

UART は 115200 bps, 8N1 です。PC から ESP32 へ送るデータは
`Docs/PROTOCOL.md` と同じ形式です。

```text
COBS(raw_frame) || 0x00
```

`raw_frame` は 24 byte 固定です。

| Offset | Size | Field | Value |
|---:|---:|---|---|
| 0 | 1 B | version | `0x01` |
| 1 | 1 B | type | `0x01` (`THRUSTER_COMMAND`) |
| 2 | 2 B | sequence | little-endian, 送信ごとに増加 |
| 4 | 1 B | payload_length | `17` |
| 5 | 17 B | payload | 下記 |
| 22 | 2 B | crc | CRC-16/CCITT-FALSE, little-endian |

CRC は version から payload 末尾までの 22 byte を対象にする。

## Payload（17 バイト）

| フィールド | 型 | サイズ | 説明 |
|---|---|---|---|
| th1 | float32 | 4 B | スラスタ 1 の推力（N） |
| th2 | float32 | 4 B | スラスタ 2 の推力（N） |
| th3 | float32 | 4 B | スラスタ 3 の推力（N） |
| th4 | float32 | 4 B | スラスタ 4 の推力（N） |
| setting | uint8 | 1 B | 各種制御フラグ（後述） |

推力からデューティ比への変換には `thrust_duty_profile.hpp` を使用すること。

### setting バイト（各 1 bit）

```
  bit7  bit6  bit5  bit4   bit3     bit2    bit1    bit0
[  0  ][  0  ][  0  ][  0  ][ 緊急停止 ][ LED緑 ][ LED黄 ][ LED赤 ]
```

| ビット | 内容 |
|---|---|
| bit3 | 緊急停止（1 = 停止指令） |
| bit2 | LED 緑（1 = 点灯） |
| bit1 | LED 黄（1 = 点灯） |
| bit0 | LED 赤（1 = 点灯） |

---

## 安全動作

不正 frame、CRC 不一致、非有限 float は出力へ反映しない。

software emergency stop が 1、または有効な指令が 1000 ms 途絶した場合、
全スラスターを中立にして FET を LOW にする。GPIO2 の物理 E-stop 入力は
firmware では使用しない。

## ESP → PC

現 firmware は protocol frame の応答を返さない。

旧 firmware は 1 byte のリレー状態を返していた。

```
  bit7  bit6  bit5  bit4  bit3  bit2  bit1      bit0
[  0  ][  0  ][  0  ][  0  ][  0  ][  0  ][  0  ][ 緊急停止リレー動作状態 ]
```

| ビット | 内容 |
|---|---|
| bit0 | 緊急停止リレーの動作状態（1 = 動作中） |
