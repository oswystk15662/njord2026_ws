# シリアル通信仕様

## PC → ESP（17 バイト）

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

## ESP → PC（1 バイト）

```
  bit7  bit6  bit5  bit4  bit3  bit2  bit1      bit0
[  0  ][  0  ][  0  ][  0  ][  0  ][  0  ][  0  ][ 緊急停止リレー動作状態 ]
```

| ビット | 内容 |
|---|---|
| bit0 | 緊急停止リレーの動作状態（1 = 動作中） |
