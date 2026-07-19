#!/usr/bin/env python3
"""Benchmark YOLO11s inference speed (FPS / latency) for Task 2.

Jetson (または CUDA マシン) の YOLO venv 上で実行する想定。
ダミー画像 (ランダムノイズ) か、--image で実画像を指定して推論を回し、
平均 / p50 / p95 レイテンシと FPS を表示する。

使い方:

    python3 scripts/task2/benchmark_yolo11s.py \
        --weights /path/to/yolo11s.pt --device cuda:0 --iters 200

TensorRT engine のベンチも同じ (--weights に .engine を渡す):

    python3 scripts/task2/benchmark_yolo11s.py \
        --weights /path/to/yolo11s.engine --iters 200
"""

import argparse
import statistics
import sys
import time


def parse_args():
    parser = argparse.ArgumentParser(description='Benchmark YOLO11s inference.')
    parser.add_argument('--weights', required=True,
                        help='Path to .pt weights or .engine file')
    parser.add_argument('--imgsz', type=int, default=640,
                        help='Inference image size (default: 640)')
    parser.add_argument('--device', default='cuda:0',
                        help='Device for .pt weights (default: cuda:0). '
                             'Ignored for .engine.')
    parser.add_argument('--iters', type=int, default=100,
                        help='Number of timed iterations (default: 100)')
    parser.add_argument('--warmup', type=int, default=10,
                        help='Number of warmup iterations (default: 10)')
    parser.add_argument('--image', default=None,
                        help='Optional path to a real image. '
                             'Default: random dummy image.')
    return parser.parse_args()


def percentile(sorted_values, p):
    """単純な percentile (線形補間)。sorted_values は昇順ソート済み。"""
    if not sorted_values:
        return float('nan')
    k = (len(sorted_values) - 1) * (p / 100.0)
    lo = int(k)
    hi = min(lo + 1, len(sorted_values) - 1)
    frac = k - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def main():
    args = parse_args()

    try:
        import torch  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        print(f'ERROR: torch is not available: {exc}\n'
              'This benchmark requires a Python environment with torch installed '
              '(Jetson YOLO venv, see src/detection/yolo/README_jetson_yolo11.md). '
              'Do not pip-install torch blindly on Jetson: use the NVIDIA wheel.',
              file=sys.stderr)
        return 1

    try:
        from ultralytics import YOLO
    except Exception as exc:  # noqa: BLE001
        print(f'ERROR: ultralytics is not available: {exc}\n'
              'Activate the Jetson YOLO venv first.', file=sys.stderr)
        return 1

    import numpy as np
    import torch

    is_engine = args.weights.endswith('.engine')

    if args.image:
        import cv2
        image = cv2.imread(args.image)
        if image is None:
            print(f'ERROR: failed to read image: {args.image}', file=sys.stderr)
            return 1
        print(f'Using image {args.image} ({image.shape[1]}x{image.shape[0]})')
    else:
        rng = np.random.default_rng(0)
        image = rng.integers(0, 256, size=(args.imgsz, args.imgsz, 3),
                             dtype=np.uint8)
        print(f'Using random dummy image ({args.imgsz}x{args.imgsz})')

    print(f'Loading model: {args.weights} '
          f'({"TensorRT engine" if is_engine else "PyTorch weights"})')
    model = YOLO(args.weights)
    if not is_engine:
        model.to(args.device)

    predict_kwargs = {'imgsz': args.imgsz, 'verbose': False}

    print(f'Warmup: {args.warmup} iters')
    for _ in range(args.warmup):
        model.predict(image, **predict_kwargs)

    print(f'Benchmark: {args.iters} iters')
    latencies_ms = []
    for _ in range(args.iters):
        t0 = time.perf_counter()
        model.predict(image, **predict_kwargs)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        latencies_ms.append((time.perf_counter() - t0) * 1000.0)

    latencies_ms.sort()
    mean_ms = statistics.fmean(latencies_ms)
    p50_ms = percentile(latencies_ms, 50)
    p95_ms = percentile(latencies_ms, 95)

    print('')
    print('=== YOLO11s benchmark result ===')
    print(f'weights   : {args.weights}')
    print(f'imgsz     : {args.imgsz}')
    print(f'device    : {"(engine)" if is_engine else args.device}')
    print(f'iterations: {args.iters} (warmup {args.warmup})')
    print(f'latency   : mean {mean_ms:.2f} ms | p50 {p50_ms:.2f} ms | '
          f'p95 {p95_ms:.2f} ms')
    print(f'throughput: {1000.0 / mean_ms:.2f} FPS (from mean latency)')
    print('')
    print('Jetson 電力/温度の併記手順:')
    print('  別ターミナルで `sudo tegrastats --interval 1000` を実行しながら')
    print('  このベンチを回し、GR3D_FREQ (GPU 使用率) / 温度 / 電力を記録する。')
    print('  結果報告には nvpmodel のモード (`sudo nvpmodel -q`) と')
    print('  jetson_clocks の有無も併記すること。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
