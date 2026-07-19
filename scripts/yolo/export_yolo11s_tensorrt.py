#!/usr/bin/env python3
"""Export YOLO11s weights (.pt) to a TensorRT engine (.engine) on Jetson.

★このスクリプトは Jetson 上でのみ実行すること。Mac では実行禁止★
TensorRT engine はビルドしたデバイス (GPU / TensorRT バージョン) 専用の
バイナリになるため、実機の Jetson 上で export する必要がある。
Mac には TensorRT が存在しないので、そもそも動かない。

使い方 (Jetson の YOLO venv を activate した状態で):

    python3 scripts/yolo/export_yolo11s_tensorrt.py \
        --weights /path/to/yolo11s.pt \
        --imgsz 640 \
        --output-dir /path/to/out

FP16 (デフォルト有効) を無効にする場合は --no-half。
INT8 を使う場合は --int8 と --calib-data (キャリブレーション用データ) が必須。

実行時に torch / ultralytics / tensorrt / CUDA / JetPack のバージョンを
export_env.json として出力先に記録する (engine の再現性確認用)。
"""

import argparse
import json
import os
import shutil
import sys
from datetime import datetime, timezone


def read_jetpack_release():
    """/etc/nv_tegra_release があれば JetPack / L4T 情報を返す。"""
    path = '/etc/nv_tegra_release'
    if not os.path.exists(path):
        return None
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            return f.read().strip()
    except OSError:
        return None


def collect_environment():
    env = {
        'exported_at': datetime.now(timezone.utc).isoformat(),
        'python': sys.version,
        'platform': sys.platform,
        'nv_tegra_release': read_jetpack_release(),
    }

    try:
        import torch
        env['torch'] = torch.__version__
        env['cuda'] = torch.version.cuda
        env['cuda_available'] = torch.cuda.is_available()
        if torch.cuda.is_available():
            env['cuda_device_name'] = torch.cuda.get_device_name(0)
    except Exception as exc:  # noqa: BLE001
        env['torch'] = f'unavailable: {exc}'

    try:
        import ultralytics
        env['ultralytics'] = ultralytics.__version__
    except Exception as exc:  # noqa: BLE001
        env['ultralytics'] = f'unavailable: {exc}'

    try:
        import tensorrt
        env['tensorrt'] = tensorrt.__version__
    except Exception as exc:  # noqa: BLE001
        env['tensorrt'] = f'unavailable: {exc}'

    return env


def parse_args():
    parser = argparse.ArgumentParser(
        description='Export YOLO11s .pt weights to TensorRT .engine (Jetson only).')
    parser.add_argument('--weights', required=True,
                        help='Path to trained YOLO11s .pt weights (required)')
    parser.add_argument('--imgsz', type=int, default=640,
                        help='Inference image size baked into the engine (default: 640)')
    parser.add_argument('--half', dest='half', action='store_true', default=True,
                        help='Build FP16 engine (default: enabled)')
    parser.add_argument('--no-half', dest='half', action='store_false',
                        help='Disable FP16 (build FP32 engine)')
    parser.add_argument('--int8', action='store_true', default=False,
                        help='Build INT8 engine. Requires --calib-data.')
    parser.add_argument('--calib-data', default=None,
                        help='Dataset YAML for INT8 calibration (required with --int8)')
    parser.add_argument('--workspace', type=float, default=4.0,
                        help='TensorRT builder workspace size in GB (default: 4.0)')
    parser.add_argument('--output-dir', default='.',
                        help='Directory to place the .engine and export_env.json '
                             '(default: current directory)')
    return parser.parse_args()


def main():
    args = parse_args()

    if sys.platform == 'darwin':
        print('ERROR: This script must be run on Jetson, not on macOS. '
              'TensorRT engines are device-specific.', file=sys.stderr)
        return 1

    if args.int8 and not args.calib_data:
        print('ERROR: --int8 requires --calib-data (calibration dataset YAML). '
              'Without calibration data the INT8 engine accuracy cannot be '
              'guaranteed, so the export is refused.', file=sys.stderr)
        return 1

    if not os.path.exists(args.weights):
        print(f'ERROR: weights not found: {args.weights}', file=sys.stderr)
        return 1

    if args.calib_data and not os.path.exists(args.calib_data):
        print(f'ERROR: calib data not found: {args.calib_data}', file=sys.stderr)
        return 1

    os.makedirs(args.output_dir, exist_ok=True)

    env = collect_environment()
    env['args'] = {
        'weights': os.path.abspath(args.weights),
        'imgsz': args.imgsz,
        'half': args.half,
        'int8': args.int8,
        'calib_data': args.calib_data,
        'workspace_gb': args.workspace,
    }
    env_path = os.path.join(args.output_dir, 'export_env.json')
    with open(env_path, 'w', encoding='utf-8') as f:
        json.dump(env, f, indent=2, ensure_ascii=False)
    print(f'Environment recorded to {env_path}')
    print(json.dumps(env, indent=2, ensure_ascii=False))

    try:
        from ultralytics import YOLO
    except Exception as exc:  # noqa: BLE001
        print(f'ERROR: failed to import ultralytics: {exc}. '
              'Activate the Jetson YOLO venv first '
              '(see src/detection/yolo/README_jetson_yolo11.md).', file=sys.stderr)
        return 1

    model = YOLO(args.weights)

    export_kwargs = {
        'format': 'engine',
        'imgsz': args.imgsz,
        'half': args.half and not args.int8,
        'int8': args.int8,
        'workspace': args.workspace,
        'device': 0,
    }
    if args.int8:
        export_kwargs['data'] = args.calib_data

    print(f'Exporting with: {export_kwargs}')
    engine_path = model.export(**export_kwargs)
    print(f'Export finished: {engine_path}')

    # export は weights と同じディレクトリに .engine を作るので、output-dir へ移す
    engine_path = str(engine_path)
    if os.path.exists(engine_path):
        dest = os.path.join(args.output_dir, os.path.basename(engine_path))
        if os.path.abspath(dest) != os.path.abspath(engine_path):
            shutil.move(engine_path, dest)
            engine_path = dest
    print(f'TensorRT engine: {engine_path}')
    print('NOTE: engines are specific to this Jetson (GPU / TensorRT version). '
          'Re-export after JetPack or TensorRT upgrades.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
