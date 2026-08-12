#!/usr/bin/env python3
"""Export a YOLO-World ``boat`` detector as the ZED SDK TensorRT contract.

Run this on the Jetson. The output is deliberately limited to one text prompt
(``boat``) and includes NMS, so zed2i_driver can consume its fixed
``[1, N, 6]`` output directly from the ZED GPU image.
"""

import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(description='Export YOLO-World boat TensorRT engine on Jetson.')
    parser.add_argument(
        '--weights', default='src/robot/config/yolo_model/yolov8s-world.pt',
        help='YOLO-World .pt weights (default: repository copy)')
    parser.add_argument(
        '--output', default='src/robot/config/yolo_model/yolov8s-world-boat.engine',
        help='Output TensorRT .engine path')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--device', default='0')
    parser.add_argument('--no-half', dest='half', action='store_false', default=True)
    args = parser.parse_args()

    if not os.path.isfile(args.weights):
        parser.error(f'weights not found: {args.weights}')
    try:
        import torch
        from ultralytics import YOLOWorld
    except ImportError as error:
        parser.error(f'activate the Jetson Ultralytics environment first: {error}')
    if not torch.cuda.is_available():
        parser.error('CUDA is unavailable; export must run on the target Jetson')

    model = YOLOWorld(args.weights)
    model.set_classes(['boat'])
    output = os.path.abspath(args.output)
    exported = model.export(
        format='engine', imgsz=args.imgsz, device=args.device, half=args.half,
        nms=True, dynamic=False)
    exported = os.path.abspath(str(exported))
    if exported != output:
        os.makedirs(os.path.dirname(output), exist_ok=True)
        os.replace(exported, output)
    print(output)
    print('Launch with: enable_vessel_perception:=true vessel_engine_path:=' + output)


if __name__ == '__main__':
    sys.exit(main())
