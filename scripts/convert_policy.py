#!/usr/bin/env python3
"""Export a single-input, single-output TorchScript actor to ONNX."""
# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0
import argparse
import inspect
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path, help="TorchScript .pt policy")
    parser.add_argument("--input-dim", required=True, type=int, help="Includes all historical frames")
    parser.add_argument("--output", type=Path, help="Defaults to the same name with .onnx suffix")
    parser.add_argument("--dynamic-batch", action="store_true")
    args = parser.parse_args()
    if args.input_dim <= 0:
        parser.error("--input-dim must be positive")
    if not args.model.is_file():
        parser.error(f"Model not found: {args.model}")

    # Python packages are needed only for this export tool, never by the C++ runner.
    import torch
    import onnx

    model = torch.jit.load(str(args.model), map_location="cpu").eval()
    output = args.output or args.model.with_suffix(".onnx")
    sample = torch.zeros(1, args.input_dim, dtype=torch.float32)
    with torch.no_grad():
        actions = model(sample)
        if not isinstance(actions, torch.Tensor) or actions.ndim != 2 or actions.shape[0] != 1:
            raise ValueError("Policy must return a single [1, num_actions] tensor")
        options = dict(input_names=["observations"], output_names=["actions"], opset_version=17)
        if args.dynamic_batch:
            options["dynamic_axes"] = {"observations": {0: "batch"}, "actions": {0: "batch"}}
        if "dynamo" in inspect.signature(torch.onnx.export).parameters:
            options["dynamo"] = False  # TorchScript uses the classic exporter.
        torch.onnx.export(model, sample, str(output), **options)
    onnx.checker.check_model(str(output))
    print(f"Exported {output}: [1, {args.input_dim}] -> [1, {actions.shape[1]}]")


if __name__ == "__main__":
    main()
