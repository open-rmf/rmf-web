#!/usr/bin/env python3

import argparse
import importlib
import os
import os.path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--target",
        help="target language",
        required=True,
    )
    parser.add_argument(
        "-o",
        "--outdir",
        help="Output directory",
        required=True,
    )
    parser.add_argument(
        "pkgs",
        help="ROS2 packages to generate files from",
        nargs="+",
    )
    args = parser.parse_args()
    target = importlib.import_module(f"ros_translator.{args.target}")
    os.makedirs(args.outdir, exist_ok=True)
    target.generate(args.pkgs, args.outdir)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
