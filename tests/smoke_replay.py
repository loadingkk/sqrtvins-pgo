"""Exercise the container runner with one legacy packet from each camera model."""

import json
from pathlib import Path
import struct
import subprocess
import tempfile
import zlib


def chunk(tag, data):
    return (struct.pack(">I", len(data)) + tag + data +
            struct.pack(">I", zlib.crc32(tag + data)))


def grayscale_png():
    size = 64
    header = struct.pack(">IIBBBBB", size, size, 8, 0, 0, 0, 0)
    pixels = zlib.compress(bytes((size + 1) * size))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) +
            chunk(b"IDAT", pixels) + chunk(b"IEND", b""))


def main():
    entrypoint = Path(__file__).resolve().parents[1] / "scripts/entrypoint.sh"
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        for model in (0, 1):
            packets = root / f"model-{model}"
            packets.mkdir()
            for index in range(2):
                stem = f"{index:06}"
                (packets / f"{stem}.txt").write_text(
                    f"{1 + index} 0 3\n{index} 0 0 0 0 0 1\n"
                    f"0 0 0 0 0 0 1\n{model} 64 64 48 48 32 32 0 0 0 0\n",
                    encoding="ascii",
                )
                (packets / f"{stem}.png").write_bytes(grayscale_png())
            output = root / f"output-{model}"
            subprocess.run(["bash", str(entrypoint), "replay", "--packets", str(packets),
                            "--output", str(output), "--no-loop"], check=True)
            summary = json.loads((output / "summary.json").read_text())
            assert summary["accepted_loops"] == 0 and summary["keyframes"] == 2
            assert (output / "vio_keyframes.tum").read_bytes() == (output / "pgo_keyframes.tum").read_bytes()
            (packets / "000001.png").unlink()
            failed = subprocess.run(["bash", str(entrypoint), "replay", "--packets", str(packets),
                                     "--output", str(root / "incomplete")], capture_output=True, text=True)
            assert failed.returncode == 1 and "000001.png" in failed.stderr, failed.stderr


if __name__ == "__main__":
    main()
