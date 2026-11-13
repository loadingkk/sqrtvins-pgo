"""Compare a replay with a frozen VINS-Fusion-format pose graph."""

import argparse
import math
from pathlib import Path


def load(path):
    return [line.split() for line in path.read_text().splitlines()]


def rotation_error(first, second):
    a, b = [[float(value) for value in q] for q in (first, second)]
    sign = 1 if sum(x*y for x, y in zip(a, b)) >= 0 else -1
    chord = math.sqrt(sum((x - sign*y)**2 for x, y in zip(a, b)))
    return 4 * math.asin(min(1, chord / 2))


def compare(reference, actual):
    if not reference or len(reference) != len(actual):
        raise ValueError(f"Keyframe count differs ({len(reference)} vs {len(actual)}).")
    maximum_position, maximum_rotation, loops = 0, 0, 0
    for old, new in zip(reference, actual):
        if old[0] != new[0] or abs(float(old[1]) - float(new[1])) > 1e-7:
            raise ValueError(f"Keyframe index or timestamp changed at {old[0]}.")
        if old[16] != new[16]:
            raise ValueError(f"Loop target changed at {old[0]} ({old[16]} vs {new[16]}).")
        if int(new[16]) >= 0:
            loops += 1
        maximum_position = max(maximum_position, math.dist(list(map(float, old[5:8])), list(map(float, new[5:8]))))
        maximum_rotation = max(maximum_rotation, rotation_error(old[12:16], new[12:16]))
    return maximum_position, maximum_rotation, loops


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("actual", type=Path)
    parser.add_argument("--position-tolerance", type=float, default=0.001)
    parser.add_argument("--rotation-tolerance", type=float, default=0.001)
    args = parser.parse_args()
    reference = load(args.reference)
    position, rotation, loops = compare(reference, load(args.actual))
    print(f"{len(reference)} keyframes, {loops} identical loop targets, max position delta "
          f"{position:.6g} m, max rotation delta {rotation:.6g} rad")
    if position > args.position_tolerance or rotation > args.rotation_tolerance:
        parser.exit(1, "Replay differs from the frozen trajectory.\n")
