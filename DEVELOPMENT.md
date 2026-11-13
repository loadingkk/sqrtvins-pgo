# Delivery checks

1. Freeze a copy of the experiment source and archived outputs with SHA-256 hashes. Work in a separate directory.
2. Combine the two existing camera adapters under one offline runner. Keep the VIO update, loop geometry, acceptance thresholds, graph weights and optimizer from the frozen versions.
3. Build from the pinned public ROS base and the recorded upstream source revisions. Check the geometry test and malformed-input handling.
4. Replay archived EGO1GS equidistant and EuRoC radtan packets. Compare keyframe timestamps, accepted loop targets and optimized poses against their frozen graphs.
5. Run the documented bag-to-trajectory path on EuRoC V1_01_easy. Confirm matching keyframes and usable TUM outputs. Exercise the monocular path separately before claiming that coverage.
6. Verify the backup hashes and review the checkout for local paths, private data and third-party notices.

The first release targets offline, fixed spatial camera calibration with camera 0 used for loop closure. This pass is scoped to adapting inputs and making the implementation usable independently of the experiment directory.

## Local regression cases

| Input | Keyframes | Accepted loop edges | Optimized pose difference from archived v2 |
|---|---:|---:|---|
| EGO1GS 06, equidistant packet replay | 1334 | 52 | 0 m, 0 rad |
| EuRoC V1_01_easy, radtan packet replay | 1391 | 158 | 0 m, 0 rad |
| EuRoC V1_01_easy, full stereo bag | 1391 | 158 | 0 m, 0 rad |
| EuRoC V1_01_easy, first 30 s with cam0 only | 226 | 0 | Monocular input and export smoke test |
| EGO1GS 00018, first 30 s of a stereo ROS 1 bag with its own equidistant calibration | 285 | 35 | Bag-to-trajectory test, zero dropped or failed keyframes |
| EuRoC V1_01_easy, 15 s stereo bag with custom camera and IMU topics | 91 | 0 | Topics selected through calibration YAML |

The three reference comparisons also matched the accepted historical frame for every loop. Run `python3 tests/compare_replay.py REFERENCE/pose_graph.txt OUTPUT/graph/pose_graph.txt` to check a local archived graph. CI runs the 100-case gravity solver test, input validation, and a two-camera-model replay smoke test with loop detection disabled.

The full stereo and 30-second monocular bag runs both exported all keyframes and exited cleanly after the ROS visualizer teardown fix.
