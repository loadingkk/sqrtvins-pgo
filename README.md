# sqrtVINS + PGO

Run sqrtVINS on a recorded camera and IMU sequence, then apply offline loop closure with a gravity-aligned four-degree-of-freedom pose graph. The backend estimates yaw and translation corrections. The original VIO trajectory is saved alongside the optimized result.

## Quick start

Build the Docker image from this directory. The release was tested on Linux x86-64.

```sh
docker build -t sqrtvins-pgo .
docker run --rm sqrtvins-pgo test
```

Download the ROS bag for **V1_01_easy** from the [EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/euroc-mav/). Place it at `data/V1_01_easy.bag`. The image includes the corresponding fixed-camera calibration.

```sh
mkdir -p data out
docker run --rm --network none \
  -v "$PWD/data:/data:ro" -v "$PWD/out:/out" sqrtvins-pgo \
  run --bag /data/V1_01_easy.bag --output /out/V1_01_easy
```

Each run uses a new output directory. On Linux, add `--user "$(id -u):$(id -g)" -e HOME=/tmp` before the image name to write files as your own user.

Use `check --bag /data/V1_01_easy.bag` with the same input mount to list the configured camera models and message counts without running VIO. For your own rig, pass the same `--config` and `--cameras` options you plan to use with `run`.

## Use your own recording

The runner expects a ROS 1 bag containing raw `sensor_msgs/Image` messages and one `sensor_msgs/Imu` topic. Put the estimator, camera, and IMU YAML files together in a mounted directory. Set the topic names and sensor calibrations in the referenced YAML files, and set `calib_cam_extrinsics` and `calib_cam_intrinsics` to `false`. The camera-to-IMU time offset may still be estimated.

Pass `--config /data/rig/estimator_config.yaml` to select that configuration. Use `--cameras 2` for stereo, the default. The monocular input path (`--cameras 1`) was exercised on a 30-second EuRoC sample with zero accepted loop edges. The loop detector uses `cam0`. Both `pinhole` + `equidistant` and `pinhole` + `radtan` calibration are recognized. The stereo reader pairs frames less than 20 ms apart in bag record time. Convert other recording formats to a ROS 1 bag with the expected message types before running the pipeline.

The PGO backend reads camera images, metric IMU poses, fixed spatial calibration and triangulated features from the exported keyframes. Raw IMU samples are handled by sqrtVINS. Successful PGO correction depends on visually recognizable revisits and usable VIO geometry. See `summary.json` for the accepted loop edges.

## Outputs

| File | Contents |
|---|---|
| `vio_full.txt` | Full-rate sqrtVINS pose recorder output for the bag run |
| `vio_keyframes.tum` | Original IMU poses on the graph keyframes |
| `pgo_keyframes.tum` | Optimized IMU poses on the same keyframes |
| `summary.json` | Keyframe count and accepted loop targets |
| `packets/`, `graph/` | Exported keyframes and backend diagnostics |
| `sqrtvins.log`, `pgo.log` | Logs for the two stages |

The two TUM files contain `time x y z qx qy qz qw`. Timestamps are seconds on the IMU clock, XYZ is in meters in the VIO world frame, and the quaternion rotates IMU coordinates into that frame. Loop indices in the summary are zero-based. A successful run may report zero loop edges.

To repeat only PGO, use `replay --packets /out/V1_01_easy/packets --output /out/replay` with the same mount. Add `--no-loop` to produce a control trajectory with loop detection disabled.

## Source and licensing

The checkout contains the exact upstream source subsets and BRIEF assets used for the build. See [third-party notices](THIRD_PARTY.md) for revisions and licensing. The PGO integration follows the [GPLv3 license](LICENSE).
