# sqrtVINS + PGO

Run sqrtVINS on a recorded camera and IMU sequence, then apply offline loop closure with a gravity-aligned four-degree-of-freedom pose graph. The backend estimates yaw and translation corrections. The original VIO trajectory is saved alongside the optimized result.

## Quick start

Build the Docker image from the checkout in a Linux x86-64 shell with Docker available. Keep recordings, calibration files for your rig, and outputs in directories next to the checkout.

```sh
docker build -t sqrtvins-pgo .
docker run --rm sqrtvins-pgo test
mkdir -p ../pgo-data ../pgo-output
data_dir="$(cd ../pgo-data && pwd)"
output_dir="$(cd ../pgo-output && pwd)"
```

Use a new destination under `/out` for each run. On Linux, add `--user "$(id -u):$(id -g)" -e HOME=/tmp` before the image name if you want output files owned by your user.

### EuRoC ROS 1 bag

Get the **V2_01_easy** ROS 1 bag from the [EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/euroc-mav/) and place it at `../pgo-data/V2_01_easy.bag`. The [dataset mirror](https://huggingface.co/datasets/GlowBond/EuRoC_MAV_Dataset/blob/main/vicon_room2.zip) contains the bag at `vicon_room2/V2_01_easy/V2_01_easy.bag`. The Docker image includes the EuRoC stereo calibration.

```sh
docker run --rm --network none -v "$data_dir:/data:ro" sqrtvins-pgo \
  check --bag /data/V2_01_easy.bag
docker run --rm --network none \
  -v "$data_dir:/data:ro" -v "$output_dir:/out" sqrtvins-pgo \
  run --bag /data/V2_01_easy.bag --output /out/V2_01_easy
```

Find the trajectories and logs in `../pgo-output/V2_01_easy/`. Use the same commands with a different bag filename and output name for other EuRoC sequences.

### Visio device ROS 1 bag

Place the Visio ROS 1 bag at `../pgo-data/ego.bag`. This example uses raw stereo images on `/cam0/image_raw` and `/cam1/image_raw`, and IMU samples on `/imu0`. If your bag has a different filename, use that filename in both commands below.

Put your device's `estimator_config.yaml` and its camera and IMU calibration YAMLs in `../pgo-data/rig/`. In the estimator file, point `relative_config_imucam` and `relative_config_imu` to those two files. The calibration must match the device and recorded image resolution. Set the `rostopic` entries to the three topic names above and follow the [configuration requirements](#use-your-own-recording). The [EuRoC example](examples/euroc) shows how the three files refer to one another.

```sh
docker run --rm --network none -v "$data_dir:/data:ro" sqrtvins-pgo \
  check --bag /data/ego.bag --config /data/rig/estimator_config.yaml --cameras 2
docker run --rm --network none \
  -v "$data_dir:/data:ro" -v "$output_dir:/out" sqrtvins-pgo \
  run --bag /data/ego.bag --config /data/rig/estimator_config.yaml --cameras 2 \
  --output /out/ego
```

Find the results in `../pgo-output/ego/`. For either example, `check` lists the configured camera models and the message counts for the selected topics before you run VIO.

## Use your own recording

The runner expects a ROS 1 bag containing raw `sensor_msgs/Image` messages and one `sensor_msgs/Imu` topic. Put the estimator, camera, and IMU YAML files together in a mounted directory. Set the topic names and sensor calibrations in the referenced YAML files, and set `calib_cam_extrinsics` and `calib_cam_intrinsics` to `false`. The camera-to-IMU time offset may still be estimated.

Pass `--config /data/rig/estimator_config.yaml` to select that configuration. Use `--cameras 2` for stereo, the default. The monocular input path (`--cameras 1`) was exercised on a 30-second EuRoC sample with zero accepted loop edges. The loop detector uses `cam0`. Both `pinhole` + `equidistant` and `pinhole` + `radtan` calibration are recognized. The stereo reader pairs frames less than 20 ms apart in bag record time.

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

To repeat only PGO, use `replay --packets /out/V2_01_easy/packets --output /out/replay` with the same output mount. Add `--no-loop` to produce a control trajectory with loop detection disabled.

## Source and licensing

The checkout contains the exact upstream source subsets and BRIEF assets used for the build. See [third-party notices](THIRD_PARTY.md) for revisions and licensing. The PGO integration follows the [GPLv3 license](LICENSE).
