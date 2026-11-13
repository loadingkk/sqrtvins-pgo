"""Run the offline sqrtVINS exporter and gravity-aligned PGO."""

import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml


PROJECT = Path(__file__).resolve().parents[1]
REPLAY = "/opt/ws/devel/lib/loop_fusion/packet_replay"


def read_config(path):
    lines = path.read_text(encoding="utf-8").splitlines()
    values = yaml.safe_load("\n".join(line for line in lines if not line.startswith("%YAML:")))
    if not isinstance(values, dict):
        raise ValueError(f"Expected a YAML mapping in {path}.")
    return values


def inspect_inputs(bag, config, cameras):
    import rosbag

    estimator = read_config(config)
    if estimator.get("calib_cam_extrinsics") not in (False, 0) or estimator.get("calib_cam_intrinsics") not in (False, 0):
        raise ValueError("Set calib_cam_extrinsics and calib_cam_intrinsics to false for PGO.")
    cam_chain = read_config(config.parent / estimator["relative_config_imucam"])
    imu_chain = read_config(config.parent / estimator["relative_config_imu"])
    expected = [(imu_chain["imu0"]["rostopic"], "sensor_msgs/Imu")]
    camera_models = {}
    for index in range(cameras):
        name = f"cam{index}"
        camera = cam_chain[name]
        if camera["camera_model"] != "pinhole" or camera["distortion_model"] not in {"radtan", "equidistant"}:
            raise ValueError(f"{name} requires pinhole-radtan or pinhole-equidistant calibration.")
        expected.append((camera["rostopic"], "sensor_msgs/Image"))
        camera_models[name] = f"pinhole-{camera['distortion_model']}"
    if len(dict(expected)) != len(expected):
        raise ValueError("Camera and IMU topics must be distinct.")
    with rosbag.Bag(str(bag), "r") as recording:
        topics = recording.get_type_and_topic_info().topics
        sensors = {}
        for topic, datatype in expected:
            info = topics.get(topic)
            if info is None or info.msg_type != datatype or info.message_count == 0:
                raise ValueError(f"Bag must contain messages on {topic} with type {datatype}.")
            sensors[topic] = {"type": datatype, "messages": info.message_count}
    return {"camera_models": camera_models, "loop_camera": "cam0", "topics": sensors}


def packet_count(directory):
    summary_path = directory / "summary.json"
    if summary_path.is_file():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if summary["dropped"] or summary["failed"]:
            raise ValueError(f"Packet export was incomplete: {summary}")
        count = summary["written"]
    else:
        count = len(list(directory.glob("[0-9]*.txt")))
    if count < 1:
        raise ValueError(f"No keyframes in {directory}.")
    for extension in ("txt", "png"):
        actual = {path.name for path in directory.glob(f"[0-9]*.{extension}")}
        expected = {f"{index:06}.{extension}" for index in range(count)}
        if actual != expected:
            missing = ", ".join(sorted(expected - actual)[:3])
            extra = ", ".join(sorted(actual - expected)[:3])
            raise ValueError(f"Incomplete {extension} packets (missing: {missing}; unexpected: {extra}).")
    return count


@contextmanager
def ros_master(output):
    with (output / "roscore.log").open("w") as log:
        master = subprocess.Popen(["roscore"], stdout=log, stderr=subprocess.STDOUT)
        try:
            import rosgraph

            for _ in range(60):
                if master.poll() is not None:
                    raise RuntimeError("roscore exited. See roscore.log.")
                if rosgraph.is_master_online():
                    break
                time.sleep(0.25)
            else:
                raise RuntimeError("roscore did not become ready. See roscore.log.")
            yield
        finally:
            master.terminate()
            try:
                master.wait(timeout=10)
            except subprocess.TimeoutExpired:
                master.kill()
                master.wait()


def run_logged(command, log_path, env=None):
    print(f"Running {log_path.stem}. Log: {log_path}", flush=True)
    with log_path.open("w") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env)
    if result.returncode:
        raise RuntimeError(f"{log_path.stem} exited with status {result.returncode}. See {log_path}.")


def export_packets(args, output):
    packets = output / "packets"
    log_path = output / "sqrtvins.log"
    run_logged(
        [
            "roslaunch", str(PROJECT / "scripts" / "export.launch"),
            f"bag:={args.bag.resolve()}", f"config_path:={args.config.resolve()}",
            f"max_cameras:={args.cameras}", f"use_stereo:={str(args.cameras == 2).lower()}",
            f"bag_start:={args.start}", f"bag_durr:={args.duration}",
            f"path_est:={output / 'vio_full.txt'}", f"path_time:={output / 'timing.txt'}",
        ],
        log_path,
        {**os.environ, "SQRTVINS_PACKET_DIR": str(packets)},
    )
    with log_path.open() as log:
        if any("process has died" in row for row in log):
            raise RuntimeError("A ROS child process exited unexpectedly. See sqrtvins.log.")
    if not (packets / "summary.json").is_file():
        raise ValueError("The exporter did not finish. See sqrtvins.log.")
    full_trajectory = output / "vio_full.txt"
    if not full_trajectory.is_file() or not any(
        row.strip() and not row.lstrip().startswith("#")
        for row in full_trajectory.read_text().splitlines()
    ):
        raise ValueError("sqrtVINS did not record a VIO trajectory. See sqrtvins.log.")
    return packets


def replay(packets, output, count, loops):
    raw = output / "graph"
    raw.mkdir()
    run_logged([REPLAY, str(packets), str(raw), str(int(loops))], output / "pgo.log")
    original, optimized, edges = [], [], []
    for line in (raw / "pose_graph.txt").read_text().splitlines():
        row = line.split()
        if len(row) != 26 or int(row[0]) != len(original):
            raise ValueError("Invalid PGO graph output. See graph/pose_graph.txt.")
        index, stamp = row[:2]
        original.append(" ".join([stamp, *row[2:5], *row[9:12], row[8]]))
        optimized.append(" ".join([stamp, *row[5:8], *row[13:16], row[12]]))
        if int(row[16]) >= 0:
            edges.append({"current": int(index), "historical": int(row[16])})
    if len(original) != count:
        raise ValueError(f"PGO wrote {len(original)} of {count} keyframes.")
    for name, trajectory in (("vio_keyframes.tum", original), ("pgo_keyframes.tum", optimized)):
        if not all(math.isfinite(float(value)) for line in trajectory for value in line.split()):
            raise ValueError(f"Non-finite values in {name}.")
        (output / name).write_text("\n".join(trajectory) + "\n", encoding="ascii")
    (output / "summary.json").write_text(json.dumps({
        "keyframes": count, "loop_detection": loops, "accepted_loops": len(edges), "edges": edges
    }, indent=2) + "\n", encoding="ascii")
    print(f"Saved {count} keyframes and {len(edges)} accepted loop edges to {output}.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run", help="Run sqrtVINS on a ROS 1 bag, then replay the exported packets.")
    check = commands.add_parser("check", help="Inspect bag topics and supported camera models without running VIO.")
    for command in (run, check):
        command.add_argument("--bag", type=Path, required=True)
        command.add_argument("--config", type=Path, default=PROJECT / "examples/euroc/estimator_config.yaml")
        command.add_argument("--cameras", type=int, choices=(1, 2), default=2)
    run.add_argument("--start", type=float, default=0)
    run.add_argument("--duration", type=float, default=-1)
    replay_parser = commands.add_parser("replay", help="Run PGO on previously exported packets.")
    replay_parser.add_argument("--packets", type=Path, required=True)
    for command in (run, replay_parser):
        command.add_argument("--output", type=Path, required=True, help="New directory for this run.")
        command.add_argument("--no-loop", action="store_true", help="Replay odometry with loop detection disabled.")
    args = parser.parse_args()
    try:
        if args.command == "check":
            print(json.dumps(inspect_inputs(args.bag.resolve(), args.config.resolve(), args.cameras), indent=2))
            return 0
        if args.command == "run":
            if not math.isfinite(args.start) or args.start < 0 or not math.isfinite(args.duration) or not (args.duration == -1 or args.duration > 0):
                raise ValueError("Start must be nonnegative and duration must be positive or -1.")
            inspect_inputs(args.bag.resolve(), args.config.resolve(), args.cameras)
        else:
            packets = args.packets.resolve()
            count = packet_count(packets)
        output = args.output.resolve()
        output.mkdir(parents=True, exist_ok=False)
        with ros_master(output):
            if args.command == "run":
                packets = export_packets(args, output)
                count = packet_count(packets)
            replay(packets, output, count, not args.no_loop)
    except Exception as error:
        print(f"sqrtvins-pgo: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
