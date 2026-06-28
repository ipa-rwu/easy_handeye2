import argparse
from pathlib import Path
from typing import Iterable

import yaml
from easy_handeye2_msgs.msg import HandeyeCalibration, HandeyeCalibrationParameters, SampleList
from rosidl_runtime_py import message_to_yaml, set_message_fields

from easy_handeye2.handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV


class _Logger:
    def info(self, message: str) -> None:
        print(f"[INFO] {message}")

    def warn(self, message: str) -> None:
        print(f"[WARN] {message}")

    def err(self, message: str) -> None:
        print(f"[ERROR] {message}")


class _Node:
    def __init__(self) -> None:
        self._logger = _Logger()

    def get_logger(self) -> _Logger:
        return self._logger


def _load_sample_list(path: Path) -> SampleList:
    with path.open() as handle:
        payload = yaml.full_load(handle.read())
    sample_list = SampleList()
    set_message_fields(sample_list, payload)
    return sample_list


def _load_parameters(path: Path) -> HandeyeCalibrationParameters:
    with path.open() as handle:
        payload = yaml.full_load(handle.read())
    params = HandeyeCalibrationParameters()
    set_message_fields(params, payload.get("parameters", {}))
    return params


def _first_non_empty(values: Iterable[str]) -> str:
    for value in values:
        if value:
            return value
    return ""


def _resolve_parameters(sample_paths: list[Path], args: argparse.Namespace) -> HandeyeCalibrationParameters:
    loaded = [_load_parameters(path) for path in sample_paths]
    params = HandeyeCalibrationParameters()
    params.name = args.name or _first_non_empty(item.name for item in loaded)
    params.calibration_type = args.calibration_type or _first_non_empty(item.calibration_type for item in loaded)
    params.robot_base_frame = args.robot_base_frame or _first_non_empty(item.robot_base_frame for item in loaded)
    params.robot_effector_frame = args.robot_effector_frame or _first_non_empty(
        item.robot_effector_frame for item in loaded
    )
    params.tracking_base_frame = args.tracking_base_frame or _first_non_empty(item.tracking_base_frame for item in loaded)
    params.tracking_marker_frame = args.tracking_marker_frame or _first_non_empty(
        item.tracking_marker_frame for item in loaded
    )
    params.freehand_robot_movement = args.freehand_robot_movement or any(
        item.freehand_robot_movement for item in loaded
    )

    missing = [
        field
        for field in [
            "name",
            "calibration_type",
            "robot_base_frame",
            "robot_effector_frame",
            "tracking_base_frame",
            "tracking_marker_frame",
        ]
        if not getattr(params, field)
    ]
    if missing:
        raise ValueError(
            "Missing calibration parameters: {}. Supply them via the .samples file or CLI arguments.".format(
                ", ".join(missing)
            )
        )
    return params


def _merge_sample_lists(sample_paths: list[Path]) -> SampleList:
    merged = SampleList()
    for path in sample_paths:
        merged.samples.extend(_load_sample_list(path).samples)
    return merged


def _algorithm_slug(name: str) -> str:
    return name.lower().replace("/", "_").replace(" ", "_")


def _write_calibration(calibration: HandeyeCalibration, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w") as handle:
        handle.write(message_to_yaml(calibration))


def _build_output_name(base_name: str, algorithm: str) -> str:
    return f"{base_name}_{_algorithm_slug(algorithm)}"


def _compute(sample_paths: list[Path], args: argparse.Namespace, output_base: str) -> list[Path]:
    params = _resolve_parameters(sample_paths, args)
    params.name = output_base
    samples = _merge_sample_lists(sample_paths)

    backend = HandeyeCalibrationBackendOpenCV()
    node = _Node()

    algorithms = args.algorithms or list(backend.AVAILABLE_ALGORITHMS.keys())
    written: list[Path] = []
    for algorithm in algorithms:
        if algorithm not in backend.AVAILABLE_ALGORITHMS:
            raise ValueError(
                f"Unsupported algorithm '{algorithm}'. Available: {', '.join(backend.AVAILABLE_ALGORITHMS.keys())}"
            )
        calibration = backend.compute_calibration(node, params, samples, algorithm=algorithm)
        if calibration is None:
            raise RuntimeError(f"Calibration failed for algorithm '{algorithm}'")
        calibration.parameters.name = _build_output_name(output_base, algorithm)
        output_path = args.output_dir / f"{calibration.parameters.name}.calib"
        _write_calibration(calibration, output_path)
        written.append(output_path)
    return written


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Process one or more easy_handeye2 .samples files and compute .calib results with multiple algorithms."
    )
    parser.add_argument("sample_files", nargs="+", type=Path, help="Input .samples files")
    parser.add_argument(
        "--merge",
        action="store_true",
        help="Merge all input files into one sample set and compute one calibration set.",
    )
    parser.add_argument(
        "--algorithms",
        nargs="+",
        default=None,
        help="Subset of OpenCV hand-eye algorithms to run. Default: all supported algorithms.",
    )
    parser.add_argument("--output-dir", type=Path, default=Path.cwd(), help="Directory for generated .calib files.")
    parser.add_argument("--name", default="", help="Override calibration name prefix.")
    parser.add_argument("--calibration-type", default="", help="Override calibration_type parameter.")
    parser.add_argument("--robot-base-frame", default="", help="Override robot_base_frame parameter.")
    parser.add_argument("--robot-effector-frame", default="", help="Override robot_effector_frame parameter.")
    parser.add_argument("--tracking-base-frame", default="", help="Override tracking_base_frame parameter.")
    parser.add_argument("--tracking-marker-frame", default="", help="Override tracking_marker_frame parameter.")
    parser.add_argument(
        "--freehand-robot-movement",
        action="store_true",
        help="Set freehand_robot_movement=true in the generated calibration metadata.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sample_paths = [path.resolve() for path in args.sample_files]
    for path in sample_paths:
        if not path.exists():
            raise FileNotFoundError(f"Sample file not found: {path}")

    if args.merge:
        base_name = args.name or sample_paths[0].stem
        written = _compute(sample_paths, args, base_name)
    else:
        written = []
        for sample_path in sample_paths:
            base_name = args.name or sample_path.stem
            written.extend(_compute([sample_path], args, base_name))

    for path in written:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
