"""Package the model, firmware filter and training inputs for a CPU worker."""

import argparse
from pathlib import Path
import sys
import tarfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
SIM = Path("sim/sg90_1000mah")
FIRMWARE = Path("firmware/standing_xiao")
UPSTREAM = Path("upstream/2dof_standing")
UPSTREAM_FILES = (
    "sim/environment.py",
    "sim/model_geometry.py",
    "sim/recovery/evaluate_filter.py",
    "sim/recovery/filter_runtime.py",
    "sim/recovery/search.py",
    "sim/enclosure_c3/upright/strategy.py",
    "firmware/policy_control/RecoveryStrategy.h",
    "firmware/imu_bench/measurements/2026-11-01/mount_rotation_provisional.json",
)
FIRMWARE_FILES = ("GravityFilter.h", "MountCalibration.h", "RecoveryStrategy.h")
SIM_FILES = (
    "configuration.json", "model_report.json", "robot.xml", "requirements-lock.txt",
    "env.py", "build_model.py", "filter_runtime_local.py",
    "gated/fall_env.py", "gated/train_gated.py",
    "results/run_20261111/best_trained.zip",
)


def dataless(path):
    return bool(getattr(path.stat(), "st_flags", 0) & 0x40000000)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    files = [SIM / name for name in SIM_FILES]
    files.extend(FIRMWARE / name for name in FIRMWARE_FILES)
    files.extend(UPSTREAM / name for name in UPSTREAM_FILES)
    xml = ET.parse(ROOT / SIM / "robot.xml")
    files.extend(SIM / "assets" / mesh.attrib["file"] for mesh in xml.findall(".//asset/mesh"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(".partial")
    with tarfile.open(temporary, "w:gz") as tar:
        for archive_name in files:
            source = ROOT / archive_name
            if dataless(source) and archive_name.name in ("build_model.py", "filter_runtime_local.py"):
                # The cached modules come from the previous SG90 run. These
                # two source files may be represented by iCloud placeholders.
                tag = sys.implementation.cache_tag
                source = source.parent / "__pycache__" / f"{source.stem}.{tag}.pyc"
                archive_name = archive_name.with_suffix(".pyc")
                print(f"Using {tag} cached module for {archive_name}")
            if dataless(source):
                raise OSError(f"Download the iCloud file before packaging: {source}")
            tar.add(source, arcname=str(archive_name))
    temporary.replace(args.output)
    print(args.output)


if __name__ == "__main__":
    main()
