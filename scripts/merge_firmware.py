Import("env")

import subprocess
from pathlib import Path


def merge_firmware(source, target, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    project_dir = Path(env.subst("$PROJECT_DIR"))
    output = build_dir / "firmware-merged.bin"

    app_bin = Path(str(target[0])) if target else build_dir / "firmware.bin"
    partition_bin = build_dir / "partitions.bin"
    if not partition_bin.exists():
        partition_bin = build_dir / "partition_table" / "partition-table.bin"

    parts = [
        ("0x0", build_dir / "bootloader.bin"),
        ("0x8000", partition_bin),
        ("0x10000", app_bin),
    ]

    missing = [str(path) for _, path in parts if not path.exists()]
    if missing:
        print("Skipping merged firmware; missing: " + ", ".join(missing))
        return

    cmd = [
        env.subst("$PYTHONEXE"),
        env.subst("$UPLOADER"),
        "--chip",
        "esp32s3",
        "merge_bin",
        "--output",
        str(output),
        "--flash_mode",
        "dio",
        "--flash_size",
        "2MB",
        "--flash_freq",
        "80m",
    ]
    for offset, path in parts:
        cmd.extend([offset, str(path)])

    print("Merging firmware image: " + str(output))
    subprocess.check_call(cmd, cwd=str(project_dir))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", merge_firmware)
