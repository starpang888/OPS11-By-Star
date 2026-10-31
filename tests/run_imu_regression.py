"""Cross-compile and emulate hardware-free G354/Fusion regression cases.

Requires arm-none-eabi-gcc and Python unicorn. No probe or MCU is accessed.
Local dependency option: python -m pip install --target build/imu-test-tools unicorn
"""
import argparse
from pathlib import Path
import shutil
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "build" / "imu-test-tools"))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import (
    UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_ARM_REG_LR,
    UC_ARM_REG_R0, UC_ARM_REG_SP,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=shutil.which("arm-none-eabi-gcc"))
    args = parser.parse_args()
    if not args.compiler:
        raise SystemExit("arm-none-eabi-gcc is required")
    output = ROOT / "build" / "imu-regression"
    output.mkdir(parents=True, exist_ok=True)
    executable = output / "imu-regression.elf"
    sources = [
        "tests/imu_regression.c", "modules/g354/g354.c",
        "modules/attitude/attitude.c", "modules/fusion/vendor/FusionAhrs.c",
        "modules/fusion/vendor/FusionBias.c",
    ]
    command = [args.compiler, "-std=gnu11", "-O2", "-g", "-Wall", "-Wextra", "-Werror",
               "-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard",
               "-DFUSION_USE_NORMAL_SQRT", "-ffunction-sections", "-fdata-sections",
               "-nostartfiles", "--specs=nosys.specs", "--specs=nano.specs",
               "-Wl,--gc-sections", "-Ttests/imu_regression.ld",
               "-Imodules/g354", "-Imodules/attitude", "-Imodules/fusion/vendor",
               *sources, "-lm", "-o", str(executable)]
    subprocess.run(command, cwd=ROOT, check=True)

    # ELF32 little-endian PT_LOAD segments include initialized data. Mapped RAM starts zeroed.
    data = executable.read_bytes()
    assert data[:7] == b"\x7fELF\x01\x01\x01"
    entry, phoff = struct.unpack_from("<II", data, 24)
    phentsize, phnum = struct.unpack_from("<HH", data, 42)
    machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    machine.mem_map(0x00010000, 0x100000)
    machine.mem_map(0x20000000, 0x100000)
    for index in range(phnum):
        kind, offset, address, _, size, _, _, _ = struct.unpack_from("<8I", data, phoff + index * phentsize)
        if kind == 1 and size:
            machine.mem_write(address, data[offset:offset + size])
    machine.reg_write(UC_ARM_REG_C1_C0_2, 0x00F00000)
    machine.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    machine.reg_write(UC_ARM_REG_SP, 0x200FF000)
    stop = 0x0010FFF0
    machine.reg_write(UC_ARM_REG_LR, stop | 1)
    machine.emu_start(entry | 1, stop, timeout=30000000, count=20000000)
    # A timeout/instruction budget must not be interpreted as a completed test return.
    from unicorn.arm_const import UC_ARM_REG_PC
    if machine.reg_read(UC_ARM_REG_PC) != stop:
        raise SystemExit("Regression execution did not complete")
    failures = machine.reg_read(UC_ARM_REG_R0)
    cases = ["burst decoding, signs, units, corrupt packets",
             "initialization, register readback, burst timing, bus failure",
             "stationary tilt, 90-degree yaw, disturbance and overrange recovery"]
    for bit, label in enumerate(cases):
        print(("FAIL: " if failures & (1 << bit) else "PASS: ") + label)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
