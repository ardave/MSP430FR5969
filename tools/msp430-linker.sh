#!/bin/sh
# Linker wrapper: locate TI's msp430-elf-gcc and exec it with the arguments
# rustc passes. Referenced by `[target.msp430-none-elf] linker` in
# .cargo/config.toml so the repo has no machine-specific toolchain path
# checked in.
#
# Resolution order:
#   1. $MSP430_GCC          — explicit override (full path to msp430-elf-gcc)
#   2. msp430-elf-gcc       — on PATH
#   3. Known install roots  — ~/ti, /Applications/ti, /opt/ti, /usr/local
#
# TI's MSP430-GCC is a free download (no login):
#   https://www.ti.com/tool/MSP430-GCC-OPENSOURCE

if [ -n "$MSP430_GCC" ]; then
    exec "$MSP430_GCC" "$@"
fi

if command -v msp430-elf-gcc >/dev/null 2>&1; then
    exec msp430-elf-gcc "$@"
fi

for candidate in \
    "$HOME"/ti/msp430-gcc-*/bin/msp430-elf-gcc \
    /Applications/ti/msp430-gcc-*/bin/msp430-elf-gcc \
    /opt/ti/msp430-gcc-*/bin/msp430-elf-gcc \
    /usr/local/msp430-gcc-*/bin/msp430-elf-gcc; do
    if [ -x "$candidate" ]; then
        exec "$candidate" "$@"
    fi
done

echo "error: msp430-elf-gcc not found." >&2
echo "Install TI's MSP430-GCC (https://www.ti.com/tool/MSP430-GCC-OPENSOURCE) and either:" >&2
echo "  - add its bin/ directory to PATH, or" >&2
echo "  - set MSP430_GCC=/path/to/msp430-gcc-x.y.z/bin/msp430-elf-gcc" >&2
exit 1
