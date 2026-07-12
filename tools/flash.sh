#!/bin/sh
# Flash an MSP430 ELF to the attached LaunchPad via TI's DSLite debug server.
#
#   usage: tools/flash.sh <elf> [ccxml]
#
# This is the one place that knows where DSLite lives: cargo's `runner`
# (.cargo/config.toml) and the hal_integration_tests host runner both invoke
# it. The target-configuration file defaults to the repo's MSP430FR5969.ccxml,
# resolved relative to this script so it works from any working directory; an
# explicit ccxml may be passed as the second argument (the two-board runner
# uses this to pin a specific eZ-FET probe when two boards are attached).
#
# DSLite resolution order:
#   1. $MSP430_DSLITE       — explicit override (full path to DSLite)
#   2. DSLite               — on PATH
#   3. Known install roots  — Code Composer Studio under /Applications/ti,
#                             ~/ti, /opt/ti
#
# DSLite ships with Code Composer Studio (https://www.ti.com/tool/CCSTUDIO);
# on Apple-silicon Macs it runs under Rosetta.

set -e

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    echo "usage: $0 <elf> [ccxml]" >&2
    exit 2
fi
ELF=$1

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
CCXML=${2:-"$SCRIPT_DIR/../MSP430FR5969.ccxml"}

resolve_dslite() {
    if [ -n "$MSP430_DSLITE" ]; then
        echo "$MSP430_DSLITE"
        return 0
    fi
    if command -v DSLite >/dev/null 2>&1; then
        echo "DSLite"
        return 0
    fi
    for candidate in \
        /Applications/ti/ccs*/ccs/ccs_base/DebugServer/bin/DSLite \
        "$HOME"/ti/ccs*/ccs/ccs_base/DebugServer/bin/DSLite \
        /opt/ti/ccs*/ccs/ccs_base/DebugServer/bin/DSLite; do
        if [ -x "$candidate" ]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

if ! DSLITE=$(resolve_dslite); then
    echo "error: DSLite not found." >&2
    echo "Install Code Composer Studio (https://www.ti.com/tool/CCSTUDIO) and either:" >&2
    echo "  - add DSLite's directory to PATH, or" >&2
    echo "  - set MSP430_DSLITE=/path/to/ccs/ccs_base/DebugServer/bin/DSLite" >&2
    exit 1
fi

exec "$DSLITE" load -c "$CCXML" -f "$ELF"
