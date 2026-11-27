#!/usr/bin/env bash
# setup_and_build_phase1.sh (Phase 2: Firmware Embed + MCU/MAC/BBP/RF5390 Init + IORegistry Telemetry)
# Runs on macOS GitHub Actions runner (x86_64) with Xcode installed.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORK_DIR="${1:-$(pwd)/ralwm_workspace}"

echo "=== [Step 0] Creating workspace at: $WORK_DIR ==="
mkdir -p "$WORK_DIR"
cd "$WORK_DIR"

# A. Base Framework & UI Client (Shallow Clone)
if [ ! -d "itlwm" ]; then
    git clone --depth 1 --recurse-submodules https://github.com/OpenIntelWireless/itlwm.git
fi
if [ ! -d "itlwm/MacKernelSDK/Headers" ]; then
    echo "=== [Step 0] Fetching Acidanthera MacKernelSDK ==="
    rm -rf itlwm/MacKernelSDK
    git clone --depth 1 https://github.com/acidanthera/MacKernelSDK.git itlwm/MacKernelSDK
fi
if [ ! -d "HeliPort" ]; then
    git clone --depth 1 https://github.com/OpenIntelWireless/HeliPort.git
fi

# B. Upstream BSD, Linux, Vendor & Firmware Fetches
mkdir -p refs/openbsd refs/netbsd refs/linux refs/vendor refs/fw

fetch_file() {
    local url="$1"
    local out="$2"
    local fallback="${3:-}"
    if ! curl -fsSL "$url" -o "$out"; then
        if [ -n "$fallback" ]; then
            echo "Primary failed ($url), trying fallback..."
            curl -fsSL "$fallback" -o "$out"
        else
            echo "ERROR: Failed to download $url" >&2
            exit 1
        fi
    fi
}

echo "=== [Step 0] Fetching upstream reference sources & 8192-byte rt2860.bin ==="
fetch_file \
    https://gitlab.com/kernel-firmware/linux-firmware/-/raw/main/rt2860.bin \
    refs/fw/rt2860.bin \
    https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/rt2860.bin

FW_SIZE=$(wc -c < refs/fw/rt2860.bin | tr -d ' ')
if [ "$FW_SIZE" != "8192" ]; then
    echo "ERROR: refs/fw/rt2860.bin size is $FW_SIZE bytes (MUST be 8192)!" >&2
    exit 1
fi
echo "PASS: refs/fw/rt2860.bin is exactly 8192 bytes."

echo "=== [Phase 2] Copying hal_ral & generating embedded rt2860_fw.h ==="
mkdir -p "$WORK_DIR/itlwm/itlwm/hal_ral"
cp -v "$SCRIPT_DIR/hal_ral/rt5390reg.h" "$WORK_DIR/itlwm/itlwm/hal_ral/"
cp -v "$SCRIPT_DIR/hal_ral/ItlRal.hpp"  "$WORK_DIR/itlwm/itlwm/hal_ral/"
cp -v "$SCRIPT_DIR/hal_ral/ItlRal.cpp"  "$WORK_DIR/itlwm/itlwm/hal_ral/"

python3 - "$WORK_DIR/refs/fw/rt2860.bin" "$WORK_DIR/itlwm/itlwm/hal_ral/rt2860_fw.h" <<'FWPY'
import sys
fw_in, hdr_out = sys.argv[1], sys.argv[2]
data = open(fw_in, "rb").read()
assert len(data) == 8192, f"Expected 8192 bytes, got {len(data)}"
lines = [
    "#ifndef RAL_RT2860_FW_H",
    "#define RAL_RT2860_FW_H",
    "#include <stdint.h>",
    "static const uint32_t rt2860_fw_len = 8192U;",
    "static const uint8_t rt2860_fw_data[8192] = {",
]
for i in range(0, len(data), 16):
    chunk = ", ".join(f"0x{b:02x}" for b in data[i:i+16])
    lines.append(f"    {chunk},")
lines.append("};")
lines.append("#endif /* RAL_RT2860_FW_H */")
open(hdr_out, "w", encoding="utf-8").write("\n".join(lines) + "\n")
print(f"PASS: Generated {hdr_out} ({len(data)} bytes embedded).")
FWPY

echo "=== [Phase 2] Patching Info.plist, itlwm.cpp, and itlwm.xcodeproj ==="
python3 - "$WORK_DIR/itlwm" <<'PYEOF'
import copy, os, re, sys, plistlib

itlwm_root = sys.argv[1]

# 1. Patch itlwm/Info.plist
plist_path = os.path.join(itlwm_root, "itlwm", "Info.plist")
with open(plist_path, "rb") as f:
    pl = plistlib.load(f)

pers = pl.setdefault("IOKitPersonalities", {})
base_pers = None
for key, val in pers.items():
    if key != "ralwm" and isinstance(val, dict) and ("IOPCIPrimaryMatch" in val or "IOPCIMatch" in val):
        base_pers = val
        break

ral_pers = copy.deepcopy(base_pers) if base_pers else {}
ral_pers.pop("IOPCIPrimaryMatch", None)
ral_pers.pop("IOPCISecondaryMatch", None)
ral_pers["CFBundleIdentifier"] = ral_pers.get("CFBundleIdentifier", "com.zxystd.itlwm")
ral_pers["IOClass"] = "itlwm"
ral_pers["IOProviderClass"] = "IOPCIDevice"
ral_pers["IOPCIMatch"] = "0x539f1814 0x53901814"
ral_pers["IOProbeScore"] = 2000
pers["ralwm"] = ral_pers

with open(plist_path, "wb") as f:
    plistlib.dump(pl, f)
print("PASS: Patched IOKitPersonalities['ralwm'].")

# 2. Patch itlwm/itlwm.cpp & itlwm/itlwm.hpp
itlwm_cpp = os.path.join(itlwm_root, "itlwm", "itlwm.cpp")
with open(itlwm_cpp, "r", encoding="utf-8") as f:
    src = f.read()

itlwm_hpp = os.path.join(itlwm_root, "itlwm", "itlwm.hpp")
if "ItlRal.hpp" not in src:
    inc_m = re.search(r'^(#include\s*["<](?:[^">]*/)?ItlIwn\.hpp[">])', src, re.MULTILINE)
    if inc_m:
        inc_line = inc_m.group(1)
        src = src.replace(inc_line, inc_line + '\n#include "hal_ral/ItlRal.hpp"', 1)
    else:
        inc_main = re.search(r'^(#include\s*"itlwm\.hpp")', src, re.MULTILINE)
        if inc_main:
            src = src.replace(inc_main.group(1), inc_main.group(1) + '\n#include "hal_ral/ItlRal.hpp"', 1)
        else:
            src = '#include "hal_ral/ItlRal.hpp"\n' + src

if os.path.exists(itlwm_hpp):
    with open(itlwm_hpp, "r", encoding="utf-8") as hf:
        hsrc = hf.read()
    if "ItlRal.hpp" not in hsrc:
        h_m = re.search(r'^(#include\s*["<](?:[^">]*/)?ItlIwn\.hpp[">])', hsrc, re.MULTILINE)
        if h_m:
            hsrc = hsrc.replace(h_m.group(1), h_m.group(1) + '\n#include "hal_ral/ItlRal.hpp"', 1)
            with open(itlwm_hpp, "w", encoding="utf-8") as hf:
                hf.write(hsrc)

if "ItlRal::ral_match" not in src:
    block_re = re.compile(
        r'([ \t]*)(if\s*\([^\n;]*?ItlIwn::iwn_match\s*\(\s*(\w+)\s*\)[^\n;]*?\)\s*\{[^}]*\})',
        re.DOTALL
    )
    m = block_re.search(src)
    if not m:
        raise SystemExit("ERROR: Could not locate ItlIwn::iwn_match block in itlwm.cpp")
    indent, iwn_block = m.group(1), m.group(2)
    ral_block = iwn_block.replace("ItlIwn::iwn_match", "ItlRal::ral_match").replace("ItlIwn", "ItlRal")
    src = src[:m.start()] + f"{indent}{ral_block}\n{indent}{iwn_block}" + src[m.end():]

if "OSDynamicCast(ItlRal, fHalService)" not in src:
    msi_re = re.compile(
        r'([ \t]*if\s*\()([^\n;]*(?:msi|MSI|kIOInterruptTypePCIMessaged)[^\n;]*)(\)\s*(?:\{\s*return\s+NULL\s*;|\s*return\s+NULL\s*;))'
    )
    msi_m = msi_re.search(src)
    if msi_m:
        guarded = f"{msi_m.group(1)}!OSDynamicCast(ItlRal, fHalService) && ({msi_m.group(2)}){msi_m.group(3)}"
        src = src[:msi_m.start()] + guarded + src[msi_m.end():]

with open(itlwm_cpp, "w", encoding="utf-8") as f:
    f.write(src)

# 3. Patch itlwm.xcodeproj/project.pbxproj
pbx_path = os.path.join(itlwm_root, "itlwm.xcodeproj", "project.pbxproj")
with open(pbx_path, "r", encoding="utf-8") as f:
    pbx = f.read()

if "ItlRal.cpp" not in pbx:
    ID_CPP_REF   = "AA539F010000000000000001"
    ID_HPP_REF   = "AA539F010000000000000002"
    ID_REG_REF   = "AA539F010000000000000003"
    ID_CPP_BUILD = "AA539F010000000000000004"

    file_ref_anchor = "/* End PBXFileReference section */"
    file_refs = (
        f'\t\t{ID_CPP_REF} /* ItlRal.cpp */ = {{isa = PBXFileReference; lastKnownFileType = sourcecode.cpp.cpp; name = ItlRal.cpp; path = itlwm/hal_ral/ItlRal.cpp; sourceTree = SOURCE_ROOT; }};\n'
        f'\t\t{ID_HPP_REF} /* ItlRal.hpp */ = {{isa = PBXFileReference; lastKnownFileType = sourcecode.cpp.h; name = ItlRal.hpp; path = itlwm/hal_ral/ItlRal.hpp; sourceTree = SOURCE_ROOT; }};\n'
        f'\t\t{ID_REG_REF} /* rt5390reg.h */ = {{isa = PBXFileReference; lastKnownFileType = sourcecode.c.h; name = rt5390reg.h; path = itlwm/hal_ral/rt5390reg.h; sourceTree = SOURCE_ROOT; }};\n'
    )
    pbx = pbx.replace(file_ref_anchor, file_refs + file_ref_anchor, 1)

    build_file_anchor = "/* End PBXBuildFile section */"
    build_entry = f'\t\t{ID_CPP_BUILD} /* ItlRal.cpp in Sources */ = {{isa = PBXBuildFile; fileRef = {ID_CPP_REF} /* ItlRal.cpp */; }};\n'
    pbx = pbx.replace(build_file_anchor, build_entry + build_file_anchor, 1)

    group_re = re.compile(r'([0-9A-Fa-f]{24}\s*/\*\s*itlwm\.cpp\s*\*/,)')
    group_entries = (
        f'\n\t\t\t\t{ID_CPP_REF} /* ItlRal.cpp */,'
        f'\n\t\t\t\t{ID_HPP_REF} /* ItlRal.hpp */,'
        f'\n\t\t\t\t{ID_REG_REF} /* rt5390reg.h */,'
    )
    pbx, g_count = group_re.subn(r'\1' + group_entries, pbx, count=1)
    if g_count == 0:
        raise SystemExit("ERROR: Could not locate main itlwm PBXGroup in project.pbxproj")

    pbx, s_count = re.subn(
        r'([0-9A-Fa-f]{24}\s*/\*\s*ItlIwn\.cpp in Sources\s*\*/,)',
        r'\1\n\t\t\t\t' + ID_CPP_BUILD + ' /* ItlRal.cpp in Sources */,',
        pbx,
        count=1
    )
    if s_count == 0:
        raise SystemExit("ERROR: Could not locate ItlIwn.cpp in Sources in project.pbxproj")

    with open(pbx_path, "w", encoding="utf-8") as f:
        f.write(pbx)
    print("PASS: Patched project.pbxproj.")
PYEOF

echo "=== [Phase 2] Generating firmware headers (itlwm build prerequisite) ==="
cd "$WORK_DIR/itlwm"
if [ -x "./scripts/fw_gen.sh" ]; then
    PROJECT_DIR="$WORK_DIR/itlwm" ./scripts/fw_gen.sh || true
elif [ -x "./fw_gen.sh" ]; then
    PROJECT_DIR="$WORK_DIR/itlwm" ./fw_gen.sh || true
fi

echo "=== [Phase 2] Building itlwm.kext (with hal_ral Phase 2) for macOS 12 Monterey (x86_64) ==="
xcodebuild \
    -project itlwm.xcodeproj \
    -target itlwm \
    -configuration Debug \
    ARCHS=x86_64 \
    ONLY_ACTIVE_ARCH=NO \
    MACOSX_DEPLOYMENT_TARGET=12.0 \
    CODE_SIGN_IDENTITY="" \
    CODE_SIGNING_REQUIRED=NO \
    -quiet \
    build

echo "=== Phase 2 Build Finished ==="
echo "Output kext: $WORK_DIR/itlwm/build/Debug/itlwm.kext"
