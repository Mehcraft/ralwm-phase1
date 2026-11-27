# ralwm.kext — Phase 1 (Fixed): Build, PCI Probe & Hardware Identification

Target hardware: **Ralink RT5390F / RT5390R** (`0x1814:0x539F`, HP Subsystem `0x103C:0x1774`) on **macOS 12 Monterey x86_64** (Darwin 21.x).

## Build Instructions (macOS 12 with Xcode)
```bash
unzip ralwm_phase1_fixed.zip
cd ralwm_phase1_fixed
chmod +x setup_and_build_phase1.sh
./setup_and_build_phase1.sh
```

The script prints `git diff itlwm/itlwm.cpp` before compiling so you can inspect the auto-patched `ItlRal::ral_match(...)` probe block.

## Safe Loading & Verification on macOS 12 Real Hardware
Ensure OpenCore has `DisableIoMapper=true`, `SecureBootModel=Disabled`, `amfi=0x80`, and `-v debug=0x100 keepsyms=1` in `boot-args`.

**WARNING:** Never install an untested kext into `/Library/Extensions`.

Option A — Live load with `kmutil` (requires SIP configured via `csr-active-config` to allow untrusted kexts, e.g. `0x67` or `0x03080000` with `KEXT_SIGNING` disabled):
```bash
sudo chown -R root:wheel ralwm_workspace/itlwm/build/Debug/itlwm.kext
sudo kmutil load -p ralwm_workspace/itlwm/build/Debug/itlwm.kext
log show --last 2m --predicate 'eventMessage CONTAINS "ralwm:"'
# or: sudo dmesg | grep -i ralwm
```

Option B — OpenCore EFI injection (recommended if SIP blocks live `kmutil load`):
1. Copy `ralwm_workspace/itlwm/build/Debug/itlwm.kext` into `EFI/OC/Kexts/` (or rename to `ralwm.kext` and update `Kernel -> Add` in `config.plist`).
2. Reboot and collect `sudo dmesg | grep -i ralwm` or `log show --last 2m --predicate 'eventMessage CONTAINS "ralwm:"'`.

## Expected `dmesg` Output (Phase 1 Gate)
```text
ralwm: ral_match: matched Ralink PCI device 1814:539f
ralwm: === Phase 1 Probe & Hardware Identification ===
ralwm: PCI ID=1814:539f Subsys=103c:1774 PCI_Rev=0x00
ralwm: Mapped BAR0: virt=0xffffff... phys=0x... len=0x10000
ralwm: PCIe cap @ 0x50, LNKCTL=0x0043 (ASPM bits=0x3)
ralwm: MAC_CSR0=0x53901502 -> chipset=0x5390 rev=0x1502 (RT5390R)
ralwm: EFUSE_CTRL (0x0580)=0x80000000 -> eFuse PRESENT
ralwm: EEPROM/eFuse raw dump (first 64 bytes / words 0x00..0x1F):
ralwm:   [0x00]: 5390 ...
ralwm: EEPROM_CHIP_ID=0x5390 EEPROM_VERSION=0x....
ralwm: MAC Address: xx:xx:xx:xx:xx:xx
ralwm: NIC_CONF0=0x.... -> 1T1R, RF_TYPE=0x...
ralwm: NIC_CONF1=0x.... -> ANT_DIVERSITY=... (...), HW_RADIO=..., EXT_LNA_2G=..., INT_TX_ALC=...
ralwm: Crypto policy: software WPA2-PSK/CCMP only (nohwcrypt=1)
ralwm: === Phase 1 Attach Complete (ready for hardware dmesg verification) ===
```
