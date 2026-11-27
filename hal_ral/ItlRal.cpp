/*
 * ItlRal.cpp — Ralink RT5390F/RT5390R (0x1814:0x539F) HAL backend for itlwm
 *
 * Phase 1 (Verified): PCI Probe, BAR0 MMIO, ASPM Guard, MAC_CSR0 (0x53900502 RT5390F), eFuse/EEPROM.
 * Phase 2 (Verified): 8192-byte rt2860.bin MCU upload (1ms), MAC/BBP (R0=0x67)/RF5390 init, BBP R152.
 * Phase 3: 32-bit physical-masked DMA RX/TX rings, Interrupts (MSI/Legacy), RT5390 channel tuning
 *          (ch 1..13), 802.11 RX -> net80211/HeliPort scanning, MGMT/ProbeReq TX, and live IORegistry telemetry.
 */

#include "ItlRal.hpp"
#include "rt2860_fw.h"

#include <IOKit/IOLib.h>
#include <IOKit/network/IONetworkController.h>
#include <IOKit/network/IONetworkMedium.h>
#include <libkern/OSByteOrder.h>
#include <net/ethernet.h>

#define super ItlHalService
OSDefineMetaClassAndStructors(ItlRal, ItlHalService)

#define RAL_LOG(fmt, ...) XYLog("ralwm: " fmt "\n", ##__VA_ARGS__)

/* Static singleton pointer for net80211 C callbacks (matches ItlIwn idiom) */
static ItlRal *g_ral_instance = NULL;

/* -------------------------------------------------------------------------- */
/* Hardware Tables (OpenBSD rt2860reg.h & Linux rt2800lib.c)                  */
/* -------------------------------------------------------------------------- */

/* Verified exact MMIO offsets from Linux drivers/net/wireless/ralink/rt2x00/rt2800.h */
static const struct { uint16_t reg; uint32_t val; } ral_def_mac[] = {
    { RAL_WMM_AIFSN_CFG,    0x00002273U }, /* 0x0214 */
    { RAL_WMM_CWMIN_CFG,    0x00002344U }, /* 0x0218 */
    { RAL_WMM_CWMAX_CFG,    0x000034aaU }, /* 0x021C */
    { RAL_PBF_MAX_PCNT,     0x1f3fbf9fU }, /* 0x040C */
    { RAL_BCN_OFFSET0,      0xf8f0e8e0U }, /* 0x042C */
    { RAL_BCN_OFFSET1,      0x6f77d0c8U }, /* 0x0430 */
    { RAL_MAC_SYS_CTRL,     0x00000000U }, /* 0x1004 */
    { RAL_MAX_LEN_CFG,      0x00001fffU }, /* 0x1018 (max MPDU 4095, max PSDU 16K) */
    { RAL_LED_CFG,          0x7f031e46U }, /* 0x102C */
    { RAL_XIFS_TIME_CFG,    0x33a41010U }, /* 0x1100 (CCK/OFDM SIFS=16us, EIFS=314us, BB_RXEND) */
    { RAL_BKOFF_SLOT_CFG,   0x00000209U }, /* 0x1104 (9us short slot, CC_DELAY=2) */
    { RAL_PWR_PIN_CFG,      0x00000003U }, /* 0x1204 */
    { RAL_MIMO_PS_CFG,      0x00000000U }, /* 0x1210 (disable MIMO PS) */
    { RAL_EDCA_AC0_CFG,     0x000a4300U }, /* 0x1300 (AC_BE: AIFSN=3, CWmin=4, CWmax=10) */
    { RAL_EDCA_AC1_CFG,     0x000a4700U }, /* 0x1304 (AC_BK) */
    { RAL_EDCA_AC2_CFG,     0x0004325eU }, /* 0x1308 (AC_VI) */
    { RAL_EDCA_AC3_CFG,     0x0003222fU }, /* 0x130C (AC_VO) */
    { RAL_TX_BAND_CFG,      0x00000004U }, /* 0x132C (2.4 GHz BG, 20 MHz) */
    { RAL_TX_SW_CFG0,       0x00000404U }, /* 0x1330 (RT5390) */
    { RAL_TX_SW_CFG1,       0x00000000U }, /* 0x1334 (RT5390) */
    { RAL_TX_SW_CFG2,       0x00000000U }, /* 0x1338 (RT5390) */
    { RAL_TXOP_CTRL_CFG,    0x0000583fU }, /* 0x1340 */
    { RAL_TX_RTS_CFG,       0x01092b20U }, /* 0x1344 (RTS thres=2347, rty=32, RTS fallback=1) */
    { RAL_TX_TIMEOUT_CFG,   0x000a2090U }, /* 0x1348 (MPDU life=9, RX_ACK_TIMEOUT=0x20 slots) */
    { RAL_TX_RTY_CFG,       0x47d00f07U }, /* 0x134C (TX_AUTO_FB_ENABLE=1, short=7, long=15) */
    { RAL_TX_LINK_CFG,      0x00001020U }, /* 0x1350 (TX_CF_ACK_EN=1, MFB_LIFE=32) */
    { RAL_HT_FBK_CFG0,      0x65432100U }, /* 0x1354 (MCS 7->6->5->4->3->2->1->0) */
    { RAL_HT_FBK_CFG1,      0xedcba988U }, /* 0x1358 (MCS 15..8) */
    { RAL_LG_FBK_CFG0,      0xedcba988U }, /* 0x135C (OFDM 54->48->36->24->18->12->9->6) */
    { RAL_LG_FBK_CFG1,      0x00002100U }, /* 0x1360 (CCK 11->5.5->2->1) */
    { RAL_CCK_PROT_CFG,     0x05740003U }, /* 0x1364 */
    { RAL_OFDM_PROT_CFG,    0x05740003U }, /* 0x1368 */
    { RAL_MM20_PROT_CFG,    0x01744004U }, /* 0x136C */
    { RAL_MM40_PROT_CFG,    0x03f54084U }, /* 0x1370 */
    { RAL_GF20_PROT_CFG,    0x01744004U }, /* 0x1374 */
    { RAL_GF40_PROT_CFG,    0x03f44084U }, /* 0x1378 */
    { RAL_EXP_ACK_TIME,     0x002400caU }, /* 0x1380 */
    { RAL_RX_FILTR_CFG,     RAL_RX_FILTR_SCAN }, /* 0x1400 */
    { RAL_AUTO_RSP_CFG,     0x00000013U }, /* 0x1404 (AUTORESPONDER=1, BAC_ACK=1, SHORT_PRE=1) */
    { RAL_LEGACY_BASIC_RATE,0x0000015fU }, /* 0x1408 (1,2,5.5,11,6,12,24 Mbps basic rates) */
    { RAL_HT_BASIC_RATE,    0x00008003U }, /* 0x140C */
    { RAL_TXOP_HLDR_ET,     0x00000002U }, /* 0x1608 */
};

static const struct { uint8_t reg; uint8_t val; } rt5390_def_bbp[] = {
    {  31, 0x08 }, {  65, 0x2c }, {  66, 0x38 }, {  68, 0x0b },
    {  69, 0x0d }, {  70, 0x06 }, {  73, 0x13 }, {  75, 0x46 },
    {  76, 0x28 }, {  77, 0x59 }, {  81, 0x37 }, {  82, 0x62 },
    {  83, 0x7a }, {  84, 0x9a }, {  86, 0x38 }, {  91, 0x04 },
    {  92, 0x02 }, { 103, 0xc0 }, { 104, 0x92 }, { 105, 0x3c },
    { 106, 0x03 }, { 128, 0x12 },
};

static const struct { uint8_t reg; uint8_t val; } rt5390_def_rf[] = {
    {  1, 0x0f }, {  2, 0x80 }, {  3, 0x88 }, {  5, 0x10 },
    {  6, 0xa0 }, {  7, 0x00 }, { 10, 0x53 }, { 11, 0x4a },
    { 12, 0x46 }, { 13, 0x9f }, { 14, 0x00 }, { 15, 0x00 },
    { 16, 0x00 }, { 18, 0x03 }, { 19, 0x00 }, { 20, 0x00 },
    { 21, 0x00 }, { 22, 0x20 }, { 23, 0x00 }, { 24, 0x00 },
    { 25, 0xc0 }, { 26, 0x00 }, { 27, 0x09 }, { 28, 0x00 },
    { 29, 0x10 }, { 30, 0x10 }, { 31, 0x80 }, { 32, 0x80 },
    { 33, 0x00 }, { 34, 0x07 }, { 35, 0x12 }, { 36, 0x00 },
    { 37, 0x08 }, { 38, 0x85 }, { 39, 0x1b }, { 40, 0x0b },
    { 41, 0xbb }, { 42, 0xd2 }, { 43, 0x9a }, { 44, 0x0e },
    { 45, 0xa2 }, { 46, 0x7b }, { 47, 0x00 }, { 48, 0x10 },
    { 49, 0x94 }, { 52, 0x38 }, { 53, 0x84 }, { 54, 0x78 },
    { 55, 0x44 }, { 56, 0x22 }, { 57, 0x80 }, { 58, 0x7f },
    { 59, 0x8f }, { 60, 0x45 }, { 61, 0xdd }, { 62, 0x00 },
    { 63, 0x00 },
};

/* RT5390 2.4GHz PLL (N, R, K) table for channels 1..14 (20 MHz crystal, OpenBSD rt2860.c) */
static const struct { uint8_t n; uint8_t r; uint8_t k; } rt5390_rf_ch[14] = {
    { 241, 2, 2 }, /* ch 1  (2412 MHz) */
    { 241, 2, 7 }, /* ch 2  (2417 MHz) */
    { 242, 2, 2 }, /* ch 3  (2422 MHz) */
    { 242, 2, 7 }, /* ch 4  (2427 MHz) */
    { 243, 2, 2 }, /* ch 5  (2432 MHz) */
    { 243, 2, 7 }, /* ch 6  (2437 MHz) */
    { 244, 2, 2 }, /* ch 7  (2442 MHz) */
    { 244, 2, 7 }, /* ch 8  (2447 MHz) */
    { 245, 2, 2 }, /* ch 9  (2452 MHz) */
    { 245, 2, 7 }, /* ch 10 (2457 MHz) */
    { 246, 2, 2 }, /* ch 11 (2462 MHz) */
    { 246, 2, 7 }, /* ch 12 (2467 MHz) */
    { 247, 2, 2 }, /* ch 13 (2472 MHz) */
    { 248, 2, 4 }, /* ch 14 (2484 MHz) */
};

/* RT5390F/R (rev >= 0x0502) non-BT per-channel RF R55/R59 tables (Linux rt2800lib.c) */
static const uint8_t rt5390_r55_nonbt_rev[14] = {
    0x23, 0x23, 0x23, 0x23, 0x13, 0x13, 0x03, 0x03,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03
};
static const uint8_t rt5390_r59_nonbt_rev[14] = {
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    0x07, 0x07, 0x06, 0x05, 0x04, 0x04
};

/* -------------------------------------------------------------------------- */
/* PCI Matching & MMIO Accessors                                              */
/* -------------------------------------------------------------------------- */

bool ItlRal::ral_match(IOPCIDevice *device)
{
    if (!device)
        return false;

    uint16_t vendor = device->configRead16(kIOPCIConfigVendorID);
    uint16_t prod   = device->configRead16(kIOPCIConfigDeviceID);

    return (vendor == PCI_VENDOR_RALINK &&
            (prod == PCI_PRODUCT_RALINK_RT539F || prod == PCI_PRODUCT_RALINK_RT5390));
}

uint32_t ItlRal::ral_read32(uint32_t reg)
{
    if (!com.sc_mmio_base || (reg + 4) > com.sc_mmio_len)
        return 0xFFFFFFFFU;
    volatile uint32_t *ptr = (volatile uint32_t *)(com.sc_mmio_base + reg);
    uint32_t val = OSReadLittleInt32(ptr, 0);
    OSSynchronizeIO();
    return val;
}

void ItlRal::ral_write32(uint32_t reg, uint32_t val)
{
    if (!com.sc_mmio_base || (reg + 4) > com.sc_mmio_len)
        return;
    volatile uint32_t *ptr = (volatile uint32_t *)(com.sc_mmio_base + reg);
    OSWriteLittleInt32(ptr, 0, val);
    OSSynchronizeIO();
}

/* -------------------------------------------------------------------------- */
/* Phase 1: PCIe ASPM Guard, MAC_CSR0 & eFuse/EEPROM                          */
/* -------------------------------------------------------------------------- */

uint8_t ItlRal::ral_find_pci_cap(IOPCIDevice *device, uint8_t cap_id)
{
    uint16_t status = device->configRead16(kIOPCIConfigStatus);
    if ((status & 0x0010) == 0)
        return 0;

    uint8_t ptr = device->configRead8(0x34) & 0xFC;
    for (int guard = 0; guard < 48 && ptr != 0; guard++) {
        uint8_t id   = device->configRead8(ptr);
        uint8_t next = device->configRead8(ptr + 1) & 0xFC;
        if (id == cap_id)
            return ptr;
        ptr = next;
    }
    return 0;
}

bool ItlRal::ral_check_aspm_and_mmio(struct ral_softc *sc)
{
    sc->sc_pcie_cap_off = ral_find_pci_cap(sc->sc_pci_dev, RAL_PCI_CAP_ID_EXP);
    if (sc->sc_pcie_cap_off != 0) {
        sc->sc_aspm_orig = sc->sc_pci_dev->configRead16(sc->sc_pcie_cap_off + RAL_PCIE_LNKCTL_OFF);
        if (sc->sc_aspm_orig & RAL_PCIE_LNKCTL_ASPM_MASK) {
            uint16_t lnkctl = sc->sc_aspm_orig & ~RAL_PCIE_LNKCTL_ASPM_MASK;
            sc->sc_pci_dev->configWrite16(sc->sc_pcie_cap_off + RAL_PCIE_LNKCTL_OFF, lnkctl);
            sc->sc_aspm_cleared = true;
            IODelay(1000);
        }
    }

    uint32_t csr0 = 0xFFFFFFFFU;
    for (int i = 0; i < 20; i++) {
        csr0 = ral_read32(RAL_MAC_CSR0);
        if (csr0 != 0x00000000U && csr0 != 0xFFFFFFFFU)
            break;
        IODelay(1000);
    }

    if ((csr0 == 0x00000000U || csr0 == 0xFFFFFFFFU) && sc->sc_pcie_cap_off != 0) {
        if (sc->sc_aspm_orig & RAL_PCIE_LNKCTL_ASPM_MASK) {
            uint16_t lnkctl = sc->sc_aspm_orig & ~RAL_PCIE_LNKCTL_ASPM_MASK;
            sc->sc_pci_dev->configWrite16(sc->sc_pcie_cap_off + RAL_PCIE_LNKCTL_OFF, lnkctl);
            sc->sc_aspm_cleared = true;
            IODelay(5000);
            for (int i = 0; i < 20; i++) {
                csr0 = ral_read32(RAL_MAC_CSR0);
                if (csr0 != 0x00000000U && csr0 != 0xFFFFFFFFU)
                    break;
                IODelay(1000);
            }
        }
    }

    sc->sc_mac_csr0 = csr0;
    sc->sc_chipset  = (uint16_t)((csr0 & RAL_MAC_CSR0_CHIPSET_MASK) >> RAL_MAC_CSR0_CHIPSET_SHIFT);
    sc->sc_rev      = (uint16_t)(csr0 & RAL_MAC_CSR0_REV_MASK);
    return (csr0 != 0x00000000U && csr0 != 0xFFFFFFFFU);
}

const char *ItlRal::ral_rev_name(uint16_t chipset, uint16_t rev)
{
    if (chipset == RAL_CHIP_RT5390) {
        switch (rev) {
            case RAL_REV_RT5390F: return "RT5390F";
            case RAL_REV_RT5390R: return "RT5390R";
            case RAL_REV_RT5370G: return "RT5370G/RT5390G";
            default:              return "RT5390 (other rev)";
        }
    }
    return "Unknown Ralink ASIC";
}

const char *ItlRal::ral_ant_div_name(uint8_t mode)
{
    switch (mode) {
        case RAL_ANT_DIV_DISABLE:    return "Disabled";
        case RAL_ANT_DIV_DIVERSITY:  return "Hardware Diversity";
        case RAL_ANT_DIV_FIXED_MAIN: return "Fixed Main";
        case RAL_ANT_DIV_FIXED_AUX:  return "Fixed Aux";
        default:                     return "Unknown";
    }
}

bool ItlRal::ral_efuse_detect(struct ral_softc *sc)
{
    return (ral_read32(RAL_EFUSE_CTRL) & RAL_EFUSE_CTRL_PRESENT) != 0;
}

bool ItlRal::ral_efuse_read_block(struct ral_softc *sc, uint16_t word_base, uint16_t out_words[8])
{
    uint32_t reg = ral_read32(RAL_EFUSE_CTRL);
    reg &= ~(RAL_EFUSE_CTRL_AIN_MASK | RAL_EFUSE_CTRL_MODE_MASK);
    reg |= (((uint32_t)(word_base & ~0x7U)) << RAL_EFUSE_CTRL_AIN_SHIFT) & RAL_EFUSE_CTRL_AIN_MASK;
    reg |= RAL_EFUSE_CTRL_KICK;
    ral_write32(RAL_EFUSE_CTRL, reg);

    int tries;
    for (tries = 0; tries < 1000; tries++) {
        if ((ral_read32(RAL_EFUSE_CTRL) & RAL_EFUSE_CTRL_KICK) == 0)
            break;
        IODelay(10);
    }
    if (tries == 1000)
        return false;

    uint32_t d3 = ral_read32(RAL_EFUSE_DATA3);
    uint32_t d2 = ral_read32(RAL_EFUSE_DATA2);
    uint32_t d1 = ral_read32(RAL_EFUSE_DATA1);
    uint32_t d0 = ral_read32(RAL_EFUSE_DATA0);

    out_words[0] = (uint16_t)(d3 & 0xFFFFU);
    out_words[1] = (uint16_t)((d3 >> 16) & 0xFFFFU);
    out_words[2] = (uint16_t)(d2 & 0xFFFFU);
    out_words[3] = (uint16_t)((d2 >> 16) & 0xFFFFU);
    out_words[4] = (uint16_t)(d1 & 0xFFFFU);
    out_words[5] = (uint16_t)((d1 >> 16) & 0xFFFFU);
    out_words[6] = (uint16_t)(d0 & 0xFFFFU);
    out_words[7] = (uint16_t)((d0 >> 16) & 0xFFFFU);
    return true;
}

bool ItlRal::ral_read_eeprom_efuse(struct ral_softc *sc)
{
    for (uint16_t w = 0; w < RAL_EEPROM_WORDS; w += 8) {
        if (!ral_efuse_read_block(sc, w, &sc->sc_eeprom[w]))
            return false;
    }
    bool all_zero = true, all_ff = true;
    for (int i = 0; i < 32; i++) {
        if (sc->sc_eeprom[i] != 0x0000) all_zero = false;
        if (sc->sc_eeprom[i] != 0xFFFF) all_ff   = false;
    }
    return !(all_zero || all_ff);
}

void ItlRal::ral_eeprom_pulse_clk(uint32_t base)
{
    ral_write32(RAL_E2PROM_CSR, base | RAL_E2PROM_CSR_DATA_CLOCK);
    IODelay(2);
    ral_write32(RAL_E2PROM_CSR, base);
    IODelay(2);
}

uint16_t ItlRal::ral_eeprom_93cx6_read_word(struct ral_softc *sc, uint16_t word_addr, int addr_bits)
{
    uint32_t cs = RAL_E2PROM_CSR_CHIP_SELECT;
    ral_write32(RAL_E2PROM_CSR, 0);
    IODelay(2);
    ral_write32(RAL_E2PROM_CSR, cs);
    IODelay(2);

    uint32_t cmd = (0x6U << addr_bits) | (word_addr & ((1U << addr_bits) - 1U));
    for (int bit = addr_bits + 2; bit >= 0; bit--) {
        uint32_t val = cs | (((cmd >> bit) & 1U) ? RAL_E2PROM_CSR_DATA_IN : 0);
        ral_write32(RAL_E2PROM_CSR, val);
        IODelay(1);
        ral_eeprom_pulse_clk(val);
    }

    uint16_t result = 0;
    for (int bit = 15; bit >= 0; bit--) {
        ral_write32(RAL_E2PROM_CSR, cs | RAL_E2PROM_CSR_DATA_CLOCK);
        IODelay(2);
        if (ral_read32(RAL_E2PROM_CSR) & RAL_E2PROM_CSR_DATA_OUT)
            result |= (uint16_t)(1U << bit);
        ral_write32(RAL_E2PROM_CSR, cs);
        IODelay(2);
    }

    ral_write32(RAL_E2PROM_CSR, 0);
    IODelay(2);
    return result;
}

bool ItlRal::ral_read_eeprom_pci(struct ral_softc *sc)
{
    uint32_t csr = ral_read32(RAL_E2PROM_CSR);
    int addr_bits = (((csr & RAL_E2PROM_CSR_TYPE_MASK) >> RAL_E2PROM_CSR_TYPE_SHIFT) == 0) ? 6 : 8;
    for (uint16_t w = 0; w < RAL_EEPROM_WORDS; w++)
        sc->sc_eeprom[w] = ral_eeprom_93cx6_read_word(sc, w, addr_bits);
    bool all_zero = true, all_ff = true;
    for (int i = 0; i < 32; i++) {
        if (sc->sc_eeprom[i] != 0x0000) all_zero = false;
        if (sc->sc_eeprom[i] != 0xFFFF) all_ff   = false;
    }
    return !(all_zero || all_ff);
}

void ItlRal::ral_parse_eeprom(struct ral_softc *sc)
{
    uint16_t m0 = sc->sc_eeprom[RAL_EEPROM_MAC_ADDR_0];
    uint16_t m1 = sc->sc_eeprom[RAL_EEPROM_MAC_ADDR_1];
    uint16_t m2 = sc->sc_eeprom[RAL_EEPROM_MAC_ADDR_2];
    sc->sc_mac_addr[0] = (uint8_t)(m0 & 0xFF);
    sc->sc_mac_addr[1] = (uint8_t)((m0 >> 8) & 0xFF);
    sc->sc_mac_addr[2] = (uint8_t)(m1 & 0xFF);
    sc->sc_mac_addr[3] = (uint8_t)((m1 >> 8) & 0xFF);
    sc->sc_mac_addr[4] = (uint8_t)(m2 & 0xFF);
    sc->sc_mac_addr[5] = (uint8_t)((m2 >> 8) & 0xFF);

    bool mac_zero = true, mac_ff = true;
    for (int i = 0; i < 6; i++) {
        if (sc->sc_mac_addr[i] != 0x00) mac_zero = false;
        if (sc->sc_mac_addr[i] != 0xFF) mac_ff   = false;
    }
    if (mac_zero || mac_ff || (sc->sc_mac_addr[0] & 0x01)) {
        sc->sc_mac_addr[0] = 0x02; sc->sc_mac_addr[1] = 0x18; sc->sc_mac_addr[2] = 0x14;
        sc->sc_mac_addr[3] = 0x53; sc->sc_mac_addr[4] = 0x9F; sc->sc_mac_addr[5] = 0x01;
    }

    sc->sc_nic_conf0    = sc->sc_eeprom[RAL_EEPROM_NIC_CONF0];
    sc->sc_nic_conf1    = sc->sc_eeprom[RAL_EEPROM_NIC_CONF1];
    sc->sc_nrxchains    = (uint8_t)(sc->sc_nic_conf0 & RAL_NIC_CONF0_RXPATH_MASK);
    sc->sc_ntxchains    = (uint8_t)((sc->sc_nic_conf0 & RAL_NIC_CONF0_TXPATH_MASK) >> RAL_NIC_CONF0_TXPATH_SHIFT);
    sc->sc_rf_type      = (uint8_t)((sc->sc_nic_conf0 & RAL_NIC_CONF0_RFTYPE_MASK) >> RAL_NIC_CONF0_RFTYPE_SHIFT);
    if (sc->sc_ntxchains == 0 || sc->sc_ntxchains > 3) sc->sc_ntxchains = 1;
    if (sc->sc_nrxchains == 0 || sc->sc_nrxchains > 3) sc->sc_nrxchains = 1;

    sc->sc_ant_div_mode = (uint8_t)((sc->sc_nic_conf1 & RAL_NIC_CONF1_ANT_DIV_MASK) >> RAL_NIC_CONF1_ANT_DIV_SHIFT);
    sc->sc_hw_radio     = (sc->sc_nic_conf1 & RAL_NIC_CONF1_HW_RADIO) != 0;
    sc->sc_ext_lna_2g   = (sc->sc_nic_conf1 & RAL_NIC_CONF1_EXT_LNA_2G) != 0;
    sc->sc_int_tx_alc   = (sc->sc_nic_conf1 & RAL_NIC_CONF1_INT_TX_ALC) != 0;
}

/* -------------------------------------------------------------------------- */
/* IORegistry Live Telemetry                                                  */
/* -------------------------------------------------------------------------- */

void ItlRal::ral_publish_diagnostics(struct ral_softc *sc)
{
    IOService *ctrl = getController();
    if (!ctrl)
        return;

    char buf[192];

    ctrl->setProperty("ralwm-phase", "Phase 7 (NoASPM+AMPDU+MCS5-7)");

    snprintf(buf, sizeof(buf), "0x%08x (%s) efuse=%d conf1=0x%04x aspm_off=%d",
             sc->sc_mac_csr0, ral_rev_name(sc->sc_chipset, sc->sc_rev),
             sc->sc_efuse_present ? 1 : 0, sc->sc_nic_conf1, sc->sc_aspm_cleared ? 1 : 0);
    ctrl->setProperty("ralwm-hw", buf);

    snprintf(buf, sizeof(buf), "%s (mcu_ms=%d bbp_r0=0x%02x bbp_r152=0x%02x rf=[%02x %02x %02x])",
             sc->sc_phase2_status[0] ? sc->sc_phase2_status : "pending",
             sc->sc_mcu_ready_ms, sc->sc_bbp_r0, sc->sc_bbp_r152,
             sc->sc_rf_r1, sc->sc_rf_r2, sc->sc_rf_r3);
    ctrl->setProperty("ralwm-phase2", buf);

    snprintf(buf, sizeof(buf), "dma=%d run=%d irqs=%u rx=%u crc=%u tx=%u ch=%d st=%d rssi=%ddBm",
             sc->sc_dma_allocated ? 1 : 0, sc->sc_running ? 1 : 0,
             sc->sc_irq_count, sc->sc_rx_count, sc->sc_rx_crc_err,
             sc->sc_tx_count, sc->sc_cur_chan, (int)sc->sc_ic.ic_state, sc->sc_last_rssi);
    ctrl->setProperty("ralwm-phase3", buf);

    ctrl->setProperty("ralwm-ssids", sc->sc_seen_ssids[0] ? sc->sc_seen_ssids : "(none yet)");

    if (sc->sc_mmio_base) {
        snprintf(buf, sizeof(buf), "drx=%u calc=%u gpio=0x%08x sta0=0x%08x sta1=0x%08x bbp152=0x%02x",
                 ral_read32(RAL_RX_DRX_IDX), ral_read32(RAL_RX_CALC_IDX),
                 ral_read32(0x0228), ral_read32(0x1700), ral_read32(0x1704),
                 sc->sc_bbp_r152);
        ctrl->setProperty("ralwm-rx-dbg", buf);
        snprintf(buf, sizeof(buf), "txpin=0x%08x arsp=0x%08x lbr=0x%08x rty=0x%08x pwr0=0x%08x pwr1=0x%08x",
                 ral_read32(RAL_TX_PIN_CFG), ral_read32(RAL_AUTO_RSP_CFG),
                 ral_read32(RAL_LEGACY_BASIC_RATE), ral_read32(RAL_TX_RTY_CFG),
                 ral_read32(RAL_TX_PWR_CFG_0), ral_read32(RAL_TX_PWR_CFG_1));
        ctrl->setProperty("ralwm-rx-dbg2", buf);
        int is_ht = 0;
#ifdef IEEE80211_NODE_HT
        if (sc->sc_ic.ic_bss && (sc->sc_ic.ic_bss->ni_flags & IEEE80211_NODE_HT))
            is_ht = 1;
#endif
        uint32_t txs0 = ral_read32(RAL_TX_STA_CNT0);
        uint32_t txs1 = ral_read32(RAL_TX_STA_CNT1);
        sc->sc_tx_hw_fail += (txs0 & 0xFFFFU);
        sc->sc_tx_hw_ok   += (txs1 & 0xFFFFU);
        sc->sc_tx_hw_rty  += ((txs1 >> 16) & 0xFFFFU);
        snprintf(buf, sizeof(buf), "ok=%u rty=%u fail=%u mcs=%u rxdup=0x%08x ht=%d",
                 sc->sc_tx_hw_ok, sc->sc_tx_hw_rty, sc->sc_tx_hw_fail,
                 sc->sc_tx_mcs, ral_read32(RAL_RX_STA_CNT2), is_ht);
        ctrl->setProperty("ralwm-tx-dbg", buf);
    }
}

/* -------------------------------------------------------------------------- */
/* Phase 2: MCU Firmware Upload, MAC, BBP, and RF5390 Initialization          */
/* -------------------------------------------------------------------------- */

IOReturn ItlRal::ral_wait_dma_idle(const char *ctx)
{
    for (int i = 0; i < 100; i++) {
        if ((ral_read32(RAL_WPDMA_GLO_CFG) & (RAL_TX_DMA_BUSY | RAL_RX_DMA_BUSY)) == 0)
            return kIOReturnSuccess;
        IODelay(1000);
    }
    return kIOReturnTimeout;
}

IOReturn ItlRal::ral_wait_mac_idle()
{
    for (int i = 0; i < 100; i++) {
        if ((ral_read32(RAL_MAC_STATUS_REG) & (RAL_TX_STATUS_BUSY | RAL_RX_STATUS_BUSY)) == 0)
            return kIOReturnSuccess;
        IODelay(1000);
    }
    return kIOReturnTimeout;
}

void ItlRal::ral_set_macaddr(const uint8_t *addr)
{
    ral_write32(RAL_MAC_ADDR_DW0,
                (uint32_t)addr[0] | ((uint32_t)addr[1] << 8) |
                ((uint32_t)addr[2] << 16) | ((uint32_t)addr[3] << 24));
    ral_write32(RAL_MAC_ADDR_DW1,
                (uint32_t)addr[4] | ((uint32_t)addr[5] << 8) | (0xFFU << 16));
}

void ItlRal::ral_mac_init()
{
    for (size_t i = 0; i < sizeof(ral_def_mac) / sizeof(ral_def_mac[0]); i++)
        ral_write32(ral_def_mac[i].reg, ral_def_mac[i].val);
}

IOReturn ItlRal::ral_fw_upload(struct ral_softc *sc)
{
    if (rt2860_fw_len != RAL_FW_SIZE) {
        strlcpy(sc->sc_phase2_status, "FAIL: fw size != 8192", sizeof(sc->sc_phase2_status));
        return kIOReturnBadArgument;
    }

    ral_write32(RAL_SYS_CTRL, RAL_HST_PM_SEL);
    for (uint32_t off = 0; off < rt2860_fw_len; off += 4) {
        uint32_t word = (uint32_t)rt2860_fw_data[off + 0] |
                        ((uint32_t)rt2860_fw_data[off + 1] << 8) |
                        ((uint32_t)rt2860_fw_data[off + 2] << 16) |
                        ((uint32_t)rt2860_fw_data[off + 3] << 24);
        ral_write32(RAL_FW_BASE + off, word);
    }

    ral_write32(RAL_SYS_CTRL, 0);
    ral_write32(RAL_SYS_CTRL, RAL_MCU_RESET);
    ral_write32(RAL_H2M_BBPAGENT, 0);
    ral_write32(RAL_H2M_MAILBOX, 0);

    for (int ms = 0; ms < 1000; ms++) {
        if (ral_read32(RAL_SYS_CTRL) & RAL_MCU_READY) {
            sc->sc_mcu_ready_ms = ms;
            return kIOReturnSuccess;
        }
        IODelay(1000);
    }

    sc->sc_mcu_ready_ms = -1;
    strlcpy(sc->sc_phase2_status, "FAIL: MCU timeout", sizeof(sc->sc_phase2_status));
    return kIOReturnTimeout;
}

int ItlRal::ral_mcu_cmd(uint8_t cmd, uint16_t arg, bool wait)
{
    int ntries;
    for (ntries = 0; ntries < 100; ntries++) {
        if ((ral_read32(RAL_H2M_MAILBOX) & RAL_H2M_BUSY) == 0)
            break;
        IODelay(2);
    }
    if (ntries == 100)
        return EIO;

    uint8_t cid = wait ? cmd : RAL_TOKEN_NO_INTR;
    ral_write32(RAL_H2M_MAILBOX, RAL_H2M_BUSY | ((uint32_t)cid << 16) | arg);
    ral_write32(RAL_HOST_CMD, cmd);

    if (!wait)
        return 0;

    for (ntries = 0; ntries < 200; ntries++) {
        uint32_t tmp = ral_read32(RAL_H2M_MAILBOX_CID);
        for (int slot = 0; slot < 4; slot++, tmp >>= 8) {
            if ((tmp & 0xFFU) == cid) {
                uint32_t st = ral_read32(RAL_H2M_MAILBOX_STATUS) & 0xFFU;
                ral_write32(RAL_H2M_MAILBOX_STATUS, 0xFFFFFFFFU);
                ral_write32(RAL_H2M_MAILBOX_CID, 0xFFFFFFFFU);
                return (st == 1) ? 0 : EIO;
            }
        }
        IODelay(100);
    }
    ral_write32(RAL_H2M_MAILBOX_STATUS, 0xFFFFFFFFU);
    ral_write32(RAL_H2M_MAILBOX_CID, 0xFFFFFFFFU);
    return ETIMEDOUT;
}

void ItlRal::ral_bbp_write(uint8_t reg, uint8_t val)
{
    int ntries;
    for (ntries = 0; ntries < 100; ntries++) {
        if ((ral_read32(RAL_H2M_BBPAGENT) & RAL_BBP_CSR_KICK) == 0)
            break;
        IODelay(10);
    }
    if (ntries == 100)
        return;

    ral_write32(RAL_H2M_BBPAGENT,
                RAL_BBP_RW_PARALLEL | RAL_BBP_CSR_KICK | ((uint32_t)reg << 8) | val);
    ral_mcu_cmd(RAL_MCU_CMD_BBP, 0, false);

    for (ntries = 0; ntries < 100; ntries++) {
        if ((ral_read32(RAL_H2M_BBPAGENT) & RAL_BBP_CSR_KICK) == 0)
            break;
        IODelay(10);
    }
}

uint8_t ItlRal::ral_bbp_read(uint8_t reg)
{
    int ntries;
    for (ntries = 0; ntries < 100; ntries++) {
        if ((ral_read32(RAL_H2M_BBPAGENT) & RAL_BBP_CSR_KICK) == 0)
            break;
        IODelay(10);
    }
    if (ntries == 100)
        return 0;

    ral_write32(RAL_H2M_BBPAGENT,
                RAL_BBP_RW_PARALLEL | RAL_BBP_CSR_KICK | RAL_BBP_CSR_READ | ((uint32_t)reg << 8));
    ral_mcu_cmd(RAL_MCU_CMD_BBP, 0, false);

    for (ntries = 0; ntries < 100; ntries++) {
        uint32_t v = ral_read32(RAL_H2M_BBPAGENT);
        if ((v & RAL_BBP_CSR_KICK) == 0)
            return (uint8_t)(v & 0xFFU);
        IODelay(10);
    }
    return 0;
}

void ItlRal::ral_rf_write(uint8_t reg, uint8_t val)
{
    int ntries;
    for (ntries = 0; ntries < 100; ntries++) {
        if ((ral_read32(RAL_RF_CSR_CFG) & RAL_RF_KICK) == 0)
            break;
        IODelay(10);
    }
    if (ntries == 100)
        return;

    ral_write32(RAL_RF_CSR_CFG, RAL_RF_WRITE | RAL_RF_KICK | ((uint32_t)reg << 8) | val);
}

uint8_t ItlRal::ral_rf_read(uint8_t reg)
{
    int ntries;
    for (ntries = 0; ntries < 100; ntries++) {
        if ((ral_read32(RAL_RF_CSR_CFG) & RAL_RF_KICK) == 0)
            break;
        IODelay(10);
    }
    if (ntries == 100)
        return 0xFF;

    ral_write32(RAL_RF_CSR_CFG, RAL_RF_KICK | ((uint32_t)reg << 8));
    for (ntries = 0; ntries < 100; ntries++) {
        uint32_t v = ral_read32(RAL_RF_CSR_CFG);
        if ((v & RAL_RF_KICK) == 0)
            return (uint8_t)(v & 0xFFU);
        IODelay(10);
    }
    return 0xFF;
}

bool ItlRal::ral_bbp_init(struct ral_softc *sc)
{
    uint8_t bbp0 = 0;
    for (int i = 0; i < 20; i++) {
        bbp0 = ral_bbp_read(0);
        if (bbp0 != 0x00 && bbp0 != 0xFF)
            break;
        IODelay(1000);
    }
    sc->sc_bbp_r0 = bbp0;
    if (bbp0 == 0x00 || bbp0 == 0xFF) {
        strlcpy(sc->sc_phase2_status, "FAIL: BBP R0 unresponsive", sizeof(sc->sc_phase2_status));
        return false;
    }

    for (size_t i = 0; i < sizeof(rt5390_def_bbp) / sizeof(rt5390_def_bbp[0]); i++)
        ral_bbp_write(rt5390_def_bbp[i].reg, rt5390_def_bbp[i].val);

    uint8_t bbp = ral_bbp_read(152);
    ral_bbp_write(152, bbp | RAL_BBP152_RX_DEFAULT_ANT);
    ral_bbp_write(154, 0x00);

    ral_bbp_write(79, 0x13);
    ral_bbp_write(80, 0x05);
    ral_bbp_write(81, 0x33);
    ral_bbp_write(103, 0xc0);

    bbp = ral_bbp_read(138);
    if (sc->sc_ntxchains == 1) bbp |= 0x20;
    if (sc->sc_nrxchains == 1) bbp &= ~0x02;
    ral_bbp_write(138, bbp);

    /* Explicit 1T1R BBP R1 / R3 antenna path config */
    bbp = ral_bbp_read(1);
    ral_bbp_write(1, bbp & ~0x18U);
    bbp = ral_bbp_read(3);
    ral_bbp_write(3, bbp & ~0x18U);

    /* LNA gain from EEPROM word 0x22 low byte (fallback 0x08) -> BBP R62/63/64 & R66 */
    uint8_t lna = (uint8_t)(sc->sc_eeprom[0x22] & 0xFFU);
    if (lna == 0x00 || lna == 0xFF || lna > 0x20)
        lna = 0x08;
    uint8_t r62 = (uint8_t)(0x37 - lna);
    ral_bbp_write(62, r62);
    ral_bbp_write(63, r62);
    ral_bbp_write(64, r62);
    ral_bbp_write(66, (uint8_t)(0x1C + 2 * lna));

    bbp = ral_bbp_read(4);
    ral_bbp_write(4, bbp | RAL_BBP_MAC_IF_CTRL);
    return true;
}

void ItlRal::ral_rf_rev_patch(struct ral_softc *sc)
{
    if (sc->sc_rev >= RAL_REV_RT5390F) {
        ral_rf_write(6,  0xe0);
        ral_rf_write(46, 0x73);
        ral_rf_write(53, 0x00);
        ral_rf_write(56, 0x42);
        ral_rf_write(61, 0xd5);
    } else {
        ral_rf_write(61, 0xb5);
    }
}

void ItlRal::ral_rf_init(struct ral_softc *sc)
{
    /* RT5390 RF LDO & GPIO switch power-up (OpenBSD rt2860.c / Linux rt2800lib.c) */
    uint32_t ldo = ral_read32(0x05D4);
    ral_write32(0x05D4, (ldo & ~0x1F000000U) | 0x0D000000U);
    ral_write32(0x05DC, ral_read32(0x05DC) & ~0x20U);

    uint8_t rf = ral_rf_read(2);
    ral_rf_write(2, rf | RAL_RF_RESCAL);
    IODelay(1000);
    ral_rf_write(2, rf & ~RAL_RF_RESCAL);

    for (size_t i = 0; i < sizeof(rt5390_def_rf) / sizeof(rt5390_def_rf[0]); i++)
        ral_rf_write(rt5390_def_rf[i].reg, rt5390_def_rf[i].val);

    ral_rf_rev_patch(sc);

    ral_write32(RAL_OPT_14, ral_read32(RAL_OPT_14) | 1U);
    ral_write32(RAL_TX_SW_CFG1, 0);
    ral_write32(RAL_TX_SW_CFG2, 0);

    rf = ral_rf_read(38);
    ral_rf_write(38, rf & ~RAL_RF_RX_LO1);
    rf = ral_rf_read(39);
    ral_rf_write(39, rf & ~RAL_RF_RX_LO2);

    rf = ral_rf_read(30);
    ral_rf_write(30, (rf & ~0x18U) | 0x10U);

    /* Crystal frequency offset from EEPROM word 0x1D low byte -> RF R17 */
    uint8_t freq_off = (uint8_t)(sc->sc_eeprom[RAL_EEPROM_FREQ_LED] & 0x7FU);
    if (freq_off != 0x00 && freq_off != 0x7F)
        ral_rf_write(17, freq_off);

    sc->sc_rf_r1 = ral_rf_read(1);
    sc->sc_rf_r2 = ral_rf_read(2);
    sc->sc_rf_r3 = ral_rf_read(3);
}

void ItlRal::ral_set_rx_antenna(struct ral_softc *sc, bool aux)
{
    if (sc->sc_ant_div_mode == RAL_ANT_DIV_FIXED_AUX)
        aux = true;
    else if (sc->sc_ant_div_mode == RAL_ANT_DIV_FIXED_MAIN)
        aux = false;

    uint8_t bbp = ral_bbp_read(152);
    if (aux) bbp &= ~RAL_BBP152_RX_DEFAULT_ANT;
    else     bbp |= RAL_BBP152_RX_DEFAULT_ANT;
    ral_bbp_write(152, bbp);
    sc->sc_bbp_r152 = ral_bbp_read(152);
}

IOReturn ItlRal::ral_init_phase2(struct ral_softc *sc)
{
    sc->sc_phase2_attempted = true;
    sc->sc_phase2_ok = false;

    ral_write32(RAL_INT_MASK_CSR, 0);
    ral_write32(RAL_PWR_PIN_CFG, RAL_IO_RA_PE);
    ral_write32(RAL_WPDMA_GLO_CFG, ral_read32(RAL_WPDMA_GLO_CFG) & 0xFF0U);

    ral_write32(RAL_SYS_CTRL, 0xE1FU);
    ral_write32(RAL_SYS_CTRL, 0xE00U);

    IOReturn err = ral_fw_upload(sc);
    if (err != kIOReturnSuccess)
        return err;

    ral_set_macaddr(sc->sc_mac_addr);

    if (ral_wait_dma_idle("pre-init") != kIOReturnSuccess) {
        strlcpy(sc->sc_phase2_status, "FAIL: DMA idle timeout", sizeof(sc->sc_phase2_status));
        return kIOReturnTimeout;
    }
    ral_write32(RAL_WPDMA_GLO_CFG, ral_read32(RAL_WPDMA_GLO_CFG) & 0xFF0U);
    ral_write32(RAL_WPDMA_RST_IDX, 0x1003FU);

    ral_write32(RAL_SYS_CTRL, 0xE1FU);
    ral_write32(RAL_SYS_CTRL, 0xE00U);
    ral_write32(RAL_PWR_PIN_CFG, RAL_IO_RA_PE | RAL_IO_RF_PE);
    ral_write32(RAL_MAC_SYS_CTRL, RAL_BBP_HRST | RAL_MAC_SRST);
    ral_write32(RAL_MAC_SYS_CTRL, 0);

    ral_mac_init();
    ral_set_macaddr(sc->sc_mac_addr); /* MUST be set AFTER MAC_SRST so HW ACKs unicast frames! */
    ral_write32(RAL_TX_SW_CFG0, 0x00000404U);
    ral_write32(RAL_TX_SW_CFG1, 0);
    ral_write32(RAL_TX_SW_CFG2, 0);

    if ((ral_read32(RAL_PCI_CFG) & RAL_PCI_CFG_PCI) == 0) {
        uint32_t tmp = ral_read32(RAL_US_CYC_CNT);
        ral_write32(RAL_US_CYC_CNT, (tmp & ~0xFFU) | 0x7DU);
    }

    if (ral_wait_mac_idle() != kIOReturnSuccess) {
        strlcpy(sc->sc_phase2_status, "FAIL: MAC idle timeout", sizeof(sc->sc_phase2_status));
        return kIOReturnTimeout;
    }

    ral_write32(RAL_H2M_BBPAGENT, 0);
    ral_write32(RAL_H2M_MAILBOX, 0);
    ral_mcu_cmd(RAL_MCU_CMD_RFRESET, 0, false);
    IODelay(1000);
    ral_mcu_cmd(0x31 /* MCU_CMD_WAKEUP */, 0x00FF, false);
    IODelay(1000);
    ral_write32(0x0408 /* PBF_CFG */, ral_read32(0x0408) | 0x0FU);

    if (!ral_bbp_init(sc))
        return kIOReturnIOError;

    ral_rf_init(sc);
    ral_set_rx_antenna(sc, false);

    /* Clear WCID & WCID_ATTR tables so HW crypto never corrupts RX/TX (nohwcrypt) */
    for (uint32_t off = 0; off < 256 * 8; off += 4)
        ral_write32(0x1800U + off, 0);
    for (uint32_t off = 0; off < 256 * 4; off += 4)
        ral_write32(0x6800U + off, 0);

    sc->sc_phase2_ok = true;
    strlcpy(sc->sc_phase2_status, "OK: MCU+MAC+BBP+RF+LDO+WCID", sizeof(sc->sc_phase2_status));
    return kIOReturnSuccess;
}

/* -------------------------------------------------------------------------- */
/* Phase 3: 32-bit Physical-Masked DMA Rings, IRQ, Channel Tune, RX/TX        */
/* -------------------------------------------------------------------------- */

bool ItlRal::ral_alloc_ring(struct ral_dma_ring *ring, uint32_t count, bool is_rx)
{
    memset(ring, 0, sizeof(*ring));
    ring->count = count;

    /* Ralink RT5390 DMA uses 32-bit physical addresses (< 4GB mask mandatory!) */
    const mach_vm_address_t kMask32 = 0x00000000FFFFF000ULL;
    size_t desc_bytes = (size_t)count * 16;

    ring->desc_mem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous, desc_bytes, kMask32);
    if (!ring->desc_mem)
        return false;
    ring->desc_mem->prepare();
    ring->desc_virt = ring->desc_mem->getBytesNoCopy();
    addr64_t dphys = ring->desc_mem->getPhysicalAddress();
    if (dphys == 0 || (dphys >> 32) != 0)
        return false;
    ring->desc_phys = (uint32_t)dphys;
    memset(ring->desc_virt, 0, desc_bytes);

    for (uint32_t i = 0; i < count; i++) {
        ring->buf_mem[i] = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
            kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous, RAL_DMA_BUF_SIZE, kMask32);
        if (!ring->buf_mem[i])
            return false;
        ring->buf_mem[i]->prepare();
        ring->buf_virt[i] = (uint8_t *)ring->buf_mem[i]->getBytesNoCopy();
        addr64_t bphys = ring->buf_mem[i]->getPhysicalAddress();
        if (bphys == 0 || (bphys >> 32) != 0)
            return false;
        ring->buf_phys[i] = (uint32_t)bphys;
        memset(ring->buf_virt[i], 0, RAL_DMA_BUF_SIZE);

        if (is_rx) {
            struct ral_rxd *rxd = &((struct ral_rxd *)ring->desc_virt)[i];
            rxd->sdp0  = OSSwapHostToLittleInt32(ring->buf_phys[i]);
            rxd->sdl0  = OSSwapHostToLittleInt16((uint16_t)RAL_DMA_BUF_SIZE); /* DDONE=0: owned by HW */
            rxd->sdl1  = 0;
            rxd->sdp1  = 0;
            rxd->flags = 0;
        } else {
            struct ral_txd *txd = &((struct ral_txd *)ring->desc_virt)[i];
            txd->sdp0  = OSSwapHostToLittleInt32(ring->buf_phys[i]);
            txd->sdl0  = OSSwapHostToLittleInt16(RAL_TX_DDONE); /* DDONE=1: owned by CPU/free */
            txd->sdl1  = 0;
            txd->sdp1  = 0;
            txd->flags = 0;
        }
    }
    return true;
}

void ItlRal::ral_free_ring(struct ral_dma_ring *ring)
{
    for (uint32_t i = 0; i < RAL_RX_RING_COUNT; i++) {
        if (ring->buf_mem[i]) {
            ring->buf_mem[i]->complete();
            ring->buf_mem[i]->release();
            ring->buf_mem[i] = NULL;
        }
        ring->buf_virt[i] = NULL;
        ring->buf_phys[i] = 0;
    }
    if (ring->desc_mem) {
        ring->desc_mem->complete();
        ring->desc_mem->release();
        ring->desc_mem = NULL;
    }
    ring->desc_virt = NULL;
    ring->desc_phys = 0;
}

bool ItlRal::ral_alloc_dma(struct ral_softc *sc)
{
    if (sc->sc_dma_allocated)
        return true;

    if (!ral_alloc_ring(&sc->sc_rxq, RAL_RX_RING_COUNT, true) ||
        !ral_alloc_ring(&sc->sc_txq_ac0, RAL_TX_RING_COUNT, false) ||
        !ral_alloc_ring(&sc->sc_txq_mgmt, RAL_TX_RING_COUNT, false)) {
        ral_free_dma(sc);
        return false;
    }
    sc->sc_dma_allocated = true;
    return true;
}

void ItlRal::ral_free_dma(struct ral_softc *sc)
{
    ral_free_ring(&sc->sc_rxq);
    ral_free_ring(&sc->sc_txq_ac0);
    ral_free_ring(&sc->sc_txq_mgmt);
    sc->sc_dma_allocated = false;
}

void ItlRal::ral_setup_dma_hw(struct ral_softc *sc)
{
    /* CRITICAL: Reset WPDMA ring indices FIRST, BEFORE programming RX_CALC_IDX!
     * Writing WPDMA_RST_IDX=0x1003F after RX_CALC_IDX resets RX_CALC_IDX to 0,
     * which makes HW see RX_CALC_IDX==RX_DRX_IDX (0 free RX descriptors) and drop all RX! */
    ral_write32(RAL_WPDMA_RST_IDX, 0x1003FU);
    ral_write32(RAL_DELAY_INT_CFG, 0);

    /* Zero unused TX rings 1..4, program AC0 (0) and MGMT (5) */
    for (int q = 0; q < 6; q++) {
        ral_write32(RAL_TX_BASE_PTR(q), 0);
        ral_write32(RAL_TX_MAX_CNT(q),  0);
        ral_write32(RAL_TX_CTX_IDX(q),  0);
    }

    sc->sc_txq_ac0.cur = 0;
    sc->sc_txq_ac0.next = 0;
    ral_write32(RAL_TX_BASE_PTR(RAL_TXQ_AC0), sc->sc_txq_ac0.desc_phys);
    ral_write32(RAL_TX_MAX_CNT(RAL_TXQ_AC0),  sc->sc_txq_ac0.count);
    ral_write32(RAL_TX_CTX_IDX(RAL_TXQ_AC0),  0);

    sc->sc_txq_mgmt.cur = 0;
    sc->sc_txq_mgmt.next = 0;
    ral_write32(RAL_TX_BASE_PTR(RAL_TXQ_MGMT), sc->sc_txq_mgmt.desc_phys);
    ral_write32(RAL_TX_MAX_CNT(RAL_TXQ_MGMT),  sc->sc_txq_mgmt.count);
    ral_write32(RAL_TX_CTX_IDX(RAL_TXQ_MGMT),  0);

    /* Program RX ring LAST so RX_CALC_IDX stays at count - 1 (63 free slots) */
    sc->sc_rxq.cur = 0;
    for (uint32_t i = 0; i < sc->sc_rxq.count; i++) {
        struct ral_rxd *rxd = &((struct ral_rxd *)sc->sc_rxq.desc_virt)[i];
        rxd->sdp0  = OSSwapHostToLittleInt32(sc->sc_rxq.buf_phys[i]);
        rxd->sdl0  = OSSwapHostToLittleInt16((uint16_t)RAL_DMA_BUF_SIZE);
        rxd->flags = 0;
    }
    OSSynchronizeIO();
    ral_write32(RAL_RX_BASE_PTR, sc->sc_rxq.desc_phys);
    ral_write32(RAL_RX_MAX_CNT,  sc->sc_rxq.count);
    ral_write32(RAL_RX_CALC_IDX, sc->sc_rxq.count - 1);
}

bool ItlRal::ral_setup_interrupts(struct ral_softc *sc)
{
    if (fInterruptSource)
        return true;

    IOWorkLoop *wl = getController() ? getController()->getWorkLoop() : NULL;
    if (!wl || !sc->sc_pci_dev)
        return false;

    /* Prefer MSI if available, otherwise fall back cleanly to Legacy INTx (index 0) */
    int irq_idx = 0;
    for (int idx = 0; idx < 4; idx++) {
        int type = 0;
        if (sc->sc_pci_dev->getInterruptType(idx, &type) == kIOReturnSuccess) {
            if (type & kIOInterruptTypePCIMessaged) {
                irq_idx = idx;
                break;
            }
        } else {
            break;
        }
    }
    sc->sc_irq_index = irq_idx;

    fInterruptSource = IOInterruptEventSource::interruptEventSource(
        this, &ItlRal::ral_interrupt_occurred, sc->sc_pci_dev, irq_idx);
    if (!fInterruptSource)
        return false;

    if (wl->addEventSource(fInterruptSource) != kIOReturnSuccess) {
        fInterruptSource->release();
        fInterruptSource = NULL;
        return false;
    }

    if (!fScanTimer) {
        fScanTimer = IOTimerEventSource::timerEventSource(this, &ItlRal::ral_scan_timer_fired);
        if (fScanTimer)
            wl->addEventSource(fScanTimer);
    }

    fInterruptSource->enable();
    return true;
}

void ItlRal::ral_teardown_interrupts()
{
    IOWorkLoop *wl = getController() ? getController()->getWorkLoop() : NULL;
    if (fScanTimer) {
        fScanTimer->cancelTimeout();
        if (wl) wl->removeEventSource(fScanTimer);
        fScanTimer->release();
        fScanTimer = NULL;
    }
    if (fInterruptSource) {
        fInterruptSource->disable();
        if (wl) wl->removeEventSource(fInterruptSource);
        fInterruptSource->release();
        fInterruptSource = NULL;
    }
}

static uint16_t ral_clamp_rpwr_word(uint16_t w)
{
    if (w == 0x0000 || w == 0xFFFF)
        return 0x8888U;
    uint16_t out = 0;
    for (int shift = 0; shift < 16; shift += 4) {
        uint16_t nib = (w >> shift) & 0xFU;
        if (nib < 0x6U)
            nib = 0x8U;
        if (nib > 0xCU)
            nib = 0xCU;
        out |= (uint16_t)(nib << shift);
    }
    return out;
}

void ItlRal::ral_config_txpower(struct ral_softc *sc)
{
    /* Program TX_PWR_CFG_0..4 (0x1314..0x1324) from EEPROM 0x6F..0x77 (9 words, 20 MHz per-rate) */
    uint16_t w[10];
    for (int i = 0; i < 9; i++)
        w[i] = ral_clamp_rpwr_word(sc->sc_eeprom[RAL_EEPROM_RPWR + i]);
    w[9] = 0x0000U;

    ral_write32(RAL_TX_PWR_CFG_0, (uint32_t)w[0] | ((uint32_t)w[1] << 16));
    ral_write32(RAL_TX_PWR_CFG_1, (uint32_t)w[2] | ((uint32_t)w[3] << 16));
    ral_write32(RAL_TX_PWR_CFG_2, (uint32_t)w[4] | ((uint32_t)w[5] << 16));
    ral_write32(RAL_TX_PWR_CFG_3, (uint32_t)w[6] | ((uint32_t)w[7] << 16));
    ral_write32(RAL_TX_PWR_CFG_4, (uint32_t)w[8] | ((uint32_t)w[9] << 16));
}

void ItlRal::ral_config_tx_pin_2g()
{
    /* Safe Read-Modify-Write on TX_PIN_CFG (0x1328):
     * Preserve ALL polarity bits (PA_PE_*_POL, LNA_PE_*_POL, RFTR_POL, TRSW_POL, RFRX_*).
     * Enable 2.4 GHz chain-0 PA (bit 1), chain-0 LNA (bit 9), RFTR (bit 16), TRSW (bit 18).
     * Clear 5 GHz PA (bits 0, 2) and chain-1 PA (bit 3) for 1T1R RT5390F. */
    uint32_t txpin = ral_read32(RAL_TX_PIN_CFG);
    txpin &= ~(RAL_TX_PIN_PA_PE_A0_EN | RAL_TX_PIN_PA_PE_A1_EN | RAL_TX_PIN_PA_PE_G1_EN);
    txpin |= (RAL_TX_PIN_PA_PE_G0_EN | RAL_TX_PIN_LNA_PE_G0_EN |
              RAL_TX_PIN_RFTR_EN | RAL_TX_PIN_TRSW_EN);
    ral_write32(RAL_TX_PIN_CFG, txpin);
}

void ItlRal::ral_set_wcid(uint8_t wcid, const uint8_t *mac)
{
    uint32_t base = RAL_MAC_WCID_BASE + ((uint32_t)wcid * 8U);
    if (mac) {
        ral_write32(base + 0,
                    (uint32_t)mac[0] | ((uint32_t)mac[1] << 8) |
                    ((uint32_t)mac[2] << 16) | ((uint32_t)mac[3] << 24));
        ral_write32(base + 4,
                    (uint32_t)mac[4] | ((uint32_t)mac[5] << 8));
    } else {
        ral_write32(base + 0, 0);
        ral_write32(base + 4, 0);
    }
    /* Keep WCID_ATTR zeroed so HW crypto stays completely disabled (RAL_NOHWCRYPT = 1) */
    ral_write32(RAL_MAC_WCID_ATTR_BASE + ((uint32_t)wcid * 4U), 0);
}

void ItlRal::ral_set_chan(struct ral_softc *sc, int chan)
{
    if (chan < 1 || chan > 14)
        chan = 1;
    int idx = chan - 1;

    ral_rf_write(8, rt5390_rf_ch[idx].n);
    uint8_t rf9 = ral_rf_read(9);
    ral_rf_write(9, (rf9 & ~0x0FU) | (rt5390_rf_ch[idx].k & 0x0FU));

    uint8_t rf = ral_rf_read(11);
    ral_rf_write(11, (rf & ~0x03U) | (rt5390_rf_ch[idx].r & 0x03U));

    /* Per-channel TX power from EEPROM (word 0x29+, 1 byte per ch) with safe clamp */
    uint16_t pwr_word = sc->sc_eeprom[RAL_EEPROM_TXPOW1_2G + (idx / 2)];
    uint8_t txpow = (idx & 1) ? (uint8_t)((pwr_word >> 8) & 0xFF) : (uint8_t)(pwr_word & 0xFF);
    if (txpow == 0 || txpow > 0x27)
        txpow = 0x14;
    else if (txpow < 0x10)
        txpow = 0x10;
    rf = ral_rf_read(49);
    ral_rf_write(49, (rf & ~0x3FU) | (txpow & 0x3FU));

    /* Enable RF block, PLL, RX0, TX0 */
    rf = ral_rf_read(1);
    ral_rf_write(1, (rf & ~0x7CU) | 0x0FU);

    /* RT5390F/R non-BT channel-specific RF R55 / R59 */
    if (sc->sc_rev >= RAL_REV_RT5390F) {
        ral_rf_write(55, rt5390_r55_nonbt_rev[idx]);
        ral_rf_write(59, rt5390_r59_nonbt_rev[idx]);
    }

    /* 20 MHz bandwidth & 2.4 GHz TX band / pin / per-rate power */
    rf = ral_rf_read(30);
    ral_rf_write(30, rf & ~0x06U);
    uint8_t bbp = ral_bbp_read(4);
    ral_bbp_write(4, bbp & ~0x18U);
    ral_write32(RAL_TX_BAND_CFG, RAL_TX_BAND_CFG_BG);
    ral_config_tx_pin_2g();
    ral_config_txpower(sc);

    /* Kick VCO calibration */
    rf = ral_rf_read(3);
    ral_rf_write(3, rf | RAL_RF_VCOCAL_EN);
    IODelay(1000);

    sc->sc_cur_chan = chan;
}

void ItlRal::ral_start_hw(struct ral_softc *sc)
{
    if (!sc->sc_phase2_ok || !sc->sc_dma_allocated)
        return;

    ral_setup_dma_hw(sc);
    ral_mac_init();
    ral_set_macaddr(sc->sc_mac_addr);
    ral_set_chan(sc, 1);

    /* Clear any pending interrupts, then enable RX + TX DMA + MAC RX/TX */
    ral_write32(RAL_INT_SOURCE_CSR, 0xFFFFFFFFU);
    ral_write32(RAL_WPDMA_GLO_CFG,
                RAL_TX_DMA_EN | RAL_RX_DMA_EN | RAL_WPDMA_BT_SIZE_64 | RAL_TX_WB_DDONE);
    ral_write32(RAL_RX_FILTR_CFG, RAL_RX_FILTR_SCAN);
    ral_write32(RAL_MAC_SYS_CTRL, RAL_MAC_TX_EN | RAL_MAC_RX_EN);
    ral_write32(RAL_INT_MASK_CSR, RAL_RX_DONE_INT | RAL_AC0_DMA_DONE | RAL_MGMT_DMA_DONE);

    sc->sc_running = true;
}

void ItlRal::ral_stop_hw(struct ral_softc *sc)
{
    sc->sc_running = false;
    if (fScanTimer)
        fScanTimer->cancelTimeout();
    if (sc->sc_mmio_base) {
        ral_write32(RAL_INT_MASK_CSR, 0);
        ral_write32(RAL_MAC_SYS_CTRL, 0);
        ral_write32(RAL_WPDMA_GLO_CFG, ral_read32(RAL_WPDMA_GLO_CFG) & 0xFF0U);
        ral_write32(RAL_INT_SOURCE_CSR, 0xFFFFFFFFU);
    }
}

void ItlRal::ral_record_ssid(struct ral_softc *sc, const uint8_t *frame, uint16_t len)
{
    /* 802.11 Beacon (0x80) or Probe Response (0x50): 24-byte header + 12-byte fixed params */
    if (len < 38)
        return;
    uint8_t fc0 = frame[0];
    if (fc0 != 0x80 && fc0 != 0x50)
        return;
    sc->sc_rx_beacons++;

    /* First IE at offset 36 should be Tag 0 (SSID) */
    uint16_t pos = 36;
    while (pos + 2 <= len) {
        uint8_t tag = frame[pos];
        uint8_t tlen = frame[pos + 1];
        if (pos + 2 + tlen > len)
            break;
        if (tag == 0 && tlen > 0 && tlen <= 32) {
            char ssid[33];
            uint8_t clean_len = 0;
            for (uint8_t i = 0; i < tlen; i++) {
                uint8_t c = frame[pos + 2 + i];
                if (c >= 0x20 && c <= 0x7E)
                    ssid[clean_len++] = (char)c;
            }
            ssid[clean_len] = '\0';
            if (clean_len > 0 && strnstr(sc->sc_seen_ssids, ssid, sizeof(sc->sc_seen_ssids)) == NULL) {
                size_t cur_len = strlen(sc->sc_seen_ssids);
                if (cur_len + clean_len + 3 < sizeof(sc->sc_seen_ssids)) {
                    if (cur_len > 0)
                        strlcat(sc->sc_seen_ssids, ", ", sizeof(sc->sc_seen_ssids));
                    strlcat(sc->sc_seen_ssids, ssid, sizeof(sc->sc_seen_ssids));
                }
            }
            break;
        }
        pos += (uint16_t)(2 + tlen);
    }
}

void ItlRal::ral_rx_intr(struct ral_softc *sc)
{
    struct ieee80211com *ic  = &sc->sc_ic;
    struct _ifnet       *ifp = &ic->ic_if;
    IONetworkController *ctrl = getController();

    for (uint32_t budget = 0; budget < sc->sc_rxq.count; budget++) {
        uint32_t idx = sc->sc_rxq.cur;
        struct ral_rxd *rxd = &((struct ral_rxd *)sc->sc_rxq.desc_virt)[idx];
        uint16_t sdl0 = OSSwapLittleToHostInt16(rxd->sdl0);

        if ((sdl0 & RAL_RX_DDONE) == 0)
            break;

        uint32_t flags = OSSwapLittleToHostInt32(rxd->flags);
        uint8_t *buf   = sc->sc_rxq.buf_virt[idx];

        if (flags & RAL_RX_CRCERR) {
            sc->sc_rx_crc_err++;
        } else {
            struct ral_rxwi *rxwi = (struct ral_rxwi *)buf;
            uint16_t mpdu_len = OSSwapLittleToHostInt16(rxwi->len) & 0x0FFFU;
            if (mpdu_len >= 16 && (size_t)(sizeof(struct ral_rxwi) + mpdu_len + 2) <= RAL_DMA_BUF_SIZE) {
                uint8_t *frame = buf + sizeof(struct ral_rxwi);
                sc->sc_rx_count++;
                ral_record_ssid(sc, frame, mpdu_len);

                if (ctrl) {
                    mbuf_t m = ctrl->allocatePacket(mpdu_len);
                    if (m) {
                        uint8_t *dst = (uint8_t *)mbuf_data(m);
                        if (flags & RAL_RX_L2PAD) {
                            int hdrlen = ieee80211_get_hdrlen((struct ieee80211_frame *)frame);
                            if (hdrlen > 0 && (uint16_t)hdrlen < mpdu_len &&
                                (size_t)(sizeof(struct ral_rxwi) + mpdu_len + 2) <= RAL_DMA_BUF_SIZE) {
                                memcpy(dst, frame, hdrlen);
                                memcpy(dst + hdrlen, frame + hdrlen + 2, mpdu_len - hdrlen);
                            } else {
                                memcpy(dst, frame, mpdu_len);
                            }
                        } else {
                            memcpy(dst, frame, mpdu_len);
                        }
                        mbuf_setlen(m, mpdu_len);
                        mbuf_pkthdr_setlen(m, mpdu_len);

                        struct ieee80211_frame *wh = (struct ieee80211_frame *)dst;
                        struct ieee80211_node *ni = ieee80211_find_rxnode(ic, wh);
                        struct ieee80211_rxinfo rxi;
                        memset(&rxi, 0, sizeof(rxi));
                        int rssi_dbm = -12 - (int)rxwi->rssi[0];
                        if (rssi_dbm < -100) rssi_dbm = -100;
                        if (rssi_dbm > -10)  rssi_dbm = -10;
                        sc->sc_last_rssi = rssi_dbm;
                        rxi.rxi_rssi = rssi_dbm + 100;

                        ieee80211_input(ifp, m, ni, &rxi);
                        ieee80211_release_node(ic, ni);
                    }
                }
            }
        }

        /* Hand descriptor back to hardware */
        rxd->flags = 0;
        rxd->sdl0  = OSSwapHostToLittleInt16((uint16_t)RAL_DMA_BUF_SIZE);
        sc->sc_rxq.cur = (idx + 1) % sc->sc_rxq.count;
    }

    /* Single batched MMIO write to advance RX_CALC_IDX */
    OSSynchronizeIO();
    uint32_t last_idx = (sc->sc_rxq.cur + sc->sc_rxq.count - 1) % sc->sc_rxq.count;
    ral_write32(RAL_RX_CALC_IDX, last_idx);

    if ((sc->sc_irq_count & 0x1FFU) == 0)
        ral_publish_diagnostics(sc);
}

void ItlRal::ral_tx_reclaim(struct ral_dma_ring *ring, int qid)
{
    uint32_t dtx = ral_read32(RAL_TX_DTX_IDX(qid));
    if (dtx < ring->count)
        ring->next = dtx;
}

bool ItlRal::ral_tx_has_space(struct ral_dma_ring *ring, int qid)
{
    uint32_t dtx = ral_read32(RAL_TX_DTX_IDX(qid));
    if (dtx < ring->count)
        ring->next = dtx;
    return ((ring->cur + 1) % ring->count) != ring->next;
}

int ItlRal::ral_tx_frame(struct ral_softc *sc, mbuf_t m, bool is_mgmt)
{
    if (!m || !sc->sc_running) {
        if (m) mbuf_freem(m);
        return ENETDOWN;
    }

    size_t mpdu_len = mbuf_pkthdr_len(m);
    if (mpdu_len == 0 || sizeof(struct ral_txwi) + mpdu_len > RAL_DMA_BUF_SIZE) {
        mbuf_freem(m);
        return EMSGSIZE;
    }

    struct ral_dma_ring *ring = is_mgmt ? &sc->sc_txq_mgmt : &sc->sc_txq_ac0;
    int qid = is_mgmt ? RAL_TXQ_MGMT : RAL_TXQ_AC0;

    IOInterruptState irqState = 0;
    if (fTxLock)
        irqState = IOSimpleLockLockDisableInterrupt(fTxLock);

    uint32_t dtx = ral_read32(RAL_TX_DTX_IDX(qid));
    if (dtx < ring->count)
        ring->next = dtx;

    uint32_t idx = ring->cur;
    uint32_t next_cur = (idx + 1) % ring->count;
    if (next_cur == ring->next) {
        if (fTxLock)
            IOSimpleLockUnlockEnableInterrupt(fTxLock, irqState);
        mbuf_freem(m);
        return ENOBUFS;
    }

    uint8_t *buf = ring->buf_virt[idx];
    struct ral_txwi *txwi = (struct ral_txwi *)buf;
    memset(txwi, 0, sizeof(*txwi));
    uint8_t *payload = buf + sizeof(struct ral_txwi);
    mbuf_copydata(m, 0, mpdu_len, payload);
    mbuf_freem(m);

    bool is_mcast = (mpdu_len >= 10) ? ((payload[4] & 0x01) != 0) : true;
    /* Only treat actual unencrypted Data frames (not QoS Null Data) as EAPOL/handshake */
    uint8_t fc0 = (mpdu_len >= 1) ? payload[0] : 0;
    bool is_null_data = ((fc0 & 0x0C) == 0x08) && ((fc0 & 0x40) != 0);
    bool is_eapol = (!is_mgmt && !is_mcast && !is_null_data && mpdu_len >= 2 &&
                     (sc->sc_ic.ic_flags & IEEE80211_F_RSNON) &&
                     (payload[1] & IEEE80211_FC1_PROTECTED) == 0);
    uint16_t phy = 0; /* CCK 1 Mbps default for mgmt, mcast/bcast, and WPA2 EAPOL handshake */
    uint8_t  wcid = 0xFF;
    if (!is_mgmt && !is_mcast && !is_eapol) {
        wcid = 1; /* AP WCID entry (WCID_ATTR=0 -> SW crypto preserved) */
#ifdef IEEE80211_NODE_HT
        if (sc->sc_ic.ic_bss && (sc->sc_ic.ic_bss->ni_flags & IEEE80211_NODE_HT)) {
            uint8_t mcs = sc->sc_tx_mcs;
            if (mcs > 7) mcs = 7;
            /* 802.11n HT MixMode 20 MHz with adaptive MCS + HW auto-fallback via HT_FBK_CFG0 */
            phy = RAL_PHY_HT | mcs;
        } else
#endif
        {
            /* 802.11g OFDM 54 Mbps + HW auto-fallback via LG_FBK_CFG0 */
            phy = RAL_PHY_OFDM | 0x07U;
        }
    }
    txwi->txop   = RAL_TX_TXOP_BACKOFF;
    txwi->phy    = OSSwapHostToLittleInt16(phy);
    txwi->xflags = RAL_TX_NSEQ | (is_mcast ? 0 : RAL_TX_ACK);
    txwi->wcid   = wcid;
    txwi->len    = OSSwapHostToLittleInt16((uint16_t)mpdu_len);

    struct ral_txd *txd = &((struct ral_txd *)ring->desc_virt)[idx];
    uint16_t total_len = (uint16_t)(sizeof(struct ral_txwi) + mpdu_len);
    txd->sdp0  = OSSwapHostToLittleInt32(ring->buf_phys[idx]);
    txd->sdl1  = 0;
    txd->sdp1  = 0;
    txd->flags = (uint8_t)(RAL_TX_WIV | (is_mgmt ? RAL_TX_QSEL_MGMT : RAL_TX_QSEL_EDCA));
    txd->sdl0  = OSSwapHostToLittleInt16(total_len | RAL_TX_LS0); /* DDONE=0: hand to HW */
    OSSynchronizeIO();

    ring->cur = next_cur;
    ral_write32(RAL_TX_CTX_IDX(qid), ring->cur);
    sc->sc_tx_count++;

    if (fTxLock)
        IOSimpleLockUnlockEnableInterrupt(fTxLock, irqState);

    if (!is_mgmt && (sc->sc_tx_count & 0x7FU) == 0)
        ral_ratectl_update(sc);
    return 0;
}

void ItlRal::ral_interrupt_occurred(OSObject *owner, IOInterruptEventSource *src, int count)
{
    ItlRal *self = OSDynamicCast(ItlRal, owner);
    if (!self || !self->com.sc_mmio_base || !self->com.sc_running)
        return;

    /* Mask interrupts during drain, then re-arm at exit so PCIe MSI edge never stalls */
    self->ral_write32(RAL_INT_MASK_CSR, 0);

    bool need_tx_start = false;
    for (int pass = 0; pass < 8; pass++) {
        uint32_t r = self->ral_read32(RAL_INT_SOURCE_CSR);
        if (r == 0 || r == 0xFFFFFFFFU)
            break;

        self->ral_write32(RAL_INT_SOURCE_CSR, r);
        self->com.sc_irq_count++;

        if (r & RAL_RX_DONE_INT)
            self->ral_rx_intr(&self->com);
        if (r & RAL_AC0_DMA_DONE) {
            self->ral_tx_reclaim(&self->com.sc_txq_ac0, RAL_TXQ_AC0);
            need_tx_start = true;
        }
        if (r & RAL_MGMT_DMA_DONE)
            self->ral_tx_reclaim(&self->com.sc_txq_mgmt, RAL_TXQ_MGMT);
    }

    if (need_tx_start)
        ral_start(&self->com.sc_ic.ic_if);

    if (self->com.sc_running)
        self->ral_write32(RAL_INT_MASK_CSR, RAL_RX_DONE_INT | RAL_AC0_DMA_DONE | RAL_MGMT_DMA_DONE);
}

void ItlRal::ral_scan_timer_fired(OSObject *owner, IOTimerEventSource *timer)
{
    ItlRal *self = OSDynamicCast(ItlRal, owner);
    if (!self || !self->com.sc_running)
        return;

    struct ieee80211com *ic  = &self->com.sc_ic;
    struct _ifnet       *ifp = &ic->ic_if;

    /* Update live IORegistry counters so user can inspect scan progress anytime */
    self->ral_publish_diagnostics(&self->com);

    if (ic->ic_state == IEEE80211_S_SCAN) {
        /* If no beacons heard after wrapping past ch 13, try toggling MAIN/AUX antenna (HP single-antenna wiring) */
        if (self->com.sc_cur_chan >= 13 && self->com.sc_rx_beacons == 0) {
            bool currently_main = (self->com.sc_bbp_r152 & RAL_BBP152_RX_DEFAULT_ANT) != 0;
            self->ral_set_rx_antenna(&self->com, currently_main);
        }
        ieee80211_next_scan(ifp);
        if (self->fScanTimer && ic->ic_state == IEEE80211_S_SCAN)
            self->fScanTimer->setTimeoutMS(200);
    }
}

int ItlRal::ral_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int mgt)
{
    ItlRal *self = g_ral_instance;
    if (!self)
        return 0;

    struct ral_softc *sc = &self->com;
    if (self->fScanTimer)
        self->fScanTimer->cancelTimeout();

    if (nstate == IEEE80211_S_SCAN) {
        self->ral_set_wcid(1, NULL);
        self->ral_write32(RAL_BCN_TIME_CFG, 0);
        self->ral_write32(RAL_RX_FILTR_CFG, RAL_RX_FILTR_SCAN);
        int ch = 1;
        if (ic->ic_bss && ic->ic_bss->ni_chan && ic->ic_bss->ni_chan != IEEE80211_CHAN_ANYC)
            ch = ieee80211_chan2ieee(ic, ic->ic_bss->ni_chan);
        self->ral_set_chan(sc, ch);
        if (self->fScanTimer)
            self->fScanTimer->setTimeoutMS(200);
    } else if (nstate == IEEE80211_S_AUTH || nstate == IEEE80211_S_ASSOC || nstate == IEEE80211_S_RUN) {
        if (ic->ic_bss && ic->ic_bss->ni_chan && ic->ic_bss->ni_chan != IEEE80211_CHAN_ANYC)
            self->ral_set_chan(sc, ieee80211_chan2ieee(ic, ic->ic_bss->ni_chan));
        if (nstate == IEEE80211_S_RUN && ic->ic_bss) {
            const uint8_t *b = ic->ic_bss->ni_bssid;
            self->ral_write32(RAL_MAC_BSSID_DW0,
                (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24));
            self->ral_write32(RAL_MAC_BSSID_DW1,
                (uint32_t)b[4] | ((uint32_t)b[5] << 8));
            sc->sc_tx_mcs = 5; /* Start at MCS5 (52 Mbps) and adapt 2..7 based on HW retry ratio */
            self->ral_set_wcid(1, b);
            uint32_t intval = ic->ic_bss->ni_intval ? (uint32_t)ic->ic_bss->ni_intval : 100U;
            self->ral_write32(RAL_BCN_TIME_CFG,
                ((intval << 4) & 0xFFFFU) | (1U << 16) /* TSF_TICKING */ | (1U << 17) /* TSF_SYNC STA */);
            self->ral_write32(RAL_RX_FILTR_CFG, RAL_RX_FILTR_RUN);
        }
    } else if (nstate == IEEE80211_S_INIT) {
        self->ral_set_wcid(1, NULL);
        self->ral_write32(RAL_BCN_TIME_CFG, 0);
        self->ral_write32(RAL_MAC_BSSID_DW0, 0);
        self->ral_write32(RAL_MAC_BSSID_DW1, 0);
    }

    int ret = sc->sc_newstate ? sc->sc_newstate(ic, nstate, mgt) : 0;
    self->ral_publish_diagnostics(sc);
    return ret;
}

/* -------------------------------------------------------------------------- */
/* Attach / Detach / Free                                                     */
/* -------------------------------------------------------------------------- */

bool ItlRal::ral_attach_phase1(struct ral_softc *sc, IOPCIDevice *device)
{
    memset(sc, 0, sizeof(*sc));
    g_ral_instance      = this;
    fInterruptSource    = NULL;
    fScanTimer          = NULL;
    fTxLock             = IOSimpleLockAlloc();
    sc->sc_pci_dev      = device;
    sc->sc_fwname       = "rt2860.bin";
    sc->sc_noise        = -95;
    sc->sc_mcu_ready_ms = -1;
    sc->sc_irq_index    = -1;
    sc->sc_cur_chan     = 1;
    sc->sc_tx_mcs       = 5;
    strlcpy(sc->sc_dev.dv_xname, "ral0", sizeof(sc->sc_dev.dv_xname));
    strlcpy(sc->sc_phase2_status, "pending", sizeof(sc->sc_phase2_status));

    device->setMemoryEnable(true);
    device->setBusMasterEnable(true);

    sc->sc_bar0_map = device->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
    if (!sc->sc_bar0_map)
        return false;

    sc->sc_mmio_base = (volatile uint8_t *)sc->sc_bar0_map->getVirtualAddress();
    sc->sc_mmio_len  = sc->sc_bar0_map->getLength();

    if (!ral_check_aspm_and_mmio(sc))
        return false;

    ral_write32(RAL_INT_MASK_CSR, 0);

    sc->sc_efuse_present = ral_efuse_detect(sc);
    if (sc->sc_efuse_present) {
        sc->sc_eeprom_valid = ral_read_eeprom_efuse(sc);
        if (!sc->sc_eeprom_valid)
            sc->sc_eeprom_valid = ral_read_eeprom_pci(sc);
    } else {
        sc->sc_eeprom_valid = ral_read_eeprom_pci(sc);
    }

    if (sc->sc_eeprom_valid) {
        ral_parse_eeprom(sc);
    } else {
        sc->sc_ntxchains = 1; sc->sc_nrxchains = 1;
        sc->sc_mac_addr[0] = 0x02; sc->sc_mac_addr[1] = 0x18; sc->sc_mac_addr[2] = 0x14;
        sc->sc_mac_addr[3] = 0x53; sc->sc_mac_addr[4] = 0x9F; sc->sc_mac_addr[5] = 0x01;
    }

    struct ieee80211com *ic  = &sc->sc_ic;
    struct _ifnet       *ifp = &ic->ic_if;

    memcpy(ic->ic_myaddr, sc->sc_mac_addr, ETHER_ADDR_LEN);
    ic->ic_phytype  = IEEE80211_T_OFDM;
    ic->ic_opmode   = IEEE80211_M_STA;
    ic->ic_state    = IEEE80211_S_INIT;
    ic->ic_caps     = IEEE80211_C_WEP | IEEE80211_C_RSN |
                      IEEE80211_C_SHSLOT | IEEE80211_C_SHPREAMBLE |
                      IEEE80211_C_QOS;

    /* Enable 802.11n HT (1T1R 20 MHz, SGI20, MCS 0..7 up to 72.2 Mbps PHY) */
    ic->ic_htcaps       = IEEE80211_HTCAP_SGI20 |
                          (IEEE80211_HTCAP_SMPS_DIS << IEEE80211_HTCAP_SMPS_SHIFT);
    ic->ic_htxcaps      = 0;
    ic->ic_txbfcaps     = 0;
    ic->ic_aselcaps     = 0;
    ic->ic_ampdu_params = (IEEE80211_AMPDU_PARAM_SS_4 | 0x3);
    ic->ic_sup_mcs[0]   = 0xff; /* MCS 0..7 (1 spatial stream) */
    ic->ic_tx_mcs_set   = IEEE80211_TX_MCS_SET_DEFINED;

    ic->ic_sup_rates[IEEE80211_MODE_11B] = ieee80211_std_rateset_11b;
    ic->ic_sup_rates[IEEE80211_MODE_11G] = ieee80211_std_rateset_11g;

    for (int ch = 1; ch <= 13; ch++) {
        ic->ic_channels[ch].ic_freq  = ieee80211_ieee2mhz(ch, IEEE80211_CHAN_2GHZ);
        ic->ic_channels[ch].ic_flags = IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
                                       IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ
#ifdef IEEE80211_CHAN_HT
                                       | IEEE80211_CHAN_HT
#endif
                                       ;
    }
    ic->ic_ibss_chan = &ic->ic_channels[1];
    ic->ic_max_rssi  = 100;

    ifp->if_softc    = sc;
    ifp->if_flags    = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
    ifp->if_ioctl    = ral_ioctl;
    ifp->if_start    = ral_start;
    ifp->if_watchdog = ral_watchdog;
    memcpy(ifp->if_xname, sc->sc_dev.dv_xname, IFNAMSIZ);

    taskq_init();
    if_attach(ifp);
    ieee80211_ifattach(ifp, getController());
    ieee80211_media_init(ifp);

    ic->ic_ampdu_rx_start = ral_ampdu_rx_start;
    ic->ic_ampdu_rx_stop  = ral_ampdu_rx_stop;

    sc->sc_newstate = ic->ic_newstate;
    ic->ic_newstate = ral_newstate;

    sc->sc_attached = true;
    ral_publish_diagnostics(sc);
    return true;
}

bool ItlRal::attach(IOPCIDevice *device)
{
    if (!ral_attach_phase1(&com, device)) {
        releaseAll();
        return false;
    }
    return true;
}

void ItlRal::detach(IOPCIDevice *device)
{
    ral_stop_hw(&com);
    ral_teardown_interrupts();
    ral_free_dma(&com);
    if (fTxLock) {
        IOSimpleLockFree(fTxLock);
        fTxLock = NULL;
    }
    if (com.sc_attached) {
        struct _ifnet *ifp = &com.sc_ic.ic_ac.ac_if;
        ieee80211_ifdetach(ifp);
        taskq_destroy(systq);
        com.sc_attached = false;
    }
    if (g_ral_instance == this)
        g_ral_instance = NULL;
    releaseAll();
}

void ItlRal::releaseAll()
{
    ral_teardown_interrupts();
    ral_free_dma(&com);
    if (com.sc_bar0_map) {
        com.sc_bar0_map->release();
        com.sc_bar0_map = NULL;
    }
    com.sc_mmio_base = NULL;
    com.sc_mmio_len  = 0;
    com.sc_pci_dev   = NULL;
}

void ItlRal::free()
{
    releaseAll();
    super::free();
}

/* -------------------------------------------------------------------------- */
/* ItlHalService / ItlDriverInfo / ItlDriverController Overrides              */
/* -------------------------------------------------------------------------- */

ItlDriverInfo *ItlRal::getDriverInfo()         { return this; }
ItlDriverController *ItlRal::getDriverController() { return this; }

IOReturn ItlRal::enable(IONetworkInterface *netif)
{
    if (!com.sc_phase2_ok) {
        IOReturn err = ral_init_phase2(&com);
        if (err != kIOReturnSuccess) {
            ral_publish_diagnostics(&com);
            return err;
        }
    }

    if (!ral_alloc_dma(&com)) {
        strlcpy(com.sc_phase2_status, "FAIL: DMA alloc (<4GB)", sizeof(com.sc_phase2_status));
        ral_publish_diagnostics(&com);
        return kIOReturnNoMemory;
    }

    if (!ral_setup_interrupts(&com)) {
        strlcpy(com.sc_phase2_status, "FAIL: IRQ setup", sizeof(com.sc_phase2_status));
        ral_publish_diagnostics(&com);
        return kIOReturnError;
    }

    ral_start_hw(&com);

    struct ieee80211com *ic  = &com.sc_ic;
    struct _ifnet       *ifp = &ic->ic_if;
    ifp->if_flags |= IFF_UP | IFF_RUNNING;
    ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);

    ral_publish_diagnostics(&com);
    return kIOReturnSuccess;
}

IOReturn ItlRal::disable(IONetworkInterface *netif)
{
    struct ieee80211com *ic  = &com.sc_ic;
    struct _ifnet       *ifp = &ic->ic_if;
    ieee80211_new_state(ic, IEEE80211_S_INIT, -1);
    ral_stop_hw(&com);
    ifp->if_flags &= ~(IFF_UP | IFF_RUNNING);
    ral_publish_diagnostics(&com);
    return kIOReturnSuccess;
}

void ItlRal::clearScanningFlags()                  {}
IOReturn ItlRal::setMulticastList(IOEthernetAddress *addr, int count) { return kIOReturnSuccess; }
const char *ItlRal::getFirmwareVersion()           { return "rt2860.bin (Phase 7)"; }
const char *ItlRal::getFirmwareName()              { return com.sc_fwname ? com.sc_fwname : "rt2860.bin"; }
UInt32 ItlRal::supportedFeatures()                 { return 0; }
const char *ItlRal::getFirmwareCountryCode()       { return "ZZ"; }
uint32_t ItlRal::getTxQueueSize()                  { return RAL_TX_RING_COUNT; }
int16_t ItlRal::getBSSNoise()                      { return com.sc_noise; }
bool ItlRal::is5GBandSupport()                     { return false; }
int ItlRal::getTxNSS()                             { return com.sc_ntxchains ? com.sc_ntxchains : 1; }
struct ieee80211com *ItlRal::get80211Controller()  { return &com.sc_ic; }

/* -------------------------------------------------------------------------- */
/* net80211 Callbacks                                                         */
/* -------------------------------------------------------------------------- */

int ItlRal::ral_ioctl(struct _ifnet *ifp, u_long cmd, caddr_t data)
{
    ItlRal *self = g_ral_instance;
    if (!self)
        return ENXIO;

    struct ieee80211com *ic = &self->com.sc_ic;
    int err = 0;

    switch (cmd) {
        case SIOCSIFFLAGS:
            if (ifp->if_flags & IFF_UP) {
                if (!self->com.sc_running && self->com.sc_phase2_ok && self->com.sc_dma_allocated) {
                    self->ral_start_hw(&self->com);
                    ifp->if_flags |= IFF_RUNNING;
                    ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
                }
            } else {
                if (self->com.sc_running) {
                    ieee80211_new_state(ic, IEEE80211_S_INIT, -1);
                    self->ral_stop_hw(&self->com);
                    ifp->if_flags &= ~IFF_RUNNING;
                }
            }
            break;
        default:
            err = ieee80211_ioctl(ifp, cmd, data);
            break;
    }

    if (err == ENETRESET) {
        if ((ifp->if_flags & (IFF_UP | IFF_RUNNING)) == (IFF_UP | IFF_RUNNING))
            ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
        err = 0;
    }
    return err;
}

void ItlRal::ral_start(struct _ifnet *ifp)
{
    ItlRal *self = g_ral_instance;
    if (!self || !self->com.sc_running)
        return;

    struct ieee80211com *ic = &self->com.sc_ic;
    mbuf_t m;

    /* Drain management queue first (Probe Requests, Auth, Assoc) */
    while ((m = mq_dequeue(&ic->ic_mgtq)) != NULL) {
        self->ral_tx_frame(&self->com, m, true);
    }

    /* Drain data queue (encapsulate Ethernet -> 802.11 + software WPA2 encrypt) */
    if (ic->ic_state != IEEE80211_S_RUN)
        return;

    while (self->ral_tx_has_space(&self->com.sc_txq_ac0, RAL_TXQ_AC0)) {
        m = ifq_dequeue(&ifp->if_snd);
        if (m == NULL)
            break;
        struct ieee80211_node *ni = NULL;
        m = ieee80211_encap(ifp, m, &ni);
        if (m == NULL)
            continue;
        struct ieee80211_frame *wh = (struct ieee80211_frame *)mbuf_data(m);
        if (wh->i_fc[1] & IEEE80211_FC1_PROTECTED) {
            struct ieee80211_key *k = ieee80211_get_txkey(ic, wh, ni);
            if (k)
                m = ieee80211_encrypt(ic, m, k);
        }
        int txerr = 0;
        if (m)
            txerr = self->ral_tx_frame(&self->com, m, false);
        if (ni)
            ieee80211_release_node(ic, ni);
        if (txerr == ENOBUFS)
            break;
    }
}

void ItlRal::ral_watchdog(struct _ifnet *ifp)
{
    ifp->if_timer = 0;
    ieee80211_watchdog(ifp);
}

int ItlRal::ral_ampdu_rx_start(struct ieee80211com *ic, struct ieee80211_node *ni, uint8_t tid)
{
    /* RT5390 hardware de-aggregates incoming A-MPDUs and generates HW BlockAck via AUTO_RSP_CFG */
    return 0;
}

void ItlRal::ral_ampdu_rx_stop(struct ieee80211com *ic, struct ieee80211_node *ni, uint8_t tid)
{
}

void ItlRal::ral_ratectl_update(struct ral_softc *sc)
{
    uint32_t txs0 = ral_read32(RAL_TX_STA_CNT0);
    uint32_t txs1 = ral_read32(RAL_TX_STA_CNT1);
    sc->sc_tx_hw_fail += (txs0 & 0xFFFFU);
    sc->sc_tx_hw_ok   += (txs1 & 0xFFFFU);
    sc->sc_tx_hw_rty  += ((txs1 >> 16) & 0xFFFFU);

    uint32_t dok  = sc->sc_tx_hw_ok  - sc->sc_rate_last_ok;
    uint32_t drty = sc->sc_tx_hw_rty - sc->sc_rate_last_rty;
    sc->sc_rate_last_ok  = sc->sc_tx_hw_ok;
    sc->sc_rate_last_rty = sc->sc_tx_hw_rty;

    if (dok < 16)
        return;

    /* If retries exceed successes, step down MCS (min MCS 2 = 19.5 Mbps); if <25% retries, step up */
    if (drty > (dok * 2U) && sc->sc_tx_mcs > 5)
        sc->sc_tx_mcs--;
    else if ((drty * 4U) < dok && sc->sc_tx_mcs < 7)
        sc->sc_tx_mcs++;
}
