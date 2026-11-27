/*
 * rt5390reg.h — Verified Ralink RT5390 / RT5390F / RT5390R MMIO, DMA & EEPROM definitions
 *
 * Traced directly to:
 *   - Linux drivers/net/wireless/ralink/rt2x00/rt2800.h
 *   - Linux drivers/net/wireless/ralink/rt2x00/rt2800pci.c
 *   - Linux drivers/net/wireless/ralink/rt2x00/rt2800lib.c
 *   - OpenBSD sys/dev/ic/rt2860reg.h & rt2860.c
 */

#ifndef RAL_RT5390REG_H
#define RAL_RT5390REG_H

#include <stdint.h>

/* PCI Identifiers */
#define PCI_VENDOR_RALINK           0x1814
#define PCI_PRODUCT_RALINK_RT5390   0x5390
#define PCI_PRODUCT_RALINK_RT539F   0x539F

/* PCIe Capability Registers */
#define RAL_PCI_CAP_ID_EXP          0x10
#define RAL_PCIE_LNKCTL_OFF         0x10
#define RAL_PCIE_LNKCTL_ASPM_MASK   0x0003  /* Bit 0: L0s, Bit 1: L1 */

/* PCI Configuration / Clock Registers */
#define RAL_PCI_CFG                 0x0000
#define RAL_PCI_CFG_PCI             (1U << 16)
#define RAL_OPT_14                  0x0114

/* Interrupt & WPDMA Registers (Linux rt2800.h / OpenBSD rt2860reg.h) */
#define RAL_INT_SOURCE_CSR          0x0200
#define RAL_INT_MASK_CSR            0x0204

#define RAL_RX_DELAY_INT            (1U << 0)
#define RAL_TX_DELAY_INT            (1U << 1)
#define RAL_RX_DONE_INT             (1U << 2)
#define RAL_AC0_DMA_DONE            (1U << 3)
#define RAL_AC1_DMA_DONE            (1U << 4)
#define RAL_AC2_DMA_DONE            (1U << 5)
#define RAL_AC3_DMA_DONE            (1U << 6)
#define RAL_HCCA_DMA_DONE           (1U << 7)
#define RAL_MGMT_DMA_DONE           (1U << 8)
#define RAL_MCU_CMD_INT             (1U << 9)
#define RAL_TX_RX_COHERENT          (1U << 10)
#define RAL_TX_STAT_INT             (1U << 12)

#define RAL_WPDMA_GLO_CFG           0x0208
#define RAL_TX_DMA_EN               (1U << 0)
#define RAL_TX_DMA_BUSY             (1U << 1)
#define RAL_RX_DMA_EN               (1U << 2)
#define RAL_RX_DMA_BUSY             (1U << 3)
#define RAL_WPDMA_BT_SIZE_64        (2U << 4)
#define RAL_TX_WB_DDONE             (1U << 6)

#define RAL_WPDMA_RST_IDX           0x020C
#define RAL_DELAY_INT_CFG           0x0210

/* WMM / EDCA DMA Scheduler Registers */
#define RAL_WMM_AIFSN_CFG           0x0214
#define RAL_WMM_CWMIN_CFG           0x0218
#define RAL_WMM_CWMAX_CFG           0x021C
#define RAL_WMM_TXOP0_CFG           0x0220
#define RAL_WMM_TXOP1_CFG           0x0224

/* TX Ring Registers: qid 0..3 = AC0..AC3, 4 = HCCA, 5 = MGMT */
#define RAL_TX_BASE_PTR(qid)        (0x0230U + (uint32_t)(qid) * 0x10U)
#define RAL_TX_MAX_CNT(qid)         (0x0234U + (uint32_t)(qid) * 0x10U)
#define RAL_TX_CTX_IDX(qid)         (0x0238U + (uint32_t)(qid) * 0x10U)
#define RAL_TX_DTX_IDX(qid)         (0x023CU + (uint32_t)(qid) * 0x10U)

#define RAL_TXQ_AC0                 0
#define RAL_TXQ_MGMT                5

/* RX Ring Registers */
#define RAL_RX_BASE_PTR             0x0290
#define RAL_RX_MAX_CNT              0x0294
#define RAL_RX_CALC_IDX             0x0298
#define RAL_RX_DRX_IDX              0x029C

#define RAL_US_CYC_CNT              0x02A4

/* PBF / MCU / Firmware Registers */
#define RAL_SYS_CTRL                0x0400
#define RAL_MCU_RESET               (1U << 0)
#define RAL_MCU_READY               (1U << 7)
#define RAL_HST_PM_SEL              (1U << 16)
#define RAL_HOST_CMD                0x0404
#define RAL_PBF_CFG                 0x0408
#define RAL_PBF_MAX_PCNT            0x040C
#define RAL_BCN_OFFSET0             0x042C
#define RAL_BCN_OFFSET1             0x0430
#define RAL_FW_BASE                 0x2000
#define RAL_FW_SIZE                 8192

/* H2M Mailbox & BBP Agent */
#define RAL_H2M_MAILBOX             0x7010
#define RAL_H2M_MAILBOX_CID         0x7014
#define RAL_H2M_MAILBOX_STATUS      0x701C
#define RAL_H2M_BBPAGENT            0x7028
#define RAL_H2M_BUSY                (1U << 24)
#define RAL_TOKEN_NO_INTR           0xFFU

#define RAL_MCU_CMD_WAKEUP          0x31
#define RAL_MCU_CMD_LED1            0x52
#define RAL_MCU_CMD_LED2            0x53
#define RAL_MCU_CMD_LED3            0x54
#define RAL_MCU_CMD_RFRESET         0x72
#define RAL_MCU_CMD_BBP             0x80

#define RAL_BBP_CSR_READ            (1U << 16)
#define RAL_BBP_CSR_KICK            (1U << 17)
#define RAL_BBP_RW_PARALLEL         (1U << 19)

/* RF CSR Interface (RT3070 / RT5390 shared layout) */
#define RAL_RF_CSR_CFG              0x0500
#define RAL_RF_WRITE                (1U << 16)
#define RAL_RF_KICK                 (1U << 17)

/* RT5390 RF / BBP Control Bits */
#define RAL_RF_RESCAL               (1U << 7)   /* RF R2 bit 7 */
#define RAL_RF_VCOCAL_EN            (1U << 7)   /* RF R3 bit 7 */
#define RAL_RF_RX_LO1               (1U << 5)   /* RF R38 bit 5 */
#define RAL_RF_RX_LO2               (1U << 7)   /* RF R39 bit 7 */
#define RAL_BBP_MAC_IF_CTRL         (1U << 6)   /* BBP R4 bit 6 */

/* eFuse Control & Data Registers (RT30x0 / RT5390) */
#define RAL_EFUSE_CTRL              0x0580
#define RAL_EFUSE_CTRL_MODE_MASK    0x000000C0U
#define RAL_EFUSE_CTRL_AIN_MASK     0x03FE0000U
#define RAL_EFUSE_CTRL_AIN_SHIFT    17
#define RAL_EFUSE_CTRL_KICK         0x40000000U
#define RAL_EFUSE_CTRL_PRESENT      0x80000000U

#define RAL_EFUSE_DATA0             0x0590
#define RAL_EFUSE_DATA1             0x0594
#define RAL_EFUSE_DATA2             0x0598
#define RAL_EFUSE_DATA3             0x059C

/* PCI Serial EEPROM Control Register */
#define RAL_E2PROM_CSR              0x0004
#define RAL_E2PROM_CSR_DATA_CLOCK   0x00000001U
#define RAL_E2PROM_CSR_CHIP_SELECT  0x00000002U
#define RAL_E2PROM_CSR_DATA_IN      0x00000004U
#define RAL_E2PROM_CSR_DATA_OUT     0x00000008U
#define RAL_E2PROM_CSR_TYPE_MASK    0x00000030U
#define RAL_E2PROM_CSR_TYPE_SHIFT   4

/* ASIC Version & MAC Registers (Verified exact offsets from Linux rt2800.h) */
#define RAL_MAC_CSR0                0x1000
#define RAL_MAC_CSR0_CHIPSET_SHIFT  16
#define RAL_MAC_CSR0_CHIPSET_MASK   0xFFFF0000U
#define RAL_MAC_CSR0_REV_MASK       0x0000FFFFU

#define RAL_MAC_SYS_CTRL            0x1004
#define RAL_MAC_SRST                (1U << 0)
#define RAL_BBP_HRST                (1U << 1)
#define RAL_MAC_TX_EN               (1U << 2)
#define RAL_MAC_RX_EN               (1U << 3)

#define RAL_MAC_ADDR_DW0            0x1008
#define RAL_MAC_ADDR_DW1            0x100C
#define RAL_MAC_BSSID_DW0           0x1010
#define RAL_MAC_BSSID_DW1           0x1014
#define RAL_MAX_LEN_CFG             0x1018
#define RAL_LED_CFG                 0x102C

#define RAL_XIFS_TIME_CFG           0x1100
#define RAL_BKOFF_SLOT_CFG          0x1104
#define RAL_BCN_TIME_CFG            0x1114

#define RAL_MAC_STATUS_REG          0x1200
#define RAL_TX_STATUS_BUSY          (1U << 0)
#define RAL_RX_STATUS_BUSY          (1U << 1)

#define RAL_PWR_PIN_CFG             0x1204
#define RAL_IO_RF_PE                (1U << 0)
#define RAL_IO_RA_PE                (1U << 1)
#define RAL_MIMO_PS_CFG             0x1210

/* MAC EDCA, TX Power, Pin, Timing & Fallback Registers (Verified Linux rt2800.h) */
#define RAL_EDCA_AC0_CFG            0x1300
#define RAL_EDCA_AC1_CFG            0x1304
#define RAL_EDCA_AC2_CFG            0x1308
#define RAL_EDCA_AC3_CFG            0x130C

#define RAL_TX_PWR_CFG_0            0x1314
#define RAL_TX_PWR_CFG_1            0x1318
#define RAL_TX_PWR_CFG_2            0x131C
#define RAL_TX_PWR_CFG_3            0x1320
#define RAL_TX_PWR_CFG_4            0x1324

#define RAL_TX_PIN_CFG              0x1328
#define RAL_TX_PIN_PA_PE_A0_EN      (1U << 0)
#define RAL_TX_PIN_PA_PE_G0_EN      (1U << 1)
#define RAL_TX_PIN_PA_PE_A1_EN      (1U << 2)
#define RAL_TX_PIN_PA_PE_G1_EN      (1U << 3)
#define RAL_TX_PIN_LNA_PE_A0_EN     (1U << 8)
#define RAL_TX_PIN_LNA_PE_G0_EN     (1U << 9)
#define RAL_TX_PIN_LNA_PE_A1_EN     (1U << 10)
#define RAL_TX_PIN_LNA_PE_G1_EN     (1U << 11)
#define RAL_TX_PIN_RFTR_EN          (1U << 16)
#define RAL_TX_PIN_TRSW_EN          (1U << 18)

#define RAL_TX_BAND_CFG             0x132C
#define RAL_TX_BAND_CFG_HT40_MINUS  (1U << 0)
#define RAL_TX_BAND_CFG_A           (1U << 1)
#define RAL_TX_BAND_CFG_BG          (1U << 2)

#define RAL_TX_SW_CFG0              0x1330
#define RAL_TX_SW_CFG1              0x1334
#define RAL_TX_SW_CFG2              0x1338

#define RAL_TXOP_CTRL_CFG           0x1340
#define RAL_TX_RTS_CFG              0x1344
#define RAL_TX_TIMEOUT_CFG          0x1348
#define RAL_TX_RTY_CFG              0x134C
#define RAL_TX_LINK_CFG             0x1350
#define RAL_HT_FBK_CFG0             0x1354
#define RAL_HT_FBK_CFG1             0x1358
#define RAL_LG_FBK_CFG0             0x135C
#define RAL_LG_FBK_CFG1             0x1360
#define RAL_CCK_PROT_CFG            0x1364
#define RAL_OFDM_PROT_CFG           0x1368
#define RAL_MM20_PROT_CFG           0x136C
#define RAL_MM40_PROT_CFG           0x1370
#define RAL_GF20_PROT_CFG           0x1374
#define RAL_GF40_PROT_CFG           0x1378
#define RAL_EXP_ACK_TIME            0x1380

/* RX Filter & Hardware Auto-Responder (ACK/CTS/BA) Registers (Verified Linux rt2800.h) */
#define RAL_RX_FILTR_CFG            0x1400
#define RAL_RX_FILTR_SCAN           0x00017F93U /* Allow beacons, probe resp, broadcast; drop CRC/PHY err */
#define RAL_RX_FILTR_RUN            0x00017F9FU /* Drop not-to-me, other BSSIDs & HW duplicates when connected */
#define RAL_AUTO_RSP_CFG            0x1404
#define RAL_LEGACY_BASIC_RATE       0x1408
#define RAL_HT_BASIC_RATE           0x140C
#define RAL_TXOP_HLDR_ET            0x1608

/* RX / TX Hardware Statistics Counters */
#define RAL_RX_STA_CNT0             0x1700
#define RAL_RX_STA_CNT1             0x1704
#define RAL_RX_STA_CNT2             0x1708
#define RAL_TX_STA_CNT0             0x170C
#define RAL_TX_STA_CNT1             0x1710
#define RAL_TX_STA_CNT2             0x1714

/* WCID & Security Table Bases */
#define RAL_MAC_WCID_BASE           0x1800
#define RAL_MAC_WCID_ATTR_BASE      0x6800

/* Verified RT5390 silicon revisions */
#define RAL_CHIP_RT5390             0x5390
#define RAL_REV_RT5390F             0x0502
#define RAL_REV_RT5370G             0x0503
#define RAL_REV_RT5390R             0x1502

/* EEPROM Raw 16-bit Word Offsets */
#define RAL_EEPROM_SIZE             0x0200
#define RAL_EEPROM_WORDS            (RAL_EEPROM_SIZE / 2)

#define RAL_EEPROM_CHIP_ID          0x00
#define RAL_EEPROM_VERSION          0x01
#define RAL_EEPROM_MAC_ADDR_0       0x02
#define RAL_EEPROM_MAC_ADDR_1       0x03
#define RAL_EEPROM_MAC_ADDR_2       0x04
#define RAL_EEPROM_NIC_CONF0        0x1A
#define RAL_EEPROM_NIC_CONF1        0x1B
#define RAL_EEPROM_FREQ_LED         0x1D
#define RAL_EEPROM_NIC_CONF2        0x21
#define RAL_EEPROM_TXPOW1_2G        0x29        /* Per-channel 2.4GHz TX power (ch 1..14, 1 byte each) */
#define RAL_EEPROM_RPWR             0x6F        /* Per-rate 20MHz TX power (9 words: 0x6F..0x77 / byte 0xDE..0xEE) */

#define RAL_NIC_CONF0_RXPATH_MASK   0x000F
#define RAL_NIC_CONF0_TXPATH_MASK   0x00F0
#define RAL_NIC_CONF0_TXPATH_SHIFT  4
#define RAL_NIC_CONF0_RFTYPE_MASK   0x0F00
#define RAL_NIC_CONF0_RFTYPE_SHIFT  8

#define RAL_NIC_CONF1_HW_RADIO      0x0001
#define RAL_NIC_CONF1_EXT_TX_ALC    0x0002
#define RAL_NIC_CONF1_EXT_LNA_2G    0x0004
#define RAL_NIC_CONF1_EXT_LNA_5G    0x0008
#define RAL_NIC_CONF1_ANT_DIV_MASK  0x1800
#define RAL_NIC_CONF1_ANT_DIV_SHIFT 11
#define RAL_NIC_CONF1_INT_TX_ALC    0x2000
#define RAL_NIC_CONF1_BT_COEXIST    0x4000

#define RAL_ANT_DIV_DISABLE         0
#define RAL_ANT_DIV_DIVERSITY       1
#define RAL_ANT_DIV_FIXED_MAIN      2
#define RAL_ANT_DIV_FIXED_AUX       3

#define RAL_BBP152_RX_DEFAULT_ANT   0x80
#define RAL_NOHWCRYPT               1

/* -------------------------------------------------------------------------- */
/* DMA Descriptor & Wireless Info Structures (Linux rt2800.h / OpenBSD)       */
/* -------------------------------------------------------------------------- */

#define RAL_RX_RING_COUNT           256
#define RAL_TX_RING_COUNT           128
#define RAL_DMA_BUF_SIZE            4096

/* RXD sdl0 flags */
#define RAL_RX_DDONE                (1U << 15)
#define RAL_RX_LS0                  (1U << 14)

/* RXD word 3 flags (OpenBSD rt2860reg.h exact bit positions) */
#define RAL_RX_CRCERR               (1U << 8)
#define RAL_RX_ICVERR               (1U << 9)
#define RAL_RX_MICERR               (1U << 10)
#define RAL_RX_L2PAD                (1U << 14)

/* TXD sdl0 flags */
#define RAL_TX_DDONE                (1U << 15)
#define RAL_TX_LS0                  (1U << 14)

/* TXD word 3 high byte (flags) */
#define RAL_TX_WIV                  (1U << 0)
#define RAL_TX_QSEL_MGMT            (0U << 1)
#define RAL_TX_QSEL_HCCA            (1U << 1)
#define RAL_TX_QSEL_EDCA            (2U << 1)

/* TXWI txop / phy / xflags (Verified Linux rt2800.h) */
#define RAL_TX_TXOP_HT              0
#define RAL_TX_TXOP_PIFS            1
#define RAL_TX_TXOP_BACKOFF         2           /* 2 = Backoff (NOT 3 which is SIFS!) */
#define RAL_TX_TXOP_SIFS            3

#define RAL_PHY_CCK                 (0U << 14)
#define RAL_PHY_OFDM                (1U << 14)
#define RAL_PHY_HT                  (2U << 14)
#define RAL_PHY_HT_GF               (3U << 14)
#define RAL_PHY_SGI                 (1U << 8)
#define RAL_PHY_BW40                (1U << 7)

#define RAL_TX_ACK                  (1U << 0)
#define RAL_TX_NSEQ                 (1U << 1)

struct ral_rxd {
    uint32_t sdp0;
    uint16_t sdl1;      /* Word 1 low  (bits 15:0) */
    uint16_t sdl0;      /* Word 1 high (bits 31:16): bit15=DDONE, bit14=LS0, bits13:0=len */
    uint32_t sdp1;
    uint32_t flags;
} __attribute__((packed));

struct ral_rxwi {
    uint8_t  wcid;
    uint8_t  keyidx;
    uint16_t len;       /* bits [11:0] = MPDU length */
    uint16_t seq;
    uint16_t phy;
    uint8_t  rssi[3];
    uint8_t  reserved1;
    uint8_t  snr[2];
    uint16_t reserved2;
} __attribute__((packed));

struct ral_txd {
    uint32_t sdp0;
    uint16_t sdl1;      /* Word 1 low  (bits 15:0) */
    uint16_t sdl0;      /* Word 1 high (bits 31:16): bit15=DDONE, bit14=LS0, bits13:0=len */
    uint32_t sdp1;
    uint8_t  reserved[3];
    uint8_t  flags;
} __attribute__((packed));

struct ral_txwi {
    uint8_t  flags;
    uint8_t  txop;
    uint16_t phy;
    uint8_t  xflags;
    uint8_t  wcid;
    uint16_t len;       /* bits [11:0] = MPDU length */
    uint32_t iv;
    uint32_t eiv;
} __attribute__((packed));

#endif /* RAL_RT5390REG_H */
