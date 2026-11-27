/*
 * ItlRal.hpp — Ralink RT5390F/RT5390R (0x1814:0x539F) HAL backend for itlwm
 *
 * Phase 1: PCI Probe, BAR0 Mapping, ASPM Guard, MAC_CSR0 Revision, eFuse/EEPROM.
 * Phase 2: 8051 Firmware Upload (rt2860.bin), MAC/BBP/RF5390 Init, Antenna Select.
 * Phase 3-4: 32-bit DMA RX/TX Rings, MSI IRQ, Channel Switching, WPA2-PSK Data Path.
 * Phase 5: Fixed MMIO Register Offsets (Auto-ACK 0x1404 / Basic Rates 0x1408 / Retry 0x134C),
 *          Safe TX_PIN_CFG RMW, EEPROM TX_PWR_CFG_0..4, WCID AP Entry, 802.11n HT (MCS 0..7).
 */

#ifndef ITL_RAL_HPP
#define ITL_RAL_HPP

#include <compat.h>
#include <HAL/ItlHalService.hpp>
#include <HAL/ItlDriverInfo.hpp>
#include <HAL/ItlDriverController.hpp>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOWorkLoop.h>

#include "rt5390reg.h"

struct ral_dma_ring {
    IOBufferMemoryDescriptor *desc_mem;
    void                     *desc_virt;
    uint32_t                  desc_phys;
    IOBufferMemoryDescriptor *buf_mem[RAL_RX_RING_COUNT];
    uint8_t                  *buf_virt[RAL_RX_RING_COUNT];
    uint32_t                  buf_phys[RAL_RX_RING_COUNT];
    uint32_t                  cur;
    uint32_t                  next;
    uint32_t                  count;
};

struct ral_softc {
    struct device           sc_dev;
    struct ieee80211com     sc_ic;

    IOPCIDevice            *sc_pci_dev;
    IOMemoryMap            *sc_bar0_map;
    volatile uint8_t       *sc_mmio_base;
    IOByteCount             sc_mmio_len;

    /* PCIe capability & ASPM tracking */
    uint8_t                 sc_pcie_cap_off;
    uint16_t                sc_aspm_orig;
    bool                    sc_aspm_cleared;

    /* Silicon identification (MAC_CSR0 = 0x1000) */
    uint32_t                sc_mac_csr0;
    uint16_t                sc_chipset;
    uint16_t                sc_rev;

    /* EEPROM / eFuse state */
    bool                    sc_efuse_present;
    bool                    sc_eeprom_valid;
    uint16_t                sc_eeprom[RAL_EEPROM_WORDS];

    /* Parsed hardware capabilities */
    uint8_t                 sc_mac_addr[6];
    uint16_t                sc_nic_conf0;
    uint16_t                sc_nic_conf1;
    uint8_t                 sc_ntxchains;
    uint8_t                 sc_nrxchains;
    uint8_t                 sc_rf_type;
    uint8_t                 sc_ant_div_mode;
    bool                    sc_hw_radio;
    bool                    sc_ext_lna_2g;
    bool                    sc_int_tx_alc;

    /* Phase 2 hardware bring-up telemetry */
    bool                    sc_phase2_attempted;
    bool                    sc_phase2_ok;
    int                     sc_mcu_ready_ms;
    uint8_t                 sc_bbp_r0;
    uint8_t                 sc_bbp_r152;
    uint8_t                 sc_rf_r1;
    uint8_t                 sc_rf_r2;
    uint8_t                 sc_rf_r3;
    char                    sc_phase2_status[96];

    /* Phase 3-5 DMA, Interrupts, Channel & Scan state */
    struct ral_dma_ring     sc_rxq;
    struct ral_dma_ring     sc_txq_ac0;
    struct ral_dma_ring     sc_txq_mgmt;
    bool                    sc_dma_allocated;
    bool                    sc_running;
    int                     sc_irq_index;
    int                     sc_cur_chan;
    int                     (*sc_newstate)(struct ieee80211com *, enum ieee80211_state, int);

    /* Live counters & discovered SSIDs */
    uint32_t                sc_irq_count;
    uint32_t                sc_rx_count;
    uint32_t                sc_rx_beacons;
    uint32_t                sc_rx_crc_err;
    uint32_t                sc_tx_count;
    uint32_t                sc_tx_hw_ok;
    uint32_t                sc_tx_hw_rty;
    uint32_t                sc_tx_hw_fail;
    uint8_t                 sc_tx_mcs;
    uint32_t                sc_rate_last_ok;
    uint32_t                sc_rate_last_rty;
    int                     sc_last_rssi;
    char                    sc_seen_ssids[160];

    /* Driver status */
    bool                    sc_attached;
    int16_t                 sc_noise;
    const char             *sc_fwname;
};

class ItlRal : public ItlHalService, public ItlDriverInfo, public ItlDriverController {
    OSDeclareDefaultStructors(ItlRal)

public:
    static bool ral_match(IOPCIDevice *device);

    /* ItlHalService lifecycle */
    virtual bool attach(IOPCIDevice *device) override;
    virtual void detach(IOPCIDevice *device) override;
    virtual void free() override;

    virtual ItlDriverInfo *getDriverInfo() override;
    virtual ItlDriverController *getDriverController() override;

    /* ItlDriverController overrides */
    virtual IOReturn enable(IONetworkInterface *netif) override;
    virtual IOReturn disable(IONetworkInterface *netif) override;
    virtual void clearScanningFlags() override;
    virtual IOReturn setMulticastList(IOEthernetAddress *addr, int count) override;

    /* ItlDriverInfo overrides */
    virtual const char *getFirmwareVersion() override;
    virtual const char *getFirmwareName() override;
    virtual UInt32 supportedFeatures() override;
    virtual const char *getFirmwareCountryCode() override;
    virtual uint32_t getTxQueueSize() override;
    virtual int16_t getBSSNoise() override;
    virtual bool is5GBandSupport() override;
    virtual int getTxNSS() override;
    virtual struct ieee80211com *get80211Controller() override;

private:
    struct ral_softc        com;
    IOInterruptEventSource *fInterruptSource;
    IOTimerEventSource     *fScanTimer;
    IOSimpleLock           *fTxLock;

    /* MMIO helpers */
    uint32_t ral_read32(uint32_t reg);
    void     ral_write32(uint32_t reg, uint32_t val);

    /* Phase 1 */
    bool     ral_attach_phase1(struct ral_softc *sc, IOPCIDevice *device);
    uint8_t  ral_find_pci_cap(IOPCIDevice *device, uint8_t cap_id);
    bool     ral_check_aspm_and_mmio(struct ral_softc *sc);
    const char *ral_rev_name(uint16_t chipset, uint16_t rev);
    const char *ral_ant_div_name(uint8_t mode);
    bool     ral_efuse_detect(struct ral_softc *sc);
    bool     ral_efuse_read_block(struct ral_softc *sc, uint16_t word_base, uint16_t out_words[8]);
    bool     ral_read_eeprom_efuse(struct ral_softc *sc);
    void     ral_eeprom_pulse_clk(uint32_t base);
    uint16_t ral_eeprom_93cx6_read_word(struct ral_softc *sc, uint16_t word_addr, int addr_bits);
    bool     ral_read_eeprom_pci(struct ral_softc *sc);
    void     ral_parse_eeprom(struct ral_softc *sc);

    /* Phase 2 */
    IOReturn ral_init_phase2(struct ral_softc *sc);
    IOReturn ral_fw_upload(struct ral_softc *sc);
    IOReturn ral_wait_dma_idle(const char *ctx);
    IOReturn ral_wait_mac_idle();
    void     ral_set_macaddr(const uint8_t *addr);
    void     ral_mac_init();
    int      ral_mcu_cmd(uint8_t cmd, uint16_t arg, bool wait);
    void     ral_bbp_write(uint8_t reg, uint8_t val);
    uint8_t  ral_bbp_read(uint8_t reg);
    void     ral_rf_write(uint8_t reg, uint8_t val);
    uint8_t  ral_rf_read(uint8_t reg);
    bool     ral_bbp_init(struct ral_softc *sc);
    void     ral_rf_init(struct ral_softc *sc);
    void     ral_rf_rev_patch(struct ral_softc *sc);
    void     ral_set_rx_antenna(struct ral_softc *sc, bool aux);

    /* Phase 3-5: DMA, IRQ, Channel/TX Power/Pin, WCID, RX/TX & HT */
    bool     ral_alloc_ring(struct ral_dma_ring *ring, uint32_t count, bool is_rx);
    void     ral_free_ring(struct ral_dma_ring *ring);
    bool     ral_alloc_dma(struct ral_softc *sc);
    void     ral_free_dma(struct ral_softc *sc);
    void     ral_setup_dma_hw(struct ral_softc *sc);
    bool     ral_setup_interrupts(struct ral_softc *sc);
    void     ral_teardown_interrupts();
    void     ral_config_txpower(struct ral_softc *sc);
    void     ral_config_tx_pin_2g();
    void     ral_set_wcid(uint8_t wcid, const uint8_t *mac);
    void     ral_set_chan(struct ral_softc *sc, int chan);
    void     ral_start_hw(struct ral_softc *sc);
    void     ral_stop_hw(struct ral_softc *sc);
    void     ral_rx_intr(struct ral_softc *sc);
    void     ral_tx_reclaim(struct ral_dma_ring *ring, int qid);
    bool     ral_tx_has_space(struct ral_dma_ring *ring, int qid);
    int      ral_tx_frame(struct ral_softc *sc, mbuf_t m, bool is_mgmt);
    void     ral_record_ssid(struct ral_softc *sc, const uint8_t *frame, uint16_t len);

    /* Static IOKit & net80211 callbacks */
    static void ral_interrupt_occurred(OSObject *owner, IOInterruptEventSource *src, int count);
    static void ral_scan_timer_fired(OSObject *owner, IOTimerEventSource *timer);
    static int  ral_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int mgt);
    static int  ral_ioctl(struct _ifnet *ifp, u_long cmd, caddr_t data);
    static void ral_start(struct _ifnet *ifp);
    static void ral_watchdog(struct _ifnet *ifp);
    static int  ral_ampdu_rx_start(struct ieee80211com *ic, struct ieee80211_node *ni, uint8_t tid);
    static void ral_ampdu_rx_stop(struct ieee80211com *ic, struct ieee80211_node *ni, uint8_t tid);
    void        ral_ratectl_update(struct ral_softc *sc);

    /* IORegistry telemetry */
    void     ral_publish_diagnostics(struct ral_softc *sc);
    void     releaseAll();
};

#endif /* ITL_RAL_HPP */
