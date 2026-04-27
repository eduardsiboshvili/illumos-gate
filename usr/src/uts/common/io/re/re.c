/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License (the "License").
 * You may not use this file except in compliance with the License.
 *
 * You can obtain a copy of the license at usr/src/OPENSOLARIS.LICENSE
 * or http://www.opensolaris.org/os/licensing.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file at usr/src/OPENSOLARIS.LICENSE.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2026 Eduard Siboshvili <edsiboshvili@gmail.com>
 * Use is subject to license terms.
 */

/*
 * Realtek RTL8125/RTL8126/RTL8127 Ethernet driver
 * ------------------------------------------------
 *
 * This is an illumos GLD/mac_provider driver for modern Realtek PCIe
 * Ethernet controllers, starting with RTL8125-class devices.  Older
 * RTL8169/RTL8111-style controllers are intentionally outside of the
 * design scope.
 *
 *
 * File layout
 * -----------
 *
 *   re.c          attach/detach, start/stop lifecycle, reset task,
 *                 watchdog, suspend/resume and module entry points.
 *
 *   re_gld.c      GLD/mac_provider callbacks, MAC properties,
 *                 advertised link capabilities, promiscuous/multicast
 *                 handling and MAC statistics.
 *
 *   re_ring.c     RX/TX descriptor rings, DMA buffers, packet receive,
 *                 transmit, reclaim and datapath resource lifetime.
 *
 *   re_intr.c     MSI-X/MSI allocation, interrupt handlers, interrupt
 *                 masking, interrupt moderation and timer interrupt mode.
 *
 *   re_chip.c     chip identification, MAC version handling and high-level
 *                 chip initialization.
 *
 *   if_re_hw.c    low-level register, OCP, MAC and PHY programming helpers.
 *
 *   re_kstat.c    driver kstats and debug counters.
 *
 *   re_sensor.c   temperature sensor support.
 *
 *   re_rss.c      RSS key/table setup and receive hash configuration.
 *
 *
 * Supported hardware policy
 * -------------------------
 *
 *   The driver targets RTL8125 and newer Realtek PCIe Ethernet controllers,
 *   including RTL8125, RTL8126 and RTL8127 families.
 *
 *   New PCI IDs must not be treated as "same chip with a new ID" without
 *   checking the MAC version.  Realtek revisions may differ in:
 *
 *     - RX descriptor layout;
 *     - checksum status bits;
 *     - PHY initialization sequence;
 *     - interrupt table layout;
 *     - DMA quirks;
 *     - offload behavior;
 *     - power-management sensitivity.
 *
 *
 * Driver lifetime model
 * ---------------------
 *
 *   attach:
 *     - allocate softc;
 *     - setup PCI config access;
 *     - map MMIO registers;
 *     - identify chip revision;
 *     - initialize software defaults;
 *     - allocate persistent ring structures;
 *     - allocate interrupt resources;
 *     - initialize hardware enough to read permanent MAC address;
 *     - register with MAC framework.
 *
 *   mac_start:
 *     - allocate runtime RX/TX DMA buffers;
 *     - program descriptor rings;
 *     - initialize MAC/PHY datapath;
 *     - enable RX/TX;
 *     - enable interrupt delivery.
 *
 *   mac_stop:
 *     - disable interrupt delivery;
 *     - stop RX/TX DMA activity;
 *     - stop the datapath;
 *     - free runtime RX/TX DMA buffers.
 *
 *   detach:
 *     - unregister from MAC framework;
 *     - destroy sensors and kstats;
 *     - free interrupts;
 *     - free rings;
 *     - unmap MMIO;
 *     - release PCI config handle;
 *     - free softc.
 *
 *
 * PCI configuration notes
 * -----------------------
 *
 *   RTL8125/RTL8126/RTL8127 controllers are sensitive to unnecessary PCIe
 *   configuration-space changes after firmware/BIOS handoff.
 *
 *   Avoid changing PCIe power-management state, ASPM policy, clock request
 *   behavior or link-power configuration from the driver unless a specific
 *   chip/platform quirk has been validated.
 *
 *   Incorrect PCIe power-state changes may cause:
 *
 *     - lost MMIO access;
 *     - DMA stalls;
 *     - link flap after resume;
 *     - TX/RX lockups;
 *     - failure to leave low-power state;
 *     - watchdog resets.
 *
 *   ASPM policy should normally be controlled by BIOS/UEFI setup.  If a
 *   platform shows unexplained interrupt loss, packet stalls or link
 *   instability, disable ASPM in BIOS first and retest before adding driver
 *   workarounds.
 *
 *
 * Interrupt architecture
 * ----------------------
 *
 *   The driver supports MSI-X and MSI.
 *
 *   MSI-X is preferred for multiqueue operation.  MSI is used as fallback
 *   and as the single-vector path for timer interrupt mode.
 *
 *   Realtek MSI-X entries are hardware-defined event slots, not a simple
 *   "vector == queue index" array.  Do not derive interrupt numbers from
 *   ring indexes.
 *
 *   RTL8125B example:
 *
 *        +---------------- RTL8125B MSI-X table ----------------+
 *        |                                                       |
 *        |   [0]   RX queue 0 interrupt                         |
 *        |   [1]   RX queue 1 interrupt                         |
 *        |   [2]   RX queue 2 interrupt                         |
 *        |   [3]   RX queue 3 interrupt                         |
 *        |                                                       |
 *        |   [16]  TX queue 0 interrupt                         |
 *        |   [18]  TX queue 1 interrupt                         |
 *        |                                                       |
 *        |   [21]  Link change / misc interrupt                 |
 *        |                                                       |
 *        +-------------------------------------------------------+
 *
 *   RX path:
 *
 *        RXQ0 ---> MSI-X[0] ----+
 *        RXQ1 ---> MSI-X[1] ----+
 *        RXQ2 ---> MSI-X[2] ----+---> RX interrupt handler
 *        RXQ3 ---> MSI-X[3] ----+
 *
 *   TX path:
 *
 *        TXQ0 ---> MSI-X[16] ------> TX interrupt handler
 *        TXQ1 ---> MSI-X[18] ------> TX interrupt handler
 *
 *   Link path:
 *
 *        link/misc event ---> MSI-X[21] ---> link interrupt handler
 *
 *   MSI fallback:
 *
 *        +-------------------------------+
 *        | Realtek device                |
 *        |                               |
 *        | MSI vector 0                  |
 *        | RX + TX + LINK + ERR          |
 *        +---------------+---------------+
 *                        |
 *                        v
 *        +-------------------------------+
 *        | single MSI interrupt handler  |
 *        +-------------------------------+
 *
 *
 * Hardware interrupt mask control
 * -------------------------------
 *
 *   RTL8125-class controllers have two interrupt control paths:
 *
 *     1. legacy interrupt mask/status registers;
 *     2. newer extended interrupt control registers.
 *
 *   The legacy path uses the traditional interrupt mask/status model:
 *
 *     - enable interrupts:       write mask to IMR;
 *     - disable interrupts:      write 0 to IMR;
 *     - acknowledge interrupts:  write handled bits to ISR.
 *
 *   Newer chips also expose extended interrupt controls for queue/vector
 *   routing, MSI-X operation, moderation and split RX/TX masking.
 *
 *   Timer interrupt mode must use the legacy interrupt mask path.  In this
 *   mode the device depends on the old interrupt generation logic.  Writing
 *   only the new interrupt control registers may leave interrupts
 *   unarmed or prevent RX/TX completions from being delivered.
 *
 *
 * Interrupt moderation
 * --------------------
 *
 *   MSI-X mode uses interrupt mitigation/coalescing.  The device may group
 *   multiple RX/TX completions into one interrupt to reduce CPU overhead.
 *
 *   This improves throughput and reduces lock contention under load, but
 *   may slightly increase low-rate latency.
 *
 *
 * Timer interrupt mode
 * --------------------
 *
 *   Timer interrupt mode is a compatibility/stability mode for systems or
 *   revisions where direct interrupt completion handling is unreliable.
 *
 *   In this mode RX/TX events are accumulated and delivered through the
 *   legacy timer/coalescing interrupt path, typically using the single MSI
 *   vector.
 *
 *   This mode is useful for:
 *
 *     - interrupt storm avoidance;
 *     - lost interrupt workarounds;
 *     - high-rate traffic stabilization;
 *     - debugging interrupt routing issues.
 *
 *
 * Descriptor rings
 * ----------------
 *
 *   RX and TX datapaths use DMA-backed descriptor rings.
 *
 *   RX descriptor formats are revision-dependent.  Different Realtek MAC
 *   versions may place ownership, first/last segment, length, error,
 *   checksum and VLAN/status fields at different bit offsets.
 *
 *   Revision-aware helpers/macros must be used for RX descriptor decoding.
 *   Do not assume RX descriptor bits are compatible across chip revisions.
 *
 *   RX checksum offload is also revision-dependent.  IPv4 checksum status,
 *   TCP/UDP checksum status and error bits may use different descriptor
 *   bits on different MAC versions.
 *
 *   TX descriptors are mostly common across the supported RTL8125+ family
 *   and are treated as shared by this driver.
 *
 *
 * DMA ordering rules
 * ------------------
 *
 *   Descriptor ownership and DMA synchronization order are critical.
 *
 *   TX/RX descriptor handoff to hardware:
 *
 *     1. fill buffer address and descriptor fields;
 *     2. synchronize descriptor/buffer for device access;
 *     3. set OWN bit last.
 *
 *   Completion processing:
 *
 *     1. synchronize descriptor for CPU access;
 *     2. check OWN/status bits;
 *     3. process packet or reclaim buffer;
 *     4. only then recycle descriptor.
 *
 *   Descriptors owned by hardware must not be modified by the driver.
 *
 *
 * RTL8125A / MAC_R25 DMA quirk
 * ----------------------------
 *
 *   RTL8125A-class chips, represented by MAC_R25, have a DMA-related quirk.
 *   Treat this revision conservatively.
 *
 *   Do not relax DMA sync, descriptor ownership ordering or ring restart
 *   logic based only on behavior observed on newer RTL8125B/RTL8126/RTL8127
 *   chips.
 *
 *   Any change touching RX refill, TX reclaim, descriptor ownership,
 *   DMA barriers or reset/restart paths must be tested on MAC_R25
 *   separately.
 *
 *
 * TSO / LSO policy
 * ----------------
 *
 *   TSO/LSO is not currently implemented in the GLD datapath.
 *
 *   Hardware descriptor fields have an MSS limit of 2047 bytes.  This is
 *   enough for normal ETHERMTU operation, but it is not a general jumbo TSO
 *   solution.
 *
 *   Keep TSO disabled unless the full GLD integration, MSS handling,
 *   checksum interaction and jumbo-frame behavior are validated.
 *
 *
 * Recommended MTU policy
 * ----------------------
 *
 *   Default/safe mode:
 *
 *     MTU = ETHERMTU
 *
 *   Jumbo frames require separate validation for:
 *
 *     - RX buffer sizing;
 *     - packet split/merge handling;
 *     - checksum status decoding;
 *     - MSS limits;
 *     - DMA boundary/alignment behavior.
 *
 *
 * Link configuration
 * ------------------
 *
 *   The driver keeps PHY autonegotiation enabled.
 *
 *   Manual speed selection is implemented by restricting advertised PHY
 *   capabilities and restarting autonegotiation.
 *
 *   Example:
 *
 *     To select 100 Mbit/s full-duplex, advertise only 100baseTX full-duplex
 *     capability and disable other advertised speeds.
 *
 *   This provides practical fixed-speed behavior while still using the PHY
 *   autonegotiation mechanism required by modern twisted-pair Ethernet
 *   modes.
 *
 *   Direct fixed-speed PHY mode with autonegotiation disabled through
 *   MAC_PROP_SPEED/MAC_PROP_DUPLEX is not used.
 *
 *
 * Link state handling
 * -------------------
 *
 *   Link change interrupts are notifications, not authoritative state.
 *   Always re-read PHY/MAC link state after receiving a link event.
 *
 *   The driver may also use periodic link polling as a fallback or safety
 *   mechanism.
 *
 *
 * Reset and watchdog model
 * ------------------------
 *
 *   Fatal datapath conditions, TX watchdog timeouts, missed interrupt
 *   recovery or hardware error states schedule an asynchronous reset task.
 *
 *   Reset must not run directly from interrupt context.
 *
 *   Preferred flow:
 *
 *     interrupt/watchdog/error path
 *             |
 *             v
 *     mark reset pending
 *             |
 *             v
 *     schedule reset taskq worker
 *             |
 *             v
 *     stop datapath -> reinitialize hardware/rings -> restart datapath
 *
 *   Interrupt handlers should request reset through the reset scheduling
 *   path rather than performing heavy recovery inline.
 *
 *
 * Locking model
 * -------------
 *
 *   sc->mtx protects global device state, attach state, reset state and
 *   high-level start/stop transitions.
 *
 *   RX and TX rings have separate locks for datapath-local state.
 *
 *   Avoid calling blocking MAC framework routines from hard interrupt
 *   context or while holding locks that may be needed by start/stop/reset
 *   paths.
 *
 *
 * Multiqueue and RSS
 * ------------------
 *
 *   Queue count depends on:
 *
 *     - chip revision;
 *     - available MSI-X vectors;
 *     - platform interrupt allocation result;
 *     - selected fallback mode.
 *
 *   The driver may reduce queue count during attach if the platform cannot
 *   provide the required MSI-X entries.
 *
 *   RSS setup must match the number of active RX rings.
 *
 *
 * Debugging policy
 * ----------------
 *
 *   Prefer kstat counters and explicit debug counters before changing
 *   datapath logic.
 *
 *   Useful symptoms to distinguish:
 *
 *     - interrupt claimed but no RX/TX completion;
 *     - RX descriptors still owned by hardware;
 *     - RX descriptors completed but packets dropped by driver;
 *     - TX descriptors submitted but not reclaimed;
 *     - link interrupt received but PHY state unchanged;
 *     - watchdog reset after TX queue stall.
 *
 *
 * Suspend/resume
 * --------------
 *
 *   Device state after resume must not be trusted.
 *
 *   Prefer full MAC/PHY/ring reinitialization over assuming that descriptor
 *   state, interrupt masks, PHY state or PCIe low-power state survived
 *   suspend/resume unchanged.
 *
 *
 * Current non-goals
 * -----------------
 *
 *   - support for legacy RTL8169/RTL8111 generations;
 *   - firmware-loader dependency;
 *   - full TSO/LSO implementation;
 *   - aggressive PCIe ASPM/power-management reprogramming;
 *   - feature parity with every vendor-driver WoL/offload option.
 */



#include "re.h"

static const ddi_device_acc_attr_t re_reg_map_accattr = {
	DDI_DEVICE_ATTR_V1,
	DDI_STRUCTURE_LE_ACC,
	DDI_STRICTORDER_ACC,
	DDI_DEFAULT_ACC
};

static const ddi_device_acc_attr_t re_stat_accattr = {
	DDI_DEVICE_ATTR_V1,
	DDI_NEVERSWAP_ACC,
	DDI_STRICTORDER_ACC,
	DDI_DEFAULT_ACC
};

static const ddi_dma_attr_t re_stats_dma_attr = {
    .dma_attr_version   = DMA_ATTR_V0,
    .dma_attr_addr_lo   = 0,
    .dma_attr_addr_hi   = UINT64_MAX,
    .dma_attr_count_max = sizeof (struct re_stats),
    .dma_attr_align     = RE_STATS_ALIGNMENT,
    .dma_attr_burstsizes= 0xFFFFFFFF,
    .dma_attr_minxfer   = 1,
    .dma_attr_maxxfer   = sizeof (struct re_stats),
    .dma_attr_seg       = UINT64_MAX,
    .dma_attr_sgllen    = 1,
    .dma_attr_granular  = 1,
    .dma_attr_flags     = 0
};

static char force_msi_propname[] = "force_msi";
static char timer_int_enable_propname[] = "timer_int_enable";
static char timer_count_propname[] = "timer_count";

static void
re_pci_cfg_init(struct re_softc *sc)
{
	sc->pci_cfg.command = pci_config_get16(sc->cfg_handle, PCI_CONF_COMM);
	sc->pci_cfg.vendor = pci_config_get16(sc->cfg_handle, PCI_CONF_VENID);
	sc->pci_cfg.subven = pci_config_get16(sc->cfg_handle, PCI_CONF_SUBVENID);
	sc->pci_cfg.subdev = pci_config_get16(sc->cfg_handle, PCI_CONF_SUBSYSID);
	sc->pci_cfg.device = pci_config_get16(sc->cfg_handle, PCI_CONF_DEVID);
	sc->pci_cfg.revision = pci_config_get8(sc->cfg_handle,  PCI_CONF_REVID);
	sc->pci_cfg.clsize = pci_config_get8(sc->cfg_handle,  PCI_CONF_CACHE_LINESZ);
	sc->pci_cfg.ilr = pci_config_get8(sc->cfg_handle,  PCI_CONF_ILINE);
	sc->re_device_id = sc->pci_cfg.device;
	
	sc->pci_cfg.command |= PCI_COMM_ME | PCI_COMM_MAE;
	pci_config_put16(sc->cfg_handle, PCI_CONF_COMM, sc->pci_cfg.command);
}

static int
re_alloc_stats(struct re_softc *sc)
{
    int err;
    re_dma_buffer_t *dma_p = &sc->re_tally.tally_buf;

    dma_p->re_db_va = NULL;
    dma_p->re_acc_hdl = NULL;
    dma_p->re_dma_hdl = NULL;
    dma_p->ncookies = 0;
    dma_p->re_db_alloc_len = 0;

    err = ddi_dma_alloc_handle(sc->dev, &re_stats_dma_attr,
        DDI_DMA_SLEEP, NULL, &dma_p->re_dma_hdl);
    if (err != DDI_SUCCESS)
        return (DDI_FAILURE);

    err = ddi_dma_mem_alloc(dma_p->re_dma_hdl, RE_STATS_BUF_SIZE,
        &re_stat_accattr, DDI_DMA_CONSISTENT, DDI_DMA_SLEEP, NULL,
        &dma_p->re_db_va, &dma_p->re_db_alloc_len, &dma_p->re_acc_hdl);
    if (err != DDI_SUCCESS) {
        ddi_dma_free_handle(&dma_p->re_dma_hdl);
        dma_p->re_dma_hdl = NULL;
        return (DDI_FAILURE);
    }

    err = ddi_dma_addr_bind_handle(dma_p->re_dma_hdl, NULL,
        dma_p->re_db_va, dma_p->re_db_alloc_len,
        DDI_DMA_RDWR | DDI_DMA_CONSISTENT, DDI_DMA_SLEEP, NULL,
        &dma_p->cookie, &dma_p->ncookies);

    if (err != DDI_DMA_MAPPED || dma_p->ncookies != 1) {
        if (err == DDI_DMA_MAPPED)
            (void) ddi_dma_unbind_handle(dma_p->re_dma_hdl);

        ddi_dma_mem_free(&dma_p->re_acc_hdl);
        dma_p->re_acc_hdl = NULL;
        dma_p->re_db_va = NULL;

        ddi_dma_free_handle(&dma_p->re_dma_hdl);
        dma_p->re_dma_hdl = NULL;

        return (DDI_FAILURE);
    }

    sc->re_tally.stats = (struct re_stats *)(void *)dma_p->re_db_va;
    bzero(sc->re_tally.stats, sizeof (struct re_stats));

    return (DDI_SUCCESS);
}

static void
re_free_stats(struct re_softc *sc)
{
	re_dma_buffer_t *dma_p = &sc->re_tally.tally_buf;

	if (dma_p->re_dma_hdl != NULL) {
		if (dma_p->ncookies != 0) {
			(void) ddi_dma_unbind_handle(dma_p->re_dma_hdl);
			dma_p->ncookies = 0;
			bzero(&dma_p->cookie, sizeof (dma_p->cookie));
		}
	}

	if (dma_p->re_acc_hdl != NULL) {
		ddi_dma_mem_free(&dma_p->re_acc_hdl);
		dma_p->re_acc_hdl = NULL;
	}
	dma_p->re_db_va = NULL;
	dma_p->re_db_alloc_len = 0;

	if (dma_p->re_dma_hdl != NULL) {
		ddi_dma_free_handle(&dma_p->re_dma_hdl);
		dma_p->re_dma_hdl = NULL;
	}

	sc->re_tally.stats = NULL;
}

static int
re_reinit_rings_for_reset(struct re_softc *sc)
{
	int i;

	for (i = 0; i < sc->num_tx_rings; i++) {
		re_tx_ring_t *tr = &sc->tx_rings[i];

		tr->cur_tx = 0;
		tr->dirty_tx = 0;

		re_tx_desc_init_ring(tr);
	}

	for (i = 0; i < sc->num_rx_rings; i++) {
		re_rx_ring_t *rr = &sc->rx_rings[i];

		if (re_rx_ring_fill(sc, rr) != DDI_SUCCESS)
			return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

static void
re_reset_task(void *arg)
{
	struct re_softc *sc = arg;

	mutex_enter(&sc->mtx);
	sc->re_reset_counter++;

	if (sc->suspended ||
	    ((sc->attach_state & RE_ATTACH_MAC_START) == 0)) {
		sc->reset_pending = B_FALSE;
		mutex_exit(&sc->mtx);
		return;
	}

	if (sc->reset_running) {
		mutex_exit(&sc->mtx);
		return;
	}

	sc->reset_pending = B_FALSE;
	sc->reset_running = B_TRUE;
#ifdef DEBUG
	dev_err(sc->dev, CE_WARN,
	    "reset task running, reason=0x%x", sc->reset_reason);
#endif
	re_stop_locked(sc);

	if (re_reinit_rings_for_reset(sc) != DDI_SUCCESS) {
		sc->sc_stopped = B_TRUE;
		sc->reset_running = B_FALSE;
		mutex_exit(&sc->mtx);
		return;
	}

	re_init_locked(sc);

	sc->reset_running = B_FALSE;

	mutex_exit(&sc->mtx);
}

void
re_schedule_reset_locked(struct re_softc *sc, uint32_t reason)
{
	RE_LOCK_ASSERT(sc);

	if (sc->reset_pending || sc->reset_running)
		return;

	sc->reset_pending = B_TRUE;
	sc->reset_reason = reason;

	if (ddi_taskq_dispatch(sc->reset_tq, re_reset_task, sc,
	    DDI_NOSLEEP) == DDI_FAILURE) {
		sc->reset_pending = B_FALSE;
#ifdef DEBUG
		dev_err(sc->dev, CE_WARN,
		    "failed to dispatch reset task");
#endif
	}
}

static boolean_t
re_tx_ring_has_pending(re_tx_ring_t *tr)
{
	uint32_t i;
	boolean_t pending = B_FALSE;

	mutex_enter(&tr->tx_lock);

	for (i = 0; i < tr->num_desc; i++) {
		if (tr->tx_work_list[i] != NULL) {
			pending = B_TRUE;
			break;
		}
	}

	mutex_exit(&tr->tx_lock);

	return (pending);
}

#define RE_TX_WATCHDOG_NS	(6LL * NANOSEC)

static void
re_watchdog_periodic(void *arg)
{
	struct re_softc *sc = arg;
	hrtime_t now;
	int i;

	mutex_enter(&sc->mtx);

	if (sc->suspended ||
	    sc->reset_pending ||
	    sc->reset_running ||
	    ((sc->attach_state & RE_ATTACH_MAC_START) == 0)) {
		mutex_exit(&sc->mtx);
		return;
	}

	now = gethrtime();

	for (i = 0; i < sc->num_tx_rings; i++) {
		re_tx_ring_t *tr = &sc->tx_rings[i];
		hrtime_t last;

		if (!re_tx_ring_has_pending(tr))
			continue;

		last = tr->tx_last_reclaim_time;

		if (last == 0)
			last = tr->tx_last_submit_time;

		if (last == 0)
			continue;

		if (now - last > RE_TX_WATCHDOG_NS) {
			tr->tx_watchdog_timeouts++;
#ifdef DEBUG
			dev_err(sc->dev, CE_WARN,
			    "TX watchdog timeout on ring %d", i);
#endif
			re_schedule_reset_locked(sc, RE_RESET_TX_TIMEOUT);
			break;
		}
	}

	mutex_exit(&sc->mtx);
}

static void
re_unattach(struct re_softc *sc)
{
	sc->sc_stopped = B_TRUE;
	re_disable_extend_tally_couter(sc);

	if (sc->link_status != NULL) {
		ddi_periodic_delete(sc->link_status);
		sc->link_status = NULL;
	}

	if (sc->watchdog_periodic != NULL) {
		ddi_periodic_delete(sc->watchdog_periodic);
		sc->watchdog_periodic = NULL;
	}

	if (sc->reset_tq != NULL) {
		ddi_taskq_destroy(sc->reset_tq);
		sc->reset_tq = NULL;
	}

	if ((sc->attach_state & RE_ATTACH_MAC_START) != 0) {
		mutex_enter(&sc->mtx);
		re_stop_locked(sc);
		re_free_rings_data(sc);
		sc->attach_state &= ~RE_ATTACH_MAC_START;
		mutex_exit(&sc->mtx);
	}

	if ((sc->attach_state & RE_ATTACH_MAC) != 0) {
		(void) mac_unregister(sc->mh);
		sc->mh = NULL;
		sc->attach_state &= ~RE_ATTACH_MAC;
	}

	/*
	 * If at some point attach/start path enables DDI interrupts explicitly
	 * and tracks RE_ATTACH_ENABLE_INTR, disable them here before tearing
	 * handlers down.
	 *
	 * If re_intr_disable_all() remains static in re_intr.c, either export it
	 * or keep RE_ATTACH_ENABLE_INTR unused in attach/unattach.
	 */
	if ((sc->attach_state & RE_ATTACH_ENABLE_INTR) != 0) {
		re_intr_disable_all(sc);
		sc->attach_state &= ~RE_ATTACH_ENABLE_INTR;
	}

	if ((sc->attach_state & RE_ATTACH_ADD_INTR) != 0) {
		re_intr_teardown_handlers(sc);
		sc->attach_state &= ~RE_ATTACH_ADD_INTR;
	}

	if ((sc->attach_state & RE_ATTACH_ALLOC_INTR) != 0) {
		re_free_intr(sc);
		sc->attach_state &= ~RE_ATTACH_ALLOC_INTR;
	}

	if ((sc->attach_state & RE_ATTACH_IRM) != 0) {
		re_irm_unregister(sc);
		sc->attach_state &= ~RE_ATTACH_IRM;
	}

	re_sensor_fini(sc);

	if ((sc->attach_state & RE_ATTACH_ALLOC_RINGS) != 0) {
		re_free_rings(sc);
		sc->attach_state &= ~RE_ATTACH_ALLOC_RINGS;
	}

	if ((sc->attach_state & RE_ATTACH_KSTATS) != 0) {
		re_softc_kstat_fini(sc);
		sc->attach_state &= ~RE_ATTACH_KSTATS;
	}
	
	if ((sc->attach_state & RE_ATTACH_STATS) != 0) {
		re_free_stats(sc);
		sc->attach_state &= ~RE_ATTACH_STATS;
	}
	
	if ((sc->attach_state & RE_ATTACH_LOCKS) != 0) {
		mutex_destroy(&sc->mtx);
		sc->attach_state &= ~RE_ATTACH_LOCKS;
	}

	if ((sc->attach_state & RE_ATTACH_REGS_MAP) != 0) {
		ddi_regs_map_free(&sc->io_handle);
		sc->io_regs = NULL;
		sc->attach_state &= ~RE_ATTACH_REGS_MAP;
	}

	if ((sc->attach_state & RE_ATTACH_PCI_CONFIG) != 0) {
		pci_config_teardown(&sc->cfg_handle);
		sc->attach_state &= ~RE_ATTACH_PCI_CONFIG;
	}

	ddi_set_driver_private(sc->dev, NULL);
	kmem_free(sc, sizeof (*sc));
}

static int
re_suspend(dev_info_t *devinfo)
{
	struct re_softc *sc = ddi_get_driver_private(devinfo);

	RE_LOCK(sc);
	re_stop_locked(sc);
	re_free_rings_data(sc);
	sc->suspended = 1;
	RE_UNLOCK(sc);

	return (DDI_SUCCESS);
}

static int
re_resume(dev_info_t *devinfo)
{
	struct re_softc *sc = ddi_get_driver_private(devinfo);

	RE_LOCK(sc);

	/* reinit if required */
	if (sc->attach_state & RE_ATTACH_MAC_START)
		re_init_locked(sc);

	sc->suspended = 0;

	RE_UNLOCK(sc);

	return (DDI_SUCCESS);
}

static int
re_attach(dev_info_t *devinfo, ddi_attach_cmd_t cmd)
{
	struct re_softc *sc;
	caddr_t regs;
	uint8_t eaddr[ETHERADDRL];
	int err;
	char tq_name[64];

	switch (cmd) {
	case DDI_ATTACH:
		break;
	case DDI_RESUME:
		return (re_resume(devinfo));
	default:
		return (DDI_FAILURE);
	}

	sc = kmem_zalloc(sizeof (*sc), KM_SLEEP);
	ddi_set_driver_private(devinfo, sc);

	sc->force_msi = ddi_prop_get_int(DDI_DEV_T_ANY, devinfo,
	    DDI_PROP_DONTPASS, force_msi_propname, 0);
	sc->timer_int_enable = ddi_prop_get_int(DDI_DEV_T_ANY, devinfo,
	    DDI_PROP_DONTPASS, timer_int_enable_propname, 0);
	sc->timer_count = ddi_prop_get_int(DDI_DEV_T_ANY, devinfo,
	    DDI_PROP_DONTPASS, timer_count_propname, 0x2600);

	sc->dev = devinfo;
	sc->mh = NULL;
	sc->io_regs = NULL;
	sc->mtu = ETHERMTU;
	sc->suspended = 0;
	sc->attach_state = 0;
	sc->re_link_state = LINK_STATE_UNKNOWN;
	sc->re_link_duplex = LINK_DUPLEX_UNKNOWN;
	sc->re_link_speed = 0;

	err = pci_config_setup(devinfo, &sc->cfg_handle);
	if (err != DDI_SUCCESS) {
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "pci_config_setup failed");
#endif
		goto fail;
	}
	sc->attach_state |= RE_ATTACH_PCI_CONFIG;

	re_pci_cfg_init(sc);

	err = ddi_regs_map_setup(devinfo, 2, &regs, 0, 0,
	    &re_reg_map_accattr, &sc->io_handle);
	if (err != DDI_SUCCESS) {
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "ddi_regs_map_setup failed");
#endif
		goto fail;
	}
	sc->io_regs = regs;
	sc->attach_state |= RE_ATTACH_REGS_MAP;

	err = re_check_mac_version(sc);
	if (err != DDI_SUCCESS) {
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "re_check_mac_version failed");
#endif
		goto fail;
	}

	re_init_software_variable(sc);

	mutex_init(&sc->mtx, NULL, MUTEX_DRIVER, NULL);
	sc->attach_state |= RE_ATTACH_LOCKS;

	err = re_alloc_stats(sc);
	if (err != DDI_SUCCESS) {
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "re_alloc_stats failed");
#endif
		goto fail;
	}
	sc->attach_state |= RE_ATTACH_STATS;

	if (!re_softc_kstat_init(sc)) {
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "re_softc_kstat_init failed");
#endif
		goto fail;
	}
	sc->attach_state |= RE_ATTACH_KSTATS;

	/*
	 * Register IRM before interrupt allocation so allocator can account
	 * for desired MSI-X layout immediately.
	 */
	if (sc->msix_min_reqired > 1) {
		err = re_irm_register(sc);
		if (err != DDI_SUCCESS) {
#ifdef DEBUG
			dev_err(devinfo, CE_WARN, "re_irm_register failed");
#endif
			goto fail;
		}
		sc->attach_state |= RE_ATTACH_IRM;
	}

	err = re_alloc_intr(sc);
	if (err != DDI_SUCCESS) {
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "re_alloc_intr failed");
#endif
		goto fail;
	}
	sc->attach_state |= RE_ATTACH_ALLOC_INTR;

	if (sc->HwCurrIsrVer > 1) {
		if (!(sc->re_intr_type & DDI_INTR_TYPE_MSIX) ||
			sc->intr_cnt < sc->msix_min_reqired) {
			sc->num_rx_rings = 1;
			sc->num_tx_rings = 1;
			if(sc->timer_int_enable) {
				sc->HwCurrIsrVer = 1;
				sc->use_new_intr_mapping = 0;
			}
			re_setup_interrupt_mask(sc);
		}
	}

	err = re_alloc_rings(sc);
	if (err != DDI_SUCCESS) {
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "re_alloc_rings failed");
#endif
		goto fail;
	}
	sc->attach_state |= RE_ATTACH_ALLOC_RINGS;

	err = re_intr_setup_handlers(sc);
	if (err != DDI_SUCCESS) {
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "re_intr_setup_handlers failed");
#endif
		goto fail;
	}
	sc->attach_state |= RE_ATTACH_ADD_INTR;

	err = re_intr_enable_all(sc);
	if (err != DDI_SUCCESS) {
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "re_intr_enable_all failed");
#endif
		goto fail;
	}
	sc->attach_state |= RE_ATTACH_ENABLE_INTR;

	mutex_enter(&sc->mtx);

	err = re_chipinit(sc);
	if (err != 0) {
		mutex_exit(&sc->mtx);
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "re_chipinit failed");
#endif
		goto fail;
	}

	re_get_macaddr(sc, eaddr);
	bcopy(eaddr, sc->org_mac_addr, ETHERADDRL);

	mutex_exit(&sc->mtx);

	re_sensor_init(sc);

	if (!re_mac_register(sc)) {
#ifdef DEBUG
		dev_err(devinfo, CE_WARN, "re_mac_register failed");
#endif
		goto fail;
	}
	sc->attach_state |= RE_ATTACH_MAC;

#ifdef DEBUG
	dev_err(sc->dev, CE_NOTE,
	    "!%s%d: intr: type=%s count=%u (max=%u) rx_rings=%u tx_rings=%u",
	    RE_MOD_NAME,
	    ddi_get_instance(sc->dev),
	    (sc->re_intr_type == DDI_INTR_TYPE_MSIX) ? "msix" :
	    (sc->re_intr_type == DDI_INTR_TYPE_MSI) ? "msi" : "unknown",
	    sc->intr_cnt,
	    sc->intr_cnt_max,
	    sc->num_rx_rings,
	    sc->num_tx_rings);
#endif

	sc->link_status = ddi_periodic_add(re_check_link_status, sc,
	    RE_CYCLIC_PERIOD, DDI_IPL_0);

	(void) snprintf(tq_name, sizeof (tq_name),
    "%s%d_reset_taskq", RE_MOD_NAME, ddi_get_instance(sc->dev));

	
	sc->reset_tq = ddi_taskq_create(sc->dev, "re_reset_taskq",
    1, TASKQ_DEFAULTPRI, 0);

	if (sc->reset_tq == NULL)
		return (DDI_FAILURE);

	sc->watchdog_periodic = ddi_periodic_add(re_watchdog_periodic,
		sc, RE_CYCLIC_PERIOD, DDI_IPL_0);

	return (DDI_SUCCESS);

fail:
	re_unattach(sc);
	return (DDI_FAILURE);
}

static int
re_detach(dev_info_t *devinfo, ddi_detach_cmd_t cmd)
{
	struct re_softc *sc = ddi_get_driver_private(devinfo);

	switch (cmd) {
	case DDI_DETACH:
		break;
	case DDI_SUSPEND:
		return (re_suspend(devinfo));
	default:
		return (DDI_FAILURE);
	}

	if (sc == NULL)
		return (DDI_FAILURE);

	/*
	 * MAC framework should have stopped us before detach.
	 * Be defensive and refuse detach if runtime start is still active.
	 */
	if ((sc->attach_state & RE_ATTACH_MAC_START) != 0)
		return (DDI_FAILURE);

	re_unattach(sc);
	return (DDI_SUCCESS);
}

static struct cb_ops re_cb_ops = {
	.cb_open = nulldev,
	.cb_close = nulldev,
	.cb_strategy = nodev,
	.cb_print = nodev,
	.cb_dump = nodev,
	.cb_read = nodev,
	.cb_write = nodev,
	.cb_ioctl = nodev,
	.cb_devmap = nodev,
	.cb_mmap = nodev,
	.cb_segmap = nodev,
	.cb_chpoll = nochpoll,
	.cb_prop_op = ddi_prop_op,
	.cb_flag = D_MP,
	.cb_rev = CB_REV,
	.cb_aread = nodev,
	.cb_awrite = nodev
};

static struct dev_ops re_dev_ops = {
	.devo_rev = DEVO_REV,
	.devo_refcnt = 0,
	.devo_getinfo = NULL,
	.devo_identify = nulldev,
	.devo_probe = nulldev,
	.devo_attach = re_attach,
	.devo_detach = re_detach,
	.devo_reset = nodev,
	.devo_quiesce = ddi_quiesce_not_supported,
	.devo_cb_ops = &re_cb_ops
};

static struct modldrv re_modldrv = {
	.drv_modops = &mod_driverops,
	.drv_linkinfo = "Realtek RTL8125/8126/8127 Ethernet Controller",
	.drv_dev_ops = &re_dev_ops
};

static struct modlinkage re_modlinkage = {
	.ml_rev = MODREV_1,
	.ml_linkage = { &re_modldrv, NULL }
};

int
_init(void)
{
	int ret;

	mac_init_ops(&re_dev_ops, RE_MOD_NAME);

	ret = mod_install(&re_modlinkage);
	if (ret != 0)
		mac_fini_ops(&re_dev_ops);

	return (ret);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&re_modlinkage, modinfop));
}

int
_fini(void)
{
	int ret;

	ret = mod_remove(&re_modlinkage);
	if (ret == 0)
		mac_fini_ops(&re_dev_ops);

	return (ret);
}
