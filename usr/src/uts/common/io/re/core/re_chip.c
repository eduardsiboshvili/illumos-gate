/*
 * Copyright (c) 2025 Adrian Chadd <adrian@FreeBSD.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "re.h"



void
re_stop_locked(struct re_softc *sc)
{
	int ret;

	RE_LOCK_ASSERT(sc);

	sc->sc_stopped = B_TRUE;

	if ((sc->attach_state & RE_ATTACH_MAC_START) == 0)
		return;

    /*
	 * Block packet reception in hardware before reset.
	 */

	RE_CLRBIT_4(sc, RE_RXCFG, RE_RXCFG_ALLPHYS | RE_RXCFG_INDIV |
	    RE_RXCFG_MULTI | RE_RXCFG_BROAD | RE_RXCFG_RUNT |
	    RE_RXCFG_ERRPKT);
	/*
	 * Stop RX/TX paths, wait for interrupt taskq handlers,
	 * then drain software-visible RX/TX work.
	 */
	ret = re_stop_datapath(sc, drv_usectohz(5000000));

#ifdef DEBUG
	if (ret != 0) {
		dev_err(sc->dev, CE_WARN,
		    "re_stop_datapath failed: %d", ret);
	}
#endif

	re_hw_reset(sc);

	/*
	 * Undo runtime interrupt/layer enable if still needed for your path.
	 */
	RE_MAC_CLRBIT(sc, 0xc0ac, 0x1f80);

	sc->re_link_state = LINK_STATE_UNKNOWN;
	sc->re_link_speed = 0;
	sc->re_link_duplex = LINK_DUPLEX_UNKNOWN;
}

static void re_enable_extend_tally_couter(struct re_softc *sc)
{
	RE_MAC_SETBIT(sc, 0xEA84, 0x0003);
}

void re_disable_extend_tally_couter(struct re_softc *sc)
{
	RE_MAC_CLRBIT(sc, 0xEA84, 0x0003);
}

static void
re_hw_config_interrupt_type(struct re_softc *sc)
{
	uint8_t val;

	if (sc->HwSuppIsrVer < 2)
		return;

	val = RE_READ_1(sc, RE_INT_CFG0);

	switch (sc->HwSuppIsrVer) {
	case 7:
		val &= ~RE_INT_CFG0_AVOID_MISS_INTR;
		val &= ~RE_INT_CFG0_AUTO_CLEAR_IMR;
		break;
	case 4:
	case 5:
		val &= ~RE_INT_CFG0_MSIX_ENTRY_NUM_MODE;
		break;
	case 2:
	case 3:
		val &= ~RE_INT_CFG0_EN;
		if (sc->HwCurrIsrVer > 1)
			val |= RE_INT_CFG0_EN;
		break;
	default:
		return;
	}

	RE_WRITE_1(sc, RE_INT_CFG0, val);
}

static void
re_hw_clear_int_miti(struct re_softc *sc)
{
	int i;
	uint8_t v;

	switch (sc->HwSuppIntMitiVer) {
	case 3:
	case 6:
		/* IntMITI_0 - IntMITI_31 */
		for (i = RE_INT_MITI_BASE; i < RE_INT_MITI_END_V3; i += 4)
			RE_WRITE_4(sc, i, 0);
		break;

	case 4:
	case 5:
		/* IntMITI_0 - IntMITI_15 */
		for (i = RE_INT_MITI_BASE; i < RE_INT_MITI_END_V2; i += 4)
			RE_WRITE_4(sc, i, 0);

		v = RE_READ_1(sc, RE_INT_CFG0);
		if (sc->HwSuppIntMitiVer == 5) {
			v &= ~(RE_INT_CFG0_TIMEOUT_BYPASS |
			    RE_INT_CFG0_MITIGATION_BYPASS |
			    RE_INT_CFG0_RDU_BYPASS_8126);
		} else {
			v &= ~(RE_INT_CFG0_TIMEOUT_BYPASS |
			    RE_INT_CFG0_MITIGATION_BYPASS);
		}
		RE_WRITE_1(sc, RE_INT_CFG0, v);

		RE_WRITE_2(sc, RE_INT_CFG1, 0);
		break;
	}
}

static void
re_hw_set_timer_int(struct re_softc *sc, uint8_t timer_val)
{
	int i;

	switch (sc->HwSuppIntMitiVer) {
	case 4:
	case 5:
	case 6:
	 	if ((sc->HwCurrIsrVer == 2))
			timer_val = 0;

		for (i = 0; i < sc->num_rx_rings; i++) {
			re_rx_ring_t *rr = &sc->rx_rings[i];

			if (rr == NULL)
				continue;

			RE_WRITE_1(sc,
			    RE_INT_MITI_V2_0_RX + 8 * rr->index,
			    timer_val);
		}

		for (i = 0; i < sc->num_tx_rings; i++) {
			re_tx_ring_t *tr = &sc->tx_rings[i];

			if (tr == NULL)
				continue;

			RE_WRITE_1(sc,
			    RE_INT_MITI_V2_0_TX + 8 * tr->index,
			    timer_val);
		}
		break;
	}

	/*if (sc->re_type == MAC_R25D_1)
		RE_WRITE_4(sc, 0x0A00, 0x00140014);
	else
		RE_WRITE_4(sc, 0x0A00, 0x00630063);*/
}

void
re_init_locked(struct re_softc *sc)
{
	uint32_t rxconf, val;
	RE_ASSERT_LOCKED(sc);

	/* Don't double-init the hardware */
	if (((sc->attach_state & RE_ATTACH_MAC_START) != 0) && !sc->reset_running) {
		return;
	}

	/*
	* Bring the hardware down so we know it's in a good known
	* state before we bring it up in a good known state.
	*/
	re_stop_locked(sc);

	/* Set MAC address. */
	re_set_macaddr(sc, sc->org_mac_addr);

	if (re_chipinit(sc)) {
		return;
	}

	if (re_phy_config(sc))
		return;

	RE_SETBIT_1(sc, RE_EECMD, RE_EECMD_WRITECFG);

	RE_CLRBIT_1(sc, 0xf1, 0x80);
	re_disable_aspm_clkreq(sc);
	RE_WRITE_2(sc, RE_EEE_TXIDLE_TIMER,
		RE_JUMBO_MTU + ETHER_HDR_LEN + 32);

	/* rings mapping to dev */
	re_map_rings_to_dev(sc);

	/* Set the initial RX and TX configurations. */
	if (sc->re_type == MAC_R25)
		rxconf = RE_RXCFG_CONFIG;
	else if (sc->re_type == MAC_R25B)
		rxconf = RE_RXCFG_CONFIG_8125B;
	else if (RE_TYPE_R25D(sc))
		rxconf = RE_RXCFG_CONFIG_8125D;
	else
		rxconf = RE_RXCFG_CONFIG_8126;

	RE_WRITE_4(sc, RE_RXCFG, rxconf);
	RE_WRITE_4(sc, RE_TXCFG, RE_TXCFG_CONFIG);
	
	re_enable_extend_tally_couter(sc);

	val = re_read_csi(sc, 0x70c) & ~0x3f000000;
	re_write_csi(sc, 0x70c, val | 0x27000000);

	if (RE_TYPE_R26(sc) || sc->re_type == MAC_R27) {
		/* Disable L1 timeout. */
		val = re_read_csi(sc, 0x890) & ~0x00000001;
		re_write_csi(sc, 0x890, val);
	} else if (!RE_TYPE_R25D(sc))
		RE_WRITE_2(sc, 0x0382, 0x221b);

	/* rss config */
	re_config_rss(sc);
	re_set_rx_q_num(sc, sc->num_rx_rings);

	RE_CLRBIT_1(sc, RE_CFG1, RE_CFG1_SPEED_DOWN);

	re_write_mac_ocp(sc, 0xc140, 0xffff);
	re_write_mac_ocp(sc, 0xc142, 0xffff);

	/* new tx desc format */
	RE_MAC_SETBIT(sc, 0xeb58, 0x0001);

	if (RE_TYPE_R26(sc) || sc->re_type == MAC_R27) {
		RE_CLRBIT_1(sc, 0xd8, 0x02);
		if (sc->re_type == MAC_R27) {
			RE_CLRBIT_1(sc, 0x20e4, 0x04);
			RE_MAC_CLRBIT(sc, 0xe00c, 0x1000);
			RE_MAC_CLRBIT(sc, 0xc0c2, 0x0040);
		}
	}

	val = re_read_mac_ocp(sc, 0xe614);
	val &= (sc->re_type == MAC_R27) ? ~0x0f00 : ~0x0700;
	if (sc->re_type == MAC_R25 || RE_TYPE_R25D(sc))
		re_write_mac_ocp(sc, 0xe614, val | 0x0300);
	else if (sc->re_type == MAC_R25B)
		re_write_mac_ocp(sc, 0xe614, val | 0x0200);
	else if (RE_TYPE_R26(sc))
		re_write_mac_ocp(sc, 0xe614, val | 0x0300);
	else
		re_write_mac_ocp(sc, 0xe614, val | 0x0f00);

	/* set tx q num */
	re_set_tx_q_num(sc, sc->num_tx_rings);

	val = re_read_mac_ocp(sc, 0xe63e) & ~0x0030;
	re_write_mac_ocp(sc, 0xe63e, val | 0x0020);

	RE_MAC_CLRBIT(sc, 0xc0b4, 0x0001);
	RE_MAC_SETBIT(sc, 0xc0b4, 0x0001);

	RE_MAC_SETBIT(sc, 0xc0b4, 0x000c);

	val = re_read_mac_ocp(sc, 0xeb6a) & ~0x00ff;
	re_write_mac_ocp(sc, 0xeb6a, val | 0x0033);

	val = re_read_mac_ocp(sc, 0xeb50) & ~0x03e0;
	re_write_mac_ocp(sc, 0xeb50, val | 0x0040);

	RE_MAC_CLRBIT(sc, 0xe056, 0x00f0);

	RE_WRITE_1(sc, RE_TDFNR, 0x10);

	RE_MAC_CLRBIT(sc, 0xe040, 0x1000);

	val = re_read_mac_ocp(sc, 0xea1c) & ~0x0003;
	re_write_mac_ocp(sc, 0xea1c, val | 0x0001);

	if (RE_TYPE_R25D(sc))
		re_write_mac_ocp(sc, 0xe0c0, 0x4403);
	else
		re_write_mac_ocp(sc, 0xe0c0, 0x4000);

	RE_MAC_SETBIT(sc, 0xe052, 0x0060);
	RE_MAC_CLRBIT(sc, 0xe052, 0x0088);

	val = re_read_mac_ocp(sc, 0xd430) & ~0x0fff;
	re_write_mac_ocp(sc, 0xd430, val | 0x045f);

	RE_SETBIT_1(sc, RE_DLLPR, RE_DLLPR_PFM_EN | RE_DLLPR_TX_10M_PS_EN);

	if (sc->re_type == MAC_R25)
		RE_SETBIT_1(sc, RE_MCUCMD, 0x01);

	if (!RE_TYPE_R25D(sc)) {
		/* Disable EEE plus. */
		RE_MAC_CLRBIT(sc, 0xe080, 0x0002);
	}

	if (RE_TYPE_R26(sc) || sc->re_type == MAC_R27)
		RE_MAC_CLRBIT(sc, 0xea1c, 0x0304);
	else
		RE_MAC_CLRBIT(sc, 0xea1c, 0x0004);

	/* Clear tcam entries. */
	RE_MAC_SETBIT(sc, 0xeb54, 0x0001);
	DELAY(1);
	RE_MAC_CLRBIT(sc, 0xeb54, 0x0001);

	RE_CLRBIT_2(sc, 0x1880, 0x0030);

	if (sc->re_rx_ring_desc_type == RE_RX_RING_DESC_TYPE_4) {
		if (sc->Enable_Rss) {
			RE_WRITE_1(sc, 0xd8, RE_READ_1(sc, 0xd8) |
						EnableRxDescV4_0);
		} else {
			RE_WRITE_1(sc, 0xd8, RE_READ_1(sc, 0xd8) &
						~EnableRxDescV4_0);
		}
	}

	if (sc->re_type == MAC_R27) {
		val = re_read_mac_ocp(sc, 0xd40c) & ~0xe038;
		re_write_phy_ocp(sc, 0xd40c, val | 0x8020);
	}

	re_hw_config_interrupt_type(sc);

	/* Clear timer interrupts. */
	RE_WRITE_4(sc, RE_TIMERINT0, 0);
	RE_WRITE_4(sc, RE_TIMERINT1, 0);
	RE_WRITE_4(sc, RE_TIMERINT2, 0);
	RE_WRITE_4(sc, RE_TIMERINT3, 0);

	/* Clear interrupt moderation timer. */
	re_hw_clear_int_miti(sc);

	if (sc->timer_int_enable &&
		sc->HwCurrIsrVer > 1 &&
		sc->HwSuppIntMitiVer > 3 &&
		sc->re_intr_type == DDI_INTR_TYPE_MSIX) {
		re_hw_set_timer_int(sc, sc->timer_count_v2);
	}

	RE_MAC_SETBIT(sc, 0xc0ac, 0x1f80);

	re_write_mac_ocp(sc, 0xe098, 0xc302);

	RE_MAC_CLRBIT(sc, 0xe032, 0x0003);
	val = re_read_csi(sc, 0x98) & ~0x0000ff00;
	re_write_csi(sc, 0x98, val);

	if (RE_TYPE_R25D(sc)) {
		val = re_read_mac_ocp(sc, 0xe092) & ~0x00ff;
		re_write_mac_ocp(sc, 0xe092, val | 0x0008);
	} else
		RE_MAC_CLRBIT(sc, 0xe092, 0x00ff);

	/* Enable/disable HW VLAN tagging based on enabled capability */
	/*if ((if_getcapabilities(sc->sc_ifp) & IFCAP_VLAN_HWTAGGING) != 0)
		RE_SETBIT_4(sc, RE_RXCFG, RE_RXCFG_VLANSTRIP);
	else*/

	/* stack not support vlan stripping offload, always disable */
	RE_CLRBIT_4(sc, RE_RXCFG, RE_RXCFG_VLANSTRIP);

	/* Enable/disable RX checksum based on enabled capability */
	/* now we alway enable it, clear to disable */
	/*if (sc->enable_csum_hw)*/

	RE_SETBIT_2(sc, RE_CPLUSCMD, RE_CPLUSCMD_RXCSUM);

	/*else RE_CLRBIT_2(sc, RE_CPLUSCMD, RE_CPLUSCMD_RXCSUM);*/

	(void) RE_READ_2(sc, RE_CPLUSCMD);

	/* Set Maximum frame size. */
	RE_WRITE_2(sc, RE_RXMAXSIZE, RE_JUMBO_FRAMELEN);

	/* Disable RXDV gate. */
	RE_CLRBIT_1(sc, RE_PPSW, 0x08);
	DELAY(2000);

	/* Program promiscuous mode and multicast filters. */
	re_set_rx_packet_filter(sc);

	if (sc->re_type == MAC_R27)
		RE_CLRBIT_1(sc, RE_RADMFIFO_PROTECT, 0x2001);

	re_disable_aspm_clkreq(sc);

	RE_CLRBIT_1(sc, RE_EECMD, RE_EECMD_WRITECFG);
	DELAY(10);

	(void) re_apply_adv_caps(sc);

	/* Enable transmit and receive. */
	RE_WRITE_1(sc, RE_CMD, RE_CMD_TXENB | RE_CMD_RXENB);

	/* Enable interrupts. */
	re_enable_imr_8125(sc);

	/* Unblock transmit when we release the lock */
	sc->sc_stopped = B_FALSE;
}

#define RE_HASH_POLY 0x04C11DB7U
#define RE_HASH_CRC  0xFFFFFFFFU

uint32_t
re_mcast_hash_index(const uint8_t *mca)
{
	uint32_t crc = RE_HASH_CRC;
	uint32_t msb;
	uint32_t index;
	int bytes, bit;
	uint8_t currentbyte;

	for (bytes = 0; bytes < ETHERADDRL; bytes++) {
		currentbyte = mca[bytes];

		for (bit = 0; bit < 8; bit++) {
			msb = crc >> 31;
			crc <<= 1;

			if ((msb ^ (currentbyte & 1U)) != 0)
				crc ^= RE_HASH_POLY;

			currentbyte >>= 1;
		}
	}

	index = (crc >> 26) & 0x3f;
	return (index);
}

static void
re_set_multicast_reg(struct re_softc *sc, uint32_t mask0, uint32_t mask4)
{
	RE_WRITE_4(sc, RE_MAR0, mask0);
	RE_WRITE_4(sc, RE_MAR4, mask4);
}

static void
re_setmulti(struct re_softc *sc)
{
	uint32_t mask0, mask4;
	uint32_t i;

	/*
	 * promisc/allmulti:
	 * hash table can be all ones, actual RX acceptance is also controlled
	 * by RE_RXCFG_RX_MULTI / RE_RXCFG_RX_ALLPHYS in re_set_rx_packet_filter().
	 */
	if (sc->re_promisc || sc->re_allmulti) {
		re_set_multicast_reg(sc, 0xFFFFFFFFU, 0xFFFFFFFFU);
		return;
	}

	mask0 = 0;
	mask4 = 0;

	for (i = 0; i < 32; i++) {
		if (sc->mcast_refs[i] != 0)
			mask0 |= (1U << i);
	}

	for (i = 32; i < 64; i++) {
		if (sc->mcast_refs[i] != 0)
			mask4 |= (1U << (i - 32));
	}

	re_set_multicast_reg(sc, mask0, mask4);
}

void
re_set_rx_packet_filter(struct re_softc *sc)
{
	uint32_t base_cfg;
	uint32_t rx_mode;
	uint32_t new_cfg;

	base_cfg = RE_READ_4(sc, RE_RXCFG) & 0xff7e5880;

	rx_mode = 0;
	rx_mode |= RE_RXCFG_INDIV;
	rx_mode |= RE_RXCFG_BROAD;
	if (sc->re_promisc) {
		rx_mode |= RE_RXCFG_ALLPHYS |
				RE_RXCFG_MULTI |
				RE_RXCFG_RUNT |
				RE_RXCFG_ERRPKT;
	} else {
		if (sc->re_allmulti || sc->re_mc_count != 0)
			rx_mode |= RE_RXCFG_MULTI;
	}

	if (sc->num_rx_rings > 1) {
		if (sc->re_rx_ring_desc_type == RE_RX_RING_DESC_TYPE_3)
			base_cfg |= EnableRxDescV3;
		else if (sc->re_rx_ring_desc_type == RE_RX_RING_DESC_TYPE_4)
			base_cfg &= ~EnableRxDescV4_1;
	}

	new_cfg = base_cfg | rx_mode;

	RE_WRITE_4(sc, RE_RXCFG, new_cfg);
	re_setmulti(sc);
}
