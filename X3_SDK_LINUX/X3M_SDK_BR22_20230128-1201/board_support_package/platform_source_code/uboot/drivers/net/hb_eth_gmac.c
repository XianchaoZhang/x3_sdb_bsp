/*
 * Copyright (c) 2016, NVIDIA CORPORATION.
 *
 * SPDX-License-Identifier: GPL-2.0
 *
 * Portions based on U-Boot's rtl8169.c.
 */

/*
 * This driver supports the Synopsys Designware Ethernet QOS (Quality Of
 * Service) IP block. The IP supports multiple options for bus type, clocking/
 * reset structure, and feature list.
 *
 * The driver is written such that generic core logic is kept separate from
 * configuration-specific logic. Code that interacts with configuration-
 * specific resources is split out into separate functions to avoid polluting
 * common code. If/when this driver is enhanced to support multiple
 * configurations, the core code should be adapted to call all configuration-
 * specific functions through function pointers, with the definition of those
 * function pointers being supplied by struct udevice_id eqos_ids[]'s .data
 * field.
 *
 * The following configurations are currently supported:
 * tegra186:
 *    NVIDIA's Tegra186 chip. This configuration uses an AXI master/DMA bus, an
 *    AHB slave/register bus, contains the DMA, MTL, and MAC sub-blocks, and
 *    supports a single RGMII PHY. This configuration also has SW control over
 *    all clock and reset signals to the HW block.
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <errno.h>
#include <memalign.h>
#include <miiphy.h>
#include <net.h>
#include <netdev.h>
#include <phy.h>
#include <reset.h>
#include <wait_bit.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/hb_sysctrl.h>
#include <hb_info.h>

/* GPIO PIN MUX */
#if defined(CONFIG_TARGET_X2_FPGA) || defined(CONFIG_TARGET_X2)
#define PIN_MUX_BASE    0xA6003000
#define GPIO2_CFG (PIN_MUX_BASE + 0x20)
#define GPIO2_DIR (PIN_MUX_BASE + 0x28)
#define GPIO2_VAL (PIN_MUX_BASE + 0x2C)
#define GPIO4_CFG (PIN_MUX_BASE + 0x40)
#define GPIO4_DIR (PIN_MUX_BASE + 0x48)
#define GPIO4_VAL (PIN_MUX_BASE + 0x4C)
#else
#define	GPIO_EPHY_CLK	(PIN_MUX_BASE + 0x98)
#define	GPIO_MDCK	(PIN_MUX_BASE+0x9C)
#define GPIO_MDIO (PIN_MUX_BASE +0xA0)
#define GPIO_RX_CLK (PIN_MUX_BASE +0xA4)
#define GPIO_RGMII_RXD0 (PIN_MUX_BASE +0xA8)
#define GPIO_RGMII_RXD1 (PIN_MUX_BASE +0xAC)
#define GPIO_RGMII_RXD2 (PIN_MUX_BASE +0xB0)
#define GPIO_RGMII_RXD3 (PIN_MUX_BASE +0xB4)
#define GPIO_RGMII_RX_DV (PIN_MUX_BASE +0xB8)
#define GPIO_RGMII_TX_CLK (PIN_MUX_BASE +0xBC)
#define GPIO_RGMII_TXD0 (PIN_MUX_BASE +0xC0)
#define GPIO_RGMII_TXD1 (PIN_MUX_BASE +0xC4)
#define	 GPIO_RGMII_TXD2 (PIN_MUX_BASE +0xC8)
#define GPIO_RGMII_TXD3 (PIN_MUX_BASE +0xCC)
#define GPIO_RGMII_TX_EN (PIN_MUX_BASE +0xD0)
#define GPIO1_DIR (X2_GPIO_BASE + 0x18)
#define GPIO7_DIR (X2_GPIO_BASE + 0x78)
#define ETH0_MODE_CTRL  (0xA1000000 + 0x384)
#define ETH0_MODE_CTRL_RMII_MODE         BIT(0)
#define ETH0_MODE_CTRL_SEL_CLK_RMII_IN   BIT(4)
#define ETH0_MODE_CTRL_SEL_CLK_RMII_INV  BIT(8)
#endif
/* Core registers */

#define EQOS_MAC_REGS_BASE 0x000
struct eqos_mac_regs {
    uint32_t configuration;				/* 0x000 */
    uint32_t unused_004[(0x070 - 0x004) / 4];	/* 0x004 */
    uint32_t q0_tx_flow_ctrl;			/* 0x070 */
    uint32_t unused_070[(0x090 - 0x074) / 4];	/* 0x074 */
    uint32_t rx_flow_ctrl;				/* 0x090 */
    uint32_t unused_094;				/* 0x094 */
    uint32_t txq_prty_map0;				/* 0x098 */
    uint32_t unused_09c;				/* 0x09c */
    uint32_t rxq_ctrl0;				/* 0x0a0 */
    uint32_t unused_0a4;				/* 0x0a4 */
    uint32_t rxq_ctrl2;				/* 0x0a8 */
    uint32_t unused_0ac[(0x0dc - 0x0ac) / 4];	/* 0x0ac */
    uint32_t us_tic_counter;			/* 0x0dc */
    uint32_t unused_0e0[(0x11c - 0x0e0) / 4];	/* 0x0e0 */
    uint32_t hw_feature0;				/* 0x11c */
    uint32_t hw_feature1;				/* 0x120 */
    uint32_t hw_feature2;				/* 0x124 */
    uint32_t unused_128[(0x200 - 0x128) / 4];	/* 0x128 */
    uint32_t mdio_address;				/* 0x200 */
    uint32_t mdio_data;				/* 0x204 */
    uint32_t unused_208[(0x300 - 0x208) / 4];	/* 0x208 */
    uint32_t address0_high;				/* 0x300 */
    uint32_t address0_low;				/* 0x304 */
};

#define EQOS_MAC_CONFIGURATION_GPSLCE			BIT(23)
#define EQOS_MAC_CONFIGURATION_CST			BIT(21)
#define EQOS_MAC_CONFIGURATION_ACS			BIT(20)
#define EQOS_MAC_CONFIGURATION_WD			BIT(19)
#define EQOS_MAC_CONFIGURATION_JD			BIT(17)
#define EQOS_MAC_CONFIGURATION_JE			BIT(16)
#define EQOS_MAC_CONFIGURATION_PS			BIT(15)
#define EQOS_MAC_CONFIGURATION_FES			BIT(14)
#define EQOS_MAC_CONFIGURATION_DM			BIT(13)
#define EQOS_MAC_CONFIGURATION_TE			BIT(1)
#define EQOS_MAC_CONFIGURATION_RE			BIT(0)

#define EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT		16
#define EQOS_MAC_Q0_TX_FLOW_CTRL_PT_MASK		0xffff
#define EQOS_MAC_Q0_TX_FLOW_CTRL_TFE			BIT(1)

#define EQOS_MAC_RX_FLOW_CTRL_RFE			BIT(0)

#define EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_SHIFT		0
#define EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_MASK		0xff

#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT			0
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_MASK			3
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_NOT_ENABLED		0
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB		2

#define EQOS_MAC_RXQ_CTRL2_PSRQ0_SHIFT			0
#define EQOS_MAC_RXQ_CTRL2_PSRQ0_MASK			0xff

#define EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT		6
#define EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK		0x1f
#define EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT		0
#define EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK		0x1f

#define EQOS_MAC_MDIO_ADDRESS_PA_SHIFT			21
#define EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT			16
#define EQOS_MAC_MDIO_ADDRESS_CR_SHIFT			8
#define EQOS_MAC_MDIO_ADDRESS_CR_20_35			5
#define EQOS_MAC_MDIO_ADDRESS_SKAP			BIT(4)
#define EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT			2
#define EQOS_MAC_MDIO_ADDRESS_GOC_READ			3
#define EQOS_MAC_MDIO_ADDRESS_GOC_WRITE			1
#define EQOS_MAC_MDIO_ADDRESS_C45E			BIT(1)
#define EQOS_MAC_MDIO_ADDRESS_GB			BIT(0)

#define EQOS_MAC_MDIO_DATA_GD_MASK			0xffff

#define EQOS_MTL_REGS_BASE 0xd00
struct eqos_mtl_regs {
    uint32_t txq0_operation_mode;			/* 0xd00 */
    uint32_t unused_d04;				/* 0xd04 */
    uint32_t txq0_debug;				/* 0xd08 */
    uint32_t unused_d0c[(0xd18 - 0xd0c) / 4];	/* 0xd0c */
    uint32_t txq0_quantum_weight;			/* 0xd18 */
    uint32_t unused_d1c[(0xd30 - 0xd1c) / 4];	/* 0xd1c */
    uint32_t rxq0_operation_mode;			/* 0xd30 */
    uint32_t unused_d34;				/* 0xd34 */
    uint32_t rxq0_debug;				/* 0xd38 */
};

#define EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT		16
#define EQOS_MTL_TXQ0_OPERATION_MODE_TQS_MASK		0x1ff
#define EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT	2
#define EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_MASK		3
#define EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_ENABLED	2
#define EQOS_MTL_TXQ0_OPERATION_MODE_TSF		BIT(1)
#define EQOS_MTL_TXQ0_OPERATION_MODE_FTQ		BIT(0)

#define EQOS_MTL_TXQ0_DEBUG_TXQSTS			BIT(4)
#define EQOS_MTL_TXQ0_DEBUG_TRCSTS_SHIFT		1
#define EQOS_MTL_TXQ0_DEBUG_TRCSTS_MASK			3

#define EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT		20
#define EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK		0x3ff
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT		14
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFD_MASK		0x3f
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT		8
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFA_MASK		0x3f
#define EQOS_MTL_RXQ0_OPERATION_MODE_EHFC		BIT(7)
#define EQOS_MTL_RXQ0_OPERATION_MODE_RSF		BIT(5)

#define EQOS_MTL_RXQ0_DEBUG_PRXQ_SHIFT			16
#define EQOS_MTL_RXQ0_DEBUG_PRXQ_MASK			0x7fff
#define EQOS_MTL_RXQ0_DEBUG_RXQSTS_SHIFT		4
#define EQOS_MTL_RXQ0_DEBUG_RXQSTS_MASK			3

#define EQOS_DMA_REGS_BASE 0x1000
struct eqos_dma_regs {
    uint32_t mode;					/* 0x1000 */
    uint32_t sysbus_mode;				/* 0x1004 */
    uint32_t unused_1008[(0x1100 - 0x1008) / 4];	/* 0x1008 */
    uint32_t ch0_control;				/* 0x1100 */
    uint32_t ch0_tx_control;			/* 0x1104 */
    uint32_t ch0_rx_control;			/* 0x1108 */
    uint32_t unused_110c;				/* 0x110c */
    uint32_t ch0_txdesc_list_haddress;		/* 0x1110 */
    uint32_t ch0_txdesc_list_address;		/* 0x1114 */
    uint32_t ch0_rxdesc_list_haddress;		/* 0x1118 */
    uint32_t ch0_rxdesc_list_address;		/* 0x111c */
    uint32_t ch0_txdesc_tail_pointer;		/* 0x1120 */
    uint32_t unused_1124;				/* 0x1124 */
    uint32_t ch0_rxdesc_tail_pointer;		/* 0x1128 */
    uint32_t ch0_txdesc_ring_length;		/* 0x112c */
    uint32_t ch0_rxdesc_ring_length;		/* 0x1130 */
};

#define EQOS_DMA_MODE_SWR				BIT(0)

#define EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT		16
#define EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_MASK		0xf
#define EQOS_DMA_SYSBUS_MODE_EAME			BIT(11)
#define EQOS_DMA_SYSBUS_MODE_BLEN16			BIT(3)
#define EQOS_DMA_SYSBUS_MODE_BLEN8			BIT(2)
#define EQOS_DMA_SYSBUS_MODE_BLEN4			BIT(1)

#define EQOS_DMA_CH0_CONTROL_PBLX8			BIT(16)

#define EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT		16
#define EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK		0x3f
#define EQOS_DMA_CH0_TX_CONTROL_OSP			BIT(4)
#define EQOS_DMA_CH0_TX_CONTROL_ST			BIT(0)

#define EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT		16
#define EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK		0x3f
#define EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT		1
#define EQOS_DMA_CH0_RX_CONTROL_RBSZ_MASK		0x3fff
#define EQOS_DMA_CH0_RX_CONTROL_SR			BIT(0)

/* Descriptors */

#define EQOS_DESCRIPTOR_WORDS	4
#define EQOS_DESCRIPTOR_SIZE	(EQOS_DESCRIPTOR_WORDS * 4)
/* We assume ARCH_DMA_MINALIGN >= 16; 16 is the EQOS HW minimum */
#define EQOS_DESCRIPTOR_ALIGN	ARCH_DMA_MINALIGN
#define EQOS_DESCRIPTORS_TX	4
#define EQOS_DESCRIPTORS_RX	4
#define EQOS_DESCRIPTORS_NUM	(EQOS_DESCRIPTORS_TX + EQOS_DESCRIPTORS_RX)
#define EQOS_DESCRIPTORS_SIZE	ALIGN(EQOS_DESCRIPTORS_NUM * \
				      EQOS_DESCRIPTOR_SIZE, ARCH_DMA_MINALIGN)
#define EQOS_BUFFER_ALIGN	ARCH_DMA_MINALIGN
#define EQOS_MAX_PACKET_SIZE	ALIGN(1568, ARCH_DMA_MINALIGN)
#define EQOS_RX_BUFFER_SIZE	(EQOS_DESCRIPTORS_RX * EQOS_MAX_PACKET_SIZE)

/*
 * Warn if the cache-line size is larger than the descriptor size. In such
 * cases the driver will likely fail because the CPU needs to flush the cache
 * when requeuing RX buffers, therefore descriptors written by the hardware
 * may be discarded. Architectures with full IO coherence, such as x86, do not
 * experience this issue, and hence are excluded from this condition.
 *
 * This can be fixed by defining CONFIG_SYS_NONCACHED_MEMORY which will cause
 * the driver to allocate descriptors from a pool of non-cached memory.
 */
#if EQOS_DESCRIPTOR_SIZE < ARCH_DMA_MINALIGN
#if !defined(CONFIG_SYS_NONCACHED_MEMORY) && \
	!defined(CONFIG_SYS_DCACHE_OFF) && !defined(CONFIG_X86)
#warning Cache line size is larger than descriptor size
#endif
#endif

struct eqos_desc {
    u32 des0;
    u32 des1;
    u32 des2;
    u32 des3;
};

#define EQOS_DESC3_OWN		BIT(31)
#define EQOS_DESC3_FD		BIT(29)
#define EQOS_DESC3_LD		BIT(28)
#define EQOS_DESC3_BUF1V	BIT(24)

struct eqos_config {
    bool reg_access_always_ok;
};

struct eqos_priv {
    struct udevice *dev;
    const struct eqos_config *config;
    fdt_addr_t regs;
    struct eqos_mac_regs *mac_regs;
    struct eqos_mtl_regs *mtl_regs;
    struct eqos_dma_regs *dma_regs;
    struct reset_ctl reset_ctl;
    struct gpio_desc phy_reset_gpio;
    struct clk clk_master_bus;
    struct clk clk_rx;
    struct clk clk_ptp_ref;
    struct clk clk_tx;
    struct clk clk_slave_bus;
    struct mii_dev *mii;
    struct phy_device *phy;
    void *descs;
    struct eqos_desc *tx_descs;
    struct eqos_desc *rx_descs;
    int tx_desc_idx, rx_desc_idx;
    void *tx_dma_buf;
    void *rx_dma_buf;
    void *rx_pkt;
    bool started;
    bool reg_access_ok;
    unsigned is_88e6321;
    phy_interface_t interface;
    int speed;
    int duplex;
};

/*
 * TX and RX descriptors are 16 bytes. This causes problems with the cache
 * maintenance on CPUs where the cache-line size exceeds the size of these
 * descriptors. What will happen is that when the driver receives a packet
 * it will be immediately requeued for the hardware to reuse. The CPU will
 * therefore need to flush the cache-line containing the descriptor, which
 * will cause all other descriptors in the same cache-line to be flushed
 * along with it. If one of those descriptors had been written to by the
 * device those changes (and the associated packet) will be lost.
 *
 * To work around this, we make use of non-cached memory if available. If
 * descriptors are mapped uncached there's no need to manually flush them
 * or invalidate them.
 *
 * Note that this only applies to descriptors. The packet data buffers do
 * not have the same constraints since they are 1536 bytes large, so they
 * are unlikely to share cache-lines.
 */
static void *eqos_alloc_descs(unsigned int num)
{
#ifdef CONFIG_SYS_NONCACHED_MEMORY
    return (void *)noncached_alloc(EQOS_DESCRIPTORS_SIZE,
                                   EQOS_DESCRIPTOR_ALIGN);
#else
    return memalign(EQOS_DESCRIPTOR_ALIGN, EQOS_DESCRIPTORS_SIZE);
#endif
}

static void eqos_free_descs(void *descs)
{
#ifdef CONFIG_SYS_NONCACHED_MEMORY
    /* FIXME: noncached_alloc() has no opposite */
#else
    free(descs);
#endif
}

static void eqos_inval_desc(void *desc)
{
#ifndef CONFIG_SYS_NONCACHED_MEMORY
    unsigned long start = (unsigned long)desc & ~(ARCH_DMA_MINALIGN - 1);
    unsigned long end = ALIGN(start + EQOS_DESCRIPTOR_SIZE,
                              ARCH_DMA_MINALIGN);

    invalidate_dcache_range(start, end);
#endif
}

static void eqos_flush_desc(void *desc)
{
#ifndef CONFIG_SYS_NONCACHED_MEMORY
    flush_cache((unsigned long)desc, EQOS_DESCRIPTOR_SIZE);
#endif
}

/*static void eqos_inval_buffer(void *buf, size_t size)
{
    unsigned long start = (unsigned long)buf & ~(ARCH_DMA_MINALIGN - 1);
    unsigned long end = ALIGN(start + size, ARCH_DMA_MINALIGN);

    invalidate_dcache_range(start, end);
}*/

static void eqos_flush_buffer(void *buf, size_t size)
{
    flush_cache((unsigned long)buf, size);
}

static int eqos_mdio_wait_idle(struct eqos_priv *eqos)
{
    return wait_for_bit_le32(&eqos->mac_regs->mdio_address,
                 EQOS_MAC_MDIO_ADDRESS_GB, false,
                 1000000, true);
}

static int eqos_mdio_read(struct mii_dev *bus, int mdio_addr, int mdio_devad,
                          int mdio_reg)
{
    struct eqos_priv *eqos = bus->priv;
    u32 val;
    int ret;

    debug("%s(dev=%p, addr=%x, reg=%d):\n", __func__, eqos->dev, mdio_addr,
          mdio_reg);

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        pr_err("MDIO not idle at entry\n");
        return ret;
    }

    val = readl(&eqos->mac_regs->mdio_address);
    val &= EQOS_MAC_MDIO_ADDRESS_SKAP |
           EQOS_MAC_MDIO_ADDRESS_C45E;
    val |= (mdio_addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT) |
           (mdio_reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT) |
           (EQOS_MAC_MDIO_ADDRESS_CR_20_35 <<
            EQOS_MAC_MDIO_ADDRESS_CR_SHIFT) |
           (EQOS_MAC_MDIO_ADDRESS_GOC_READ <<
            EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT) |
           EQOS_MAC_MDIO_ADDRESS_GB;
    writel(val, &eqos->mac_regs->mdio_address);

    udelay(10);

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        pr_err("MDIO read didn't complete\n");
        return ret;
    }

    val = readl(&eqos->mac_regs->mdio_data);
    val &= EQOS_MAC_MDIO_DATA_GD_MASK;

    debug("%s: val=%x\n", __func__, val);

    return val;
}

static int eqos_mdio_write(struct mii_dev *bus, int mdio_addr, int mdio_devad,
                           int mdio_reg, u16 mdio_val)
{
    struct eqos_priv *eqos = bus->priv;
    u32 val;
    int ret;

    debug("%s(dev=%p, addr=%x, reg=%d, val=%x):\n", __func__, eqos->dev,
          mdio_addr, mdio_reg, mdio_val);

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        pr_err("MDIO not idle at entry\n");
        return ret;
    }

    writel(mdio_val, &eqos->mac_regs->mdio_data);

    val = readl(&eqos->mac_regs->mdio_address);
    val &= EQOS_MAC_MDIO_ADDRESS_SKAP |
           EQOS_MAC_MDIO_ADDRESS_C45E;
    val |= (mdio_addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT) |
           (mdio_reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT) |
           (EQOS_MAC_MDIO_ADDRESS_CR_20_35 <<
            EQOS_MAC_MDIO_ADDRESS_CR_SHIFT) |
           (EQOS_MAC_MDIO_ADDRESS_GOC_WRITE <<
            EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT) |
           EQOS_MAC_MDIO_ADDRESS_GB;
    writel(val, &eqos->mac_regs->mdio_address);

    udelay(10);

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        pr_err("MDIO read didn't complete\n");
        return ret;
    }

    return 0;
}

static int eqos_start_clks_tegra186(struct udevice *dev)
{
    int ret = 0;

    debug("%s(dev=%p):\n", __func__, dev);

    return ret;
}

void eqos_stop_clks_tegra186(struct udevice *dev)
{
    debug("%s(dev=%p):\n", __func__, dev);
}

static int eqos_start_resets_tegra186(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret;


    debug("%s(dev=%p):\n", __func__, dev);


    /* phy reset by gpio in furture */
/*
    ret = dm_gpio_set_value(&eqos->phy_reset_gpio, 1);
    if (ret < 0) {
        pr_err("dm_gpio_set_value(phy_reset, assert) failed: %d", ret);
        return ret;
    }

    udelay(2);


    ret = dm_gpio_set_value(&eqos->phy_reset_gpio, 0);
    if (ret < 0) {
        pr_err("dm_gpio_set_value(phy_reset, deassert) failed: %d", ret);
        return ret;
    }
*/
#if  !defined(CONFIG_TARGET_X2_FPGA) && !defined(CONFIG_TARGET_X3_FPGA)

    ret = reset_assert(&eqos->reset_ctl);
    if (ret < 0) {
        pr_err("reset_assert() failed: %d\n", ret);
        return ret;
    }

    udelay(2);

    ret = reset_deassert(&eqos->reset_ctl);
    if (ret < 0) {
        pr_err("reset_deassert() failed: %d\n", ret);
        return ret;
    }
    debug("%s: OK\n", __func__);
#endif
    return 0;
}

static int eqos_stop_resets_tegra186(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    reset_assert(&eqos->reset_ctl);

    return 0;
}

static ulong eqos_get_tick_clk_rate_tegra186(struct udevice *dev)
{
    return 0;
}

static int eqos_set_full_duplex(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    setbits_le32(&eqos->mac_regs->configuration, EQOS_MAC_CONFIGURATION_DM);

    return 0;
}

static int eqos_set_half_duplex(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    clrbits_le32(&eqos->mac_regs->configuration, EQOS_MAC_CONFIGURATION_DM);

    /* WAR: Flush TX queue when switching to half-duplex */
    setbits_le32(&eqos->mtl_regs->txq0_operation_mode,
                 EQOS_MTL_TXQ0_OPERATION_MODE_FTQ);

    return 0;
}

static int eqos_set_gmii_speed(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    clrbits_le32(&eqos->mac_regs->configuration,
                 EQOS_MAC_CONFIGURATION_PS | EQOS_MAC_CONFIGURATION_FES);

    return 0;
}

static int eqos_set_mii_speed_100(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    setbits_le32(&eqos->mac_regs->configuration,
                 EQOS_MAC_CONFIGURATION_PS | EQOS_MAC_CONFIGURATION_FES);

    return 0;
}
#if 0
static int eqos_set_mii_speed_10(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    clrsetbits_le32(&eqos->mac_regs->configuration,
                    EQOS_MAC_CONFIGURATION_FES, EQOS_MAC_CONFIGURATION_PS);

    return 0;
}
#endif
static int eqos_set_tx_clk_speed_tegra186(struct udevice *dev)
{
    /* X2 fpga not support clk setting */
    return 0;
}

static int eqos_rgmii_adjust_link(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret, duplex, speed;
    debug("%s(dev=%p):\n", __func__, dev);

	duplex = (eqos->is_88e6321) ? 1 : eqos->phy->duplex;
	speed = (eqos->is_88e6321) ? SPEED_1000 : eqos->phy->speed;

    if (duplex)
        ret = eqos_set_full_duplex(dev);
    else
        ret = eqos_set_half_duplex(dev);
    if (ret < 0) {
        pr_err("eqos_set_*_duplex() failed: %d\n", ret);
        return ret;
    }

    switch (speed) {
    case SPEED_1000:
        ret = eqos_set_gmii_speed(dev);
        clk_set_rate(&eqos->clk_master_bus, 125 * 1000000UL);
        clk_set_rate(&eqos->clk_tx, 125 * 1000000UL);
        debug("set mac_div_clk = %lu", clk_get_rate(&eqos->clk_tx));
        printf("set mac_div_clk = %lu", clk_get_rate(&eqos->clk_tx));
        break;
    case SPEED_100:
        ret = eqos_set_mii_speed_100(dev);
        clk_set_rate(&eqos->clk_master_bus, 125 * 1000000UL);
        clk_set_rate(&eqos->clk_tx, 25 * 1000000UL);
        debug("set mac_div_clk = %lu", clk_get_rate(&eqos->clk_tx));
        printf("set mac_div_clk = %lu", clk_get_rate(&eqos->clk_tx));
        break;
    case SPEED_10:
        ret = eqos_set_mii_speed_100(dev);
        clk_set_rate(&eqos->clk_master_bus, 25 * 1000000UL);
        clk_set_rate(&eqos->clk_tx, 25 * 100000UL);
        debug("set mac_div_clk = %lu", clk_get_rate(&eqos->clk_tx));
        printf("set mac_div_clk = %lu", clk_get_rate(&eqos->clk_tx));
        break;
    default:
        pr_err("invalid speed %d\n", speed);
        return -EINVAL;
    }
    if (ret < 0) {
        pr_err("eqos_set_*mii_speed*() failed: %d\n", ret);
        return ret;
    }

    ret = eqos_set_tx_clk_speed_tegra186(dev);
    if (ret < 0) {
        pr_err("eqos_set_tx_clk_speed_tegra186() failed: %d\n", ret);
        return ret;
    }

    return 0;
}

static int eqos_rmii_adjust_link(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret, duplex, speed;

    debug("%s(dev=%p):\n", __func__, dev);

    duplex = eqos->phy->duplex;
	speed = eqos->phy->speed;
    if (duplex)
        ret = eqos_set_full_duplex(dev);
    else
        ret = eqos_set_half_duplex(dev);
    if (ret < 0) {
        pr_err("eqos_set_rmii_duplex() failed: %d\n", ret);
        return ret;
    }

    switch (speed) {
    case SPEED_100:
    case SPEED_10:
        ret = eqos_set_mii_speed_100(dev);
        clk_set_rate(&eqos->clk_master_bus, 150 * 1000000UL);
        clk_set_rate(&eqos->clk_tx, 50 * 1000000UL);
        debug("set mac_div_clk = %lu", clk_get_rate(&eqos->clk_tx));
        printf("set mac_div_clk = %lu", clk_get_rate(&eqos->clk_tx));
        break;
    default:
        pr_err("invalid speed %d\n", speed);
        return -EINVAL;
    }
    if (ret < 0) {
        pr_err("eqos_set_rmii_speed*() failed: %d\n", ret);
        return ret;
    }

    ret = eqos_set_tx_clk_speed_tegra186(dev);
    if (ret < 0) {
        pr_err("eqos_set_tx_clk_speed_tegra186() failed: %d\n", ret);
        return ret;
    }

    return 0;
}

static int eqos_write_hwaddr(struct udevice *dev)
{
    struct eth_pdata *plat = dev_get_platdata(dev);
    struct eqos_priv *eqos = dev_get_priv(dev);
    uint32_t val;

    /*
     * This function may be called before start() or after stop(). At that
     * time, on at least some configurations of the EQoS HW, all clocks to
     * the EQoS HW block will be stopped, and a reset signal applied. If
     * any register access is attempted in this state, bus timeouts or CPU
     * hangs may occur. This check prevents that.
     *
     * A simple solution to this problem would be to not implement
     * write_hwaddr(), since start() always writes the MAC address into HW
     * anyway. However, it is desirable to implement write_hwaddr() to
     * support the case of SW that runs subsequent to U-Boot which expects
     * the MAC address to already be programmed into the EQoS registers,
     * which must happen irrespective of whether the U-Boot user (or
     * scripts) actually made use of the EQoS device, and hence
     * irrespective of whether start() was ever called.
     *
     * Note that this requirement by subsequent SW is not valid for
     * Tegra186, and is likely not valid for any non-PCI instantiation of
     * the EQoS HW block. This function is implemented solely as
     * future-proofing with the expectation the driver will eventually be
     * ported to some system where the expectation above is true.
     */

    if (!plat || !eqos || !eqos->config)
        return 0;

    if (!eqos->config->reg_access_always_ok && !eqos->reg_access_ok)
        return 0;

    /* Update the MAC address */
    val = (plat->enetaddr[5] << 8) |
          (plat->enetaddr[4]);
    writel(val, &eqos->mac_regs->address0_high);
    val = (plat->enetaddr[3] << 24) |
          (plat->enetaddr[2] << 16) |
          (plat->enetaddr[1] << 8) |
          (plat->enetaddr[0]);
    writel(val, &eqos->mac_regs->address0_low);

    return 0;
}

static int get_product_id(struct mii_dev *bus, int addr, int devad)
{
    int value, reg;

    value = eqos_mdio_read(bus, addr, devad, 0x03);
    reg = value & 0xfff0;
    return reg;
}

static int eqos_start(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret, i;
    ulong rate;
    u32 val, tx_fifo_sz, rx_fifo_sz, tqs, rqs, pbl;
    ulong last_rx_desc;
    const void *blob = gd->fdt_blob;
    int node = dev_of_offset(dev);
    int phy_addr = 0;
	const char *phy_mode;

    debug("%s(dev=%p):\n", __func__, dev);

    eqos->tx_desc_idx = 0;
    eqos->rx_desc_idx = 0;

	val = readl(&eqos->mac_regs->unused_0e0[(0xf8 - 0x0e0) / 4]);
	val |= 0x3;
	writel(val, &eqos->mac_regs->unused_0e0[(0xf8 - 0x0e0) / 4]);
    ret = eqos_start_clks_tegra186(dev);
    if (ret < 0) {
        pr_err("eqos_start_clks_tegra186() failed: %d\n", ret);
        goto err;
    }

    ret = eqos_start_resets_tegra186(dev);
    if (ret < 0) {
        pr_err("eqos_start_resets_tegra186() failed: %d\n", ret);
        goto err_stop_clks;
    }

    udelay(10);

    eqos->reg_access_ok = true;
#if !defined(CONFIG_TARGET_X3_FPGA) && !defined(CONFIG_TARGET_X2_FPGA)
    ret = wait_for_bit_le32(&eqos->dma_regs->mode,
                       EQOS_DMA_MODE_SWR, false, 10, false);
    if (ret) {
        pr_err("EQOS_DMA_MODE_SWR stuck\n");
        goto err_stop_resets;
    }
#endif

    rate = eqos_get_tick_clk_rate_tegra186(dev);
    val = (rate / 1000000) - 1;
    writel(val, &eqos->mac_regs->us_tic_counter);

    phy_addr = fdtdec_get_int(blob, node, "phyaddr",0);

    // if baseboard is SDB or CVB, set phy addr to 0x0
    if (hb_base_board_type_get() == BASE_BOARD_X3_SDB ||
        hb_base_board_type_get() == BASE_BOARD_CVB) {
        debug("SDB or CVB\n");
        phy_addr = 0x0;
    }
#if defined CONFIG_TARGET_XJ3 || defined CONFIG_TARGET_X3_FPGA

	phy_mode = fdt_getprop(blob, node, "phy-mode", NULL);
	if (phy_mode)
		eqos->interface = phy_get_interface_by_name(phy_mode);
	else
		eqos->interface = 0;
#if 0
	fl_node = fdt_subnode_offset(blob, node, "fixed-link");

	if (fl_node != -FDT_ERR_NOTFOUND) {
		eqos->speed = fdtdec_get_int(blob, fl_node, "speed", 0);
		eqos->duplex = fdtdec_get_bool(blob, fl_node, "full-duplex");
	}

	printf("%s, and speed:%d, duplex:%d\n", __func__, eqos->speed, eqos->duplex);
#endif

#endif
	if (!eqos->is_88e6321) {
		// eqos->phy = phy_connect(eqos->mii, phy_addr, dev, 0);
		eqos->phy = phy_connect(eqos->mii, phy_addr, dev, eqos->interface);
		if (!eqos->phy || eqos->phy->drv->uid == 0xffffffff) {
			eqos->is_88e6321 = 2;
		} else {
			fprintf(stdout, "Phy name:%s\n",eqos->phy->drv->name);
			fprintf(stdout, "Phy uid:%x\n",eqos->phy->drv->uid);

#if 0
#ifdef CONFIG_TARGET_X3_FPGA
	eqos->phy->autoneg = AUTONEG_DISABLE;
	eqos->phy->speed = eqos->speed;
	eqos->phy->duplex = eqos->duplex;
#endif
#endif

			ret = phy_config(eqos->phy);
			if (ret < 0) {
				pr_err("phy_config() failed: %d\n", ret);
				goto err_shutdown_phy;
			}
			ret = phy_startup(eqos->phy);
			if (ret < 0) {
				pr_err("phy_startup() failed: %d\n", ret);
				goto err_shutdown_phy;
			}

			if (!eqos->phy->link) {
				pr_err("No link\n");
				goto err_shutdown_phy;
			}
		}
	}

    if (eqos->interface == PHY_INTERFACE_MODE_RGMII
        || eqos->interface == PHY_INTERFACE_MODE_RGMII_ID
        || eqos->interface == PHY_INTERFACE_MODE_RGMII_TXID
        || eqos->interface == PHY_INTERFACE_MODE_RGMII_RXID) {
        ret = eqos_rgmii_adjust_link(dev);
    } else if (eqos->interface == PHY_INTERFACE_MODE_RMII) {
        ret = eqos_rmii_adjust_link(dev);
    }

    if (ret < 0) {
        pr_err("eqos_adjust_link() failed: %d\n", ret);
        goto err_shutdown_phy;
    }

    /* Configure MTL */

    /* Enable Store and Forward mode for TX */
    /* Program Tx operating mode */
    setbits_le32(&eqos->mtl_regs->txq0_operation_mode,
                 EQOS_MTL_TXQ0_OPERATION_MODE_TSF |
                 (EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_ENABLED <<
                  EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT));

    /* Transmit Queue weight */
    writel(0x10, &eqos->mtl_regs->txq0_quantum_weight);

    /* Enable Store and Forward mode for RX, since no jumbo frame */
    setbits_le32(&eqos->mtl_regs->rxq0_operation_mode,
                 EQOS_MTL_RXQ0_OPERATION_MODE_RSF);

    /* Transmit/Receive queue fifo size; use all RAM for 1 queue */
    val = readl(&eqos->mac_regs->hw_feature1);
    tx_fifo_sz = (val >> EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT) &
                 EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK;
    rx_fifo_sz = (val >> EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT) &
                 EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK;
	if (eqos->is_88e6321) {
		/*set for mac for mavell*/
		val |= ((1 << 28) | (1 << 5) | ( 1 << 1));
		writel(val, &eqos->mac_regs->hw_feature1);

		val = readl(&eqos->mac_regs->unused_0e0[0]);
		val |= (( 1 << 12) | (1 << 9));
		writel(val, &eqos->mac_regs->unused_0e0[0]);
	}
    /*
     * r/tx_fifo_sz is encoded as log2(n / 128). Undo that by shifting.
     * r/tqs is encoded as (n / 256) - 1.
     */
    tqs = (128 << tx_fifo_sz) / 256 - 1;
    rqs = (128 << rx_fifo_sz) / 256 - 1;

    clrsetbits_le32(&eqos->mtl_regs->txq0_operation_mode,
                    EQOS_MTL_TXQ0_OPERATION_MODE_TQS_MASK <<
                    EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT,
                    tqs << EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
    clrsetbits_le32(&eqos->mtl_regs->rxq0_operation_mode,
                    EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK <<
                    EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT,
                    rqs << EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT);

    /* Flow control used only if each channel gets 4KB or more FIFO */
    if (rqs >= ((4096 / 256) - 1)) {
        u32 rfd, rfa;

        setbits_le32(&eqos->mtl_regs->rxq0_operation_mode,
                     EQOS_MTL_RXQ0_OPERATION_MODE_EHFC);

        /*
         * Set Threshold for Activating Flow Contol space for min 2
         * frames ie, (1500 * 1) = 1500 bytes.
         *
         * Set Threshold for Deactivating Flow Contol for space of
         * min 1 frame (frame size 1500bytes) in receive fifo
         */
        if (rqs == ((4096 / 256) - 1)) {
            /*
             * This violates the above formula because of FIFO size
             * limit therefore overflow may occur inspite of this.
             */
            rfd = 0x3;	/* Full-3K */
            rfa = 0x1;	/* Full-1.5K */
        } else if (rqs == ((8192 / 256) - 1)) {
            rfd = 0x6;	/* Full-4K */
            rfa = 0xa;	/* Full-6K */
        } else if (rqs == ((16384 / 256) - 1)) {
            rfd = 0x6;	/* Full-4K */
            rfa = 0x12;	/* Full-10K */
        } else {
            rfd = 0x6;	/* Full-4K */
            rfa = 0x1E;	/* Full-16K */
        }

        clrsetbits_le32(&eqos->mtl_regs->rxq0_operation_mode,
                        (EQOS_MTL_RXQ0_OPERATION_MODE_RFD_MASK <<
                         EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
                        (EQOS_MTL_RXQ0_OPERATION_MODE_RFA_MASK <<
                         EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT),
                        (rfd <<
                         EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
                        (rfa <<
                         EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT));
    }

    /* Configure MAC */

    clrsetbits_le32(&eqos->mac_regs->rxq_ctrl0,
                    EQOS_MAC_RXQ_CTRL0_RXQ0EN_MASK <<
                    EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT,
                    EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB <<
                    EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT);

    /* Set TX flow control parameters */
    /* Set Pause Time */
    setbits_le32(&eqos->mac_regs->q0_tx_flow_ctrl,
                 0xffff << EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT);
    /* Assign priority for TX flow control */
    clrbits_le32(&eqos->mac_regs->txq_prty_map0,
                 EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_MASK <<
                 EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_SHIFT);
    /* Assign priority for RX flow control */
    clrbits_le32(&eqos->mac_regs->rxq_ctrl2,
                 EQOS_MAC_RXQ_CTRL2_PSRQ0_MASK <<
                 EQOS_MAC_RXQ_CTRL2_PSRQ0_SHIFT);
    /* Enable flow control */
    setbits_le32(&eqos->mac_regs->q0_tx_flow_ctrl,
                 EQOS_MAC_Q0_TX_FLOW_CTRL_TFE);
    setbits_le32(&eqos->mac_regs->rx_flow_ctrl,
                 EQOS_MAC_RX_FLOW_CTRL_RFE);

    clrsetbits_le32(&eqos->mac_regs->configuration,
                    EQOS_MAC_CONFIGURATION_GPSLCE |
                    EQOS_MAC_CONFIGURATION_WD |
                    EQOS_MAC_CONFIGURATION_JD |
                    EQOS_MAC_CONFIGURATION_JE,
                    EQOS_MAC_CONFIGURATION_CST |
                    EQOS_MAC_CONFIGURATION_ACS);

    eqos_write_hwaddr(dev);

    /* Configure DMA */

    /* Enable OSP mode */
    setbits_le32(&eqos->dma_regs->ch0_tx_control,
                 EQOS_DMA_CH0_TX_CONTROL_OSP);

    /* RX buffer size. Must be a multiple of bus width */
    clrsetbits_le32(&eqos->dma_regs->ch0_rx_control,
                    EQOS_DMA_CH0_RX_CONTROL_RBSZ_MASK <<
                    EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT,
                    EQOS_MAX_PACKET_SIZE <<
                    EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT);

    setbits_le32(&eqos->dma_regs->ch0_control,
                 EQOS_DMA_CH0_CONTROL_PBLX8);

    /*
     * Burst length must be < 1/2 FIFO size.
     * FIFO size in tqs is encoded as (n / 256) - 1.
     * Each burst is n * 8 (PBLX8) * 16 (AXI width) == 128 bytes.
     * Half of n * 256 is n * 128, so pbl == tqs, modulo the -1.
     */
    pbl = tqs + 1;
    if (pbl > 32)
        pbl = 32;
    clrsetbits_le32(&eqos->dma_regs->ch0_tx_control,
                    EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK <<
                    EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT,
                    pbl << EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT);

    clrsetbits_le32(&eqos->dma_regs->ch0_rx_control,
                    EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK <<
                    EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT,
                    8 << EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT);

    /* DMA performance configuration */
    val = (2 << EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT) |
          EQOS_DMA_SYSBUS_MODE_EAME | EQOS_DMA_SYSBUS_MODE_BLEN16 |
          EQOS_DMA_SYSBUS_MODE_BLEN8 | EQOS_DMA_SYSBUS_MODE_BLEN4;
    writel(val, &eqos->dma_regs->sysbus_mode);

    /* Set up descriptors */

    memset(eqos->descs, 0, EQOS_DESCRIPTORS_SIZE);
    for (i = 0; i < EQOS_DESCRIPTORS_RX; i++) {
        struct eqos_desc *rx_desc = &(eqos->rx_descs[i]);
        rx_desc->des0 = (u32)(ulong)(eqos->rx_dma_buf +
                                     (i * EQOS_MAX_PACKET_SIZE));
        rx_desc->des3 |= EQOS_DESC3_OWN | EQOS_DESC3_BUF1V;
    }
    flush_cache((unsigned long)eqos->descs, EQOS_DESCRIPTORS_SIZE);

    writel(0, &eqos->dma_regs->ch0_txdesc_list_haddress);
    writel((ulong)eqos->tx_descs, &eqos->dma_regs->ch0_txdesc_list_address);
    writel(EQOS_DESCRIPTORS_TX - 1,
           &eqos->dma_regs->ch0_txdesc_ring_length);

    writel(0, &eqos->dma_regs->ch0_rxdesc_list_haddress);
    writel((ulong)eqos->rx_descs, &eqos->dma_regs->ch0_rxdesc_list_address);
    writel(EQOS_DESCRIPTORS_RX - 1,
           &eqos->dma_regs->ch0_rxdesc_ring_length);

    /* Enable everything */

    setbits_le32(&eqos->mac_regs->configuration,
                 EQOS_MAC_CONFIGURATION_TE | EQOS_MAC_CONFIGURATION_RE);

    setbits_le32(&eqos->dma_regs->ch0_tx_control,
                 EQOS_DMA_CH0_TX_CONTROL_ST);
    setbits_le32(&eqos->dma_regs->ch0_rx_control,
                 EQOS_DMA_CH0_RX_CONTROL_SR);

    /* TX tail pointer not written until we need to TX a packet */
    /*
     * Point RX tail pointer at last descriptor. Ideally, we'd point at the
     * first descriptor, implying all descriptors were available. However,
     * that's not distinguishable from none of the descriptors being
     * available.
     */
    last_rx_desc = (ulong) & (eqos->rx_descs[(EQOS_DESCRIPTORS_RX - 1)]);
    writel(last_rx_desc, &eqos->dma_regs->ch0_rxdesc_tail_pointer);

    eqos->started = true;

    debug("%s: OK\n", __func__);
    return 0;

err_shutdown_phy:
    phy_shutdown(eqos->phy);
    eqos->phy = NULL;
err_stop_resets:
    eqos_stop_resets_tegra186(dev);
err_stop_clks:
    eqos_stop_clks_tegra186(dev);
err:
    pr_err("FAILED: %d\n", ret);
    return ret;
}

void eqos_stop(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int i;

    debug("%s(dev=%p):\n", __func__, dev);

    if (!eqos->started)
        return;
    eqos->started = false;
    eqos->reg_access_ok = false;

    /* Disable TX DMA */
    clrbits_le32(&eqos->dma_regs->ch0_tx_control,
                 EQOS_DMA_CH0_TX_CONTROL_ST);

    /* Wait for TX all packets to drain out of MTL */
    for (i = 0; i < 1000000; i++) {
        u32 val = readl(&eqos->mtl_regs->txq0_debug);
        u32 trcsts = (val >> EQOS_MTL_TXQ0_DEBUG_TRCSTS_SHIFT) &
                     EQOS_MTL_TXQ0_DEBUG_TRCSTS_MASK;
        u32 txqsts = val & EQOS_MTL_TXQ0_DEBUG_TXQSTS;
        if ((trcsts != 1) && (!txqsts))
            break;
    }

    /* Turn off MAC TX and RX */
    clrbits_le32(&eqos->mac_regs->configuration,
                 EQOS_MAC_CONFIGURATION_TE | EQOS_MAC_CONFIGURATION_RE);

    /* Wait for all RX packets to drain out of MTL */
    for (i = 0; i < 1000000; i++) {
        u32 val = readl(&eqos->mtl_regs->rxq0_debug);
        u32 prxq = (val >> EQOS_MTL_RXQ0_DEBUG_PRXQ_SHIFT) &
                   EQOS_MTL_RXQ0_DEBUG_PRXQ_MASK;
        u32 rxqsts = (val >> EQOS_MTL_RXQ0_DEBUG_RXQSTS_SHIFT) &
                     EQOS_MTL_RXQ0_DEBUG_RXQSTS_MASK;
        if ((!prxq) && (!rxqsts))
            break;
    }

    /* Turn off RX DMA */
    clrbits_le32(&eqos->dma_regs->ch0_rx_control,
                 EQOS_DMA_CH0_RX_CONTROL_SR);

    if (eqos->phy) {
        phy_shutdown(eqos->phy);
        eqos->phy = NULL;
    }
    eqos_stop_resets_tegra186(dev);
    eqos_stop_clks_tegra186(dev);

    debug("%s: OK\n", __func__);
}

int eqos_send(struct udevice *dev, void *packet, int length)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    struct eqos_desc *tx_desc;
    int i;

    debug("%s(dev=%p, packet=%p, length=%d):\n", __func__, dev, packet,
          length);

    memcpy(eqos->tx_dma_buf, packet, length);
    eqos_flush_buffer(eqos->tx_dma_buf, length);

    tx_desc = &(eqos->tx_descs[eqos->tx_desc_idx]);
    eqos->tx_desc_idx++;
    eqos->tx_desc_idx %= EQOS_DESCRIPTORS_TX;

    tx_desc->des0 = (ulong)eqos->tx_dma_buf;
    tx_desc->des1 = 0;
    tx_desc->des2 = length;
    /*
     * Make sure that if HW sees the _OWN write below, it will see all the
     * writes to the rest of the descriptor too.
     */
    mb();
    tx_desc->des3 = EQOS_DESC3_OWN | EQOS_DESC3_FD | EQOS_DESC3_LD | length;
    eqos_flush_desc(tx_desc);

    writel((ulong)(tx_desc + 1), &eqos->dma_regs->ch0_txdesc_tail_pointer);

    for (i = 0; i < 1000000; i++) {
        eqos_inval_desc(tx_desc);
        if (!(readl(&tx_desc->des3) & EQOS_DESC3_OWN))
            return 0;
        udelay(1);
    }

    debug("%s: TX timeout\n", __func__);

    return -ETIMEDOUT;
}

int eqos_recv(struct udevice *dev, int flags, uchar **packetp)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    struct eqos_desc *rx_desc;
    int length;

    debug("%s(dev=%p, flags=%x):\n", __func__, dev, flags);

    rx_desc = &(eqos->rx_descs[eqos->rx_desc_idx]);
    if (rx_desc->des3 & EQOS_DESC3_OWN) {
        debug("%s: RX packet not available\n", __func__);
        return -EAGAIN;
    }

    *packetp = eqos->rx_dma_buf +
               (eqos->rx_desc_idx * EQOS_MAX_PACKET_SIZE);
    length = rx_desc->des3 & 0x7fff;
    debug("%s: *packetp=%p, length=%d\n", __func__, *packetp, length);

    /* after loading ethernet data to RAM with DMA, flush all cache
    to ensure the consistency of RAM and dcache */
    flush_dcache_all();

    return length;
}

int eqos_free_pkt(struct udevice *dev, uchar *packet, int length)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    uchar *packet_expected;
    struct eqos_desc *rx_desc;

    debug("%s(packet=%p, length=%d)\n", __func__, packet, length);

    packet_expected = eqos->rx_dma_buf +
                      (eqos->rx_desc_idx * EQOS_MAX_PACKET_SIZE);
    if (packet != packet_expected) {
        debug("%s: Unexpected packet (expected %p)\n", __func__,
              packet_expected);
        return -EINVAL;
    }

    rx_desc = &(eqos->rx_descs[eqos->rx_desc_idx]);
    rx_desc->des0 = (u32)(ulong)packet;
    rx_desc->des1 = 0;
    rx_desc->des2 = 0;
    /*
     * Make sure that if HW sees the _OWN write below, it will see all the
     * writes to the rest of the descriptor too.
     */
    mb();
    rx_desc->des3 |= EQOS_DESC3_OWN | EQOS_DESC3_BUF1V;
    eqos_flush_desc(rx_desc);

    writel((ulong)rx_desc, &eqos->dma_regs->ch0_rxdesc_tail_pointer);

    eqos->rx_desc_idx++;
    eqos->rx_desc_idx %= EQOS_DESCRIPTORS_RX;

    return 0;
}

static int eqos_probe_resources_core(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret;

    debug("%s(dev=%p):\n", __func__, dev);

    eqos->descs = eqos_alloc_descs(EQOS_DESCRIPTORS_TX +
                                   EQOS_DESCRIPTORS_RX);
    if (!eqos->descs) {
        debug("%s: eqos_alloc_descs() failed\n", __func__);
        ret = -ENOMEM;
        goto err;
    }
    eqos->tx_descs = (struct eqos_desc *)eqos->descs;
    eqos->rx_descs = (eqos->tx_descs + EQOS_DESCRIPTORS_TX);
    debug("%s: tx_descs=%p, rx_descs=%p\n", __func__, eqos->tx_descs,
          eqos->rx_descs);

    eqos->tx_dma_buf = memalign(EQOS_BUFFER_ALIGN, EQOS_MAX_PACKET_SIZE);
    if (!eqos->tx_dma_buf) {
        debug("%s: memalign(tx_dma_buf) failed\n", __func__);
        ret = -ENOMEM;
        goto err_free_descs;
    }
    debug("%s: rx_dma_buf=%p\n", __func__, eqos->rx_dma_buf);

    eqos->rx_dma_buf = memalign(EQOS_BUFFER_ALIGN, EQOS_RX_BUFFER_SIZE);
    if (!eqos->rx_dma_buf) {
        debug("%s: memalign(rx_dma_buf) failed\n", __func__);
        ret = -ENOMEM;
        goto err_free_tx_dma_buf;
    }
    debug("%s: tx_dma_buf=%p\n", __func__, eqos->tx_dma_buf);

    eqos->rx_pkt = malloc(EQOS_MAX_PACKET_SIZE);
    if (!eqos->rx_pkt) {
        debug("%s: malloc(rx_pkt) failed\n", __func__);
        ret = -ENOMEM;
        goto err_free_rx_dma_buf;
    }
    debug("%s: rx_pkt=%p\n", __func__, eqos->rx_pkt);

    debug("%s: OK\n", __func__);
    return 0;

err_free_rx_dma_buf:
    free(eqos->rx_dma_buf);
err_free_tx_dma_buf:
    free(eqos->tx_dma_buf);
err_free_descs:
    eqos_free_descs(eqos->descs);
err:

    debug("%s: returns %d\n", __func__, ret);
    return ret;
}

static int eqos_remove_resources_core(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    free(eqos->rx_pkt);
    free(eqos->rx_dma_buf);
    free(eqos->tx_dma_buf);
    eqos_free_descs(eqos->descs);

    debug("%s: OK\n", __func__);
    return 0;
}

static int eqos_probe_resources_tegra186(struct udevice *dev)
{

    debug("%s(dev=%p):\n", __func__, dev);
    return 0;
}

static int eqos_remove_resources_tegra186(struct udevice *dev)
{
    debug("%s: OK\n", __func__);
    return 0;
}

/* set driver data from dts */
static int eqos_ofdata_to_platdata(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    const char *phy_mode;

	phy_mode = dev_read_string(dev, "phy-mode");
	if (phy_mode) {
		eqos->interface = phy_get_interface_by_name(phy_mode);
    } else {
		eqos->interface = 0;
    }
    return 0;
}

static int eqos_probe(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret, type;
    u64 reg_addr;
    unsigned int reg_val;
	unsigned long mclk;
#if defined(CONFIG_TARGET_X2_FPGA) || defined(CONFIG_TARGET_X2)
    struct hb_info_hdr* boot_info;
#endif

    debug("%s(dev=%p):\n", __func__, dev);
#if defined(CONFIG_TARGET_X2_FPGA) || defined(CONFIG_TARGET_X2)
	boot_info = (struct hb_info_hdr*) HB_BOOTINFO_ADDR;
    if (boot_info->board_id == X2_MONO_BOARD_ID) {
		/* GPIO4   7 */
	reg_val = readl(GPIO4_CFG);
	reg_val |= 0x0000c000;
	writel(reg_val, GPIO4_CFG);
	reg_val = readl(GPIO4_DIR);
	reg_val |= 0x00800000;
	reg_val &= 0xffffff7f;
	writel(reg_val, GPIO4_DIR);
	printf("mono board reset phy...\n");
	udelay(25000);
	reg_val = readl(GPIO4_DIR);
	reg_val |= 0x00800080;
	writel(reg_val, GPIO4_DIR);
    }
#elif defined(CONFIG_TARGET_XJ3)
	/* GPIO_EPHY_CLK as reset gpio, oths as eth pin */
	reg_val = readl(GPIO_EPHY_CLK);
	reg_val |= 0x03;
	writel(reg_val, GPIO_EPHY_CLK);
	for (reg_addr = GPIO_MDCK; reg_addr <= GPIO_RGMII_TX_EN; reg_addr += 4) {
		reg_val = readl(reg_addr);
		reg_val &= ~(0x03);
		writel(reg_val, reg_addr);
	}

    if (hb_som_type_get() == SOM_TYPE_X3SDB
		|| hb_som_type_get() == SOM_TYPE_X3SDBV4
		|| hb_som_type_get() == SOM_TYPE_X3E) {
        // reset
        reg_val = readl(PIN_MUX_BASE + (1*16 + 8)*4);
        reg_val |= 0x03;
        writel(reg_val, PIN_MUX_BASE + (1*16 + 8)*4);

        // intb
        reg_val = readl(PIN_MUX_BASE + (1*16 + 9)*4);
        reg_val |= 0x03;
        writel(reg_val, PIN_MUX_BASE + (1*16 + 9)*4);

        //reg_val = readl(GPIO1_CFG);
        //reg_val |= 0x00030000;
        //writel(reg_val, GPIO1_CFG);

        reg_val = readl(GPIO1_DIR);
        reg_val |= 0x01000000;
        reg_val &= 0xfdfffeff; // 1.8 output, 1.9 input
        writel(reg_val, GPIO1_DIR);

        mdelay(100);

        reg_val |= 0x01000100;

        writel(reg_val, GPIO1_DIR);
        mdelay(200);
        reg_val = readl(GPIO1_DIR);

        reg_val |= 0x01000000;
        reg_val &= 0xfffffeff;

        writel(reg_val, GPIO1_DIR);
        mdelay(500);
    }else if (hb_som_type_get() == SOM_TYPE_X3PI || hb_som_type_get() == SOM_TYPE_X3PIV2) {
        // reset gpio 20
        reg_val = readl(PIN_MUX_BASE + (1*16 + 4)*4);
        reg_val |= 0x03;
        writel(reg_val, PIN_MUX_BASE + (1*16 + 4)*4);

        // intb gpio 120
        reg_val = readl(PIN_MUX_BASE + (7*16 + 8)*4);
        reg_val |= 0x03;
        writel(reg_val, PIN_MUX_BASE + (7*16 + 8)*4);

        // set gpio20 to output 0
        reg_val = readl(GPIO1_DIR);
        reg_val |= 0x00100000;
        reg_val &= 0xffffffef; // 1.8 output, 1.9 input
        writel(reg_val, GPIO1_DIR);

        // set gpio120 to input
        reg_val = readl(GPIO7_DIR);
        reg_val &= 0xfeffffff; // 1.8 output, 1.9 input
        writel(reg_val, GPIO7_DIR);

        mdelay(100);

        // set gpio20 to output 1
        reg_val |= 0x00000010;
        writel(reg_val, GPIO1_DIR);
        mdelay(200);
        pr_err("x3 sdb reset eth phy done\n");
    } else {
        /* reset phy: GPIO_EPHY_CLK(GPIO2[6]) */
        reg_val = readl(GPIO_BASE + 0x28);
        reg_val |= (1<<22);
        reg_val &= ~(1<<6);
        writel(reg_val, GPIO_BASE + 0x28);
        mdelay(10);
        reg_val |= (1<<6);
        writel(reg_val, GPIO_BASE + 0x28);
    }

#if 0
    /*modify pin voltage to 1.8v*/
    reg_val = 0;
    reg_val = readl(IO_MODE_CTRL);
    reg_val |= (3 << 10);
    writel(reg_val, IO_MODE_CTRL);
    printf("%s, reg_val:0x%x, io_mode_ctrl:0x%lx\n", __func__, reg_val, readl(IO_MODE_CTRL));
#endif

#else
	reg_val = readl(GPIO_EPHY_CLK);
	for (reg_addr = GPIO_EPHY_CLK; reg_addr <= GPIO_RGMII_TX_EN; reg_addr += 4) {
		reg_val = readl(reg_addr);
		reg_val &= ~(0x03);
		writel(reg_val, reg_addr);
	}
#endif
    eqos->dev = dev;
    eqos->config = (void *)dev_get_driver_data(dev);

    eqos->regs = devfdt_get_addr(dev);
    if (eqos->regs == FDT_ADDR_T_NONE) {
        pr_err("dev_get_addr() failed\n");
        return -ENODEV;
    }
    eqos->mac_regs = (void *)(eqos->regs + EQOS_MAC_REGS_BASE);
    eqos->mtl_regs = (void *)(eqos->regs + EQOS_MTL_REGS_BASE);
    eqos->dma_regs = (void *)(eqos->regs + EQOS_DMA_REGS_BASE);

    /* set RMII/RGMII MODE */
    switch (eqos->interface) {
    case PHY_INTERFACE_MODE_RGMII:
    case PHY_INTERFACE_MODE_RGMII_ID:
    case PHY_INTERFACE_MODE_RGMII_TXID:
    case PHY_INTERFACE_MODE_RGMII_RXID:
        clrbits_le32(ETH0_MODE_CTRL, ETH0_MODE_CTRL_RMII_MODE);
        setbits_le32(&eqos->dma_regs->mode, EQOS_DMA_MODE_SWR);
        break;
    case PHY_INTERFACE_MODE_RMII:
        setbits_le32(ETH0_MODE_CTRL,
                    ETH0_MODE_CTRL_RMII_MODE | ETH0_MODE_CTRL_SEL_CLK_RMII_INV);
        setbits_le32(&eqos->dma_regs->mode, EQOS_DMA_MODE_SWR);
        break;
    default:
        printf("Nod interface defined!\n");
        return -ENXIO;
    }

	ret = clk_get_by_name(dev, "mac_pre_div_clk", &eqos->clk_master_bus);
	if (ret) {
		pr_err("clk_get_by_name(%s) failed\n", "mac_pre_div_clk");
		return -ENODEV;
	}
	ret = clk_get_by_name(dev, "mac_div_clk", &eqos->clk_tx);
	if (ret) {
		pr_err("clk_get_by_name(%s) failed\n", "mac_div_clk");
		return -ENODEV;
	}
	ret = clk_get_by_name(dev, "phy_ref_clk", &eqos->clk_ptp_ref);
	if (ret) {
		pr_err("clk_get_by_name(%s) failed\n", "phy_ref_clk");
		return -ENODEV;
	}
#if 1
	clk_set_rate(&eqos->clk_master_bus, 125 * 1000000UL);
	mclk = clk_get_rate(&eqos->clk_master_bus);
	if (mclk != 125 * 1000000UL)
		pr_warn("warnning: mac_pre_div_clk = %lu\n", mclk);
#endif

    ret = eqos_probe_resources_core(dev);
    if (ret < 0) {
        pr_err("eqos_probe_resources_core() failed: %d\n", ret);
        return ret;
    }

    ret = eqos_probe_resources_tegra186(dev);
    if (ret < 0) {
        pr_err("eqos_probe_resources_tegra186() failed: %d\n", ret);
        goto err_remove_resources_core;
    }

    eqos->mii = mdio_alloc();
    if (!eqos->mii) {
        pr_err("mdio_alloc() failed\n");
        goto err_remove_resources_tegra;
    }
    eqos->mii->read = eqos_mdio_read;
    eqos->mii->write = eqos_mdio_write;
    eqos->mii->priv = eqos;
    strcpy(eqos->mii->name, dev->name);

    ret = mdio_register(eqos->mii);
    if (ret < 0) {
        pr_err("mdio_register() failed: %d\n", ret);
        goto err_free_mdio;
    }

    /* set GPIO2 PINs to function0(RGMII) */

	#if defined(CONFIG_TARGET_X2_FPGA) || defined(CONFIG_TARGET_X2)
    		writel(0, GPIO2_CFG);
	#endif

    type = get_product_id(eqos->mii, 0x12, 0);
    if (type == 0x3100)
         eqos->is_88e6321 =  1;
    else
         eqos->is_88e6321 =  0;

    if (eqos->is_88e6321 == 1) {
        /*set port 2, 6 1000M full duplex*/
        eqos_mdio_write(eqos->mii, 0x12, 0, 4, 0x3 | (0x3 << 2) );
        eqos_mdio_write(eqos->mii, 0x12, 0, 1, 0xc03e );

        eqos_mdio_write(eqos->mii, 0x16, 0, 4, 0x3 | (0x3 << 2) );
        eqos_mdio_write(eqos->mii, 0x16, 0, 1, 0xc03e );

        /*disable port 0/1*/
        eqos_mdio_write(eqos->mii, 0x10, 0, 1, 0x10 );
        eqos_mdio_write(eqos->mii, 0x11, 0, 1, 0x10 );
    }

    debug("%s: OK\n", __func__);
    return 0;

err_free_mdio:
    mdio_free(eqos->mii);
err_remove_resources_tegra:
    eqos_remove_resources_tegra186(dev);
err_remove_resources_core:
    eqos_remove_resources_core(dev);

    debug("%s: returns %d\n", __func__, ret);
    return ret;
}

static int eqos_remove(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    mdio_unregister(eqos->mii);
    mdio_free(eqos->mii);
    eqos_remove_resources_tegra186(dev);
    eqos_probe_resources_core(dev);

    debug("%s: OK\n", __func__);
    return 0;
}

static const struct eth_ops eqos_ops = {
    .start = eqos_start,
    .stop = eqos_stop,
    .send = eqos_send,
    .recv = eqos_recv,
    .free_pkt = eqos_free_pkt,
    .write_hwaddr = eqos_write_hwaddr,
};

static const struct eqos_config eqos_tegra186_config = {
    .reg_access_always_ok = false,
};

static const struct udevice_id eqos_ids[] = {
    {
        .compatible = "hobot,hb-gmac",
        .data = (ulong)&eqos_tegra186_config
    },
    { }
};

U_BOOT_DRIVER(eth_eqos) = {
    .name = "eth_eqos",
    .id = UCLASS_ETH,
    .of_match = eqos_ids,
    .ofdata_to_platdata = eqos_ofdata_to_platdata,
    .probe = eqos_probe,
    .remove = eqos_remove,
    .ops = &eqos_ops,
    .priv_auto_alloc_size = sizeof(struct eqos_priv),
    .platdata_auto_alloc_size = sizeof(struct eth_pdata),
};
