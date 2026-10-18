// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos SPEEDY (Serial Protocol in an EffEctive Digital waY) master
 *
 * SPEEDY is the one-wire bus the AP uses to talk to its PMIC. Every transfer is
 * run by the IP batcher: the driver loads a small program (opcode table plus
 * payload words) that writes the SPEEDY registers, lets it run and then looks at
 * the status. The 12-bit SPEEDY address is the slave block id (11:8) and the
 * register number (7:0).
 *
 * Based on the downstream i2c-speedy driver:
 * Copyright (C) 2015 Samsung Electronics Ltd.
 * Youngmin Nam <youngmin.nam@samsung.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-speedy.h>

/* SPEEDY registers */
#define SPEEDY_CTRL			0x000
#define SPEEDY_FIFO_CTRL		0x004
#define SPEEDY_CMD			0x008
#define SPEEDY_INT_ENABLE		0x00c
#define SPEEDY_INT_STATUS		0x010
#define SPEEDY_TX_DATA			0x034
#define SPEEDY_RX_DATA			0x038

#define SPEEDY_CTRL_ENABLE		BIT(0)
#define SPEEDY_CTRL_REMOTE_RESET_REQ	BIT(30)
#define SPEEDY_CTRL_SW_RST		BIT(31)
#define SPEEDY_CTRL_RESET_VAL		0x30050

#define SPEEDY_FIFO_RX_TRIGGER(x)	((x) << 0)
#define SPEEDY_FIFO_TX_TRIGGER(x)	((x) << 8)
#define SPEEDY_FIFO_RESET		BIT(31)

#define SPEEDY_CMD_BURST_LEN(x)		((x) << 0)
#define SPEEDY_CMD_BURST_INCR		(1 << 5)
#define SPEEDY_CMD_ADDRESS(x)		(((x) & 0xfff) << 7)
#define SPEEDY_CMD_ACCESS_RANDOM	BIT(19)
#define SPEEDY_CMD_WRITE		BIT(20)

/* SPEEDY_INT_ENABLE / SPEEDY_INT_STATUS */
#define SPEEDY_INT_TRANSFER_DONE	BIT(0)
#define SPEEDY_INT_TIMEOUT_CMD		BIT(1)
#define SPEEDY_INT_TIMEOUT_STANDBY	BIT(2)
#define SPEEDY_INT_TIMEOUT_DATA		BIT(3)
#define SPEEDY_INT_TX_ALMOST_EMPTY	BIT(4)
#define SPEEDY_INT_RX_ALMOST_FULL	BIT(8)
#define SPEEDY_INT_RX_FIFO_TRAILER	BIT(9)
#define SPEEDY_INT_RX_MODEBIT_ERR	BIT(16)
#define SPEEDY_INT_RX_GLITCH_ERR	BIT(17)
#define SPEEDY_INT_RX_ENDBIT_ERR	BIT(18)
#define SPEEDY_INT_TX_LINE_BUSY_ERR	BIT(20)
#define SPEEDY_INT_TX_STOPBIT_ERR	BIT(21)

#define SPEEDY_FIFO_EMPTY		BIT(6)

/* IP batcher registers */
#define BATCHER_CON			0x500
#define BATCHER_STATE			0x504
#define BATCHER_INT_EN			0x508
#define BATCHER_FSM_UNEXPEN		0x50c
#define BATCHER_FSM_TXEN		0x510
#define BATCHER_FSM_RXFIFO		0x514
#define BATCHER_FSM_CON			0x518
#define BATCHER_OPCODE			0x600
#define BATCHER_PAYLOAD			0x1000
#define BATCHER_PAYLOAD_END		0x1060
#define BATCHER_IP_INT_STATUS		0x520
#define BATCHER_SEMA_REL		0x200

#define BATCHER_CON_ENABLE		BIT(0)
#define BATCHER_CON_DEDICATED_APB	BIT(1)
#define BATCHER_CON_START		BIT(4)
#define BATCHER_CON_IP_SW_RST		BIT(6)
#define BATCHER_CON_SEMA_SW_RST		BIT(7)
#define BATCHER_CON_SW_RESET		BIT(31)

#define BATCHER_STATE_COMPLETE		BIT(0)
#define BATCHER_STATE_UNEXPECTED_IRQ	BIT(1)
#define BATCHER_STATE_IDLE		BIT(3)
#define BATCHER_STATE_INIT		BIT(4)
#define BATCHER_STATE_WAIT_INT		BIT(7)
#define BATCHER_STATE_SEMA_LOCKED	BIT(20)

#define BATCHER_INT_ENABLE		BIT(0)
#define BATCHER_FSM_DISABLE_STOP_CMD	BIT(0)

#define BATCHER_OPCODE_END		0xff
#define BATCHER_OPCODE_EMPTY		0xffffffff
#define BATCHER_MAX_PAYLOAD		((BATCHER_PAYLOAD_END - BATCHER_PAYLOAD) / 4 + 1)

#define SPEEDY_TIMEOUT_US		500000
#define SPEEDY_SRP_TRIES		3
#define SPEEDY_XFER_TRIES		3
#define SPEEDY_MAX_BURST		16

#define SPEEDY_ERR_MASK			(SPEEDY_INT_TIMEOUT_CMD | SPEEDY_INT_TIMEOUT_STANDBY | \
					 SPEEDY_INT_TIMEOUT_DATA | SPEEDY_INT_RX_MODEBIT_ERR | \
					 SPEEDY_INT_RX_GLITCH_ERR | SPEEDY_INT_RX_ENDBIT_ERR | \
					 SPEEDY_INT_TX_LINE_BUSY_ERR)

struct exynos_speedy {
	struct device *dev;
	void __iomem *regs;
	struct mutex lock;

	/* batcher program under construction */
	u32 opcodes;
	unsigned int op_idx;
	unsigned int op_reg;
	unsigned int payload;

	bool always_irq_high;
	bool is_srp;
};

struct exynos_speedy_regmap {
	struct exynos_speedy *speedy;
	unsigned int blkid;
};

static void speedy_prog_reset(struct exynos_speedy *s)
{
	s->opcodes = BATCHER_OPCODE_EMPTY;
	s->op_idx = 0;
	s->op_reg = 0;
	s->payload = 0;
}

/* queue "write @val to SPEEDY register @reg" (or, for RX_DATA, a read slot) */
static void speedy_prog_add(struct exynos_speedy *s, u32 val, u32 reg)
{
	if (BATCHER_PAYLOAD + s->payload * 4 > BATCHER_PAYLOAD_END) {
		dev_err(s->dev, "batcher program too long\n");
		return;
	}

	s->opcodes &= ~(0xff << (8 * s->op_idx));
	s->opcodes |= reg << (8 * s->op_idx);
	writel(s->opcodes, s->regs + BATCHER_OPCODE + s->op_reg * 4);
	writel(val, s->regs + BATCHER_PAYLOAD + s->payload * 4);
	s->op_idx++;
	s->payload++;

	if (s->op_idx == 4) {
		s->op_idx = 0;
		s->op_reg++;
		s->opcodes = BATCHER_OPCODE_EMPTY;
	}
}

static void speedy_batcher_con(struct exynos_speedy *s, u32 set, u32 clear)
{
	u32 v = readl(s->regs + BATCHER_CON);

	writel((v & ~clear) | set, s->regs + BATCHER_CON);
}

static u32 speedy_err_bits(struct exynos_speedy *s)
{
	u32 mask = SPEEDY_ERR_MASK;

	if (!s->is_srp)
		mask |= SPEEDY_INT_TX_STOPBIT_ERR;
	return mask;
}

static void speedy_set_cmd(struct exynos_speedy *s, bool write, u16 addr,
			   bool burst, unsigned int len)
{
	u32 fifo = SPEEDY_FIFO_RESET;
	u32 cmd = SPEEDY_CMD_ADDRESS(addr);
	u32 inten = SPEEDY_INT_TIMEOUT_CMD | SPEEDY_INT_TIMEOUT_STANDBY |
		    SPEEDY_INT_TIMEOUT_DATA;

	if (burst) {
		cmd |= SPEEDY_CMD_BURST_INCR | SPEEDY_CMD_BURST_LEN(len - 1);
		if (s->always_irq_high) {
			/* keep the IP interrupt high or the batcher times out */
			if (!write)
				fifo |= SPEEDY_FIFO_RX_TRIGGER(len) | SPEEDY_FIFO_TX_TRIGGER(16);
			else
				fifo |= SPEEDY_FIFO_RX_TRIGGER(0) | SPEEDY_FIFO_TX_TRIGGER(1);
		} else {
			fifo |= SPEEDY_FIFO_RX_TRIGGER(len) | SPEEDY_FIFO_TX_TRIGGER(1);
		}
	} else {
		cmd |= SPEEDY_CMD_ACCESS_RANDOM;
		fifo |= SPEEDY_FIFO_RX_TRIGGER(1) | SPEEDY_FIFO_TX_TRIGGER(1);
	}

	speedy_prog_add(s, fifo, SPEEDY_FIFO_CTRL);

	if (!write) {
		inten |= SPEEDY_INT_RX_ALMOST_FULL | SPEEDY_INT_RX_FIFO_TRAILER |
			 SPEEDY_INT_RX_MODEBIT_ERR | SPEEDY_INT_RX_GLITCH_ERR |
			 SPEEDY_INT_RX_ENDBIT_ERR;
		if (s->always_irq_high)
			inten |= SPEEDY_INT_TX_ALMOST_EMPTY;
	} else {
		cmd |= SPEEDY_CMD_WRITE;
		inten |= SPEEDY_INT_TRANSFER_DONE | SPEEDY_INT_TX_ALMOST_EMPTY |
			 SPEEDY_INT_TX_LINE_BUSY_ERR;
		if (!s->is_srp)
			inten |= SPEEDY_INT_TX_STOPBIT_ERR;
		if (s->always_irq_high)
			inten |= SPEEDY_INT_RX_ALMOST_FULL;
	}

	speedy_prog_add(s, 0xffffffff, SPEEDY_INT_STATUS);
	speedy_prog_add(s, inten, SPEEDY_INT_ENABLE);
	speedy_prog_add(s, cmd, SPEEDY_CMD);
}

static void speedy_prog_finish(struct exynos_speedy *s)
{
	speedy_prog_add(s, 0, BATCHER_OPCODE_END);
}

static void speedy_program_fsm(struct exynos_speedy *s)
{
	u32 unexp = SPEEDY_ERR_MASK;

	if (!s->is_srp)
		unexp |= SPEEDY_INT_TX_STOPBIT_ERR;

	/* interrupts that count as errors, as TX events, and the RX FIFO empty flag */
	writel(unexp, s->regs + BATCHER_FSM_UNEXPEN);
	writel(SPEEDY_INT_TRANSFER_DONE | SPEEDY_INT_TX_ALMOST_EMPTY,
	       s->regs + BATCHER_FSM_TXEN);
	writel(SPEEDY_FIFO_EMPTY, s->regs + BATCHER_FSM_RXFIFO);
	writel(BATCHER_FSM_DISABLE_STOP_CMD, s->regs + BATCHER_FSM_CON);
}

/* batcher state once it finished or hit a real error, 0 while still running */
static u32 speedy_run_state(struct exynos_speedy *s)
{
	u32 state = readl(s->regs + BATCHER_STATE);

	if (state & BATCHER_STATE_COMPLETE)
		return state;
	if ((state & BATCHER_STATE_UNEXPECTED_IRQ) &&
	    (readl(s->regs + BATCHER_IP_INT_STATUS) & speedy_err_bits(s)))
		return state;
	return 0;
}

static int speedy_run(struct exynos_speedy *s)
{
	u32 state;
	int ret;

	speedy_prog_finish(s);
	speedy_program_fsm(s);

	/* the batcher interrupt is only to wake the IP, completion is polled */
	writel(readl(s->regs + BATCHER_INT_EN) | BATCHER_INT_ENABLE, s->regs + BATCHER_INT_EN);
	speedy_batcher_con(s, BATCHER_CON_START, 0);

	ret = read_poll_timeout_atomic(speedy_run_state, state, state, 10,
				       SPEEDY_TIMEOUT_US, false, s);
	writel(readl(s->regs + BATCHER_INT_EN) & ~BATCHER_INT_ENABLE, s->regs + BATCHER_INT_EN);

	if (ret) {
		dev_err(s->dev, "batcher operation timeout\n");
		return -ETIMEDOUT;
	}

	if (readl(s->regs + BATCHER_IP_INT_STATUS) & speedy_err_bits(s)) {
		dev_err(s->dev, "speedy timeout or bus error (state %#x)\n", state);
		ret = -EIO;
	}

	if (state & BATCHER_STATE_COMPLETE)
		writel(state | BATCHER_STATE_COMPLETE, s->regs + BATCHER_STATE);

	return ret;
}

static void speedy_batcher_idle(struct exynos_speedy *s)
{
	writel(0, s->regs + BATCHER_CON);
}

static void speedy_batcher_enable(struct exynos_speedy *s)
{
	speedy_batcher_con(s, BATCHER_CON_ENABLE | BATCHER_CON_DEDICATED_APB, 0);
}

static void speedy_batcher_swreset(struct exynos_speedy *s)
{
	speedy_batcher_con(s, BATCHER_CON_SW_RESET, 0);
	speedy_batcher_con(s, 0, BATCHER_CON_SW_RESET);
}

static void speedy_ctrl_swreset(struct exynos_speedy *s)
{
	writel(readl(s->regs + SPEEDY_CTRL) | SPEEDY_CTRL_SW_RST, s->regs + SPEEDY_CTRL);
	udelay(10);
}

static void speedy_wait_batcher_idle(struct exynos_speedy *s)
{
	u32 state;

	if (readl_poll_timeout_atomic(s->regs + BATCHER_STATE, state,
				      state & (BATCHER_STATE_IDLE | BATCHER_STATE_INIT),
				      10, SPEEDY_TIMEOUT_US))
		dev_err(s->dev, "timeout waiting for batcher idle\n");
}

/* bring the batcher and the SPEEDY IP back after a failed transfer */
static void speedy_recover(struct exynos_speedy *s)
{
	u32 state = readl(s->regs + BATCHER_STATE);

	if (state & BATCHER_STATE_SEMA_LOCKED) {
		if (state & BATCHER_STATE_WAIT_INT) {
			speedy_batcher_con(s, BATCHER_CON_IP_SW_RST, 0);
			udelay(10);
		} else {
			speedy_ctrl_swreset(s);
		}

		udelay(100);
		if (!(readl(s->regs + BATCHER_STATE) & BATCHER_STATE_INIT)) {
			speedy_batcher_con(s, BATCHER_CON_SEMA_SW_RST, 0);
			speedy_batcher_con(s, 0, BATCHER_CON_SEMA_SW_RST);
			udelay(100);
			speedy_batcher_swreset(s);
			speedy_wait_batcher_idle(s);
		}
		writel(1, s->regs + BATCHER_SEMA_REL);
	} else {
		speedy_batcher_swreset(s);
	}

	speedy_wait_batcher_idle(s);
}

/* send the PMIC a remote reset request so it drops any half-finished packet */
static void speedy_remote_reset(struct exynos_speedy *s)
{
	u32 ctrl;
	int i, ret;

	for (i = 0; i < SPEEDY_SRP_TRIES; i++) {
		ctrl = SPEEDY_CTRL_RESET_VAL | SPEEDY_CTRL_ENABLE;

		speedy_batcher_idle(s);
		speedy_batcher_enable(s);
		speedy_prog_reset(s);
		s->is_srp = true;

		speedy_set_cmd(s, true, 0, false, 0);
		speedy_prog_add(s, ctrl | SPEEDY_CTRL_REMOTE_RESET_REQ, SPEEDY_CTRL);
		speedy_prog_add(s, 0, SPEEDY_TX_DATA);
		speedy_prog_add(s, ctrl, SPEEDY_CTRL);

		ret = speedy_run(s);
		s->is_srp = false;
		speedy_batcher_idle(s);
		if (!ret)
			return;

		speedy_recover(s);
	}
}

/* one random/burst access of @len bytes */
static int speedy_xfer_once(struct exynos_speedy *s, bool write, u16 addr,
			    u8 *buf, unsigned int len)
{
	unsigned int rx_payload = 0, i;
	int ret;

	speedy_prog_reset(s);
	speedy_batcher_idle(s);
	speedy_batcher_enable(s);

	speedy_prog_add(s, SPEEDY_CTRL_RESET_VAL | SPEEDY_CTRL_ENABLE, SPEEDY_CTRL);
	speedy_set_cmd(s, write, addr, len > 1, len);

	if (!write) {
		rx_payload = s->payload;
		for (i = 0; i < len; i++)
			speedy_prog_add(s, 0x77, SPEEDY_RX_DATA);
	} else {
		u32 inten;

		for (i = 0; i < len; i++)
			speedy_prog_add(s, buf[i], SPEEDY_TX_DATA);

		/* a pending TX_ALMOST_EMPTY would hang the batcher after the last byte */
		inten = SPEEDY_INT_TIMEOUT_CMD | SPEEDY_INT_TIMEOUT_STANDBY |
			SPEEDY_INT_TIMEOUT_DATA | SPEEDY_INT_TRANSFER_DONE |
			SPEEDY_INT_TX_LINE_BUSY_ERR | SPEEDY_INT_TX_STOPBIT_ERR;
		if (s->always_irq_high)
			inten |= SPEEDY_INT_RX_ALMOST_FULL;
		speedy_prog_add(s, inten, SPEEDY_INT_ENABLE);
	}

	ret = speedy_run(s);
	if (!ret) {
		for (i = 0; !write && i < len; i++)
			buf[i] = readl(s->regs + BATCHER_PAYLOAD + (rx_payload + i) * 4);
		speedy_batcher_idle(s);
		return 0;
	}

	dev_err(s->dev, "%s of %u byte(s) at %#x failed\n", write ? "write" : "read", len, addr);
	speedy_batcher_idle(s);
	speedy_recover(s);
	speedy_remote_reset(s);
	udelay(1);
	return ret;
}

static int speedy_xfer(struct exynos_speedy *s, bool write, u16 addr, u8 *buf,
		       unsigned int len)
{
	int ret = 0, i;

	mutex_lock(&s->lock);
	for (i = 0; i < SPEEDY_XFER_TRIES; i++) {
		ret = speedy_xfer_once(s, write, addr, buf, len);
		if (!ret)
			break;
		udelay(100);
	}
	mutex_unlock(&s->lock);

	return ret;
}

static int speedy_regmap_write(void *context, const void *data, size_t count)
{
	struct exynos_speedy_regmap *ctx = context;
	u8 buf[SPEEDY_MAX_BURST];
	const u8 *d = data;

	if (count < 2 || count - 1 > SPEEDY_MAX_BURST)
		return -EINVAL;

	memcpy(buf, d + 1, count - 1);
	return speedy_xfer(ctx->speedy, true, (ctx->blkid << 8) | d[0], buf, count - 1);
}

static int speedy_regmap_read(void *context, const void *reg, size_t reg_size,
			      void *val, size_t val_size)
{
	struct exynos_speedy_regmap *ctx = context;

	if (reg_size != 1 || !val_size || val_size > SPEEDY_MAX_BURST)
		return -EINVAL;

	return speedy_xfer(ctx->speedy, false, (ctx->blkid << 8) | *(const u8 *)reg,
			   val, val_size);
}

static const struct regmap_bus speedy_regmap_bus = {
	.write = speedy_regmap_write,
	.read = speedy_regmap_read,
	.max_raw_read = SPEEDY_MAX_BURST,
	.max_raw_write = SPEEDY_MAX_BURST,
};

static const struct regmap_config speedy_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

struct regmap *devm_exynos_speedy_regmap(struct device *dev, unsigned int blkid)
{
	struct exynos_speedy_regmap *ctx;
	struct regmap_config config = speedy_regmap_config;

	if (blkid > 0xf)
		return ERR_PTR(-EINVAL);

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->speedy = dev_get_drvdata(dev->parent);
	if (!ctx->speedy)
		return ERR_PTR(-EPROBE_DEFER);
	ctx->blkid = blkid;

	config.name = devm_kasprintf(dev, GFP_KERNEL, "blk%u", blkid);
	if (!config.name)
		return ERR_PTR(-ENOMEM);

	return devm_regmap_init(dev, &speedy_regmap_bus, ctx, &config);
}
EXPORT_SYMBOL_GPL(devm_exynos_speedy_regmap);

static int exynos_speedy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_speedy *s;
	struct clk_bulk_data *clks;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = dev;
	mutex_init(&s->lock);
	s->always_irq_high = of_property_read_bool(dev->of_node, "samsung,always-interrupt-high");

	s->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s->regs))
		return PTR_ERR(s->regs);

	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot enable clocks\n");

	/* the bootloader may have left the IP in any state */
	writel(0xffffffff, s->regs + SPEEDY_INT_STATUS);
	speedy_ctrl_swreset(s);
	writel(1, s->regs + BATCHER_SEMA_REL);
	speedy_batcher_swreset(s);

	platform_set_drvdata(pdev, s);

	return devm_of_platform_populate(dev);
}

static const struct of_device_id exynos_speedy_of_match[] = {
	{ .compatible = "samsung,exynos7570-speedy" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_speedy_of_match);

static struct platform_driver exynos_speedy_driver = {
	.probe = exynos_speedy_probe,
	.driver = {
		.name = "exynos-speedy",
		.of_match_table = exynos_speedy_of_match,
	},
};
module_platform_driver(exynos_speedy_driver);

MODULE_DESCRIPTION("Samsung Exynos SPEEDY PMIC bus driver");
MODULE_LICENSE("GPL");
