// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Anbernic single-ADC / MCU-SPI dual-joystick gamepad driver
 *
 * Based on Anbernic; adapted for mainline (no MTK deps).
 * Digital buttons via GPIO; analog sticks/triggers via SPI MCU when present.
 */
#define SUPPORT_MCU_SPI_ADC

#include <linux/crc-itu-t.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/soc/qcom/geni-se.h>
#include <linux/spi/spi.h>
#include <dt-bindings/gpio/gpio.h>

#define DRV_NAME "rocknix-joypad"

/*
 * Mainline sm4450.dtsi: tlmm @ 0x0f100000 (NOT stock flattened 0x0f000000).
 * Wrong base caused MMIO faults / SCM_IO -EINVAL.
 */
#define RG55G1_TLMM_PHYS	0x0f100000UL
#define RG55G1_TLMM_SIZE	0x00300000UL
#define TLMM_REG_SIZE		0x1000
#define TLMM_CTL_OE		BIT(9)
#define TLMM_CTL_MUX_MASK	(0x7 << 2)
#define TLMM_CTL_PULL_MASK	0x3
#define TLMM_CTL_PULL_UP	0x3
#define TLMM_IO_IN		BIT(0)
#define TLMM_IO_OUT		BIT(1)
#define TLMM_MAX_GPIO		136

/* GENI SE3 @ 0x98c000 — MCU analog sticks / LT / RT */
#define RG55_GENI_SPI_PHYS	0x0098c000UL
#define RG55_GENI_SPI_SIZE	0x4000UL
#define SE_SPI_CPHA		0x224
#define SE_SPI_LOOPBACK		0x22c
#define SE_SPI_CPOL		0x230
#define SE_SPI_DEMUX_SEL	0x250
#define SE_SPI_TRANS_CFG	0x25c
#define SE_SPI_WORD_LEN		0x268
#define SE_SPI_TX_TRANS_LEN	0x26c
#define SE_SPI_RX_TRANS_LEN	0x270
#define CS_TOGGLE		BIT(1)
#define WORD_LEN_MSK		GENMASK(9, 0)
#define MIN_WORD_LEN		4
#define TRANS_LEN_MSK		GENMASK(23, 0)
/* Full-duplex uses TX_ONLY|RX_ONLY (==3). SPI_TX_RX(7) is wrong for FIFO. */
#define SPI_TX_ONLY		1
#define SPI_RX_ONLY		2
#define SPI_FULL_DUPLEX		(SPI_TX_ONLY | SPI_RX_ONLY)

#ifdef SUPPORT_MCU_SPI_ADC
#define ADC_MAX_VOLTAGE			32768
#else
#define ADC_MAX_VOLTAGE			1800
#endif
#define ADC_DATA_TUNING(x, p)		(((x) * (p)) / 100)
#define ADC_TUNING_DEFAULT		180

enum {
	ADC_CHANNEL_RY = 0,
	ADC_CHANNEL_RX,
	ADC_CHANNEL_LY,
	ADC_CHANNEL_LX,
	ADC_CHANNEL_Z,
	ADC_CHANNEL_RZ,
	ADC_CHANNEL_COUNT,
};

struct bt_adc {
	int value;
	int old_value;
	int report_type;
	int max;
	int min;
	int cal;
	int scale;
	bool invert;
	int tuning_p;
	int tuning_n;
};

struct bt_gpio {
	const char *label;
	int gpio;		/* TLMM gpio number, -1 if unused */
	int report_type;
	int linux_code;
	int abs_value;
	int old_value;
	bool active_low;
};

struct joypad {
	struct device *dev;
	struct input_dev *input;
	void __iomem *tlmm;	/* direct MMIO at 0x0f100000 */
	bool gpio_live;
	int poll_interval;
	bool enable;

	int amux_count;
	struct bt_adc *adcs;

	int bt_gpio_count;
	struct bt_gpio *gpios;

	int auto_repeat;
	int bt_adc_fuzz;
	int bt_adc_flat;
	int bt_adc_scale;
	int bt_adc_deadzone;

	int pwr_gpio_pins[8];
	int pwr_gpio_count;

	int zero_samples;
	int zero_acc[ADC_CHANNEL_COUNT];

	int hat0x;
	int hat0y;

	struct mutex lock;
};

#ifdef SUPPORT_MCU_SPI_ADC
struct spi_mcu_data {
	struct device *dev;
	struct spi_device *spi;	/* optional (spi-geni path) */
	void __iomem *geni;	/* MMIO GENI SE3 fallback */
	u32 tx_fifo_depth;
	bool mmio;
	char *rx_buf;
	char *tx_buf;
	int buf_len;
	u32 crc_ok;
	u32 crc_err;
	/* Joystick RGB state, mirrored into the request frame by joypad_led_apply() */
	u8 led_switch;
	u8 led_mode;
	u8 led_level;
	u8 led_speed;
	u8 led_r, led_g, led_b;
	u8 led_seg[48];	/* 16 x RGB for mode 5 */
	u8 led_tag;
};

static struct spi_mcu_data *g_smd;
static void joypad_led_apply(struct spi_mcu_data *smd);
#define JOYPAD_LED_SPEED_DEFAULT 0x32

/*
 * Report request, as sent by the stock spidev3.0 test tool shipped in the
 * RG55G1 firmware. Bytes [7..13] and [59..62] are the LED fields, tag and CRC
 * (see the LED block below); joypad_led_apply() rewrites them before the frame
 * is first used, so only the header [0..6] really matters here.
 */
static const char joypad_mcu_tx_cmd[64] = {
	0x5a, 0x3a, 0x66, 0x66, 0x00, 0x0d, 0x0a, 0x04,
	0x01, 0xff, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x02, 0xd5, 0x0d, 0x00
};
#endif

static struct joypad *poll_dev_private;
static struct input_dev *poll_dev_input;

static unsigned int g_button_adc_fuzz;
static unsigned int g_button_adc_flat;
static unsigned int g_button_adc_scale;
static unsigned int g_button_adc_deadzone;

module_param_named(button_adc_fuzz, g_button_adc_fuzz, uint, 0644);
module_param_named(button_adc_flat, g_button_adc_flat, uint, 0644);
module_param_named(button_adc_scale, g_button_adc_scale, uint, 0644);
module_param_named(button_adc_deadzone, g_button_adc_deadzone, uint, 0644);

#ifdef SUPPORT_MCU_SPI_ADC
static int spi_write_and_read_slt(struct spi_device *spi, const void *tx_buf,
				  void *rx_buf, size_t len)
{
	struct spi_transfer t = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = len,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

static int joypad_geni_mmio_xfer(struct spi_mcu_data *smd, const u8 *tx,
				 u8 *rx, size_t len)
{
	void __iomem *base = smd->geni;
	struct geni_se se = { .base = base, .dev = smd->dev };
	size_t tx_left = len, rx_left = len, tx_off = 0, rx_off = 0;
	const unsigned int bpw = 4;
	unsigned long deadline = jiffies + msecs_to_jiffies(200);
	u32 depth = smd->tx_fifo_depth ? smd->tx_fifo_depth : 16;
	u32 cfg;

	memset(rx, 0, len);

	if (readl_relaxed(base + SE_GENI_STATUS) & M_GENI_CMD_ACTIVE) {
		writel_relaxed(M_GENI_CMD_ABORT,
			       base + SE_GENI_M_CMD_CTRL_REG);
		udelay(50);
	}
	writel_relaxed(0xffffffff, base + SE_GENI_M_IRQ_CLEAR);

	/* Drain any stale RX words */
	while (readl_relaxed(base + SE_GENI_RX_FIFO_STATUS) & RX_FIFO_WC_MSK)
		readl_relaxed(base + SE_GENI_RX_FIFOn);

	cfg = readl_relaxed(base + SE_SPI_TRANS_CFG);
	cfg &= ~CS_TOGGLE;
	writel_relaxed(cfg, base + SE_SPI_TRANS_CFG);

	writel_relaxed(len & TRANS_LEN_MSK, base + SE_SPI_TX_TRANS_LEN);
	writel_relaxed(len & TRANS_LEN_MSK, base + SE_SPI_RX_TRANS_LEN);

	/* Match spi-geni-qcom: opcode = TX_ONLY|RX_ONLY, then fill TX FIFO */
	geni_se_setup_m_cmd(&se, SPI_FULL_DUPLEX, 0);

	while (tx_left || rx_left) {
		while (tx_left) {
			u32 wc = readl_relaxed(base + SE_GENI_TX_FIFO_STATUS) &
				 TX_FIFO_WC;
			u32 free;

			if (wc >= depth)
				break;
			for (free = depth - wc; free && tx_left; free--) {
				u32 word = 0;
				u8 *b = (u8 *)&word;
				unsigned int n = min_t(size_t, bpw, tx_left);
				unsigned int j;

				for (j = 0; j < n; j++)
					b[j] = tx[tx_off++];
				writel_relaxed(word, base + SE_GENI_TX_FIFOn);
				tx_left -= n;
			}
		}

		while (rx_left) {
			u32 rx_st = readl_relaxed(base + SE_GENI_RX_FIFO_STATUS);
			u32 rx_wc = rx_st & RX_FIFO_WC_MSK;
			unsigned int i;

			if (!rx_wc)
				break;
			for (i = 0; i < rx_wc && rx_left; i++) {
				u32 word = readl_relaxed(base + SE_GENI_RX_FIFOn);
				u8 *b = (u8 *)&word;
				unsigned int n = min_t(size_t, bpw, rx_left);
				unsigned int j;

				for (j = 0; j < n; j++)
					rx[rx_off++] = b[j];
				rx_left -= n;
			}
		}

		if ((readl_relaxed(base + SE_GENI_M_IRQ_STATUS) &
		     M_CMD_DONE_EN) && !tx_left) {
			/* Final RX drain after CMD_DONE */
			while (rx_left) {
				u32 rx_st = readl_relaxed(base +
							  SE_GENI_RX_FIFO_STATUS);
				u32 rx_wc = rx_st & RX_FIFO_WC_MSK;
				unsigned int i;

				if (!rx_wc)
					break;
				for (i = 0; i < rx_wc && rx_left; i++) {
					u32 word = readl_relaxed(base +
								 SE_GENI_RX_FIFOn);
					u8 *b = (u8 *)&word;
					unsigned int n = min_t(size_t, bpw,
							       rx_left);
					unsigned int j;

					for (j = 0; j < n; j++)
						rx[rx_off++] = b[j];
					rx_left -= n;
				}
			}
			if (!rx_left)
				break;
		}

		if (time_after(jiffies, deadline)) {
			writel_relaxed(M_GENI_CMD_ABORT,
				       base + SE_GENI_M_CMD_CTRL_REG);
			writel_relaxed(0xffffffff, base + SE_GENI_M_IRQ_CLEAR);
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	writel_relaxed(0xffffffff, base + SE_GENI_M_IRQ_CLEAR);
	return 0;
}

static int joypad_mcu_xfer(struct spi_mcu_data *smd)
{
	if (smd->geni)
		return joypad_geni_mmio_xfer(smd, smd->tx_buf, smd->rx_buf,
					     smd->buf_len);
	if (smd->spi)
		return spi_write_and_read_slt(smd->spi, smd->tx_buf,
					      smd->rx_buf, smd->buf_len);
	return -ENODEV;
}

static int joypad_spi_data_parse(u16 *lx, u16 *ly, u16 *rx, u16 *ry,
				 u16 *lt, u16 *rt)
{
	char *data;
	u16 crc16 = 0;
	int i, xret;

	if (!g_smd)
		return -ENODEV;

	data = g_smd->rx_buf;
	xret = joypad_mcu_xfer(g_smd);
	if (xret)
		return xret;

	/* Frame magic at [32..33], as the vendor driver requires. */
	if ((u8)data[32] != 0x5a || (u8)data[33] != 0x3a) {
		static int dumps;

		if (dumps < 4) {
			dumps++;
			dev_warn(g_smd->dev, "MCU SPI bad magic %02x%02x\n",
				 (u8)data[32], (u8)data[33]);
			print_hex_dump(KERN_WARNING, "mcu-spi: ",
				       DUMP_PREFIX_OFFSET, 16, 1,
				       data, min(g_smd->buf_len, 64), false);
		}
		return -EINVAL;
	}

	for (i = 34; i < (34 + 12); i++)
		crc16 += (u8)data[i];

	/*
	 * sum(data[34..45]) == data[46]|data[47]<<8. The counts stay visible
	 * via mcu_crc so a regression shows up as a rising err.
	 */
	if (((crc16 & 0xff) != (u8)data[46]) ||
	    ((crc16 >> 8) != (u8)data[47])) {
		g_smd->crc_err++;
		return -EINVAL;
	}
	g_smd->crc_ok++;

	*ly = ((u8)data[35] << 8) | (u8)data[34];
	*lx = ((u8)data[37] << 8) | (u8)data[36];
	*ry = ((u8)data[39] << 8) | (u8)data[38];
	*rx = ((u8)data[41] << 8) | (u8)data[40];
	*lt = ((u8)data[43] << 8) | (u8)data[42];
	*rt = ((u8)data[45] << 8) | (u8)data[44];

	*lx *= 8;
	*ly *= 8;
	*rx *= 8;
	*ry *= 8;
	*lt *= 8;
	*rt *= 8;
	return 0;
}

static ssize_t mcu_info_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	if (!g_smd || !g_smd->rx_buf)
		return sysfs_emit(buf, "000000\n");

	return sysfs_emit(buf, "%02x%02x%02x\n",
			  (u8)g_smd->rx_buf[61],
			  (u8)g_smd->rx_buf[62],
			  (u8)g_smd->rx_buf[63]);
}
static DEVICE_ATTR_RO(mcu_info);

static ssize_t mcu_frame_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	if (!g_smd || !g_smd->rx_buf)
		return -ENODEV;

	for (i = 0; i < g_smd->buf_len; i++)
		len += sysfs_emit_at(buf, len, "%02x%c", (u8)g_smd->rx_buf[i],
				     (i % 16 == 15) ? '\n' : ' ');
	return len;
}
static DEVICE_ATTR_RO(mcu_frame);

/* Accepted vs rejected frame counts, to keep an eye on the reject rate. */
static ssize_t mcu_crc_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	if (!g_smd)
		return -ENODEV;

	return sysfs_emit(buf, "ok %u err %u\n", g_smd->crc_ok, g_smd->crc_err);
}
static DEVICE_ATTR_RO(mcu_crc);

/*
 * The request the MCU is polled with. Writable so the report and LED fields
 * can be swept on the running device; write hex bytes, e.g. "5a 3a 66 66".
 * Bytes not covered by the write keep their current value.
 */
static ssize_t mcu_cmd_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	if (!g_smd || !g_smd->tx_buf)
		return -ENODEV;

	for (i = 0; i < g_smd->buf_len; i++)
		len += sysfs_emit_at(buf, len, "%02x%c", (u8)g_smd->tx_buf[i],
				     (i % 16 == 15) ? '\n' : ' ');
	return len;
}

static ssize_t mcu_cmd_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	const char *p = buf;
	int i = 0;

	if (!g_smd || !g_smd->tx_buf)
		return -ENODEV;

	while (i < g_smd->buf_len) {
		unsigned int val;
		int consumed;

		if (sscanf(p, "%2x%n", &val, &consumed) != 1)
			break;
		g_smd->tx_buf[i++] = val;
		p += consumed;
		while (*p == ' ' || *p == ',' || *p == '\n')
			p++;
	}

	dev_info(dev, "MCU cmd updated (%d bytes)\n", i);
	return count;
}
static DEVICE_ATTR_RW(mcu_cmd);

/*
 * geni_se_get_tx_fifo_depth() needs se->wrapper (QUP HW ver). We do not
 * probe the wrapper — read SE_HW_PARAM_0 (+ optional wrap@0x9c0000 VER).
 */
#define RG55_GENI_WRAP_PHYS	0x009c0000UL
#define RG55_GENI_WRAP_SIZE	0x2000UL
#define QUPV3_HW_VER_REG	0x4

static u32 joypad_geni_tx_fifo_depth(void __iomem *se_base)
{
	void __iomem *wrap;
	u32 hw = 0, mask, val;

	wrap = ioremap(RG55_GENI_WRAP_PHYS, RG55_GENI_WRAP_SIZE);
	if (wrap) {
		hw = readl_relaxed(wrap + QUPV3_HW_VER_REG);
		iounmap(wrap);
	}

	if ((GENI_SE_VERSION_MAJOR(hw) == 3 &&
	     GENI_SE_VERSION_MINOR(hw) >= 10) ||
	    GENI_SE_VERSION_MAJOR(hw) > 3)
		mask = TX_FIFO_DEPTH_MSK_256_BYTES;
	else
		mask = TX_FIFO_DEPTH_MSK;

	val = readl_relaxed(se_base + SE_HW_PARAM_0);
	return (val & mask) >> TX_FIFO_DEPTH_SHFT;
}

static int joypad_mmio_spi_init(struct device *dev)
{
	struct spi_mcu_data *smd;
	struct geni_se se = { };
	void __iomem *base;
	u32 proto, depth, width, fw_rev, hp0;

	if (g_smd)
		return 0;

	base = ioremap(RG55_GENI_SPI_PHYS, RG55_GENI_SPI_SIZE);
	if (!base)
		return -ENOMEM;

	/* Only base/dev — never call helpers that need se->wrapper. */
	se.base = base;
	se.dev = dev;
	fw_rev = readl_relaxed(base + GENI_FW_REVISION_RO);
	proto = geni_se_read_proto(&se);
	dev_info(dev, "GENI SE3 @0x%lx fw_rev=0x%x proto=%u (SPI=%u)\n",
		 (unsigned long)RG55_GENI_SPI_PHYS, fw_rev, proto, GENI_SE_SPI);

	if (proto != GENI_SE_SPI) {
		dev_warn(dev,
			 "SE3 not in SPI mode — sticks/LT/RT need SE firmware\n");
		iounmap(base);
		return -ENODEV;
	}

	hp0 = readl_relaxed(base + SE_HW_PARAM_0);
	depth = joypad_geni_tx_fifo_depth(base);
	width = (hp0 & TX_FIFO_WIDTH_MSK) >> TX_FIFO_WIDTH_SHFT;
	if (!depth)
		depth = 16;
	if (!width)
		width = 32;

	/* Prefer FIFO; keep BL packing if already sane, else configure 4x8 */
	geni_se_select_mode(&se, GENI_SE_FIFO);
	geni_se_config_packing(&se, 8, width / 8, true, true, true);
	geni_se_init(&se, depth - 3, depth - 2);

	writel_relaxed(0, base + SE_SPI_LOOPBACK);
	writel_relaxed(0, base + SE_SPI_DEMUX_SEL);
	/* Mode 3, mirroring spi->mode on the spi-geni path. */
	writel_relaxed(1, base + SE_SPI_CPHA);
	writel_relaxed(1, base + SE_SPI_CPOL);
	writel_relaxed((8 - MIN_WORD_LEN) & WORD_LEN_MSK,
		       base + SE_SPI_WORD_LEN);
	{
		u32 tcfg = readl_relaxed(base + SE_SPI_TRANS_CFG);

		tcfg &= ~CS_TOGGLE;
		writel_relaxed(tcfg, base + SE_SPI_TRANS_CFG);
	}
	/* SE @19.2 MHz → div 20 ≈ 960 kHz (stock ≤1.4 MHz). */
	writel_relaxed(0, base + SE_GENI_CLK_SEL);
	writel_relaxed((20 << CLK_DIV_SHFT) | SER_CLK_EN,
		       base + GENI_SER_M_CLK_CFG);

	smd = kzalloc(sizeof(*smd), GFP_KERNEL);
	if (!smd) {
		iounmap(base);
		return -ENOMEM;
	}
	smd->dev = dev;
	smd->geni = base;
	smd->mmio = true;
	smd->tx_fifo_depth = depth;
	smd->buf_len = 64;
	smd->rx_buf = kzalloc(smd->buf_len, GFP_KERNEL);
	smd->tx_buf = kzalloc(smd->buf_len, GFP_KERNEL);
	if (!smd->rx_buf || !smd->tx_buf) {
		kfree(smd->rx_buf);
		kfree(smd->tx_buf);
		kfree(smd);
		iounmap(base);
		return -ENOMEM;
	}
	memcpy(smd->tx_buf, joypad_mcu_tx_cmd, 64);
	smd->led_speed = JOYPAD_LED_SPEED_DEFAULT;
	joypad_led_apply(smd);
	g_smd = smd;

	{
		u16 lx, ly, rx, ry, lt, rt;
		int tret = joypad_spi_data_parse(&lx, &ly, &rx, &ry, &lt, &rt);

		if (!tret)
			dev_info(dev,
				 "MCU SPI frame OK lx=%u ly=%u rx=%u ry=%u lt=%u rt=%u\n",
				 lx, ly, rx, ry, lt, rt);
		else
			dev_warn(dev, "MCU SPI first frame err %d (will retry in poll)\n",
				 tret);
	}

	dev_info(dev, "MCU SPI MMIO ready (fifo_depth=%u width=%u hp0=0x%x)\n",
		 depth, width, hp0);
	return 0;
}

/*
 * Joystick RGB LEDs.
 *
 * The MCU drives the stick LEDs from fields in the 64-byte request frame we
 * poll it with. Layout recovered from the stock single_mcu_joy.ko
 * (custom_led_setting), offsets in the frame:
 *
 *   [7]  mode     1 = fixed colour from [11..13] (both sticks), 4 = MCU's
 *                 own rainbow, 5 = per-segment colours from [11..58]
 *                 (16 x RGB: slots 0..7 the left stick ring, 8..15 the
 *                 right); 2 = per-stick colour, 6 = stock's sync mode,
 *                 0 and 3 leave the LEDs dark
 *   [8]  switch   0 = LEDs off, 1 = on
 *   [9]  level    brightness 0..255 (sysfs takes 0..100 like stock)
 *   [10] speed    effect speed for the MCU-driven modes
 *   [11..13]      R, G, B for mode 1
 *   [59]  tag0    always 0
 *   [60]  tag1    incremented on every commit so the MCU sees a new command
 *   [61..62]      CRC-16/XMODEM (poly 0x1021, init 0) over [7..60], big endian
 *
 * The sysfs names (led_switch, led_mode, led_level, led_speed,
 * custum_rgb_{r,g,b}, led_set) keep the stock spelling so the same userspace
 * works on both; led_segments (16 RRGGBB hex values) is ours, for mode 5.
 * Values are staged and only land in the frame on a write to led_set, again
 * as stock does.
 */
#define JOYPAD_LED_OFF_MODE	7
#define JOYPAD_LED_OFF_SWITCH	8
#define JOYPAD_LED_OFF_LEVEL	9
#define JOYPAD_LED_OFF_SPEED	10
#define JOYPAD_LED_OFF_R	11
#define JOYPAD_LED_OFF_G	12
#define JOYPAD_LED_OFF_B	13
#define JOYPAD_LED_SEG_LEN	48
#define JOYPAD_LED_MODE_SEGMENTS 5
#define JOYPAD_LED_OFF_TAG0	59
#define JOYPAD_LED_OFF_TAG1	60
#define JOYPAD_LED_OFF_CRC	61
#define JOYPAD_LED_CRC_LEN	(JOYPAD_LED_OFF_CRC - JOYPAD_LED_OFF_MODE)

/* Write the current LED state into the frame; it goes out on the next poll. */
static void joypad_led_apply(struct spi_mcu_data *smd)
{
	u8 *f = (u8 *)smd->tx_buf;
	u16 crc;

	if (!f || smd->buf_len < 64)
		return;

	f[JOYPAD_LED_OFF_MODE] = smd->led_mode;
	f[JOYPAD_LED_OFF_SWITCH] = smd->led_switch;
	f[JOYPAD_LED_OFF_LEVEL] = smd->led_level * 255 / 100;
	f[JOYPAD_LED_OFF_SPEED] = smd->led_speed;
	if (smd->led_mode == JOYPAD_LED_MODE_SEGMENTS) {
		memcpy(f + JOYPAD_LED_OFF_R, smd->led_seg, JOYPAD_LED_SEG_LEN);
	} else {
		f[JOYPAD_LED_OFF_R] = smd->led_r;
		f[JOYPAD_LED_OFF_G] = smd->led_g;
		f[JOYPAD_LED_OFF_B] = smd->led_b;
		memset(f + JOYPAD_LED_OFF_B + 1, 0,
		       JOYPAD_LED_SEG_LEN - 3);
	}
	f[JOYPAD_LED_OFF_TAG0] = 0;
	f[JOYPAD_LED_OFF_TAG1] = ++smd->led_tag;

	crc = crc_itu_t(0, f + JOYPAD_LED_OFF_MODE, JOYPAD_LED_CRC_LEN);
	f[JOYPAD_LED_OFF_CRC] = crc >> 8;
	f[JOYPAD_LED_OFF_CRC + 1] = crc & 0xff;

	dev_dbg(smd->dev, "led apply sw=%u mode=%u lvl=%u spd=%u rgb=%u,%u,%u tag=%u crc=%04x\n",
		smd->led_switch, smd->led_mode, smd->led_level, smd->led_speed,
		smd->led_r, smd->led_g, smd->led_b, smd->led_tag, crc);
}

static ssize_t joypad_led_u8_store(u8 *field, unsigned int max,
				   const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (val > max)
		val = max;
	*field = val;
	return count;
}

#define JOYPAD_LED_ATTR(_name, _field, _max)				\
static ssize_t _name##_show(struct device *dev,				\
			    struct device_attribute *attr, char *buf)	\
{									\
	if (!g_smd)							\
		return -ENODEV;						\
	return sysfs_emit(buf, "%u\n", g_smd->_field);			\
}									\
static ssize_t _name##_store(struct device *dev,			\
			     struct device_attribute *attr,		\
			     const char *buf, size_t count)		\
{									\
	if (!g_smd)							\
		return -ENODEV;						\
	return joypad_led_u8_store(&g_smd->_field, _max, buf, count);	\
}									\
static DEVICE_ATTR_RW(_name)

JOYPAD_LED_ATTR(led_switch, led_switch, 1);
JOYPAD_LED_ATTR(led_mode, led_mode, 255);
JOYPAD_LED_ATTR(led_level, led_level, 100);
JOYPAD_LED_ATTR(led_speed, led_speed, 255);
JOYPAD_LED_ATTR(custum_rgb_r, led_r, 255);
JOYPAD_LED_ATTR(custum_rgb_g, led_g, 255);
JOYPAD_LED_ATTR(custum_rgb_b, led_b, 255);

/* 16 x RRGGBB (hex) for mode 5; missing trailing slots are cleared. */
static ssize_t led_segments_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	if (!g_smd)
		return -ENODEV;
	for (i = 0; i < JOYPAD_LED_SEG_LEN; i += 3)
		len += sysfs_emit_at(buf, len, "%02x%02x%02x%c",
				     g_smd->led_seg[i], g_smd->led_seg[i + 1],
				     g_smd->led_seg[i + 2],
				     i + 3 < JOYPAD_LED_SEG_LEN ? ' ' : '\n');
	return len;
}

static ssize_t led_segments_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	u8 seg[JOYPAD_LED_SEG_LEN] = { 0 };
	const char *p = buf;
	int i = 0;

	if (!g_smd)
		return -ENODEV;

	while (i < JOYPAD_LED_SEG_LEN) {
		unsigned int val;
		int consumed;

		while (*p == ' ' || *p == ',' || *p == '\n')
			p++;
		if (!*p)
			break;
		if (sscanf(p, "%6x%n", &val, &consumed) != 1 || consumed != 6)
			return -EINVAL;
		seg[i++] = val >> 16;
		seg[i++] = val >> 8;
		seg[i++] = val;
		p += consumed;
	}
	memcpy(g_smd->led_seg, seg, JOYPAD_LED_SEG_LEN);
	return count;
}
static DEVICE_ATTR_RW(led_segments);

/* Writing anything here commits the staged state to the frame. */
static ssize_t led_set_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	if (!g_smd)
		return -ENODEV;
	joypad_led_apply(g_smd);
	return count;
}
static DEVICE_ATTR_WO(led_set);

static struct attribute *joypad_attrs[] = {
	&dev_attr_mcu_info.attr,
	&dev_attr_mcu_frame.attr,
	&dev_attr_mcu_cmd.attr,
	&dev_attr_mcu_crc.attr,
	&dev_attr_led_switch.attr,
	&dev_attr_led_mode.attr,
	&dev_attr_led_level.attr,
	&dev_attr_led_speed.attr,
	&dev_attr_custum_rgb_r.attr,
	&dev_attr_custum_rgb_g.attr,
	&dev_attr_custum_rgb_b.attr,
	&dev_attr_led_segments.attr,
	&dev_attr_led_set.attr,
	NULL,
};
ATTRIBUTE_GROUPS(joypad);
#endif

static int joypad_parse_gpio_prop(struct device_node *np, const char *prop,
				  int *gpio, bool *active_low)
{
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_args(np, prop, "#gpio-cells", 0, &args);
	if (ret)
		return ret;

	*gpio = args.args[0];
	*active_low = args.args_count > 1 &&
		      (args.args[1] & GPIO_ACTIVE_LOW);
	of_node_put(args.np);

	if (*gpio < 0 || *gpio > TLMM_MAX_GPIO)
		return -EINVAL;
	return 0;
}

static int tlmm_read(struct joypad *jp, unsigned int gpio, u32 reg_off, u32 *val)
{
	phys_addr_t addr = RG55G1_TLMM_PHYS +
			   (phys_addr_t)gpio * TLMM_REG_SIZE + reg_off;

	if (jp->tlmm) {
		*val = readl_relaxed(jp->tlmm +
				     (unsigned long)gpio * TLMM_REG_SIZE +
				     reg_off);
		return 0;
	}
	return qcom_scm_io_readl(addr, val);
}

static int tlmm_write(struct joypad *jp, unsigned int gpio, u32 reg_off, u32 val)
{
	phys_addr_t addr = RG55G1_TLMM_PHYS +
			   (phys_addr_t)gpio * TLMM_REG_SIZE + reg_off;

	if (jp->tlmm) {
		writel_relaxed(val, jp->tlmm +
			       (unsigned long)gpio * TLMM_REG_SIZE + reg_off);
		return 0;
	}
	return qcom_scm_io_writel(addr, val);
}

static int joypad_tlmm_get(struct joypad *jp, struct bt_gpio *gpio)
{
	u32 val;
	int ret;

	if (!jp->gpio_live || gpio->gpio < 0)
		return -ENODEV;

	ret = tlmm_read(jp, gpio->gpio, 0x4, &val);
	if (ret)
		return ret;

	return !!(val & TLMM_IO_IN);
}

static void joypad_tlmm_cfg_input(struct joypad *jp, int gpio)
{
	u32 val;

	if (!jp->gpio_live || gpio < 0 || gpio > TLMM_MAX_GPIO)
		return;

	if (tlmm_read(jp, gpio, 0x0, &val))
		return;
	val &= ~(TLMM_CTL_MUX_MASK | TLMM_CTL_PULL_MASK | TLMM_CTL_OE);
	val |= TLMM_CTL_PULL_UP;
	tlmm_write(jp, gpio, 0x0, val);
}

static void joypad_tlmm_cfg_output_high(struct joypad *jp, int gpio)
{
	u32 ctl, io;

	if (!jp->gpio_live || gpio < 0 || gpio > TLMM_MAX_GPIO)
		return;

	if (tlmm_read(jp, gpio, 0x0, &ctl) || tlmm_read(jp, gpio, 0x4, &io))
		return;
	ctl &= ~(TLMM_CTL_MUX_MASK | TLMM_CTL_PULL_MASK);
	ctl |= TLMM_CTL_OE;
	io |= TLMM_IO_OUT;
	tlmm_write(jp, gpio, 0x4, io);
	tlmm_write(jp, gpio, 0x0, ctl);
}

static void joypad_gpio_check(struct joypad *joypad)
{
	int nbtn;
	bool changed = false;
	int hat0x = 0, hat0y = 0;

	if (!joypad->gpio_live)
		return;

	for (nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];
		int raw, pressed;

		raw = joypad_tlmm_get(joypad, gpio);
		if (raw < 0)
			continue;

		pressed = gpio->active_low ? !raw : raw;

		/* Aggregate ABS_HAT0* from all related GPIOs each poll. */
		if (gpio->abs_value) {
			if (pressed) {
				int dir = (gpio->abs_value == 1) ? 1 :
					  (gpio->abs_value == 2) ? -1 : 0;

				if (gpio->linux_code == ABS_HAT0X)
					hat0x = dir;
				else if (gpio->linux_code == ABS_HAT0Y)
					hat0y = dir;
			}
			gpio->old_value = pressed;
			continue;
		}

		if (pressed == gpio->old_value)
			continue;

		input_event(poll_dev_input, gpio->report_type,
			    gpio->linux_code, pressed ? 1 : 0);
		gpio->old_value = pressed;
		changed = true;
	}

	if (hat0x != joypad->hat0x) {
		input_report_abs(poll_dev_input, ABS_HAT0X, hat0x);
		joypad->hat0x = hat0x;
		changed = true;
	}
	if (hat0y != joypad->hat0y) {
		input_report_abs(poll_dev_input, ABS_HAT0Y, hat0y);
		joypad->hat0y = hat0y;
		changed = true;
	}

	if (changed)
		input_sync(poll_dev_input);
}

#ifdef SUPPORT_MCU_SPI_ADC
/*
 * Neither the sticks nor the triggers rest where the driver assumed: this
 * unit idles around 2062/1782/2046/2320 on the four stick axes, and the two
 * triggers rest at different points again. Average the first frames and use
 * that as each channel's zero. A stick reading far from the nominal centre
 * is one that was held while we sampled, so it keeps the nominal value.
 */
#define ADC_ZERO_SAMPLES	8
#define ADC_ZERO_MAX_OFFSET	(ADC_MAX_VOLTAGE / 8)

static void joypad_zero_calibrate(struct joypad *joypad)
{
	int count = min(joypad->amux_count, (int)ADC_CHANNEL_COUNT);
	int nbtn;

	if (joypad->zero_samples >= ADC_ZERO_SAMPLES)
		return;

	for (nbtn = 0; nbtn < count; nbtn++)
		joypad->zero_acc[nbtn] += joypad->adcs[nbtn].value;

	if (++joypad->zero_samples < ADC_ZERO_SAMPLES)
		return;

	for (nbtn = 0; nbtn < count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];
		int avg = joypad->zero_acc[nbtn] / ADC_ZERO_SAMPLES;

		if (nbtn == ADC_CHANNEL_Z || nbtn == ADC_CHANNEL_RZ)
			adc->cal = avg;
		else if (abs(avg - (ADC_MAX_VOLTAGE / 2)) <= ADC_ZERO_MAX_OFFSET)
			adc->cal = avg;

		dev_info(joypad->dev, "adc%d zero %d\n", nbtn, adc->cal);
	}
}

static void joypad_spi_check(struct joypad *joypad)
{
	short lx = 0, ly = 0, rx = 0, ry = 0, lt = 0, rt = 0;
	int nbtn;
	int ret;

	if (!g_smd)
		return;

	ret = joypad_spi_data_parse(&lx, &ly, &rx, &ry, &lt, &rt);
	if (ret) {
		/*
		 * Keep the last good reading rather than running the loop
		 * below over it again: that loop subtracts adc->cal, so
		 * reprocessing a stale value walks every axis away from
		 * centre one poll at a time.
		 */
		dev_warn_ratelimited(joypad->dev,
				     "MCU SPI frame err %d\n", ret);
		return;
	}

	joypad->adcs[ADC_CHANNEL_LX].value = lx;
	joypad->adcs[ADC_CHANNEL_LY].value = ly;
	joypad->adcs[ADC_CHANNEL_RX].value = rx;
	joypad->adcs[ADC_CHANNEL_RY].value = ry;
	joypad->adcs[ADC_CHANNEL_Z].value = lt;
	joypad->adcs[ADC_CHANNEL_RZ].value = rt;

	joypad_zero_calibrate(joypad);

	for (nbtn = 0; nbtn < joypad->amux_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];

		adc->value = adc->value - adc->cal;

		if (nbtn == ADC_CHANNEL_LX || nbtn == ADC_CHANNEL_LY ||
		    nbtn == ADC_CHANNEL_RX || nbtn == ADC_CHANNEL_RY) {
			if (adc->value <= 0) {
				if ((-adc->value) <= joypad->bt_adc_deadzone)
					adc->value = 0;
			} else if (adc->value <= joypad->bt_adc_deadzone) {
				adc->value = 0;
			}
		} else {
			if (adc->value < 0)
				adc->value = -adc->value;
			if (adc->value <= joypad->bt_adc_deadzone)
				adc->value = 0;
		}

		if (adc->tuning_n && adc->value < 0)
			adc->value = ADC_DATA_TUNING(adc->value, adc->tuning_n);
		if (adc->tuning_p && adc->value > 0)
			adc->value = ADC_DATA_TUNING(adc->value, adc->tuning_p);

		if (adc->value >= 0) {
			if (adc->value >= adc->max)
				adc->value = adc->max;
		} else if (adc->value <= -adc->max) {
			adc->value = -adc->max;
		}

		if (adc->value != adc->old_value)
			input_report_abs(poll_dev_input, adc->report_type,
					 adc->invert ? -adc->value : adc->value);
		adc->old_value = adc->value;
	}
	input_sync(poll_dev_input);
}
#else
static void joypad_spi_check(struct joypad *joypad) { }
#endif

static void joypad_poll(struct input_dev *gamepad)
{
	struct joypad *joypad = input_get_drvdata(gamepad);

	if (!joypad || !joypad->enable)
		return;

	joypad_gpio_check(joypad);
	joypad_spi_check(joypad);
}

static void joypad_open_enable(struct joypad *joypad)
{
	int nbtn;

	for (nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++)
		joypad->gpios[nbtn].old_value = 0;
	joypad->hat0x = 0;
	joypad->hat0y = 0;

	mutex_lock(&joypad->lock);
	joypad->enable = true;
	mutex_unlock(&joypad->lock);

	dev_info(joypad->dev, "joypad enabled (gpio=%s spi=%s)\n",
		 joypad->gpio_live ? (joypad->tlmm ? "mmio" : "scm") : "off",
#ifdef SUPPORT_MCU_SPI_ADC
		 g_smd ? (g_smd->mmio ? "mmio" : "on") : "off");
#else
		 "n/a");
#endif
}

static int joypad_input_open(struct input_dev *dev)
{
	struct joypad *joypad = input_get_drvdata(dev);

	joypad_open_enable(joypad);
	return 0;
}

static void joypad_input_close(struct input_dev *dev)
{
	struct joypad *joypad = input_get_drvdata(dev);

	mutex_lock(&joypad->lock);
	joypad->enable = false;
	mutex_unlock(&joypad->lock);
}

static int joypad_adc_setup(struct device *dev, struct joypad *joypad)
{
	int nbtn;

	joypad->adcs = devm_kcalloc(dev, joypad->amux_count,
				    sizeof(*joypad->adcs), GFP_KERNEL);
	if (!joypad->adcs)
		return -ENOMEM;

	for (nbtn = 0; nbtn < joypad->amux_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];

		adc->scale = joypad->bt_adc_scale;
		adc->max = ADC_MAX_VOLTAGE / 2;
		adc->min = (nbtn == ADC_CHANNEL_Z || nbtn == ADC_CHANNEL_RZ) ?
			   0 : -(ADC_MAX_VOLTAGE / 2);
		/*
		 * MCU sticks idle at ~0x0800, then *=8 → 16384 (= ADC_MAX/2).
		 * Subtract that mid-point so ABS_X/Y/RX/RY rest at 0 (center
		 * of [-16384, 16384]). The triggers rest wherever the hardware
		 * puts them, so their cal is measured from the first frame.
		 */
		adc->cal = (nbtn == ADC_CHANNEL_Z || nbtn == ADC_CHANNEL_RZ) ?
			   0 : (ADC_MAX_VOLTAGE / 2);
		if (adc->scale) {
			adc->max *= adc->scale;
			adc->min *= adc->scale;
		}

		switch (nbtn) {
		case ADC_CHANNEL_RY:
			adc->report_type = ABS_RY;
			if (device_property_read_u32(dev, "abs_ry-p-tuning",
						     &adc->tuning_p))
				adc->tuning_p = ADC_TUNING_DEFAULT;
			if (device_property_read_u32(dev, "abs_ry-n-tuning",
						     &adc->tuning_n))
				adc->tuning_n = ADC_TUNING_DEFAULT;
			break;
		case ADC_CHANNEL_RX:
			adc->report_type = ABS_RX;
			if (device_property_read_u32(dev, "abs_rx-p-tuning",
						     &adc->tuning_p))
				adc->tuning_p = ADC_TUNING_DEFAULT;
			if (device_property_read_u32(dev, "abs_rx-n-tuning",
						     &adc->tuning_n))
				adc->tuning_n = ADC_TUNING_DEFAULT;
			break;
		case ADC_CHANNEL_LY:
			adc->report_type = ABS_Y;
			if (device_property_read_u32(dev, "abs_y-p-tuning",
						     &adc->tuning_p))
				adc->tuning_p = ADC_TUNING_DEFAULT;
			if (device_property_read_u32(dev, "abs_y-n-tuning",
						     &adc->tuning_n))
				adc->tuning_n = ADC_TUNING_DEFAULT;
			break;
		case ADC_CHANNEL_LX:
			adc->report_type = ABS_X;
			if (device_property_read_u32(dev, "abs_x-p-tuning",
						     &adc->tuning_p))
				adc->tuning_p = ADC_TUNING_DEFAULT;
			if (device_property_read_u32(dev, "abs_x-n-tuning",
						     &adc->tuning_n))
				adc->tuning_n = ADC_TUNING_DEFAULT;
			break;
		case ADC_CHANNEL_Z:
			adc->report_type = ABS_Z;
			if (device_property_read_u32(dev, "abs_z-p-tuning",
						     &adc->tuning_p))
				adc->tuning_p = ADC_TUNING_DEFAULT;
			if (device_property_read_u32(dev, "abs_z-n-tuning",
						     &adc->tuning_n))
				adc->tuning_n = ADC_TUNING_DEFAULT;
			break;
		case ADC_CHANNEL_RZ:
			adc->report_type = ABS_RZ;
			if (device_property_read_u32(dev, "abs_rz-p-tuning",
						     &adc->tuning_p))
				adc->tuning_p = ADC_TUNING_DEFAULT;
			if (device_property_read_u32(dev, "abs_rz-n-tuning",
						     &adc->tuning_n))
				adc->tuning_n = ADC_TUNING_DEFAULT;
			break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

static int joypad_power_setup(struct device *dev, struct joypad *joypad)
{
	static const char *const names[] = {
		"hall-power-ctl0-gpios",
		"mcu-3v3-power-ctl1-gpios",
		"rgb-power-gpios",
		"adc-power-ctl0-gpios",
		"adc-power-ctl1-gpios",
		"adc-power-ctl2-gpios",
		"adc-power-ctl3-gpios",
		"adc-power-ctl4-gpios",
	};
	static const char *const supplies[] = { "vdd", "hall", "rgb" };
	struct device_node *np = dev->of_node;
	int i, n = 0;

	/*
	 * Boards that model the rails as fixed regulators instead of the stock
	 * *-power-ctl*-gpios properties must hold them enabled, or the core
	 * switches the unclaimed rails off once late init completes and the MCU
	 * dies a few seconds after boot.
	 */
	for (i = 0; i < ARRAY_SIZE(supplies); i++) {
		int ret = devm_regulator_get_enable_optional(dev, supplies[i]);

		if (ret < 0 && ret != -ENODEV)
			return dev_err_probe(dev, ret, "failed to enable %s supply\n",
					     supplies[i]);
	}

	if (!np)
		return 0;

	for (i = 0; i < ARRAY_SIZE(names) && n < ARRAY_SIZE(joypad->pwr_gpio_pins); i++) {
		int gpio;
		bool active_low;

		if (joypad_parse_gpio_prop(np, names[i], &gpio, &active_low))
			continue;

		joypad->pwr_gpio_pins[n++] = gpio;
		if (joypad->gpio_live)
			joypad_tlmm_cfg_output_high(joypad, gpio);
	}
	joypad->pwr_gpio_count = n;
	if (n)
		dev_info(dev, "power gpios=%d (drive %s)\n", n,
			 joypad->gpio_live ? "on" : "skip");
	return 0;
}

static int joypad_gpio_setup(struct device *dev, struct joypad *joypad)
{
	struct device_node *node = dev->of_node, *pp;
	int nbtn = 0;

	if (!node)
		return -ENODEV;

	joypad->gpios = devm_kcalloc(dev, joypad->bt_gpio_count,
				     sizeof(*joypad->gpios), GFP_KERNEL);
	if (!joypad->gpios)
		return -ENOMEM;

	for_each_child_of_node(node, pp) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];
		bool active_low = false;
		int ret;

		gpio->gpio = -1;
		gpio->label = of_get_property(pp, "label", NULL);

		ret = joypad_parse_gpio_prop(pp, "gpios", &gpio->gpio,
					     &active_low);
		if (ret) {
			dev_warn(dev, "skip button %s: no gpios (%d)\n",
				 gpio->label ?: "?", ret);
			continue;
		}
		gpio->active_low = active_low;

		if (of_property_read_u32(pp, "linux,code", &gpio->linux_code)) {
			dev_err(dev, "button without linux,code\n");
			of_node_put(pp);
			return -EINVAL;
		}
		if (of_property_read_u32(pp, "linux,input-type",
					 &gpio->report_type))
			gpio->report_type = EV_KEY;
		if (of_property_read_u32(pp, "linux,abs-value",
					 &gpio->abs_value))
			gpio->abs_value = 0;

		/* Configure as GPIO input with pull-up when SCM works. */
		if (joypad->gpio_live)
			joypad_tlmm_cfg_input(joypad, gpio->gpio);
		nbtn++;
	}

	joypad->bt_gpio_count = nbtn;
	dev_info(dev, "parsed %d button gpios\n", nbtn);
	return 0;
}

static int joypad_input_setup(struct device *dev, struct joypad *joypad)
{
	struct input_dev *input;
	int nbtn, error;
	u32 joypad_revision = 0;
	u32 joypad_product = 0;
	const char *name = DRV_NAME;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	poll_dev_input = input;
	joypad->input = input;
	input_set_drvdata(input, joypad);

	device_property_read_string(dev, "joypad-name", &name);
	input->name = name;
	input->phys = DRV_NAME "/input0";
	input->open = joypad_input_open;
	input->close = joypad_input_close;

	device_property_read_u32(dev, "joypad-revision", &joypad_revision);
	device_property_read_u32(dev, "joypad-product", &joypad_product);
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x484B;
	input->id.product = (u16)joypad_product;
	input->id.version = (u16)joypad_revision;

	__set_bit(EV_ABS, input->evbit);
	for (nbtn = 0; nbtn < joypad->amux_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];
		int fuzz = joypad->bt_adc_fuzz;

		if (nbtn == ADC_CHANNEL_Z || nbtn == ADC_CHANNEL_RZ)
			fuzz = 320;
		input_set_abs_params(input, adc->report_type,
				     adc->min, adc->max,
				     fuzz, joypad->bt_adc_flat);
	}

	__set_bit(EV_KEY, input->evbit);
	/*
	 * Do NOT set INPUT_PROP_DIRECT. udev/input_id treats DIRECT+ABS as a
	 * touchscreen (ID_INPUT_TOUCHSCREEN), so EmulationStation/SDL never
	 * see this device as a joystick.
	 */
	for (nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];

		if (gpio->report_type == EV_ABS)
			input_set_abs_params(input, gpio->linux_code,
					     -1, 1, 0, 0);
		else
			input_set_capability(input, gpio->report_type,
					     gpio->linux_code);
	}

	if (joypad->auto_repeat)
		__set_bit(EV_REP, input->evbit);

	error = input_setup_polling(input, joypad_poll);
	if (error)
		return error;

	input_set_poll_interval(input, joypad->poll_interval);

	error = input_register_device(input);
	if (error)
		return error;

	return 0;
}

static void joypad_setup_value_check(struct device *dev, struct joypad *joypad)
{
	if (g_button_adc_fuzz)
		joypad->bt_adc_fuzz = g_button_adc_fuzz;
	else
		device_property_read_u32(dev, "button-adc-fuzz",
					 &joypad->bt_adc_fuzz);

	if (g_button_adc_flat)
		joypad->bt_adc_flat = g_button_adc_flat;
	else
		device_property_read_u32(dev, "button-adc-flat",
					 &joypad->bt_adc_flat);

	if (g_button_adc_scale)
		joypad->bt_adc_scale = g_button_adc_scale;
	else
		device_property_read_u32(dev, "button-adc-scale",
					 &joypad->bt_adc_scale);

	if (g_button_adc_deadzone)
		joypad->bt_adc_deadzone = g_button_adc_deadzone;
	else
		device_property_read_u32(dev, "button-adc-deadzone",
					 &joypad->bt_adc_deadzone);
}

static int joypad_dt_parse(struct device *dev, struct joypad *joypad)
{
	int error;

	joypad_setup_value_check(dev, joypad);

	if (device_property_read_u32(dev, "amux-count", &joypad->amux_count))
		joypad->amux_count = ADC_CHANNEL_COUNT;

	if (device_property_read_u32(dev, "poll-interval",
				     &joypad->poll_interval))
		joypad->poll_interval = 10;

	joypad->auto_repeat = device_property_present(dev, "autorepeat");
	joypad->bt_gpio_count = device_get_child_node_count(dev);

	error = joypad_adc_setup(dev, joypad);
	if (error)
		return error;

	error = joypad_power_setup(dev, joypad);
	if (error)
		return error;

	error = joypad_gpio_setup(dev, joypad);
	if (error)
		return error;

	dev_info(dev, "adc axes=%d gpio keys=%d poll=%dms\n",
		 joypad->amux_count, joypad->bt_gpio_count,
		 joypad->poll_interval);
	return 0;
}

static int joypad_probe(struct platform_device *pdev)
{
	struct joypad *joypad;
	struct device *dev = &pdev->dev;
	int error;
	u32 probe_val = 0;

	joypad = devm_kzalloc(dev, sizeof(*joypad), GFP_KERNEL);
	if (!joypad)
		return -ENOMEM;

	joypad->dev = dev;
	poll_dev_private = joypad;
	mutex_init(&joypad->lock);

	/*
	 * Prefer direct MMIO at mainline TLMM base 0x0f100000. Stock DT
	 * lists 0x0f000000 (wrong) — that faulted; SCM_IO also rejects it.
	 */
	joypad->tlmm = ioremap(RG55G1_TLMM_PHYS, RG55G1_TLMM_SIZE);
	if (joypad->tlmm) {
		u32 v = readl_relaxed(joypad->tlmm + 8 * TLMM_REG_SIZE + 0x4);

		joypad->gpio_live = true;
		probe_val = v;
		dev_info(dev, "TLMM MMIO ok @0x%lx (gpio8_in=0x%x)\n",
			 (unsigned long)RG55G1_TLMM_PHYS, probe_val);
	} else {
		int ret = tlmm_read(joypad, 8, 0x4, &probe_val);

		if (!ret) {
			joypad->gpio_live = true;
			dev_info(dev, "TLMM SCM ok (gpio8_in=0x%x)\n",
				 probe_val);
		} else {
			dev_warn(dev, "TLMM unavailable (mmio/scm=%d); buttons off\n",
				 ret);
		}
	}

	error = joypad_dt_parse(dev, joypad);
	if (error)
		return error;

#ifdef SUPPORT_MCU_SPI_ADC
	/* MCU needs 3V3 (gpio95) from power_setup; give it time to boot. */
	msleep(30);
	if (joypad_mmio_spi_init(dev))
		dev_warn(dev, "MCU SPI MMIO init failed; LT/RT/sticks off\n");
#endif

	platform_set_drvdata(pdev, joypad);

	error = joypad_input_setup(dev, joypad);
	if (error)
		return error;

	joypad_open_enable(joypad);

	dev_info(dev, "singleadc-joypad probe success (%s)\n",
		 joypad->input->name);
	return 0;
}

#ifdef SUPPORT_MCU_SPI_ADC
static int sprd_spi_mcu_probe(struct spi_device *spi)
{
	struct spi_mcu_data *smd;
	int ret;

	if (!spi->dev.of_node)
		return -ENODEV;

	/* Prefer already-running MMIO path; avoid double ownership. */
	if (g_smd && g_smd->mmio) {
		dev_info(&spi->dev, "MCU already via MMIO; skip spi client\n");
		return 0;
	}

	smd = kzalloc(sizeof(*smd), GFP_KERNEL);
	if (!smd)
		return -ENOMEM;

	spi->bits_per_word = 8;
	/*
	 * Mode 3, matching the vendor spi_mcu_driver.ko. Modes 0 and 3 both
	 * sample on the rising edge and differ only in SCLK idle level, so
	 * mode 0 runs but corrupts single bits.
	 */
	spi->mode = SPI_MODE_3;
	if (of_property_read_u32(spi->dev.of_node, "spi-max-frequency",
				 &spi->max_speed_hz))
		spi->max_speed_hz = 1000000;

	smd->spi = spi;
	smd->dev = &spi->dev;
	g_smd = smd;

	ret = spi_setup(spi);
	if (ret < 0) {
		kfree(smd);
		g_smd = NULL;
		return ret;
	}

	if (of_property_read_u32(spi->dev.of_node, "buf-len", &smd->buf_len))
		smd->buf_len = 64;

	smd->rx_buf = kzalloc(smd->buf_len, GFP_KERNEL);
	smd->tx_buf = kzalloc(smd->buf_len, GFP_KERNEL);
	if (!smd->rx_buf || !smd->tx_buf) {
		kfree(smd->rx_buf);
		kfree(smd->tx_buf);
		kfree(smd);
		g_smd = NULL;
		return -ENOMEM;
	}
	memcpy(smd->tx_buf, joypad_mcu_tx_cmd, min_t(int, smd->buf_len, 64));
	/* LEDs stay off until userspace commits a state, as on stock */
	smd->led_speed = JOYPAD_LED_SPEED_DEFAULT;
	joypad_led_apply(smd);

	spi_set_drvdata(spi, smd);

	if (poll_dev_private)
		joypad_open_enable(poll_dev_private);

	dev_info(&spi->dev, "MCU SPI joystick probe success\n");
	return 0;
}

static void sprd_spi_mcu_remove(struct spi_device *spi)
{
	struct spi_mcu_data *smd = spi_get_drvdata(spi);

	if (smd == g_smd)
		g_smd = NULL;
	if (smd) {
		kfree(smd->rx_buf);
		kfree(smd->tx_buf);
		kfree(smd);
	}
}

static const struct of_device_id sprd_spi_mcu_dt_match[] = {
	{ .compatible = "mcu,spi_joystick" },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_spi_mcu_dt_match);

static struct spi_driver spi_mcu_joy_driver = {
	.driver = {
		.name = "spi_mcu",
		.of_match_table = sprd_spi_mcu_dt_match,
	},
	.probe = sprd_spi_mcu_probe,
	.remove = sprd_spi_mcu_remove,
};
#endif

static const struct of_device_id joypad_of_match[] = {
	{ .compatible = "singleadc-joypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, joypad_of_match);

static struct platform_driver joypad_driver = {
	.probe = joypad_probe,
	.driver = {
		.name = "singleadc-joypad",
		.of_match_table = joypad_of_match,
#ifdef SUPPORT_MCU_SPI_ADC
		.dev_groups = joypad_groups,
#endif
	},
};

static int __init joypad_driver_init(void)
{
	int ret;

#ifdef SUPPORT_MCU_SPI_ADC
	ret = spi_register_driver(&spi_mcu_joy_driver);
	if (ret)
		return ret;
#endif
	ret = platform_driver_register(&joypad_driver);
	if (ret) {
#ifdef SUPPORT_MCU_SPI_ADC
		spi_unregister_driver(&spi_mcu_joy_driver);
#endif
		return ret;
	}
	return 0;
}

static void __exit joypad_driver_exit(void)
{
#ifdef SUPPORT_MCU_SPI_ADC
	spi_unregister_driver(&spi_mcu_joy_driver);
#endif
	platform_driver_unregister(&joypad_driver);
}

module_init(joypad_driver_init);
module_exit(joypad_driver_exit);

MODULE_DESCRIPTION("Anbernic single-ADC / MCU SPI gamepad driver");
MODULE_AUTHOR("Anbernic RG 55G1 port");
MODULE_LICENSE("GPL");
