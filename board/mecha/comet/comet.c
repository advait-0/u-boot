#include <env.h>
#include <efi_loader.h>
#include <init.h>
#include <fdt_support.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <asm/mach-imx/boot_mode.h>
#include <linux/delay.h>
#include <power/regulator.h>
#include <scmi_agent.h>
#include "../dts/upstream/src/arm64/freescale/imx95-power.h"
#include <i2c.h>
#include <asm/arch/sys_proto.h>
#include <dm/uclass.h>
#include <dm/uclass-internal.h>

#ifdef CONFIG_USB_TCPC
#include "../../nxp/common/tcpc.h"
#endif

extern int board_fix_fdt_fuse(void *fdt);

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
#define IMX_BOOT_IMAGE_GUID \
	EFI_GUID(0x2c4db6b3, 0x0b15, 0x4a36, 0xbe, 0xae, \
		 0x1e, 0xa1, 0x35, 0x46, 0x4f, 0x5b)

struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"IMX95-EVK-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 0=flash-bin raw 0 0x2000 mmcpart 1",
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};
#endif /* EFI_HAVE_CAPSULE_SUPPORT */

static int imx9_scmi_power_domain_enable(u32 domain, bool enable)
{
	struct udevice *dev;
	int ret;

	/* The SCMI clock device is only borrowed as a handle to the SCMI channel */
	ret = uclass_get_device_by_name(UCLASS_CLK, "protocol@14", &dev);
	if (ret)
		return ret;

	return scmi_pwd_state_set(dev, 0, domain, enable ? 0 : BIT(30));
}

/*
 * ---------------------------------------------------------------------------
 * USB Type-C / USB-PD (DISABLED - power)
 *
 * The Type-C controller in DRP mode keeps toggling CC, the PD sink path turns
 * on the external 12 V rail, and host mode sources VBUS to attached devices.
 *
 * WARNING: if the Comet is powered over USB-PD, the PD negotiation below may
 * be what gets it above 5 V. Only leave this off if the board's supply does
 * not depend on it. Also set CONFIG_USB_TCPC=n in the defconfig.
 * ---------------------------------------------------------------------------
 */
#if 0
struct tcpc_port port;
struct tcpc_port portpd;
struct tcpc_port_config port_config = {
	.i2c_bus = 2, /* i2c3 */
	.addr = 0x50,
	.port_type = TYPEC_PORT_DRP,
	.disable_pd = true,
};

struct tcpc_port_config portpd_config = {
	.i2c_bus = 2, /* i2c3 */
	.addr = 0x52,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 20000,
	.max_snk_ma = 3000,
	.max_snk_mw = 15000,
	.op_snk_mv = 9000,
};

ulong tca_base;

void tca_mux_select(enum typec_cc_polarity pol)
{
	if (!tca_base)
		return;

	/* Set OP mode to System configure Mode */
	clrbits_le32(tca_base + 0x10, 0x3);

	setbits_le32(tca_base + 0x18, BIT(3));
	udelay(1);

	if (pol == TYPEC_POLARITY_CC1)
		clrbits_le32(tca_base + 0x18, BIT(2));
	else
		setbits_le32(tca_base + 0x18, BIT(2));

	udelay(1);

	clrbits_le32(tca_base + 0x18, BIT(3));
}

static void setup_typec(void)
{
	struct gpio_desc ext_12v_desc;
	int ret;

	tca_base = USB1_BASE_ADDR + 0xfc000;

	ret = tcpc_init(&portpd, portpd_config, NULL);
	if (ret) {
		printf("%s: tcpc portpd init failed, err=%d\n", __func__, ret);
	} else if (tcpc_pd_sink_check_charging(&portpd)) {
		printf("Power supply on USB PD\n");

		/* Enable EXT 12V */
		ret = dm_gpio_lookup_name("gpio@22_1", &ext_12v_desc);
		if (ret)
			return;

		ret = dm_gpio_request(&ext_12v_desc, "ext_12v_en");
		if (ret)
			return;

		dm_gpio_set_dir_flags(&ext_12v_desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	}

	ret = tcpc_init(&port, port_config, &tca_mux_select);
	if (ret)
		printf("%s: tcpc init failed, err=%d\n", __func__, ret);
}

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	if (index == 0 && init == USB_INIT_DEVICE)
		ret = tcpc_setup_ufp_mode(&port);
	else if (index == 0 && init == USB_INIT_HOST)
		ret = tcpc_setup_dfp_mode(&port);

	return ret;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	if (index == 0 && init == USB_INIT_HOST)
		return tcpc_disable_src_vbus(&port);

	return 0;
}
#endif /* USB Type-C */

/*
 * ---------------------------------------------------------------------------
 * Ethernet / NETC (DISABLED - power)
 *
 * Releasing the PHYs from reset leaves them powered and auto-negotiating
 * (tens to hundreds of mW each) all the way into Linux. If Linux needs
 * Ethernet, describe the PHY reset GPIOs (reset-gpios) in the Linux device
 * tree and let the Linux driver handle them.
 * ---------------------------------------------------------------------------
 */
#if 0
static void netc_phy_rst(const char *gpio_name, const char *label)
{
	struct gpio_desc desc;
	int ret;

	ret = dm_gpio_lookup_name(gpio_name, &desc);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, gpio_name, ret);
		return;
	}

	ret = dm_gpio_request(&desc, label);
	if (ret) {
		printf("%s request %s failed ret = %d\n", __func__, label, ret);
		return;
	}

	/* assert ENET_RST_B, then release it */
	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);
	udelay(10000);
	dm_gpio_set_value(&desc, 0);
	udelay(80000);
}

void netc_init(void)
{
	int ret;

	imx9_scmi_power_domain_enable(IMX95_PD_NETC, false);
	udelay(10000);

	/* Power up the NETC MIX */
	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, true);
	if (ret) {
		printf("SCMI_POWER_STATE_SET failed for NETC MIX\n");
		return;
	}

	netc_phy_rst("gpio@22_4", "ENET1_RST_B");
	netc_phy_rst("gpio@22_5", "ENET2_RST_B");
}
#endif /* Ethernet */

/*
 * ---------------------------------------------------------------------------
 * M.2 slot power / FlexSPI NOR on M.2 (DISABLED - power)
 *
 * Turns on the M.2 3V3 rail and 1.8 V LDO and releases the slot reset. These
 * stay on into Linux. Re-enable only if you boot from or use the M.2 slot.
 * ---------------------------------------------------------------------------
 */
#if 0
static void flexspi_nor_setup(void)
{
	struct gpio_desc desc;
	int ret;

	/* Power cycle the M2 3V3 */
	ret = dm_gpio_lookup_name("gpio@22_10", &desc);
	if (ret)
		return;
	ret = dm_gpio_request(&desc, "M2_PWREN");
	if (ret)
		return;
	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);
	udelay(100000);
	dm_gpio_set_value(&desc, 1);

	/* Enable 1.8V LDO */
	ret = dm_gpio_lookup_name("gpio@22_11", &desc);
	if (ret)
		return;
	ret = dm_gpio_request(&desc, "M2_DIS1");
	if (ret)
		return;
	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 1);

	/* Deassert M2_SD3_nRST */
	ret = dm_gpio_lookup_name("GPIO5_9", &desc);
	if (ret)
		return;
	ret = dm_gpio_request(&desc, "M2_SD3_nRST");
	if (ret)
		return;
	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 1);
}
#endif /* M.2 */

/*
 * ---------------------------------------------------------------------------
 * LVDS backlight (DISABLED - power)
 *
 * Drives the PCA9632 LED outputs fully on, and they stay lit until a Linux
 * driver reprograms the chip - even though the DISPLAY mix is off here.
 * ---------------------------------------------------------------------------
 */
#if 0
void lvds_backlight_on(void)
{
	struct udevice *dev;
	u8 reg;

	if (i2c_get_chip_for_busnum(2, 0x62, 1, &dev)) {
		printf("%s: Cannot find pca9632 led dev\n", __func__);
		return;
	}

	reg = 1;
	dm_i2c_write(dev, 0x1, &reg, 1);

	reg = 5;
	dm_i2c_write(dev, 0x8, &reg, 1);
}
#endif /* backlight */

int board_init(void)
{
	/*
	 * Request every mix U-Boot does not need OFF. Failures are reported
	 * but never fatal: a failed OFF request must not stop the boot.
	 *
	 * HSIO_TOP covers USB and PCIe. With it off, U-Boot cannot use USB
	 * (fastboot/UUU/usb storage) or PCIe/NVMe. Turn it back ON here if
	 * you need those in U-Boot - and see the defconfig notes about
	 * CONFIG_PCI_INIT_R.
	 */
    	/*
	 * Keep USB (HSIO) powered when loaded over USB, so UUU/fastboot
	 * works. Normal SD/eMMC boots still get it turned off.
	 */
	if (get_boot_device() == USB_BOOT) {
		if (imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, true))
			printf("%s: HSIO MIX on request failed\n", __func__);
	} else {
		if (imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, false))
			printf("%s: HSIO MIX off request failed\n", __func__);
	}
	if (imx9_scmi_power_domain_enable(IMX95_PD_NETC, false))
		printf("%s: NETC MIX off request failed\n", __func__);
	if (imx9_scmi_power_domain_enable(IMX95_PD_DISPLAY, false))
		printf("%s: DISPLAY MIX off request failed\n", __func__);
	if (imx9_scmi_power_domain_enable(IMX95_PD_CAMERA, false))
		printf("%s: CAMERA MIX off request failed\n", __func__);

	/* setup_typec();		USB Type-C / PD    - disabled, see above */
	/* netc_init();			Ethernet PHYs      - disabled, see above */
	/* flexspi_nor_setup();		M.2 rails          - disabled, see above */
	/* power_on_m7("mx95evkrpmsg");	M7 - no M7 image is packed in flash_a55 */
	/* lvds_backlight_on();		LVDS backlight     - disabled, see above */

	return 0;
}

/*
 * No power cost: picks the MMC device the environment is loaded from, based
 * on the actual boot device. Required because CONFIG_BOARD_LATE_INIT=y.
 */
int board_late_init(void)
{
	if (IS_ENABLED(CONFIG_ENV_IS_IN_MMC))
		board_late_mmc_env_init();

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	char *p, *b, *s;
	char *token = NULL;
	int i, ret = 0;
	u64 base[CONFIG_NR_DRAM_BANKS] = {0};
	u64 size[CONFIG_NR_DRAM_BANKS] = {0};

	p = env_get("jh_root_mem");
	if (!p)
		return 0;

	i = 0;
	token = strtok(p, ",");
	while (token) {
		if (i >= CONFIG_NR_DRAM_BANKS) {
			printf("Error: The number of size@base exceeds CONFIG_NR_DRAM_BANKS.\n");
			return -EINVAL;
		}

		b = token;
		s = strsep(&b, "@");
		if (!s) {
			printf("The format of jh_root_mem is size@base[,size@base...].\n");
			return -EINVAL;
		}
		base[i] = simple_strtoull(b, NULL, 16);
		size[i] = simple_strtoull(s, NULL, 16);
		token = strtok(NULL, ",");
		i++;
	}

	ret = fdt_fixup_memory_banks(blob, base, size, CONFIG_NR_DRAM_BANKS);
	if (ret)
		return ret;

	return 0;
}
#endif

/*
 * Called right before jumping to the OS. Tear down whatever U-Boot used and
 * make sure every optional mix is off. No early returns: one failed request
 * must not leave the rest powered.
 */
void board_quiesce_devices(void)
{
	struct uclass *uc_dev;
	int ret;

	ret = uclass_get(UCLASS_PCI, &uc_dev);
	if (!ret && uc_dev && uclass_destroy(uc_dev))
		printf("couldn't remove PCI devices\n");

	ret = uclass_get(UCLASS_SPI_FLASH, &uc_dev);
	if (!ret && uc_dev && uclass_destroy(uc_dev))
		printf("couldn't remove SPI FLASH devices\n");

	/* regulator-m2-pwr is a 19x19 EVK rail; it is never enabled here */

	if (imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, false))
		printf("%s: HSIO MIX off request failed\n", __func__);
	if (imx9_scmi_power_domain_enable(IMX95_PD_NETC, false))
		printf("%s: NETC MIX off request failed\n", __func__);
}

#if IS_ENABLED(CONFIG_OF_BOARD_FIXUP)
/*
 * The EVK fixups (audio-board MDIO pin swap, 19x19 NETC-config node
 * disabling) probe EVK-only I2C devices and do not apply to the Comet.
 * Only the fuse-based node removal is kept.
 */
int board_fix_fdt(void *fdt)
{
	/* Remove nodes for blocks fused off on this part */
	board_fix_fdt_fuse(fdt);

	return 0;
}
#endif
