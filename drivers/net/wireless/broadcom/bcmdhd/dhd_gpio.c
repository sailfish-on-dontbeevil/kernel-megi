
#include <osl.h>
#include <dhd_linux.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#ifdef BCMDHD_PLATDEV
#include <linux/platform_device.h>
#endif

#ifdef CONFIG_DHD_USE_STATIC_BUF
#ifdef DHD_STATIC_IN_DRIVER
extern int dhd_static_buf_init(void);
extern void dhd_static_buf_exit(void);
#endif /* DHD_STATIC_IN_DRIVER */
#if defined(BCMDHD_MDRIVER) && !defined(DHD_STATIC_IN_DRIVER)
extern void *dhd_wlan_mem_prealloc(uint bus_type, int index,
	int section, unsigned long size);
#else
extern void *dhd_wlan_mem_prealloc(int section, unsigned long size);
#endif
#endif /* CONFIG_DHD_USE_STATIC_BUF */

/* This is sample code in dts file.
bcmdhd_wlan {
	compatible = "android,bcmdhd_wlan";
	gpio_wl_reg_on = <&gpio GPIOH_4 GPIO_ACTIVE_HIGH>;
	gpio_wl_host_wake = <&gpio GPIOZ_15 GPIO_ACTIVE_HIGH>;
};
*/
#define DHD_DT_COMPAT_ENTRY		"android,bcmdhd_wlan"
#define GPIO_WL_REG_ON_PROPNAME		"gpio_wl_reg_on"
#define GPIO_WL_HOST_WAKE_PROPNAME	"gpio_wl_host_wake"

static int
dhd_wlan_set_power(int on, wifi_adapter_info_t *adapter)
{
	int gpio_wl_reg_on = adapter->gpio_wl_reg_on;
	int err = 0;

	return 0;

	if (on) {
		printf("======== PULL WL_REG_ON(%d) HIGH! ========\n", gpio_wl_reg_on);
		if (gpio_wl_reg_on >= 0) {
			err = gpio_direction_output(gpio_wl_reg_on, 1);
			if (err) {
				printf("%s: WL_REG_ON didn't output high\n", __FUNCTION__);
				return -EIO;
			}
		}
		//rk_pcie_power_on_atu_fixup();

#ifdef BUS_POWER_RESTORE
		if (adapter->pci_dev) {
			mdelay(100);
			printf("======== pci_set_power_state PCI_D0! ========\n");
			pci_set_power_state(adapter->pci_dev, PCI_D0);
			if (adapter->pci_saved_state)
				pci_load_and_free_saved_state(adapter->pci_dev, &adapter->pci_saved_state);
			pci_restore_state(adapter->pci_dev);
			err = pci_enable_device(adapter->pci_dev);
			if (err < 0)
				printf("%s: PCI enable device failed", __FUNCTION__);
			pci_set_master(adapter->pci_dev);
		}
#endif /* BUS_POWER_RESTORE */
		/* Lets customer power to get stable */
		mdelay(100);
	} else {
#ifdef BUS_POWER_RESTORE
		if (adapter->pci_dev) {
			printf("======== pci_set_power_state PCI_D3hot! ========\n");
			pci_save_state(adapter->pci_dev);
			adapter->pci_saved_state = pci_store_saved_state(adapter->pci_dev);
			if (pci_is_enabled(adapter->pci_dev))
				pci_disable_device(adapter->pci_dev);
			pci_set_power_state(adapter->pci_dev, PCI_D3hot);
		}
#endif /* BUS_POWER_RESTORE */
		printf("======== PULL WL_REG_ON(%d) LOW! ========\n", gpio_wl_reg_on);
		if (gpio_wl_reg_on >= 0) {
			err = gpio_direction_output(gpio_wl_reg_on, 0);
			if (err) {
				printf("%s: WL_REG_ON didn't output low\n", __FUNCTION__);
				return -EIO;
			}
		}
	}

	return err;
}

static int
dhd_wlan_set_reset(int onoff)
{
	return 0;
}

static int
dhd_wlan_set_carddetect(int present)
{
	int err = 0;

#if !defined(BUS_POWER_RESTORE)
	if (present) {
		printf("======== Card detection to detect PCIE card! ========\n");
	} else {
		printf("======== Card detection to remove PCIE card! ========\n");
	}
#endif /* BUS_POWER_RESTORE */

	return err;
}

static int
dhd_wlan_get_mac_addr(unsigned char *buf, int ifidx)
{
	if (ifidx == 1) {
		struct ether_addr ea_example = {{0x00, 0x11, 0x22, 0x33, 0x44, 0xFF}};
		bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
	} else {
		struct ether_addr ea_example = {{0x02, 0x11, 0x22, 0x33, 0x44, 0x55}};
		bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
		//err = rockchip_wifi_mac_addr(buf);
	}

	return 0;
}

static struct cntry_locales_custom brcm_wlan_translate_custom_table[] = {
	/* Table should be filled out based on custom platform regulatory requirement */
#ifdef EXAMPLE_TABLE
	{"",   "XT", 49},  /* Universal if Country code is unknown or empty */
	{"US", "US", 0},
#endif /* EXMAPLE_TABLE */
};

#ifdef CUSTOM_FORCE_NODFS_FLAG
struct cntry_locales_custom brcm_wlan_translate_nodfs_table[] = {
#ifdef EXAMPLE_TABLE
	{"",   "XT", 50},  /* Universal if Country code is unknown or empty */
	{"US", "US", 0},
#endif /* EXMAPLE_TABLE */
};
#endif

static void *dhd_wlan_get_country_code(char *ccode
#ifdef CUSTOM_FORCE_NODFS_FLAG
	, u32 flags
#endif
)
{
	struct cntry_locales_custom *locales;
	int size;
	int i;

	if (!ccode)
		return NULL;

#ifdef CUSTOM_FORCE_NODFS_FLAG
	if (flags & WLAN_PLAT_NODFS_FLAG) {
		locales = brcm_wlan_translate_nodfs_table;
		size = ARRAY_SIZE(brcm_wlan_translate_nodfs_table);
	} else {
#endif
		locales = brcm_wlan_translate_custom_table;
		size = ARRAY_SIZE(brcm_wlan_translate_custom_table);
#ifdef CUSTOM_FORCE_NODFS_FLAG
	}
#endif

	for (i = 0; i < size; i++)
		if (strcmp(ccode, locales[i].iso_abbrev) == 0)
			return &locales[i];
	return NULL;
}

struct wifi_platform_data dhd_wlan_control = {
	.set_power	= dhd_wlan_set_power,
	.set_reset	= dhd_wlan_set_reset,
	.set_carddetect	= dhd_wlan_set_carddetect,
	.get_mac_addr	= dhd_wlan_get_mac_addr,
#ifdef CONFIG_DHD_USE_STATIC_BUF
	.mem_prealloc	= dhd_wlan_mem_prealloc,
#endif /* CONFIG_DHD_USE_STATIC_BUF */
	.get_country_code = dhd_wlan_get_country_code,
};

static int
dhd_wlan_init_gpio(wifi_adapter_info_t *adapter)
{
	char wlan_node[32];
	struct device_node *root_node = NULL;
	int err = 0;
	int gpio_wl_reg_on = -1;
#ifdef CUSTOMER_OOB
	int gpio_wl_host_wake = -1;
	int host_oob_irq = -1;
	uint host_oob_irq_flags = 0;
#ifdef CUSTOMER_HW_ROCKCHIP
#ifdef HW_OOB
	int irq_flags = -1;
#endif
#endif
#endif

	/* Please check your schematic and fill right GPIO number which connected to
	* WL_REG_ON and WL_HOST_WAKE.
	*/
	strcpy(wlan_node, DHD_DT_COMPAT_ENTRY);
	root_node = of_find_compatible_node(NULL, NULL, wlan_node);

	printf("======== Get GPIO from DTS(%s) ========\n", wlan_node);
	if (root_node) {
		gpio_wl_reg_on = of_get_named_gpio(root_node, GPIO_WL_REG_ON_PROPNAME, 0);
		gpio_wl_host_wake = of_get_named_gpio(root_node, GPIO_WL_HOST_WAKE_PROPNAME, 0);
	} else {
		gpio_wl_reg_on = -1;
		gpio_wl_host_wake = -1;
	}

	if (gpio_wl_reg_on >= 0) {
		err = gpio_request(gpio_wl_reg_on, "WL_REG_ON");
		if (err < 0) {
			printf("%s: gpio_request(%d) for WL_REG_ON failed %d\n",
				__FUNCTION__, gpio_wl_reg_on, err);
			gpio_wl_reg_on = -1;
		}
	}
	adapter->gpio_wl_reg_on = gpio_wl_reg_on;

#ifdef CUSTOMER_OOB
	if (gpio_wl_host_wake >= 0) {
		err = gpio_request(gpio_wl_host_wake, "bcmdhd");
		if (err < 0) {
			printf("%s: gpio_request(%d) for WL_HOST_WAKE failed %d\n",
				__FUNCTION__, gpio_wl_host_wake, err);
			return -1;
		}
		adapter->gpio_wl_host_wake = gpio_wl_host_wake;
		err = gpio_direction_input(gpio_wl_host_wake);
		if (err < 0) {
			printf("%s: gpio_direction_input(%d) for WL_HOST_WAKE failed %d\n",
				__FUNCTION__, gpio_wl_host_wake, err);
			gpio_free(gpio_wl_host_wake);
			return -1;
		}
		host_oob_irq = gpio_to_irq(gpio_wl_host_wake);
		if (host_oob_irq < 0) {
			printf("%s: gpio_to_irq(%d) for WL_HOST_WAKE failed %d\n",
				__FUNCTION__, gpio_wl_host_wake, host_oob_irq);
			gpio_free(gpio_wl_host_wake);
			return -1;
		}
	}

#ifdef HW_OOB
#ifdef HW_OOB_LOW_LEVEL
	host_oob_irq_flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL | IORESOURCE_IRQ_SHAREABLE;
#else
	host_oob_irq_flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE;
#endif
#else
	host_oob_irq_flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_SHAREABLE;
#endif
	host_oob_irq_flags &= IRQF_TRIGGER_MASK;

	adapter->irq_num = host_oob_irq;
	adapter->intr_flags = host_oob_irq_flags;
	printf("%s: WL_HOST_WAKE=%d, oob_irq=%d, oob_irq_flags=0x%x\n", __FUNCTION__,
		gpio_wl_host_wake, host_oob_irq, host_oob_irq_flags);
#endif /* CUSTOMER_OOB */
	printf("%s: WL_REG_ON=%d\n", __FUNCTION__, gpio_wl_reg_on);

	return 0;
}

static void
dhd_wlan_deinit_gpio(wifi_adapter_info_t *adapter)
{
	int gpio_wl_reg_on = adapter->gpio_wl_reg_on;
#ifdef CUSTOMER_OOB
	int gpio_wl_host_wake = adapter->gpio_wl_host_wake;
#endif

	if (gpio_wl_reg_on >= 0) {
		printf("%s: gpio_free(WL_REG_ON %d)\n", __FUNCTION__, gpio_wl_reg_on);
		gpio_free(gpio_wl_reg_on);
		gpio_wl_reg_on = -1;
	}
#ifdef CUSTOMER_OOB
	if (gpio_wl_host_wake >= 0) {
		printf("%s: gpio_free(WL_HOST_WAKE %d)\n", __FUNCTION__, gpio_wl_host_wake);
		gpio_free(gpio_wl_host_wake);
		gpio_wl_host_wake = -1;
	}
#endif /* CUSTOMER_OOB */
}

#if defined(BCMDHD_MDRIVER)
static void
dhd_wlan_init_adapter(wifi_adapter_info_t *adapter)
{
	adapter->bus_num = 1;
	adapter->slot_num = 1;
	adapter->index = 0;
	adapter->bus_type = PCI_BUS;

	printf("bus_type=%d, bus_num=%d, slot_num=%d\n",
		adapter->bus_type, adapter->bus_num, adapter->slot_num);
}
#endif /* BCMDHD_MDRIVER */

int
dhd_wlan_init_plat_data(wifi_adapter_info_t *adapter)
{
	int err = 0;

#ifdef BCMDHD_MDRIVER
	dhd_wlan_init_adapter(adapter);
#endif /* BCMDHD_MDRIVER */

	//err = dhd_wlan_init_gpio(adapter);
	//if (err)
		//goto exit;

#ifdef DHD_STATIC_IN_DRIVER
	err = dhd_static_buf_init();
#endif

exit:
	return err;
}

void
dhd_wlan_deinit_plat_data(wifi_adapter_info_t *adapter)
{
#ifdef DHD_STATIC_IN_DRIVER
	dhd_static_buf_exit();
#endif
	//dhd_wlan_deinit_gpio(adapter);
}
