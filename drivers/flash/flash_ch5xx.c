/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_flash_controller

#define FLASH_WRITE_BLK_SZ DT_PROP(DT_NODELABEL(flash0), write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(DT_NODELABEL(flash0), erase_block_size)
#define DATA_FLASH_OFFSET DT_INST_PROP(0, data_flash_offset)

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/flash.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_ch5xx, CONFIG_FLASH_LOG_LEVEL);

#define FLASH_SEM_TIMEOUT (k_is_in_isr() ? K_NO_WAIT : K_FOREVER)

struct flash_ch5xx_dev_data {
#ifdef CONFIG_MULTITHREADING
	struct k_sem sem;
#endif
};

static const struct flash_parameters flash_ch5xx_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

#ifdef CONFIG_MULTITHREADING
static inline void flash_ch5xx_sem_take(const struct device *dev)
{
	struct flash_ch5xx_dev_data *data = dev->data;

	k_sem_take(&data->sem, FLASH_SEM_TIMEOUT);
}

static inline void flash_ch5xx_sem_give(const struct device *dev)
{
	struct flash_ch5xx_dev_data *data = dev->data;

	k_sem_give(&data->sem);
}
#else

#define flash_ch5xx_sem_take(dev) do {} while (0)
#define flash_ch5xx_sem_give(dev) do {} while (0)

#endif /* CONFIG_MULTITHREADING */

static int flash_ch5xx_read(const struct device *dev, off_t address, 
								void *buffer, size_t length)
{

	flash_ch5xx_sem_take(dev);
    int ret = 0;
    if (address < DATA_FLASH_OFFSET) {
        FLASH_ROM_READ(address, buffer, length);
    } else {
        ret = EEPROM_READ(address - DATA_FLASH_OFFSET, buffer, length);
    }

	flash_ch5xx_sem_give(dev);
	return ret;
}

static int flash_ch5xx_write(const struct device *dev, off_t address,
			     const void *buffer, size_t length)
{
	int ret = 0;

	flash_ch5xx_sem_take(dev);

    if (address < DATA_FLASH_OFFSET) {
        ret = FLASH_ROM_WRITE(address, (void *)buffer, length);
    } else {
        ret = EEPROM_WRITE(address - DATA_FLASH_OFFSET, (void *)buffer, length);
    }
	flash_ch5xx_sem_give(dev);
	return ret;
}

static int flash_ch5xx_erase(const struct device *dev, off_t start, size_t len)
{   
	flash_ch5xx_sem_take(dev);

    int ret;
    if (start < DATA_FLASH_OFFSET) {
        ret = FLASH_ROM_ERASE(start, len);
    } else {
        ret = EEPROM_ERASE(start - DATA_FLASH_OFFSET, len);
    }

	flash_ch5xx_sem_give(dev);
	return ret;
}

#if CONFIG_FLASH_PAGE_LAYOUT

#define CH5XX_FLASH_PAGES_LAYOUT(node_id, prop, idx)						\
	{											\
		.pages_count = DT_PHA_BY_IDX(node_id, prop, idx, pages_count),			\
		.pages_size = DT_PHA_BY_IDX(node_id, prop, idx, pages_size),			\
	}

#define CH5XX_FLASH_PAGES_LAYOUTS									\
	DT_FOREACH_PROP_ELEM_SEP(DT_INST(0, wch_ch5xx_flash), \
        erase_blocks, CH5XX_FLASH_PAGES_LAYOUT, (,))


static const struct flash_pages_layout ch5xx_pages_layout[] = {
    CH5XX_FLASH_PAGES_LAYOUTS
};

void flash_rom_ch5xx_page_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	*layout = ch5xx_pages_layout;
	*layout_size = ARRAY_SIZE(ch5xx_pages_layout);
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
flash_ch5xx_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_ch5xx_parameters;
}

static int flash_ch5xx_init(const struct device *dev)
{
	struct flash_ch5xx_dev_data *const dev_data = dev->data;

#ifdef CONFIG_MULTITHREADING
	k_sem_init(&dev_data->sem, 1, 1);
#endif /* CONFIG_MULTITHREADING */

	return 0;
}

static const struct flash_driver_api flash_ch5xx_driver_api = {
	.read = flash_ch5xx_read,
	.write = flash_ch5xx_write,
	.erase = flash_ch5xx_erase,
	.get_parameters = flash_ch5xx_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_rom_ch5xx_page_layout,
#endif
};

static struct flash_ch5xx_dev_data flash_ch5xx_data;

DEVICE_DT_INST_DEFINE(0, flash_ch5xx_init,
		      NULL,
		      &flash_ch5xx_data, NULL,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_ch5xx_driver_api);
