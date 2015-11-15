/*
 * Copyright (c) 2012 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/ihex.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/time.h>
#include "rmi_driver.h"
#include "rmi_f01.h"
#include "rmi_f34.h"

#define HAS_BSR_MASK 0x20

#define CHECKSUM_OFFSET 0
#define BOOTLOADER_VERSION_OFFSET 0x07
#define IMAGE_SIZE_OFFSET 0x08
#define CONFIG_SIZE_OFFSET 0x0C

#define ENABLE_WAIT_US (300 * 1000)

#define MAX_FIRMWARE_ID_LEN 10

#define SENSOR_ID_PIN  0 /*A0*/

extern int read_f01_properties(struct rmi_function *fn, struct rmi_device *rmi_dev,
				struct f01_basic_properties *props, u16 query_addr,
				struct device *dev);

/** Image file V5, Option 0
 */
struct rmi_image_header {
	u32 checksum;
	unsigned int image_size;
	unsigned int config_size;
	u8 options;
	u8 bootloader_version;
	u8 product_id[RMI_PRODUCT_ID_LENGTH + 1];
	u8 product_info[RMI_PRODUCT_INFO_LENGTH];
};

static u32 extract_u32(const u8 *ptr)
{
	return (u32)ptr[0] +
		(u32)ptr[1] * 0x100 +
		(u32)ptr[2] * 0x10000 +
		(u32)ptr[3] * 0x1000000;
}

static u32 extract_u32_be(const u8 *ptr)
{
      return (u32)ptr[3] +
               (u32)ptr[2] * 0x100 +
               (u32)ptr[1] * 0x10000 +
               (u32)ptr[0] * 0x1000000;
}
#define RMI_NAME_BUFFER_SIZE 64

struct rmi_reflash_data {
	struct rmi_device *rmi_dev;
	struct pdt_entry *f01_pdt;
	struct f01_basic_properties f01_properties;
	struct f01_device_control f01_controls;
	u8 device_status;
	char product_id[RMI_PRODUCT_ID_LENGTH+1];
	struct pdt_entry *f34_pdt;
	u8 bootloader_id[2];
	union f34_query_regs f34_queries;
	union f34_control_status f34_controls;
	const u8 *firmware_data;
	const u8 *config_data;
	const u8 *lockdown_data;
	unsigned int lockdown_size;
	char *firmware_name;
	bool sensor_id;
	bool force;
	ulong busy;
	char *img_name;
	char name_buf[RMI_NAME_BUFFER_SIZE];
	struct work_struct reflash_work;
};

enum flash_area {
       NONE,
       UI_FIRMWARE,
       CONFIG_AREA
};

/* If this parameter is true, we will update the firmware regardless of
 * the versioning info.
 */

#define RMI4_IMAGE_FILE_REV1_OFFSET 30
#define RMI4_IMAGE_FILE_REV2_OFFSET 31
#define IMAGE_FILE_CHECKSUM_SIZE 4
#define FIRMWARE_IMAGE_AREA_OFFSET 0x100

#define RMI4_IMAGE_LOCKDOWN_DATA_V2_SIZE    0x30
#define RMI4_IMAGE_LOCKDOWN_DATA_V3_SIZE    0x40
#define RMI4_IMAGE_LOCKDOWN_DATA_V5_SIZE    0x50

#define RMI4_IMAGE_LOCKDOWN_DATA_V2_OFFSET  0xD0
#define RMI4_IMAGE_LOCKDOWN_DATA_V3_OFFSET  0xC0
#define RMI4_IMAGE_LOCKDOWN_DATA_V5_OFFSET  0xB0

static void rmi_extract_header(const u8 *data, int pos, struct rmi_image_header *header)
{
	header->checksum = extract_u32(&data[pos + CHECKSUM_OFFSET]);
	header->bootloader_version = data[pos + BOOTLOADER_VERSION_OFFSET];
	header->image_size = extract_u32(&data[pos + IMAGE_SIZE_OFFSET]);
	header->config_size = extract_u32(&data[pos + CONFIG_SIZE_OFFSET]);
	memcpy(header->product_id, &data[pos + PRODUCT_ID_OFFSET],
	       RMI_PRODUCT_ID_LENGTH);
	header->product_id[RMI_PRODUCT_ID_LENGTH] = 0;
	memcpy(header->product_info, &data[pos + PRODUCT_INFO_OFFSET],
	       RMI_PRODUCT_INFO_LENGTH);
}

static int rescan_pdt(struct rmi_reflash_data *data)
{
	int retval;
	bool f01_found;
	bool f34_found;
	struct pdt_entry pdt_entry;
	int i;
	struct rmi_device *rmi_dev = data->rmi_dev;
	struct pdt_entry *f34_pdt = data->f34_pdt;
	struct pdt_entry *f01_pdt = data->f01_pdt;

	/* Per spec, once we're in reflash we only need to look at the first
	 * PDT page for potentially changed F01 and F34 information.
	 */
	for (i = PDT_START_SCAN_LOCATION; i >= PDT_END_SCAN_LOCATION;
			i -= sizeof(pdt_entry)) {
		retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					sizeof(pdt_entry));
		if (retval != sizeof(pdt_entry)) {
			dev_err(&rmi_dev->dev,
				"Read PDT entry at %#06x failed: %d.\n",
				i, retval);
			return retval;
		}

		if (RMI4_END_OF_PDT(pdt_entry.function_number))
			break;

		if (pdt_entry.function_number == 0x01) {
			memcpy(f01_pdt, &pdt_entry, sizeof(pdt_entry));
			f01_found = true;
		} else if (pdt_entry.function_number == 0x34) {
			memcpy(f34_pdt, &pdt_entry, sizeof(pdt_entry));
			f34_found = true;
		}
	}

	if (!f01_found) {
		dev_err(&rmi_dev->dev, "Failed to find F01 PDT entry.\n");
		retval = -ENODEV;
	} else if (!f34_found) {
		dev_err(&rmi_dev->dev, "Failed to find F34 PDT entry.\n");
		retval = -ENODEV;
	} else
		retval = 0;

	return retval;
}

static int read_f34_controls(struct rmi_reflash_data *data)
{
	int retval;

	retval = rmi_read(data->rmi_dev, data->f34_controls.address,
			  data->f34_controls.regs);
	if (retval < 0)
		return retval;

	return 0;
}

static int read_f01_status(struct rmi_reflash_data *data)
{
	int retval;

	retval = rmi_read(data->rmi_dev, data->f01_pdt->data_base_addr,
			  &data->device_status);
	if (retval < 0)
		return retval;

	return 0;
}

static int read_f01_controls(struct rmi_reflash_data *data)
{
	int retval;

	retval = rmi_read(data->rmi_dev, data->f01_pdt->control_base_addr,
			  &data->f01_controls.ctrl0);
	if (retval < 0)
		return retval;
	return 0;
}

static int write_f01_controls(struct rmi_reflash_data *data)
{
	int retval;

	retval = rmi_write_block(data->rmi_dev,
			data->f01_pdt->control_base_addr,
			&data->f01_controls.ctrl0, sizeof(data->f01_controls.ctrl0));
	if (retval < 0)
		return retval;
	return 0;
}

#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100

/* Wait until the status is idle and we're ready to continue */
static int wait_for_idle(struct rmi_reflash_data *data, int timeout_ms)
{
	int timeout_count = ((timeout_ms * 1000) / MAX_SLEEP_TIME_US) + 1;
	int count = 0;
	union f34_control_status *controls = &data->f34_controls;
	int retval;

	do {
		if (count || timeout_count == 1)
			usleep_range(MIN_SLEEP_TIME_US, MAX_SLEEP_TIME_US);
		retval = read_f34_controls(data);
		count++;
		if (retval == -EREMOTEIO) {
			dev_err(&data->rmi_dev->dev, "Aborting %s do to Remote I/O error.\n",
				__func__);
			return retval;
		}
		if (retval < 0)
			continue;
		else if (IS_IDLE(controls)) {
			if (!data->f34_controls.program_enabled) {
				/** This works around a bug in certain device
				 * firmwares, where the idle state is reached,
				 * but the program_enabled bit is not yet set.
				 */
				dev_warn(&data->rmi_dev->dev, "Yikes!  We're not enabled!\n");
				msleep(1000);
				read_f34_controls(data);
			}
			return 0;
		}
	} while (count < timeout_count);

	dev_err(&data->rmi_dev->dev,
		"ERROR: Timeout waiting for idle status, last status: %#04x.\n",
		controls->regs[0]);
	dev_err(&data->rmi_dev->dev, "Command: %#04x\n", controls->command);
	dev_err(&data->rmi_dev->dev, "Status:  %#04x\n", controls->status);
	dev_err(&data->rmi_dev->dev, "Enabled: %d\n",
			controls->program_enabled);
	dev_err(&data->rmi_dev->dev, "Idle:    %d\n", IS_IDLE(controls));
	return -ETIMEDOUT;
}

static int read_f34_queries(struct rmi_reflash_data *data)
{
	int retval;
	u8 id_str[3];

	retval = rmi_read_block(data->rmi_dev, data->f34_pdt->query_base_addr,
				data->bootloader_id, 2);
	if (retval < 0) {
		dev_err(&data->rmi_dev->dev,
			"Failed to read F34 bootloader_id (code %d).\n",
			retval);
		return retval;
	}
	retval = rmi_read_block(data->rmi_dev, data->f34_pdt->query_base_addr+2,
			data->f34_queries.regs,
			ARRAY_SIZE(data->f34_queries.regs));
	if (retval < 0) {
		dev_err(&data->rmi_dev->dev,
			"Failed to read F34 queries (code %d).\n", retval);
		return retval;
	}
	data->f34_queries.block_size =
			le16_to_cpu(data->f34_queries.block_size);
	data->f34_queries.fw_block_count =
			le16_to_cpu(data->f34_queries.fw_block_count);
	data->f34_queries.config_block_count =
			le16_to_cpu(data->f34_queries.config_block_count);
	id_str[0] = data->bootloader_id[0];
	id_str[1] = data->bootloader_id[1];
	id_str[2] = 0;

	dev_dbg(&data->rmi_dev->dev, "F34 bootloader id: %s (%#04x %#04x)\n",
		id_str, data->bootloader_id[0], data->bootloader_id[1]);
	dev_dbg(&data->rmi_dev->dev, "F34 has config id: %d\n",
		data->f34_queries.has_config_id);
	dev_dbg(&data->rmi_dev->dev, "F34 unlocked:      %d\n",
		data->f34_queries.unlocked);
	dev_dbg(&data->rmi_dev->dev, "F34 regMap:        %d\n",
		data->f34_queries.reg_map);
	dev_dbg(&data->rmi_dev->dev, "F34 block size:    %d\n",
		data->f34_queries.block_size);
	dev_dbg(&data->rmi_dev->dev, "F34 fw blocks:     %d\n",
		data->f34_queries.fw_block_count);
	dev_dbg(&data->rmi_dev->dev, "F34 config blocks: %d\n",
		data->f34_queries.config_block_count);

	data->f34_controls.address = data->f34_pdt->data_base_addr +
			F34_BLOCK_DATA_OFFSET + data->f34_queries.block_size;

	return 0;
}

static int write_bootloader_id(struct rmi_reflash_data *data)
{
	int retval;
	struct rmi_device *rmi_dev = data->rmi_dev;
	struct pdt_entry *f34_pdt = data->f34_pdt;

	retval = rmi_write_block(rmi_dev,
			f34_pdt->data_base_addr + F34_BLOCK_DATA_OFFSET,
			data->bootloader_id, ARRAY_SIZE(data->bootloader_id));
	if (retval < 0) {
		dev_err(&rmi_dev->dev,
			"Failed to write bootloader ID. Code: %d.\n", retval);
		return retval;
	}

	return 0;
}

static int write_f34_command(struct rmi_reflash_data *data, u8 command)
{
	int retval;
	struct rmi_device *rmi_dev = data->rmi_dev;

	retval = rmi_write(rmi_dev, data->f34_controls.address, command);
	if (retval < 0) {
		dev_err(&rmi_dev->dev,
			"Failed to write F34 command %#04x. Code: %d.\n",
			command, retval);
		return retval;
	}

	return 0;
}

static int enter_flash_programming(struct rmi_reflash_data *data)
{
	int retval;
	struct rmi_device *rmi_dev = data->rmi_dev;

	retval = write_bootloader_id(data);
	if (retval < 0)
		return retval;

	dev_dbg(&rmi_dev->dev, "Enabling flash programming.\n");
	retval = write_f34_command(data, F34_ENABLE_FLASH_PROG);
	if (retval < 0)
		return retval;

	if (data->rmi_dev->xport->info.proto_type == RMI_PROTOCOL_HID)
                msleep(100);

	retval = wait_for_idle(data, F34_ENABLE_WAIT_MS);
	if (retval) {
		dev_err(&rmi_dev->dev, "Did not reach idle state after %d ms. Code: %d.\n",
			F34_ENABLE_WAIT_MS, retval);
		return retval;
	}
	if (!data->f34_controls.program_enabled) {
		dev_err(&rmi_dev->dev, "Reached idle, but programming not enabled (current status register: %#04x).\n",
					data->f34_controls.regs[0]);
		return -EINVAL;
	}
	dev_dbg(&rmi_dev->dev, "HOORAY! Programming is enabled!\n");

	retval = rescan_pdt(data);
	if (retval) {
		dev_err(&rmi_dev->dev, "Failed to rescan pdt.  Code: %d.\n",
			retval);
		return retval;
	}

	retval = read_f01_status(data);
	if (retval) {
		dev_err(&rmi_dev->dev, "Failed to read F01 status after enabling reflash. Code: %d.\n",
			retval);
		return retval;
	}
	if (!RMI_F01_STATUS_BOOTLOADER(data->device_status)) {
		dev_err(&rmi_dev->dev, "Device reports as not in flash programming mode.\n");
		return -EINVAL;
	}

	retval = read_f34_queries(data);
	if (retval) {
		dev_err(&rmi_dev->dev, "F34 queries failed, code = %d.\n",
			retval);
		return retval;
	}

	retval = read_f01_controls(data);
	if (retval) {
		dev_err(&rmi_dev->dev, "F01 controls read failed, code = %d.\n",
			retval);
		return retval;
	}
	data->f01_controls.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;
	data->f01_controls.ctrl0 |= RMI_SLEEP_MODE_NORMAL;

	retval = write_f01_controls(data);
	if (retval) {
		dev_err(&rmi_dev->dev, "F01 controls write failed, code = %d.\n",
			retval);
		return retval;
	}

	return retval;
}

static void rmi_reset_device(struct rmi_reflash_data *data)
{
	int retval;
	struct rmi_device_platform_data *pdata =
		to_rmi_platform_data(data->rmi_dev);

	dev_dbg(&data->rmi_dev->dev, "Resetting...\n");
	retval = rmi_write(data->rmi_dev, data->f01_pdt->command_base_addr,
			   F01_RESET_MASK);
	if (retval < 0)
		dev_warn(&data->rmi_dev->dev,
			 "WARNING - post-flash reset failed, code: %d.\n",
			 retval);
	msleep(pdata->reset_delay_ms);
	if (data->rmi_dev->xport->post_reset)
		data->rmi_dev->xport->post_reset(data->rmi_dev->xport);
	dev_dbg(&data->rmi_dev->dev, "Reset completed.\n");

	rescan_pdt(data);
}

/*
 * Send data to the device one block at a time.
 */
static int write_blocks(struct rmi_reflash_data *data, u8 *block_ptr,
			u16 block_count, u8 cmd)
{
	int block_num;
	u8 zeros[] = {0, 0};
	int retval;
	u16 addr = data->f34_pdt->data_base_addr + F34_BLOCK_DATA_OFFSET;

	retval = rmi_write_block(data->rmi_dev, data->f34_pdt->data_base_addr,
				 zeros, ARRAY_SIZE(zeros));
	if (retval < 0) {
		dev_err(&data->rmi_dev->dev, "Failed to write initial zeros. Code=%d.\n",
			retval);
		return retval;
	}

	for (block_num = 0; block_num < block_count; ++block_num) {
		retval = rmi_write_block(data->rmi_dev, addr, block_ptr,
					 data->f34_queries.block_size);
		if (retval < 0) {
			dev_err(&data->rmi_dev->dev, "Failed to write block %d. Code=%d.\n",
				block_num, retval);
			return retval;
		}

		retval = write_f34_command(data, cmd);
		if (retval) {
			dev_err(&data->rmi_dev->dev, "Failed to write command for block %d. Code=%d.\n",
				block_num, retval);
			return retval;
		}


		retval = wait_for_idle(data, F34_IDLE_WAIT_MS);
		if (retval) {
			dev_err(&data->rmi_dev->dev, "Failed to go idle after writing block %d. Code=%d.\n",
				block_num, retval);
			return retval;
		}

		block_ptr += data->f34_queries.block_size;
	}

	return 0;
}

static int write_firmware(struct rmi_reflash_data *data)
{
	return write_blocks(data, (u8 *) data->firmware_data,
		data->f34_queries.fw_block_count, F34_WRITE_FW_BLOCK);
}

static int write_configuration(struct rmi_reflash_data *data)
{
	return write_blocks(data, (u8 *) data->config_data,
		data->f34_queries.config_block_count, F34_WRITE_CONFIG_BLOCK);
}

static int write_lockdown(struct rmi_reflash_data *data)
{
       int num_blocks;

       if (data->f34_queries.unlocked && data->lockdown_data) {
               num_blocks = data->lockdown_size / 0x10;

               return write_blocks(data, (u8 *) data->lockdown_data, num_blocks,
                                       F34_WRITE_LOCKDOWN_BLOCK);
       }

       return 0;
}

static void rmi_reflash_firmware(struct rmi_reflash_data *data)
{
	struct timespec start;
	struct timespec end;
	s64 duration_ns;
	int retval = 0;

	retval = enter_flash_programming(data);
	if (retval)
		return;

	retval = write_lockdown(data);
        if (retval)
                return;

	retval = enter_flash_programming(data);
        if (retval)
                return;

	retval = write_bootloader_id(data);
	if (retval)
		return;

	dev_dbg(&data->rmi_dev->dev, "Erasing FW...\n");
	getnstimeofday(&start);
	retval = write_f34_command(data, F34_ERASE_ALL);
	if (retval)
		return;

	if (data->rmi_dev->xport->info.proto_type == RMI_PROTOCOL_HID)
                msleep(F34_ERASE_WAIT_MS);

	retval = wait_for_idle(data, F34_ERASE_WAIT_MS);
	if (retval) {
		dev_err(&data->rmi_dev->dev,
			"Failed to reach idle state. Code: %d.\n", retval);
		return;
	}
	getnstimeofday(&end);
	duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
	dev_dbg(&data->rmi_dev->dev,
		 "Erase complete, time: %lld ns.\n", duration_ns);

	if (data->firmware_data) {
		dev_dbg(&data->rmi_dev->dev, "Writing firmware...\n");
		getnstimeofday(&start);
		retval = write_firmware(data);
		if (retval)
			return;
		getnstimeofday(&end);
		duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
		dev_dbg(&data->rmi_dev->dev,
			 "Done writing FW, time: %lld ns.\n", duration_ns);
	}

	if (data->config_data) {
		dev_dbg(&data->rmi_dev->dev, "Writing configuration...\n");
		getnstimeofday(&start);
		retval = write_configuration(data);
		if (retval)
			return;
		getnstimeofday(&end);
		duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
		dev_dbg(&data->rmi_dev->dev,
			 "Done writing config, time: %lld ns.\n", duration_ns);
	}
}

static void reflash_config(struct rmi_reflash_data *data)
{
        struct timespec start;
        struct timespec end;
        s64 duration_ns;
        int retval = 0;

        retval = enter_flash_programming(data);
        if (retval)
                return;

        retval = write_bootloader_id(data);
        if (retval)
                return;

        dev_dbg(&data->rmi_dev->dev, "Erasing config block...\n");
        getnstimeofday(&start);
        retval = write_f34_command(data, F34_ERASE_CONFIG);
        if (retval)
                return;

        retval = wait_for_idle(data, F34_ERASE_WAIT_MS);
        if (retval) {
                dev_err(&data->rmi_dev->dev,
                        "Failed to reach idle state. Code: %d.\n", retval);
                return;
        }
        getnstimeofday(&end);
        duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
        dev_dbg(&data->rmi_dev->dev,
                 "Erase complete, time: %lld ns.\n", duration_ns);

        if (data->config_data) {
                dev_dbg(&data->rmi_dev->dev, "Writing configuration...\n");
                getnstimeofday(&start);
                retval = write_configuration(data);
                if (retval)
                        return;
                getnstimeofday(&end);
                duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
                dev_dbg(&data->rmi_dev->dev,
                         "Done writing config, time: %lld ns.\n", duration_ns);
        }
}

static int read_sensor_id(struct rmi_reflash_data *data, u16 *sensor_id)
{
	int retval;
	struct rmi_device *rmi_dev = data->rmi_dev;
	u8 sensor_id_mask[2];
	u8 sensor_id_pullup_mask[2];
	u16 pin_mask;
	u8 sensor_id_val[2];

	pin_mask = 0x0001 << SENSOR_ID_PIN;

	sensor_id_mask[0] = sensor_id_pullup_mask[0]
			= pin_mask & 0x00FF;
	sensor_id_mask[1] = sensor_id_pullup_mask[1]
			= (pin_mask & 0xFF00) >> 8;
	/* go into bootloader mode */
	enter_flash_programming(data);

	/* set sensor mask */
	retval = rmi_write_block(data->rmi_dev, data->f34_pdt->data_base_addr + 2,
			sensor_id_mask, ARRAY_SIZE(sensor_id_mask));
	if (retval < 0) {
		dev_err(&data->rmi_dev->dev, "Failed to write sensor id mask. Code=%d.\n",
			retval);
		return retval;
	}

	/* set sensor pullup mask */
	retval = rmi_write_block(data->rmi_dev, data->f34_pdt->data_base_addr + 4,
			sensor_id_pullup_mask, ARRAY_SIZE(sensor_id_pullup_mask));
	if (retval < 0) {
		dev_err(&data->rmi_dev->dev, "Failed to write sensor id pullup mask. Code=%d.\n",
			retval);
		return retval;
	}

	/* send command to read sensor id */
	retval = write_f34_command(data, F34_READ_SENSOR_ID);
	if (retval < 0)
		return retval;

	retval = wait_for_idle(data, F34_ENABLE_WAIT_MS);
	if (retval) {
		dev_err(&rmi_dev->dev, "Did not reach idle state after %d ms. Code: %d.\n",
			F34_IDLE_WAIT_MS, retval);
		return retval;
	}

	/* read sensor id from data register*/
	retval = rmi_read_block(data->rmi_dev, data->f34_pdt->data_base_addr + 6,
		sensor_id_val, 2);
	if (retval < 0) {
		dev_err(&data->rmi_dev->dev, "Failed to read sensor id. Code=%d.\n",
			retval);
		return retval;
	}

	*sensor_id = (u16)sensor_id_val[1] <<8 | sensor_id_val[0];
	/* go back to UI mode */
	rmi_reset_device(data);

	retval = read_f01_status(data);
	if (retval) {
		dev_err(&data->rmi_dev->dev,
			"Failed to read F01 status. Code: %d.\n", retval);
		return retval;
	}

	if (RMI_F01_STATUS_BOOTLOADER(data->device_status))
		dev_warn(&rmi_dev->dev, "Device reports to be in flash programming mode.\n");

	return retval;
}

/* Returns false if the firmware should not be reflashed.
 */
static bool rmi_go_nogo(struct rmi_reflash_data *data, struct rmi_image_header *header)
{
	int retval;
        int index = 0;
        char *strptr;
        int deviceFirmwareID;
        int imageConfigID;
        int deviceConfigID;
        unsigned long imageFirmwareID;
        unsigned char config_id[4];
        u8 fwid[3];
        enum flash_area flash_area = NONE;

        char *imagePR = kzalloc(sizeof(MAX_FIRMWARE_ID_LEN), GFP_KERNEL);

	if (data->force) {
		dev_dbg(&data->rmi_dev->dev, "Reflash force flag in effect.\n");
		flash_area = UI_FIRMWARE;
		goto exit;
	}

	if ((data->f01_properties.productinfo >> 8) < header->product_info[0] ||
		(data->f01_properties.productinfo & 0x0F) < header->product_info[1]) {
		dev_info(&data->rmi_dev->dev,
			 "FW product ID is older than image product ID.\n");
		return true;
	}

	retval = read_f01_status(data);
	if (retval) {
		dev_err(&data->rmi_dev->dev,
			"Failed to read F01 status. Code: %d.\n", retval);
		goto exit;
	}


	dev_dbg(&data->rmi_dev->dev, "Flash prog bit at go/nogo: %d\n",
			RMI_F01_STATUS_BOOTLOADER(data->device_status));

	        /* device is in bootloader mode, do firmware update */
        if (RMI_F01_STATUS_BOOTLOADER(data->device_status)) {
                flash_area = UI_FIRMWARE;
                goto exit;
        }


        /* check device firmware number */
        retval = rmi_read_block(data->rmi_dev,
                        data->f01_pdt->query_base_addr + 18,
                        fwid,
                        sizeof(fwid));
        if (retval < 0) {
                dev_err(&data->rmi_dev->dev, "Failed to read build id.\n");
                goto exit;
        }

        deviceFirmwareID = ((int)fwid[2] << 16) | ((int)fwid[1] << 8) | (int)fwid[0];

        /* check img firmware number */
        strptr = strstr(data->firmware_name, "PR");
        if (!strptr) {
                dev_err(&data->rmi_dev->dev,
                        "No valid PR number (PRxxxxxxx.img)" \
                        "found in image file name...\n");
                goto exit;
        }

        strptr += 2;
        while (strptr[index] >= '0' && strptr[index] <= '9') {
                imagePR[index] = strptr[index];
                index++;
        }
        imagePR[index] = 0;

        retval = kstrtoul(imagePR, 10, &imageFirmwareID);
        if (retval == -EINVAL) {
                dev_err(&data->rmi_dev->dev,
                        "invalid image firmware id...\n");
                goto exit;
        }

        dev_info(&data->rmi_dev->dev,
                        "Device firmware id %d, .img firmware id %d\n",
                        deviceFirmwareID,
                        (unsigned int)imageFirmwareID);
        if (imageFirmwareID > deviceFirmwareID) {
                flash_area = UI_FIRMWARE;
                goto exit;
        } else if (imageFirmwareID != deviceFirmwareID) {
                flash_area = NONE;
                goto exit;
        }
        /* check device config id */
        retval = rmi_read_block(data->rmi_dev,
                                data->f34_pdt->control_base_addr,
                                config_id,
                                sizeof(config_id));
        if (retval < 0) {
                dev_err(&data->rmi_dev->dev,
                        "Failed to read config ID (code %d).\n", retval);
                flash_area = NONE;
                goto exit;
        }
        deviceConfigID =  extract_u32_be(config_id);

        dev_info(&data->rmi_dev->dev,
                "Device config ID 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
                config_id[0], config_id[1], config_id[2], config_id[3]);


        /* check img config id */
        dev_info(&data->rmi_dev->dev,
                        ".img config ID 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
                        data->config_data[0],
                        data->config_data[1],
                        data->config_data[2],
                        data->config_data[3]);



	if(data->sensor_id != (config_id[0] & 0x01)) {
		dev_warn(&data->rmi_dev->dev,
			"device config id %d does not match device sensor id %d (pin %d)\n",
			config_id[0],
			data->sensor_id,
			SENSOR_ID_PIN);
                flash_area = UI_FIRMWARE;
                goto exit;
        }

        if (data->sensor_id != (data->config_data[0] & 0x01)) {
                dev_warn(&data->rmi_dev->dev,
			".img sensor id %d does not match device sensor id %d (pin %d)\n",
			data->config_data[0],
			data->sensor_id,
			SENSOR_ID_PIN);
	}

	imageConfigID =  extract_u32_be(data->config_data);

        dev_info(&data->rmi_dev->dev, "device config id %d, .img config id %d\n",
                                deviceConfigID, imageConfigID);

        if (imageConfigID > deviceConfigID) {
                flash_area = CONFIG_AREA;
                goto exit;
        }


exit:
        kfree(imagePR);

        if (flash_area == NONE)
                dev_info(&data->rmi_dev->dev,
                        "Nothing needs to be updated\n");
        else
                dev_info(&data->rmi_dev->dev,
                        "Update %s block\n",
                        flash_area == UI_FIRMWARE ? "UI FW" : "CONFIG");

        return flash_area;
}


void rmi4_fw_update(struct rmi_device *rmi_dev,
		struct pdt_entry *f01_pdt, struct pdt_entry *f34_pdt)
{
	struct timespec start;
	struct timespec end;
	s64 duration_ns;
	const struct firmware *fw_entry = NULL;
	int retval;
	struct rmi_image_header *header = NULL;
	struct pdt_properties pdt_props;
	struct rmi_reflash_data *data = NULL;
	unsigned short sensor_id_array;
	enum flash_area flash_area;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_device_sensor_vendor * sensor_vendor;
	int i;

	getnstimeofday(&start);
	if(drv_data->reflash_data)
		data = drv_data->reflash_data;
	else {
		//data = kzalloc(sizeof(struct rmi_reflash_data), GFP_KERNEL);
		//if (!data) {
		//	dev_err(&rmi_dev->dev, "Failed to allocate data.\n");
		//	goto done;
		//}
		data = devm_kzalloc(&rmi_dev->dev, sizeof(struct rmi_reflash_data),
			    GFP_KERNEL);
		drv_data->reflash_data = data;
	}
	data->f01_pdt = f01_pdt;
	data->f34_pdt = f34_pdt;
	data->rmi_dev = rmi_dev;

	data->firmware_name = kcalloc(NAME_BUFFER_SIZE, sizeof(char), GFP_KERNEL);
	if (!data->firmware_name) {
		dev_err(&rmi_dev->dev, "Failed to allocate firmware_name.\n");
		goto done;
	}
	header = kzalloc(sizeof(struct rmi_image_header), GFP_KERNEL);
	if (!header) {
		dev_err(&rmi_dev->dev, "Failed to allocate header.\n");
		goto done;
	}

	retval = rmi_read(rmi_dev, PDT_PROPERTIES_LOCATION, &pdt_props);
	if (retval < 0) {
		dev_warn(&rmi_dev->dev,
			 "Failed to read PDT props at %#06x (code %d). Assuming 0x00.\n",
			 PDT_PROPERTIES_LOCATION, retval);
	}
	if (pdt_props.has_bsr) {
		dev_warn(&rmi_dev->dev,
			 "Firmware update for LTS not currently supported.\n");
		goto done;
	}

	retval = read_f01_properties(NULL, rmi_dev, &data->f01_properties,
					data->f01_pdt->query_base_addr, &rmi_dev->dev);
	if (retval) {
		dev_err(&rmi_dev->dev, "F01 queries failed, code = %d.\n",
			retval);
		goto done;
	}

	retval = read_f34_queries(data);
	if (retval) {
		dev_err(&rmi_dev->dev, "F34 queries failed, code = %d.\n",
			retval);
		goto done;
	}

	if (data->img_name && strlen(data->img_name))
		snprintf(data->firmware_name, NAME_BUFFER_SIZE, "rmi4/%s.img",
			data->img_name);
	else if (pdata->firmware_name && strlen(pdata->firmware_name))
		snprintf(data->firmware_name, NAME_BUFFER_SIZE, "rmi4/%s.img",
			pdata->firmware_name);
	else {
		if (!strlen(data->product_id)) {
			dev_err(&rmi_dev->dev,
				"Product ID is missing or empty - will not reflash.\n");
			goto done;
		}
		snprintf(data->firmware_name, NAME_BUFFER_SIZE, "rmi4/%s.img",
			data->product_id);
	}

	/* Check sensor id */
	if (pdata->multi_sensor_vendor_count) {
		retval = read_sensor_id(data, &sensor_id_array);
		if (retval != 0)
			dev_err(&rmi_dev->dev, "Read sensor id failed, code = %d\n",
				retval);
		else {
			data->sensor_id = sensor_id_array & (0x0001 << SENSOR_ID_PIN);
			for (i = 0; i < pdata->multi_sensor_vendor_count; ++i) {
				sensor_vendor = &pdata->sensor_vendor_id[i];
				if (data->sensor_id == sensor_vendor->vendor_id) {
					char * suffix = strstr(data->firmware_name, "img");
					if (suffix != NULL)
						snprintf(suffix,
							NAME_BUFFER_SIZE - (suffix
							- data->firmware_name),
							"%s.img", sensor_vendor->vendor_name);
				}
			}

		}

	}

	dev_info(&rmi_dev->dev, "Requesting %s.\n", data->firmware_name);
	retval = request_firmware(&fw_entry, data->firmware_name, &rmi_dev->dev);
	if (retval != 0) {
		dev_err(&rmi_dev->dev, "Firmware %s not available, code = %d\n",
			data->firmware_name, retval);
		goto done;
	}

	dev_dbg(&rmi_dev->dev, "Got firmware, size: %d.\n", (int)fw_entry->size);
	rmi_extract_header(fw_entry->data, 0, header);
	dev_dbg(&rmi_dev->dev, "Img checksum:           %#08X\n",
		header->checksum);
	dev_dbg(&rmi_dev->dev, "Img image size:         %d\n",
		header->image_size);
	dev_dbg(&rmi_dev->dev, "Img config size:        %d\n",
		header->config_size);
	dev_dbg(&rmi_dev->dev, "Img bootloader version: %d\n",
		header->bootloader_version);
	dev_dbg(&rmi_dev->dev, "Img product id:         %s\n",
		header->product_id);
	dev_dbg(&rmi_dev->dev, "Img product info:       %#04x %#04x\n",
		header->product_info[0], header->product_info[1]);

	if (header->image_size)
		data->firmware_data = fw_entry->data + F34_FW_IMAGE_OFFSET;
	if (header->config_size)
		data->config_data = fw_entry->data + F34_FW_IMAGE_OFFSET +
			header->image_size;

        if (header->bootloader_version == 2) {
                data->lockdown_data = fw_entry->data
                                        + RMI4_IMAGE_LOCKDOWN_DATA_V2_OFFSET;
                data->lockdown_size = RMI4_IMAGE_LOCKDOWN_DATA_V2_SIZE;
        } else if (header->bootloader_version <= 4) {
                data->lockdown_data = fw_entry->data
                                        + RMI4_IMAGE_LOCKDOWN_DATA_V3_OFFSET;
                data->lockdown_size = RMI4_IMAGE_LOCKDOWN_DATA_V3_SIZE;
        } else if (header->bootloader_version >= 5) {
                data->lockdown_data = fw_entry->data
                                        + RMI4_IMAGE_LOCKDOWN_DATA_V5_OFFSET;
                data->lockdown_size = RMI4_IMAGE_LOCKDOWN_DATA_V5_SIZE;
        }

        flash_area = rmi_go_nogo(data, header);
        if (flash_area != NONE) {
		if (flash_area == UI_FIRMWARE)
		{
			if(data->force)
				rmi_free_function_list(data->rmi_dev);
			rmi_reflash_firmware(data);
		}
		else if (flash_area == CONFIG_AREA)
			reflash_config(data);
		if(data->force)
			rmi_driver_redetect(data->rmi_dev);
		else
			rmi_reset_device(data);

                retval = read_f01_status(data);
                if (retval != 0) {
                        dev_err(&data->rmi_dev->dev,
                                "Failed to read F01 status. Code: %d.\n", retval);
                        goto done;
                }

                if (RMI_F01_STATUS_BOOTLOADER(data->device_status)) {
                        dev_warn(&rmi_dev->dev,
                                "Reset in bootloader mode (status 0x%X).\n",
                                RMI_F01_STATUS_CODE(data->device_status));
                } else
                        dev_info(&rmi_dev->dev,
                                "%s update succeed!\n",
                                flash_area == UI_FIRMWARE ?
                                "Firmware" : "Config");
        } else
                dev_dbg(&rmi_dev->dev, "Go/NoGo said don't reflash.\n");

	if (fw_entry)
		release_firmware(fw_entry);

done:
	getnstimeofday(&end);
	duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
	dev_dbg(&rmi_dev->dev, "Time to reflash: %lld ns.\n", duration_ns);

	kfree(data->firmware_name);
	//kfree(data);
	kfree(header);
	return;
}


static ssize_t rmi_reflash_img_name_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data *data = drv_data->reflash_data;

	return snprintf(buf, PAGE_SIZE, "%s\n", data->img_name);
}

static ssize_t rmi_reflash_img_name_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data *data = drv_data->reflash_data;

	if (test_and_set_bit(0, &data->busy))
		return -EBUSY;

	if (!count) {
		data->img_name = NULL;
	} else {
		strlcpy(data->name_buf, buf, count);
		data->img_name = strstrip(data->name_buf);
	}

	clear_bit(0, &data->busy);
	return count;
}

static ssize_t rmi_reflash_force_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data *data = drv_data->reflash_data;

	return snprintf(buf, PAGE_SIZE, "%u\n", data->force);
}

static ssize_t rmi_reflash_force_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data *data = drv_data->reflash_data;
	int retval;
	unsigned long val;

	if (test_and_set_bit(0, &data->busy))
		return -EBUSY;

	retval = kstrtoul(buf, 10, &val);
	if (retval)
		count = retval;
	else
		data->force = !!val;

	clear_bit(0, &data->busy);

	return count;
}

static ssize_t rmi_reflash_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data *data = drv_data->reflash_data;

	return snprintf(buf, PAGE_SIZE, "%u\n", test_bit(0, &data->busy));
}

static ssize_t rmi_reflash_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int retval;
	unsigned long val;
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data *data = drv_data->reflash_data;

	retval = kstrtoul(buf, 10, &val);
	if (retval)
		return retval;

	if (test_and_set_bit(0, &data->busy))
		return -EBUSY;

	if (val)
		/*
		 * TODO: Here we start a work thread to go do the reflash, but
		 * maybe we can just use request_firmware_timeout().
		 */
		schedule_work(&data->reflash_work);
	else
		clear_bit(0, &data->busy);

	return count;
}

static void rmi_reflash_work(struct work_struct *work)
{
	struct rmi_reflash_data *data =
		container_of(work, struct rmi_reflash_data, reflash_work);
	struct rmi_device *rmi_dev = data->rmi_dev;
	int error;

	dev_dbg(&rmi_dev->dev, "%s runs.\n", __func__);
	error = rmi_device_reflash(rmi_dev);
	if (error < 0)
		dev_err(&rmi_dev->dev, "Reflash attempt failed with code: %d.",
			error);
	clear_bit(0, &data->busy);
	sysfs_notify(&rmi_dev->dev.kobj,NULL,"reflash");
}

static struct device_attribute dev_attr_reflash_force = __ATTR(reflash_force,(RMI_RW_ATTR),rmi_reflash_force_show, rmi_reflash_force_store);
static struct device_attribute dev_attr_reflash_name = __ATTR(reflash_name,(RMI_RW_ATTR),rmi_reflash_img_name_show, rmi_reflash_img_name_store);
static struct device_attribute dev_attr_reflash = __ATTR(reflash,(RMI_RW_ATTR),rmi_reflash_show, rmi_reflash_store);

static struct attribute *rmi_reflash_attrs[] = {
	&dev_attr_reflash_force.attr,
	&dev_attr_reflash_name.attr,
	&dev_attr_reflash.attr,
	NULL
};

static const struct attribute_group rmi_reflash_attributes = {
	.attrs = rmi_reflash_attrs,
};

void rmi_reflash_init(struct rmi_device *rmi_dev)
{
	int error;
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data *data;

	dev_dbg(&rmi_dev->dev, "%s called.\n", __func__);

	data = devm_kzalloc(&rmi_dev->dev, sizeof(struct rmi_reflash_data),
			    GFP_KERNEL);

	error = sysfs_create_group(&rmi_dev->dev.kobj, &rmi_reflash_attributes);
	if (error) {
		dev_warn(&rmi_dev->dev, "Failed to create reflash sysfs attributes.\n");
		return;
	}

	INIT_WORK(&data->reflash_work, rmi_reflash_work);
	data->rmi_dev = rmi_dev;
	drv_data->reflash_data = data;
}

void rmi_reflash_cleanup(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data *data = drv_data->reflash_data;

	sysfs_remove_group(&rmi_dev->dev.kobj, &rmi_reflash_attributes);
	devm_kfree(&rmi_dev->dev, data);
}
