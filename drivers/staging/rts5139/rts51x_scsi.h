/* Driver for Realtek RTS51xx USB card reader
 * Header file
 *
 * Copyright(c) 2009 Realtek Semiconductor Corp. All rights reserved.  
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http:
 *
 * Author:
 *   wwang (wei_wang@realsil.com.cn)
 *   No. 450, Shenhu Road, Suzhou Industry Park, Suzhou, China
 */

#ifndef __RTS51X_SCSI_H
#define __RTS51X_SCSI_H

#include <linux/usb.h>
#include <linux/usb_usual.h>
#include <linux/blkdev.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <scsi/scsi_host.h>

#include "rts51x_chip.h"

#define MS_SP_CMND		0xFA
#define MS_FORMAT		0xA0
#define GET_MS_INFORMATION	0xB0

#define VENDOR_CMND		0xF0

#define READ_STATUS		0x09

#define READ_MEM		0x0D
#define WRITE_MEM		0x0E
#define GET_BUS_WIDTH		0x13
#define GET_SD_CSD		0x14
#define TOGGLE_GPIO		0x15
#define TRACE_MSG		0x18

#define SCSI_APP_CMD		0x10

#define PP_READ10		0x1A
#define PP_WRITE10		0x0A
#define READ_HOST_REG		0x1D
#define WRITE_HOST_REG		0x0D
#define SET_VAR			0x05
#define GET_VAR			0x15
#define DMA_READ		0x16
#define DMA_WRITE		0x06
#define GET_DEV_STATUS		0x10
#define SET_CHIP_MODE		0x27
#define SUIT_CMD		0xE0
#define WRITE_PHY		0x07
#define READ_PHY		0x17

#define INIT_BATCHCMD		0x41
#define ADD_BATCHCMD		0x42
#define SEND_BATCHCMD		0x43
#define GET_BATCHRSP		0x44

#ifdef SUPPORT_CPRM
#define SD_PASS_THRU_MODE	0xD0
#define SD_EXECUTE_NO_DATA	0xD1
#define SD_EXECUTE_READ		0xD2
#define SD_EXECUTE_WRITE	0xD3
#define SD_GET_RSP		0xD4
#define SD_HW_RST		0xD6
#endif

#ifdef SUPPORT_MAGIC_GATE
#define CMD_MSPRO_MG_RKEY	0xA4   
#define CMD_MSPRO_MG_SKEY	0xA3   

#define KC_MG_R_PRO		0xBE   

#define KF_SET_LEAF_ID		0x31   
#define KF_GET_LOC_EKB		0x32   
#define KF_CHG_HOST		0x33   
#define KF_RSP_CHG		0x34   
#define KF_RSP_HOST		0x35   
#define KF_GET_ICV		0x36   
#define KF_SET_ICV		0x37   
#endif

struct rts51x_chip;

/*-----------------------------------
    Start-Stop-Unit
-----------------------------------*/
#define STOP_MEDIUM			0x00    
#define MAKE_MEDIUM_READY		0x01    
#define UNLOAD_MEDIUM			0x02    
#define LOAD_MEDIUM			0x03    

/*-----------------------------------
    STANDARD_INQUIRY
-----------------------------------*/
#define QULIFIRE                0x00
#define AENC_FNC                0x00
#define TRML_IOP                0x00
#define REL_ADR                 0x00
#define WBUS_32                 0x00
#define WBUS_16                 0x00
#define SYNC                    0x00
#define LINKED                  0x00
#define CMD_QUE                 0x00
#define SFT_RE                  0x00

#define VEN_ID_LEN              8               
#define PRDCT_ID_LEN            16              
#define PRDCT_REV_LEN           4               

#define DRCT_ACCESS_DEV         0x00    
#define RMB_DISC                0x80    
#define ANSI_SCSI2              0x02    

#define SCSI                    0x00    

void scsi_show_command(struct scsi_cmnd *srb);
void set_sense_type(struct rts51x_chip *chip, unsigned int lun, int sense_type);
void set_sense_data(struct rts51x_chip *chip, unsigned int lun, u8 err_code, u8 sense_key, 
		u32 info, u8 asc, u8 ascq, u8 sns_key_info0, u16 sns_key_info1);
		
int rts51x_scsi_handler(struct scsi_cmnd *srb, struct rts51x_chip *chip);

#endif 
