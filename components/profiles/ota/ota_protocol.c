/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/


#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"

#include "hal_mcu.h"
//#include "ota.h"
#include "ota_service.h"
#include "ota_protocol.h"
#include "ota_flash.h"
#include "flash.h"
#include "crc16.h"
#include "version.h"
#include "log.h"
#include "error.h"


#define OTA_MODE_OTA_APPLICATION  0  
#define OTA_MODE_OTA_FCT          1
#define OTA_MODE_OTA              2
#define OTA_MODE_RESOURCE         3

#define OTA_MODE_SELECT_REG 0x4000f034


#define OTA_BLOCK_REQ_TIMEOUT    4000
#define OTA_BLOCK_BURST_TIMEOUT   1000


#define Bytes2U32(u32val, b) {u32val = ((uint32_t)(b[0]&0xff)) | (((uint32_t)(b[1]&0xff))<<8)| (((uint32_t)(b[2]&0xff))<<16)| (((uint32_t)(b[3]&0xff))<<24);}

#define Bytes2U16(u16val, b) {u16val = ((uint16_t)(b[0]&0xff)) | (((uint16_t)(b[1]&0xff))<<8);}


enum{
  OTA_ST_UNCONNECTED = 0,
  OTA_ST_CONNECTED,
  //OTA_ST_PARAM,
  OTA_ST_WAIT_PARTITION_INFO,
  OTA_ST_DATA,
  OTA_ST_COMPLETE,
  OTA_ST_ERROR,
};


typedef struct{
  uint32_t  flash_addr;
  uint32_t  run_addr;
  uint32_t  size;
  uint16_t  checksum;
}ota_part_t;

typedef struct{
  uint8_t   ota_state;
  uint8_t   task_id;
  uint16_t  tm_evt;

  bool      ota_resource; //just upgrade resource data
  uint16_t  mtu_a;
  bool      notif_en;
  uint8_t   part_num;
  uint8_t   current_part;

  uint8_t   bank_mode;  //single bank or 
  
  uint32_t  bank_addr;

  bool      reboot_flag;
  //uint8_t   param_size;
  //uint8_t   param_offset;
  //uint32_t  param_buf[MAX_OTA_PARAM_SIZE/4];

  ota_part_t part[MAX_SECT_SUPPORT];

  uint32_t  block_offset;
  
  uint32_t  block_offset_retry;
  
  
  uint8_t*  partition_buf;
}ota_context_t;

#define OTA_PBUF_SIZE (64*1024+16)
uint8_t ota_patition_buffer[OTA_PBUF_SIZE] __attribute__((section("ota_partition_buffer_area")));

static uint16_t s_ota_burst_size = 16;

static bool s_ota_resource = FALSE;

static ota_context_t s_ota_ctx;


extern void jump_to_application(uint32_t run_addr);
extern bool verify_mic(unsigned char* buf, int size);
extern bool finidv(unsigned char* iv);

static void start_timer(uint32_t timeout)
{
  osal_start_timerEx(s_ota_ctx.task_id, s_ota_ctx.tm_evt, (uint32)timeout);
}
static void stop_timer(void)
{
  osal_clear_event(s_ota_ctx.task_id, s_ota_ctx.tm_evt);
  osal_stop_timerEx(s_ota_ctx.task_id, s_ota_ctx.tm_evt);
}


static void reset_ctx(void){

  uint8_t task_id;
  uint16_t tm_evt;

  task_id  = s_ota_ctx.task_id;
  tm_evt = s_ota_ctx.tm_evt;
  
  osal_memset(&s_ota_ctx, 0, sizeof(s_ota_ctx));


  s_ota_ctx.bank_mode = CFG_OTA_BANK_MODE;
  s_ota_ctx.ota_resource = s_ota_resource;
  s_ota_ctx.mtu_a = 20;
	s_ota_ctx.task_id = task_id;
	s_ota_ctx.tm_evt = tm_evt;
}

static int sector_crc(void)
{
  uint16 crc = 0;
  ota_part_t* ppart = NULL;

  ppart = &s_ota_ctx.part[s_ota_ctx.current_part]; 
  crc = crc16(0, (void*)s_ota_ctx.partition_buf, ppart->size);
  if(crc != ppart->checksum)
    return PPlus_ERR_OTA_CRC;
  return PPlus_SUCCESS;
}



static int sector_crypto(void)
{
  bool chk = FALSE;
  ota_part_t* ppart = NULL;
  uint8_t iv[16];

  if(s_ota_ctx.ota_resource)//resource no crypto
    return PPlus_SUCCESS;
  
  if(finidv(iv) == FALSE)
    return PPlus_SUCCESS;

  ppart = &s_ota_ctx.part[s_ota_ctx.current_part];
  chk = verify_mic(s_ota_ctx.partition_buf,ppart->size);
  if(chk == FALSE)
    return PPlus_ERR_OTA_CRYPTO;

  ppart->size -= 4; //remove MIC when store to flash
  ppart->checksum = crc16(0, (void*)s_ota_ctx.partition_buf, ppart->size); //figure out new checksum because the data length changed
  
  return PPlus_SUCCESS;
}


static void response(int state_cmd, int err)
{
  attHandleValueNoti_t notif;
  
  osal_memset(&notif, 0, sizeof(notif));

  notif.len = 2;
  notif.value[0] = (uint8_t)err;
  notif.value[1] = (uint8_t)state_cmd;

  ota_Notify(&notif);
}

static void response_err(int error)
{
  attHandleValueNoti_t notif;
  
  osal_memset(&notif, 0, sizeof(notif));

  notif.len = 1;
  notif.value[0] = error;
  notif.value[1] = 0xff;

  ota_Notify(&notif);

}

static void handle_error_fatal(int error)
{
  response_err(error);
  if(s_ota_ctx.notif_en){
    s_ota_ctx.ota_state = OTA_ST_CONNECTED;
    s_ota_ctx.part_num = 0;
    s_ota_ctx.partition_buf = ota_patition_buffer;
  }
  else
  {
    reset_ctx();
  }
}

static void handle_error_state(void)
{
  response_err(PPlus_ERR_OTA_INVALID_STATE);
  if(s_ota_ctx.notif_en){
    s_ota_ctx.ota_state = OTA_ST_CONNECTED;
    s_ota_ctx.part_num = 0;
    s_ota_ctx.partition_buf = ota_patition_buffer;
  }
  else
  {
    reset_ctx();
  }
}

bool validate_partition_parameter(ota_part_t* ppart)
{
    if(s_ota_ctx.ota_resource){
      if(ppart->flash_addr != 0)
        return FALSE;
      if(ppart->run_addr&OTAF_BASE_ADDR != OTAF_BASE_ADDR || ppart->run_addr + ppart->size > OTAF_END_ADDR+1)
        return FALSE;
        
      if(ppart->run_addr < OTAF_1st_BOOTINFO_ADDR + OTAF_1st_BOOTINFO_SIZE)
        return FALSE;
    }
    else if(ppart->run_addr == ppart->flash_addr)
    {
      //check if address out of flash area
      if(ppart->flash_addr > OTAF_END_ADDR || ppart->flash_addr < OTAF_BASE_ADDR)
        return FALSE;
      //for XIP, only No FCT and single bank allowed
      if(USE_FCT && CFG_OTA_BANK_MODE!=OTA_SINGLE_BANK)
        return FALSE;
    }
    else{
      if(ppart->run_addr < SRAM0_BASE_ADDRESS || ppart->run_addr > SRAM0_BASE_ADDRESS + 138*1024)
        return FALSE;
      if((ppart->flash_addr | OTAF_BASE_ADDR) + ppart->size > OTAF_END_ADDR+1)
        return FALSE;
    }
    return TRUE;
}

static void handle_error(int error)
{
  response_err(error);
  if(s_ota_ctx.notif_en){
    s_ota_ctx.ota_state = OTA_ST_CONNECTED;
    s_ota_ctx.part_num = 0;
    s_ota_ctx.partition_buf = ota_patition_buffer;
  }
  else
  {
    reset_ctx();
  }

}

void process_ctrl_cmd(uint8_t* cmdbuf, uint8_t size){
  ota_cmd_t cmd;
  int ret = PPlus_SUCCESS;
  if(size > sizeof(cmd)){
    return;
  }
  osal_memcpy(&cmd, cmdbuf, size);
  switch(cmd.cmd){
#ifdef CFG_OTA_MESH
  case OTA_CMD_START_OTA:
    if(s_ota_ctx.ota_state != OTA_ST_CONNECTED){
      //case invalid state
      handle_error_state();
      break;
    }
    
    if(cmd.p.start.sector_num > MAX_SECT_SUPPORT){
      //case invalid state
      handle_error_fatal(PPlus_ERR_INVALID_PARAM);
      break;
    }
    
    s_ota_ctx.bank_mode = CFG_OTA_BANK_MODE;
    s_ota_ctx.ota_resource = FALSE;
    ota_flash_read_bootsector(&s_ota_ctx.bank_addr);

    s_ota_ctx.ota_state = OTA_ST_WAIT_PARTITION_INFO;
    s_ota_ctx.part_num = cmd.p.start.sector_num;
    //s_ota_ctx.param_size = cmd.p.start.param_size;
    s_ota_ctx.partition_buf = ota_patition_buffer;
    osal_memset(&(s_ota_ctx.part[0]), 0, sizeof(ota_part_t)*MAX_SECT_SUPPORT);
    //osal_memset(&(s_ota_ctx.param_buf[0]), 0xff, MAX_OTA_PARAM_SIZE);

    s_ota_burst_size = 16;
    if(cmd.p.start.burst_size > 0)
      s_ota_burst_size = cmd.p.start.burst_size;
    if(cmd.p.start.burst_size == 0xff)
      s_ota_burst_size = 0xffff;

    AT_LOG("s_ota_burst_size is %x\n", s_ota_burst_size);
    ret = otafm_format();
    
    response(OTA_RSP_START_OTA, ret);
    break;
#else
  case OTA_CMD_START_OTA:
    if(s_ota_ctx.ota_state != OTA_ST_CONNECTED){
      //case invalid state
      handle_error_state();
      break;
    }
    
    if(cmd.p.start.sector_num > MAX_SECT_SUPPORT){
      //case invalid state
      handle_error_fatal(PPlus_ERR_INVALID_PARAM);
      break;
    }
    
    s_ota_ctx.bank_mode = CFG_OTA_BANK_MODE;
    s_ota_ctx.ota_resource = s_ota_resource;
    ota_flash_read_bootsector(&s_ota_ctx.bank_addr);

    s_ota_ctx.ota_state = OTA_ST_WAIT_PARTITION_INFO;
    s_ota_ctx.part_num = cmd.p.start.sector_num;
    //s_ota_ctx.param_size = cmd.p.start.param_size;
    s_ota_ctx.partition_buf = ota_patition_buffer;
    osal_memset(&(s_ota_ctx.part[0]), 0, sizeof(ota_part_t)*MAX_SECT_SUPPORT);
    //osal_memset(&(s_ota_ctx.param_buf[0]), 0xff, MAX_OTA_PARAM_SIZE);

    s_ota_burst_size = 16;
    if(cmd.p.start.burst_size > 0)
      s_ota_burst_size = cmd.p.start.burst_size;
    if(cmd.p.start.burst_size == 0xff)
      s_ota_burst_size = 0xffff;

    AT_LOG("s_ota_burst_size is %x\n", s_ota_burst_size);
    if(!s_ota_ctx.ota_resource){
      //if(cmd.p.start.param_size >0)
      //  s_ota_ctx.ota_state = OTA_ST_PARAM;
      ret = ota_flash_erase(s_ota_ctx.bank_addr);
    }
    
    response(OTA_RSP_START_OTA, ret);
    break;
#endif
  case OTA_CMD_PARTITION_INFO:
  {
    uint8_t idx = cmd.p.part.index;
    ota_part_t* ppart = &s_ota_ctx.part[idx];
    if(s_ota_ctx.ota_state != OTA_ST_WAIT_PARTITION_INFO){
      //case invalid state
      handle_error_state();
      break;
    }

    Bytes2U32(ppart->flash_addr,cmd.p.part.flash_addr);
    Bytes2U32(ppart->run_addr,cmd.p.part.run_addr);
    Bytes2U32(ppart->size,cmd.p.part.size);
    Bytes2U16(ppart->checksum,cmd.p.part.checksum);

    //check parameter
    if(!validate_partition_parameter(ppart)){
      handle_error(PPlus_ERR_INVALID_PARAM);
      break;
    }

    s_ota_ctx.ota_state = OTA_ST_DATA;
    
    s_ota_ctx.current_part = idx;
    s_ota_ctx.block_offset = 0;
    s_ota_ctx.block_offset_retry = 0;
    
    s_ota_ctx.partition_buf = ota_patition_buffer;
    osal_memset(ota_patition_buffer, 0xff, OTA_PBUF_SIZE);
    
    response(OTA_RSP_PARTITION_INFO, PPlus_SUCCESS);
    break;
  }  
/*  case OTA_CMD_BLOCK_INFO:
  {
    uint8_t b_idx = cmd.p.block.index;
    if(s_ota_ctx.ota_state != OTA_ST_WAIT_BLOCK_INFO){
      //case invalid state
      handle_error_state();
      return;
    }
    s_ota_ctx.ota_state = OTA_ST_DATA;
    s_ota_ctx.block_idx = b_idx;
    s_ota_ctx.block_offset = 0;
    s_ota_ctx.block_offset_retry = 0;
    Bytes2U16(s_ota_ctx.block_size ,cmd.p.block.size);
    response(OTA_RSP_BLOCK_INFO, PPlus_SUCCESS);
    start_timer(OTA_BLOCK_REQ_TIMEOUT);
    break;
  }  */
  case OTA_CMD_REBOOT:
  {
    s_ota_ctx.reboot_flag = FALSE;
    if(size == 1){
      NVIC_SystemReset();
    }
    else if(size == 2){
      if(cmd.p.reboot_flag == 1)
      {
        s_ota_ctx.reboot_flag = TRUE;
        response(OTA_RSP_REBOOT, PPlus_SUCCESS);
        break;
      }
    }
    response(OTA_RSP_REBOOT, PPlus_ERR_INVALID_PARAM);
    break;
  }
  case OTA_CMD_ERASE:
  {
    uint32_t flash_addr;
    uint32_t flash_size;
    if(s_ota_ctx.ota_state != OTA_ST_WAIT_PARTITION_INFO){
      //case invalid state
      handle_error_state();
      break;
    }
    if(!s_ota_ctx.ota_resource){
      //case invalid state
      handle_error_state();
      break;
    }
    Bytes2U32(flash_addr,cmd.p.erase.flash_addr);
    Bytes2U32(flash_size,cmd.p.erase.size);

    //erase
    ret = ota_flash_erase_area(flash_addr, flash_size);
    if(ret != PPlus_SUCCESS){
      handle_error(ret);
      break;
    }
    response(OTA_RSP_ERASE, PPlus_SUCCESS);
    break;
  }
  default:
    s_ota_ctx.ota_state = OTA_ST_ERROR;
    response(OTA_RSP_ERROR, PPlus_ERR_NOT_SUPPORTED);
    break;
  }
}

/*
boot sector, total 256 bytes, 64 words:
count by words:
(0)             : number of(N)
(1~3)           : reserved
(4~7)           : partition 1 information: flash address, run address, partition size, checksum
...
(N*4 ~ (N*4+3)  : partition N information: flash address, run address, partition size, checksum 

(20 ~63)        : reserved
*/



#ifdef CFG_OTA_MESH
static int write_app_boot_sector(void)
{
  //write application boot sector data
  int idx;
  int ret;
  uint32_t bs[4];
  osal_memset(bs, 0, 16);
  bs[0] = 0x4641544f; //"OTAF"
  bs[1] = s_ota_ctx.part_num;
  bs[2] = 0xffffffff;
  bs[3] = 0xffffffff;
  ret = otafm_write_boot_sector(bs, 16, 0);
  if(ret)
    return ret;
  for(idx = 0; idx < s_ota_ctx.part_num; idx ++){
    bs[0]   = s_ota_ctx.part[idx].flash_addr;
    bs[1]   = s_ota_ctx.part[idx].run_addr;
    bs[2]   = s_ota_ctx.part[idx].size;
    bs[3]   = (uint32_t)s_ota_ctx.part[idx].checksum;
    ret = otafm_write_boot_sector(bs, 16, 16*(idx+1));
    if(ret)
      return ret;
  }
    
  return ret;
}

static void partition_program(void)
{
  int ret;
  ota_part_t* ppart = NULL;
  uint32_t flash_addr = 0;

  ppart = &s_ota_ctx.part[s_ota_ctx.current_part];
  flash_addr = ppart->flash_addr;
  ret = otafm_write_partition(flash_addr, (uint32_t*)s_ota_ctx.partition_buf, ppart->size);

  if(ret != PPlus_SUCCESS){
    handle_error(ret);
    return;
  }
      
  //case all partition data finished
  if(s_ota_ctx.current_part+1 == s_ota_ctx.part_num){
    if(!s_ota_ctx.ota_resource)
      ret = write_app_boot_sector();
    s_ota_ctx.ota_state = OTA_ST_COMPLETE;
    response(OTA_RSP_OTA_COMPLETE,PPlus_SUCCESS);
  }
  else{
    s_ota_ctx.ota_state = OTA_ST_WAIT_PARTITION_INFO;
    response(OTA_RSP_PARTITION_COMPLETE, PPlus_SUCCESS);
  }
  
}
#else //normal OTA
static int write_app_boot_sector(void)
{
  //write application boot sector data
  int idx;
  int ret;
  uint32_t bs[4];
  osal_memset(bs, 0, 16);

  flash_sector_erase(OTAF_2nd_BOOTINFO_ADDR);
  
  bs[0] = s_ota_ctx.part_num;
  if(CFG_OTA_BANK_MODE==OTA_SINGLE_BANK)
    bs[1] = OTAF_SINGLE_BANK;
  else
    bs[1] = (s_ota_ctx.bank_addr == OTAF_APP_BANK_0_ADDR) ? OTAF_DUAL_BANK_0 : OTAF_DUAL_BANK_1;
  bs[2] = 0;
  bs[3] = 0xffffffff;
  ret = ota_flash_write_boot_sector(bs, 16, 0);
  if(ret)
    return ret;
  
  for(idx = 0; idx < s_ota_ctx.part_num; idx ++){
    bs[0]   = s_ota_ctx.part[idx].flash_addr;
    bs[1]   = s_ota_ctx.part[idx].run_addr;
    bs[2]   = s_ota_ctx.part[idx].size;
    bs[3]   = (uint32_t)s_ota_ctx.part[idx].checksum;
    ret = ota_flash_write_boot_sector(bs, 16, 16*(idx+1));
    if(ret)
      return ret;
  }
  return PPlus_SUCCESS;
}

static void partition_program(void)
{
  int ret;
  ota_part_t* ppart = NULL;
  uint32_t flash_addr = 0;

  ppart = &s_ota_ctx.part[s_ota_ctx.current_part];
  //write partition data
  if(s_ota_ctx.ota_resource)
  {
    flash_addr = ppart->run_addr;
  }
  else if(ppart->flash_addr == ppart->run_addr)
  {
    uint32_t er_addr, er_size;
    flash_addr = ppart->run_addr;
    er_addr = flash_addr & 0xfffff000;//make address 4k align
    er_size = flash_addr + ppart->size + 0xfff - er_addr;
    er_size = er_size &  0xfffff000;
    ret = ota_flash_erase_area(er_addr, er_size);
    if(ret != PPlus_SUCCESS){
      handle_error(ret);
      return;
    }
  }
  else
  {
    flash_addr = ppart->flash_addr + s_ota_ctx.bank_addr;
  }
  ret = ota_flash_write_partition(flash_addr, (uint32_t*)s_ota_ctx.partition_buf, ppart->size);

  if(ret != PPlus_SUCCESS){
    handle_error(ret);
    return;
  }
      
  //case all partition data finished
  if(s_ota_ctx.current_part+1 == s_ota_ctx.part_num){
    if(!s_ota_ctx.ota_resource)
      ret = write_app_boot_sector();
    s_ota_ctx.ota_state = OTA_ST_COMPLETE;
    response(OTA_RSP_OTA_COMPLETE,PPlus_SUCCESS);
  }
  else{
    s_ota_ctx.ota_state = OTA_ST_WAIT_PARTITION_INFO;
    response(OTA_RSP_PARTITION_COMPLETE, PPlus_SUCCESS);
  }
  
}
#endif
static void process_ota_partition_data(uint8_t* data, uint8_t size)
{
  uint32_t block_offset = s_ota_ctx.block_offset;
  ota_part_t* ppart = NULL;

  ppart = &s_ota_ctx.part[s_ota_ctx.current_part];
  osal_memcpy(s_ota_ctx.partition_buf + block_offset, data, size);
  
  block_offset += size;
  AT_LOG("boff[%d], rty[%d]\n", block_offset,s_ota_ctx.block_offset_retry);
  if(block_offset > ppart->size){
    handle_error(PPlus_ERR_OTA_DATA_SIZE);
    return;
  }

  s_ota_ctx.block_offset = block_offset;

  if(s_ota_ctx.block_offset - s_ota_ctx.block_offset_retry == OTA_DATA_BURST_SIZE){
      response(OTA_RSP_BLOCK_BURST, PPlus_SUCCESS);
      s_ota_ctx.block_offset_retry = s_ota_ctx.block_offset;
      start_timer(OTA_BLOCK_REQ_TIMEOUT);
  }
  else{
    start_timer(OTA_BLOCK_BURST_TIMEOUT);
  }
	
  if(block_offset == ppart->size){
    stop_timer();
    //cec check
    if(sector_crc() != PPlus_SUCCESS){
      handle_error(PPlus_ERR_OTA_CRC);
      return;
    }
    if(sector_crypto()!=PPlus_SUCCESS){
      handle_error(PPlus_ERR_OTA_CRYPTO);
      return;
    }

    partition_program();
    return;
  }
  
}

/*
static void process_ota_param_data(uint8_t* data, uint8_t size)
{
  uint8_t offset = s_ota_ctx.param_offset;
  uint8_t* param_buf = (uint8_t*)(s_ota_ctx.param_buf);

  osal_memcpy(param_buf + offset, data, size);
  offset += size;
  
  if(offset > s_ota_ctx.param_size){
    handle_error(PPlus_ERR_OTA_DATA_SIZE);
    return;
  }
  
  if(offset == s_ota_ctx.param_size){
    //case param data finished
    s_ota_ctx.ota_state = OTA_ST_WAIT_PARTITION_INFO;
    response(OTA_RSP_PARAM, PPlus_SUCCESS);
  }
  return;
}
*/


void process_ota_data(uint8_t* data, uint8_t size){
  switch(s_ota_ctx.ota_state){
  case OTA_ST_DATA:
    process_ota_partition_data(data, size);
    break;
  //case OTA_ST_PARAM:
  //  process_ota_param_data(data, size);
  //  break;
  default:
    handle_error_state();
    break;
  }
}



void process_service_evt(ota_Evt_t* pev)
{
  AT_LOG("PSE: ev[%d], st[%d ]\n",pev->ev, s_ota_ctx.ota_state);
  switch(pev->ev){
  case OTA_EVT_CONTROL:
    process_ctrl_cmd(pev->data, pev->size);
    break;
  case OTA_EVT_DATA:
    process_ota_data(pev->data, pev->size);
    break;
  case OTA_EVT_CONNECTED:
    break;
  case OTA_EVT_DISCONNECTED:
    LOG("[OTA_EVT_DISCONNECTED]Disconnected!\n");
    if(s_ota_ctx.reboot_flag){
      LOG("Reboot!\n");
      NVIC_SystemReset();
    }
    reset_ctx();
    break;
  case OTA_EVT_NOTIF_ENABLE:
    s_ota_ctx.ota_state = OTA_ST_CONNECTED;
    s_ota_ctx.notif_en = TRUE;
    break;
  case OTA_EVT_NOTIF_DISABLE:
    s_ota_ctx.ota_state = OTA_ST_UNCONNECTED;
    s_ota_ctx.notif_en = FALSE;
    break;
  default:
		break;
  }
}


void __attribute__((section("ota_app_loader_area"))) otaProtocol_RunApp(void)
{
  uint32_t* prunpc = (uint32_t*)(0x1fff4800);
  jump_to_application(prunpc[1]);
}

int __attribute__((section("ota_app_loader_area"))) run_application(void)
{
  int ret;
  HAL_ENTER_CRITICAL_SECTION();
  ret = ota_flash_load_app();
  if(ret == PPlus_SUCCESS){
    otaProtocol_RunApp();
  }
  
  HAL_EXIT_CRITICAL_SECTION();
	return PPlus_SUCCESS;

}

int run_fct(void)
{
  HAL_ENTER_CRITICAL_SECTION();
	{
#if(CFG_FLASH >= 512)
	  int ret = ota_flash_load_fct();
      if(ret != PPlus_SUCCESS)
#endif
        otaProtocol_RunApp();
  }
  HAL_EXIT_CRITICAL_SECTION();
	return PPlus_SUCCESS;
}

void otaProtocol_mtu(uint16_t mtu)
{
  s_ota_ctx.mtu_a = mtu - 3;
  //response(OTA_RSP_BLOCK_BURST, PPlus_ERR_OTA_BAD_DATA);
}

void otaProtocol_TimerEvt(void)
{
  s_ota_ctx.block_offset = s_ota_ctx.block_offset_retry;
  response(OTA_RSP_BLOCK_BURST, PPlus_ERR_OTA_BAD_DATA);
}

void otaProtocol_BootMode(void)
{
  uint32_t ota_mode = read_reg(OTA_MODE_SELECT_REG) & 0xf;
  uint32_t reg = ((SDK_VER_MAJOR &0xf) << 4) | ((SDK_VER_MINOR &0xf)<< 8) | ((SDK_VER_REVISION &0xff)<<12);
  #ifdef SDK_VER_TEST_BUILD
    reg |= (((SDK_VER_TEST_BUILD - 'a' + 1)&0xf) << 20);
  #endif

  write_reg(OTA_MODE_SELECT_REG,reg);
  
  switch(ota_mode){
  case OTA_MODE_OTA_APPLICATION:
    run_application();
    break;
  case OTA_MODE_OTA_FCT:
    run_fct();
    break;
  case OTA_MODE_RESOURCE:
    s_ota_resource = TRUE;
    break;
  default:
    break;
  }
}



int otaProtocol_init(uint8_t task_id, uint16_t tm_evt)
{
  
	s_ota_ctx.task_id = task_id;
	s_ota_ctx.tm_evt = tm_evt;
	
  reset_ctx();
  
  ota_flash_read_bootsector(&s_ota_ctx.bank_addr);
  
  ota_AddService(process_service_evt);
  return PPlus_SUCCESS;
}

