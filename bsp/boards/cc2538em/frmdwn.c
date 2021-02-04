#include <stdint.h>
#include <stdio.h>
#include <stdalign.h>
#include "opendefs.h"
#include "osens.h"
#include "osens_itf.h"
#include "opentimers.h"
#include "scheduler.h"
#include "debug.h"
#include "udma.h"
#include "hw_udma.h"
#include "hw_flash_ctrl.h"
#include "openserial.h"
#include "leds.h"
#include "flash.h"
#if SENSOR_ACCEL
#include "acc_bma250.h"
#endif



osens_brd_id_t board_info;

#define MAX_FRAME_CHUNCK 128
#define MAX_CHUNCK_WRITE 8
#define OFFSET_FAKE 0

#define WRITE_FLASH_USING_DMA 0

#if WRITE_FLASH_USING_DMA
unsigned char ucDMAControlTable[1024] __attribute__ ((aligned(1024)));
#endif

osens_frm_t  osens_frm;
flash_wrwin_t flash_wrwin;
frm_info_t   frminfo;

uint16_t uirffcount=0;
uint8_t  u8chuncktype=0;
uint32_t u32chuncklen=0;
uint16_t value = 0x1234;


uint32_t countrff=0;
uint32_t counttask = 0;
uint32_t counttask2 = 0;
uint16_t lastpos=0;

static uint8_t u8winbuf[MAX_FRAME_CHUNCK];
uint8_t dstaddrrff=0;
uint8_t reset_timerId;

#define FRMWINDOW_START_ADDR (uint8_t *) &u8winbuf[0]

//=========================== prototypes ======================================
#if WRITE_FLASH_USING_DMA == 1
#if 0
uint8_t writeflash(uint32_t *srcaddr, uint32_t *dstaddr, uint8_t datatype, uint32_t bufLen);
#else
uint8_t writeflash(uint32_t srcaddr, uint32_t dstaddr, uint8_t datatype, uint32_t bufLen);
#endif
uint8_t writeflashDMA32(uint32_t srcaddr, uint32_t dstaddr);
#endif

void frmdwn_flash_handling_cb(opentimers_id_t id);
void frmdwn_flash_handling (void);

uint8_t readchunck(addr_cmd_t *plen, uint8_t *pbufin, uint8_t *pbufout);
uint8_t getAdLen(osens_frm_t *pucAddr);
uint8_t *setASLSVal(uint8_t lentype, addr_cmd_t *paddress, uint8_t *pbuf);
uint32_t getchunklenbytes(addr_cmd_t *plength);
uint8_t frm_fillinfo(uint8_t lentype, uint32_t *paddress, uint8_t *pbuf);
uint8_t frmdwn_frame_parser(uint8_t *pbuf, uint8_t count);
uint8_t runtestequal1 (uint8_t count);
uint8_t writelastwordflash(void);

uint8_t frmdwn_init(void)
{
#if FRM_DOWNLOAD
	memset(&board_info, 0, sizeof(board_info));
	
    //
    // Fill Source buffer (to be copied to flash) with some data
    //
	memset(&u8winbuf,0x00,sizeof(u8winbuf));

    reset_timerId  = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_OPENSERIAL);

	osens_frm.flashnewcmd = iFlashFrmInit;
	osens_frm.frwnewEndAddr = FRWNEW_START_ADDR + 200;

	opentimers_scheduleIn(
    	reset_timerId,
		100,
        TIME_MS,
        TIMER_PERIODIC,
		frmdwn_flash_handling_cb
    );
#endif
    return 1;
}


void frmdwn_flash_handling_cb(opentimers_id_t id){

	scheduler_push_task((task_cbt) frmdwn_flash_handling, TASKPRIO_OSENS_MAIN);
}

#if WRITE_FLASH_USING_DMA == 1
#if 0
uint8_t writeflash(uint32_t *srcaddr, uint32_t *dstaddr, uint8_t datatype, uint32_t bufLen){

    uint8_t ret=FALSE, i=0;
    uint32_t rest;

    rest = bufLen % 4;
	if ((bufLen == 0) || (bufLen > MAX_CHUNCK_WRITE) || (datatype != iAdLen4B) || (rest > 0) ){
		return ret;
	}

/*
	for (i=0;i<bufLen;i+=4) {
		ret = writeflashDMA32 ((uint32_t) srcaddr , (uint32_t) dstaddr);
		srcaddr++;
		dstaddr++;
	}
*/
	ret = FlashMainPageProgram( (uint32_t *) srcaddr,
			    			    (uint32_t) dstaddr,
						        (uint32_t) bufLen);

    return ret;
}

#else

uint8_t writeflash(uint32_t srcaddr, uint32_t dstaddr, uint8_t datatype, uint32_t bufLen){

    uint8_t ret=TRUE;
    uint32_t channelctrl=0;
    uint32_t len=0;


	if ((bufLen == 0) || (bufLen > MAX_CHUNCK_WRITE)){
		return ret;
	}

    //
    // Enable the uDMA controller.
    //
    uDMAEnable();

    //
    // Disable the uDMA channel to be used, before modifications are done.
    //
    uDMAChannelDisable(UDMA_CH2_FLASH);

    //
    // Set the base for the channel control table.
    //
    uDMAControlBaseSet(&ucDMAControlTable[0]);

    //
    // Assign the DMA channel
    //
    uDMAChannelAssign(UDMA_CH2_FLASH);

    //
    // Set attributes for the channel.
    //
    uDMAChannelAttributeDisable(UDMA_CH2_FLASH, UDMA_ATTR_HIGH_PRIORITY);

    //
    // Now set up the characteristics of the transfer.
    // 32-bit data size, with source increments in words (32 bits),
    // no destination increment.
    // A bus arbitration size of 1 must be used.
    //
/*
	switch (datatype){
		case iAdLen1B:
			channelctrl = UDMA_SIZE_8 | UDMA_SRC_INC_8  |UDMA_DST_INC_NONE | UDMA_ARB_1;
			break;
		case iAdLen2B:
			channelctrl = UDMA_SIZE_16 | UDMA_SRC_INC_16 |UDMA_DST_INC_NONE | UDMA_ARB_2;
			break;
		case iAdLen4B:
			channelctrl = UDMA_SIZE_32 | UDMA_SRC_INC_32 |UDMA_DST_INC_NONE | UDMA_ARB_1;
			break;
		default:
			break;
	}
*/
	channelctrl = UDMA_SIZE_32 | UDMA_SRC_INC_32 |UDMA_DST_INC_NONE | UDMA_ARB_1;

    uDMAChannelControlSet(UDMA_CH2_FLASH, channelctrl);

    //
    // Set transfer parameters.
    // Source address is the location of the data to write
    // and destination address is the FLASH_CTRL_FWDATA register.


    uDMAChannelTransferSet(UDMA_CH2_FLASH, UDMA_MODE_BASIC,  (void *) srcaddr,
                           (void *) FLASH_CTRL_FWDATA, bufLen);

    //
    // Asure that the flash controller is not busy.
    //
    while(HWREG(FLASH_CTRL_FCTL) & FLASH_CTRL_FCTL_BUSY)
    {
    }

    //
    // Initialize Flash control register without changing the cache mode.
    //
    HWREG(FLASH_CTRL_FCTL) &= FLASH_CTRL_FCTL_CM_M;

    //
    // Setup Flash Address register to address of first data word (32-bit)
    //

	HWREG(FLASH_CTRL_FADDR) = dstaddr;

    //
    // Finally, the DMA channel must be enabled.
    //
    uDMAChannelEnable(UDMA_CH2_FLASH);

    //
    // Set FCTL.WRITE, to trigger flash write
    //
    HWREG(FLASH_CTRL_FCTL) |= FLASH_CTRL_FCTL_WRITE;

    //
    // Wait until all words has been programmed.
    //
    while( HWREG(FLASH_CTRL_FCTL) & FLASH_CTRL_FCTL_FULL )
    {
    }

    //
    // Check if flash write was successfull
    //
    if (HWREG(FLASH_CTRL_FCTL) & FLASH_CTRL_FCTL_ABORT)
    { //Write error
          ret = TRUE;
    }
    else
    {  //write oK
    	  ret = FALSE;
    }

    //
    // Set control register back to reset value without changing the cache mode.
    //
    HWREG(FLASH_CTRL_FCTL) &= FLASH_CTRL_FCTL_CM_M;


    return ret;
}
#endif

uint8_t writeflashDMA32(uint32_t srcaddr, uint32_t dstaddr){

    uint8_t ret=TRUE;
    uint32_t channelctrl=0;
    uint32_t bufLen=4;
    uint8_t  datatype=iAdLen4B;

    //
    // Enable the uDMA controller.
    //
    uDMAEnable();

    //
    // Disable the uDMA channel to be used, before modifications are done.
    //
    uDMAChannelDisable(UDMA_CH2_FLASH);

    //
    // Set the base for the channel control table.
    //
    uDMAControlBaseSet(&ucDMAControlTable[0]);

    //
    // Assign the DMA channel
    //
    uDMAChannelAssign(UDMA_CH2_FLASH);

    //
    // Set attributes for the channel.
    //
    uDMAChannelAttributeDisable(UDMA_CH2_FLASH, UDMA_ATTR_HIGH_PRIORITY);

    //
    // Now set up the characteristics of the transfer.
    // 32-bit data size, with source increments in words (32 bits),
    // no destination increment.
    // A bus arbitration size of 1 must be used.
    //
   uDMAChannelControlSet(UDMA_CH2_FLASH, UDMA_SIZE_32 | UDMA_SRC_INC_32 |
                         UDMA_DST_INC_NONE | UDMA_ARB_1);

    //
    // Set transfer parameters.
    // Source address is the location of the data to write
    // and destination address is the FLASH_CTRL_FWDATA register.


    uDMAChannelTransferSet(UDMA_CH2_FLASH, UDMA_MODE_BASIC,  (void *) srcaddr,
                           (void *) FLASH_CTRL_FWDATA, bufLen);

    //
    // Asure that the flash controller is not busy.
    //
    while(HWREG(FLASH_CTRL_FCTL) & FLASH_CTRL_FCTL_BUSY)
    {
    }

    //
    // Initialize Flash control register without changing the cache mode.
    //
    HWREG(FLASH_CTRL_FCTL) &= FLASH_CTRL_FCTL_CM_M;

    //
    // Setup Flash Address register to address of first data word (32-bit)
    //

	HWREG(FLASH_CTRL_FADDR) = dstaddr;

    //
    // Finally, the DMA channel must be enabled.
    //
    uDMAChannelEnable(UDMA_CH2_FLASH);

    //
    // Set FCTL.WRITE, to trigger flash write
    //
    HWREG(FLASH_CTRL_FCTL) |= FLASH_CTRL_FCTL_WRITE;

    //
    // Wait until all words has been programmed.
    //
    while( HWREG(FLASH_CTRL_FCTL) & FLASH_CTRL_FCTL_FULL )
    {
    }

    //
    // Check if flash write was successfull
    //
    if (HWREG(FLASH_CTRL_FCTL) & FLASH_CTRL_FCTL_ABORT)
    { //Write error
  	   ret = TRUE;
    }
    else
    {  //write oK
        ret = FALSE;
    }

    //
    // Set control register back to reset value without changing the cache mode.
    //
    HWREG(FLASH_CTRL_FCTL) &= FLASH_CTRL_FCTL_CM_M;

    return ret;
}
#endif

void frmdwn_flash_handling(void){

	int32_t i32Res=0;
	uint8_t ret=FALSE;
	uint8_t mc;
	uint8_t count = 0;
	uint8_t retend = 0;
    uint32_t u32val = 0;
    //uint8_t buf[100];
    uint32_t dstaddr = 0,i=0;
    uint32_t srcaddr = 0;
    uint8_t lenaux=0;
    uint8_t u8bufaux[4];
    uint8_t auxpos=0;
    uint8_t len;
    uint32_t u32address;

	 if (osens_frm.flashnewcmd == iFlashFrmInit) {
		osens_frm.flashnewcmd = iFlashNone;

		/* PROVISORIO!!!! aqui eh feito uma pagina fake para somente simular valores para o "firmware old" */
#if 0
		i32Res = FlashMainPageErase(FRWOLD_START_ADDR);
		osens_frm.frameID = 0x0000;
		if (i32Res == 0){
			for (i=0;i<256;i+=4){
				u8bufaux[0] = i;
				u8bufaux[1] = u8bufaux[0] + 1;
				u8bufaux[2] = u8bufaux[1] + 1;
				u8bufaux[3] = u8bufaux[2] + 1;

				ret = FlashMainPageProgram( (uint32_t *) &u8bufaux[0],
											(uint32_t) (FRWOLD_START_ADDR+i),
											(uint32_t) 4);
			}
		}
#endif
		counttask = 0;
		counttask2 = 0;
		count = 0;
		lastpos = 0;

		#if ENABLE_DEBUG_RFF
		{
			 uint8_t pos=0;

			 rffbuf[pos++]= RFF_OSENS + 0;
			 rffbuf[pos++]= i32Res;
			 rffbuf[pos++]= (uint8_t) ret;

			 openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

			 uirffcount++;
		}
		#endif
		//osens_frm.flashnewcmd = iFlashErase;

	 }
	 else if (osens_frm.flashnewcmd == iFlashErase) {
 		osens_frm.flashnewcmd = iFlashNone;
 		u32address = FRWNEW_START_ADDR;

 		for (i=0;i<MAX_PAGES;i++){
 			i32Res = FlashMainPageErase(u32address);
 			u32address = u32address + PAGE_SIZE;
 		}

		osens_frm.frameID = 0x0000;
		flash_wrwin.dstaddrbegin =  0;
		flash_wrwin.dstaddrend   =  0;
		flash_wrwin.wrpending    =  0;
		dstaddrrff = 0;
		countrff = 0;

		#if ENABLE_DEBUG_RFF
		{
			 uint8_t pos=0;

			 rffbuf[pos++]= RFF_OSENS + 1;
			 rffbuf[pos++]= RFF_OSENS + 1;
			 rffbuf[pos++]= RFF_OSENS + 1;
			 rffbuf[pos++]= MAX_PAGES;
			 rffbuf[pos++]= (uint8_t) i32Res;

			 openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
		#endif

		if (i32Res > 0)
		{
			// log the error Adaptado...criar um codigo para o erro de download de firmware
			osens_frm.framestatus = ST_FRWUPD_ERROR_ERASE_FLASH;
			openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_WRITE_FLASH,
								  (errorparameter_t)osens_frm.framestatus,
								  (errorparameter_t)i32Res);
		}
		else
			osens_frm.framestatus = ST_FRWUPD_OK;

 	 }
 	 else if (osens_frm.flashnewcmd == iFlashChunck) {

		uint8_t *pframe = (uint8_t *) (osens_frm.payload+2);

		osens_frm.flashnewcmd = iFlashNone;
   	    osens_frm.framepos = 0;

		count = 0;
		retend = 0;

   	    while ((retend == 0)){
			//get the header (1 byte)
			osens_frm.header = pframe[osens_frm.framepos];
			osens_frm.framepos++;
			//bufaux[count]= osens_frm.header;

			ret = frmdwn_frame_parser(pframe,count);
            count++;

			//Pega o primeiro bit e verifica se existe mais comandos
			mc = MC_SHIFT (osens_frm.header);
			if ((mc == 0) || (ret==1))
				retend = 1;
   	    }

		#if 1
		{
			uint8_t pos=0;
			uint8_t *pucaux=(uint8_t *) &osens_frm.frameID;
			uint8_t *pucaux1=(uint8_t *) &flash_wrwin.dstaddrbegin;
			uint8_t *pucaux2=(uint8_t *) &flash_wrwin.dstaddrend;

			rffbuf[pos++] = 0x9A;
			rffbuf[pos++] = *(pucaux+1);
			rffbuf[pos++] = *(pucaux+0);
			rffbuf[pos++] = count;
			rffbuf[pos++] = *(pucaux1+1);
			rffbuf[pos++] = *(pucaux1+0);
			rffbuf[pos++] = *(pucaux2+1);
			rffbuf[pos++] = *(pucaux2+0);
			rffbuf[pos++] = flash_wrwin.wrpending;
			rffbuf[pos++] = ret;

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
		#endif

		osens_frm.framestatus = ST_FRWUPD_OK;
 	 }


	 if (counttask == 100) {
#if 0
		 if (counttask2 < 4)
			 runtestequal1(counttask2);
		counttask = 0;
		counttask2++;
		//leds_radio_toggle();
		if (counttask2 == 4){
			//verify if there is pending write
			//if (flash_wrwin.wrpending == 2){
		    //    writelastwordflash();
			//}
		}
#else
		counttask = 0;
#endif
	 }
	 else {
		 counttask++;
		 //leds_debug_toggle();
	 }

}

uint8_t writelastwordflash(void){
    uint32_t dstaddr;
    uint8_t ret;

 	dstaddr = (uint32_t) (FRWNEW_START_ADDR + flash_wrwin.dstaddrbegin);
	if ((dstaddr % 4) > 0)
		return 0;

	ret = FlashMainPageProgram( (uint32_t *) &u8winbuf[0],
								(uint32_t) dstaddr,
								(uint32_t) 4);
    return ret;
}

uint8_t writewinflash(uint32_t length){
    uint8_t ret=0, uc,i;
    uint32_t len=0, lenaux;
    uint8_t *pucbuf;
    uint32_t dstaddr,dstaddraux;
    uint8_t u8auxbuf[4];

    if (length < MAX_CHUNCK_WRITE)
		return ST_WRFLASH_WINDOW_NOT_FULL;

    if (length > MAX_FRAME_CHUNCK)
		return ST_WRFLASH_TOO_BIG;

    dstaddr = (uint32_t) (FRWNEW_START_ADDR + flash_wrwin.dstaddrbegin - OFFSET_FAKE);
    dstaddraux = dstaddr;

#if 1
{
	uint8_t pos=0;
	uint32_t dstaddraux = (uint32_t) (FRWNEW_START_ADDR + flash_wrwin.dstaddrbegin);
	uint8_t *paddr =  (uint8_t *) &dstaddraux;

	rffbuf[pos++] = 0x99;
	rffbuf[pos++] = (uint8_t) osens_frm.frameID;
	rffbuf[pos++] = (uint8_t) len;
	rffbuf[pos++] = *(paddr+3);
	rffbuf[pos++] = *(paddr+2);
	rffbuf[pos++] = *(paddr+1);
	rffbuf[pos++] = *(paddr+0);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

	if ((dstaddr % 4) > 0)
		return ST_WRFLASH_DST_ADDR_NOT_ALIGNED;

    lenaux = length % 4;
    if (lenaux == 0)
        len = length;
    else
        len = length - lenaux;

#if WRITE_FLASH_USING_DMA == 1
	ret =  writeflash( (uint32_t ) FRMWINDOW_START_ADDR,
					  (uint32_t ) dstaddr ,
					  iAdLen4B ,
					  (uint32_t) len);
#else

	for ( i=0; i < len ; i+=4 ) {
		ret = FlashMainPageProgram( (uint32_t *) &u8winbuf[i],
									(uint32_t) (dstaddr+i),
									(uint32_t) 4);
		if (ret != 0){
            // log the error
			osens_frm.framestatus = ST_FRWUPD_GENERAL_ERROR;
            openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_WRITE_FLASH,
                              (errorparameter_t)osens_frm.framestatus,
                              (errorparameter_t)ret);
		}
	}

#endif

	//ajusta o que sobrou para o inicio do proximo buffer
	if (lenaux > 0){
	   	pucbuf = (uint8_t *) (FRMWINDOW_START_ADDR + len);
		for (i=0;i<4;i++)
			u8auxbuf[i] = 0xFF;
		for (i=0;i<lenaux;i++)
			u8auxbuf[i] = *(pucbuf++);
		for (i=0;i<4;i++)
			u8winbuf[i] = u8auxbuf[i];

		flash_wrwin.dstaddrbegin = (uint32_t) flash_wrwin.dstaddrend - lenaux;
		flash_wrwin.wrpending = 2;
	}
	else
		flash_wrwin.wrpending = 0;



	/*
	if (ret) {
		// Compare source buffer and destination flash page
		if (memcmp((void *) flash_wrwin.dstaddrbegin, (void *) dstaddr, len) == 0)
			ret = TRUE;
		else
			ret = FALSE;
	}*/
    if (len == 0)
    	ret = ST_NO_FLASH_WRITE;
    else
       ret = 1;

	return ret;
}

uint8_t iswindowavailable(uint32_t offset, uint32_t len){
   uint32_t offsetestimated;
   uint8_t ret=0;

   offsetestimated = offset + len;
   if (offsetestimated <= MAX_CHUNCK_WRITE)
     ret = TRUE;

   return ret;
}

uint32_t getaddress(addr_cmd_t *paddress){
    uint32_t u32address=0;

    switch (paddress->type) {
        case iAdLen1B:
            u32address = (uint32_t) paddress->u8val;
            break;
        case iAdLen2B:
            u32address = (uint32_t) paddress->u16val;
            break;
        case iAdLen4B:
            u32address = (uint32_t) paddress->u32val;
            break;
        default:
            u32address = (uint32_t) FLASH_LENGTH;
            break;
    }

    return u32address;
}

/**

  Gets a long from a little endian buffer

  @param pucBuffer Little endian buffer
  @return Long value read

 */
uint32_t __get_unaligned32 (uint8_t *pucBuffer, uint8_t u8pos){

	uint32_t ulRet=0;
	switch (u8pos){
	case 0 :
		ulRet = (pucBuffer[0] << 0 ) |
				(pucBuffer[1] << 8 ) |
				(pucBuffer[2] << 16) |
				(pucBuffer[3] << 32);
        break;
	case 1 :
		ulRet = (pucBuffer[1] << 0 ) |
				(pucBuffer[2] << 8) |
				(pucBuffer[3] << 16);
        break;
	case 2 :
		ulRet = (pucBuffer[2] << 0 ) |
				(pucBuffer[3] << 8) ;
        break;
	case 3 :
		ulRet = (pucBuffer[3] << 0 );
        break;
	}

  return ulRet;
}

uint8_t copyflash2win (osens_frm_t *posens_frm, uint8_t *bufout,uint32_t len){

  uint32_t u32data = 0;
  uint8_t  *pucdata;
  uint32_t u32srcaddr;
  uint32_t lenaux=0,lenpos=0,length=0;
  uint32_t u32auxsrcaddr,lenrest,addrrest,u32dataaux=0;
  uint8_t ret=0,tstrff=0;

  lenpos = 0;
  u32srcaddr = getaddress(&posens_frm->addrold);
  while (lenpos < len) {
      u32auxsrcaddr = (uint32_t) (u32srcaddr + lenpos - OFFSET_FAKE);
      if (u32auxsrcaddr < FLASH_LENGTH) {
    	  addrrest = (u32auxsrcaddr % 4);

    	  //verifica se endereco esta alinhado
    	  if (addrrest > 0) {
    		  //caso negativo alinha para tras
    		  u32auxsrcaddr = u32auxsrcaddr - addrrest;
    	      u32dataaux = FlashGet (FRWOLD_START_ADDR + u32auxsrcaddr);
    	      u32data = __get_unaligned32((uint8_t *)&u32dataaux,addrrest);

    	      length = 4 - addrrest;
    	  }
    	  else {
			  u32data = FlashGet (FRWOLD_START_ADDR + u32auxsrcaddr);

			  lenaux = (len-lenpos) / 4;
			  lenrest = (len-lenpos) % 4;
			  if ((lenaux < 1)&& (lenrest > 0))
				  length = lenrest;
			  else
				  length = 4;
    	  }


    	  pucdata = (uint8_t *) &u32data;
          switch (length){
              case 1:
                  *bufout++ = *pucdata++;
                  break;
              case 2:
                  *bufout++ = *pucdata++;
                  *bufout++ = *pucdata++;
                  break;
              case 3:
                  *bufout++ = *pucdata++;
                  *bufout++ = *pucdata++;
                  *bufout++ = *pucdata++;
                  break;
              case 4:
                  *bufout++ = *pucdata++;
                  *bufout++ = *pucdata++;
                  *bufout++ = *pucdata++;
                  *bufout++ = *pucdata++;
                  break;
              default:
                  break;
          }

          lenpos += length;

#if 0
	{
		uint8_t pos=0;
		//uint32_t dstaddraux = (uint8_t *) (FRMWINDOW_START_ADDR + offset);
		uint8_t *paddr =  (uint8_t *) &u32data;

		rffbuf[pos++] = (uint8_t) 0x97;
		rffbuf[pos++] = osens_frm.frameID;
		rffbuf[pos++] = len;
		rffbuf[pos++] = (uint8_t) length;
		rffbuf[pos++] = (uint8_t) lenpos;
		rffbuf[pos++] = addrrest;
		rffbuf[pos++] = tstrff;
		//rffbuf[pos++] = *(paddr+0);
		//rffbuf[pos++] = *(paddr+1);
		//rffbuf[pos++] = *(paddr+2);
		//rffbuf[pos++] = *(paddr+3);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif


      }
      else {

	      // log the error
    	  ret = 250;
		  osens_frm.framestatus = ST_FRWUPD_GENERAL_ERROR;
		  openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_WRONG_ADDRESS,
	                            (errorparameter_t) u32auxsrcaddr,
	                            (errorparameter_t)osens_frm.framestatus);
		  return (ret);
      }
  }
  return ret;
}

//calculate the buffer len and the next
uint32_t getnewbuflen(osens_frm_t *posens_frm){

    uint32_t totaloffset=0,newlen=0;

    totaloffset = posens_frm->offset + posens_frm->len;
    if (totaloffset <= MAX_CHUNCK_WRITE){
      newlen = totaloffset;
      posens_frm->remaining = 0;
    }
    else {
       newlen = MAX_CHUNCK_WRITE;
       posens_frm->remaining = (totaloffset - MAX_CHUNCK_WRITE);
    }

    if (posens_frm->offset > 0)
  	  posens_frm->usedlen = newlen - posens_frm->offset;
    else
  	  posens_frm->usedlen = newlen;

    return (newlen);
}


/*
 * Equal Function - deve copiar o bloco determinado do endereco old para o endereco new
 * nao eh possivel ler da flash e escrever da flash ao mesmo tempo...desta forma é
 * necessario guardar o dado a ser escrito na memoria SRAM.
 */
uint8_t exefunctionEqual(osens_frm_t *posens_frm){

    uint32_t lenpos=0,newlen=0;
    uint8_t windowok;
    //static uint32_t remaining=0;
    uint8_t cmdexeend=0,ret=0;
    //uint32_t offset;
    uint8_t *bufout;
    //uint8_t countrff=0;
    uint32_t firstlen=0,maxloop=0,countloop=0;

    //protecao para nao ficar em um loop infinito
    firstlen = getchunklenbytes (&posens_frm->chuncklen);
    maxloop = (uint32_t) ((firstlen / MAX_CHUNCK_WRITE) + 5);

    while (cmdexeend == 0){
      //protecao para nao ficar em um loop infinito
      countloop++;
      if (countloop > maxloop){
    	  cmdexeend = 1;
		  osens_frm.framestatus = ST_LOOP_INFINITY;
		  openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_FCTEQUAL,
								(errorparameter_t)osens_frm.framestatus,
								(errorparameter_t)countloop);
      }

      posens_frm->offset = dstaddrupdate(posens_frm);
      bufout = (uint8_t *) (FRMWINDOW_START_ADDR + posens_frm->offset);
      posens_frm->len = getchunklenbytes (&posens_frm->chuncklen);

      windowok = iswindowavailable(posens_frm->offset, posens_frm->len);
      if (windowok){
          //copio o dado da flash para buffer da SRAM
          ret= copyflash2win (posens_frm, bufout, posens_frm->len);
          if (ret == 0) {
        	ret = writewinflash(flash_wrwin.length);
            if (ret > ST_WRFLASH_WINDOW_NOT_FULL) {
      		   //Aqui foi escrito 0 bytes por algum motivo
      		   osens_frm.framestatus = ret;
      		   openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_FCTEQUAL,
      								(errorparameter_t)osens_frm.framestatus,
      								(errorparameter_t)1);
            }
          }
          cmdexeend = 1;


			#if 1
			{
        	  //92 00 04 08 08 04 04
				uint8_t pos=0;

				rffbuf[pos++] = (uint8_t) 0x92;
				rffbuf[pos++] = osens_frm.frameID;
				rffbuf[pos++] = (uint8_t) flash_wrwin.length;
				rffbuf[pos++] = (uint8_t) flash_wrwin.dstaddrbegin;
				rffbuf[pos++] = (uint8_t) posens_frm->addrnew.u32val;
				rffbuf[pos++] = (uint8_t) posens_frm->chuncklen.u32val;
				rffbuf[pos++] = (uint8_t) posens_frm->len;

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
			#endif

      }
      else {
          /*aqui a janela de memoria acabou ou esta acabando primeiramente deve ser preenchida a memoria no maxixo
            entao deve ser escrito na flash por ultimo deve continuar lendo a escrevendo a memoria ate terminar o comando */
          newlen = getnewbuflen(posens_frm);

          //fill the window memory with the bytes remaining
          ret = copyflash2win (posens_frm, bufout, newlen);
		  if (ret == 0) {
			ret = writewinflash(newlen);
			if (ret > ST_WRFLASH_WINDOW_NOT_FULL) {
			   osens_frm.framestatus = ret;
			   openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_FCTEQUAL,
									(errorparameter_t)osens_frm.framestatus,
									(errorparameter_t)2);
			   cmdexeend = 1;
			}
		  }

#if 1
{
  //92 00 04 08 08 04 04
	uint8_t pos=0;

	rffbuf[pos++] = (uint8_t) 0x93;
	rffbuf[pos++] = osens_frm.frameID;
	rffbuf[pos++] = (uint8_t) flash_wrwin.length;
	rffbuf[pos++] = (uint8_t) flash_wrwin.dstaddrbegin;
	rffbuf[pos++] = (uint8_t) posens_frm->addrnew.u32val;
	rffbuf[pos++] = (uint8_t) posens_frm->chuncklen.u32val;
	rffbuf[pos++] = (uint8_t) posens_frm->len;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

		  // reposition the buffer hands - mudo o type pois o endereco pode ser maior.
		  posens_frm->addrold.type = iAdLen4B;
		  posens_frm->addrold.u32val += posens_frm->usedlen;
		  posens_frm->addrnew.type = iAdLen4B;
		  posens_frm->addrnew.u32val += posens_frm->usedlen;
		  posens_frm->chuncklen.type = iAdLen4B;
		  posens_frm->chuncklen.u32val = posens_frm->remaining;

		}
    }

    return (ret);
}

/*
 * Funcao Replace deve copiar o bloco do frame no endereco new
 */
uint8_t exefunctionReplace(osens_frm_t *posens_frm, uint8_t *bufin) {

    uint32_t newlen=0;
    uint8_t windowok;
    uint8_t cmdexeend=0,ret=0;
    uint8_t *bufout;
    uint8_t *pu8AuxBufIn;
    uint32_t firstlen=0,maxloop=0,countloop=0;

    //aqui eu uso o addrold somente para guardar a ultima posicao do buffer recebido
    posens_frm->addrold.type = iAdLen4B;
    posens_frm->addrnew.type = iAdLen4B;
    posens_frm->addrold.u32val = 0;

    //protecao para nao ficar em um loop infinito
    firstlen = getchunklenbytes (&posens_frm->chuncklen);
    maxloop = (uint32_t) ((firstlen / MAX_CHUNCK_WRITE) + 5);

    while (cmdexeend == 0){
		//protecao para nao ficar em um loop infinito
		countloop++;
		if (countloop> maxloop){
			cmdexeend = 1;
			osens_frm.framestatus = ST_LOOP_INFINITY;
			openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_FCTREPLACE,
									(errorparameter_t)osens_frm.framestatus,
									(errorparameter_t)countloop);
		}

		posens_frm->offset = dstaddrupdate(posens_frm);
		bufout = (uint8_t *) (FRMWINDOW_START_ADDR + posens_frm->offset);
		posens_frm->len = getchunklenbytes (&posens_frm->chuncklen);

		windowok = iswindowavailable(posens_frm->offset, posens_frm->len);
		if (windowok){
			//copio o dado da flash para buffer da SRAM
			pu8AuxBufIn = (uint8_t *) (bufin + posens_frm->addrold.u32val);
			memcpy((void *) bufout , (void *) pu8AuxBufIn, posens_frm->len);

			#if 1
			{
				uint8_t pos=0;

				rffbuf[pos++] = (uint8_t) 0x95;
				rffbuf[pos++] = osens_frm.frameID;
				rffbuf[pos++] = (uint8_t) posens_frm->offset;
				rffbuf[pos++] = (uint8_t) posens_frm->len;
				rffbuf[pos++] = (uint8_t) posens_frm->usedlen;
				rffbuf[pos++] = (uint8_t) flash_wrwin.length;
				rffbuf[pos++] = (uint8_t) newlen;

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
			#endif

			ret = writewinflash(flash_wrwin.length);
			if (ret > ST_WRFLASH_WINDOW_NOT_FULL) {
			   osens_frm.framestatus = ret;
			   openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_FCTREPLACE,
									(errorparameter_t)osens_frm.framestatus,
									(errorparameter_t)3);
			}
            cmdexeend = 1;
		}
		else {
			//calculate the buffer len and the next
	        newlen = getnewbuflen(posens_frm);

			//fill the window memory with the bytes remaining
			pu8AuxBufIn = (uint8_t *) (bufin + posens_frm->addrold.u32val);
			memcpy((void *) bufout , (void *) pu8AuxBufIn, posens_frm->usedlen);

			ret = writewinflash(newlen);
			if (ret > ST_WRFLASH_WINDOW_NOT_FULL) {
			   osens_frm.framestatus = ret;
			   openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_FCTREPLACE,
									(errorparameter_t)osens_frm.framestatus,
									(errorparameter_t)4);
				cmdexeend = 1;
			}

			#if 1
			{
				uint8_t pos=0;

				rffbuf[pos++] = (uint8_t) 0x96;
				rffbuf[pos++] = osens_frm.frameID;
				rffbuf[pos++] = (uint8_t) firstlen;
				rffbuf[pos++] = (uint8_t) posens_frm->addrnew.u32val;
				rffbuf[pos++] = (uint8_t) posens_frm->chuncklen.u32val;
				rffbuf[pos++] = (uint8_t) posens_frm->offset;
				rffbuf[pos++] = (uint8_t) posens_frm->len;
				rffbuf[pos++] = (uint8_t) posens_frm->usedlen;
				rffbuf[pos++] = (uint8_t) posens_frm->remaining;
				rffbuf[pos++] = (uint8_t) newlen;
				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
			#endif

			// reposition the buffer hands - mudo o type pois o endereco pode ser maior.
			posens_frm->addrold.type = iAdLen4B;
			posens_frm->addrold.u32val += posens_frm->usedlen;
			posens_frm->addrnew.type = iAdLen4B;
			posens_frm->addrnew.u32val += posens_frm->usedlen;
			posens_frm->chuncklen.type = iAdLen4B;
			posens_frm->chuncklen.u32val = posens_frm->remaining;
		}
    }

    return ret;
}
/*
 * Funcao Insert deve copiar o bloco do frame no endereco new e deslocar o restante do codigo
 */
uint8_t exefunctionInsert(osens_frm_t *posens_frm, uint8_t *bufin) {

    uint32_t len=0,lenaux=0,lenpos=0,newlen=0;
    uint8_t windowok;
    static uint32_t remaining=0;
    uint8_t cmdexeend=0,ret=0;
    uint32_t offset;
    uint8_t *bufout;
    uint32_t totaloffset,usedlen;
    uint8_t *pu8AuxBufIn;
    uint32_t firstlen=0,maxloop=0,countloop=0;

    //aqui eu uso o addrold somente para guardar a ultima posicao do buffer recebido
    posens_frm->addrold.type = iAdLen4B;
    posens_frm->addrnew.type = iAdLen4B;
    posens_frm->addrold.u32val = 0;

    //protecao para nao ficar em um loop infinito
    firstlen = getchunklenbytes (&posens_frm->chuncklen);
    maxloop = (uint32_t) ((firstlen / MAX_CHUNCK_WRITE) + 5);

    while (cmdexeend == 0){
		//protecao para nao ficar em um loop infinito
		countloop++;
		if (countloop> maxloop){
			cmdexeend = 1;
			osens_frm.framestatus = ST_LOOP_INFINITY;
			openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_FCTINSERT,
									(errorparameter_t)osens_frm.framestatus,
									(errorparameter_t)countloop);
		}
		offset = dstaddrupdate(posens_frm);
		bufout = (uint8_t *) (FRMWINDOW_START_ADDR + offset);
		len = getchunklenbytes (&posens_frm->chuncklen);

		windowok = iswindowavailable(offset, len);
		if (windowok){
			//copio o dado da flash para buffer da SRAM
			pu8AuxBufIn = (uint8_t *) (bufin + posens_frm->addrold.u32val);
			memcpy((void *) bufout , (void *) pu8AuxBufIn, len);

			#if 1
			{
				uint8_t pos=0;
				uint32_t dstaddraux = (uint8_t *) (FRMWINDOW_START_ADDR + offset);
				uint8_t *paddr =  (uint8_t *) dstaddraux;

				rffbuf[pos++] = (uint8_t) 0x95;
				rffbuf[pos++] = osens_frm.frameID;
				rffbuf[pos++] = offset;
				rffbuf[pos++] = len;
				rffbuf[pos++] = *(paddr+0);
				rffbuf[pos++] = *(paddr+1);
				rffbuf[pos++] = *(paddr+2);
				rffbuf[pos++] = *(paddr+3);

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
			#endif

			ret = writewinflash(flash_wrwin.length);
			if (ret > ST_WRFLASH_WINDOW_NOT_FULL) {
			   osens_frm.framestatus = ret;
			   openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_FCTINSERT,
									(errorparameter_t)osens_frm.framestatus,
									(errorparameter_t)5);
			}
            cmdexeend = 1;

		}
		else {
			//calculate the buffer len and the next
			totaloffset = offset + len;
			if (totaloffset <= MAX_CHUNCK_WRITE){
			  newlen = totaloffset;
			  remaining = 0;
			}
			else {
			   newlen = MAX_CHUNCK_WRITE;
			   remaining = (totaloffset - MAX_CHUNCK_WRITE);
			}

			if (offset > 0)
			  usedlen = newlen - offset;
			else
			  usedlen = newlen;

			//fill the window memory with the bytes remaining
			pu8AuxBufIn = (uint8_t *) (bufin + posens_frm->addrold.u32val);
			memcpy((void *) bufout , (void *) pu8AuxBufIn, usedlen);

			ret = writewinflash(newlen);
			if (ret > ST_WRFLASH_WINDOW_NOT_FULL) {
			   osens_frm.framestatus = ret;
			   openserial_printError(COMPONENT_CFRMDWN,ERR_FRWUPD_FCTINSERT,
									(errorparameter_t)osens_frm.framestatus,
									(errorparameter_t)6);
			}

			#if 1
			{
				uint8_t pos=0;

				rffbuf[pos++] = (uint8_t) 0x96;
				rffbuf[pos++] = osens_frm.frameID;
				rffbuf[pos++] = (uint8_t) offset;
				rffbuf[pos++] = (uint8_t) len;
				rffbuf[pos++] = (uint8_t) usedlen;
				rffbuf[pos++] = (uint8_t) remaining;
				rffbuf[pos++] = (uint8_t) newlen;
				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
			#endif

			// reposition the buffer hands - mudo o type pois o endereco pode ser maior.
			posens_frm->addrold.type = iAdLen4B;
			posens_frm->addrold.u32val += usedlen;
			posens_frm->addrnew.type = iAdLen4B;
			posens_frm->addrnew.u32val += usedlen;
			posens_frm->chuncklen.type = iAdLen4B;
			posens_frm->chuncklen.u32val = remaining;

		}
	  }

	return ret;
}

/* esta funcao le o buffer da mensagem recebida e armazena em um buffer interno em memoria SRAM
 * para ser posteriormente escrito na flash.
 */
uint8_t readchunck(addr_cmd_t *plen, uint8_t *pbufin, uint8_t *pbufout){

	uint32_t i;
    uint32_t len=0;
    uint8_t ret=FALSE;

    len = getchunklenbytes(plen);
	if (len < MAX_FRAME_CHUNCK){
		for (i=0;i<len;i++){
			*pbufout++ = *pbufin++;
		}
		ret = len;
	}

   osens_frm.framepos += len;

   return ret;
}

uint32_t dstaddrupdate (osens_frm_t *posens_frm){
	uint32_t offset;
	uint8_t *pu8bufout;
	uint32_t addrnewval;
	uint32_t chucklen;
	uint8_t wrpendingrff;

	addrnewval = (uint32_t) getaddress(&posens_frm->addrnew);
	wrpendingrff = flash_wrwin.wrpending;

	if (flash_wrwin.wrpending == 0) {
	 flash_wrwin.dstaddrbegin = (uint32_t) addrnewval;
	 flash_wrwin.wrpending = 1;
	}

	offset = (addrnewval - flash_wrwin.dstaddrbegin );

	chucklen = getaddress(&posens_frm->chuncklen);
	flash_wrwin.dstaddrend = addrnewval + chucklen;

	flash_wrwin.length = (flash_wrwin.dstaddrend - flash_wrwin.dstaddrbegin);

	#if 0
	{
		uint8_t pos=0;
		uint8_t *pucaux = (uint8_t *) &osens_frm.chuncklen.u32val;
	//	DBG_MOTE > [19:25:28.761000] 90 03 f8 fc 00 08 00
	//	DBG_MOTE > [19:25:28.814000] 95 03 f8 fc 04 08 0c
	//	DBG_MOTE > [19:25:28.834000] 95 03 f8 00 08 04 0c
	//	DBG_MOTE > [19:25:28.850000] 95 03 f8 00 08 04 0c

		rffbuf[pos++]= 0x95;
		rffbuf[pos++]= (uint8_t) osens_frm.frameID;
		rffbuf[pos++]= (uint8_t) wrpendingrff;
		rffbuf[pos++]= (uint8_t) flash_wrwin.dstaddrbegin;
		rffbuf[pos++]= (uint8_t) addrnewval;
		rffbuf[pos++]= (uint8_t) offset;
		rffbuf[pos++]= (uint8_t) chucklen;
		rffbuf[pos++]= (uint8_t) flash_wrwin.length;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif

	return offset;
}

uint8_t frmdwn_frame_parser(uint8_t *pbuf, uint8_t count){

	uint8_t *pframe = pbuf;
	uint8_t opt=0;
	uint8_t ast=0;
	uint8_t lst=0;
	uint8_t ret=255;
	uint8_t auxpos;
	uint8_t *pu8bufout;
	uint8_t u8auxbuf[20];
    uint8_t auxchunklen=0;
    uint8_t auxret=0;
    uint8_t *pu8frame;
	uint8_t *pu8outbuf;
	uint8_t u8aux;

     //get the address (depends on the OPcode)
	 getAdLen(&osens_frm);
	 opt = OP_SHIFT(osens_frm.header);
	 //leds_radio_toggle();

	 switch (opt) {
		 case iInfo :
			 //read firmware Old
			 frm_fillinfo(iAdLen4B, (uint32_t *)&frminfo.frmware_vOld, &pframe[osens_frm.framepos]);
			 //read firmware New
			 frm_fillinfo(iAdLen4B, (uint32_t *)&frminfo.frmware_vNew, &pframe[osens_frm.framepos]);
			 //read firmware nrIds
			 frm_fillinfo(iAdLen4B, (uint32_t *)&frminfo.frmware_nrIds, &pframe[osens_frm.framepos]);
			 //read firmware fullSize
			 frm_fillinfo(iAdLen4B, (uint32_t *)&frminfo.frmware_fullSize, &pframe[osens_frm.framepos]);
			 //read firmware CRC
			 frm_fillinfo(iAdLen2B, (uint16_t *)&frminfo.frmware_CRC, &pframe[osens_frm.framepos]);
			 ret = 1;
			 break;
		 case iEqual :
			 ast = AS_SHIFT(osens_frm.header);
			 lst = LS_SHIFT(osens_frm.header);
			 //read AS old
			 osens_frm.addrold.type = (uint32_t) ast;
			 frm_fillinfo(ast, (uint32_t *)&osens_frm.addrold.u32val, &pframe[osens_frm.framepos]);
			 //read AS new
			 osens_frm.addrnew.type = (uint32_t) ast;
			 frm_fillinfo(ast, (uint32_t *)&osens_frm.addrnew.u32val, &pframe[osens_frm.framepos]);
			 //read LS
			 osens_frm.chuncklen.type = (uint32_t) lst;
			 auxret = frm_fillinfo(lst, (uint32_t *) &osens_frm.chuncklen.u32val,&pframe[osens_frm.framepos] );

			 exefunctionEqual(&osens_frm);

             ret = 0;
			 break;
		 case iReplace :
			 ast = AS_SHIFT(osens_frm.header);
			 lst = LS_SHIFT(osens_frm.header);
			 //read AS new
			 osens_frm.addrold.type = 0;
			 osens_frm.addrold.u8val = 0;
			 osens_frm.addrnew.type = ast;
			 frm_fillinfo(ast, (uint32_t *)&osens_frm.addrnew.u32val, &pframe[osens_frm.framepos]);
			 //read LS
			 osens_frm.chuncklen.type = lst;
			 frm_fillinfo(lst, (uint32_t *)&osens_frm.chuncklen.u32val, &pframe[osens_frm.framepos]);
			 //read chunck
			 readchunck(&osens_frm.chuncklen,&pframe[osens_frm.framepos],u8auxbuf);

			#if 0
			{
				uint8_t pos=0;

				rffbuf[pos++]= 0x95;
				rffbuf[pos++]= (uint8_t) osens_frm.frameID;
				rffbuf[pos++]= (uint8_t) osens_frm.header;
				rffbuf[pos++]= (uint8_t) osens_frm.addrold.u8val;
				rffbuf[pos++]= (uint8_t) osens_frm.addrnew.u8val;
				rffbuf[pos++]= (uint8_t) osens_frm.chuncklen.u32val;

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
			#endif

			 exefunctionReplace(&osens_frm, u8auxbuf);
             ret = 0;
			 break;
		 case iInsert :
			 ast = AS_SHIFT(osens_frm.header);
			 lst = LS_SHIFT(osens_frm.header);
			 //read AS new
			 frm_fillinfo(ast, (uint32_t *)&osens_frm.addrnew.u32val, &pframe[osens_frm.framepos]);
			 //read LS
			 frm_fillinfo(lst, (uint32_t *)&osens_frm.chuncklen.u32val, &pframe[osens_frm.framepos]);
			 //read chunck
			 readchunck(&osens_frm.chuncklen,&pframe[osens_frm.framepos],u8auxbuf);

			 exefunctionInsert(&osens_frm,u8auxbuf);
			 ret = 0;
			 break;
		 default:
			 ret = 255;
			 break;
	 }

#if 1
{
	uint8_t pos=0;
	uint8_t *pucaux = (uint8_t *) &osens_frm.frameID;
	uint8_t *pucaux1 = (uint8_t *) &osens_frm.addrnew.u32val;
	uint8_t *pucaux2 = (uint8_t *) &osens_frm.chuncklen.u32val;

	rffbuf[pos++]= 0x98;
	rffbuf[pos++]= (uint8_t) *(pucaux+1);
	rffbuf[pos++]= (uint8_t) *(pucaux+0);
	rffbuf[pos++]= (uint8_t) osens_frm.header;
	rffbuf[pos++]= (uint8_t) *(pucaux1+3);
	rffbuf[pos++]= (uint8_t) *(pucaux1+2);
	rffbuf[pos++]= (uint8_t) *(pucaux1+1);
	rffbuf[pos++]= (uint8_t) *(pucaux1+0);
	rffbuf[pos++]= (uint8_t) *(pucaux2+3);
	rffbuf[pos++]= (uint8_t) *(pucaux2+2);
	rffbuf[pos++]= (uint8_t) *(pucaux2+1);
	rffbuf[pos++]= (uint8_t) *(pucaux2+0);
	rffbuf[pos++]= (uint8_t) ret;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

	  if (ret > 1) {
	      // log the error
		  osens_frm.framestatus = ST_FRWUPD_GENERAL_ERROR;
		  openserial_printError(COMPONENT_CFRMDWN,ERR_WRONG_OPTION,
	                            (errorparameter_t)ret,
	                            (errorparameter_t)osens_frm.framestatus);
	  }

   return ret;
}

uint8_t getAdLen(osens_frm_t *pucAddr){

	uint8_t u8AS = AS(pucAddr->header);
	uint8_t u8LS = LS(pucAddr->header);
	uint8_t ret=0;

	switch (u8AS) {
		case iAdLen1B:
			pucAddr->addrnew.type = iAdLen1B;
			pucAddr->addrold.type = iAdLen1B;
			ret=1;
			break;
		case iAdLen2B:
			pucAddr->addrnew.type = iAdLen2B;
			pucAddr->addrold.type = iAdLen2B;
			ret=1;
			break;
		case iAdLen4B:
			pucAddr->addrnew.type = iAdLen4B;
			pucAddr->addrold.type = iAdLen4B;
			ret=1;
			break;
	}

	switch (u8LS) {
		case iAdLen1B:
			pucAddr->chuncklen.type = iAdLen1B;
			ret=2;
			break;
		case iAdLen2B:
			pucAddr->chuncklen.type = iAdLen2B;
			ret=2;
			break;
		case iAdLen4B:
			pucAddr->chuncklen.type = iAdLen4B;
			ret=2;
			break;
	}
	return ret;
}

/*
uint8_t *setASLSVal(uint8_t lentype, uint8_t *paddress, uint8_t *pbuf){

 uint8_t *pu8frame = (uint8_t *) pbuf;
 uint8_t *pu8val;

	switch (lentype) {
		case iAdLen1B:{
			paddress->u8val = *pu8frame++;
		     osens_frm.framepos+=1;
		     break;
		}
		case iAdLen2B: {
			pu8val = (uint8_t *) &paddress->u16val;
			*(pu8val+1) = (uint8_t) *pu8frame++;
			*(pu8val+0) = *pu8frame++;
		     osens_frm.framepos+=2;
			break;
		}
		case iAdLen4B: {
			pu8val = (uint8_t *) &paddress->u32val;
			*(pu8val+3) = *pu8frame++;
			*(pu8val+2) = *pu8frame++;
			*(pu8val+1) = *pu8frame++;
			*(pu8val+0) = *pu8frame++;
		     osens_frm.framepos+=4;
			break;
		}
		default:
			break;
	}

    return (uint8_t *) pu8frame;

}
*/

uint8_t frm_fillinfo(uint8_t lentype, uint32_t *paddress, uint8_t *pbuf){

	uint8_t *pu8frame = (uint8_t *) pbuf;
	uint8_t *pu8outbuf = (uint8_t *) paddress;
	//uint8_t *pu8aux = (uint8_t *) &value;
    uint8_t ret=0;

	switch (lentype) {
		case iAdLen1B:{
			*pu8outbuf = *pu8frame++;
		     osens_frm.framepos++;
			ret = 1;
			break;
		}
		case iAdLen2B: {
			*(pu8outbuf+1) = *pu8frame++;
			*pu8outbuf = *pu8frame++;
		     osens_frm.framepos+=2;
		     ret = 2;
			break;
		}
		case iAdLen4B: {
			*(pu8outbuf+3) = *pu8frame++;
			*(pu8outbuf+2) = *pu8frame++;
			*(pu8outbuf+1) = *pu8frame++;
			*(pu8outbuf+0) = *pu8frame++;
		     osens_frm.framepos+=4;
		     ret = 4;
			break;
		}
		default:
			break;
	}

    return ret;

}

uint32_t getchunklenbytes(addr_cmd_t *plength) {
    uint32_t len=0;

	switch (plength->type) {
	case iAdLen1B:
		len = (uint32_t) plength->u8val;
		break;
	case iAdLen2B:
		len = (uint32_t) plength->u16val;       //len = (uint32_t) plength->u16val * 2;
		break;
	case iAdLen4B:
		len = (uint32_t) plength->u32val;      //len = (uint32_t) plength->u32val * 4;
		break;
	default:
		len = 0;
		break;
	}
	return len;
}

/*
uint8_t osens_get_brd_desc(osens_brd_id_t *brd)
{
    return 0;
}
uint8_t osens_get_num_points(void)
{
    return 0;
}

uint8_t osens_get_pdesc(uint8_t index, osens_point_desc_t *desc)
{
    return 0;
}

int8_t osens_get_ptype( uint8_t index)
{
    return 0;
}

uint8_t osens_get_point(uint8_t index, osens_point_t *point)
{
    return 0;
}

uint8_t osens_set_pvalue(uint8_t index, osens_point_t *point)
{
    return 0;
}

*/
uint8_t printf1(){
	uint8_t pos=0;
    uint8_t *pucbuf = (uint8_t *) &osens_frm.addrnew.u16val;
    uint8_t *pucbuf1 = (uint8_t *) &osens_frm.chuncklen.u16val;
    uint8_t *pucbuf2 = (uint8_t *) &flash_wrwin.dstaddrbegin;

	rffbuf[pos++]= (uint8_t) 0x90;
	rffbuf[pos++]= (uint8_t) osens_frm.frameID;
	rffbuf[pos++] =(uint8_t) *(pucbuf+1);
	rffbuf[pos++] =(uint8_t) *(pucbuf+0);
	rffbuf[pos++] =(uint8_t) *(pucbuf1+1);
	rffbuf[pos++] =(uint8_t) *(pucbuf1+0);
	rffbuf[pos++] =(uint8_t) *(pucbuf2+1);
	rffbuf[pos++] =(uint8_t) *(pucbuf2+0);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

	return 0;
}

uint8_t runtestequal1 (uint8_t count) {
	 uint8_t ret=0;
	 uint32_t i=0;
     uint8_t *pu8bufout;
     uint32_t len=0;
     uint8_t retlen=0;
     uint32_t offset=0;
     uint8_t u8auxbuf[300];
     uint16_t iniaddr = 0x0000;

     osens_frm.addrold.type   = iAdLen2B;
	 osens_frm.addrnew.type   = iAdLen2B;
	 osens_frm.chuncklen.type = iAdLen2B;
	 osens_frm.frameID = count;

     if (count == 0){
 		flash_wrwin.dstaddrbegin  = iniaddr;
 		flash_wrwin.wrpending     = 0;

 		osens_frm.addrold.u16val   = iniaddr;
    	osens_frm.addrnew.u16val   = iniaddr;
    	osens_frm.chuncklen.u16val = 4;
		lastpos = iniaddr + osens_frm.chuncklen.u16val;

        printf1();

		ret = exefunctionEqual(&osens_frm);
     }
     else if (count == 2){
    	osens_frm.addrold.u16val = 0;
    	osens_frm.addrnew.u16val = lastpos;
    	osens_frm.chuncklen.u16val = 4;
		lastpos = lastpos + osens_frm.chuncklen.u16val;

		u8auxbuf[0]= 0x00;
		u8auxbuf[1]= 0x01;
		u8auxbuf[2]= 0x02;
		u8auxbuf[3]= 0x03;
		u8auxbuf[4]= 0x04;
		u8auxbuf[5]= 0x05;

        printf1();

		exefunctionReplace(&osens_frm, u8auxbuf);

     }
     else if (count == 4){
  		osens_frm.addrold.u16val = 0;
     	osens_frm.addrnew.u16val = lastpos;
     	osens_frm.chuncklen.u16val = 2;
		lastpos = lastpos + osens_frm.chuncklen.u16val;
/*
		for (i=0;i<300;i++){
			u8auxbuf[i]= 0xAA;
		}
*/
        printf1();

        exefunctionEqual(&osens_frm);
      }
     else if (count == 5){
 		osens_frm.addrold.u16val   = lastpos;
    	osens_frm.addrnew.u16val   = lastpos;
    	osens_frm.chuncklen.u16val = 4;
		lastpos = lastpos + osens_frm.chuncklen.u16val;

        printf1();

		ret = exefunctionEqual(&osens_frm);
     }


	 return retlen;
}

