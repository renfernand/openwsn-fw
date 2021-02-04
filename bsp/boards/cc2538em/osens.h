/**
@file osens.h
@brief Open sensors driver interface

@author Marcelo Barros de Almeida <marcelobarrosalmeida@gmail.com>
*/

#ifndef __OSENS_H__
#define __OSENS_H__

#define OSENS_LATEST_VERSION     0
#define OSENS_MODEL_NAME_SIZE    8
#define OSENS_MANUF_NAME_SIZE    8
#define OSENS_POINT_NAME_SIZE    8
#define OSENS_MAX_POINTS        32


#define FLASH_BASE               0x00200000  // Flash
#define PAGE_SIZE                2048
#define PAGE_TST_NEW             50      // 52 = 0x0021A000
#define PAGE_TST_OLD             0       // 50 = 0x00219000
#define MAX_PAGES                20

#define FRWNEW_START_ADDR (FLASH_BASE + (PAGE_TST_NEW * PAGE_SIZE))
#define FRWOLD_START_ADDR (FLASH_BASE + (PAGE_TST_OLD * PAGE_SIZE))
#define FLASH_LENGTH     PAGE_SIZE*MAX_PAGES

#define OSENS_SM_TICK_MS 250

enum osens_access_rights_e
{
	OSENS_ACCESS_READ_ONLY = 0x01,
	OSENS_ACCESS_WRITE_ONLY = 0x02,
	OSENS_ACCESS_READ_WRITE = 0x03,
};

enum osens_sensor_capabilities_e
{
	OSENS_CAPABILITIES_BATTERY = 0x01,
	OSENS_CAPABILITIES_DISPLAY = 0x02,
	OSENS_CAPABILITIES_WPAN_STATUS = 0x04,
	OSENS_CAPABILITIES_BATTERY_STATUS = 0x08,
};

/** Sensor interface standard datatypes */
enum osens_datatypes_e
{
	OSENS_DT_U8     = 0x00, /**< 8 bits unsigned */
	OSENS_DT_S8     = 0x01, /**< 8 bits signed */
	OSENS_DT_U16    = 0x02, /**< 16 bits unsigned */
	OSENS_DT_S16    = 0x03, /**< 16 bits signed */
	OSENS_DT_U32    = 0x04, /**< 32 bits unsigned */
	OSENS_DT_S32    = 0x05, /**< 32 bits signed */
	OSENS_DT_U64    = 0x06, /**< 64 bits unsigned */
	OSENS_DT_S64    = 0x07, /**< 64 bits signed */
	OSENS_DT_FLOAT  = 0x08, /**< IEEE 754 single precision */
	OSENS_DT_DOUBLE = 0x09, /**< IEEE 754 double precision */
};

union osens_point_data_u
{
	uint8_t  u8;
	int8_t   s8;
	uint16_t u16;
	int16_t  s16;
	uint32_t u32;
	int32_t  s32;
	uint64_t u64;
	int64_t  s64;
	float    fp32;
	double   fp64;
};

typedef struct osens_brd_id_s
{
	uint8_t model[OSENS_MODEL_NAME_SIZE];
	uint8_t manufactor[OSENS_MANUF_NAME_SIZE];
	uint32_t sensor_id;
	uint8_t hardware_revision;
	uint8_t num_of_points;
	uint8_t capabilities;
} osens_brd_id_t;


typedef struct osens_point_desc_s
{
	uint8_t name[OSENS_POINT_NAME_SIZE];
	uint8_t type;
	uint8_t unit;
	uint8_t access_rights;
	uint32_t sampling_time_x250ms;
} osens_point_desc_t;

typedef struct osens_point_s
{
	union osens_point_data_u value;
    uint8_t type;
} osens_point_t;

//BEGIN_PACK
typedef struct addr_cmd_t_s
{                                 // always written big endian, i.e. MSB in addr[0]
   uint32_t type;
   union {
      uint8_t  u8val;            //address type 0
      uint16_t u16val;           //address type 1
      uint32_t u32val;           //address type 2
   };
} addr_cmd_t;
//END_PACK;
typedef struct addr_cmd_s
{                                 // always written big endian, i.e. MSB in addr[0]
   uint32_t type;
      uint32_t u32val;           //address type 2
} addr_cmd;

typedef struct
{
	uint16_t   frameID;
	uint8_t    frameLen;
	uint8_t    framestatus;
	addr_cmd_t chuncklen;
	uint8_t    flashnewcmd;
    uint8_t    framepos;
	uint8_t    header;
	uint32_t   offset;
	uint32_t   len;
	uint32_t   remaining;
	uint32_t   usedlen;
	uint32_t   frwnewEndAddr;
	addr_cmd_t addrold;
	addr_cmd_t addrnew;
	uint8_t    payload[64];
    uint8_t    chunck[48];
} osens_frm_t;

typedef struct
{
	uint32_t frmware_vOld;
	uint32_t frmware_vNew;
	uint32_t frmware_nrIds;
	uint32_t frmware_fullSize;
	uint16_t frmware_CRC;
} frm_info_t;

typedef struct
{
	uint32_t    dstaddrbegin;   /* ponteiro para inicio da janela novo firmware */
	uint32_t    dstaddrend;     /* endereco do fim da janela novo firmware */
	uint8_t    wrpending;      /* sinaliza se existe bytes pendentes a serem escritos na flash */
	uint32_t    length;         /* tamanho da janela a ser escrita na flash */
	uint8_t    status;          /* codigo de erro da escrita na flash */
} flash_wrwin_t;



/* Mask definition for each field of status atribute of input parameter - FF IRDS */
#define MC_HEAD             (uint8_t)0x80
#define AS_HEAD             (uint8_t)0x60
#define LS_HEAD             (uint8_t)0x18
#define OP_HEAD             (uint8_t)0x07

/* Number of shift required for each bit field - FF IRDS */
#define MC_NS               7
#define AS_NS               5
#define LS_NS               3
#define OP_NS               0


#define MC_SHIFT(i)         (uint8_t)(((i) & MC_HEAD) >> MC_NS)
#define AS_SHIFT(i)         (uint8_t)(((i) & AS_HEAD) >> AS_NS)
#define LS_SHIFT(i)         (uint8_t)(((i) & LS_HEAD) >> LS_NS)
#define OP_SHIFT(i)         (uint8_t)(((i) & OP_HEAD) >> OP_NS)

#define MC(i)               (uint8_t)(((i) << MC_NS) & MC_HEAD)
#define AS(i)               (uint8_t)(((i) << AS_NS) & AS_HEAD)
#define LS(i)               (uint8_t)(((i) << LS_NS) & LS_HEAD)
#define OP(i)               (uint8_t)(((i) << OP_NS) & OP_HEAD)


//enderecos SONOMA
#define SONOMA_ADDR_CMD        0x00
#define SONOMA_ADDR_FWDATE     0x01
#define SONOMA_ADDR_SAMPLES    0x08     //DEFAULT 400
#define SONOMA_ADDR_S0_GAIN    0x0D
#define SONOMA_ADDR_S1_GAIN    0x0C
#define SONOMA_ADDR_S2_GAIN    0x0F
#define SONOMA_ADDR_S3_GAIN    0x0E
#define SONOMA_ADDR_S0_OFFSET  0x10
#define SONOMA_ADDR_S2_OFFSET  0x11
#define SONOMA_ADDR_S1_OFFSET  0x12
#define SONOMA_ADDR_S3_OFFSET  0x13
#define SONOMA_ADDR_T_GAIN     0x14
#define SONOMA_ADDR_T_OFFSET   0x15
#define SONOMA_ADDR_VA_RMS	   0x2B	// [S.23] RMS Voltage for VA source
#define SONOMA_ADDR_IA_RMS	   0x3E	// [S.23] RMS Current for IA source
#define SONOMA_ADDR_WATT_A	   0x4B	// [S.23] Active Power for VA source
#define SONOMA_ADDR_UART_BAUD  0xB1  //UART Baudrate 9600

// Ajuste para os valores recebidos pela SONOMA
#define SONOMA_TRIMER_VA_RMS	   0.0015	// [S.23]
#define SONOMA_TRIMER_IA_RMS	   0.0015	// [S.23] (NÃO CONFERIDO)
#define SONOMA_TRIMER_WATT_A	   0.0015	// [S.23] (NÃO CONFERIDO)

uint32_t CalculaInt(uint8_t rx[]);
float CalculaS23(uint8_t *rx);
float CalculaS21(uint8_t *rx);

enum EOPCode {iEqual=0, iReplace, iInsert, iDelete, iInfo, iCommit, iRetransmit, ifirsttime};
enum ECommand {iAdLen1B=0, iAdLen2B, iAdLen4B};
enum EFlashNewCmd {iFlashNone=0, iFlashErase, iFlashChunck, iFlashFrmEnd, iFlashFrmInit};

/* sub status from response */
enum status_frwupd_u {
   ST_FRWUPD_OK                        = 0,
   ST_FRWUPD_ERROR_ERASE_FLASH         = 1,
   ST_FRWUPD_GENERAL_ERROR             = 2,
   ST_FRWUPD_CODE3                     = 3,
   ST_WRITE_FLASH                      = 4, //
   ST_LOOP_INFINITY                    = 6, // received an echo request
   ST_WRFLASH_WINDOW_NOT_FULL          = 240,
   ST_WRFLASH_TOO_BIG                  = 241,
   ST_WRFLASH_DST_ADDR_NOT_ALIGNED     = 242,
   ST_NO_FLASH_WRITE                   = 243,
};

uint8_t osens_init(void);
uint8_t frmdwn_init(void);
uint8_t osens_get_brd_desc(osens_brd_id_t *brd);
uint8_t osens_get_num_points(void);
uint8_t osens_get_point(uint8_t index, osens_point_t *point);
uint8_t osens_get_pdesc(uint8_t index, osens_point_desc_t *desc);
int8_t osens_get_ptype(uint8_t index);
uint8_t osens_set_pvalue(uint8_t index, osens_point_t *point);
uint8_t osens_liga_lampada_local(void);
extern int32_t FlashMainPageErase(uint32_t ui32Address);
uint8_t exefunctionInsert(osens_frm_t *posens_frm, uint8_t *bufin);
uint8_t exefunctionReplace(osens_frm_t *posens_frm, uint8_t *bufin);
uint8_t exefunctionEqual(osens_frm_t *posens_frm);
uint8_t writewinflash(uint32_t length);
uint32_t dstaddrupdate (osens_frm_t *posens_frm);
#endif /* __OSENS_H__ */

