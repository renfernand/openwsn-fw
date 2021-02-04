#ifndef __MYLINKXS_H
#define __MYLINKXS_H

#define SIMULADO 1

/*estudo placa CC2538EM
 * botao SEL : RF1.14 ligado no conector P1 - pino de IO PA3
 * pino PB0  : RF1.3 ligado no conector P1 - pino de IO PB0
 * pino PB1  : RF1.5 ligado no conector P1 - pino de IO PB1
 *
 *
 * vou usar PB0 e PB1 para o controle da luz
 * PB0 - Entrada
 * PB1 - Saida
 */
// Board LED defines
#define BSP_LED_BASE            GPIO_C_BASE
#define BSP_LED_1               GPIO_PIN_0      //!< PC0
#define BSP_LED_2               GPIO_PIN_1      //!< PC1
#define BSP_LED_3               GPIO_PIN_2      //!< PC2
#define BSP_LED_4               GPIO_PIN_3      //!< PC3


//#if MYLINKXS_REMOTE_CONTROL

// Units Values for decode_type
#define NEC 1
#define SONY 2
#define RC5 3
#define RC6 4
#define DISH 5
#define SHARP 6
#define PANASONIC 7
#define JVC 8
#define SANYO 9
#define MITSUBISHI 10
#define SAMSUNG 11
#define LG 12
#define UNKNOWN -1

//#endif

//logica da Lampada
//rele eh aceso em baixa
#define BSP_LIGHT_ON  0
#define BSP_LIGHT_OFF  BSP_LIGHT_LED_OUT

#if SIMULADO

#define BSP_LIGHT_BOTAO_BASE    GPIO_A_BASE
#define BSP_LIGHT_BOTAO         GPIO_PIN_3      //!< PA3  //botao SEL
#define BSP_LIGHT_INT_GPIO      INT_GPIOA

#define BSP_LIGHT_LED_OUT_BASE  GPIO_B_BASE
#define BSP_LIGHT_LED_OUT       GPIO_PIN_4      //!< RF1.15  PB4

//USO O LED 3 para indicar a lampada

#define BSP_LIGHT_LED_IN_BASE    GPIO_C_BASE
#define BSP_LIGHT_LED_IN         GPIO_PIN_2      //!< LED 3 indica a lampada ligada ou desligada


#else
#define BSP_LIGHT_BASE          GPIO_B_BASE
#define BSP_LIGHT_IN            GPIO_PIN_0      //!< PB0
#define BSP_LIGHT_OUT           GPIO_PIN_1      //!< PB1

//USO O LED 3 para indicar a lampada

#define BSP_LIGHT_LED_IN_BASE    GPIO_C_BASE
#define BSP_LIGHT_LED_IN         GPIO_PIN_2      //!< LED 3 indica a lampada ligada ou desligada


#endif





/** Sensor interface standard datatypes */
enum sens_itf_datatypes_e
{
	SENS_ITF_DT_U8     = 0x00, /**< 8 bits unsigned */
	SENS_ITF_DT_S8     = 0x01, /**< 8 bits signed */
	SENS_ITF_DT_U16    = 0x02, /**< 16 bits unsigned */
	SENS_ITF_DT_S16    = 0x03, /**< 16 bits signed */
	SENS_ITF_DT_U32    = 0x04, /**< 32 bits unsigned */
	SENS_ITF_DT_S32    = 0x05, /**< 32 bits signed */
	SENS_ITF_DT_U64    = 0x06, /**< 64 bits unsigned */
	SENS_ITF_DT_S64    = 0x07, /**< 64 bits signed */
	SENS_ITF_DT_FLOAT  = 0x08, /**< IEEE 754 single precision */
	SENS_ITF_DT_DOUBLE = 0x09, /**< IEEE 754 double precision */
};


enum sens_itf_access_rights_e
{
	SENS_ITF_ACCESS_READ_ONLY = 0x01,
	SENS_ITF_ACCESS_WRITE_ONLY = 0x02,
	SENS_ITF_ACCESS_READ_WRITE = 0x03,
};



void light_mote_sm(void);
void light_timer(void);
void light_off(void);
void light_on(void);
void light_init(void);
uint8_t light_get_value(void);
#endif


