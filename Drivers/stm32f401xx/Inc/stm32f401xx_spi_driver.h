#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"

/*
 *  Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define SPI1	( (SPI_RegDef_t*)SPI1_BASEADDR )
#define SPI2	( (SPI_RegDef_t*)SPI2_I2S2_BASEADDR )
#define SPI3	( (SPI_RegDef_t*)SPI3_I2S3_BASEADDR )
#define SPI4	( (SPI_RegDef_t*)SPI4_BASEADDR )

/*
 * Bit-field structure for SPI control register 1
 */
typedef struct
{ 
    uint32_t CPHA:1;                /* Clock phase                              Bit: 0      */ 
    uint32_t CPOL:1;                /* Clock polarity                           Bit: 1      */ 
    uint32_t MSTR:1;                /* Master selection                         Bit: 2      */
    uint32_t BR:3;                  /* Baud rate control                        Bit: 3-5    */
    uint32_t SPE:1;                 /* SPI enable                               Bit: 6      */
    uint32_t LSBFIRST:1;            /* Frame format                             Bit: 7      */
    uint32_t SSI:1;                 /* Internal slave select                    Bit: 8      */
    uint32_t SSM:1;                 /* Software slave management                Bit: 9      */
    uint32_t RXONLY:1;              /* Receive only                             Bit: 10     */
    uint32_t DFF:1;                 /* Data frame format                        Bit: 11     */
    uint32_t CRCNEXT:1;             /* CRC transfer next                        Bit: 12     */
    uint32_t CRCEN:1;               /* Hardware CRC calculation enable          Bit: 13     */
    uint32_t BIDIOE:1;              /* Output enable in bidirectional mode      Bit: 14     */
    uint32_t BIDIMODE:1;            /* Bidirectional data mode enable           Bit: 15     */
    uint32_t RESERVED:16;           /* Reserved, must be kept at reset value    Bit: 16-31  */
} SPI_CR1_t;

/*
 * Bit-field structure for SPI control register 2
 */
typedef struct
{ 
    uint32_t RXDMAEN:1;             /* Clock phase                              Bit: 0      */ 
    uint32_t TXDMAEN:1;             /* Tx buffer DMA enable                     Bit: 1      */ 
    uint32_t SSOE:1;                /* SS output enable                         Bit: 2      */
    uint32_t RESERVED1:1;           /* Reserved. Forced to 0 by hardware.       Bit: 3      */
    uint32_t FRF:1;                 /* Frame format                             Bit: 4      */
    uint32_t ERRIE:1;               /* Error interrupt enable                   Bit: 5      */
    uint32_t RXNEIE:1;              /* RX buffer not empty interrupt enable     Bit: 6      */
    uint32_t TXEIE:1;               /* Tx buffer empty interrupt enable         Bit: 7      */
    uint32_t RESERVED2:24;          /* Reserved, must be kept at reset value    Bit: 8-31   */
} SPI_CR2_t;

/*
 * Bit-field structure for SPI status register
 */
typedef struct
{ 
    uint32_t RXNE:1;                /* Receive buffer not empty                 Bit: 0      */ 
    uint32_t TXE:1;                 /* Transmit buffer empty                    Bit: 1      */ 
    uint32_t CHSIDE:1;              /* Channel side                             Bit: 2      */
    uint32_t UDR:1;                 /* Underrun flag                            Bit: 3      */
    uint32_t CRCERR:1;              /* CRC error flag                           Bit: 4      */
    uint32_t MODF:1;                /* Mode fault                               Bit: 5      */
    uint32_t OVR:1;                 /* Overrun flag                             Bit: 6      */
    uint32_t BSY:1;                 /* Busy flag                                Bit: 7      */
    uint32_t FRE:1;                 /* Frame format error                       Bit: 8      */
    uint32_t RESERVED:23;           /* Reserved, must be kept at reset value    Bit: 9-31   */
} SPI_SR_t;

/*
 * Bit-field structure for SPI data register
 */
typedef struct
{ 
    uint32_t DR:16;                 /* Data register                            Bit: 0-15   */    
    uint32_t RESERVED:16;           /* Reserved, must be kept at reset value    Bit: 16-31  */
} SPI_DR_t;

/*
 * Bit-field structure for SPI CRC polynomial register
 */
typedef struct
{ 
    uint32_t CRCPOLY:16;            /* CRC polynomial register                  Bit: 0-15   */    
    uint32_t RESERVED:16;           /* Reserved, must be kept at reset value    Bit: 16-31  */
} SPI_CRCPR_t;

/*
 * Bit-field structure for SPI RX CRC register
 */
typedef struct
{ 
    uint32_t RXCRC:16;              /* Rx CRC register                          Bit: 0-15   */    
    uint32_t RESERVED:16;           /* Reserved, must be kept at reset value    Bit: 16-31  */
} SPI_RXCRCR_t;

/*
 * Bit-field structure for SPI TX CRC register
 */
typedef struct
{ 
    uint32_t TXCRC:16;              /* Tx CRC register                          Bit: 0-15   */    
    uint32_t RESERVED:16;           /* Reserved, must be kept at reset value    Bit: 16-31  */
} SPI_TXCRCR_t;

/*
 * Bit-field structure for SPI_I2S configuration register
 */
typedef struct
{ 
    uint32_t CHLEN:1;               /* Channel length. Not used in SPI mode                 Bit: 0      */ 
    uint32_t DATLEN:2;              /* Data length to be transferred. Not used in SPI mode  Bit: 1-2    */ 
    uint32_t CKPOL:1;               /* Steady state clock polarity. Not used in SPI mode    Bit: 3      */
    uint32_t I2SSTD:2;              /* I2S standard selection. Not used in SPI mode         Bit: 4-5    */
    uint32_t RESERVED1:1;           /* Reserved. Forced to 0 by hardware.                   Bit: 6      */
    uint32_t PCMSYNC:1;             /* PCM frame synchronization. Not used in SPI mode      Bit: 7      */
    uint32_t I2SCFG:2;              /* I2S configuration mode. Not used in SPI mode         Bit: 8-9    */
    uint32_t I2SE:1;                /* I2S Enable. Not used in SPI mode                     Bit: 10     */
    uint32_t I2SMOD:1;              /* 0: SPI mode is selected, 1: I2S mode is selected     Bit: 11     */
    uint32_t RESERVED:20;           /* Reserved, must be kept at reset value                Bit: 12-31  */
} SPI_I2SCFGR_t;

/*
 * Bit-field structure for SPI_I2S prescaler register
 */
typedef struct
{ 
    uint32_t I2SDIV:8;              /* I2S Linear prescaler. Not used in SPI mode           Bit: 0      */ 
    uint32_t ODD:1;                 /* Odd factor for the prescaler. Not used in SPI mode   Bit: 8      */ 
    uint32_t MCKOE:1;               /* Master clock output enable. Not used in SPI mode     Bit: 9      */    
    uint32_t RESERVED:22;           /* Reserved, must be kept at reset value                Bit: 15-31  */
} SPI_I2SPR_t;

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
	volatile SPI_CR1_t CR1;	        /* SPI control register 1 					Address offset: 0x00 */
	volatile SPI_CR2_t CR2;	        /* SPI control register 2			        Address offset: 0x04 */
	volatile SPI_SR_t SR;	        /* SPI status register 			            Address offset: 0x08 */
	volatile uint32_t DR;	        /* SPI data register		                Address offset: 0x0C */
	volatile SPI_CRCPR_t CRCPR;		/* SPI CRC polynomial register 			    Address offset: 0x10 */
	volatile SPI_RXCRCR_t RXCRCR;	/* SPI RX CRC register			            Address offset: 0x14 */
	volatile SPI_TXCRCR_t TXCRCR;	/* SPI TX CRC register          			Address offset: 0x18 */
	volatile SPI_I2SCFGR_t I2SCFGR;	/* SPI_I2S configuration register       	Address offset: 0x1C */
	volatile SPI_I2SPR_t I2SPR;		/* SPI_I2S prescaler register   			Address offset: 0x20 */
} SPI_RegDef_t;

/*
* @SPI_device_mode_t
*/
typedef enum {
  SPI_SLAVE,
  SPI_MASTER
} SPI_device_mode_t;

/*
* @SPI_bus_config_t
*/
typedef enum {
  SPI_FULL_DUPLEX = 1,
  SPI_HALF_DUPLEX,
  SPI_SIMPLEX_RX_ONLY
} SPI_bus_config_t;

/*
* @SPI_sclk_speed_t
*/
typedef enum {
  SPI_FPCLK_DIV2,
  SPI_FPCLK_DIV4,
  SPI_FPCLK_DIV8,
  SPI_FPCLK_DIV16,
  SPI_FPCLK_DIV32,
  SPI_FPCLK_DIV64,
  SPI_FPCLK_DIV128,
  SPI_FPCLK_DIV256
} SPI_sclk_speed_t;

/*
* @SPI_data_frame_format_t
*/
typedef enum {
  SPI_DFF_8BITS,
  SPI_DFF_16BITS
} SPI_data_frame_format_t;

/*
* @SPI_clock_polarity_t
*/
typedef enum {
  SPI_CPOL_LOW,
  SPI_CPOL_HIGH
} SPI_clock_polarity_t;

/*
* @SPI_clock_phase_t
*/
typedef enum {
  SPI_CPHA_LOW,
  SPI_CPHA_HIGH
} SPI_clock_phase_t;

/*
* @SPI_ssm_mode_t
*/
typedef enum {
  SPI_SSM_DISABLED,
  SPI_SSM_ENABLED
} SPI_ssm_mode_t;

/*
* @SPI_ssm_mode_t
* SPI related status flags definitions
*/
typedef enum {
  SPI_TXE_FLAG,
  SPI_RXNE_FLAG,
  SPI_BUSY_FLAG
} SPI_status_flag_t;

/*
* @SPI_app_state_t
* SPI related status flags definitions
*/
typedef enum {
  SPI_READY,
  SPI_BUSY_IN_RX,
  SPI_BUSY_IN_TX
} SPI_app_state_t;

/*
* @SPI_Config_t
* Configuration structure for SPIx peripheral
*/
typedef struct {
    SPI_device_mode_t SPI_DeviceMode;   /* SPI device mode from @SPI_device_mode_t */
    SPI_bus_config_t SPI_BusConfig;     /* SPI Bus Configuration from @SPI_bus_config_t */
    SPI_sclk_speed_t SPI_SclkSpeed;     /* SPI clock speed from @SPI_sclk_speed_t */
    SPI_data_frame_format_t SPI_DFF;    /* SPI Data Frame Format from @SPI_data_frame_format_t */
    SPI_clock_polarity_t SPI_CPOL;      /* SPI Clock Polarity from @SPI_clock_polarity_t */
    SPI_clock_phase_t SPI_CPHA;         /* SPI Clock Phase from @SPI_clock_phase_t */
    SPI_ssm_mode_t SPI_SSM;             /* SPI Software Slave Managment from @SPI_ssm_mode_t */
} SPI_Config_t;

/*
* @SPI_Handle_t
* Handle structure for SPIx peripheral
*/
typedef struct 
{
    SPI_RegDef_t    *pSPIx; // This holds the base address of SPIx(x:0,1,2,3) peripheral
    SPI_Config_t    SPI_Config;
    uint8_t         *pTxBuffer; // To store the application Tx buffer address
    uint8_t         *pRxBuffer; // To store the application Rx buffer address
    uint32_t        TxLen;    
    uint32_t        RxLen;    
    SPI_app_state_t TxState;  
    SPI_app_state_t RxState;  
} SPI_Handle_t;


/************************************************************************************************
 *
 * APIs supported by this driver
 * For more information about the APIs check the function definitions
 *
 *************************************************************************************************/

/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

SPI_app_state_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
SPI_app_state_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , SPI_status_flag_t FlagName);

#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */