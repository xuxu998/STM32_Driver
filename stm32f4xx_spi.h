#ifndef _SPI_HEADER_
#define _SPI_HEADER_
#define AA


#include <stdint.h>

typedef struct 
{
     /* data */
     __vo uint32_t CR1;                                     /*  */
     __vo uint32_t CR2;                                     /*  */
     __vo uint32_t SR;                                      /*  */
     __vo uint32_t DR;                                      /*  */
     __vo uint32_t CRCPR;
     __vo uint32_t RXCRCR;
     __vo uint32_t TXCRCR;
     __vo uint32_t I2SCFGR;
     __vo uint32_t I2SPR;

}SPI_RegDef_t;

typedef struct 
{
     /* data */
     uint8_t SPI_DeviceMode;
     uint8_t SPI_BusConfig;
     uint8_t SPI_ClkSpeed;
     uint8_t SPI_DFF;
     uint8_t SPI_CPOL;
     uint8_t SPI_CPHA;
     uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct 
{
     /* data */
     SPI_RegDef_t *SPIx;
     SPI_Config_t SPIConfig;

}SPI_Handle_t;


#endif 


