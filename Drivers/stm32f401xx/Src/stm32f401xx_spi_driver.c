#include "stm32f401xx_spi_driver.h"

/**
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- Enables or disables peripheral clock for the given SPI port
 *
 * @param		- pSPIx: Pointer to the base address of the SPI peripheral
 * @param		- EnorDi: Either enable or disable the SPI peripheral, use ENABLE or DISABLE macros
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
    if (EnorDi == ENABLE){
		if 	    (pSPIx == SPI1) SPI1_PERI_CLK_EN();
		else if (pSPIx == SPI2) SPI2_PERI_CLK_EN();
		else if (pSPIx == SPI3) SPI3_PERI_CLK_EN();
		else if (pSPIx == SPI4) SPI4_PERI_CLK_EN();
	} else {
		if 	    (pSPIx == SPI1) SPI1_PERI_CLK_DI();
		else if (pSPIx == SPI2) SPI2_PERI_CLK_DI();
		else if (pSPIx == SPI3) SPI3_PERI_CLK_DI();
		else if (pSPIx == SPI4) SPI4_PERI_CLK_DI();
	}      
}

/**
 * @fn			- SPI_Init
 *
 * @brief		- Initializes and configures the  given SPI port
 *
 * @param		- pSPIHandle: Pointer to the handle of SPI port configuration
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void SPI_Init(SPI_Handle_t *pSPIHandle){
    // Peripheral clock enable
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // 1. Configure device mode
    pSPIHandle->pSPIx->CR1.MSTR = pSPIHandle->SPI_Config.SPI_DeviceMode; 

    // 2. Bus configuration
    if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_FULL_DUPLEX) {
        // bidi mode should be cleared
        pSPIHandle->pSPIx->CR1.BIDIMODE = 0;
    } else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_HALF_DUPLEX) {
        // bidi mode should be set
        pSPIHandle->pSPIx->CR1.BIDIMODE = 1;
    } else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_SIMPLEX_RX_ONLY) {
        // bidi mode should be cleared and RXONLY bit must be set
        pSPIHandle->pSPIx->CR1.BIDIMODE = 0;
        pSPIHandle->pSPIx->CR1.RXONLY = 1;
    }

    // 3. Configure bus clock speed
    pSPIHandle->pSPIx->CR1.BR = pSPIHandle->SPI_Config.SPI_SclkSpeed;
    // 4. Configure data frame format
    pSPIHandle->pSPIx->CR1.DFF = pSPIHandle->SPI_Config.SPI_DFF;
    // 5. Configure clock polarity
    pSPIHandle->pSPIx->CR1.CPOL = pSPIHandle->SPI_Config.SPI_CPOL;
    // 6. Configure clock phase
    pSPIHandle->pSPIx->CR1.CPHA = pSPIHandle->SPI_Config.SPI_CPHA;
    // 7. Configure software slave managment
    pSPIHandle->pSPIx->CR1.SSM = pSPIHandle->SPI_Config.SPI_SSM;    

}

/**
 * @fn			- SPI_DeInit
 *
 * @brief		- Resets the given SPI port to the default state
 *
 * @param		- pSPIx: Pointer to the base address of the SPI peripheral
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void SPI_DeInit(SPI_RegDef_t *pSPIx){
    
}

/**
 * @fn			- SPI_SendData
 *
 * @brief		- Send data over the SPI port
 *
 * @param		- pSPIx: Pointer to the base address of the SPI peripheral
 * @param		- pTxBuffer: Pointer to the transmision buffer
 * @param		- Len: Lenght of the data to be transmitted
 *
 * @return		- None
 *
 * @note		- This is a blocking call
 *
 **/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
    while (Len > 0) {
        // 1. Wait until TXE is set
        while (pSPIx->SR.TXE == 0);
        // 2. Check the DFF bit in CR1
        if (pSPIx->CR1.DFF == 1) {  // 16 bit DFF 
            // Load the data into the DR
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len -= 2;
            (uint16_t*)pTxBuffer++;
        } else {    // 8 bit DFF            
            pSPIx->DR = *pTxBuffer;
            //*((volatile uint8_t *)&pSPIx->DR) = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}

/**
 * @fn			- SPI_ReceiveData
 *
 * @brief		- Send data over the SPI port
 *
 * @param		- pSPIx: Pointer to the base address of the SPI peripheral
 * @param		- pRxBuffer: Pointer to the reception buffer
 * @param		- Len: Lenght of the data to be recieved
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
    while (Len > 0) {
        // 1. Wait until RXNE is set
        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == 0);
        // 2. Check the DFF bit in CR1
        if (pSPIx->CR1.DFF == 1) {  // 16 bit DFF 
            // Load the data from DR to Rx buffer address
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            Len -= 2;
            (uint16_t*)pRxBuffer++;
        } else {    // 8 bit DFF            
            *pRxBuffer = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}

/**
 * @fn			- SPI_SendDataIT
 *
 * @brief		- Send data over the SPI port
 *
 * @param		- pSPIHandle: Pointer to the handle of SPI port configuration
 * @param		- pTxBuffer: Pointer to the transmision buffer
 * @param		- Len: Lenght of the data to be transmitted
 *
 * @return		- SPI_app_state_t
 *
 * @note		- This is a non blocking call
 *
 **/
SPI_app_state_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {
    SPI_app_state_t state = pSPIHandle->TxState;
    if (state != SPI_BUSY_IN_TX) {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pSPIHandle;
        pSPIHandle->TxLen = Len;
        // 2. Mark the SPI state as busy in transmission so that no other  code can 
        //    take over same SPI peripheral until transmission is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;
        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2.TXEIE = 1;  
        // 4. Data transmission will be handled by the ISR code 
    }    
    
    return state;
}

/**
 * @fn			- SPI_ReceiveDataIT
 *
 * @brief		- Send data over the SPI port
 *
 * @param		- pSPIHandle: Pointer to the handle of SPI port configuration
 * @param		- pRxBuffer: Pointer to the reception buffer
 * @param		- Len: Lenght of the data to be recieved
 *
 * @return		- SPI_app_state_t
 *
 * @note		- None
 *
 **/
SPI_app_state_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {
    SPI_app_state_t state = pSPIHandle->RxState;
    if (state != SPI_BUSY_IN_RX) {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pRxBuffer = pSPIHandle;
        pSPIHandle->RxLen = Len;
        // 2. Mark the SPI state as busy in transmission so that no other  code can 
        //    take over same SPI peripheral until transmission is over
        pSPIHandle->RxState = SPI_BUSY_IN_RX;
        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2.RXNEIE = 1;  
        // 4. Data transmission will be handled by the ISR code 
    }    
    
    return state;
}

/**
 * @fn			- SPI_IRQInterruptConfig
 *
 * @brief		- Enables or disables an interrupt
 *
 * @param		- IRQNumber: Number or position of the interrupt in the NVIC Vector Table (0-84)
 * @param		- EnorDi: Either enable or disable an interrupt, use ENABLE or DISABLE macros
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
    if (EnorDi == ENABLE){
		if (IRQNumber <= 31) {
			// 0 - 31, ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// 32 - 63, ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// 64 - 95, ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	} else {
		if (IRQNumber <= 31) {
			// 0 - 31, ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// 32 - 63, ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// 64 - 95, ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}

/**
 * @fn			- SPI_IRQPriorityConfig
 *
 * @brief		- Configures the interrupt request priority level
 *
 * @param		- IRQNumber: Number or position of the interrupt in the NVIC Vector Table (0-84)
 * @param		- IRQPriority: Priority level (0 - 15), the less the level the higher the priority
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
    // 1. Lets find out the IPR register
	uint8_t ipr_reg_index = IRQNumber / 4;	// Fields per register = 4, NVIC_IPR_index = IRQNumber / Sections per register
	uint8_t ipr_field_index = IRQNumber % 4;
	uint8_t ipr_field_bits_lenght = 8;

	// We need to shit 4 bits more because each ipr field implements the priority level in the the 4 msb, the 4 lsb are not implemented
	uint8_t shift_amount = (ipr_field_bits_lenght * ipr_field_index) + (8 - PRIORITY_BITS_IMPLEMENTED_NUM);
	*(NVIC_PRIORITY_BASE_ADDRESS + ipr_reg_index) |= IRQPriority << shift_amount;
	/*uint32_t ipr_value = IRQPriority << shift_amount;
	volatile uint32_t *ipr_reg_address = NVIC_PRIORITY_BASE_ADDRESS + ipr_reg_index;
	*ipr_reg_address |= ipr_value;*/
}

/**
 * @fn			- SPI_IRQHandling
 *
 * @brief		- Clears the EXTI Pending Register corresponding to the pin number
 *
 * @param		- pSPIHandle: Pointer to the handle of SPI port configuration
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

}

/**
 * @fn			- SPI_PeripheralControl
 *
 * @brief		- Enables or disables the SPI peripheral
 *
 * @param		- pSPIx: Pointer to the base address of the SPI peripheral
 * @param		- EnorDi: Either enable or disable the SPI peripheral, use ENABLE or DISABLE macros
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR1.SPE = 1;
    } else {
        pSPIx->CR1.SPE = 0;
    }
}

/**
 * @fn			- SPI_SSIConfig
 *
 * @brief		- Internal slave select. This bit has an effect only when the Software slave management (SSM) bit is set
 *
 * @param		- pSPIx: Pointer to the base address of the SPI peripheral
 * @param		- EnorDi: Either enable or disable the slave, use ENABLE or DISABLE macros
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR1.SSI = 1;
    } else {
        pSPIx->CR1.SSI = 0;
    }
}

/**
 * @fn			- SPI_SSOEConfig
 *
 * @brief		- SS output enable: The NSS pin may also be used as an output if enabled and driven low if the SPI is in master configuration.
 *
 * @param		- pSPIx: Pointer to the base address of the SPI peripheral
 * @param		- EnorDi: Either enable or disable the SS output, use ENABLE or DISABLE macros
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR2.SSOE = 1;
    } else {
        pSPIx->CR2.SSOE = 0;
    }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , SPI_status_flag_t FlagName)
{
	if (FlagName == SPI_TXE_FLAG) { 
		return pSPIx->SR.TXE;
	} else if (FlagName == SPI_RXNE_FLAG) {
        return pSPIx->SR.RXNE;
    } else {    // FlagName == SPI_BUSY_FLAG
        return pSPIx->SR.BSY;
    }
}