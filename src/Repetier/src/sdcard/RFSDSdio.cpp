#include "Repetier.h"

#if HAS_SDIO_SD_SLOT == 1
#if defined(STM32F4_BOARD) || defined(STM32F1_BOARD)

#include <stdint.h>
#include <stdbool.h>

// use local drivers
#if defined(STM32F103xE) || defined(STM32F103xG)
#include <stm32f1xx_hal_rcc_ex.h>
#include <stm32f1xx_hal_sd.h>
#elif defined(STM32F4xx)
#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_dma.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_sd.h>
#elif defined(STM32F7xx)
#include <stm32f7xx_hal_rcc.h>
#include <stm32f7xx_hal_dma.h>
#include <stm32f7xx_hal_gpio.h>
#include <stm32f7xx_hal_sd.h>
#else
#error "SDIO only supported with STM32F103xE, STM32F103xG, STM32F4xx, or STM32F7xx."
#endif

/*
  SDIO_INIT_CLK_DIV is 118
  SDIO clock frequency is 48MHz / (TRANSFER_CLOCK_DIV + 2)
  SDIO init clock frequency should not exceed 400kHz = 48MHz / (118 + 2)

  Default TRANSFER_CLOCK_DIV is 2 (118 / 40)
  Default SDIO clock frequency is 48MHz / (2 + 2) = 12 MHz
  This might be too fast for stable SDIO operations

  MKS Robin board seems to have stable SDIO with BusWide 1bit and ClockDiv 8 i.e. 4.8MHz SDIO clock frequency
  Additional testing is required as there are clearly some 4bit initialization problems
*/

#ifndef USBD_OK
#define USBD_OK 0
#endif

// Target Clock, configurable. Default is 18MHz, from STM32F1
#ifndef SDIO_CLOCK
#define SDIO_CLOCK 18000000 // 18 MHz
#endif

// SDIO retries, configurable. Default is 3, from STM32F1
#ifndef SDIO_READ_RETRIES
#define SDIO_READ_RETRIES 3
#endif

// SDIO Max Clock (naming from STM Manual, don't change)
#ifndef SDIOCLK
#define SDIOCLK 48000000
#endif

SD_HandleTypeDef RFSDSdio::hsd;
DMA_HandleTypeDef RFSDSdio::hdma_sdio;

static uint32_t clockToDivider(uint32_t clk) {
    // limit the SDIO master clock to 8/3 of PCLK2. See STM32 Manuals
    // Also limited to no more than 48Mhz (SDIOCLK).
    const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    clk = min(clk, (uint32_t)(pclk2 * 8 / 3));
    clk = min(clk, (uint32_t)SDIOCLK);
    // Round up divider, so we don't run the card over the speed supported,
    // and subtract by 2, because STM32 will add 2, as written in the manual:
    // SDIO_CK frequency = SDIOCLK / [CLKDIV + 2]
    return pclk2 / clk + (pclk2 % clk != 0) - 2;
}

void RFSDSdio::goToTransferSpeed() {
    /* Default SDIO peripheral configuration for SD card initialization */
    hsd.Init.ClockEdge = hsd.Init.ClockEdge;
    hsd.Init.ClockBypass = hsd.Init.ClockBypass;
    hsd.Init.ClockPowerSave = hsd.Init.ClockPowerSave;
    hsd.Init.BusWide = hsd.Init.BusWide;
    hsd.Init.HardwareFlowControl = hsd.Init.HardwareFlowControl;
    hsd.Init.ClockDiv = clockToDivider(SDIO_CLOCK);

    /* Initialize SDIO peripheral interface with default configuration */
    SDIO_Init(hsd.Instance, hsd.Init);
}

void RFSDSdio::SDLowLevelInit(void) {
    uint32_t tempreg;

    __HAL_RCC_GPIOC_CLK_ENABLE(); //enable GPIO clocks
    __HAL_RCC_GPIOD_CLK_ENABLE(); //enable GPIO clocks

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = 1; //GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

#if DISABLED(STM32F1xx)
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
#endif

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_12; // D0 & SCK
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

#if PIN_EXISTS(SDIO_D1) && PIN_EXISTS(SDIO_D2) && PIN_EXISTS(SDIO_D3) // define D1-D3 only if have a four bit wide SDIO bus
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;     // D1-D3
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

    // Configure PD.02 CMD line
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

// Setup DMA
#if defined(STM32F1xx)
    hdma_sdio.Init.Mode = DMA_NORMAL;
    hdma_sdio.Instance = DMA2_Channel4;
    HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);
#elif defined(STM32F4xx)
    hdma_sdio.Init.Mode = DMA_PFCTRL;
    hdma_sdio.Instance = DMA2_Stream3;
    hdma_sdio.Init.Channel = DMA_CHANNEL_4;
    hdma_sdio.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_sdio.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_sdio.Init.MemBurst = DMA_MBURST_INC4;
    hdma_sdio.Init.PeriphBurst = DMA_PBURST_INC4;
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
#endif
    HAL_NVIC_EnableIRQ(SDIO_IRQn);
    hdma_sdio.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sdio.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sdio.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sdio.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sdio.Init.Priority = DMA_PRIORITY_LOW;
    __HAL_LINKDMA(&hsd, hdmarx, hdma_sdio);
    __HAL_LINKDMA(&hsd, hdmatx, hdma_sdio);

#if defined(STM32F1xx)
    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
#else
    __HAL_RCC_SDIO_FORCE_RESET();
    delay(2);
    __HAL_RCC_SDIO_RELEASE_RESET();
    delay(2);
    __HAL_RCC_SDIO_CLK_ENABLE();

    //__HAL_RCC_DMA2_FORCE_RESET();
    // delay(2);
    //__HAL_RCC_DMA2_RELEASE_RESET();
    // delay(2);
    __HAL_RCC_DMA2_CLK_ENABLE();
#endif

    //Initialize the SDIO (with initial <400Khz Clock)
    tempreg = 0;                  //Reset value
    tempreg |= SDIO_CLKCR_CLKEN;  // Clock enabled
    tempreg |= SDIO_INIT_CLK_DIV; // Clock Divider. Clock = 48000 / (118 + 2) = 400Khz
    // Keep the rest at 0 => HW_Flow Disabled, Rising Clock Edge, Disable CLK ByPass, Bus Width = 0, Power save Disable
    SDIO->CLKCR = tempreg;

    // Power up the SDIO
    SDIO_PowerState_ON(SDIO);
    hsd.Instance = SDIO;
}

bool RFSDSdio::SDIOInit() {
    uint8_t retryCnt = SDIO_READ_RETRIES;

    bool status;
    hsd.Instance = SDIO;
    hsd.State = HAL_SD_STATE_RESET;

    SDLowLevelInit();

    uint8_t retry_Cnt = retryCnt;
    for (;;) {
        HAL::pingWatchdog();
        status = (bool)HAL_SD_Init(&hsd);
        if (!status) {
            break;
        }
        if (!--retry_Cnt) {
            lastErrorCode = SD_CARD_ERROR_INVALID_CARD_CONFIG;
            return false; // return failing status if retries are exhausted
        }
    }

    goToTransferSpeed();

#if PIN_EXISTS(SDIO_D1) && PIN_EXISTS(SDIO_D2) && PIN_EXISTS(SDIO_D3) // go to 4 bit wide mode if pins are defined
    retry_Cnt = retryCnt;
    for (;;) {
        HAL::pingWatchdog();
        if (!HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B)) {
            break; // some cards are only 1 bit wide so a pass here is not required
        }
        if (!--retry_Cnt) {
            break;
        }
    }
    if (!retry_Cnt) {                       // wide bus failed, go back to one bit wide mode
        hsd.State = (HAL_SD_StateTypeDef)0; // HAL_SD_STATE_RESET
        SDLowLevelInit();
        retry_Cnt = retryCnt;
        for (;;) {
            HAL::pingWatchdog();
            status = (bool)HAL_SD_Init(&hsd);
            if (!status) {
                break;
            }
            if (!--retry_Cnt) {
                lastErrorCode = SD_CARD_ERROR_INVALID_CARD_CONFIG;
                return false; // return failing status if retries are exhausted
            }
        }
        goToTransferSpeed();
    }
#endif

    return true;
}

bool RFSDSdio::SDIOReadWriteBlockDMA(uint32_t block, const uint8_t* src, uint8_t* dst) {
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
        return false;

    HAL::pingWatchdog();

    HAL_StatusTypeDef ret;
    if (src) {
        hdma_sdio.Init.Direction = DMA_MEMORY_TO_PERIPH;
        HAL_DMA_Init(&hdma_sdio);
        ret = HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t*)src, block, 1);
    } else {
        hdma_sdio.Init.Direction = DMA_PERIPH_TO_MEMORY;
        HAL_DMA_Init(&hdma_sdio);
        ret = HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t*)dst, block, 1);
    }

    if (ret != HAL_OK) {
        HAL_DMA_Abort_IT(&hdma_sdio);
        HAL_DMA_DeInit(&hdma_sdio);
        lastErrorCode = src ? SD_CARD_ERROR_WRITE_START : SD_CARD_ERROR_READ_START;
        return false;
    }

    millis_t timeout = millis() + 500;
    // Wait the transfer
    while (hsd.State != HAL_SD_STATE_READY) {
        if (ELAPSED(millis(), timeout)) {
            HAL_DMA_Abort_IT(&hdma_sdio);
            HAL_DMA_DeInit(&hdma_sdio);
            lastErrorCode = src ? SD_CARD_ERROR_WRITE_TIMEOUT : SD_CARD_ERROR_READ_TIMEOUT;
            return false;
        }
    }

    while (__HAL_DMA_GET_FLAG(&hdma_sdio, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_sdio)) != 0
           || __HAL_DMA_GET_FLAG(&hdma_sdio, __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_sdio)) != 0) { /* nada */
    }

    HAL_DMA_Abort_IT(&hdma_sdio);
    HAL_DMA_DeInit(&hdma_sdio);

    timeout = millis() + 500;
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
        if (ELAPSED(millis(), timeout)) {
            lastErrorCode = src ? SD_CARD_ERROR_WRITE_TIMEOUT : SD_CARD_ERROR_READ_TIMEOUT;
            return false;
        }

    return true;
}

bool RFSDSdio::SDIOReadBlock(uint32_t block, uint8_t* dst) {
    uint8_t retries = SDIO_READ_RETRIES;
    while (retries--) {
        if (SDIOReadWriteBlockDMA(block, nullptr, dst)) {
            return true;
        }
    }
    return false;
}

bool RFSDSdio::SDIOWriteBlock(uint32_t block, const uint8_t* src) {
    uint8_t retries = SDIO_READ_RETRIES;
    while (retries--) {
        if (SDIOReadWriteBlockDMA(block, src, nullptr)) {
            return true;
        }
    }
    return false;
}

bool RFSDSdio::SDIOIsReady() {
    return hsd.State == HAL_SD_STATE_READY;
}

bool RFSDSdio::SDIOIsBusy() {
    return hsd.State == HAL_SD_STATE_BUSY;
}

uint32_t RFSDSdio::SDIOGetCardSize() {
    return (uint32_t)(hsd.SdCard.BlockNbr) * (hsd.SdCard.BlockSize);
}

uint32_t RFSDSdio::SDIOGetBlocks() {
    return (uint32_t)(hsd.SdCard.BlockNbr);
}

#if defined(STM32F1xx)
#define DMA_IRQ_HANDLER DMA2_Channel4_5_IRQHandler
#elif defined(STM32F4xx)
#define DMA_IRQ_HANDLER DMA2_Stream3_IRQHandler
#else
#error "Unknown STM32 architecture."
#endif

extern "C" void SDIO_IRQHandler(void) { HAL_SD_IRQHandler(&RFSDSdio::hsd); }
extern "C" void DMA_IRQ_HANDLER(void) { HAL_DMA_IRQHandler(&RFSDSdio::hdma_sdio); }

bool RFSDSdio::init() {
    if (SDIOInit()) {
        lastErrorCode = SD_CARD_ERROR_NONE;
        return true;
    }
    return false;
}
/** end use of device */
void RFSDSdio::end() { }
/**
   * Check for FsBlockDevice busy.
   *
   * \return true if busy else false.
   */
bool RFSDSdio::isBusy() {
    return SDIOIsBusy();
}
/**
   * Read a sector.
   *
   * \param[in] sector Logical sector to be read.
   * \param[out] dst Pointer to the location that will receive the data.
   * \return true for success or false for failure.
   */
bool RFSDSdio::readSector(uint32_t sector, uint8_t* dst) {
    return SDIOReadBlock(sector, dst);
}

/**
   * Read multiple sectors.
   *
   * \param[in] sector Logical sector to be read.
   * \param[in] ns Number of sectors to be read.
   * \param[out] dst Pointer to the location that will receive the data.
   * \return true for success or false for failure.
   */
bool RFSDSdio::readSectors(uint32_t sector, uint8_t* dst, size_t ns) {
    while (ns) {
        if (!SDIOReadBlock(sector, dst)) {
            return false;
        }
        sector++;
        dst += 512;
        ns--;
    }
    return true;
}

/** \return device size in sectors. */
uint32_t RFSDSdio::sectorCount() {
    csd_t csd;
    return readCSD(&csd) ? csd.capacity() : 0;

    // return SDIO_GetBlocks();
}

/** End multi-sector transfer and go to idle state.
   * \return true for success or false for failure.
   */
bool RFSDSdio::syncDevice() {
    return true; // read/write wait for end already
}

/**
   * Writes a sector.
   *
   * \param[in] sector Logical sector to be written.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */
bool RFSDSdio::writeSector(uint32_t sector, const uint8_t* src) {
    return SDIOWriteBlock(sector, src);
}

/**
   * Write multiple sectors.
   *
   * \param[in] sector Logical sector to be written.
   * \param[in] ns Number of sectors to be written.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */
bool RFSDSdio::writeSectors(uint32_t sector, const uint8_t* src, size_t ns) {
    while (ns) {
        if (!SDIOWriteBlock(sector, src)) {
            return false;
        }
        sector++;
        ns--;
        src += 512;
    }
    return true;
}

/** CMD6 Switch mode: Check Function Set Function.
   * \param[in] arg CMD6 argument.
   * \param[out] status return status data.
   *
   * \return true for success or false for failure.
   */
bool RFSDSdio::cardCMD6(uint32_t arg, uint8_t* status) {
    return false; // unused externally
}

/** Erase a range of sectors.
   *
   * \param[in] firstSector The address of the first sector in the range.
   * \param[in] lastSector The address of the last sector in the range.
   *
   * \return true for success or false for failure.
   */
bool RFSDSdio::erase(uint32_t firstSector, uint32_t lastSector) {
    return false; // not used
}
/** \return error code. */
uint8_t RFSDSdio::errorCode() const {
    return lastErrorCode;
}
/** \return error data. */
uint32_t RFSDSdio::errorData() const {
    return lastErrorLine;
}
/** \return false by default */
bool RFSDSdio::hasDedicatedSpi() { return false; }
/** \return false by default */
bool RFSDSdio::isDedicatedSpi() { return false; }
/** Set SPI sharing state
   * \param[in] value desired state.
   * \return false by default.
   */
bool RFSDSdio::setDedicatedSpi(bool value) {
    (void)value;
    return false;
}
/**
   * Read a card's CID register.
   *
   * \param[out] cid pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
bool RFSDSdio::readCID(cid_t* cid) {
    return false; // unused by FatLib
}
/**
   * Read a card's CSD register.
   *
   * \param[out] csd pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
bool RFSDSdio::readCSD(csd_t* csd) {
    memcpy(csd, hsd.CSD, 16);
    return true;
}
/** Read OCR register.
   *
   * \param[out] ocr Value of OCR register.
   * \return true for success or false for failure.
   */
bool RFSDSdio::readOCR(uint32_t* ocr) {
    return false; // unused
}
/** Read SCR register.
   *
   * \param[out] scr Value of SCR register.
   * \return true for success or false for failure.
   */
bool RFSDSdio::readSCR(scr_t* scr) {
    return false; // unused
}
/** \return card status. */
uint32_t RFSDSdio::status() { return 0XFFFFFFFF; }
/** Return the card type: SD V1, SD V2 or SDHC/SDXC
   * \return 0 - SD V1, 1 - SD V2, or 3 - SDHC/SDXC.
   */
uint8_t RFSDSdio::type() const {
    switch (hsd.SdCard.CardType) {
    case CARD_SDSC:
        if (hsd.SdCard.CardVersion == CARD_V1_X) {
            return 0;
        }
        return 1;
    case CARD_SDHC_SDXC:
        return 3;
    case CARD_SECURED: // ????
        return 3;
    default:
        return 3;
    }
}
/** Write one data sector in a multiple sector write sequence.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */

bool RFSDSdio::writeData(const uint8_t* src) {
    return false; // unused
}
/** Start a write multiple sectors sequence.
   *
   * \param[in] sector Address of first sector in sequence.
   *
   * \return true for success or false for failure.
   */
bool RFSDSdio::writeStart(uint32_t sector) {
    return false; // unused
}
/** End a write multiple sectors sequence.
   * \return true for success or false for failure.
   */
bool RFSDSdio::writeStop() {
    return false; // unused
}

#endif // HAL_STM32

#endif
