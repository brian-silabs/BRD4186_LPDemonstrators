/***************************************************************************//**
 * @file main_scan_letimer_prs_ldma.c
 * @brief Use the IADC to take repeated nonblocking measurements on two external
 * inputs which in turn triggers the LDMA to transfer the IADC measurement to
 * memory, all while remaining in EM2. IADC conversion is requested periodically
 * by LETIMER via PRS, also running in EM2. After NUM_SAMPLES conversions the
 * LDMA will trigger an interrupt from EM2 and toggle LED0 on the WSTK.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *******************************************************************************
 * # Evaluation Quality
 * This code has been minimally tested to ensure that it builds and is suitable 
 * as a demonstration for evaluation purposes only. This code will be maintained
 * at the sole discretion of Silicon Labs.
 ******************************************************************************/

#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_core.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_iadc.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_ldma.h"
#include "em_letimer.h"

#include "mx25flash_spi.h"
#include "bspconfig.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#define POWER_DOWN_RAM  (0)//Set to 0 until all EM2 buffers in RAM are not linked to 16kB

// Set CLK_ADC to 10MHz
#define CLK_SRC_ADC_FREQ          20000000 // CLK_SRC_ADC

// Takes Errata IADC_E306 into account
#define CLK_ADC_FREQ_GAIN_4X      2500000 // CLK_ADC - 2.5MHz max in gain 4x
#define CLK_ADC_FREQ_GAIN_0P5X    10000000 // CLK_ADC - 10MHz max in 0.5x gain

// Number of scan channels
#define NUM_INPUTS 6

/*
 * Specify the IADC input using the IADC_PosInput_t typedef.  This
 * must be paired with a corresponding macro definition that allocates
 * the corresponding ABUS to the IADC.  These are...
 *
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AEVEN0_ADC0
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BEVEN0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BODD0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDEVEN0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDODD0_ADC0
 *
 * ...for port A, port B, and port C/D pins, even and odd, respectively.
 */
#define IADC_INPUT_0_PORT_PIN     iadcPosInputPortBPin0;

#define IADC_INPUT_1_PORT_PIN     iadcPosInputPortBPin1;

#define IADC_INPUT_2_PORT_PIN     iadcPosInputPortBPin2;

#define IADC_POS_INPUT_3_PORT_PIN     iadcPosInputPortAPin0;
#define IADC_NEG_INPUT_3_PORT_PIN     iadcPosInputPortAPin3;

#define IADC_POS_INPUT_4_PORT_PIN     iadcPosInputPortAPin4;
#define IADC_NEG_INPUT_4_PORT_PIN     iadcPosInputPortAPin7;

#define IADC_POS_INPUT_5_PORT_PIN     iadcPosInputPortAPin8;
#define IADC_NEG_INPUT_5_PORT_PIN     iadcPosInputPortAPin9;


#define IADC_INPUT_0_BUS              BBUSALLOC
#define IADC_INPUT_0_BUSALLOC         GPIO_BBUSALLOC_BEVEN0_ADC0

#define IADC_INPUT_1_BUS              BBUSALLOC
#define IADC_INPUT_1_BUSALLOC         GPIO_BBUSALLOC_BODD0_ADC0

#define IADC_INPUT_2_BUS              BBUSALLOC
#define IADC_INPUT_2_BUSALLOC         GPIO_BBUSALLOC_BEVEN0_ADC0


#define IADC_POS_INPUT_3_BUS          ABUSALLOC
#define IADC_POS_INPUT_3_BUSALLOC     GPIO_ABUSALLOC_AEVEN0_ADC0

#define IADC_NEG_INPUT_3_BUS          ABUSALLOC
#define IADC_NEG_INPUT_3_BUSALLOC     GPIO_ABUSALLOC_AODD0_ADC0

#define IADC_POS_INPUT_4_BUS          ABUSALLOC
#define IADC_POS_INPUT_4_BUSALLOC     GPIO_ABUSALLOC_AEVEN0_ADC0

#define IADC_NEG_INPUT_4_BUS          ABUSALLOC
#define IADC_NEG_INPUT_4_BUSALLOC     GPIO_ABUSALLOC_AODD0_ADC0

#define IADC_POS_INPUT_5_BUS          ABUSALLOC
#define IADC_POS_INPUT_5_BUSALLOC     GPIO_ABUSALLOC_AEVEN0_ADC0

#define IADC_NEG_INPUT_5_BUS          ABUSALLOC
#define IADC_NEG_INPUT_5_BUSALLOC     GPIO_ABUSALLOC_AODD0_ADC0


// Desired LETIMER frequency in Hz
#define LETIMER_FREQ              1
#define LETIMER_SAMPLING_TICK     14u

// Use specified LDMA/PRS channel
#define IADC_LDMA_CH              0
#define PRS_CHANNEL               0

// How many samples to capture
#define NUM_SAMPLES               64

/*******************************************************************************
 ***************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/

/// Globally declared LDMA link descriptor
LDMA_Descriptor_t descriptor;

// buffer to store IADC samples
uint32_t scanBuffer[NUM_SAMPLES] = {0xFF};

/**************************************************************************//**
 * @brief  PRS Initializer
 *****************************************************************************/
void initPRS (void)
{
  // Use LETIMER0 as async PRS to trigger IADC in EM2
  CMU_ClockEnable(cmuClock_PRS, true);

  /* Set up PRS LETIMER and IADC as producer and consumer respectively */
  PRS_SourceAsyncSignalSet(PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_LETIMER0, PRS_LETIMER0_CH0);
  PRS_ConnectConsumer(PRS_CHANNEL, prsTypeAsync, prsConsumerIADC0_SCANTRIGGER);
}

/**************************************************************************//**
 * @brief  IADC Initializer
 *****************************************************************************/
void initIADC (void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;
  IADC_ScanTable_t initScanTable = IADC_SCANTABLE_DEFAULT; // Scan Table

  // Enable IADC0 clock branch
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  // Configure IADC clock source for use while in EM2
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);  // FSRCO - 20MHz

  //init.iadcClkSuspend1 = true;//Turn off clocks between single acquisitions
  init.iadcClkSuspend0 = true;//Turn off clocks between scan acquisitions

  // Modify init structs and initialize
  init.warmup = iadcWarmupNormal;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  /*
   * Configuration 0 is used by both scan and single conversions by
   * default.  Use internal bandgap as the reference and specify the
   * reference voltage in mV.
   *
   * Resolution is not configurable directly but is based on the
   * selected oversampling ratio (osrHighSpeed), which defaults to
   * 2x and generates 12-bit results.
   */
  //I measurements
  initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
  initAllConfigs.configs[0].vRef = 1210;

  initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed32x;
  initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain4x;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency
  // Default oversampling (OSR) is 2x, and Conversion Time = ((4 * OSR) + 2) / fCLK_ADC
  // Combined with the 2 cycle delay when switching input channels, total sample rate is 833ksps
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                     CLK_ADC_FREQ_GAIN_4X,
                                                                     0,
                                                                     iadcCfgModeNormal,
                                                                     init.srcClkPrescale);



  //U measurements
  initAllConfigs.configs[1].reference = iadcCfgReferenceInt1V2;
  initAllConfigs.configs[1].vRef = 1210;

  initAllConfigs.configs[1].osrHighSpeed = iadcCfgOsrHighSpeed2x;
  initAllConfigs.configs[1].analogGain = iadcCfgAnalogGain0P5x;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency
  // Default oversampling (OSR) is 2x, and Conversion Time = ((4 * OSR) + 2) / fCLK_ADC
  // Combined with the 2 cycle delay when switching input channels, total sample rate is 833ksps
  initAllConfigs.configs[1].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                     CLK_ADC_FREQ_GAIN_0P5X,
                                                                     0,
                                                                     iadcCfgModeNormal,
                                                                     init.srcClkPrescale);


  // Scan initialization
  initScan.triggerSelect = iadcTriggerSelPrs0PosEdge;
  initScan.dataValidLevel = iadcFifoCfgDvl4;
  initScan.alignment = iadcAlignLeft16;

  // Enable triggering of scan conversion
  initScan.start = true;

  // Set to run in EM2
  initScan.fifoDmaWakeup = true;

  // Configure entries in scan table
  // Takes Errata IADC_E306 into account
  initScanTable.entries[0].posInput = IADC_POS_INPUT_3_PORT_PIN;
  initScanTable.entries[0].negInput = IADC_NEG_INPUT_3_PORT_PIN;
  initScanTable.entries[0].includeInScan = true;
  initScanTable.entries[0].configId = 1;

  initScanTable.entries[1].posInput = IADC_INPUT_0_PORT_PIN;
  initScanTable.entries[1].negInput = iadcNegInputGnd;
  initScanTable.entries[1].includeInScan = true;
  initScanTable.entries[1].configId = 0;

  initScanTable.entries[2].posInput = IADC_INPUT_1_PORT_PIN;
  initScanTable.entries[2].negInput = iadcNegInputGnd;
  initScanTable.entries[2].includeInScan = true;
  initScanTable.entries[2].configId = 1;

  initScanTable.entries[3].posInput = IADC_POS_INPUT_4_PORT_PIN;
  initScanTable.entries[3].negInput = IADC_NEG_INPUT_4_PORT_PIN;
  initScanTable.entries[3].includeInScan = true;
  initScanTable.entries[3].configId = 0;

  initScanTable.entries[4].posInput = IADC_INPUT_2_PORT_PIN;
  initScanTable.entries[4].negInput = iadcNegInputGnd;
  initScanTable.entries[4].includeInScan = true;
  initScanTable.entries[4].configId = 1;

  initScanTable.entries[5].posInput = IADC_POS_INPUT_5_PORT_PIN;
  initScanTable.entries[5].negInput = IADC_NEG_INPUT_5_PORT_PIN;
  initScanTable.entries[5].includeInScan = true;
  initScanTable.entries[5].configId = 0;

  //TODO would this be skippabe by LDMA
  //initScanTable.entries[6].posInput = IADC_POS_INPUT_5_PORT_PIN;
  //initScanTable.entries[6].negInput = IADC_NEG_INPUT_5_PORT_PIN;
  //initScanTable.entries[6].includeInScan = true;
  //initScanTable.entries[6].configId = 1;
  
  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize Scan
  IADC_initScan(IADC0, &initScan, &initScanTable);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
  GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;
  GPIO->IADC_INPUT_2_BUS |= IADC_INPUT_2_BUSALLOC;

  GPIO->IADC_POS_INPUT_3_BUS |= IADC_POS_INPUT_3_BUSALLOC;
  GPIO->IADC_NEG_INPUT_3_BUS |= IADC_NEG_INPUT_3_BUSALLOC;

  GPIO->IADC_POS_INPUT_3_BUS |= IADC_POS_INPUT_3_BUSALLOC;
  GPIO->IADC_NEG_INPUT_3_BUS |= IADC_NEG_INPUT_3_BUSALLOC;

  GPIO->IADC_POS_INPUT_3_BUS |= IADC_POS_INPUT_3_BUSALLOC;
  GPIO->IADC_NEG_INPUT_3_BUS |= IADC_NEG_INPUT_3_BUSALLOC;

}

/**************************************************************************//**
 * @brief Clock initialization
 *****************************************************************************/
void initClock(void)
{
  CMU_LFXOInit_TypeDef lfxoInit = CMU_LFXOINIT_DEFAULT;

  // Select LFXO for the LETIMER
  CMU_LFXOInit(&lfxoInit);
  CMU_ClockSelectSet(cmuClock_EM23GRPACLK, cmuSelect_LFXO);
}

/**************************************************************************//**
 * @brief LETIMER initialization
 *****************************************************************************/
void initLetimer(void)
{
  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

  // Enable LETIMER0 clock tree
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  // Calculate the top value (frequency) based on clock source
  //uint32_t topValue = CMU_ClockFreqGet(cmuClock_LETIMER0) / LETIMER_FREQ;
  uint32_t topValue = LETIMER_SAMPLING_TICK;

  // Reload top on underflow, pulse output, and run in free mode
  letimerInit.comp0Top = true;
  letimerInit.topValue = topValue;
  letimerInit.ufoa0 = letimerUFOAPulse;
  letimerInit.repMode = letimerRepeatFree;

  // Initialize LETIMER
  LETIMER_Init(LETIMER0, &letimerInit);
}

/**************************************************************************//**
 * @brief
 *   LDMA Initializer
 *
 * @param[in] buffer
 *   pointer to the array where ADC data will be stored.
 * @param[in] size
 *   size of the array
 *****************************************************************************/
void initLDMA(uint32_t *buffer, uint32_t size)
{
  LDMA_Init_t init = LDMA_INIT_DEFAULT;

  // Configure LDMA for transfer from IADC to memory
  // LDMA will loop continuously
  LDMA_TransferCfg_t transferCfg =
    LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SCAN);

  // Set up descriptors for dual buffer transfer
  descriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&IADC0->SCANFIFODATA, buffer, size, 0);

  // Loop of NUM_SAMPLES, run continuously
  descriptor.xfer.decLoopCnt = 0;
  descriptor.xfer.xferCnt = NUM_SAMPLES - 1; // 1 less than desired transfer count

  // Interrupt upon transfer complete
  descriptor.xfer.doneIfs = 1;
  descriptor.xfer.ignoreSrec = 1;
  descriptor.xfer.blockSize = ldmaCtrlBlockSizeUnit4;

  // Initialize LDMA with default configuration
  LDMA_Init(&init);

  // Start transfer, LDMA will sample the IADC NUM_SAMPLES time, and then interrupt
  LDMA_StartTransfer(IADC_LDMA_CH, &transferCfg, &descriptor);
}

/**************************************************************************//**
 * @brief  LDMA Handler
 *****************************************************************************/
void LDMA_IRQHandler(void)
{
  // Clear interrupt flags
  LDMA_IntClear(LDMA_IF_DONE0);

}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  FlashStatus status;

  CHIP_Init();

  // Turn on DCDC regulator
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_WSTK_DEFAULT;
  dcdcInit.mode = emuDcdcMode_Bypass;
  EMU_DCDCInit(&dcdcInit);

  /*
   * When developing or debugging code that enters EM2 or
   *  lower, it's a good idea to have an "escape hatch" type
   * mechanism, e.g. a way to pause the device so that a debugger can
   * connect in order to erase flash, among other things.
   *
   * Before proceeding with this example, make sure PB0 is not pressed.
   * If the PB0 pin is low, turn on LED0 and execute the breakpoint
   * instruction to stop the processor in EM0 and allow a debug
   * connection to be made.
   */
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPullFilter, 1);
  if (GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN) == 0) {
    GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 1);
    __BKPT(0);
  }
  // Pin not asserted, so disable input
  else {
    GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeDisabled, 0);
    CMU_ClockEnable(cmuClock_GPIO, false);
  }

  // Enable voltage downscaling in EM mode 2(VSCALE0)
  EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;
  em23Init.vScaleEM23Voltage = emuVScaleEM23_LowPower;

  // Initialize EM23 energy modes
  EMU_EM23Init(&em23Init);

  // Init and power-down MX25 SPI flash
  MX25_init();
  MX25_RSTEN();
  MX25_RST(&status);
  MX25_DP();
  MX25_deinit();

  // Initialize PRS
  initPRS();

  // Initialize the IADC
  initIADC();

  // Initialize LDMA
  initLDMA(scanBuffer, NUM_SAMPLES);

  // Initialize LFXO
  initClock();

  // Initialize the LETIMER
  initLetimer();

  // Power down all RAM blocks except block 0
  if (POWER_DOWN_RAM) {

    /* Disable Radio RAM memories (FRC and SEQ) */
    CMU_ClockEnable(cmuClock_SYSCFG, true);
    SYSCFG->RADIORAMRETNCTRL = 0x103UL;

    EMU_RamPowerDown(SRAM_BASE, 0);//0 means all extinguishable RAM
  }

  while (1)
  {
    // Enter EM2 sleep
    EMU_EnterEM2(true);
  }
}
