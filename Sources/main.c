/* ###################################################################
**     Filename    : main.c
**     Project     : UNIVERSAL-BT-PROGRAMMER
**     Organization: PIXELEXPERT Technology and Services PVT LTD.
**     Processor   : S32K118_64
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date        : 2022-05-05
**
**     NOTE : Please generate processor expert code before build
**
**     Contents    :
**         1. This file initialize and configures the following drivers
**         	  GPIO driver, CLOCK driver, FLASH driver and LPUART driver.
**
**         2. At first, it checks the PTC14 pin status for entering into the
**            bootloader mode or else jumping to the user application address.
**
**         3. If enters into the bootloader mode UART receive waiting for the
**            dorman key message to start the upgrade process.
**
**         4.Once the key get verified, upgrading the .bin file from nRF52
**
**         5.After successfully completed jumping to the user application
**
** ###################################################################*/

/* Header files*/
#include "Cpu.h"
#include "pin_mux.h"
#include "dmaController1.h"
#include "clockMan1.h"
#include "Flash.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Defined volatile variable */
volatile int exit_code = 0;

/* Declare a FLASH configure structure which initialized by FlashInit,
 * and will be used by all flash operations */
flash_ssd_config_t flashSSDConfig;

/* Macro Definitions */

/* Timeout in milliseconds for blocking operations */
#define TIMEOUT                    100U

/* Peripheral PTD base pointer */
#define GPIO_PTD                   PTD

/* Peripheral PTC base pointer */
#define GPIO_PTC                   PTC

/** Peripheral PORTD base pointer */
#define GPIO_PORTD                 PORTD

/** Peripheral PORTC base pointer */
#define GPIO_PORTC                 PORTC

/* PORTD's Pin number 15*/
#define PORTD_PIN15                15U

/* PORTC's Pin number 14*/
#define PORTC_PIN14                14U

/* Maximum UART receive buffer size is 128 Bytes*/
#define UART_RX_BUF_SIZE           128

/* Application start address at 0x10000 */
#define APP_START_ADDRESS	       0x10000

typedef void(*user_app_fnptr)(void);

/* Function pointer to the user application.*/
user_app_fnptr      *p_jump_to_app;

/* Global Variables */
bool g_bln_start_upgrade = false;

uint8_t g_uart_rx_buf[UART_RX_BUF_SIZE];
uint8_t g_flash_buf[FEATURE_FLS_PF_BLOCK_SECTOR_SIZE];

uint16_t g_buf_offset = 0;
uint16_t g_app_size = 0;
uint16_t g_total_count = 0;

uint32_t g_start_addr = APP_START_ADDRESS;

/* Function Prototypes*/
static void clock_init(void);
static void gpio_init(void);
static void flash_init(void);
static void lupart_init(void);
static void jump_to_user_application(void);

/******************************************************************************
**     Function Name    : clock_init
**     Description		: Install pre-defined clock configurations.
**                        This function installs the pre-defined clock configuration
**     Arguments		: void
**     Return Value		: void
*******************************************************************************/
static void clock_init(void)
{
	/* Clock configured at,
	 * SIRC_CLK - 8.0 MHz (slow internal reference clock)
	 * FIRC_CLK - 48.0 MHz (fast internal reference clock)
	 * SOSC_CLK - 16000000 (system oscillator clock)
	 * LPO_CLK  - 128 kHz (low power oscillator)
	 */
	CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
				 g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);

	/* Set clock transition policy as clock transfers gracefully */
	CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);
}


/******************************************************************************
**     Function Name    : gpio_init
**     Description      : This function configures the pins with the options provided
**                        in the given structure.
**     Arguments		: void
**     Return Value		: void
*******************************************************************************/
static void gpio_init(void)
{
	/* Initialize the PINS*/
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    /* Output direction for LEDs */
    PINS_DRV_SetPinsDirection(GPIO_PTD, (1 << PORTD_PIN15));

    /* Setup button pin */
    PINS_DRV_SetPinsDirection(GPIO_PTC, ~(1 << PORTC_PIN14));

    /* Setup button pins interrupt */
    PINS_DRV_SetPinIntSel(GPIO_PORTC, PORTC_PIN14, PORT_INT_RISING_EDGE);
}


/******************************************************************************
**     Function Name    : flash_init
**     Description      : Initializes Flash module by reporting the memory configuration via
**                        SSD configuration structure.
**     Arguments		: void
**     Return Value		: void
*******************************************************************************/
static void flash_init(void)
{
	/*Initialize the flash module*/

	/*	Flash_InitConfig0.PFlashBase  = 0x00000000U,
	 *	Flash_InitConfig0.PFlashSize  = 0x00040000U,
	 *	Flash_InitConfig0.DFlashBase  = 0x10000000U,
	 *	Flash_InitConfig0.EERAMBase   = 0x14000000U,
	 *	Flash_InitConfig0.CallBack    = NULL_CALLBACK
	 */
	status_t status = FLASH_DRV_Init(&Flash_InitConfig0, &flashSSDConfig);

	/* Checking error code status */
	DEV_ASSERT(STATUS_SUCCESS == status);
}


/******************************************************************************
**     Function Name    : lupart_init
**     Description      : This function initializes a LPUART instance for operation.
**                        This function will initialize the run-time state
**                        structure to keep track of the on-going transfers,
**                        ungate the clock to the LPUART module, initialize the
**                        module to user defined settings and default settings,
**                        configure the IRQ state structure and enable the
**                        module-level interrupt to the core, and enable the
**                        LPUART module transmitter and receiver.
**     Arguments		: void
**     Return Value		: void
*******************************************************************************/
static void lupart_init(void)
{
	/* Initialize LPUART instance */

	/*    lpuart1_InitConfig0.baudRate = 115200U;
	 *    lpuart1_InitConfig0.bitCountPerChar = LPUART_8_BITS_PER_CHAR;
	 *    lpuart1_InitConfig0.parityMode = LPUART_PARITY_DISABLED;
	 *    lpuart1_InitConfig0.stopBitCount = LPUART_ONE_STOP_BIT;
	 *    lpuart1_InitConfig0.transferType = LPUART_USING_INTERRUPTS;
	 */
	LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);
}


/******************************************************************************
**     Function Name    : delay
**     Description		: Provide some required time delay
**     Arguments		: (volatile int) cycles
**     Return Value		: void
*******************************************************************************/
void delay(volatile int cycles)
{
    /* Delay function - do nothing for a number of cycles */
    while(cycles--);
}


/******************************************************************************
**     Function Name    : jump_to_user_application
**     Description      : This function update the vector table to the
**                        reset vector of the application address.
**                        Jumping to the application address.
**     Arguments		: void
**     Return Value		: void
*******************************************************************************/
static void jump_to_user_application(void)
{
	/* Check if Entry address is erased and return if erased */
	if(*(uint32_t*)APP_START_ADDRESS == 0xFFFFFFFF ||
			*(uint32_t*)APP_START_ADDRESS == 0x00000000)
		return;

	/* Disable interrupts */
	__asm("cpsid i");

	/* Point to the start reset vector for the  application.*/
	p_jump_to_app = (void *) APP_START_ADDRESS + 4;

	/* Update the vector table to the reset vector of the application.*/
	S32_SCB->VTOR = (uint32_t)APP_START_ADDRESS;

	/* Set stack for the application */
	__asm("msr msp, r0");

	/* Jump to application */
	(*p_jump_to_app)();
}

int main(void)
{
	/* Local variable */
	uint32_t l_bytesRemaining;
	uint32_t l_pin_status;
	status_t l_status;

	/*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
	#ifdef PEX_RTOS_INIT
	PEX_RTOS_INIT(); /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
	#endif
	/*** End of Processor Expert internal initialization.                    ***/

	/* clock initialization */
	clock_init();

	/* gpio initialization */
	gpio_init();

	/* flash initialization */
	flash_init();

	/* lupart initialization */
	lupart_init();

	/* Reading the PTC14 status to enter into the bootloader mode */
	l_pin_status = ((PINS_DRV_ReadPins(GPIO_PTC)) >> PORTC_PIN14) & 0x01;

	/* If the pin get SET means enter the bootloader mode or else
	 * directly jumping to the application*/
	if(!l_pin_status)
	{
		/* Jumping to the uploaded application at starting address 0x10000 */
		jump_to_user_application();

		/* Clear the pin interrupt status flag */
		PINS_DRV_ClearPinIntFlagCmd(GPIO_PORTC, PORTC_PIN14);
	}

	/* LED Indication to defining the code enters into bootloader mode */
	/* Set the PTD15 to output high */
	PINS_DRV_SetPins(GPIO_PTD, (1 << PORTD_PIN15));

	/* wait for some delay */
	delay(10000);

	/* Set the PTD15 to output low */
	PINS_DRV_ClearPins(GPIO_PTD, (1 << PORTD_PIN15));

	/* Assigning the application start address */
	g_start_addr = APP_START_ADDRESS;

	/* RESET the start upgrade flag */
	g_bln_start_upgrade = false;

	/*Clearing the buffer before filling the data*/
	memset(g_flash_buf, 0, FEATURE_FLS_PF_BLOCK_SECTOR_SIZE);

	while(1)
	{
	  if(!g_bln_start_upgrade)
	  {
		/* clearing the uart receive buffer*/
		memset(g_uart_rx_buf, 0, 8);

		/* Receives the data via UART*/
		LPUART_DRV_ReceiveData(INST_LPUART1, g_uart_rx_buf, 8);

		/* Wait for transfer to be completed */
		while(LPUART_DRV_GetReceiveStatus(INST_LPUART1, &l_bytesRemaining) == STATUS_BUSY);

		/* Check the passkey "DORMAN" to receive the upgrading file info */
		if(memcmp(g_uart_rx_buf,"DORMAN",6) == 0)
		{
		 /* Storing MSB of the application size */
		 g_app_size = g_uart_rx_buf[6] << 8 & 0xFF00;

		 /* Storing LSB of the application size */
		 g_app_size |= g_uart_rx_buf[7] & 0x00FF;

		 if(g_app_size > 0)
		 {
			 /* Send an ack message(0xAA) to notify the start upgrade process */
			 uint8_t ackmsg = 0xAA;
			 LPUART_DRV_SendData(INST_LPUART1, &ackmsg, 1);

			 /* SET the start upgrade flag*/
			 g_bln_start_upgrade = true;
		 }
		}
	  }
	  else
	  {
        /* Clearing the uart receiver buffer */
		memset(g_uart_rx_buf, 0, UART_RX_BUF_SIZE);

		/* UART Receives the 128 bytes of data */
		LPUART_DRV_ReceiveData(INST_LPUART1, g_uart_rx_buf, UART_RX_BUF_SIZE);

		/* Wait for transfer to be completed */
		while(LPUART_DRV_GetReceiveStatus(INST_LPUART1, &l_bytesRemaining) == STATUS_BUSY);

		/* Copying the uart received data */
		memcpy(g_flash_buf+g_buf_offset, g_uart_rx_buf, UART_RX_BUF_SIZE);

        /* Incrementing the flash buffer pointer by UART_RX_BUF_SIZE */
		g_buf_offset += UART_RX_BUF_SIZE;

		if((g_buf_offset == FEATURE_FLS_PF_BLOCK_SECTOR_SIZE))
		{
		  /* Erase the last PFlash sector */
		  l_status = FLASH_DRV_EraseSector(&flashSSDConfig, g_start_addr,
				                            FEATURE_FLS_PF_BLOCK_SECTOR_SIZE);

		  /* Checking error code status */
		  DEV_ASSERT(STATUS_SUCCESS == l_status);

		  /* Verify the erase operation at margin level value of 1, user read */
		  l_status = FLASH_DRV_VerifySection(&flashSSDConfig, g_start_addr,
				     FEATURE_FLS_PF_BLOCK_SECTOR_SIZE / FTFx_DPHRASE_SIZE, 1u);

		  /* Checking error code status */
		  DEV_ASSERT(STATUS_SUCCESS == l_status);

		  /* Incrementing the total count by flash buffer pointer */
		  g_total_count += g_buf_offset;

		  /* Reset the flash buffer pointer */
		  g_buf_offset = 0;

		  /* Disable Callback */
		  flashSSDConfig.CallBack = NULL_CALLBACK;

		  /* Write flash buffer data to the erased PFlash sector */
		  l_status = FLASH_DRV_Program(&flashSSDConfig, g_start_addr,
				     FEATURE_FLS_PF_BLOCK_SECTOR_SIZE, g_flash_buf);

		  /* Checking error code status */
		  DEV_ASSERT(STATUS_SUCCESS == l_status);

		  /* Move the address pointer to next block by incrementing
		   * the application start address with FEATURE_FLS_PF_BLOCK_SECTOR_SIZE */
		  g_start_addr += FEATURE_FLS_PF_BLOCK_SECTOR_SIZE;

		  /* Clearing the flash buffer after successful flash write */
		  memset(g_flash_buf, 0, FEATURE_FLS_DF_BLOCK_SECTOR_SIZE);

		  if(g_total_count >= g_app_size)
		  {
			 /* If total counter matches the total application size,
			  * then transmit the value "0xCC" to stop the upgrade */
			 uint8_t ackmsg = 0xCC;
			 LPUART_DRV_SendData(INST_LPUART1, &ackmsg, 1);//initiate the send ACK

			 /* wait for some delay */
			 delay(1000);

			 /* Jumping to the uploaded application at starting address 0x10000 */
			 jump_to_user_application();
		  }
		}

		 /* After filling the 128 bytes of data to the flash buffer and
		  * send an ack message(0xBB) for getting the next set of data*/
		 uint8_t ackmsg = 0xBB;
		 LPUART_DRV_SendData(INST_LPUART1, &ackmsg, 1);
	  }
	}

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
