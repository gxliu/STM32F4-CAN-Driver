#include <stdio.h>
#include <stm32f4xx.h>
#include "Serial.h"
#include "CAN.h"
#include "LED.h"
#include "stm32f4xx_hal.h"

unsigned int val_Tx = 0, val_Rx = 0;              /* Globals used for display */

volatile uint32_t msTicks;                        /* counts 1ms timeTicks     */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in Delay() */
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;
  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

/*----------------------------------------------------------------------------
  display transmit and receive values
 *---------------------------------------------------------------------------*/
void val_display (void) {

  LED_Out (val_Rx);                               /* display RX val on LEDs  */
  printf ("Tx: 0x%02X, Rx: 0x%02X\r\n", val_Tx, val_Rx); // send out Printf Viewer
  Delay (10);                                     /* delay for 10ms           */
}

/* System Clock Configuration */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}


/*----------------------------------------------------------------------------
  initialize CAN interface
 *----------------------------------------------------------------------------*/
void can_Init (void) {

  CAN_setup (1);                                  /* setup CAN Controller #1  */
  CAN_setup (2);                                  /* setup CAN Controller #2  */
  CAN_wrFilter (1, 33, STANDARD_FORMAT);          /* Enable reception of msgs */
  CAN_start (1);                                  /* start CAN Controller #1  */
  CAN_start (2);                                  /* start CAN Controller #2  */
  CAN_waitReady (1);                              /* wait til tx mbx is empty */
  CAN_waitReady (2);                              /* wait til tx mbx is empty */
}



/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void)  {
  int i;
SystemClock_Config();
  LED_Init ();                                    /* initialize the LEDs      */


  SystemCoreClockUpdate();                        /* Get Core Clock Frequency */
  SysTick_Config(SystemCoreClock /1000);         /* SysTick 1 msec irq       */
	can_Init ();                                    /* initialize CAN interface */

  CAN_TxMsg[1].id = 33;                           /* initialize msg to send   */
  for (i = 0; i < 8; i++) CAN_TxMsg[0].data[i] = 0;
  CAN_TxMsg[1].len = 1;
  CAN_TxMsg[1].format = STANDARD_FORMAT;
	CAN_TxMsg[1].type = DATA_FRAME;

  while (1) {

    val_Tx = (val_Tx + 1) % 15;

    if (CAN_TxRdy[1]) {                           /* tx msg on CAN Ctrl #2    */
      CAN_TxRdy[1] = 0;

      CAN_TxMsg[1].data[0] = val_Tx;              /* data[0] = ADC value      */
			for (i = 1; i < 8; i++) CAN_TxMsg[1].data[i] = 0x77;
      CAN_wrMsg (2, &CAN_TxMsg[1]);               /* transmit message         */
    }

    Delay (10);                                   /* delay for 10ms           */

    if (CAN_RxRdy[0]) {                           /* rx msg on CAN Ctrl #1    */
      CAN_RxRdy[0] = 0;

      val_Rx = CAN_RxMsg[0].data[0];
    }

    val_display ();                               /* display TX and RX values */
    Delay (500);                                  /* delay for 500ms          */

  }
}
