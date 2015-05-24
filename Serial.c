#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */
#include "Serial.h"

#ifdef __DBG_ITM
volatile int32_t ITM_RxBuffer;
#endif

/*-----------------------------------------------------------------------------
 *       SER_Init:  Initialize Serial Interface
 *----------------------------------------------------------------------------*/
void SER_Init (void) {
#ifdef __DBG_ITM
  ITM_RxBuffer = ITM_RXBUFFER_EMPTY;

#else  
  RCC->APB1ENR  |= (1UL << 19);         /* Enable USART4 clock                */
  RCC->APB2ENR  |= (1UL <<  0);         /* Enable AFIO clock                  */
  RCC->AHB1ENR  |= (1UL <<  2);         /* Enable GPIOC clock                 */

  GPIOC->MODER  &= 0xFF0FFFFF;
  GPIOC->MODER  |= 0x00A00000;
  GPIOC->AFR[1] |= 0x00008800;          /* PC10 UART4_Tx, PC11 UART4_Rx (AF8) */

  /* Configure UART4: 115200 baud @ 42MHz, 8 bits, 1 stop bit, no parity      */
  UART4->BRR = (22 << 4) | 12;
  UART4->CR3 = 0x0000;
  UART4->CR2 = 0x0000;
  UART4->CR1 = 0x200C;
#endif
}


/*-----------------------------------------------------------------------------
 *       SER_PutChar:  Write a character to Serial Port
 *----------------------------------------------------------------------------*/
int32_t SER_PutChar (int32_t ch) {
#ifdef __DBG_ITM
  //int i;
  ITM_SendChar (ch & 0xFF);
  //for (i = 10000; i; i--);
#else
  while (!(UART4->SR & 0x0080));
  UART4->DR = (ch & 0xFF);
#endif  

  return (ch);
}


/*-----------------------------------------------------------------------------
 *       SER_GetChar:  Read a character from Serial Port
 *----------------------------------------------------------------------------*/
int32_t SER_GetChar (void) {
#ifdef __DBG_ITM
  if (ITM_CheckChar())
    return ITM_ReceiveChar();
#else
  if (UART4->SR & 0x0020)
    return (UART4->DR);
#endif
  return (-1);
}

/*-----------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
