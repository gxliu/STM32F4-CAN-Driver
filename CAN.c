#include <stm32f4xx.h>
#include "CAN.h"

/* CAN identifier type */
#define CAN_ID_STD            ((uint32_t)0x00000000)  /* Standard Id          */
#define CAN_ID_EXT            ((uint32_t)0x00000004)  /* Extended Id          */

/* CAN remote transmission request */
#define CAN_RTR_DATA          ((uint32_t)0x00000000)  /* Data frame           */
#define CAN_RTR_REMOTE        ((uint32_t)0x00000002)  /* Remote frame         */

CAN_msg       CAN_TxMsg[2];                      /* CAN message for sending */
CAN_msg       CAN_RxMsg[2];                      /* CAN message for receiving */                                

uint32_t      CAN_TxRdy[2] = {0,0};              /* CAN HW ready to transmit a message */
uint32_t      CAN_RxRdy[2] = {0,0};              /* CAN HW received a message */

static uint32_t CAN_filterIdx[2] = {0,0};        /* static variable for the filter index */


/*----------------------------------------------------------------------------
  setup CAN interface
 *----------------------------------------------------------------------------*/
void CAN_setup (uint32_t ctrl)  {
  CAN_TypeDef *pCAN = (ctrl == 1) ? CAN1 : CAN2;
  uint32_t brp;

  if (ctrl == 1) {
    /* Enable clock for CAN1 and GPIOB */
    RCC->APB1ENR   |= (1 << 25);
    RCC->AHB1ENR   |= (1 <<  1);
    /* CAN1, we use PB8, PB9 */
    GPIOB->MODER   &= ~(( 3 << ( 8*2)) | ( 3 << ( 9*2)));
    GPIOB->MODER   |=  (( 2 << ( 8*2)) | ( 2 << ( 9*2)));
    GPIOB->OTYPER  &= ~(( 1 <<   8   ) | ( 1 <<   9   ));
    GPIOB->OSPEEDR &= ~(( 3 << ( 8*2)) | ( 3 << ( 9*2)));
    GPIOB->PUPDR   &= ~(( 3 << ( 8*2)) | ( 3 << ( 9*2)));
    GPIOB->AFR[1]  &= ~((15 << ( 0*4)) | (15 << ( 1*4)));
    GPIOB->AFR[1]  |=  (( 9 << ( 0*4)) | ( 9 << ( 1*4)));
  
    NVIC_EnableIRQ   (CAN1_TX_IRQn);         /* Enable CAN1 interrupts */
    NVIC_EnableIRQ   (CAN1_RX0_IRQn);
  } else {
    /* Enable clock for CAN2 and GPIOB */
    RCC->APB1ENR   |= (1 << 25) | (1 << 26);
    RCC->AHB1ENR   |= (1 <<  1);
    /* CAN2, we use PB5, PB6 */
    GPIOB->MODER   &= ~(( 3 << ( 5*2)) | ( 3 << ( 6*2)));
    GPIOB->MODER   |=  (( 2 << ( 5*2)) | ( 2 << ( 6*2)));
    GPIOB->OTYPER  &= ~(( 1 <<   5   ) | ( 1 <<   6   ));
    GPIOB->OSPEEDR &= ~(( 3 << ( 5*2)) | ( 3 << ( 6*2)));
    GPIOB->PUPDR   &= ~(( 3 << ( 5*2)) | ( 3 << ( 6*2)));
    GPIOB->AFR[0]  &= ~((15 << ( 5*4)) | (15 << ( 6*4)));
    GPIOB->AFR[0]  |=  (( 9 << ( 5*4)) | ( 9 << ( 6*4)));

    NVIC_EnableIRQ   (CAN2_TX_IRQn);         /* Enable CAN2 interrupts */
    NVIC_EnableIRQ   (CAN2_RX0_IRQn);
  }

  pCAN->MCR = (CAN_MCR_INRQ   |           /* initialisation request           */
               CAN_MCR_NART    );         /* no automatic retransmission      */
                                          /* only FIFO 0, tx mailbox 0 used!  */
  while (!(pCAN->MSR & CAN_MCR_INRQ));

  pCAN->IER = (CAN_IER_FMPIE0 |           /* enable FIFO 0 msg pending IRQ    */
               CAN_IER_TMEIE    );        /* enable Transmit mbx empty IRQ    */

  /* Note: this calculations fit for CAN (APB1) clock = 42MHz */
  brp  = (42000000 / 7) / 500000;         /* baudrate is set to 500k bit/s    */
                                                                          
  /* set BTR register so that sample point is at about 71% bit time from bit start */
  /* TSEG1 = 4, TSEG2 = 2, SJW = 3 => 1 CAN bit = 7 TQ, sample at 71%      */
  pCAN->BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((        0x0F) << 16) | (          0x3FF));
  pCAN->BTR |=  ((((3-1) & 0x03) << 24) | (((2-1) & 0x07) << 20) | (((4-1) & 0x0F) << 16) | ((brp-1) & 0x3FF));
}


/*----------------------------------------------------------------------------
  leave initialisation mode
 *----------------------------------------------------------------------------*/
void CAN_start (uint32_t ctrl)  {
  CAN_TypeDef *pCAN = (ctrl == 1) ? CAN1 : CAN2;

  pCAN->MCR &= ~CAN_MCR_INRQ;             /* normal operating mode, reset INRQ*/
#ifndef __TEST
  while (pCAN->MSR & CAN_MCR_INRQ);
#endif
}

/*----------------------------------------------------------------------------
  set the testmode
 *----------------------------------------------------------------------------*/
void CAN_testmode (uint32_t ctrl, uint32_t testmode) {
  CAN_TypeDef *pCAN = (ctrl == 1) ? CAN1 : CAN2;

  pCAN->BTR &= ~(CAN_BTR_SILM | CAN_BTR_LBKM);     /* set testmode            */
  pCAN->BTR |=  (testmode & (CAN_BTR_SILM | CAN_BTR_LBKM));
}

/*----------------------------------------------------------------------------
  check if transmit mailbox is empty
 *----------------------------------------------------------------------------*/
void CAN_waitReady (uint32_t ctrl)  {
  CAN_TypeDef *pCAN = (ctrl == 1) ? CAN1 : CAN2;

  while ((pCAN->TSR & CAN_TSR_TME0) == 0);  /* Transmit mailbox 0 is empty    */
  CAN_TxRdy[ctrl-1] = 1;
}

/*----------------------------------------------------------------------------
  wite a message to CAN peripheral and transmit it
 *----------------------------------------------------------------------------*/
void CAN_wrMsg (uint32_t ctrl, CAN_msg *msg)  {
  CAN_TypeDef *pCAN = (ctrl == 1) ? CAN1 : CAN2;

  pCAN->sTxMailBox[0].TIR  = (uint32_t)0; /* reset TXRQ bit */
                                          /* Setup identifier information */
  if (msg->format == STANDARD_FORMAT) {   /*    Standard ID                   */
    pCAN->sTxMailBox[0].TIR |= (uint32_t)(msg->id << 21) | CAN_ID_STD;
  } else {                                /* Extended ID                      */
    pCAN->sTxMailBox[0].TIR |= (uint32_t)(msg->id <<  3) | CAN_ID_EXT;
  }

                                          /* Setup type information           */
  if (msg->type == DATA_FRAME)  {         /* DATA FRAME                       */
    pCAN->sTxMailBox[0].TIR |= CAN_RTR_DATA;
  } else {                                /* REMOTE FRAME                     */
    pCAN->sTxMailBox[0].TIR |= CAN_RTR_REMOTE;
  }
                                          /* Setup data bytes                 */
  pCAN->sTxMailBox[0].TDLR = (((uint32_t)msg->data[3] << 24) | 
                              ((uint32_t)msg->data[2] << 16) |
                              ((uint32_t)msg->data[1] <<  8) | 
                              ((uint32_t)msg->data[0])        );
  pCAN->sTxMailBox[0].TDHR = (((uint32_t)msg->data[7] << 24) | 
                              ((uint32_t)msg->data[6] << 16) |
                              ((uint32_t)msg->data[5] <<  8) |
                              ((uint32_t)msg->data[4])        );
                                          /* Setup length                     */
  pCAN->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
  pCAN->sTxMailBox[0].TDTR |=  (msg->len & CAN_TDT0R_DLC);

  pCAN->IER |= CAN_IER_TMEIE;                 /* enable  TME interrupt        */
  pCAN->sTxMailBox[0].TIR |=  CAN_TI0R_TXRQ;  /* transmit message             */
}

/*----------------------------------------------------------------------------
  read a message from CAN peripheral and release it
 *----------------------------------------------------------------------------*/
void CAN_rdMsg (uint32_t ctrl, CAN_msg *msg)  {
  CAN_TypeDef *pCAN = (ctrl == 1) ? CAN1 : CAN2;

                                              /* Read identifier information  */
  if ((pCAN->sFIFOMailBox[0].RIR & CAN_ID_EXT) == 0) {
    msg->format = STANDARD_FORMAT;
    msg->id     = 0x000007FF & (pCAN->sFIFOMailBox[0].RIR >> 21);
  } else {
    msg->format = EXTENDED_FORMAT;
    msg->id     = 0x1FFFFFFF & (pCAN->sFIFOMailBox[0].RIR >> 3);
  }
                                              /* Read type information        */
  if ((pCAN->sFIFOMailBox[0].RIR & CAN_RTR_REMOTE) == 0) {
    msg->type =   DATA_FRAME;
  } else {
    msg->type = REMOTE_FRAME;
  }
                                              /* Read number of rec. bytes    */
  msg->len     = (pCAN->sFIFOMailBox[0].RDTR      ) & 0x0F;
                                              /* Read data bytes              */
  msg->data[0] = (pCAN->sFIFOMailBox[0].RDLR      ) & 0xFF;
  msg->data[1] = (pCAN->sFIFOMailBox[0].RDLR >>  8) & 0xFF;
  msg->data[2] = (pCAN->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
  msg->data[3] = (pCAN->sFIFOMailBox[0].RDLR >> 24) & 0xFF;

  msg->data[4] = (pCAN->sFIFOMailBox[0].RDHR      ) & 0xFF;
  msg->data[5] = (pCAN->sFIFOMailBox[0].RDHR >>  8) & 0xFF;
  msg->data[6] = (pCAN->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
  msg->data[7] = (pCAN->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

  pCAN->RF0R |= CAN_RF0R_RFOM0;             /* Release FIFO 0 output mailbox */
}


/*----------------------------------------------------------------------------
  setup acceptance filter
 *----------------------------------------------------------------------------*/
void CAN_wrFilter (uint32_t ctrl, uint32_t id, uint8_t format)  {
   CAN_TypeDef *pCAN = (ctrl == 1) ? CAN1 : CAN2;
   uint32_t      CAN_msgId     = 0;
  
  if (CAN_filterIdx[ctrl-1] > 13) {                 /* check if Filter Memory is full*/
    return;
  }
                                            /* Setup identifier information  */
  if (format == STANDARD_FORMAT)  {         /*   Standard ID                 */
      CAN_msgId |= (uint32_t)(id << 21) | CAN_ID_STD;
  }  else  {                                /*   Extended ID                 */
      CAN_msgId |= (uint32_t)(id <<  3) | CAN_ID_EXT;
  }

  pCAN->FMR  |=   CAN_FMR_FINIT;            /* set initMode for filter banks */
  pCAN->FA1R &=  ~(1UL << CAN_filterIdx[ctrl-1]);   /* deactivate filter             */

                                            /* initialize filter             */
  pCAN->FS1R |= (uint32_t)(1 << CAN_filterIdx[ctrl-1]);     /* set 32-bit scale configuration    */
	
	/*! !* To disable the CAN Filters: Swap the comment on the next two lines and the two lines after the next line. */
	pCAN->FM1R |= (uint32_t)(1 << CAN_filterIdx[ctrl-1]);   /* set to 32-bit Identifier List mode */
	//pCAN->FM1R |= 0x0;    							/* DISABLES FILTERS  set to 32-bit Identifier Mask mode */
	
  pCAN->sFilterRegister[CAN_filterIdx[ctrl-1]].FR1 = 0; //CAN_msgId; /*  32-bit identifier   */
	
	/* !!*  To disable the CAN Filters: Swap the comment on the next two lines.    */
  pCAN->sFilterRegister[CAN_filterIdx[ctrl-1]].FR2 = CAN_msgId; 	/*  32-bit identifier (33) for identifier mode  */
	//pCAN->sFilterRegister[CAN_filterIdx[ctrl-1]].FR2 = 0; 	/*  DISABLES FILTERS  32-bit MASK for mask mode */
   
   
  pCAN->FFA1R &= ~(uint32_t)(1 << CAN_filterIdx[ctrl-1]);   /* assign filter to FIFO 0  */
  pCAN->FA1R  |=  (uint32_t)(1 << CAN_filterIdx[ctrl-1]);   /* activate filter          */
	
  pCAN->FMR &= ~CAN_FMR_FINIT;              /* reset initMode for filterBanks*/
	
	CAN_filterIdx[ctrl-1]++;                       /* increase filter index  */
}

/*----------------------------------------------------------------------------
  CAN transmit interrupt handler
 *----------------------------------------------------------------------------*/
void CAN1_TX_IRQHandler (void) {

  if (CAN1->TSR & CAN_TSR_RQCP0) {          /* request completed mbx 0        */
    CAN1->TSR |= CAN_TSR_RQCP0;             /* reset request complete mbx 0   */
    CAN1->IER &= ~CAN_IER_TMEIE;            /* disable  TME interrupt         */

	  CAN_TxRdy[0] = 1; 
  }
}

void CAN2_TX_IRQHandler (void) {

  if (CAN2->TSR & CAN_TSR_RQCP0) {          /* request completed mbx 0        */
    CAN2->TSR |= CAN_TSR_RQCP0;             /* reset request complete mbx 0   */
    CAN2->IER &= ~CAN_IER_TMEIE;            /* disable  TME interrupt         */
	  CAN_TxRdy[1] = 1; 
  }
}


/*----------------------------------------------------------------------------
  CAN receive interrupt handler
 *----------------------------------------------------------------------------*/
void CAN1_RX0_IRQHandler (void) {

  if (CAN1->RF0R & CAN_RF0R_FMP0) {			    /* message pending ?              */
	  CAN_rdMsg (1, &CAN_RxMsg[0]);           /* read the message               */

    CAN_RxRdy[0] = 1;                       /*  set receive flag              */
  }
}

void CAN2_RX0_IRQHandler (void) {

  if (CAN2->RF0R & CAN_RF0R_FMP0) {			    /* message pending ?              */
	  CAN_rdMsg (2, &CAN_RxMsg[1]);           /* read the message               */

    CAN_RxRdy[1] = 1;                       /*  set receive flag              */
  }
}
