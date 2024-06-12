/*
    INITALIZATION MODE::
        - To enter initalization mode:
            set  CAN->MCR firs bit ; wait until hardware set CAN->MSR first bit
        - To leave initalization mode;
            clear CAN->MCR first bit ; wait until hardware reser CAN->MSR first bit 
        - While initalization mode CAN bus output CANTX is recessive(HIGH)
        - To initalize the registers associated with the CAN filter banks(mode, scale, FIFO, assignment, activation and filter values), 
        software has to seh FINIT bit in the CAN->FMR register.Filter initalization also can be done outside the initalization mode
        - To initalize the CAN Cntroller, set CAN->BTR register and CAN->MCR registers

        - When FINIT=1, CAN reception is deactivated.
        - The filter values also can be modified by deactivating the associated filter activation bits in the CAN->FA1R register
        - If a filter bank is not used, its recommended to leave it non active (leave the corresponding FACT bit cleared)
    
    NORMAL MODE::
        - The request to enter Normal mode is issued by clearing INRQ bit in the CAN->MCR register. 
        - The bxCAN enters Normal mode and is ready totake part in bus acitivities when it has synchronized with data transfer on the CAN bus, 
        this is done by waiting for the occurence of a sequence of 11 consecutive recessive bits.
        - The initalization of the filter values is independent from initalizatio mode must be done while the filter is not active(corresponding FACTx bit cleared). 
            The filter scale and mode configuration must be configured before entering Normal mode

    LOOPBACK MODE::
        - The bxCAN can be set in Loop-Back mode by setting the LBKM bit in the CAN_BTR register. In loop-back mode, 
        - This mode is provided for self-test functions. To be independent of external events, 
        the CAN Core ignores acknowledge errors(no dominant bit sampled in the acknowledge slot of a data/remote frame) in loop-back mode. 
        In this mode, the bxCAN performs an internal feedback from its Tx output to its Rx input. 
        The acutal value of the CANRX input pin is disregarded by the bxCAN. The transmitted messages can be monitored on the CANTX pin
        the bxCAN treats its own transmitted messages as received messages and stores them (if they pass acceptance filtering) in a Receive mailbox.



    CAN_TX (Transmit Data line) Alternate Function push-pull
    CAN_RX (Receive Data line) Input floating
    CAN1_RX or CAN_RX PA11
    CAN1_TX or CAN_TX PA12
*/



/*
    RCC -> BASE ADDRESS 0x4002 1000
    RCC -> APB2ENR OFFSET 0x18
    RCC -> APB1ENR OFFSET 0x1C 
    GPIOA -> BASE ADDRESS 0x4001 0800 
    GPIOA -> CRH OFFSET 0x04

    AFIO -> BASE ADDRESS 0x4001 0000
    AFIO -> MAPR OFFSETT 0x04
    <Xz

*/

#include "CAN_Driver.h"
#include "stm32f1xx.h"

CAN_Msg CAN_TxMsg;
CAN_Msg CAN_RxMsg;

unsigned int CAN_TxRdy = 0;
unsigned int CAN_RxRdy = 0;


void TUFAN_CAN_CLOCK_ENABLE(void){
    RCC->CR       |= (1<<16);                 //Enable HSE

    while((RCC->CR & (1<<17)) == RESET);   //Wait for HSE to turn ON

    RCC->CR       |=  (1<<24);             //Enable PLL
    
    RCC->CFGR     |=  (1<<16);           //HSE clock selected as PLL input clock
    RCC->CFGR     &= ~(1<<17);           //HSE clock bit not divided before PLL entry

    //PLL Multiplication factor x9 (0111)
    RCC->CFGR     &= ~(1<<21); 
    RCC->CFGR     |=  (1<<20);
    RCC->CFGR     |=  (1<<19);
    RCC->CFGR     |=  (1<<18);

    //PLL selected as system clock 
    RCC->CFGR     |=  (1<<1);
    RCC->CFGR     &= ~(1<<0);

    //AHB prescaler System clock not divided
    RCC->CFGR     &= ~(1<<7);
    RCC->CFGR     &= ~(1<<6);
    RCC->CFGR     &= ~(1<<5);
    RCC->CFGR     &= ~(1<<4);

    //APB1 Prescaler /2 (SYSCLK divided by 2) (1000)
    RCC->CFGR     |=  (1<<7);
    RCC->CFGR     &= ~(1<<6);
    RCC->CFGR     &= ~(1<<5);
    RCC->CFGR     &= ~(1<<4);


    RCC -> APB2ENR |= (1<<0);//w           //Alternate function enable
    RCC -> APB2ENR |= (1<<2);//w           //GPIOA clock enable
    RCC -> APB1ENR |= (1<<25);//w          //CAN1 clock enable
}

void TUFAN_CAN_Setup(void){
    NVIC->ISER[0] |= (1<< (USB_HP_CAN1_TX_IRQn & 0x1F));
    NVIC->ISER[0] |= (1<< (USB_LP_CAN1_RX0_IRQn & 0x1F));

    //Initialization Request
    CAN1->MCR |= (1<<0);
    while(!(CAN1->MCR & (1<<0)));

    
    // No automatic retransmission
    CAN1->MCR |= (1<<4);
    
    // Auto retransmit
    //CAN1 -> MCR &= ~(1<<4);
    

    
    //FIFO message pending interrupt enable
    CAN1->IER |= (1<<0);
    //Transmit mailbox empty interrupt enable
    CAN1->IER |= (1<<1);
    
    //CAN1->BTR reset value=0x0123 0000 -> 0x0000 0000
    
    CAN1->BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((         0x0F) << 16) | (          0x1FF));
    
    
    //SJW -> 3, TS2 -> 4, TS1 -> 1, BRP -> 3 
    

    CAN1->BTR |=  ((3 << 24) | (4 << 20) | (1 << 16) | (3 & 0x1FF));
    
    
    }


//LEAVE INITIALIZATION MODE 
void TUFAN_CAN_Start(void){
    //Normal operating mode , reset INRQ 
    CAN1->MCR &= ~(1<<0);
    while (CAN1->MSR & (1<<0));
}


//SET THE TEST MODE

void TUFAN_CAN_TestMode(unsigned int testmode){
    CAN1->BTR |= (1<<30);   //Enable loopback mode
}



//CHECK IF TRANSMIT MAILBOX IS EMPTY
void TUFAN_CAN_WaitReady(void){
    while((CAN1->TSR & (1<<26)) == 0); //TRANSMIT MAILBOX 0 IS EMPTY
    CAN_TxRdy = 1;
}

  
// WRITE A MESSAGE TO CAN PERHIPERAL AND TRANSMIT IT
void TUFAN_CAN_WriteMessage(CAN_Msg *msg){
    
    CAN1->sTxMailBox[0].TIR = (unsigned int)0; // reset TIR register

    if(msg->format == STANDARD_FORMAT) // Setup identifier information, standard ID
        CAN1->sTxMailBox[0].TIR |= (unsigned int)(msg->id << 21) | (0x00000000U);
    else // extended ID
        CAN1->sTxMailBox[0].TIR |= (unsigned int)(msg->id << 3) | (0x00000004U);
    // Setup type information
    if(msg->type == DATA_FRAME) // Data frame
        CAN1->sTxMailBox[0].TIR |= (0x00000000U);
    else    //remote frame
        CAN1->sTxMailBox[0].TIR |= (0x00000002U);

    // Setup data bytes
    CAN1->sTxMailBox[0].TDLR = (((unsigned int)msg->data[3] << 24) | 
                                ((unsigned int)msg->data[2] << 16) |
                                ((unsigned int)msg->data[1] <<  8) | 
                              
  ((unsigned int)msg->data[0]));
    CAN1->sTxMailBox[0].TDHR = (((unsigned int)msg->data[7] << 24) | 
                                ((unsigned int)msg->data[6] << 16) |
                                ((unsigned int)msg->data[5] <<  8) |
                                ((unsigned int)msg->data[4]));

    //SETUP LENGTH
    CAN1->sTxMailBox[0].TDTR &= ~(0xFUL << (0U));
    CAN1->sTxMailBox[0].TDTR |= (msg->len & (0xFUL << (0U)));
    CAN1->IER |= CAN_IER_TMEIE;                   // Enable TME interrup
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;     // Transmit message
}

void TUFAN_CAN_ReadMessage(CAN_Msg *msg){
                                                    // Read identifier information
  if ((CAN1->sFIFOMailBox[0].RIR & 0x00000000U) == 0) { // Standard ID
    msg->format = STANDARD_FORMAT;
    msg->id     = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21);
  }  else  {                                          // Extended ID
    msg->format = EXTENDED_FORMAT;
    msg->id     = (uint32_t)0x0003FFFF & (CAN1->sFIFOMailBox[0].RIR >> 3);
  }
                                                  // Read type information
  if ((CAN1->sFIFOMailBox[0].RIR & 0x00000002U) == 0) {
    msg->type =   DATA_FRAME;                     // DATA   FRAME
  }  else  {
    msg->type = REMOTE_FRAME;                     // REMOTE FRAME
  }
                                                  // Read length (number of received bytes)
  msg->len = (unsigned char)0x0000000F & CAN1->sFIFOMailBox[0].RDTR;
                                                  // Read data bytes
  msg->data[0] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR);
  msg->data[1] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 8);
  msg->data[2] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 16);
  msg->data[3] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 24);

  msg->data[4] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR);
  msg->data[5] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 8);
  msg->data[6] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 16);
  msg->data[7] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 24);

  CAN1->RF0R |= CAN_RF0R_RFOM0;                  
}


void TUFAN_CAN_SetFilter(unsigned int id, unsigned char format){
    static unsigned short CAN_FilterId = 0;
    unsigned int CAN_MsgId             = 0;

    if (CAN_FilterId > 13) {
        return;
    }

    if (format == 0){
        CAN_MsgId |= (unsigned int)(id<<21) | (0x00000000U);
    } 
    else {
        CAN_MsgId |= (unsigned int)(id<<3) | (0x00000004U);
    }
    //SET INITIALIZATION MODE FOR FILTER BANKS
    CAN1->FMR |= (1<<0); 
    //DEACTIVE FILTER
    CAN1->FA1R &= ~(unsigned int)(1 << CAN_FilterId);//deactive filter 

    //INITIALIZE FILTER
    //SET 32-BIT SCALE CONFIg
    CAN1->FS1R |= (unsigned int)(1 << CAN_FilterId);
    //SET 2 32-bit idenfitier list mode
    CAN1->FM1R &= ~(unsigned int)(1 << CAN_FilterId); //i changed to mask mode
    
    //32-bit identifier
    CAN1->sFilterRegister[CAN_FilterId].FR1 = CAN_MsgId; //?????
    //32-bit identifier
    CAN1->sFilterRegister[CAN_FilterId].FR2 = CAN_MsgId; //??????

    //assign filter to FIFO 0
    CAN1->FFA1R &= ~(unsigned int)(1 << CAN_FilterId);
    //active filter
    CAN1->FA1R |= (unsigned int)(1 << CAN_FilterId);
    //reset initialization mode for filter banks
    CAN1->FMR &= ~(1<<0);

    //Increase filter index
    CAN_FilterId += 1;
}



void TUFAN_CAN_GPIO_Init(void){
    /*
    //PA11 input mode
    GPIOA->CRH &= ~(1<<12);
    GPIOA->CRH &= ~(1<<13);
    //PA11 floating input
    GPIOA->CRH |= (1<<14);
    GPIOA->CRH = ~(1<<15);
    //PA12 output mode, max speed 50 MHz
    GPIOA -> CRH |= (1<<16);
    GPIOA -> CRH |= (1<<17);
    //PA12 alternate function output push-pull
    GPIOA -> CRH &= ~(1<<18);
    GPIOA -> CRH |= (1<<19);
  */
    GPIOA->CRH = 0x444B4444;
    //CAN REMAP = 0
    AFIO->MAPR &= ~(1<<13); //w
    AFIO->MAPR &= ~(1<<14); //w

}


void TUFAN_CAN_Init(void){
    TUFAN_CAN_CLOCK_ENABLE();
    TUFAN_CAN_GPIO_Init();
    TUFAN_CAN_Setup();
    TUFAN_CAN_SetFilter(33, STANDARD_FORMAT);
    TUFAN_CAN_TestMode(CAN_BTR_SILM | CAN_BTR_LBKM);
    TUFAN_CAN_Start();
    TUFAN_CAN_WaitReady();

}


//CAN TRANSMIT INTERRUPT HANDLER
void USB_HP_CAN1_TX_IRQHandler(void){

    if( CAN1->TSR & (1<<0)) {            // request completed mbx 0
        CAN1->TSR |= (1<<0);            // reset request complete mbx 0
        CAN1->IER &= ~(1<<0);           // disable TME interrupt

        CAN_TxRdy = 1;                  // set transmit flag
    }
}

//CAN RECEIVE INTERRUPT HANDLER
void USB_LP_CAN1_RX0_IRQHandler(void){
    
    if( CAN1->RF0R & 0x3){               // message pending?
        TUFAN_CAN_ReadMessage(&CAN_RxMsg);    // read the message
        CAN_RxRdy = 1;                  // set receive flag
    }
}


