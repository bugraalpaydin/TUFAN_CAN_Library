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

#include "CAN_Driver.h"

CAN_Msg CAN_TxMsg;
CAN_Msg CAN_RxMsg;

unsigned int CAN_TxRdy = 0;
unsigned int CAN_RxRdy = 0;


void CAN_CLOCK_ENABLE(void){
    //Alternate function enable
    RCC -> APB2ENR |= (1<<0);
    //GPIOA clock enable
    RCC -> APB2ENR |= (1<<2);
    //CAN1 clock enable
    RCC -> APB1ENR |= (1<<25);
}
void CAN_Setup(void){
    //NVIC -> ISER[0] |= (1<< (USB_HP_CAN1_TX_IRQn & 0x1F));
    //NVIC -> ISER[0] |= (1<< (USB_LP_CAN1_RX0_IRQn & 0x1F));

    //Initialization Request
    CAN1 -> MCR |= (1<<0);
    // No automatic retransmission
    CAN1 -> MCR |= (1<<4);
    // Auto retransmit
    //CAN1 -> MCR &= ~(1<<4);
    
    //FIFO message pending interrupt enable
    CAN1 -> IER |= (1<<0);
    //Transmit mailbox empty interrupt enable
    CAN1 -> IER |= (1<<1);

    int brp = 500000;

    CAN1 -> BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((         0x0F) << 16) | (          0x1FF));
    CAN1 -> BTR |=  ((((4-1) & 0x03) << 24) | (((5-1) & 0x07) << 20) | (((12-1) & 0x0F) << 16) | ((brp-1) & 0x1FF));
}


//LEAVE INITIALIZATION MODE 
void CAN_StartNormalMode(void){
    //Normal operating mode , reset INRQ 
    CAN1 -> MCR &= ~(1<<0);
    while (CAN1->MSR & (1<<0));
}


//SET THE TEST MODE
void CAN_TestMode(unsigned int testmode){
    CAN1->BTR &= ~(CAN_BTR_SILM | CAN_BTR_LBKM);     // set testmode
    CAN1->BTR |=  (testmode & (CAN_BTR_SILM | CAN_BTR_LBKM));
}


//CHECK IF TRANSMIT MAILBOX IS EMPTY
void CAN_WaitReady(void){
    while((CAN1 -> TSR & (1<<26)) == 0); //TRANSMIT MAILBOX 0 IS EMPTY
    CAN_TxRdy = 1;
}


// WRITE A MESSAGE TO CAN PERHIPERAL AND TRANSMIT IT
void CAN_WriteMessage(CAN_Msg *msg){
    CAN1->sTxMailBox[0].TIR = (unsigned int)0; // reset TIR register

    if(msg->format == STANDARD_FORMAT) // Setup identifier information, standard ID
        CAN1 -> sTxMailBox[0].TIR |= (unsigned int)(msg->id << 21) | (0x00000000U);
    else // extended ID
        CAN1 -> sTxMailBox[0].TIR |= (unsigned int)(msg->id << 3) | (0x00000004U);
    // Setup type information
    if(msg->type == DATA_FRAME) // Data frame
        CAN1 -> sTxMailBox[0].TIR |= (0x00000000U);
    else    //remote frame
        CAN1 -> sTxMailBox[0].TIR |= (0x00000002U);

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
    CAN1 -> sTxMailBox[0].TDTR &= ~(0xFUL << (0U));
    CAN1 -> sTxMailBox[0].TDTR |= (msg->len & (0xFUL << (0U)));

    CAN1 -> IER |= CAN_IER_TMEIE;                   // Enable TME interrup
    CAN1 -> sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;     // Transmit message
}

void CAN_ReadMessage(CAN_Msg *msg){
    //TODO
}


void CAN_SetFilter(unsigned int id, unsigned char format){
    static unsigned short CAN_FilterId = 0;
    unsigned int CAN_MsgId             = 0;

    if (CAN_FilterId > 13) {
        return;
    }

    if (format == 0){
        CAN_MsgId |= (unsigned int)(id<<21) | (0x00000000U);
    }
    else {
        CAN_MsgId |= (unsigned int)(id<<3) | (0x00000004U)
;
    }
    //SET INITIALIZATION MODE FOR FILTER BANKS
    CAN1->FMR |= (1<<0); 
    //DEACTIVE FILTER
    CAN1->FA1R &= ~(unsigned int)(1 << CAN_FilterId);//deactive filter 

    //INITIALIZE FILTER
    //SET 32-BIT SCALE CONFIg
    CAN1 -> FS1R |= (unsigned int)(1 << CAN_FilterId);
    //SET 2 32-bit idenfitier list mode
    CAN1 -> FM1R |= (unsigned int)(1 << CAN_FilterId);
    
    //32-bit identifier
    CAN1->sFilterRegister[CAN_FilterId].FR1 = CAN_MsgId;
    //32-bit identifier
    CAN1->sFilterRegister[CAN_FilterId].FR2 = CAN_MsgId;

    //assign filter to FIFO 0
    CAN1->FFA1R &= ~(unsigned int)(1 << CAN_FilterId);
    //active filter
    CAN1->FA1R |= (unsigned int)(1 << CAN_FilterId);
    //reset initialization mode for filter banks
    CAN1->FMR &= ~(1<<0);

    //Increase filter index
    CAN_FilterId += 1;
}



void CAN_GPIO_Init(void){
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
    //CAN REMAP = 0
    AFIO->MAPR &= ~(1<<13);
    AFIO->MAPR &= ~(1<<14);
}



/*
//CAN TRANSMIT INTERRUPT HANDLER
void USB_HP_CAN1_TX_IRQHandler(void){

    if(CAN1->TSR & (1<<0)) {            // request completed mbx 0
        CAN1->TSR |= (1<<0);            // reset request complete mbx 0
        CAN1->IER &= ~(1<<0);           // disable TME interrupt

        CAN_TxRdy = 1;                  // set transmit flag
    }
}

//CAN RECEIVE INTERRUPT HANDLER
void USB_LP_CAN1_RX0_IRQHandler(void){
    
    if(CAN1->RF0R & 0x3){               // message pending?
        CAN_ReadMessage(&CAN_RxMsg);    // read the message
        CAN_RxRdy = 1;                  // set receive flag
    }
}

*/