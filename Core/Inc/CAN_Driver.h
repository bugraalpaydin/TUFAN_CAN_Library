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
*/

#include "stm32f1xx.h"
#include <stdint.h>


#define STANDARD_FORMAT 0
#define EXTENDED_FORMAT 1

#define DATA_FRAME      0
#define REMOTE_FRAME    1

typedef struct{
    unsigned int id;
    unsigned char data[8];
    unsigned char len;
    unsigned char format;
    unsigned char type;
}CAN_Msg;


void CAN_CLOCK_ENABLE(void);
void CAN_Setup(void);
void CAN_StartNormalMode(void);
void CAN_WaitReady(void);
void CAN_TestMode(unsigned int testmode);
void CAN_SetFilter(unsigned int id, unsigned char format);
void CAN_WriteMessage(CAN_Msg *msg);
void CAN_ReadMessage(CAN_Msg *msg);
void CAN_GPIO_Init(void);

extern CAN_Msg CAN_TxMsg;       // CAN message for sending
extern CAN_Msg CAN_RxMsg;       // CAN message for receiving
extern unsigned int CAN_TxRdy;  // CAN HW ready to transmit a message
extern unsigned int CAN_RxRdy;  // CAN HW ready to receive a message