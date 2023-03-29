#include <stdint.h>
#include "F28x_Project.h"
#include "F2837xD_Device.h"
#include "F021_F2837xD_C28x.h"

#define Bzero_SectorK_start         0xB8000
#define Bzero_SectorK_End           0xB9FFF

#define Bzero_16KSector_u32length   0x1000

#define WriteDatasize               20

#ifdef __TI_COMPILER_VERSION__
    #if __TI_COMPILER_VERSION__ >= 15009000
        #define ramFuncSection ".TI.ramfunc"
    #else
        #define ramFuncSection "ramfuncs"
    #endif
#endif

#pragma DATA_SECTION(ReadData, "saveFuncSection");
char ReadData[];

#pragma DATA_SECTION(FlashData, "saveFlashSection");
char FlashData[];

uint16_t UART_check, Flash_Flg = 0;

void scia_fifo_init(void);
void read_lidarData(uint16_t now_data);
uint16_t compareData(void);
void Write_FlashAPI(void);

__interrupt void cpu_timer0_isr(void);
interrupt void sciaRxFifoIsr(void);

int main(void)
{
    InitSysCtrl();
    InitGpio();

    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;

    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

    EINT;

    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EALLOW;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1

    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;

    scia_fifo_init();

    EDIS;

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 500000);
    CpuTimer0Regs.TCR.all = 0x4000;

    IER |= M_INT1 + M_INT9;

    memcpy(FlashData, (uint32 *)Bzero_SectorK_start, sizeof(uint16_t)*WriteDatasize);

    EINT;
    ERTM;

    SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;

    while(1){
        if(Flash_Flg){
            Write_FlashAPI();
            Flash_Flg = 0;
        }
    }
}
void scia_fifo_init()
{
   ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;    // LSPCLK = SYSCLK

   SciaRegs.SCICCR.all = 0x0007;
   SciaRegs.SCICTL1.all = 0x0003;
   SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
   SciaRegs.SCIHBAUD.all = 0x0000;
   SciaRegs.SCILBAUD.all = 0x001B;

   SciaRegs.SCIFFTX.all = 0xC000;

   SciaRegs.SCICCR.bit.LOOPBKENA = 0;

   SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
   SciaRegs.SCIFFRX.bit.RXFFIL = 1;
   SciaRegs.SCIFFCT.all = 0x00;

   SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

#pragma CODE_SECTION(Write_FlashAPI, ramFuncSection);
void Write_FlashAPI(void)
{
    Fapi_StatusType oReturnCheck;
    Fapi_FlashStatusWordType oFlashStatusWord;

    EALLOW;
    DcsmCommonRegs.FLSEM.all = 0xA501;
    EDIS;

    InitFlash();
    EALLOW;
    Flash0EccRegs.ECC_ENABLE.bit.ENABLE = 0x0;
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 200);
    if(oReturnCheck != Fapi_Status_Success)
        __asm("    ESTOP0");
    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);
    if(oReturnCheck != Fapi_Status_Success)
        __asm("    ESTOP0");
    // Erase flash K
    Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32 *)Bzero_SectorK_start);
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady);

//     Verify that Sector K is erased. The erase step itself does verification as it goes.
//     This verify is a second verification that can be done.
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Bzero_SectorK_start, Bzero_16KSector_u32length, &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
        __asm("    ESTOP0");

    // write flash K some value
    uint16_t i;
    for(i=0;i<WriteDatasize;i+=4)
        Fapi_issueProgrammingCommand((uint32 *)(Bzero_SectorK_start+i), (uint16*)(ReadData+i), 4, 0, 0, Fapi_AutoEccGeneration);

    while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

    if(oReturnCheck != Fapi_Status_Success)
        __asm("    ESTOP0");

    Flash0EccRegs.ECC_ENABLE.bit.ENABLE = 0xA;

    //
    // Release flash semaphore.
    //
    DcsmCommonRegs.FLSEM.all = 0xA500;

    EDIS;

}

uint16_t compareData(void){
    uint16_t countNum = UART_check;
    for(countNum=0;countNum<UART_check;countNum++){
        if(FlashData[countNum] != ReadData[countNum])
            return 0;
    }
    return 1;
}

interrupt void sciaRxFifoIsr(void)
{
    uint16_t Rx_data = SciaRegs.SCIRXBUF.all;

    if((char)Rx_data == '|'){
        if(!compareData())
            Flash_Flg = 1;
        else
            Flash_Flg = 0;
        UART_check = 0;
    }
    else{
        *(ReadData+UART_check++) = Rx_data;
    }

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;

    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;
}

__interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;

   GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
   GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

   PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

//
// End of file
//
