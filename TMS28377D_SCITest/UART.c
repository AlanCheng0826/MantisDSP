#include <stdint.h>
#include "F28x_Project.h"
#include "math.h"

//sci buff
#define RX_MAX_LEN 10

uint16_t UART_check = 0;

unsigned char rdataA[RX_MAX_LEN];    // Received data for SCI-A

interrupt void sciaRxFifoIsr(void);
void scia_fifo_init(void);
void read_lidarData(uint16_t now_data);

__interrupt void cpu_timer0_isr(void);

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

    SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;

    EINT;
    ERTM;

    while(1){
        ;
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
   SciaRegs.SCIFFRX.bit.RXFFIL = 5;
   SciaRegs.SCIFFCT.all = 0x00;

   SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

interrupt void sciaRxFifoIsr(void)
{
    uint16_t Rx_data = SciaRegs.SCIRXBUF.all;


    UART_check++;

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
