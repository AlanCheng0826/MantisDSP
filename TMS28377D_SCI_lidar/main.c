#include <stdint.h>
#include "F28x_Project.h"
#include "math.h"

//sci buff
#define RX_MAX_LEN 200

enum UART_State{Idle, check_start01, check_start02, check_start03, check_start04, check_start05, check_start06, check_start07,
                Version, PactekLen, Platform, frameNumber, timeCpuCycles, numDetectedObj, numTLVs, subFrameNumber,
                TL_Type, TL_tlvLen,
                data_transmit};

enum UART_State Rx_State;

unsigned int rdataA01[RX_MAX_LEN];    // Received data for SCI-A
unsigned int rdataA02[RX_MAX_LEN];    // Received data for SCI-A

uint16_t Rx_count, UART_check, count_32;
Uint32 version, Rx_len, FrameNum, Cputime, DetectedNum, TLVnums, subFrameNum, TLtype, Tlvlen;
unsigned int *pointer;

interrupt void sciaRxFifoIsr(void);
void scia_fifo_init(void);

void read_lidarData(uint16_t now_data);
void Init_lidarData(void);

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

    Init_lidarData();

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

void Init_lidarData(void){
    pointer = rdataA01;
    count_32 = 0;
    UART_check = 0;
    Rx_count = 0;
    version = 0;
    Rx_len = 0;
    FrameNum = 0;
    Cputime = 0;
    DetectedNum = 0;
    TLVnums = 0;
    subFrameNum = 0;
    TLtype = 0;
    Tlvlen = 0;
    Rx_State = Idle;

}

void read_lidarData(uint16_t now_data){
    switch (Rx_State){
        case Idle:
            if (now_data == 0x02)
                Rx_State = check_start01;
            break;
        case check_start01:
            if (now_data == 0x01)
                Rx_State = check_start02;
            else
                Rx_State = Idle;
            break;
        case check_start02:
            if (now_data == 0x04)
                Rx_State = check_start03;
            else
                Rx_State = Idle;
            break;
        case check_start03:
            if (now_data == 0x03)
                Rx_State = check_start04;
            else
                Rx_State = Idle;
            break;
        case check_start04:
            if (now_data == 0x06)
                Rx_State = check_start05;
            else
                Rx_State = Idle;
            break;
        case check_start05:
            if (now_data == 0x05)
                Rx_State = check_start06;
            else
                Rx_State = Idle;
            break;
        case check_start06:
            if (now_data == 0x08)
                Rx_State = check_start07;
            else
                Rx_State = Idle;
            break;
        case check_start07:
            if (now_data == 0x07)
                Rx_State = Version;
            else
                Rx_State = Idle;
            break;
        case Version:
            version |= ((Uint32)now_data << (8*count_32++));
            if (count_32 == 4){
                count_32 = 0;
                Rx_State = PactekLen;
            }
            break;
        case PactekLen:
            Rx_len |= ((Uint32)now_data << (8*count_32++));
            if (count_32 == 4){
                count_32 = 0;
                Rx_State = Platform;
            }
            break;
        case Platform:
            count_32++;
            if (count_32 == 4){
                count_32 = 0;
                Rx_State = frameNumber;
            }
            break;
        case frameNumber:
            FrameNum |= ((Uint32)now_data << (8*count_32++));
            if (count_32 == 4){
                count_32 = 0;
                Rx_State = timeCpuCycles;
            }
            break;
        case timeCpuCycles:
            Cputime |= ((Uint32)now_data << (8*count_32++));
            if (count_32 == 4){
                count_32 = 0;
                Rx_State = numDetectedObj;
            }
            break;
        case numDetectedObj:
            DetectedNum |= ((Uint32)now_data << (8*count_32++));
            if (count_32 == 4){
                count_32 = 0;
                Rx_State = numTLVs;
            }
            break;
        case numTLVs:
            TLVnums |= ((Uint32)now_data << (8*count_32++));
            if (count_32 == 4){
                count_32 = 0;
                Rx_State = subFrameNumber;
            }
            break;
        case subFrameNumber:
            subFrameNum |= ((Uint32)now_data << (8*count_32++));
            if (count_32 == 4){
                count_32 = 0;
                Rx_State = TL_Type;
            }
            break;
        case TL_Type:
            TLtype |= ((Uint32)now_data << (8*count_32++));
            if (count_32 == 4){
                count_32 = 0;
                Rx_State = TL_tlvLen;
            }
            break;
        case TL_tlvLen:
            Tlvlen |= ((Uint32)now_data << (8*count_32++));
            if (count_32 == 4){
                count_32 = 0;
                Rx_count = 0;
                Rx_State = data_transmit;
            }
            break;
        case data_transmit:
            if(Rx_count > RX_MAX_LEN)
                pointer = rdataA02;
            *pointer++ = now_data;
            Rx_count++;
            if((Rx_count+48) > Rx_len)
                Init_lidarData();
            break;
        default:
            Init_lidarData();
            break;
    }
}

interrupt void sciaRxFifoIsr(void)
{
    Uint16 Rx_data = SciaRegs.SCIRXBUF.bit.SAR;

    read_lidarData(Rx_data);

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
