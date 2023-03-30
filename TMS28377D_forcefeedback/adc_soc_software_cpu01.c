#include <stdint.h>
#include "F28x_Project.h"
#include "math.h"

#define Serialplot

//
// Function Prototypes
//
void ConfigureADC(void);
void SetupADCSoftware(void);

interrupt void sciaTxFifoIsr(void);
interrupt void sciaRxFifoIsr(void);
void scia_fifo_init(void);

interrupt void scibTxFifoIsr(void);
interrupt void scibRxFifoIsr(void);
void scib_fifo_init(void);
uint32_t MyCRC_GetCRC32(unsigned char *pData,uint16_t Length);

__interrupt void cpu_timer0_isr(void);

#define RS485_DIR_GPIO 16
#define MASTER_MODE 1
#define SLAVE_MODE 0
#define CRC_size 24
//
// Globals
//
//the calculate decoupled cfg
//long double cfg[6][6] = {
//    {-0.573544,-256.736808,-2.740936,257.448550,3.140312,-1.041328},
//    {4.037845,-149.929327,0.632542,-145.827896,0.349118,296.192305},
//    {-426.296884,0.305894,-424.213861,0.030523,-428.985950,0.741775},
//    {-16.749652,0.033442,16.414278,-0.014073,-0.067731,-0.057504},
//    {-9.658386,-0.068087,-9.644111,0.062401,19.400566,-0.037218},
//    {0.010493,12.257437,-0.090725,12.519111,0.040279,12.140064}
//};

//SN:16066
uint32_t const crc32_table[256] =
{
    0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b,
    0x1a864db2, 0x1e475005, 0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
    0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd, 0x4c11db70, 0x48d0c6c7,
    0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
    0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3,
    0x709f7b7a, 0x745e66cd, 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
    0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 0xbe2b5b58, 0xbaea46ef,
    0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
    0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb,
    0xceb42022, 0xca753d95, 0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
    0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d, 0x34867077, 0x30476dc0,
    0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
    0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4,
    0x0808d07d, 0x0cc9cdca, 0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
    0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02, 0x5e9f46bf, 0x5a5e5b08,
    0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
    0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc,
    0xb6238b25, 0xb2e29692, 0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
    0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a, 0xe0b41de7, 0xe4750050,
    0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
    0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34,
    0xdc3abded, 0xd8fba05a, 0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
    0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb, 0x4f040d56, 0x4bc510e1,
    0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
    0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5,
    0x3f9b762c, 0x3b5a6b9b, 0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
    0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 0xf12f560e, 0xf5ee4bb9,
    0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
    0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd,
    0xcda1f604, 0xc960ebb3, 0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
    0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 0x9b3660c6, 0x9ff77d71,
    0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
    0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2,
    0x470cdd2b, 0x43cdc09c, 0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
    0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24, 0x119b4be9, 0x155a565e,
    0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
    0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a,
    0x2d15ebe3, 0x29d4f654, 0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
    0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c, 0xe3a1cbc1, 0xe760d676,
    0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
    0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662,
    0x933eb0bb, 0x97ffad0c, 0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
    0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
};

long double cfg[6][6] = {
    {1.78082,-220.32786,-0.35461,220.64641,1.34612,0.38649},
    {0.05243,-126.40294,-0.41481,-128.42281,-3.69687,253.21641},
    {-368.09341,-2.91946,-371.25766,0.80838,-368.13826,0.55156},
    {-14.28788,-0.17189,14.68266,-0.09689,0.22487,0.05061},
    {-8.37284,0.07502,-8.43050,-0.17108,16.64074,-0.03941},
    {-0.03259,10.40799,-0.10783,10.67395,-0.09739,10.61476}
};

Uint16   adcrawbuff[6];
float64  adcvbuff[6];
float64  adcmvvbuff[6];
float    databuff[6];
Uint16   adcbuff[6];

//graph test buff
#define G_P_N 200//graph point number
Uint16 graphcount;
float  a[G_P_N];
float  b[G_P_N];
float  c[G_P_N];
float  d[G_P_N];
float  e[G_P_N];
float  f[G_P_N];

//sci-b buff
#define TX_MAX_LEN 128
#define RX_MAX_LEN 128

unsigned char sdataA[TX_MAX_LEN];    // Send data for SCI-A
unsigned char rdataA[RX_MAX_LEN];    // Received data for SCI-A

unsigned char sdataB[TX_MAX_LEN];    // Send data for SCI-B
unsigned char rdataB[RX_MAX_LEN];    // Received data for SCI-B

typedef struct{
    Uint16 slen;  //the length of the data to send
    Uint16 scount;//the count of the data already be sent
    Uint16 spcount;//the count of data package
    unsigned char *ptr;
    unsigned char sum;

}SCIRAMTYPE;
SCIRAMTYPE sciaram,scibram;

//system
typedef struct{
    Uint16 raflag;//receive complete flag
    Uint16 racnt; //reveived byte count
    Uint16 rbflag;//receive complete flag
    Uint16 rbcnt; //reveived byte count
    Uint16 sampflag;//1:start adc sampling, 0:stop adc sampling
    Uint16 DevMode;//1:master mode, 0:slave mode
}SYSRAMTYPE;
SYSRAMTYPE sysram;


void protocol(unsigned char *srcadd, SYSRAMTYPE *ramadd, Uint16 len);

void main(void)
{
    Uint32 i,j;
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    InitGpio();

    //init the scia gpio
//    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
//    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
//    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
//    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
    GpioDataRegs.GPASET.bit.GPIO28 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;

    EINT;  // Enable Global interrupt INTM

    GPIO_SetupPinMux(RS485_DIR_GPIO, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RS485_DIR_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_WritePin(RS485_DIR_GPIO, 1);//RS-485 out put

    //init the scib gpio
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 2);
    GPIO_SetupPinOptions(15, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 2);
    GPIO_SetupPinOptions(14, GPIO_OUTPUT, GPIO_ASYNC);

    //init operation mode select pin
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_INPUT, GPIO_OPENDRAIN);

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;
//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();
//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;
//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
    InitFlash();

    //enable scia fifo int
    EALLOW;  // This is needed to write to EALLOW protected registers
//    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;
//    PieVectTable.SCIA_TX_INT = &sciaTxFifoIsr;
    //EDIS;    // This is needed to disable write to EALLOW protected registers

//    scia_fifo_init();  // Init SCI-A

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;   // PIE Group 9, INT2
    //IER = 0x100;                         // Enable CPU INT
    //EINT;

    //enable scib fifo int
    //EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.SCIB_RX_INT = &scibRxFifoIsr;
    PieVectTable.SCIB_TX_INT = &scibTxFifoIsr;
    //EDIS;    // This is needed to disable write to EALLOW protected registers

    scib_fifo_init();  // Init SCI-B

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;   // PIE Group 9, INT3
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;   // PIE Group 9, INT4
    IER = 0x100;                         // Enable CPU INT

    //timer 0
    //EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    //Initialize the Device Peripheral. This function can be
    //found in F2837xD_CpuTimers.c
    InitCpuTimers();   // For this example, only initialize the Cpu Timers
    // Configure CPU-Timer 0 to __interrupt every 30 milliseconds:
    // 100MHz CPU Freq, 30 millisecond Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 200, 500000);
    CpuTimer0Regs.TCR.all = 0x4000;//stop timer
    // Enable CPU INT1 which is connected to CPU-Timer 0:
    //
    IER |= M_INT1;
    // Enable TINT0 in the PIE: Group 1 __interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

//
//Configure the ADCs and power them up
//
    ConfigureADC();

//
//Setup the ADCs for software conversions
//
    SetupADCSoftware();

    //init all ram
    for(i=0;i<RX_MAX_LEN;i++)
    {
        sdataA[i] = 0;
        rdataA[i] = 0;
        sdataB[i] = 0;
        rdataB[i] = 0;
    }
    sciaram.scount = 0;
    scibram.scount = 0;

//    if(sysram.DevMode == MASTER_MODE)//master mode, wait here
//        for(;;);//retransmission the data via scia interrupts

    sysram.DevMode = GPIO_ReadPin(1);
    sysram.sampflag = 1;//test
    //sysram.DevMode = 0;
    //sysram.sampflag = (sysram.DevMode == 0) ? 1 : 0;



    SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
    ScibRegs.SCIFFTX.bit.TXFFIENA = 0;
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

//
//take conversions indefinitely in loop
//
    do
    {
        //if(sysram.DevMode == SLAVE_MODE)
        {

            //
            //convert, wait for completion, and store results
            //start conversions immediately via software, ADCA
            //
            AdcaRegs.ADCSOCFRC1.all = 0x003f; //SOC0 to SOC5

            //
            //wait for ADCA to complete, then acknowledge flag
            //
            while(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0);
            AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

            //
            //wait for ADCB to complete, then acknowledge flag
            //
    //        while(AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0);
    //        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
            //store results
    //        AdcbResult0 = AdcbResultRegs.ADCRESULT0;
    //        AdcbResult1 = AdcbResultRegs.ADCRESULT1;
            //
            //store results
            //
            for(i=0;i<6;i++)
            {
                adcrawbuff[i] = *(&AdcaResultRegs.ADCRESULT0 + i);
                adcvbuff[i]   = adcrawbuff[i]*3.3/4096 - 1.65;//convert to V
            }
            for (i=0;i<3;i++){
                adcmvvbuff[i]   = adcvbuff[i+3]*1000/10/100;//convert to mVV, data*1000/(EXC*gain)
                adcmvvbuff[i+3] = adcvbuff[i]*1000/10/100;
            }

            for(i=0;i<6;i++)
            {
                databuff[i] = 0;
                for(j=0;j<6;j++)
                {
                    databuff[i] += cfg[i][j]*adcmvvbuff[j];
                }
            }
            //graph test
//            graphcount++;
//            if(graphcount >= G_P_N)
//                graphcount = 0;
//            a[graphcount] = databuff[0];
//            b[graphcount] = databuff[1];
//            c[graphcount] = databuff[2];
//            d[graphcount] = databuff[3];
//            e[graphcount] = databuff[4];
//            f[graphcount] = databuff[5];

            sciaram.spcount++;
            sciaram.ptr = sdataB;
//            *sciaram.ptr++ = 0xF5;//no use, to make the pack intact
            *sciaram.ptr++ = 0xAA;
            *sciaram.ptr++ = 0x55;
#ifndef Serialplot
            *sciaram.ptr++ = 0x00;
            *sciaram.ptr++ = 0x1E;
//            *sciaram.ptr++ = (unsigned char)(sciaram.spcount >> 8);
//            *sciaram.ptr++ = (unsigned char)sciaram.spcount;
            *sciaram.ptr++ = 0x00;
            *sciaram.ptr++ = 0x00;
#endif
            sciaram.sum = 0;

            for(i=0;i<6;i++)
            {
                *sciaram.ptr++ = (unsigned char)(*(Uint32*)&databuff[i] & 0x000000ff);
                *sciaram.ptr++ = (unsigned char)((*(Uint32*)&databuff[i] >> 8) & 0x000000ff);
                *sciaram.ptr++ = (unsigned char)((*(Uint32*)&databuff[i] >> 16) & 0x000000ff);
                *sciaram.ptr++ = (unsigned char)((*(Uint32*)&databuff[i] >> 24) & 0x000000ff);
            }

#ifdef Serialplot
            sciaram.slen = 26;
#else
            uint32_t crc_number = 0;
            crc_number = MyCRC_GetCRC32(sdataB, CRC_size);

//            memcpy(sciaram.ptr, &crc_number, sizeof(uint32_t));
//            sciaram.ptr += sizeof(uint32_t);

            for (i=0;i<4;i++){
                *sciaram.ptr++ = (crc_number >> (8*(3-i))) & 0x000000FF;
            }
            sciaram.slen = 34;

#endif

            sciaram.scount = 1;

            if(sysram.sampflag == 1)
            {
                ScibRegs.SCIFFTX.bit.TXFFIENA = 1;
                ScibRegs.SCITXBUF.all=sdataB[0];  // Send data
        //        for(i=0;i<scibram.slen;i++)
        //        {
        //            while (ScibRegs.SCICTL2.bit.TXRDY == 0) {}
        //            ScibRegs.SCITXBUF.all = (Uint16)sdataB[i];
        //        }
            }

            if(sysram.rbflag == 1)
            {
                protocol(rdataB, &sysram, sysram.rbcnt);
                for(i=0;i<RX_MAX_LEN;i++)
                {
                    rdataB[i] = 0;
                }
                sysram.rbflag = 0;
                sysram.rbcnt = 0;
            }
        }

        DELAY_US(2000);
    }while(1);
}

//calculate CRC data

uint32_t MyCRC_GetCRC32(unsigned char *pData,uint16_t Length)
{
    uint32_t nReg;
    uint32_t nTemp = 0x00;
    uint16_t i, n;

    nReg = 0xFFFFFFFF;
    for(n = 0x00; n < Length; n++)
    {
        nReg ^= (uint32_t)pData[n+6];
        for(i = 0x00; i < 4; i++)
        {
            nTemp = crc32_table[(unsigned char)(( nReg >> 24 ) & 0xff)];
            nReg <<= 8;
            nReg ^= nTemp;
        }
    }
    return nReg;
}

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    //AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    //AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADCs
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// SetupADCSoftware - Setup ADC channels and acquisition window
//
void SetupADCSoftware(void)
{
    Uint16 acqps;

    //
    // Determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //
    //Select the channels to convert and end of conversion flag
    //ADCA
    //
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC1 will convert pin A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC1 will convert pin A2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;  //SOC1 will convert pin A3
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;  //SOC1 will convert pin A4
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;  //SOC1 will convert pin A5
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps; //sample window is acqps +
                                           //1 SYSCLK cycles
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5; //end of SOC1 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCB
//    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin B2
//    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps +
//                                           //1 SYSCLK cycles
//    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin B3
//    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps +
//                                           //1 SYSCLK cycles
//    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
//    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}

//
// sciaTxFifoIsr - SCIA Transmit FIFO ISR
//
interrupt void sciaTxFifoIsr(void)
{
    //if(sysram.DevMode == SLAVE_MODE)
    {
        if(sciaram.scount < sciaram.slen)
        {
            SciaRegs.SCITXBUF.all = sdataA[sciaram.scount++];
        }
        else
        {
            SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
        }
    }

    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ACK
}

//
// sciaRxFifoIsr - SCIA Receive FIFO ISR
//
interrupt void sciaRxFifoIsr(void)
{

    if(sysram.DevMode == SLAVE_MODE)
    {
        rdataA[sysram.racnt++]=SciaRegs.SCIRXBUF.all;  // Read data via scia
        if(sysram.racnt >= RX_MAX_LEN)
            sysram.racnt = 0;
    }
    else
    {

//        tmp = SciaRegs.SCIRXBUF.all;
        ScibRegs.SCIFFTX.bit.TXFFIENA = 1;
        ScibRegs.SCITXBUF.all = SciaRegs.SCIRXBUF.all;
    }

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}

// scia_fifo_init - Configure SCIA FIFO
void scia_fifo_init()
{
   ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;    // LSPCLK = SYSCLK

   SciaRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                      // No parity,8 char bits,
                                      // async mode, idle-line protocol
   SciaRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
   SciaRegs.SCICTL2.bit.TXINTENA = 1;
   SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
   SciaRegs.SCIHBAUD.all = 0x0000;
   SciaRegs.SCILBAUD.all = 0x0036;
   SciaRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back
   SciaRegs.SCIFFTX.all = 0xC022;
   SciaRegs.SCIFFRX.all = 0x0022;
   SciaRegs.SCIFFCT.all = 0x00;

   SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
   SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}


// scibTxFifoIsr - SCIB Transmit FIFO ISR
//
interrupt void scibTxFifoIsr(void)
{

    {
        if(sciaram.scount < sciaram.slen)
        {
            ScibRegs.SCITXBUF.all = sdataB[sciaram.scount++];
        }
        else
        {
            ScibRegs.SCIFFTX.bit.TXFFIENA = 0;
        }
    }

    ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ACK
}

// scibRxFifoIsr - SCIB Receive FIFO ISR
//
interrupt void scibRxFifoIsr(void)
{

    rdataB[sysram.rbcnt++]=ScibRegs.SCIRXBUF.all;  // Read data
    if(sysram.rbcnt >= RX_MAX_LEN)
        sysram.rbcnt = 0;

    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

//    CpuTimer0Regs.TCR.bit.TRB = 1;//1 = reload timer
//    CpuTimer0Regs.TCR.bit.TSS = 0;//start timer0

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}

// scib_fifo_init - Configure SCIB FIFO
//
void scib_fifo_init()
{
   ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;    // LSPCLK = SYSCLK

   ScibRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                      // No parity,8 char bits,
                                      // async mode, idle-line protocol
   ScibRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
   ScibRegs.SCICTL2.bit.TXINTENA = 1;
   ScibRegs.SCICTL2.bit.RXBKINTENA = 1;
   ScibRegs.SCIHBAUD.all = 0x0000;
   ScibRegs.SCILBAUD.all = 0x001B;
   ScibRegs.SCICCR.bit.LOOPBKENA = 0; // Disable loop back
   ScibRegs.SCIFFTX.all = 0xC022;
   ScibRegs.SCIFFRX.all = 0x0022;
   ScibRegs.SCIFFCT.all = 0x00;

   ScibRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
   ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;
   ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

// cpu_timer0_isr - CPU Timer0 ISR that toggles GPIO32 once per 500ms
//
__interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;


//   sysram.rbflag = 1;
//   CpuTimer0Regs.TCR.bit.TSS = 1;//stop timer0

   GpioDataRegs.GPATOGGLE.bit.GPIO28 = 1;
   GpioDataRegs.GPATOGGLE.bit.GPIO29 = 1;
    //GPIO_WritePin(LED_RX_GPIO, 1);

   //
   // Acknowledge this __interrupt to receive more __interrupts from group 1
   //
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//protocol analysis
//srcadd : the add of the source data buff
//ramadd : the add of the system ram
//len    : the length of the data have reveived
void protocol(unsigned char *srcadd, SYSRAMTYPE *ramadd, Uint16 len)
{
    unsigned char *ptr;
    Uint16 i;

    ptr = srcadd;
    for(i=len; i>0; i--)
    {
        if(*ptr=='A' && *(ptr+1)=='T' && *(ptr+3)=='G' && *(ptr+4)=='S' && *(ptr+5)=='D')
        {
            if(*(ptr+7)=='S' && *(ptr+8)=='T' && *(ptr+9)=='O' && *(ptr+10)=='P')
            {
                ramadd->sampflag = 0;
            }
            else
                ramadd->sampflag = 1;

            break;
        }
        ptr++;
    }

}
//
// End of file
//
