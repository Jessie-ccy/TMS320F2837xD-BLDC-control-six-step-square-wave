/*
 1101_AC & DC DRIVERS
 proj.MID Sixstep01 IOver
 input_
 GPIO6_pin80_MOTOR gray LINE
 GPIO7_pin79_MOTOR green LINE
 GPIO10_pin78_MOTOR yellow LINE

 output_
 GPIO0_pin40_U+
 GPIO1_pin39_U-
 GPIO2_pin38_V+
 GPIO3_pin37_V-
 GPIO4_pin36_W+
 GPIO5_pin35_W-

 GPIO34_TEST
 GPIO31_TEST

 */



#include "F28x_Project.h"
#include "math.h"

// Function declare
void InitEPwm1Example(void); // Initialize EPWM1
void InitEPwm2Example(void); // Initialize EPWM2
void InitEPwm3Example(void); // Initialize EPWM3

void ConfigureADC(void);
void ConfigureSOC(void);
void SetupADCEpwm(void);

__interrupt void adca1_isr(void);

const float SwitchingFreq = 10000; // Switching frequency
const float EPWM_DB = 500;

int state;

//
// Defines
//
#define RESULTS_BUFFER_SIZE 20 //data pool size, 20 data
#define PI 3.14159
//
// Globals
//
Uint16 AdcaResults[RESULTS_BUFFER_SIZE]; //ADC data pool
Uint16 resultsIndex = 0;    //data input index
Uint16 DutyTable[101];  //output table for PWM
Uint16 set_period;  //output data, work with PWM period TimeBase
const Uint16 adcres = 4095;     //2^12-1
//const Uint16 TimeBase = 5000;  //period=0.1ms
Uint16 tableIndex = 0;
Uint16 sumindex = 0;

int TimeBase; // EPWM period time
int TimeBase1 = 0;
int rotatingstate = 2;

void Gpio_setup(void);
void InitECapture(void);
__interrupt void ecap1_isr(void);
void brake(void);
void CCW(void);
void CW(void);

/**
 * main.c
 */
void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

// Step 2. Initialize GPIO pins for ePWM2
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm5Gpio();

    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;  //enable epwm1 clock
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;  //enable epwm2 clock
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;  //enable epwm3 clock

//
// Configure the ADC and power it up
//
    ConfigureADC();
//
// Configure the ePWM SOC
//
    ConfigureSOC();
//
// Setup the ADC for ePWM triggered conversions
//
    SetupADCEpwm();

    Gpio_setup();
    InitECapture();
//
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

//
// Map ISR functions
//
  // This is needed to write to EALLOW protected registers
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    PieVectTable.ECAP1_INT = &ecap1_isr;
    EDIS;

    TimeBase = 50000000 / SwitchingFreq/2;    //  50M/2/10k=2.5k  Page115 for "/2"

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Closed synchronize between base clock and all enabled ePWM modules
    EDIS;

//
// InitEPwm2 - Initialize EPWM2 configuration
//
    InitEPwm1Example();
    InitEPwm2Example();
    InitEPwm3Example();

//
// sync ePWM
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

//
// Enable global Interrupts and higher priority real-time debug events:
//
    IER |= M_INT1; //Enable group 1 interrupts
    IER |= M_INT4;

//
// enable PIE interrupt
//
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM

    Uint16 i; //for index, only used in Main
    for (i = 0; i <= 100; i++)
    {
        DutyTable[i] = TimeBase * sin(PI / 2 * i / 100); // build sin table 0 to 90 degree
    }

    while (1)
    {
        if (rotatingstate == 0){
            CW();
        }
        else if(rotatingstate == 1){
            brake();
        }
        else if(rotatingstate == 2){
            CCW();
        }
        else if (rotatingstate == 3){
            brake();
        }
        else{
            rotatingstate = 0;
        }
    }
}

void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000); //This delay can also be used for kind of low inefficient debounce

    EDIS;
}

//
// SetupADCEpwm - Setup ADC EPWM acquisition window
//
void SetupADCEpwm(void)
{

    //
    //Select the channels to convert and end of conversion flag
    //
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
/*
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 2;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
*/
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //0-> EOC0 is trigger for ADCINT1, will set INT1 flag

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}

//
// ConfigureEPWM - Configure EPWM SOC and compare values
//
void ConfigureSOC(void)                                  ///only used for SOC!!!
{
    EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;    // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 2;   // Select SOC on period
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event
    EPwm1Regs.TBPRD = TimeBase * 0.05;    // Set period to TimeBase/20
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;      // up count mode
    EDIS;
}

void InitEPwm1Example()
{
    EPwm1Regs.TBPRD = TimeBase;                       // Set timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count UPDOWN
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;  // TBCLK=SYSCLKOUT/(HSPCLKDIV¡ÑCLKDIV)

    // Setup compare
    EPwm1Regs.CMPA.bit.CMPA = TimeBase;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;           // If counter>CMPA, output high
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // If counter<CMPA, output low

    EPwm1Regs.CMPB.bit.CMPB = TimeBase;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;           // If counter>CMPA, output high
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // If counter<CMPA, output low

    // Active Low PWMs - Setup Deadband
    //EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;// Set deadband mode: Both rising and falling edge
    //EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;// Active high mode. Neither EPWMxA nor EPWMxB is inverted
    //EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;// EPWMxA In (from the action-qualifier) is the source for both falling-edge and rising-edge delay.
    //EPwm1Regs.DBRED.bit.DBRED = EPWM_DB;// Setting Dead-Band Generator Rising Edge Delay Count Register
    //EPwm1Regs.DBFED.bit.DBFED = EPWM_DB;// Setting Dead-Band Generator Falling Edge Delay Count Register

    // Setup interrupt
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST; // Generate an interrupt on the first event

}

//
// InitEPwm2Example - Initialize EPWM2 configuration
//
void InitEPwm2Example()
{
    EPwm2Regs.TBPRD = TimeBase;                       // Set timer period
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count UPDOWN
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.CMPA.bit.CMPA = TimeBase;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm2Regs.CMPB.bit.CMPB = TimeBase;
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;           // If counter>CMPA, output high
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // If counter<CMPA, output low

    //EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    //EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    //EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    //EPwm2Regs.DBRED.bit.DBRED = EPWM_DB;
    //EPwm2Regs.DBFED.bit.DBFED = EPWM_DB;

    EPwm2Regs.ETSEL.bit.INTEN = 0;                // Disable interrupt

}

//
// InitEPwm3Example - Initialize EPWM3 configuration
//
void InitEPwm3Example()
{
    EPwm3Regs.TBPRD = TimeBase;
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;
    EPwm3Regs.TBCTR = 0x0000;

    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.CMPA.bit.CMPA = TimeBase;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm3Regs.CMPB.bit.CMPB = TimeBase;
    EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;           // If counter>CMPA, output high
    EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // If counter<CMPA, output low

    //EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    //EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    //EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    //EPwm3Regs.DBRED.bit.DBRED = EPWM_DB;
    //EPwm3Regs.DBFED.bit.DBFED = EPWM_DB;

    EPwm3Regs.ETSEL.bit.INTEN = 0;                 // Disable interrupt
}

//
// adca1_isr - Read ADC Buffer in ISR
//
interrupt void adca1_isr(void)
{
    //
    AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT0; //catch the ADC results
    //AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT1; //catch the ADC results
    //
    if (RESULTS_BUFFER_SIZE <= resultsIndex)    //when data full enough
    {
        Uint32 sum = 0; //set 0, also check format

        for (sumindex = 0; sumindex < RESULTS_BUFFER_SIZE; sumindex++)
        {                                  //
            sum += AdcaResults[sumindex]; //sum the result, can work as filtering
        }                                     //
        tableIndex = sum * 100 / adcres / RESULTS_BUFFER_SIZE;                //
        //set_period = DutyTable[tableIndex];
        //EPwm2Regs.CMPA.bit.CMPA = TimeBase - set_period;
        EPwm2Regs.CMPA.bit.CMPA = TimeBase - DutyTable[tableIndex];
        resultsIndex = 0;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void ecap1_isr(void)
{
    //GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;   //Writing a 1 will toggle GPIO0 output data latch 1 to 0 or 0 to 1.
    rotatingstate += 1;
    DELAY_US(2000);
    ECap1Regs.ECCLR.bit.CEVT1 = 1;
    ECap1Regs.ECCLR.bit.INT = 1;
    ECap1Regs.ECCTL2.bit.REARM = 1;

    //
    // Acknowledge this __interrupt to receive more __interrupts from group 4
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
    CpuTimer0Regs.TCR.bit.TSS = 0;         //Restart timer
}


void Gpio_setup(void){
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;       // S1, connect TECO56 yellow signal cable to PIN80
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;     // Select input/output(GPIO) mode
    GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0;     // Synchronous with SYSCLK
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 0;       // Select input mode

    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;       // S2, connect TECO56 ????? signal cable to PIN79
    GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;
    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 0;

    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;       // S3, connect TECO56 ???? signal cable to PIN78
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;
    GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;

    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;   // Enable pullup on GPIO9
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;  // GPIO9 = GPIO9
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;
    GpioDataRegs.GPADAT.bit.GPIO9 == 0;

    EDIS;
}

void InitECapture()
{
    EALLOW;
    InputXbarRegs.INPUT7SELECT = 9;         // Set eCAP1 source to GPIO9
    EDIS;
    ECap1Regs.ECEINT.all = 0x0000;          // Disable all capture __interrupts
    ECap1Regs.ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

    //
    // Configure peripheral registers
    //
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;   // One-shot
    ECap1Regs.ECCTL1.bit.PRESCALE = 0;
    ECap1Regs.ECCTL2.bit.STOP_WRAP = 0;     // Stop at 1 events
    ECap1Regs.ECCTL1.bit.CAP1POL = 0;       // Rising edge

    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
    ECap1Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units
    ECap1Regs.ECEINT.bit.CEVT1 = 1;         // 1 events = __interrupt
}


void brake(void){ //stop

    EALLOW;
    EPwm1Regs.CMPA.bit.CMPA = TimeBase;
    EPwm1Regs.CMPB.bit.CMPB = 0; //U-
    EPwm2Regs.CMPA.bit.CMPA = TimeBase;
    EPwm2Regs.CMPB.bit.CMPB = 0; //V-
    EPwm3Regs.CMPA.bit.CMPA = TimeBase;
    EPwm3Regs.CMPB.bit.CMPB = 0; //W-
    EDIS;
}

void CW(void){
    EALLOW;
          if(GpioDataRegs.GPADAT.bit.GPIO6 == 0 && GpioDataRegs.GPADAT.bit.GPIO7 == 0 && GpioDataRegs.GPADAT.bit.GPIO10 == 1){      // Step1
              EPwm1Regs.CMPA.bit.CMPA = TimeBase;
              EPwm1Regs.CMPB.bit.CMPB = TimeBase;
              EPwm2Regs.CMPA.bit.CMPA = TimeBase;
              EPwm2Regs.CMPB.bit.CMPB = 0; //V-
              EPwm3Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //W+
              EPwm3Regs.CMPB.bit.CMPB = TimeBase;

              EDIS;
              state = 1;
          }
          else if(GpioDataRegs.GPADAT.bit.GPIO6 == 0 && GpioDataRegs.GPADAT.bit.GPIO7 == 1 && GpioDataRegs.GPADAT.bit.GPIO10 == 1){  // Step2
              EPwm1Regs.CMPA.bit.CMPA = TimeBase;
              EPwm1Regs.CMPB.bit.CMPB = 0; //U-
              EPwm2Regs.CMPA.bit.CMPA = TimeBase;
              EPwm2Regs.CMPB.bit.CMPB = TimeBase;
              EPwm3Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //W+
              EPwm3Regs.CMPB.bit.CMPB = TimeBase;


              //GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
              //GpioDataRegs.GPBSET.bit.GPIO34 = 1;
              EDIS;
              state = 2;
          }
          else if(GpioDataRegs.GPADAT.bit.GPIO6 == 0 && GpioDataRegs.GPADAT.bit.GPIO7 == 1 && GpioDataRegs.GPADAT.bit.GPIO10 == 0){  // Step3
              EPwm1Regs.CMPA.bit.CMPA = TimeBase;
              EPwm1Regs.CMPB.bit.CMPB =  0; //U-
              EPwm2Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //V+
              EPwm2Regs.CMPB.bit.CMPB = TimeBase;
              EPwm3Regs.CMPA.bit.CMPA = TimeBase;
              EPwm3Regs.CMPB.bit.CMPB = TimeBase;


              EDIS;
              state = 3;
          }
          else if(GpioDataRegs.GPADAT.bit.GPIO6 == 1 && GpioDataRegs.GPADAT.bit.GPIO7 == 1 && GpioDataRegs.GPADAT.bit.GPIO10 == 0){  // Step4
              EPwm1Regs.CMPA.bit.CMPA = TimeBase;
              EPwm1Regs.CMPB.bit.CMPB = TimeBase;
              EPwm2Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //V+
              EPwm2Regs.CMPB.bit.CMPB = TimeBase;
              EPwm3Regs.CMPA.bit.CMPA = TimeBase;
              EPwm3Regs.CMPB.bit.CMPB = 0; //W-


              //GpioDataRegs.GPASET.bit.GPIO31 = 1;
              //GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
              EDIS;
              state = 4;
          }
          else if(GpioDataRegs.GPADAT.bit.GPIO6 == 1 && GpioDataRegs.GPADAT.bit.GPIO7 == 0 && GpioDataRegs.GPADAT.bit.GPIO10 == 0){  // Step5
              EPwm1Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //U+
              EPwm1Regs.CMPB.bit.CMPB = TimeBase;
              EPwm2Regs.CMPA.bit.CMPA = TimeBase;
              EPwm2Regs.CMPB.bit.CMPB = TimeBase;
              EPwm3Regs.CMPA.bit.CMPA = TimeBase;
              EPwm3Regs.CMPB.bit.CMPB = 0; //W-


              EDIS;
              state = 5;
          }
          else if(GpioDataRegs.GPADAT.bit.GPIO6 == 1 && GpioDataRegs.GPADAT.bit.GPIO7 == 0 && GpioDataRegs.GPADAT.bit.GPIO10 == 1){  // Step6
              EPwm1Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //U+
              EPwm1Regs.CMPB.bit.CMPB = TimeBase;
              EPwm2Regs.CMPA.bit.CMPA = TimeBase;
              EPwm2Regs.CMPB.bit.CMPB = 0; //V-
              EPwm3Regs.CMPA.bit.CMPA = TimeBase;
              EPwm3Regs.CMPB.bit.CMPB = TimeBase;


              //GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
              //GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
              EDIS;
              state = 6;
          }
      }
void CCW(void){
    EALLOW;
          if(GpioDataRegs.GPADAT.bit.GPIO6 == 1 && GpioDataRegs.GPADAT.bit.GPIO7 == 1 && GpioDataRegs.GPADAT.bit.GPIO10 == 0){      // Step1
              EPwm1Regs.CMPA.bit.CMPA = TimeBase;
              EPwm1Regs.CMPB.bit.CMPB = TimeBase;
              EPwm2Regs.CMPA.bit.CMPA = TimeBase;
              EPwm2Regs.CMPB.bit.CMPB = 0; //V-
              EPwm3Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //W+
              EPwm3Regs.CMPB.bit.CMPB = TimeBase;
              EDIS;
              state = 1;
          }
          else if(GpioDataRegs.GPADAT.bit.GPIO6 == 0 && GpioDataRegs.GPADAT.bit.GPIO7 == 1 && GpioDataRegs.GPADAT.bit.GPIO10 == 0){  // Step2
              EPwm1Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //U+
              EPwm1Regs.CMPB.bit.CMPB = TimeBase;
              EPwm2Regs.CMPA.bit.CMPA = TimeBase;
              EPwm2Regs.CMPB.bit.CMPB = 0; //V-
              EPwm3Regs.CMPA.bit.CMPA = TimeBase;
              EPwm3Regs.CMPB.bit.CMPB = TimeBase;

              //GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
              //GpioDataRegs.GPBSET.bit.GPIO34 = 1;
              EDIS;
              state = 2;
          }
          else if(GpioDataRegs.GPADAT.bit.GPIO6 == 0 && GpioDataRegs.GPADAT.bit.GPIO7 == 1 && GpioDataRegs.GPADAT.bit.GPIO10 == 1){  // Step3
              EPwm1Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //U+
              EPwm1Regs.CMPB.bit.CMPB = TimeBase;
              EPwm2Regs.CMPA.bit.CMPA = TimeBase;
              EPwm2Regs.CMPB.bit.CMPB = TimeBase;
              EPwm3Regs.CMPA.bit.CMPA = TimeBase;
              EPwm3Regs.CMPB.bit.CMPB = 0; //W-

              EDIS;
              state = 3;
          }
          else if(GpioDataRegs.GPADAT.bit.GPIO6 == 0 && GpioDataRegs.GPADAT.bit.GPIO7 == 0 && GpioDataRegs.GPADAT.bit.GPIO10 == 1){  // Step4
              EPwm1Regs.CMPA.bit.CMPA = TimeBase;
              EPwm1Regs.CMPB.bit.CMPB = TimeBase;
              EPwm2Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //V+
              EPwm2Regs.CMPB.bit.CMPB = TimeBase;
              EPwm3Regs.CMPA.bit.CMPA = TimeBase;
              EPwm3Regs.CMPB.bit.CMPB = 0; //W-

              //GpioDataRegs.GPASET.bit.GPIO31 = 1;
              //GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
              EDIS;
              state = 4;
          }
          else if(GpioDataRegs.GPADAT.bit.GPIO6 == 1 && GpioDataRegs.GPADAT.bit.GPIO7 == 0 && GpioDataRegs.GPADAT.bit.GPIO10 == 1){  // Step5
              EPwm1Regs.CMPA.bit.CMPA = TimeBase;
              EPwm1Regs.CMPB.bit.CMPB = 0; //U-
              EPwm2Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //V+
              EPwm2Regs.CMPB.bit.CMPB = TimeBase;
              EPwm3Regs.CMPA.bit.CMPA = TimeBase;
              EPwm3Regs.CMPB.bit.CMPB = TimeBase;

              EDIS;
              state = 5;
          }
          else if(GpioDataRegs.GPADAT.bit.GPIO6 == 1 && GpioDataRegs.GPADAT.bit.GPIO7 == 0 && GpioDataRegs.GPADAT.bit.GPIO10 == 0){  // Step6
              EPwm1Regs.CMPA.bit.CMPA = TimeBase;
              EPwm1Regs.CMPB.bit.CMPB = 0; //U-
              EPwm2Regs.CMPA.bit.CMPA = TimeBase;
              EPwm2Regs.CMPB.bit.CMPB = TimeBase;
              EPwm3Regs.CMPA.bit.CMPA = TimeBase1 + DutyTable[tableIndex]; //W+
              EPwm3Regs.CMPB.bit.CMPB = TimeBase;


              //GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
              //GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
              EDIS;
              state = 6;
          }
      }

//
// End of file
//
