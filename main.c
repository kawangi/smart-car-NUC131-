//Servo & Motor Control Demo & CAM driver
//Servo turning from -90 to 90
//Motor will run & stop upon INT1 key is pressed


//CAM Signals: CLK, SI, AO
//Read AD for 128bit pixels

//PA0 (LED1) : TMR interrupt
//PA1 (LED2) : indicate an ADC4 read cycle
//PA2 (LED3) : CLK
//PA3 (LED4) : SI
//PA4 (ADC4) : AO output
//PC3 (LED5) : indicate CAM_AO digital output directly
//PC2 (LED6) : PID operation indication
//PB5 (LED_BOARD): START_STOP_LED


//SERVO control
//PF4 (PWM1_CH4) = SERVO PWM control

//Motor IOs:
//Motor1_EN; set = 1 always in the HW
//PA12 (IN2); to be set with PWM
//PA13 (IN1); always = 0

//PWM parameters are set with resolution at 0.1% (range = 0-1000) after modification of BPWM.c and PWM.c in the lower layer


//UART BAUD = 9600
//The program will:
//1. Echo the character "x" + "ok" upon receiving this character from the UART
//2. If "a" is received, "0123" will be sent to the UART in additional
//3. If "b" is received, "4567" will be sent to the UART in additional


#include "NUC131.h"
#include "SERVO_PID.h"
//Lamp
#define TAIL_LAMP		PB12
#define HEAD_LAMP		PF5

#define LEFT_LAMP		PF8
#define RIGHT_LAMP		PB8


#define BUZZER 			PA11

#define SI 				PB2
#define CLK 			PB3

#define CAMAO			PA0

#define SERVOPWM 		PWM0

#define TEST_LED1		PA2
#define TEST_LED2		PA3
#define TEST_LED3		PA4

#define MOTOR_IN1		PF4

#define COLLSION_SWITCH		PC6
#define START_BUTTON		PB15

//CAM driver parameters
#define TIMER_PERIOD 		45 // Timer period in ms, Continuous exposure time
#define HALF_CLOCK_CYCLE 	500
#define B_W_THRESHOLD_MIN 	1000 //Min B&W threshold to detect the black line to stop
#define FINISH_LINE 		2300 //Black finish line average ADC value threshold

//Motor Control Settings
#define MOTOR_FREQ 		100
#define MOTOR_SPEED_MAX 1000
#define MOTOR_SPEED_MIN 300
#define TIME_TO_STOP 	1250 // in unit of 0.8us
#define STOP 			0

//Servo Control Settings
#define SERVO_FREQ 		50
#define SERVO_CENTRE 	85 //1520us, need to change if SERVO_FREQ is changed
#define SERVO_MAX 		SERVO_CENTRE+18 //need to change if SERVO_FREQ is changed
#define SERVO_MIN 		SERVO_CENTRE-18 //need to change if SERVO_FREQ is changed

//SERVO parameters
short int 	servo_pwm = SERVO_CENTRE;

//Motor speed parameters
short int 	speed=MOTOR_SPEED_MAX; // Max value = 1000
short int 	motor_status=0; // 0=STOP, 1=START
short int 	speed_factor = 1;
int 		pause = 100000;
short int 	ticks_per_sec; //for timer ticks


int Final_Left[64];
int Final_Right[64];
int L_Dis;
int R_Dis;
int Final_Dis;
int Last_Final_Dis =0;
int PID_Value;

int ADC_temResult[128];
int32_t  ADC_tem =0;
int ADC_MAX = 0, ADC_MIN = 4000;

int PID_COUNTER = 0;

int TIMER_STATUS = 0;
int uart_count=0;

unsigned int UART_RX_Result_Byte      = 0;
uint8_t      UART_RX_Result_Buffer[131] = {};
// 1 start 1 stop, 16 camera, 1 distance
uint8_t      UART_Bit_Buffer[19] = {};

void UART3_IRQHandler(void)
{
    uint32_t     u32IntSts= UART3->ISR;
    uint8_t      Temp_UART_RX_Result_Buffer[1] = {0};
    unsigned int Temp_UART_RX_Result_Byte      = 0;

    if(u32IntSts & UART_IS_RX_READY(UART3))
    {
       	UART_Read(UART3, Temp_UART_RX_Result_Buffer, sizeof(Temp_UART_RX_Result_Buffer));
       	Temp_UART_RX_Result_Byte = (unsigned int)Temp_UART_RX_Result_Buffer[0];
       	UART_Write(UART3, Temp_UART_RX_Result_Buffer, sizeof(Temp_UART_RX_Result_Buffer));
       	UART_Write(UART3, " ok ", 4);
       	UART_RX_Result_Byte = Temp_UART_RX_Result_Byte;
    }
}

//Create a wait state loop function in micro second
void wait(int time_0_5us)
{
	int aaa=0;
	for (aaa=0; aaa<time_0_5us; aaa++)
	{}
}

void SYS_Init(void)
{
	//-------------------------------------------------------------------------------
	// Init System Clock
	//-------------------------------------------------------------------------------

	// Enable Internal RC 22.1184MHz clock
	CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

	// Waiting for Internal RC clock ready
	CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

	// Switch HCLK clock source to Internal RC and HCLK source divide 1
	CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

	// Enable external XTAL 12MHz clock
	CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

	// Waiting for external XTAL clock ready
	CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

	// Set core clock as PLL_CLOCK from PLL
	CLK_SetCoreClock(50000000);


	// ADC clock source is 22.1184MHz, set divider to 1, ADC clock is 22.1184/7 MHz
	CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));

	// Enable module clock
	CLK_EnableModuleClock(ADC_MODULE);

	// Disable the PA0 digital input path to avoid the leakage current.
	GPIO_DISABLE_DIGITAL_PATH(PA, 0x01);

    //******************For BPWM & PWM**********************
    // Select IP clock source
	CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0_S_PCLK, 0);//Enable PWM0
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL3_PWM1_S_PCLK, 0);//Enable PWM1

    // Enable IP clock
    CLK_EnableModuleClock(PWM0_MODULE);//Enable PWM0
    CLK_EnableModuleClock(PWM1_MODULE);//Enable PWM1

    /* Reset BPWM0,PWM0,1 */
    SYS_ResetModule(PWM0_RST);
    SYS_ResetModule(PWM1_RST);
    //***********************************************

	// Enable Timer 0,1 module clock
	CLK_EnableModuleClock(TMR0_MODULE);
	//CLK_EnableModuleClock(TMR1_MODULE);

	// Select Timer 0,1 module clock source
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HCLK, NULL);
	//CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR0_S_HCLK, NULL);


    //==================================================
    // Init Module Clock - UART3
    //==================================================
    // Select UART1 module clock source
    CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    // Enable UART1 module clock
    CLK_EnableModuleClock(UART3_MODULE);
    // Reset UART1
    SYS_ResetModule(UART3_RST);


    //---------------------------------------------------------------------------------------------------------
    // Init I/O Multi-function
    //---------------------------------------------------------------------------------------------------------
    // Set GPF multi-function pins for PWM1 Channel4 (PF4), PWM0 CH0 (PA12), ICE interface
	// Set GPB multi-function pins for external interrupt

     SYS->ALT_MFP3 = SYS_ALT_MFP3_PF4_PWM1_CH4 | SYS_ALT_MFP3_PA1_PWM0_CH5;
     SYS->ALT_MFP4 = SYS_ALT_MFP4_PA6_UART3_TXD | SYS_ALT_MFP4_PA5_UART3_RXD;
     SYS->GPA_MFP = SYS_GPA_MFP_PA6_UART3_TXD | SYS_GPA_MFP_PA5_UART3_RXD | SYS_GPA_MFP_PA1_PWM0_CH5 | SYS_GPA_MFP_PA0_ADC0;

     SYS->GPF_MFP = SYS_GPF_MFP_PF7_ICE_DAT | SYS_GPF_MFP_PF6_ICE_CLK | SYS_GPF_MFP_PF4_PWM1_CH4;



}

void SYS_Exit(void)
{
	UART_Close(UART1);
}

void Startup_Init(void)
{

    // LED outputs
    GPIO_SetMode(PB, BIT12, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PF, BIT5, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT2, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT3, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT4, GPIO_PMD_OUTPUT);

    GPIO_SetMode(PF, BIT8, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PB, BIT8, GPIO_PMD_OUTPUT);

    //Set buzzer
    GPIO_SetMode(PA, BIT11, GPIO_PMD_OUTPUT);

    //Set Collison switch
    GPIO_SetMode(PC, BIT6, GPIO_PMD_INPUT);

    //Set PB15 with input pull-up
    GPIO_SetMode(PB, BIT15, GPIO_PMD_QUASI);

    //IO Init
    //LED on NUC131 board; START_STOP_LED off
    SI = 0; 			//SI is low
    CLK = 0; 			//CLK is low
    MOTOR_IN1 = 0;		//Motor IN1 = low


    //*********************PWM configure*************************
    PWM_ConfigOutputChannel(PWM1, 4, MOTOR_FREQ, speed);// Motor control, set to target speed
    PWM_ConfigOutputChannel(PWM0, 5, SERVO_FREQ, SERVO_CENTRE);// Servo PWM control

    PWM_EnableOutput(PWM1, 0x10);
    PWM_EnableOutput(PWM0, 0x20);

    PWM_Stop(PWM1, 0x10); // Stop the motor at start
    PWM_Start(PWM0, 0x20);
    //***********************************************************

	// Init (UART3)
	UART_Open(UART3, 38400);
	UART_EnableInt(UART3, (UART_IER_RDA_IEN_Msk));
	NVIC_EnableIRQ(UART3_IRQn);
}


int Boundary(int val){

	val = val -2;
	if (val > SERVO_MAX)
		val = SERVO_MAX;

	if (val < SERVO_MIN)
		val = SERVO_MIN;

	return val;

}


void SERVO_CONTROL(int val){

	PWM_ConfigOutputChannel(PWM1,4, MOTOR_FREQ, speed-(speed*abs((val-SERVO_CENTRE))/100));
	//PWM_ConfigOutputChannel(PWM0, 0, MOTOR_FREQ, speed-(speed*abs((val-76))/160));
	//PWM_ConfigOutputChannel(PWM0, 0, MOTOR_FREQ, speed);
	PWM_ConfigOutputChannel(PWM0,5, SERVO_FREQ, val);

}

void bit_pack()
{
	for(int i =0; i<16; i++)
	{
		int temp=UART_RX_Result_Buffer[i*8+8];
		for(int j =6; j>=0; j--)
		{
			// 1 start 1 stop, 16 camera, 1 distance
			if (UART_RX_Result_Buffer[i*8+j+1] == 1)
				temp = temp*2+1;
			else
				temp = temp*2;

		}
		UART_Bit_Buffer[i+1] = temp;
	}
}

void UART_SEND(){

	if(UART_Bit_Buffer[0] != 85)
		return;
	if(UART_Bit_Buffer[18] != 170)
		return;

	UART_Write(UART3, UART_Bit_Buffer, sizeof(UART_Bit_Buffer));
	UART_RX_Result_Byte=0x00; //clear UART_RX_Byte
//
//	for(int i = 0; i < 19 ; i ++)
//		UART_Bit_Buffer[i] = 0;
}


//Timer0 Interrupt Service Routine
void TMR0_IRQHandler(void)
{
    // Clear Timer0 time-out interrupt flag
    TIMER_ClearIntFlag(TIMER0);

    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    // LED1 for Timer Interrupt Event

	wait(500);
	// Add your codes here

	ADC_MAX = 0;
	ADC_MIN = 4000;

	SI = 1;
	wait(200);

	CLK = 1;
	wait(100);
	ADC_START_CONV(ADC);
	while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));	// Wait conversion done
	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
	ADC_tem = ADC_GET_CONVERSION_DATA(ADC,4);
	ADC_temResult[0] = ADC_tem;
	if (ADC_tem > ADC_MAX)
		ADC_MAX = ADC_tem;

	if (ADC_tem < ADC_MIN)
		ADC_MIN = ADC_tem;

	wait(100);

	SI = 0;
	wait(200);

	CLK=0;



	for (int var = 1; var < 128; ++var) {
		wait(200);
		CLK = 1;
		wait(100);
		ADC_START_CONV(ADC);
		while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));	// Wait conversion done
		ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
		ADC_tem = ADC_GET_CONVERSION_DATA(ADC,4);
		ADC_temResult[var] = ADC_tem;
		if (ADC_tem > ADC_MAX)
			ADC_MAX = ADC_tem;

		if (ADC_tem < ADC_MIN)
			ADC_MIN = ADC_tem;

		wait(100);
		CLK= 0;
	}



	// CHECK FINAL BLACK LINE



	int diff = (ADC_MAX - ADC_MIN) *0.7;

	int  avg_threshold = 0;

	for (int var = 0; var < 128;++var){
		avg_threshold += ADC_temResult[var];
		if ((ADC_temResult[var]-ADC_MIN) > diff){
			if (var < 65)
				Final_Left[var] = 1;

			if (var > 64)
				Final_Right[var - 65] = 1;

			//TEST_LED1 = 1;

			UART_RX_Result_Buffer[var+1] = 1;

		}
		else{
			if (var < 65)
				Final_Left[var] = 0;

			if (var > 64)
				Final_Right[var - 65] = 0;

			//TEST_LED1 = 0;
			UART_RX_Result_Buffer[var+1] = 0;
		}

		wait(200);
	}



	if (avg_threshold/128 < 2000){

		TIMER_STATUS = 1;

	}

	// Get the R and L distance
	L_Dis = 64;
	R_Dis = 64;

	int intRcount = 0;

	for (int var = 64; var > 0; --var){
		if (var > 2)
			if (L_Dis == 64)
				if (Final_Left[var] == 0 && Final_Left[var-1] == 0 && Final_Left[var -2] == 0){
					L_Dis = 63 - var;
				}else
					L_Dis = 64;

		if (intRcount <= 61)
			if (R_Dis == 64)
				if (Final_Right[intRcount] == 0 && Final_Right[intRcount+1] == 0 && Final_Right[intRcount+2] == 0){
					R_Dis = intRcount;
				}else
					R_Dis = 64;

		if ((L_Dis != 64) && (R_Dis != 64))
			break;
		intRcount++;
	}


		Final_Dis = R_Dis - L_Dis;




		if(++uart_count>5)
		{
			for (int k = 0 ; k<19;k++)
			{
				UART_Bit_Buffer[k] = 0;
			}
			bit_pack();
			UART_Bit_Buffer[0] = 85;
			UART_Bit_Buffer[17] = Final_Dis;
			UART_Bit_Buffer[18] = 170;


			UART_SEND(UART_Bit_Buffer);
						uart_count=0;
		}


		if (abs(Last_Final_Dis - Final_Dis)<30){
		//Calculate the PID
			PID_Value = control(Final_Dis);
			int TEM_SERO = SERVO_CENTRE - PID_Value/120 ;
			SERVO_CONTROL(Boundary(TEM_SERO));
			Last_Final_Dis = Final_Dis;

			if (PID_COUNTER == 30){
				//UART_SEND(PID_Value,Final_Dis,Boundary(TEM_SERO));
				PID_COUNTER = 0;
			}


		}

	PID_COUNTER++;

   //OFF LED for TMR0
}




// Key interrupt service routine
// Manual START/STOP key
void EINT1_IRQHandler(void)
{
	//Disable Timer0 INT to mask Timer0
    NVIC_DisableIRQ(TMR0_IRQn);

    GPIO_CLR_INT_FLAG(PB, BIT15);//Clear EINT1 flag


    //Stop the motor
    if ( motor_status == 0)
 	   motor_status = 1;
    else
 	   motor_status = 0;



    // Enable Timer0 NVIC
    NVIC_EnableIRQ(TMR0_IRQn);
}


int32_t main(void)
{
	SYS_UnlockReg();
    SYS_Init();
    SYS_LockReg();

    Startup_Init();


    //*********************Init Timer0***************************
	// Open Timer0 in periodic mode, enable interrupt, # of ticks per second
    NVIC_SetPriority(TMR0_IRQn,1); //set INT priority lower (=1)
	ticks_per_sec=1000/TIMER_PERIOD;
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, ticks_per_sec);
    TIMER_EnableInt(TIMER0);
    // Enable Timer0 NVIC
    NVIC_EnableIRQ(TMR0_IRQn);
    // Start Timer0 counting
    TIMER_Start(TIMER0);
	//***********************************************************


    //*********************Init EXT INT1*************************
       // Configure PB.15 as EINT1 pin and enable interrupt by falling edge trigger
   	// Manual START/STOP key
       GPIO_EnableEINT0(PB, 15, GPIO_INT_FALLING);
       NVIC_EnableIRQ(EINT1_IRQn);
       // Enable interrupt de-bounce function and
       // Select de-bounce sampling cycle time is 1024 clocks of LIRC clock
       GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_1);
       GPIO_ENABLE_DEBOUNCE(PB, BIT15);
   	//********************************************************

    // Set the ADC operation mode as continuous scan, input mode as single-end and enable the analog input channel 4 only
       ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE , 0x10);
       ADC_POWER_ON(ADC); // Power on ADC module

       setPID(80,0,0,24);
       //setPID(80,0,0,10);
      // setPID(80,0,0,5);

       PWM_ConfigOutputChannel(PWM0, 5, SERVO_FREQ, SERVO_CENTRE);
       HEAD_LAMP = 1;
       TAIL_LAMP = 0;
       TEST_LED1 = 0;
       TEST_LED2 = 0;
       TEST_LED3 = 0;
       RIGHT_LAMP = 0;
       LEFT_LAMP = 0;
       BUZZER = 0;



    while(1)
    {
    	//****************************SERVO CONTROL*****************************************
    	//Centre position: pulse width = 1.52ms
    	//PWM_ConfigOutputChannel(PWM1, 4, SERVO_FREQ, SERVO_CENTRE);

    	wait(200000);


    	if ((GPIO_GET_IN_DATA(PC)&(1<<6)) == 0)
    	{
    		PWM_ConfigOutputChannel(PWM1, 4, MOTOR_FREQ, 0);
    		PWM_Stop(PWM1, 0x10);

    	}
    	else if ((GPIO_GET_IN_DATA(PC)&(1<<6)) == 1)
    	{
    		PWM_ConfigOutputChannel(PWM1, 4, MOTOR_FREQ, 300);
    		PWM_Start(PWM1, 0x10);

    	}

	}

    SYS_Exit();

    return 0;
}
