//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "fatfs.h"
#include "usb_host.h"
#include "stm32f4xx_hal.h"
#include "Timer.h"
#include "BlinkLed.h"
#include "PlayMP3.h"
#include "cortexm/ExceptionHandlers.h"
#include "generic.h"
#include "timeKeeping.h"
#include "DebugPort.h"
#include "AudioChip.h"

#include <stm32f4xx_hal_rtc.h>




// Define display segment Pins and decimal point (E8 - E15, Anodes)
#define Segment_A                 ( GPIO_PIN_8 )
#define Segment_B                 ( GPIO_PIN_9 )
#define Segment_C                 ( GPIO_PIN_10 )
#define Segment_D                 ( GPIO_PIN_11 )
#define Segment_E                 ( GPIO_PIN_12 )
#define Segment_F                 ( GPIO_PIN_13 )
#define Segment_G                 ( GPIO_PIN_14 )
#define Dp                        ( GPIO_PIN_15 )


// Define display digits and L-pin (D4 - D10, Cathodes)
#define Digit_1                   ( GPIO_PIN_4 )
#define Digit_2                   ( GPIO_PIN_7 )
#define Digit_3                   ( GPIO_PIN_8 )
#define Digit_4                   ( GPIO_PIN_9 )
#define L                         ( GPIO_PIN_10 )


//Define push buttons (Port C)
#define Push_Button_1             ( GPIO_PIN_4 ) // 12-24 Hour Format
#define Push_Button_2             ( GPIO_PIN_5 )
#define Push_Button_3             ( GPIO_PIN_8 )
#define Push_Button_4             ( GPIO_PIN_9 )
#define Push_Button_5             ( GPIO_PIN_11 )



//	#define 	RTC_HOURFORMAT_24   ((uint32_t)0x00000000)
//#define 	RTC_HOURFORMAT_12   ((uint32_t)0x00000040)




GPIO_InitTypeDef	GPIOInitStruct;
//-----------------------------EXTI_InitTypeDef	EXTI_InitStructure;----------

//
// Disable specific warnings
//

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"



// ----------------------------------------------------------------------------
//
// Standalone STM32F4 Simple Alarm Clock Stub Code
//
// This code just plays an MP3 file off of a connected USB flash drive.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

void
	Display7Segment( void ),
	Display7Segment_Alarm(void ),
	DisplayClockMode( void ),
	CheckFlags( void),
//	ConvertTo24Hour( void ),
	SetTime( void ),
	SetAlarm( void ),
	CheckAlarm( void),
//	Snooze( void ),
	ProcessButtons( void ),
	GetCurrentTime( void ),
	GetAlarmTime( void ),
	SystemClock_Config( void ),
	MX_GPIO_Init( void ),
	MX_I2C1_Init( void ),
	MX_USB_HOST_Process( void );

//uint16_t
//	CheckButtons( void );


// STMCube Example declarations.
// static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

static void
	MSC_Application(void);

//static void
	//Error_Handler(void);

//
// Global variables


RTC_InitTypeDef
	ClockInit,
	ClockInit2;// Structure used to initialize the real time clock


RTC_TimeTypeDef
	ClockTime,
	AlarmTime;// Structure to hold/store the current time

RTC_DateTypeDef
	ClockDate;				// Structure to hold the current date

RTC_AlarmTypeDef
	ClockAlarm;				// Structure to hold/store the current alarm time

TIM_HandleTypeDef
	Timer6_44Khz,			// Structure for the audio play back timer subsystem
	DisplayTimer, 			//For multiplexing each digit
	SecondsTimer,
	ButtonPushCheck;        // interrupt that checks for button pushes



							// Structure for the LED display timer subsystem

DAC_HandleTypeDef
	AudioDac;				// Structure for the audio digital to analog converter subsystem

DMA_HandleTypeDef
	AudioDma;				// Structure for the audio DMA direct memory access controller subsystem

RTC_HandleTypeDef
	RealTimeClock;			// Structure for the real time clock subsystem

I2C_HandleTypeDef			// Structure for I2C subsystem. Used for external audio chip
	I2c;

volatile int
	DisplayClockModeCount,	// Number of display ticks to show the current clock mode time format
	PlayMusic = FALSE,		// Flag indicating if music should be played
	DebounceCount = 0;		// Buttons debounce count

volatile uint16_t
	ButtonsPushed;			// Bit field containing the bits of which buttons have been pushed

FATFS
	UsbDiskFatFs;			// File system object for USB disk logical drive

char
	UsbDiskPath[4];			// USB Host logical drive path

int
	BcdTime[5],
	BcdAlarmTime[5],// Array to hold the hours and minutes in BCD format
	DisplayedDigit = 0,
	DisplayedDigit_Alarm = 0,// Current digit being displayed on the LED display

							// Current format for the displayed time ( IE 12 or 24 hour format )
	ClockHourFormat = CLOCK_HOUR_FORMAT_12, //
	AlarmAMFlag = 1,
	TimeAMFlag = 1;

//____________________________________Hola (Global Vars)_________________________________________
void default_state( void),
		alarm_setting_state(void);	//Default state for setting time

void check_current_state( void) ;


int current_state = 0; //Current state of execution
					  //current_state = 0 -----> Display Time
					  //current_state = 1 -----> Set Alarm

RTC_HandleTypeDef rtcHandle;
RTC_HandleTypeDef rtcHandle2;

int hola = 0;
int debouncer = 0;
int debouncer2 = 0;

int ric = 0,
	ric2 = 0,
	Tairan,
	Phil = 0,
	SnoozeCounter = 300,
	SnoozeVar = 0;





//____________________________________Hola (Global Vars)_________________________________________


//
// Functions required for long files names on fat32 partitions
//

WCHAR ff_convert (WCHAR wch, UINT dir)
{
	if (wch < 0x80)
	{
//
// ASCII Char
//
		return wch;
	}

//
// unicode not supported
//
	return 0;
}

WCHAR ff_wtoupper (WCHAR wch)
{
	if (wch < 0x80)
	{
//
// ASCII Char
//
		if (wch >= 'a' && wch <= 'z')
		{
			wch &= ~0x20;
		}

		return wch;
	}

//
// unicode not supported
//
	return 0;
}









//
// Dummy interrupt handler function
//
void TIM6_DAC_IRQHandler(void)
{
	HAL_NVIC_DisableIRQ( TIM6_DAC_IRQn );
}



/*
 * Function: ConfigureAudioDma
 *
 * Description:
 *
 * Initialize DMA, DAC and timer 6 controllers for a mono channel audio to be played on PA4
 *
 */

void ConfigureAudioDma( void )
{

	TIM_MasterConfigTypeDef
		Timer6MasterConfigSync;

	GPIO_InitTypeDef
		GPIO_InitStructure;

	DAC_ChannelConfTypeDef
		DacConfig;

//
// If we have the timer 6 interrupt enabled then disable the timer from running when we halt the processor or hit a breakpoint.
// This also applies to printing using the semihosting method which also uses breakpoints to transfer data to the host computer
//


	__HAL_DBGMCU_UNFREEZE_TIM5();

//
// Enable the clocks for GPIOA, GPIOC and Timer 6
//
	__HAL_RCC_TIM6_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();


//
// Configure PA4 as an analog output ( used for D/A output of the analog signal )
//

	GPIO_InitStructure.Pin = GPIO_PIN_4;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStructure.Alternate = 0;
	HAL_GPIO_Init( GPIOA, &GPIO_InitStructure);




//
// Configure timer 6 for a clock frequency of 44Khz and a triggered output for the DAC
//
	Timer6_44Khz.Instance = TIM6;
	Timer6_44Khz.Init.Prescaler = 20; //this value may have to be changed
	Timer6_44Khz.Init.CounterMode = TIM_COUNTERMODE_UP;
	Timer6_44Khz.Init.Period = 90; // this value may have to be changed
	Timer6_44Khz.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init( &Timer6_44Khz );

	Timer6MasterConfigSync.MasterOutputTrigger = TIM_TRGO_UPDATE;
	Timer6MasterConfigSync.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization( &Timer6_44Khz, &Timer6MasterConfigSync );

//
// Set the priority of the interrupt and enable it
//
	NVIC_SetPriority(TIM6_DAC_IRQn, 0);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

//
// Clear any pending interrupts
//
	__HAL_TIM_CLEAR_FLAG( &Timer6_44Khz, TIM_SR_UIF );



//
// Enable the timer interrupt and the DAC Trigger
//

	__HAL_TIM_ENABLE_DMA( &Timer6_44Khz, TIM_DIER_UDE );


//
// Enable the clocks for the DAC
//
	__HAL_RCC_DAC_CLK_ENABLE();

	AudioDac.Instance = DAC;
	if ( HAL_OK != HAL_DAC_Init( &AudioDac ))
	{
		trace_printf("DAC initialization failure\n");
		return;
	}

//
// Enable the trigger from the DMA controller and the output buffer of the DAC
//
	DacConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	DacConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

	if ( HAL_DAC_ConfigChannel(&AudioDac, &DacConfig, DAC_CHANNEL_1) != HAL_OK )
	{
		trace_printf("DAC configuration failure\n");
		return;
	}

//
// Enable the clock for the DMA controller
//
	__HAL_RCC_DMA1_CLK_ENABLE();

//
// Initialize the stream and channel number and the memory transfer settings
//

    AudioDma.Instance = DMA1_Stream5;
    AudioDma.Init.Channel = DMA_CHANNEL_7;
    AudioDma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    AudioDma.Init.PeriphInc = DMA_PINC_DISABLE;
    AudioDma.Init.MemInc = DMA_MINC_ENABLE;
    AudioDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    AudioDma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    AudioDma.Init.Mode = DMA_NORMAL;
    AudioDma.Init.Priority = DMA_PRIORITY_MEDIUM;
    AudioDma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init( &AudioDma );

//
// Link the DMA channel the to the DAC controller
//
    __HAL_LINKDMA( &AudioDac, DMA_Handle1, AudioDma );

//
// Enable the interrupt for the specific stream
//
    HAL_NVIC_SetPriority( DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ( DMA1_Stream5_IRQn );

//
// Start the timer
//
	__HAL_TIM_ENABLE( &Timer6_44Khz );

	return;
}


void ConfigureDisplay( void )
{

//
//  Enable clocks for PWR_CLK, GPIOE, GPIOD, GPIOC.


	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();



	//ENABLE RTC CLOCK

//here.........

// Enable clock for TIM5.
	__HAL_RCC_TIM5_CLK_ENABLE();
//
	DisplayTimer.Instance = TIM5;
			DisplayTimer.Init.Period = 799;// (original value was 39999 / 4 seconds) period & prescaler combination for half second count
			DisplayTimer.Init.Prescaler = 99; // (original value was 8399)
			DisplayTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
			DisplayTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
			HAL_TIM_Base_Init( &DisplayTimer );

			HAL_NVIC_SetPriority( TIM5_IRQn, 0, 0);//set priority for the interrupt. Value 0 corresponds to highest priority
			HAL_NVIC_EnableIRQ( TIM5_IRQn );//Enable interrupt function request of Timer5

			__HAL_TIM_ENABLE_IT( &DisplayTimer, TIM_IT_UPDATE );// Enable timer interrupt flag to be set when timer count is reached
			__HAL_TIM_ENABLE( &DisplayTimer );//Enable timer to start

		    HAL_TIM_Base_Init(&DisplayTimer);
		    HAL_TIM_Base_Start(&DisplayTimer);


//timer 3 checks for buttonpushes

/*
			__HAL_RCC_TIM3_CLK_ENABLE();

			ButtonPushCheck.Instance = TIM3;
			ButtonPushCheck.Init.Period = 899;// (original value was 39999 / 4 seconds) period & prescaler combination for half second count
			ButtonPushCheck.Init.Prescaler = 8398; // (original value was 8399)
			ButtonPushCheck.Init.CounterMode = TIM_COUNTERMODE_UP;
			ButtonPushCheck.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
			HAL_TIM_Base_Init( &ButtonPushCheck );

			HAL_NVIC_SetPriority( TIM3_IRQn, 0, 0);//set priority for the interrupt. Value 0 corresponds to highest priority
			HAL_NVIC_EnableIRQ( TIM3_IRQn );//Enable interrupt function request of Timer5

			__HAL_TIM_ENABLE_IT( &ButtonPushCheck, TIM_IT_UPDATE );// Enable timer interrupt flag to be set when timer count is reached
			__HAL_TIM_ENABLE( &ButtonPushCheck );//Enable timer to start

			HAL_TIM_Base_Init(&ButtonPushCheck);
			HAL_TIM_Base_Start(&ButtonPushCheck);



*/



	__HAL_RCC_TIM4_CLK_ENABLE();


	//seconds
	SecondsTimer.Instance = TIM4;
			SecondsTimer.Init.Period = 9999;// (original value was 39999 / 4 seconds) period & prescaler combination for half second count
			SecondsTimer.Init.Prescaler = 8399; // (original value was 8399)
			SecondsTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
			SecondsTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
			HAL_TIM_Base_Init( &SecondsTimer );

			HAL_NVIC_SetPriority( TIM4_IRQn, 0, 0);//set priority for the interrupt. Value 0 corresponds to highest priority
			HAL_NVIC_EnableIRQ( TIM4_IRQn );//Enable interrupt function request of Timer5

			__HAL_TIM_ENABLE_IT( &SecondsTimer, TIM_IT_UPDATE );// Enable timer interrupt flag to be set when timer count is reached
			__HAL_TIM_ENABLE( &SecondsTimer );//Enable timer to start

			HAL_TIM_Base_Init(&SecondsTimer);
			HAL_TIM_Base_Start(&SecondsTimer);






//// Configure the input pins 4, 5, 8, 9 and 11 of port C for reading the push buttons.
	  //IO for push buttons using internal pull-up resistors
	  GPIOInitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11;
	  GPIOInitStruct.Speed = GPIO_SPEED_LOW;
	  GPIOInitStruct.Mode = GPIO_MODE_INPUT;
	  GPIOInitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOC, &GPIOInitStruct);


//// Configure GPIO for selecting each segment on a digit.
	  //configure GPIO for digits
	  GPIOInitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	  GPIOInitStruct.Speed = GPIO_SPEED_LOW;
	  GPIOInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIOInitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOE, &GPIOInitStruct);



//// Configure GPIO for selecting each digit on the LED display.
	  //configure GPIO for multiplexing
	  GPIOInitStruct.Pin =   GPIO_PIN_4 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	  GPIOInitStruct.Speed = GPIO_SPEED_LOW;
	  GPIOInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIOInitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOD, &GPIOInitStruct);


//	  	 HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_E | Segment_F | Segment_G | Dp,  GPIO_PIN_SET );
//	  	 HAL_GPIO_WritePin( GPIOD, Digit_1 | Digit_2 | Digit_3 | Digit_4 | L, GPIO_PIN_SET);// Turn-on the display segments for 8



///*****************************************************************************************************************************************

	  GPIOInitStruct.Pin = GPIO_PIN_15 ;
	  GPIOInitStruct.Speed = GPIO_SPEED_LOW;
	  GPIOInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIOInitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOD, &GPIOInitStruct);



	  GPIOInitStruct.Pin = GPIO_PIN_14 ;
	  GPIOInitStruct.Speed = GPIO_SPEED_LOW;
	  GPIOInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIOInitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOD, &GPIOInitStruct);



	  //(Alarm == Time) Flag
	  GPIOInitStruct.Pin = GPIO_PIN_13 ;
	  	  GPIOInitStruct.Speed = GPIO_SPEED_LOW;
	  	  GPIOInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  	  GPIOInitStruct.Pull = GPIO_NOPULL;
	  	  HAL_GPIO_Init(GPIOD, &GPIOInitStruct);



	  //***********************************************************


////
//// Enable the real time clock (RTC) alarm A interrupt





	      //setup the RTC for 12 hour format
	  		ClockInit.HourFormat = RTC_HOURFORMAT_12;
	  		ClockInit.AsynchPrediv = 127;
	  		ClockInit.SynchPrediv = 0x00FF;
	  	//	ClockInit.OutPut         = RTC_OUTPUT_DISABLE; maybe need next three lines
	  	//	ClockInit.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	  	//	ClockInit.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;



//for Clock Time
		  	rtcHandle.Instance = RTC;
		  	rtcHandle.Init = ClockInit;
		  	HAL_RTC_Init(&rtcHandle);

	  	  //set the time displayed on power up to 00:00 (24hr format)
	  	  ClockTime.TimeFormat = RTC_HOURFORMAT_24;
	  	  ClockTime.Hours = 0x00;
	  	  ClockTime.Minutes = 0x000;
	  	  ClockTime.Seconds = 0x000;
	  	  HAL_RTC_SetTime(&rtcHandle, &ClockTime, RTC_FORMAT_BCD);

	  	HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_SET );




//for Alarm Time

	  	ClockInit2.HourFormat = RTC_HOURFORMAT_12;
	  	ClockInit2.AsynchPrediv = 127;
	  	ClockInit2.SynchPrediv = 0x00FF;

	  	rtcHandle2.Instance = RTC;
	  	rtcHandle2.Init = ClockInit;
	  	HAL_RTC_Init(&rtcHandle2);

  	  //set the default alarm on power up to 00:00 (24hr format)
  	  AlarmTime.TimeFormat = RTC_HOURFORMAT_24;
  	  AlarmTime.Hours = 0x00;
  	  AlarmTime.Minutes = 0x000;
  	  AlarmTime.Seconds = 0x000;
  	  HAL_RTC_SetTime(&rtcHandle2, &AlarmTime, RTC_FORMAT_BCD);

//###################################
//		 RTC_ITConfig(RTC_IT_ALRA, ENABLE); maybe?
////############################################################
//// Enable the timer interrupt
		TIM5->DIER |= TIM_IT_UPDATE;
//
////
//// Enable the LED display and push button timer
		  TIM5->CR1 |= TIM_CR1_CEN;
////
//
}


int main(int argc, char* argv[])
{

//
// Reset of all peripherals, Initializes the Flash interface and the System timer.
//
	HAL_Init();

//
// Configure the system clock
//
	SystemClock_Config();

//
// Initialize all configured peripherals
//
	MX_GPIO_Init();

//
// Enable the serial debug port. This allows for text messages to be sent via the STlink virtual communications port to the host computer.
//
	DebugPortInit();

//
// Display project name with version number
//
	trace_puts(
			"*\n"
			"*\n"
			"* Alarm clock project for stm32f4discovery board V2.00\n"
			"*\n"
			"*\n"
			);

//
// Initialize the I2C port for the external CODEC
//
	MX_I2C1_Init();

//
// Configure the CODEC for analog pass through mode.
// This allows for audio to be played out of the stereo jack
//
	InitAudioChip();

//
// Initialize the flash file and the USB host adapter subsystem
//

	MX_FATFS_Init();
	MX_USB_HOST_Init();

//
// Initialize the DMA and DAC systems. This allows for audio to be played out of GPIOA pin 4
//
	ConfigureAudioDma();

//
// Initialize the seven segment display pins
//
	ConfigureDisplay();


//
// Send a greeting to the trace device (skipped on Release).
//
	trace_puts("Initialization Complete");

//
// At this stage the system clock should have already been configured at high speed.
//
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

//
// Start the system timer
//
	timer_start();


	blink_led_init();

//
// Wait until the drive is mounted before we can play some music
//
	do {
		MX_USB_HOST_Process();
	} while ( Appli_state != APPLICATION_READY );

	trace_printf( "\n" );

//
// Remove comment slashes from line line below for music to play at start
//
	PlayMusic = FALSE;




	while ( TRUE )
	{

//
//	checks for the alarm interrupt and call the music playing module
//
		if ( TRUE == PlayMusic )
		{

    	 	 MSC_Application();
		}

//
// Wait for an interrupt to occur
//
		__asm__ volatile ( "wfi" );


	}
}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

	I2c.Instance = I2C1;
	I2c.Init.ClockSpeed = 100000;
	I2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2c.Init.OwnAddress1 = 0;
	I2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2c.Init.OwnAddress2 = 0;
	I2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init( &I2c );

}

void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();



	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
			|Audio_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_I2C_SPI_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_Init( GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PDM_OUT_Pin */
	GPIO_InitStruct.Pin = PDM_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);


	/*Configure GPIO pins : PA5 PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CLK_IN_Pin */
	GPIO_InitStruct.Pin = CLK_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
			|Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/**
 * @brief  Main routine for Mass Storage Class
 * @param  None
 * @retval None
 */
static void MSC_Application(void)
{
	FRESULT
		Result;                                          /* FatFs function common result code */

//
// Mount the flash drive using a fat file format
//

	Result = f_mount( &UsbDiskFatFs, (TCHAR const*)USBH_Path, 0);
	if( FR_OK == Result )
	{

//
// File system successfully mounted, play all the music files in the directory.
//
		while ( TRUE == PlayMusic )
		{
			PlayDirectory( "", 0 );
		}
	}
	else
	{
//
// FatFs Initialization Error
//
	//	Error_Handler();
	}

//
// Unlink the USB disk I/O driver
//
	FATFS_UnLinkDriver( UsbDiskPath );
}



/*
 * Function: TIM5_IRQHandler
 *
 * Description:
 *
 * Timer interrupt handler that is called at a rate of 500Hz. This function polls the time and
 * displays it on the 7 segment display. It also checks for button presses and handles any bounce conditions.
 *
 */


//LED display
void TIM5_IRQHandler(void)
{



		//double checks that interrupt has occurred
		if( __HAL_TIM_GET_FLAG( &DisplayTimer, TIM_IT_UPDATE ) != RESET )
		{
			// MAIN LOOP



			check_current_state();

			/*
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4 == 0)){
				ClockTime.Minutes++;

			}*/

			if(current_state == 0){
				ProcessButtons();
				Display7Segment();
				DisplayedDigit++;
			}

			if(current_state == 1){

				SetAlarm();
				Display7Segment_Alarm();
				DisplayedDigit_Alarm++;
			}


			CheckFlags();


			//clears interrupt flag
		    TIM5->SR = (uint16_t)~TIM_IT_UPDATE;
		    //Increment pin

	    }


}



//timer
void TIM4_IRQHandler(void)
{


		//double checks that interrupt has occurred
		if( __HAL_TIM_GET_FLAG( &SecondsTimer, TIM_IT_UPDATE ) != RESET )
		{


			default_state();

			if(current_state ==1){
				alarm_setting_state();
			}


			if(current_state ==0){
				CheckAlarm();
			}



			//clears interrupt flag
		    TIM4->SR = (uint16_t)~TIM_IT_UPDATE;


			if(SnoozeVar == 1){
				SnoozeCounter++;
			}




		    //increment seconds (hola)
		    hola++;

	    }

}

/*
void TIM3_IRQHandler(void)
{


		//double checks that interrupt has occurred
		if( __HAL_TIM_GET_FLAG( &ButtonPushCheck, TIM_IT_UPDATE ) != RESET )
		{






			//clears interrupt flag
		    TIM3->SR = (uint16_t)~TIM_IT_UPDATE;




	    }

}*/


/*
 * Function: RTC_Alarm_IRQHandler
 *
 * Description:
 *
 * When alarm occurs, clear all the interrupt bits and flags then start playing music.
 *
 */

void RTC_Alarm_IRQHandler(void)
{

//
// Verify that this is a real time clock interrupt
//
	if( __HAL_RTC_ALARM_GET_IT( &RealTimeClock, RTC_IT_ALRA ) != RESET )
	{

//
// Clear the alarm flag and the external interrupt flag
//
    	__HAL_RTC_ALARM_CLEAR_FLAG( &RealTimeClock, RTC_FLAG_ALRAF );
    	__HAL_RTC_EXTI_CLEAR_FLAG( RTC_EXTI_LINE_ALARM_EVENT );

//
// Restore the alarm to it's original time. This could have been a snooze alarm
//
    	HAL_RTC_SetAlarm_IT( &RealTimeClock, &ClockAlarm, RTC_FORMAT_BCD );

    	PlayMusic = TRUE;

	}


}


/*
 * Function: Display7Segment
 *
 * Description:
 *
 * Displays the current time, alarm time or time format
 *
 */

void Display7Segment(void)
{

////
//// Clear the display Digits
	 HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_E | Segment_F | Segment_G | Dp,  GPIO_PIN_RESET );
	 HAL_GPIO_WritePin( GPIOD, Digit_1 | Digit_2 | Digit_3 | Digit_4 | L, GPIO_PIN_RESET);


	 //Select from one of the 5 cases.
	 	unsigned char multi = (DisplayedDigit % 5);



	 			//Select digit
	 			switch (multi)
	 			{
	 				case 0:
	 					HAL_GPIO_WritePin( GPIOD, Digit_1, GPIO_PIN_SET);
	 						break;

	 				case 1:
	 					HAL_GPIO_WritePin( GPIOD, Digit_2, GPIO_PIN_SET);
	 						break;

	 				case 2:
	 					HAL_GPIO_WritePin( GPIOD, Digit_3, GPIO_PIN_SET);
	 						break;

	 				case 3:
						HAL_GPIO_WritePin( GPIOD, Digit_4, GPIO_PIN_SET);

	 						break;

	 				case 4:
	 					HAL_GPIO_WritePin( GPIOD, L, GPIO_PIN_SET);
	 						break;

	 			}


	 	//GetCurrentTime();




				switch (BcdTime[multi])
				{
				case 0:
				HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_E | Segment_F,  GPIO_PIN_SET );
				break;

				case 1:
				HAL_GPIO_WritePin( GPIOE,  Segment_B | Segment_C,  GPIO_PIN_SET );
				break;

				case 2:
				HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_D | Segment_E | Segment_G,  GPIO_PIN_SET );
				break;

				case 3:
				HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_G,  GPIO_PIN_SET );
				break;

				case 4:
				HAL_GPIO_WritePin( GPIOE, Segment_B | Segment_C | Segment_F | Segment_G,  GPIO_PIN_SET );
				break;

				case 5:
				HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_C | Segment_D | Segment_F | Segment_G,  GPIO_PIN_SET );
				break;

				case 6:
				HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_C | Segment_D | Segment_E | Segment_F | Segment_G,  GPIO_PIN_SET );
				break;

				case 7:
				HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C,  GPIO_PIN_SET );
				break;

				case 8:
				HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_E | Segment_F | Segment_G,  GPIO_PIN_SET );
				break;

				case 9:
				HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_F | Segment_G,  GPIO_PIN_SET );
				break;

				case 10:
				HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B ,  GPIO_PIN_SET );
				break;

				}
				multi++;

////
//	GPIOD->BSRR = ( GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 ) << 16;
////
//// Segments
////
//	GPIOE->BSRR = ( GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
//			GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 ) << 16;

//
// Select with digit to display the time for
//
	  //

}

void DisplayClockMode(void)

{
//
}


/*
 * Function: SetTime
 *
 * Description:
 *
 * Advance either the time hours or minutes field. Validate the new time and then update the clock
 *
 */

void SetTime(void)
{

}

/*
 * Function: SetAlarm
 *
 * Description:
 *
 * Advance either the alarm hours or minutes field. Validate the new alarm time and then set the alarm
 *
 */

void SetAlarm(void)
{

	//set hours button
		if( HAL_GPIO_ReadPin(GPIOC, Push_Button_1) == 0)
		{
			debouncer++;
			if (debouncer == 10)
			{
				if (AlarmTime.TimeFormat == RTC_HOURFORMAT_24 || AlarmTime.TimeFormat == RTC_HOURFORMAT_12){
					AlarmTime.Hours++;
					alarm_setting_state();
				}
			}


		}// change format
		else if( HAL_GPIO_ReadPin(GPIOC, Push_Button_3) == 0){
			debouncer++;
					if (debouncer == 10)
					{
						if (AlarmTime.TimeFormat == RTC_HOURFORMAT_24 || AlarmTime.TimeFormat == RTC_HOURFORMAT_12){
							AlarmTime.Minutes++;
							alarm_setting_state();
						}
					}

		}else{
			debouncer = 0;
		}



}


/*
 * Function: Snooze
 *
 * Description:
 *
 * Add 10 Minutes to the current time and validate. Update the alarm and enable.
 *
 */

//void Snooze(void)
//{
//}


/*
 * Function: GetCurrentTime
 *
 * Description:
 *
 * Return either the alarm time or current time in binary coded decimal format store in the array BcdTime.
 *
 */

void GetCurrentTime(void)
{


		//Load time to the array
	BcdTime[0] = ClockTime.Hours /10;
	BcdTime[1] = ClockTime.Hours %10;
	BcdTime[2] = ClockTime.Minutes /10;
	BcdTime[3] = (ClockTime.Minutes) %10;
	BcdTime[4] = 10;
}

void GetAlarmTime(void)
{


		//Load time to the array
	BcdAlarmTime[0] = AlarmTime.Hours /10;
	BcdAlarmTime[1] = AlarmTime.Hours %10;
	BcdAlarmTime[2] = AlarmTime.Minutes /10;
	BcdAlarmTime[3] = (AlarmTime.Minutes) %10;
	BcdAlarmTime[4] = 10;
}

/*


 * Function: CheckButtons
 *
 * Description:
 *
 * Check the current state of all the buttons and apply debounce algorithm. Return TRUE with the ButtonPushed
 * variable set indicating the button or buttons pushed if button press is detected.
 *
 */

//uint16_t CheckButtons( void )
//{
//
//
//
////
//}


/*
 * Function: ProcessButtons
 *
 * Description:
 *
 * Test for which button or buttons has been pressed and do the appropriate operation.
 *
 */

void ProcessButtons( void )
{

	//set hours button
	if( HAL_GPIO_ReadPin(GPIOC, Push_Button_1) == 0)
	{
		debouncer++;
		if (debouncer == 10)
		{
			if (ClockTime.TimeFormat == RTC_HOURFORMAT_24 || ClockTime.TimeFormat == RTC_HOURFORMAT_12){
				ClockTime.Hours++;
				default_state();
			}
		}


	}// change format
	else if( HAL_GPIO_ReadPin(GPIOC, Push_Button_2) == 0)
	{
		debouncer++;
				if (debouncer == 10)
				{


					//if time format is 24hr change it to 12 hr
					if (ClockTime.TimeFormat == RTC_HOURFORMAT_24){
						ClockTime.TimeFormat = RTC_HOURFORMAT_12;


						if( ClockTime.Hours==0){
							ric = 1;
							TimeAMFlag= 1;
							//HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_SET );
							ClockTime.Hours= ClockTime.Hours + 12;

						} else if(ClockTime.Hours>12 && ClockTime.Hours<24) {
							TimeAMFlag= 0;
							ClockTime.Hours = (ClockTime.Hours -12);


						}else if(ClockTime.Hours==12){
							TimeAMFlag= 0;
							ric = 1;
						}
					}
					//if  time format is 12hr change it to 24 hr
					else if (ClockTime.TimeFormat == RTC_HOURFORMAT_12)
					{
						ClockTime.TimeFormat = RTC_HOURFORMAT_24;
						if ((ClockTime.Hours == 12) && (HAL_GPIO_ReadPin( GPIOD, GPIO_PIN_15)) == 1){
							ClockTime.Hours = 0;
						}else if(HAL_GPIO_ReadPin( GPIOD, GPIO_PIN_15) == 1){
							ClockTime.Hours = ClockTime.Hours + 0;
						}else if (ClockTime.Hours == 12){
							ClockTime.Hours = ClockTime.Hours + 0;
						}else{
							ClockTime.Hours = ClockTime.Hours + 12;
						}
					}









/*

					//if format is 24hr change it to 12 hr
											if (AlarmTime.TimeFormat == RTC_HOURFORMAT_24){
												AlarmTime.TimeFormat = RTC_HOURFORMAT_12;


												if( AlarmTime.Hours==0){
													ric2 = 1;
													AlarmAMFlag= 1;
													HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_SET );
													AlarmTime.Hours= AlarmTime.Hours + 12;

												} else if(AlarmTime.Hours>12 && AlarmTime.Hours<24) {
													AlarmAMFlag= 0;
													AlarmTime.Hours = (AlarmTime.Hours -12);


												}else if(AlarmTime.Hours==12){
													AlarmAMFlag= 0;
													ric2 = 1;
												}
											}
											//if format is 12hr change it to 24 hr
											else if (AlarmTime.TimeFormat == RTC_HOURFORMAT_12)
											{
												AlarmTime.TimeFormat = RTC_HOURFORMAT_24;
												if ((AlarmTime.Hours == 12) && (HAL_GPIO_ReadPin( GPIOD, GPIO_PIN_15)) == 1){
													AlarmTime.Hours = 0;
												}else if(HAL_GPIO_ReadPin( GPIOD, GPIO_PIN_15) == 1){
													AlarmTime.Hours = AlarmTime.Hours + 0;
												}else if (AlarmTime.Hours == 12){
													AlarmTime.Hours = AlarmTime.Hours + 0;
												}else{
													AlarmTime.Hours = AlarmTime.Hours + 12;
												}
											}
*/


				}


	}else if( HAL_GPIO_ReadPin(GPIOC, Push_Button_3) == 0){
		debouncer++;
				if (debouncer == 10)
				{
					if (ClockTime.TimeFormat == RTC_HOURFORMAT_24 || ClockTime.TimeFormat == RTC_HOURFORMAT_12){
						ClockTime.Minutes++;
						default_state();
					}
				}

	}else if( HAL_GPIO_ReadPin(GPIOC, Push_Button_5) == 0){
		debouncer++;
				if (debouncer == 10)
				{
					if(PlayMusic == TRUE){
						Phil = 0;
						PlayMusic = FALSE;
						SnoozeCounter = 0;
						SnoozeVar = 1;

					}else if (SnoozeCounter < 180){
						PlayMusic = FALSE;
						SnoozeCounter = 300;
					}


				}

	}
	else{
		debouncer = 0;
	}



}

//static void Error_Handler(void)
//{

//while(1)
//	{
//	}
//}




// hola extra functions
void default_state(void)
{
    //Case for time in 24 hour format
	if(ClockTime.TimeFormat == RTC_HOURFORMAT_24 )
	{
		if (hola == 60)
		{
			ClockTime.Minutes++;
			hola = 0;
		}
		if (ClockTime.Minutes == 60)
		{
						ClockTime.Hours++;
						ClockTime.Minutes = 0;
		}
		if(ClockTime.Hours == 12){
			//HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_RESET );
			TimeAMFlag = 0;
		}

		if(ClockTime.Hours == 24)
		{
			ClockTime.Hours = 0;
			//HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_SET );
			TimeAMFlag = 1;
		}

		GetCurrentTime();

	}

	//case for time in 12 hour format
	if(ClockTime.TimeFormat == RTC_HOURFORMAT_12 )
		{
			if (hola == 60)
			{
				ClockTime.Minutes++;
				hola = 0;
			}
			if (ClockTime.Minutes == 60)
			{
							ClockTime.Hours++;
							ClockTime.Minutes = 0;
			}
			if(ClockTime.Hours == 13)
			{
				ric = 0;
				//HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_RESET );
				ClockTime.Hours = 1;


			}
			//
			if(ClockTime.Hours == 12){
				if (ric == 0){
					if((HAL_GPIO_ReadPin( GPIOD, GPIO_PIN_15)) == 1){
						HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_RESET );
						TimeAMFlag = 0;

						ric = 1;
					}else{
						HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_SET );
						TimeAMFlag = 1;
						ric = 1;
					}
				}


			}
			GetCurrentTime();

		}
}

void check_current_state()
{

	if( HAL_GPIO_ReadPin(GPIOC, Push_Button_4) == 0)
	{
			debouncer2++;
			if (debouncer2 == 10)
			{

				// if button is pushed and we are in clock mode, change to alarm mode
				if(  current_state == 0)
				{
					current_state = 1;
					AlarmAMFlag = 1;
					  HAL_GPIO_WritePin( GPIOD, GPIO_PIN_14,  GPIO_PIN_SET );

					  if(ClockTime.TimeFormat == RTC_HOURFORMAT_12){
						  AlarmTime.Hours = 0;
						  AlarmAMFlag = 1;
					  }

					  if(ClockTime.TimeFormat == RTC_HOURFORMAT_24){
					  						  AlarmTime.Hours = 0;
					  						  AlarmAMFlag = 1;
					  }


				} else if(current_state == 1)
				{
					current_state = 0;
					Phil = 1;


					HAL_GPIO_WritePin( GPIOD, GPIO_PIN_14,  GPIO_PIN_RESET );
				}
			}


	}
	else{
			debouncer2= 0;
		}


}

void alarm_setting_state(){
	 //Case for time in 24 hour format
		if(AlarmTime.TimeFormat == RTC_HOURFORMAT_24 )
		{

			if (AlarmTime.Minutes == 60)
			{
							AlarmTime.Hours++;
							AlarmTime.Minutes = 0;
			}
			if(AlarmTime.Hours == 12){
				AlarmAMFlag = 0;
			}

			if(AlarmTime.Hours == 24)
			{
				AlarmTime.Hours = 0;
				AlarmAMFlag = 1;

			}

			GetAlarmTime();

		}

		//case for time in 12 hour format
		/*
		if(AlarmTime.TimeFormat == RTC_HOURFORMAT_12 )
			{

				if (AlarmTime.Minutes == 60)
				{
								AlarmTime.Hours++;
								AlarmTime.Minutes = 0;
				}
				if(AlarmTime.Hours == 13)
				{
					ric2 = 0;
					//HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_RESET );
					AlarmTime.Hours = 1;


				}
				if(AlarmTime.Hours == 12){
					if (ric2 == 0){
						if((HAL_GPIO_ReadPin( GPIOD, GPIO_PIN_15)) == 1){
							HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_RESET );
							ric2 = 1;
						}else{
							HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_SET );
							ric2 = 1;
						}
					}


				}
				GetAlarmTime();

			}
			*/
}


void Display7Segment_Alarm(){

	////
	//// Clear the display Digits
		 HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_E | Segment_F | Segment_G | Dp,  GPIO_PIN_RESET );
		 HAL_GPIO_WritePin( GPIOD, Digit_1 | Digit_2 | Digit_3 | Digit_4 | L, GPIO_PIN_RESET);


		 //Select from one of the 5 cases.
		 	unsigned char multi2 = (DisplayedDigit_Alarm % 5);



		 			//Select digit
		 			switch (multi2)
		 			{
		 				case 0:
		 					HAL_GPIO_WritePin( GPIOD, Digit_1, GPIO_PIN_SET);
		 						break;

		 				case 1:
		 					HAL_GPIO_WritePin( GPIOD, Digit_2, GPIO_PIN_SET);
		 						break;

		 				case 2:
		 					HAL_GPIO_WritePin( GPIOD, Digit_3, GPIO_PIN_SET);
		 						break;

		 				case 3:
							HAL_GPIO_WritePin( GPIOD, Digit_4, GPIO_PIN_SET);

		 						break;

		 				case 4:
		 					HAL_GPIO_WritePin( GPIOD, L, GPIO_PIN_SET);
		 						break;

		 			}


		 	//GetCurrentTime();




					switch (BcdAlarmTime[multi2])
					{
					case 0:
					HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_E | Segment_F,  GPIO_PIN_SET );
					break;

					case 1:
					HAL_GPIO_WritePin( GPIOE,  Segment_B | Segment_C,  GPIO_PIN_SET );
					break;

					case 2:
					HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_D | Segment_E | Segment_G,  GPIO_PIN_SET );
					break;

					case 3:
					HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_G,  GPIO_PIN_SET );
					break;

					case 4:
					HAL_GPIO_WritePin( GPIOE, Segment_B | Segment_C | Segment_F | Segment_G,  GPIO_PIN_SET );
					break;

					case 5:
					HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_C | Segment_D | Segment_F | Segment_G,  GPIO_PIN_SET );
					break;

					case 6:
					HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_C | Segment_D | Segment_E | Segment_F | Segment_G,  GPIO_PIN_SET );
					break;

					case 7:
					HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C,  GPIO_PIN_SET );
					break;

					case 8:
					HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_E | Segment_F | Segment_G,  GPIO_PIN_SET );
					break;

					case 9:
					HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B | Segment_C | Segment_D | Segment_F | Segment_G,  GPIO_PIN_SET );
					break;

					case 10:
					HAL_GPIO_WritePin( GPIOE, Segment_A | Segment_B ,  GPIO_PIN_SET );
					break;

					}
					multi2++;


}

void CheckAlarm()
{
	if(	SnoozeCounter == 180){
		SnoozeCounter = 300;
		PlayMusic = TRUE;
		SnoozeVar = 0;
	}
	if (Phil == 1)
	{



		if(ClockTime.TimeFormat == RTC_HOURFORMAT_24)
		{



			if((AlarmTime.Hours == ClockTime.Hours) && (AlarmTime.Minutes == ClockTime.Minutes) && (AlarmTime.Seconds == ClockTime.Seconds))
			{
			HAL_GPIO_WritePin( GPIOD, GPIO_PIN_13,  GPIO_PIN_SET );
			PlayMusic = TRUE;
		}else{
			HAL_GPIO_WritePin( GPIOD, GPIO_PIN_13,  GPIO_PIN_RESET );
		}


	} else if(ClockTime.TimeFormat == RTC_HOURFORMAT_12){

		if(ClockTime.Hours == 12){
			if(TimeAMFlag ==1){
				Tairan = ClockTime.Hours -12;
			}else{
				Tairan = ClockTime.Hours;
			}
		}else{
			if(TimeAMFlag ==1){
				Tairan = ClockTime.Hours;
			}else{
				Tairan = ClockTime.Hours +12;
			}
		}






		if((AlarmTime.Hours == Tairan) && (AlarmTime.Minutes == ClockTime.Minutes) && (AlarmTime.Minutes == ClockTime.Minutes))
				{
			PlayMusic = TRUE;
			HAL_GPIO_WritePin( GPIOD, GPIO_PIN_13,  GPIO_PIN_SET );
				}else{
						HAL_GPIO_WritePin( GPIOD, GPIO_PIN_13,  GPIO_PIN_RESET );
				}

	}
	}

}

void CheckFlags(){
	if(current_state ==0){
		if (TimeAMFlag == 1){
			HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_SET );
		} else if (TimeAMFlag == 0){
			HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_RESET );
		}
	}



	if(current_state ==1){
			if (AlarmAMFlag == 1){
				HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_SET );
			} else if (AlarmAMFlag == 0){
				HAL_GPIO_WritePin( GPIOD, GPIO_PIN_15,  GPIO_PIN_RESET );
			}
		}
}



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
