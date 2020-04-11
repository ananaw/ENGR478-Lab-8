#include <stdint.h>
#include <stdbool.h>
#include "Lab 8 PeriodInt.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"					// manually added
#include "driverlib/interrupt.h" 			// manually added
#include "inc/tm4c123gh6pm.h"					// manually added

//*****************************************************************************
volatile unsigned long count = 0;			// volatile ensures no optimization done by compiler

void
PortFunctionInit(void)
{
    //
    // Enable Peripheral Clocks 
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable pin PF0 for GPIOInput (SW2)
    //

    //
    //First open the lock and select the bits we want to modify in the GPIO commit register.
    //
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;

    //
    //Now modify the configuration of the pins that we unlocked.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
	
		//
    // Enable pin PF4 for GPIOInput (SW1)
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    //
    // Enable pin PF3 for GPIOOutput (Green)
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Enable pin PF1 for GPIOOutput (Red)
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    //
    // Enable pin PF2 for GPIOOutput (Blue)
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
		
		// Enable internal pull-up for PF0&4;
		GPIO_PORTF_PUR_R |= 0x11;
}

void Timer0A_Init(unsigned long period)
{   
	//
  // Enable Peripheral Clocks 
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); 		// configure for 32-bit timer mode
  TimerLoadSet(TIMER0_BASE, TIMER_A, period -1);      //reload value
	IntEnable(INT_TIMER0A);    				// enable interrupt 19 in NVIC (Timer0A)
	IntPrioritySet(INT_TIMER0A, 0x00);  	 // configure Timer0A interrupt priority as 0
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);      // arm timeout interrupt
  TimerEnable(TIMER0_BASE, TIMER_A);      // enable timer0A
}

void Switch_Interrupt(void)
{
	IntEnable(INT_GPIOF);							// Enable interrupt 30 in NVIC, Code: NVIC_EN0_R |= 0x40000000;
	IntPrioritySet(INT_GPIOF, 0x02);	// Set interrupt priority, Code: NVIC_PRI7_R &= ~0x00E00000
	GPIO_PORTF_IM_R |= 0x11; 					// arm interrupt on PF0&4
	GPIO_PORTF_IS_R &= ~0x11;					// PF0 & PF4 are edge sensitive
	GPIO_PORTF_IBE_R &= ~0x11; 				// PF0 & PF4 not both edge triggered, depends on IEV register
	GPIO_PORTF_IEV_R &= ~0x11; 				// PF0 & PF4 are falling-edged events
}

//interrupt handler for Timer0A
void Timer0A_Handler(void)
{
		// acknowledge flag for Timer0A timeout
		TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		
		// count incremented by 1
		count++;
}

// Switch interrupt handler
void GPIOPortF_Handler(void)
{
	// switch debounce
	NVIC_EN0_R &= ~0x40000000;
	SysCtlDelay(26667);
	NVIC_EN0_R |= 0x40000000;
	
	//SW1 has action (PF4)
	if(GPIO_PORTF_RIS_R&0x10)
	{
		// acknowledge flag for PF4
		GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);			//GPIO_PORTF_ICR_R |= 0x10; 
		
		// SW1 is pressed
		if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)== 0x00)
		{
			//counter incremented by 1
			count++;
		}
	}
		
	// SW2 has action (PF0)
	if(GPIO_PORTF_RIS_R&0x01)
	{
		// acknowledge flag for PF0
		GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);			//GPIO_PORTF_ICR_R |= 0x01; 
		
		// SW2 is pressed
		if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)== 0x00)
		{
			//counter decremented by 1
			count--;		
		}	
	}
}
int main(void)
{	
		unsigned long period = 8000000; //reload value to Timer0A to generate one second delay

		//initialize the GPIO ports	
		PortFunctionInit();
	
		// Set System clock freq to 8MHz
	  SysCtlClockSet(SYSCTL_SYSDIV_25|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	
    // Turn on the LED.
    //GPIO_PORTF_DATA_R |= 0x0E;

    //initialize Timer0A and configure the interrupt
		Timer0A_Init(period);
		
		Switch_Interrupt();
	
		IntMasterEnable();        		// globally enable interrupt
	
    //
    // Loop forever.
    //
    while(1)
    {
			// 8 cases total 
			for(int i = 1; i < 8; ++i)
			{
			 // &bitwise comparison with i. 
			 if((count&i) == i)
			 {
				 // Turn on corresponding LED, Ex: if i=4-->0b100--> green on-->PF3-->1 0 0 0 -->0x08
				 GPIO_PORTF_DATA_R |= i*2;
				 
				 //GPIO_PORTF_DATA_R &= ~(7-i);
			 }
			 else
			 {
				 // Turn off all LED's
				 GPIO_PORTF_DATA_R &= ~0x0E;
			 }
			}
    }
} 
