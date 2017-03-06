//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
//
//*****************************************************************************

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "led_task.h"
#include "switch_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "string.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>FreeRTOS Example (freertos_demo)</h1>
//!
//! This application demonstrates the use of FreeRTOS on Launchpad.
//!
//! The application blinks the user-selected LED at a user-selected frequency. 
//! To select the LED press the left button and to select the frequency press
//! the right button.  The UART outputs the application status at 115,200 baud,
//! 8-n-1 mode.
//!
//! This application utilizes FreeRTOS to perform the tasks in a concurrent
//! fashion.  The following tasks are created:
//!
//! - An LED task, which blinks the user-selected on-board LED at a
//!   user-selected rate (changed via the buttons).
//!
//! - A Switch task, which monitors the buttons pressed and passes the
//!   information to LED task.
//!
//! In addition to the tasks, this application also uses the following FreeRTOS
//! resources:
//!
//! - A Queue to enable information transfer between tasks.
//!
//! - A Semaphore to guard the resource, UART, from access by multiple tasks at
//!   the same time.
//!
//! - A non-blocking FreeRTOS Delay to put the tasks in blocked state when they
//!   have nothing to do.
//!
//! For additional details on FreeRTOS, refer to the FreeRTOS web page at:
//! http://www.freertos.org/
//
//*****************************************************************************


//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
// address of SLAVE
#define SLAVE_ADDRESS          0x68
// Configure UART0 to console data which transmitted

static unsigned long WaitI2CDone( unsigned int long ulBase){
    // Wait until done transmitting
    while( I2CMasterBusy(I2C3_MASTER_BASE));
    // Return I2C error code
    return I2CMasterErr( I2C3_MASTER_BASE);
}


void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 16000000);
}
void I2C1_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//GPIO_PORTA_DEN_R |=(1<<6)|(1<<7);
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	//GPIOPinTypeGPIOOutputOD(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    //HWREG(I2C0_MASTER_BASE + I2C_O_MCR) |= 0x01;
    I2CMasterInitExpClk(I2C1_MASTER_BASE, SysCtlClockGet(), false);
}
void I2C1_Send(uint16_t device_address, uint8_t device_data)
{
    // Determine Slave address
    // false: transmit Master --> Slave
    // true:  receive  Master <-- Slave
    I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device_address, false);
    // Put data
    I2CMasterDataPut(I2C1_MASTER_BASE, device_data);
    // Transmit data
    I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    // Wait until finish
    while(!(I2CSlaveStatus(I2C1_MASTER_BASE) & I2C_SLAVE_ACT_RREQ));
}
uint32_t burst_data[14];
void I2C1_Receive(uint16_t device_address)
{
    // Determine Slave address
    // false: transmit Master --> Slave
    // true:  receive  Master <-- Slave
	uint32_t data=220;
	int i;
    I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device_address, true);
    //
    // Initiate recieve of character from Slave to Master
    //

    I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    //
    // Wait for recieve to begin
    //
    while(!I2CMasterBusy(I2C1_MASTER_BASE));
    //
    // Delay until recieve completes
    //
    while(I2CMasterBusy(I2C1_MASTER_BASE));
    //
    // Get the character that was recieved in the data register
    //
    burst_data[0] = I2CMasterDataGet(I2C1_MASTER_BASE);

    for(i=1; i<12; i++)
    {
		//
		// Initiate recieve of character from Slave to Master
		//
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		//
		// Wait for recieve to begin
		//
		while(!I2CMasterBusy(I2C1_MASTER_BASE));
		//
		// Delay until recieve completes
		//
		while(I2CMasterBusy(I2C1_MASTER_BASE));
		//
		// Get the character that was recieved in the data register
		//
		burst_data[i] = I2CMasterDataGet(I2C1_MASTER_BASE);

    }

    //
    // Initiate recieve of character from Slave to Master
    //
    I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    //
    // Wait for recieve to begin
    //
    while(!I2CMasterBusy(I2C1_MASTER_BASE));
    //
    // Delay until recieve completes
    //
    while(I2CMasterBusy(I2C1_MASTER_BASE));
    //
    // Get the character that was recieved in the data register
    //
    burst_data[13] = I2CMasterDataGet(I2C1_MASTER_BASE);
	return;

}
void initI2C0(void)
{
   SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

   //reset I2C module
   SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

   //enable GPIO peripheral that contains I2C
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

   // Configure the pin muxing for I2C0 functions on port B2 and B3.
   GPIOPinConfigure(GPIO_PB2_I2C0SCL);
   GPIOPinConfigure(GPIO_PB3_I2C0SDA);

   // Select the I2C function for these pins.
   GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
   GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

   // Enable and initialize the I2C0 master module.  Use the system clock for
   // the I2C0 module.  The last parameter sets the I2C data transfer rate.
   // If false the data rate is set to 100kbps and if true the data rate will
   // be set to 400kbps.
   I2CMasterInitExpClk(I2C0_MASTER_BASE, SysCtlClockGet(), false);

   //clear I2C FIFOs
   //HWREG(I2C0_MASTER_BASE + I2CO_FIFOCTL) = 80008000;
}

uint8_t readI2C0(uint16_t device_address, uint16_t device_register)
{
   //specify that we want to communicate to device address with an intended write to bus
   I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, device_address, false);

   //the register to be read
   I2CMasterDataPut(I2C0_MASTER_BASE, device_register);

   //send control byte and register address byte to slave device
   I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);

   //wait for MCU to complete send transaction
   while(I2CMasterBusy(I2C0_MASTER_BASE));

   //read from the specified slave device
   I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, device_address, true);

   //send control byte and read from the register from the MCU
   I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

   //wait while checking for MCU to complete the transaction
   while(I2CMasterBusy(I2C0_MASTER_BASE));

   //Get the data from the MCU register and return to caller
   return( I2CMasterDataGet(I2C0_MASTER_BASE));
 }

void readBurstI2C0(uint16_t device_address)
{
    // Determine Slave address
    // false: transmit Master --> Slave
    // true:  receive  Master <-- Slave
	int i;
    I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device_address, true);
    //
    // Initiate recieve of character from Slave to Master
    //

    I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    //
    // Wait for recieve to begin
    //
    while(!I2CMasterBusy(I2C1_MASTER_BASE));
    //
    // Delay until recieve completes
    //
    while(I2CMasterBusy(I2C1_MASTER_BASE));
    //
    // Get the character that was recieved in the data register
    //
    burst_data[0] = I2CMasterDataGet(I2C1_MASTER_BASE);

    for(i=1; i<12; i++)
    {
		//
		// Initiate recieve of character from Slave to Master
		//
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		//
		// Wait for recieve to begin
		//
		while(!I2CMasterBusy(I2C1_MASTER_BASE));
		//
		// Delay until recieve completes
		//
		while(I2CMasterBusy(I2C1_MASTER_BASE));
		//
		// Get the character that was recieved in the data register
		//
		burst_data[i] = I2CMasterDataGet(I2C1_MASTER_BASE);

    }

    //
    // Initiate recieve of character from Slave to Master
    //
    I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    //
    // Wait for recieve to begin
    //
    while(!I2CMasterBusy(I2C1_MASTER_BASE));
    //
    // Delay until recieve completes
    //
    while(I2CMasterBusy(I2C1_MASTER_BASE));
    //
    // Get the character that was recieved in the data register
    //
    burst_data[13] = I2CMasterDataGet(I2C1_MASTER_BASE);
	return;

}

void writeI2C0(uint16_t device_address, uint16_t device_register, uint8_t device_data)
{
   //specify that we want to communicate to device address with an intended write to bus
   I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, device_address, false);

   //register to be read
   I2CMasterDataPut(I2C0_MASTER_BASE, device_register);

   //send control byte and register address byte to slave device
   I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);

   //wait for MCU to finish transaction
   while(I2CMasterBusy(I2C0_MASTER_BASE));

   I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, device_address, true);

   //specify data to be written to the above mentioned device_register
   I2CMasterDataPut(I2C0_MASTER_BASE, device_data);

   //wait while checking for MCU to complete the transaction
   I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

   //wait for MCU & device to complete transaction
   while(I2CMasterBusy(I2C0_MASTER_BASE));
}
int
main(void)
{
	unsigned long int i=0;
	uint32_t ret=0;
	uint16_t reg=0;
    //
    // Set the clocking to run at 50 MHz from the PLL.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
	 SysCtlDelay(10000);
	
    //
    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    //

	InitConsole();
    UARTprintf("\n\nWelcome to the Stellaris EK-LM4F120 FreeRTOS Demo!\n");
	initI2C0();
	UARTprintf("%u New\n",readI2C0(0x68, 0x6B));
	writeI2C0(0x68, 0x6B, 10);
	UARTprintf("%u wrote\n",readI2C0(0x68, 0x6B));
	reg=0x3B;
	for(i=0;i<14;i++)
	UARTprintf("%u New\n",readI2C0(0x68, reg++));
	
	//I2C1_Init();
    //IntMasterEnable();
	//Enable MPU6050
	/*
	I2C1_Send(0x68, 0x1B);
	I2C1_Send(0x68, (1<<7)|(1<<6)|(1<<5));
	I2C1_Send(0x68, 0x1C);
	I2C1_Send(0x68, (1<<7)|(1<<6)|(1<<5));
	I2C1_Send(0x68, 0x6B);
	I2C1_Send(0x68, 0);
	I2C1_Send(0x68, 0x1C);
	I2C1_Receive(0x68);
	for(i=0;i<14;i++)
	UARTprintf("%u but..\n",burst_data[i]);

	//I2C1_Send(0x68, 0x3A);
	//I2C1_Send(0x68, 0xFF);
	I2C1_Send(0x68, 0x6C);
	//SysCtlDelay(50000);
	I2C1_Send(0x68,0xFF);
	SysCtlDelay(500000);
//	SysCtlDelay(500000);
	I2C1_Send(0x68, 0x6B);
//	I2C1_Send(0x68, 0x01);
    UARTprintf("\n\nSendComplete!\n");
//	SysCtlDelay(500000);
	I2C1_Receive(0x68);
//	SysCtlDelay(500000);	
	for(i=0;i<14;i++)
	UARTprintf("%u but..\n",burst_data[i]);
//	SysCtlDelay(50000);
/*	
//    I2C1_Init();
	I2C1_Send(0x68, 0x3B);
    UARTprintf("\n\nSendComplete!\n");
	SysCtlDelay(50000);
	UARTprintf("%u but..\n\n",I2C1_Receive(0x68));
	I2C1_Send(0x68, 0x3C);
    UARTprintf("\n\nSendComplete!\n");
	SysCtlDelay(50000);
	UARTprintf("%u but..\n\n",I2C1_Receive(0x68));
	I2C1_Send(0x68, 0x3D);
    UARTprintf("\n\nSendComplete!\n");
	SysCtlDelay(50000);
	UARTprintf("%u but..\n\n",I2C1_Receive(0x68));
/*
	g_pUARTSemaphore = xSemaphoreCreateMutex();
	i=14;char reg=0x3B;
	while(1){
//	sprintf(str, "%d", ret);
	while(I2CMasterBusBusy(I2C1_MASTER_BASE));
		I2CMasterDataPut(I2C1_MASTER_BASE,(unsigned int)reg);
	    I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusBusy(I2C1_MASTER_BASE));
//		I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
//		ret=I2CMasterDataGet(I2C1_MASTER_BASE);
		UARTprintf("%c:%u but..\n\n",reg,ret);
        for(i = 0; i < 20000000; i++);
		reg++;

	}
*/

    //
    // Start the scheduler.  This should not return.
    //
    //vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //

    while(1)
    {
    }
}
