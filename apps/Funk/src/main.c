#include <System.h>
#include <stm32f4xx.h>
#include <stdio.h>
#include <string.h>
#include "funk.h"
#include <Spi.h>



void main(void)
{
    InitializeSystem();
    EnableDebugOutput(DEBUG_ITM);

  
    init();
    exitPowerDown();

    //writeReg5(TX_ADDR, 0x12);
  	//writeReg5(RX_ADDR_P0, 0x12);

    //enter_RX_Mode();
    tryTxStuff();
    //tryRxStuff();

    while(1)
    {
    	/*
		readMessage();
		sendMessage();
    	*/
    }
}
