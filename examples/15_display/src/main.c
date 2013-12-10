#include <System.h>
#include <stdio.h>
#include "display.h"


//Example for 16 x 2 Character LCD Display Module
 
//Pins:
//Connect Pin 0 and Pin 16 to Ground (Pin 2 probably goes to Ground too... though I'm not sure about that)
//and Pin 1 and Pin 14 to 5V
//the other Pins all go to GPIOs on the board, which I'll try to set in this code...

//Helpfull link to find out about the pins:
//http://www.engineersgarage.com/electronic-components/16x2-lcd-module-datasheet

//Datasheet and Manual:
//http://www.engineersgarage.com/sites/default/files/LCD%2016x2.pdf
//http://images.vishay.com/books/VSD-DB0047-0303_%20Information%20Display%20Products_LCD%20Displays_INTERACTIVE.pdf


int main(void)
{
    InitializeSystem();
    EnableDebugOutput(DEBUG_ITM);

    initDisplay();
    clearDisplay();
    DisplayOn();
    writeSomething("Hallo Welt!");
    //moveCursor();
    //go to second line
    setDDRamAdress(0x40);
    writeSomething("second line :D");



   /* int x = 0x40;
    for(int i = 0; i<7; i++, x++)
    {
      
      setDDRamAdress(x);
      readDDRam();
      for(int j = 7; j>0; j--)
      {
        printf("%i", ac[j]);
      }
      printf("\n");
      Delay(10);
    //printf("%i\n", readBusyFlag());
    //moveCursor();
    }*/

    while(1)
    {

    }
}
