#include "display.h"
#include <stm32f4xx_gpio.h>
#include <System.h>
#include <stdio.h>



void initDisplay(void)
{

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode = GPIO_Mode_OUT,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_UP,
        .GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
    });

    GPIO_Init(GPIOC, &(GPIO_InitTypeDef){
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode = GPIO_Mode_OUT,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_UP,
        .GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
    });

    GPIO_ResetBits(GPIOB, 0xff);
    GPIO_ResetBits(GPIOC, 0xff);

    Delay(30);
    GPIO_ResetBits(GPIOB, Display_RS | Display_RW);
    GPIO_ResetBits(GPIOC, Display_6 | Display_7);
    GPIO_SetBits(GPIOC, Display_4 | Display_5);
    sendDataToPins();
    Delay(5);
    sendDataToPins();
    Delay(1);
    sendDataToPins();

    //Function set 8 bits
    //GPIO_ResetBits(GPIOC, Display_2 );//font 5x7
    GPIO_ResetBits(GPIOB, 0xff);
    GPIO_ResetBits(GPIOC, 0xff);
    GPIO_SetBits(GPIOC, Display_2 | Display_3 | Display_4 | Display_5);
    sendDataToPins();
    //display off
    GPIO_ResetBits(GPIOB, 0xff);
    GPIO_ResetBits(GPIOC, 0xff);
    //GPIO_ResetBits(GPIOC, Display_0 | Display_1 | Display_2);
    GPIO_SetBits(GPIOC, Display_3);
    sendDataToPins();
    //display clear
    GPIO_ResetBits(GPIOB, 0xff);
    GPIO_ResetBits(GPIOC, 0xff);
    //GPIO_ResetBits(GPIOC, Display_3);
    GPIO_SetBits(GPIOC, Display_0);
    sendDataToPins();
    //entry mode set
    GPIO_ResetBits(GPIOB, 0xff);
    GPIO_ResetBits(GPIOC, 0xff);
    //GPIO_ResetBits(GPIOC, Display_0);
    GPIO_SetBits(GPIOC,Display_0 | Display_1 | Display_2);
    sendDataToPins();
    printf("init\n");

}

void clearDisplay(void)
{
    GPIO_ResetBits(GPIOB, Display_RS | Display_RW);
    GPIO_ResetBits(GPIOC, Display_1 | Display_2 | Display_3 | Display_4 | Display_5 | Display_6 | Display_7);
    GPIO_SetBits(GPIOC, Display_0);
    sendDataToPins();
    printf("clearDisplay\n");
}


void writeSomething(char *bla)
{

    uint8_t length=strlen(bla);
    //printf("length: %i\n", length);
    for(uint8_t i = 0; i < length; i++)
    {
        GPIO_SetBits(GPIOB, Display_RS);
        GPIO_ResetBits(GPIOB, Display_RW);
        GPIO_ResetBits(GPIOC, 0xff);
        GPIO_SetBits(GPIOC, bla[i]);
        
        sendDataToPins();
       // getAddressCounter();
        //Delay(100);
    }


    printf("write Something...\n");
}

void sendDataToPins(void)
{
    GPIO_ResetBits(GPIOB, Display_E);
    Delay(1);
    GPIO_SetBits(GPIOB, Display_E);
}

void DisplayOn(void)
{
    GPIO_ResetBits(GPIOB, Display_RS | Display_RW);
    GPIO_ResetBits(GPIOC, Display_4 | Display_5 | Display_6 | Display_7);
    GPIO_SetBits(GPIOC, Display_0 | Display_1 | Display_2 | Display_3);
    sendDataToPins();
    printf("display on\n");
}

void moveCursor(void)
{
    GPIO_ResetBits(GPIOB, Display_RS | Display_RW);
    GPIO_ResetBits(GPIOC, Display_3 | Display_5 | Display_6 | Display_7);
    GPIO_SetBits(GPIOC, Display_2 | Display_4);
    sendDataToPins();

    printf("move Cursor\n");
}

void shiftDisplay(void)
{
    GPIO_ResetBits(GPIOB, Display_RS | Display_RW);
    GPIO_ResetBits(GPIOC, Display_2 | Display_5 | Display_6 | Display_7);
    GPIO_SetBits(GPIOC, Display_3 | Display_4);
    sendDataToPins();

    printf("shift Display\n");
}

void setDDRamAdress(uint16_t x)
{
    GPIO_ResetBits(GPIOB, Display_RS | Display_RW);
    GPIO_ResetBits(GPIOC, 0xff);
    //GPIO_ResetBits(GPIOC, Display_0 | Display_1 | Display_2 | Display_3 | Display_4 | Display_5 | Display_6);
    
    GPIO_SetBits(GPIOC,x);
    GPIO_SetBits(GPIOC, Display_7);
    sendDataToPins();
    //1 0 0 0 0 0 0 0
    //GPIO_SetBits(GPIOC, x);
}

void getAddressCounter(void)//try this while writing something
{
    GPIO_ResetBits(GPIOB, Display_RS | Display_RW);

   // GPIO_ResetBits(GPIOB, Display_E);
    //GPIO_SetBits(GPIOB, Display_E);    //this is sendDataToPins without the Delay...
                                    //also try without sendDataToPins at all...
    bf = GPIO_ReadOutputDataBit(GPIOC, Display_7);
    ac[6] = GPIO_ReadOutputDataBit(GPIOC, Display_6);
    ac[5] = GPIO_ReadOutputDataBit(GPIOC, Display_5);
    ac[4] = GPIO_ReadOutputDataBit(GPIOC, Display_4);
    ac[3] = GPIO_ReadOutputDataBit(GPIOC, Display_3);
    ac[2] = GPIO_ReadOutputDataBit(GPIOC, Display_2);
    ac[1] = GPIO_ReadOutputDataBit(GPIOC, Display_1);
    ac[0] = GPIO_ReadOutputDataBit(GPIOC, Display_0);
    

    for(int i=6; i>=0; i--)
    {
        printf("%i", ac[i]);
    }
    printf(" bf: %i\n", bf);
}

//---------------------------------------------------------------------------------
//here are some functions that I put down here because I'm not sure if they work...


void writeCharacter(char Character)
{   //0 1 0 0 1 0 0 0
    //L H L L H L L L
    GPIO_SetBits(GPIOB, Display_RS);
    GPIO_ResetBits(GPIOB, Display_RW);

    //GPIO_SetBits(GPIOC, Display_3 | Display_6);
    //GPIO_ResetBits(GPIOC, Display_0 | Display_1 | Display_2 | Display_4 | Display_5 | Display_7);
    GPIO_ResetBits(GPIOC, 0xff);
    GPIO_SetBits(GPIOC, Character);
    sendDataToPins();
    printf("writeH\n");

    
}

uint16_t readBusyFlag(void) //this always returns 0... i probably made a mistake..
{
    //GPIO_ResetBits(GPIOB, Display_RS);
    GPIO_SetBits(GPIOB,Display_RS | Display_RW);
    sendDataToPins();

    ac[6] = GPIO_ReadInputDataBit(GPIOC, Display_0);
    ac[5] = GPIO_ReadInputDataBit(GPIOC, Display_1);
    ac[4] = GPIO_ReadInputDataBit(GPIOC, Display_2);
    ac[3] = GPIO_ReadInputDataBit(GPIOC, Display_3);
    ac[2] = GPIO_ReadInputDataBit(GPIOC, Display_4);
    ac[1] = GPIO_ReadInputDataBit(GPIOC, Display_5);
    ac[0] = GPIO_ReadInputDataBit(GPIOC, Display_6);

    return GPIO_ReadInputData(GPIOC);
}

void readDDRam(void)
{
    GPIO_SetBits(GPIOB, Display_RS);// | Display_RW);
    GPIO_ResetBits(GPIOB, Display_RW);
    sendDataToPins();
    //GPIO_SetBits(GPIOB, Display_RW);
    ac[0] = GPIO_ReadInputDataBit(GPIOC, Display_0);
    ac[1] = GPIO_ReadInputDataBit(GPIOC, Display_1);
    ac[2] = GPIO_ReadInputDataBit(GPIOC, Display_2);
    ac[3] = GPIO_ReadInputDataBit(GPIOC, Display_3);
    ac[4] = GPIO_ReadInputDataBit(GPIOC, Display_4);
    ac[5] = GPIO_ReadInputDataBit(GPIOC, Display_5);
    ac[6] = GPIO_ReadInputDataBit(GPIOC, Display_6);

}
