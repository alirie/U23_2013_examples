
#include <string.h>
#include <stm32f4xx.h>

#define Display_RS GPIO_Pin_0
#define Display_RW GPIO_Pin_1
#define Display_E  GPIO_Pin_2

#define Display_0 GPIO_Pin_0
#define Display_1 GPIO_Pin_1
#define Display_2 GPIO_Pin_2
#define Display_3 GPIO_Pin_3
#define Display_4 GPIO_Pin_4
#define Display_5 GPIO_Pin_5
#define Display_6 GPIO_Pin_6
#define Display_7 GPIO_Pin_7


uint8_t ac[7];
uint8_t bf;

void initDisplay(void);
void clearDisplay(void);
void sendDataToPins(void);
void DisplayOn(void); 
void writeSomething(char *bla);
void moveCursor(void);
void shiftDisplay(void);
void setDDRamAdress(uint16_t x);
void getAddressCounter(void);

//--------------------------------------------
void writeCharacter(char Character);
uint16_t readBusyFlag(void);//not helpful...
void readDDRam(void);