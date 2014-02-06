#include <System.h>
#include <stm32f4xx.h>
#include <stdio.h>
#include <string.h>
#include <Spi.h>
#include "funk.h"


/**
  Pins: beide VDD an 3V, beide GND an GND, 
  CE an PC0, CSN an PB12, SCK an PB13, MISO an PB14, MOSI an PB15, IRQ an PA1?
*/
uint8_t reuseCount = 0;

void init(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  //CE PC0, 
  GPIO_Init(GPIOC, &(GPIO_InitTypeDef){
      .GPIO_Pin = GPIO_Pin_0,
      .GPIO_Mode = GPIO_Mode_OUT,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_OType = GPIO_OType_PP,
      .GPIO_PuPd = GPIO_PuPd_UP,
    });

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
   
  GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
      .GPIO_Pin = GPIO_Pin_0 ,//| GPIO_Pin_1,
      .GPIO_Mode = GPIO_Mode_IN,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_OType = GPIO_OType_PP,
      .GPIO_PuPd = GPIO_PuPd_NOPULL,
    });
    //IRQ PD4
    GPIO_Init(GPIOD, &(GPIO_InitTypeDef){
      .GPIO_Pin = GPIO_Pin_4 ,//| GPIO_Pin_1,
      .GPIO_Mode = GPIO_Mode_IN,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_OType = GPIO_OType_PP,
      .GPIO_PuPd = GPIO_PuPd_NOPULL,
    });
  
  
    EXTI_Init(&(EXTI_InitTypeDef){
        .EXTI_Line = EXTI_Line0,
        .EXTI_Mode = EXTI_Mode_Interrupt,
        .EXTI_Trigger = EXTI_Trigger_Rising_Falling,
        .EXTI_LineCmd = ENABLE
         });
  EXTI_Init(&(EXTI_InitTypeDef){
        .EXTI_Line = EXTI_Line4,
        .EXTI_Mode = EXTI_Mode_Interrupt,
        .EXTI_Trigger = EXTI_Trigger_Falling,
        .EXTI_LineCmd = ENABLE
         });

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource4);
    NVIC_Init(&(NVIC_InitTypeDef){
      .NVIC_IRQChannel = EXTI0_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 2,
      .NVIC_IRQChannelSubPriority = 2,
      .NVIC_IRQChannelCmd = ENABLE,
    });

  NVIC_Init(&(NVIC_InitTypeDef){
      .NVIC_IRQChannel = EXTI4_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 1,
      .NVIC_IRQChannelSubPriority = 1,
      .NVIC_IRQChannelCmd = ENABLE,
    });

  // Enable clock for SPI hardware and GPIO port B
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  // SPI  (CSN PB12, SCK PB13, MISO PB14, MOSI PB15)
  GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
      .GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15,
      .GPIO_Mode = GPIO_Mode_AF,
      .GPIO_Speed = GPIO_Speed_50MHz,
    });
    GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
      .GPIO_Pin = GPIO_Pin_12,
      .GPIO_Mode = GPIO_Mode_OUT,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_OType = GPIO_OType_PP,
      .GPIO_PuPd = GPIO_PuPd_UP,
    });

  // Configure pins to be used by the SPI hardware (alternate function)
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

  // Init SPI
  SPI_Init(SPI2, &(SPI_InitTypeDef){
      .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64, // Configure Data speed
      .SPI_CPHA = SPI_CPHA_1Edge, // Sample data on rising edge
      .SPI_CPOL = SPI_CPOL_Low, // Clock is default low
      .SPI_CRCPolynomial = 1, // Don't use CRC
      .SPI_DataSize = SPI_DataSize_8b, // Send 16 bit words at a time
      .SPI_Direction = SPI_Direction_2Lines_FullDuplex, // Only enable sending (transmission/TX)
      .SPI_FirstBit = SPI_FirstBit_MSB, // Most Significant Bit first
      .SPI_Mode = SPI_Mode_Master, // STM32 is the master
      .SPI_NSS = SPI_NSS_Soft, // Don't use automatic chip select
    });
  
  // Enable SPI interrupt
  NVIC_Init(&(NVIC_InitTypeDef){
      .NVIC_IRQChannel = SPI2_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 0,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    });

  // Enable SPI hardware
  SPI_Cmd(SPI2, ENABLE);
  

  //CE Pin... auf Low
  GPIO_ResetBits(GPIOC, GPIO_Pin_0);

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
  Delay(100);

  writeRegDef(DYNPD, 0x01);
  writeRegDef(FEATURE, 0x07);
  //maskInterrupts(MAX_RT);
}

void EXTI0_IRQHandler()
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    printf("exti0\n");
    reset();
    //Clear the interrupt bit and tell the controller we handlet the interrupt
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}
void EXTI4_IRQHandler()
{
  if(EXTI_GetITStatus(EXTI_Line4) != RESET)
  {
    if(isBitSet(STATUS, TX_DS))
    {
      printf("Message sent\n");
      
      
      FLUSH_TX();
      resetInterrupt(TX_DS);
      

      setBitLow(CONFIG, PWR_UP);
      GPIO_ResetBits(GPIOC, GPIO_Pin_0);
      
      printf("CONFIG\n");
      readReg(CONFIG);
      exitPowerDown();
      //setBitHigh(CONFIG,PRIM_RX);
      tryRxStuff();
      
      //GPIO_ResetBits(GPIOC, GPIO_Pin_0);
      //enter_RX_Mode();
      //W_ACK_PAYLOAD();
    }
    if(isBitSet(STATUS, MAX_RT))
    {//setBitLow(STATUS, MAX_RT);
      
      //printf("maxrt.\n");
      //FLUSH_TX();
      REUSE_TX_PL();
      reuseCount++;
      printf("reuseCount %i\n", reuseCount);
      if(reuseCount>3)
      {
        //FLUSH_TX();
        //REUSE_TX_PL();
        reuseCount = 0;
        printf("transmission failed... \n");
        writeRegDef(RF_CH, RF_CH_DEFAULT);
        enter_TX_Mode();
      }
      resetInterrupt(MAX_RT);
      //setBitLow(STATUS, MAX_RT);
      
    }
    if(isBitSet(STATUS, RX_DR))
    {
      printf("RX_EMPTY: %i\n", isBitSet(FIFO_STATUS, RX_EMPTY));
      printf("Message: \n");
      R_RX_PAYLOAD();
      //printf("Received %i\n", R_RX_PAYLOAD());
      
      
      //writeRegDef(STATUS, STATUS_DEFAULT);
      if(isBitSet(FIFO_STATUS, RX_FULL))
        FLUSH_RX();
      else
        printf("RX not full...\n");
      resetInterrupt(RX_DR);
      GPIO_ResetBits(GPIOC, GPIO_Pin_0);
      tryTxStuff();
    }
  EXTI_ClearITPendingBit(EXTI_Line4);
  }
}



void resetInterrupt(uint8_t interrupt)
{
  CSN_HighLow();
  spi_send_byte(0x27);
  spi_wait_txe();
  spi_send_byte(interrupt);
  spi_wait_txe();
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}


void CSN_HighLow(void)
{
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

void tryTxStuff(void)
{
  //setBitHigh(CONFIG, MASK_MAX_RT);
  //read TX_EMPTY bit in FIFO_STATUS register. It should be 1.
  printf("TX_EMPTY: %i\n",isBitSet(FIFO_STATUS, TX_EMPTY));
  printf("RX_EMPTY: %i\n",isBitSet(FIFO_STATUS, RX_EMPTY));
  
  
  writeReg5(TX_ADDR, 0x12);
  writeReg5(RX_ADDR_P0, 0x12);
  W_TX_PAYLOAD("hello world...");
  printf("TX_EMPTY: %i\n",isBitSet(FIFO_STATUS, TX_EMPTY));
  
  enter_TX_Mode();

  //read TX_EMPTY bit again to see if it worked. It should be 0.
  printf("TX_EMPTY: %i\n",isBitSet(FIFO_STATUS, TX_EMPTY));

}

void tryRxStuff(void)
{
  
  printf("RX_EMPTY: %i\n", isBitSet(FIFO_STATUS, RX_EMPTY));

  writeReg5(TX_ADDR, 0x12);
  writeReg5(RX_ADDR_P0, 0x12);
  //writeReg(RX_PW_P0, 0x0e);
   
  enter_RX_Mode();
  W_ACK_PAYLOAD();
  readReg(FIFO_STATUS);
  //R_RX_PAYLOAD();
}


void readReg5(uint8_t reg)
{
  uint16_t receivedStuff[6]; 

  CSN_HighLow();
  spi_send_byte(reg);
  spi_wait_txe();
  receivedStuff[0] = spi_get_last_byte();
  
  for (int i = 1; i < 6; i++)
  {
    spi_send_byte(0xff);//dummy byte...
    spi_wait_txe();
    receivedStuff[i] = spi_get_last_byte();
  }
  
  GPIO_SetBits(GPIOB, GPIO_Pin_12);

  printf("Status: %#x\n", receivedStuff[0]);
  
  for (int i = 1; i < 6; i++)
  {
    printf("%#x ", receivedStuff[i]);
  }
  printf("\n");
}

void readReg(uint8_t reg)
{
  uint16_t receivedStuff[2]; 

  CSN_HighLow();
  spi_send_byte(reg);
  spi_wait_txe();
  receivedStuff[0] = spi_get_last_byte();
  
  spi_send_byte(0xff);
  spi_wait_txe();
  receivedStuff[1] = spi_get_last_byte();

  GPIO_SetBits(GPIOB, GPIO_Pin_12);

  printf("Status: %#x\n%#x\n", receivedStuff[0], receivedStuff[1]);
  
}

void exitPowerDown(void)//will ich die Funktion überhaupt? o.o
{ 
  //unklar in welchem Mode ich dann lande, oder? hmm...
  if(!isBitSet(CONFIG, PWR_UP))
  {  
    setBitHigh(CONFIG, PWR_UP);
    Delay(100); //just to be sure...
    printf("you escaped.\n");
    isBitSet(CONFIG, PWR_UP);
  }
}

void writeReg(uint8_t reg, uint8_t value)
{  
      readReg(reg);
      uint8_t write = spi_get_last_byte();
      write ^= value; //xor gleich. o.o hoffe das passt.
      CSN_HighLow();
      spi_send_byte(0x20 | reg);
      spi_wait_txe();

      spi_send_byte(write);//wie kann man nur so.. gnh
      spi_wait_txe();

      GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
void writeRegDef(uint8_t reg, uint8_t value)
{
      CSN_HighLow();
      spi_send_byte(0x20 | reg);
      spi_wait_txe();

      spi_send_byte(value);//wie kann man nur so.. gnh
      spi_wait_txe();

      GPIO_SetBits(GPIOB, GPIO_Pin_12);
  
}

void writeReg5(uint8_t reg, uint8_t value)
{   
      CSN_HighLow();
      spi_send_byte(0x20 | reg);
      spi_wait_txe();

      for (int i = 0; i < 5; i++)
      {
        spi_send_byte(value);//wie kann man nur so.. gnh
        spi_wait_txe();
      }
      
      GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

uint8_t R_RX_PAYLOAD(void)
{
  
  uint8_t length = R_RX_PL_WID();
  printf("Length of Payload: %i\n", length);
  CSN_HighLow();
  spi_send_byte(0x61);//0110 0001
  spi_wait_txe();

  for(int i=0; i<length; i++)
  { 
    spi_send_byte(0xff);
    spi_wait_txe();
    printf("%c", (char)spi_get_last_byte());
  }
  printf("\n");

  //nicht sicher... muss ich jetzt dummys senden und per SPI was empfangen oder so?

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
  return spi_get_last_byte();
}
void W_TX_PAYLOAD(uint8_t * buffer) 
{
  uint8_t length = strlen(buffer);
  CSN_HighLow();
  spi_send_byte(0xa0);//1010 0000
  spi_wait_txe();

  for (int i = 0; i < length; i++)
  {
    spi_send_byte(buffer[i]);//das ist eigentlich eher doof...
    spi_wait_txe();
  }

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
void FLUSH_TX(void) 
{
  CSN_HighLow();
  spi_send_byte(0xe1);//1110 0001
  spi_wait_txe();

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
void FLUSH_RX(void) 
{
  CSN_HighLow();
  spi_send_byte(0xe2);//1110 0010
  spi_wait_txe();

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
void REUSE_TX_PL(void) 
{
  CSN_HighLow();
  spi_send_byte(0xe3);//1110 0011
  spi_wait_txe();

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void W_ACK_PAYLOAD(void)
{
  CSN_HighLow();
  spi_send_byte(0xA8);
  spi_wait_txe();
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void enter_RX_Mode(void)
{
  readReg(0x00);
  uint8_t write = 0x01 | spi_get_last_byte();
  //
  
  CSN_HighLow();
  spi_send_byte(0x20);
  spi_wait_txe();
  spi_send_byte(write);
  spi_wait_txe();

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
  Delay(1);
  GPIO_SetBits(GPIOC, GPIO_Pin_0); //CE high
  
}

void enter_TX_Mode(void)
{
  setBitLow(CONFIG, PRIM_RX);
  printf("CONFIG: \n");
  readReg(CONFIG);
  GPIO_SetBits(GPIOC, GPIO_Pin_0); //CE high
}

uint8_t getMode(void)
{

  if(!isBitSet(CONFIG, PWR_UP))
    return 1;

}

void setAirDataRate(uint8_t rate)
{
  //RF_DR bit in the RF_SETUP register.
  readReg(0x06);
  uint8_t write = 0x20 | spi_get_last_byte(); //250kbps
  CSN_HighLow();
  

  spi_send_byte(0x26); //write to RF_SETUP register
  spi_wait_txe();

  spi_send_byte(write);
  spi_wait_txe();

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void setChannelFrequency(uint8_t frequency)
{
  //RF_CH register
  CSN_HighLow();
  spi_send_byte(0x25); //RF_CH register
  spi_wait_txe();

  spi_send_byte(frequency);
  spi_wait_txe();

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void maskInterrupts(uint8_t IRQ_masks)
{
  readReg(0x00);

  uint8_t write = IRQ_masks | spi_get_last_byte();
  CSN_HighLow();
  spi_send_byte(0x20);
  spi_wait_txe();
  spi_send_byte(write);
  spi_wait_txe();

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}


_Bool isBitSet(uint8_t reg, uint8_t bit)
{
    readReg(reg);
    uint8_t byte = spi_get_last_byte();

    return (bit == (bit & byte));
}


void setBitHigh(uint8_t reg, uint8_t bit)
{
    if(!isBitSet(reg, bit))
        writeReg(reg, bit);
}
void setBitLow(uint8_t reg, uint8_t bit)
{
    if(isBitSet(reg, bit))
        writeReg(reg, bit);
}


void reset(void)
{
  FLUSH_TX();
  FLUSH_RX();
  reuseCount = 0;

  writeRegDef(CONFIG, CONFIG_DEFAULT);
  writeRegDef(EN_AA, EN_AA_DEFAULT);
  writeRegDef(EN_RXADDR, EN_RXADDR_DEFAULT);
  writeRegDef(SETUP_AW, SETUP_AW_DEFAULT);
  writeRegDef(SETUP_RETR, SETUP_RETR_DEFAULT);
  writeRegDef(STATUS, STATUS_DEFAULT);
  ///readReg(STATUS);

  writeRegDef(RF_CH, RF_CH_DEFAULT);
  writeRegDef(RF_SETUP, RF_SETUP_DEFAULT);
  writeReg5(RX_ADDR_P0, RX_ADDR_P0_DEFAULT);
  writeReg5(RX_ADDR_P1, RX_ADDR_P1_DEFAULT);
  writeRegDef(RX_ADDR_P2, RX_ADDR_P2_DEFAULT);
  writeRegDef(RX_ADDR_P3, RX_ADDR_P3_DEFAULT);
  writeRegDef(RX_ADDR_P4, RX_ADDR_P4_DEFAULT);
  writeRegDef(RX_ADDR_P5, RX_ADDR_P5_DEFAULT);

  writeReg5(TX_ADDR, TX_ADDR_DEFAULT);
  writeRegDef(RX_PW_P0, RX_PW_P0_DEFAULT);
  writeRegDef(RX_PW_P1, RX_PW_P1_DEFAULT);
  writeRegDef(RX_PW_P2, RX_PW_P2_DEFAULT);
  writeRegDef(RX_PW_P3, RX_PW_P3_DEFAULT);
  writeRegDef(RX_PW_P4, RX_PW_P4_DEFAULT);
  writeRegDef(RX_PW_P5, RX_PW_P5_DEFAULT);

  writeRegDef(FIFO_STATUS, FIFO_STATUS_DEFAULT);
  writeRegDef(DYNPD, DYNPD_DEFAULT);
  writeRegDef(FEATURE, FEATURE_DEFAULT);
  
  GPIO_ResetBits(GPIOC, GPIO_Pin_0);

  printf("reset finished.\n");
}

void showRegisters(void)
{
  readReg(CONFIG);
  readReg(EN_AA);
  readReg(EN_RXADDR);
  readReg(SETUP_AW);
  readReg(SETUP_RETR);
  readReg(STATUS);

  readReg(RF_CH);
  readReg(RF_SETUP);
  readReg5(RX_ADDR_P0);
  readReg5(RX_ADDR_P1);
  readReg(RX_ADDR_P2);
  readReg(RX_ADDR_P3);
  readReg(RX_ADDR_P4);
  readReg(RX_ADDR_P5);

  readReg5(TX_ADDR);
  readReg(RX_PW_P0);
  readReg(RX_PW_P1);
  readReg(RX_PW_P2);
  readReg(RX_PW_P3);
  readReg(RX_PW_P4);
  readReg(RX_PW_P5);

  readReg(FIFO_STATUS);
  readReg(DYNPD);
  readReg(FEATURE);
}

uint8_t R_RX_PL_WID(void)
{
  CSN_HighLow();

  spi_send_byte(0x60);
  spi_wait_txe();

  spi_send_byte(0xff);
  spi_wait_txe();

  GPIO_SetBits(GPIOB, GPIO_Pin_12);
  return spi_get_last_byte();
}

void readMessage(void)
{
  //so ne funktion die strings einlesen kann. 
}

void sendMessage()
{
  //W_TX_PAYLOAD(message);
  enter_TX_Mode();
}