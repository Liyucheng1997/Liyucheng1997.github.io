#include "stm32f3xx.h"
#include "pinAccess.h"
#include "spi.h"
#include "stdbool.h"

#define IODIRA 0x00
#define IODIRB 0x01
#define GPPUA  0x0C
#define GPPUB  0x0D
#define GPINTENB  0x05
#define DEFVALB  0x07
#define INTCONB  0x09
#define INTCAPB  0x11
#define McGPIOA  0x12
#define McGPIOB  0x13
#define OLATA  0x14
#define OLATB  0x15
#define IOCON  0x0A
#define INTFB  0x0F

enum port{PORTA =0,PORTB =1};
enum mode{MCP_OUTPUT =0 ,MCP_INPUT =1 ,MCP_INPUT_PULLUP =2};
enum itType{RISING,FALLING ,BOTH};

typedef void (* mcpCallBack)();

void(*fpcallback0)();
void(*fpcallback1)();
void(*fpcallback2)();
void(*fpcallback3)();
void(*fpcallback4)();
void(*fpcallback5)();
void(*fpcallback6)();
void(*fpcallback7)();

static int dir = 0;

void writeReg (uint8_t reg , uint8_t val)
{
    beginTransaction();
    transfer8(0b01000000);
    transfer8(reg);
    transfer8(val);
    endTransaction();
}

uint8_t readReg (uint8_t reg)
{
    beginTransaction();
    transfer8(0b01000001);
    transfer8(reg);
    uint8_t read = transfer8(0);
    endTransaction();

    return read;
}

void setBitInReg ( uint8_t reg , uint8_t bitNum)
{
    uint8_t read = readReg (reg);
    read |= (1<<bitNum);
    writeReg (reg , read);
}

void clearBitInReg ( uint8_t reg , uint8_t bitNum)
{
    uint8_t read = readReg (reg);
    read &= ~(1<<bitNum);
    writeReg (reg , read);
}

void mcpPinMode (enum port p , unsigned char bitNum , enum mode m )
{

    switch (m)
    {
        case MCP_OUTPUT:
            switch (p)
            {
                case PORTA:
                    clearBitInReg(IODIRA, bitNum);
                break;

                case PORTB:
                    clearBitInReg(IODIRB, bitNum);
                break;
            }
            break;
        case MCP_INPUT:
            switch (p)
            {
               case PORTA:
                  setBitInReg(IODIRA, bitNum);
                break;

               case PORTB:
                   setBitInReg(IODIRB, bitNum);
                   break;
            }
            break;

        case MCP_INPUT_PULLUP:
            switch (p)
            {
                case PORTA:
                   setBitInReg(IODIRA, bitNum);
                   setBitInReg(GPPUA, bitNum);
                   break;

                case PORTB:
                    setBitInReg(IODIRB, bitNum);
                    setBitInReg(GPPUB, bitNum);
                    break;
         }
         break;
    }
}

void mcpdigitalWrite (enum port p ,unsigned char bitNum ,bool value )
{
    switch (p)
    {
        case PORTA:
            if(value == 0)
                clearBitInReg(OLATA, bitNum);
            else
                setBitInReg(OLATA, bitNum);
        break;

        case PORTB:
            if(value == 0)
                clearBitInReg(OLATB, bitNum);
            else
                setBitInReg(OLATB, bitNum);

            break;
    }
}

uint8_t readBits (enum port p)
{
    switch (p)
    {
        case PORTA:
            return readReg(McGPIOA);
            break;

        case PORTB:
            return readReg(McGPIOB);
            break;
    }
    return 0;
}

void callback()
{
    if(dir == 0)
        dir = 1;
    else
        dir = 0;
}

void EXTI9_5_IRQHandler()
{
    readReg (INTCAPB);
    uint8_t pins = readReg(INTFB);
    for(int i=0;i<8;i++)
    {
        if(pins & (1<<i))
        {
            switch (i)
            {
            case 0:
                (*fpcallback0)();
                break;
            case 1:
                (*fpcallback1)();
                break;
            case 2:
                (*fpcallback2)();
                break;
            case 3:
                (*fpcallback3)();
                break;
            case 4:
                (*fpcallback4)();
                break;
            case 5:
                (*fpcallback5)();
                break;
            case 6:
                (*fpcallback6)();
                break;
           case 7:
                (*fpcallback7)();
                break;
           }
        }
     }
     //(*fpcallback4)();
     EXTI->PR |= EXTI_PR_PR9;
}

void attachInterrupt (enum port p , uint8_t bitNum ,enum itType type,mcpCallBack fpcallback)
{

    clearBitInReg(IOCON,6);
    clearBitInReg(IOCON,2);
    setBitInReg(IOCON,1);
    if(p == PORTB)
    {

        setBitInReg (GPINTENB,bitNum);

        switch (type)
        {
           case RISING:
              clearBitInReg(DEFVALB,bitNum);
              setBitInReg(INTCONB,bitNum);
              break;

           case FALLING:
               setBitInReg(DEFVALB,bitNum);
               setBitInReg(INTCONB,bitNum);
               break;

           case BOTH:
               clearBitInReg(INTCONB,bitNum);
               break;
         }
    }
    switch (bitNum)
        {
            case 0:
                fpcallback0= fpcallback;
                break;
            case 1:
                fpcallback1= fpcallback;
                break;
            case 2:
                fpcallback2= fpcallback;
                break;
            case 3:
                fpcallback3= fpcallback;
                break;
            case 4:
                fpcallback4= fpcallback;
                break;
            case 5:
                fpcallback5= fpcallback;
                break;
            case 6:
                fpcallback6= fpcallback;
                break;
            case 7:
                fpcallback7= fpcallback;
                break;
        }
}

void setup()
{
    setupSPI();

    for(int j=0;j<8;j++)
    {
        mcpPinMode (PORTA, j,MCP_OUTPUT);
        mcpPinMode (PORTB, j,MCP_INPUT_PULLUP);
    }
    pinMode(GPIOA,9 ,INPUT);

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    __asm("nop");

    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
    __asm("nop");

    TIM2->PSC=64000-1;
    TIM2->ARR=10-1;
    TIM2->CR1 |= TIM_CR1_CEN;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    __asm( "nop" ) ;

    EXTI->IMR |= EXTI_IMR_MR9;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR3_EXTI9_PA;
    EXTI->RTSR |= EXTI_RTSR_TR9 ;
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}
int main(void) {
    setup();

    void(*fpcallback)();
    fpcallback = &callback;

    attachInterrupt (PORTB, 4,BOTH,(*fpcallback));

    static int i = 0;
    while (1)
        {
            TIM2->SR &= ~TIM_SR_UIF;

            for(int j=0;j<8;j++)
                mcpdigitalWrite (PORTA,j,0);

            if(dir == 0)
            {
                i += 1;
                if(i>7)
                    i = 0;
            }
            else
            {
                i -= 1;
                if(i<0)
                    i = 7;
            }

            mcpdigitalWrite (PORTA,i,1);

            while(! (TIM2->SR & TIM_SR_UIF));
        }
    }
