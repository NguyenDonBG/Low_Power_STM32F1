/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f10x_conf.h"
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define PWR_SLEEPENTRY_WFI              ((uint8_t)0x01)
#define PWR_SLEEPENTRY_WFE              ((uint8_t)0x02)
#define PWR_MAINREGULATOR_ON                        0x00000000U
#define PWR_LOWPOWERREGULATOR_ON                    PWR_CR_LPDS
void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef   Timer_InitStructure;
    Timer_InitStructure.TIM_Prescaler   = 36000 - 1;
    Timer_InitStructure.TIM_Period      = 6000 - 1;
    Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &Timer_InitStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    NVIC_InitTypeDef        NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel    = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd  = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    }
}

void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry)
{

    /* Check the parameters */
    /* No check on Regulator because parameter not used in SLEEP mode */
    /* Prevent unused argument(s) compilation warning */
    assert_param(IS_PWR_SLEEP_ENTRY(SLEEPEntry));

    /* Clear SLEEPDEEP bit of Cortex System Control Register */
    CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

    /* Select SLEEP mode entry -------------------------------------------------*/
    if(SLEEPEntry == PWR_SLEEPENTRY_WFI)
    {
    /* Request Wait For Interrupt */
        __WFI();
    }
    else
    {
        /* Request Wait For Event */
        __SEV();
        __WFE();
        __WFE();
    }
}
void Gpio_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef   GPIO_Init_Structure;
    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init_Structure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_Init_Structure);
}

void SysTick_Init()
{
  //SysTick->VAL  = 0;                                          // Load the SysTick Counter Value
  SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk   |
                  SysTick_CTRL_ENABLE_Msk;                    // Enable SysTick IRQ and SysTick Timer
}
uint32_t volatile SysTickCounter = 1;
char Rx_bufArr[256];
char uart1_rx[256];
static uint8_t rx_count = 0;
static uint8_t index   = 0;

void UART1_Init_A9A10(uint16_t baudrate)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);       // Configure the USART1
  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);//
  NVIC_EnableIRQ(USART1_IRQn);//
  USART_Cmd(USART1, ENABLE);  // enable UART1

  {
    GPIO_InitTypeDef	gpio_init_struct;

    // GPIOA PIN9 alternative function Tx
    gpio_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_10MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init_struct);

    // GPIOA PIN10 alternative function Rx
    gpio_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio_init_struct);
  }

}

int _write(int file, char *ptr, int len)
{
  for (int i = len; i != 0; i--)
  {
    while ((USART1->SR & USART_FLAG_TXE) == 0);
    USART1->DR = *ptr++;
  }
  return len;
}

void UART3_Config(uint32_t baudrate)
{
  /*Cap clock cho USART và port su dung*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	USART_InitTypeDef			UART_InitStructure;
	GPIO_InitTypeDef			GPIO_InitStructure;
	/* Cau Tx mode AF_PP, Rx mode FLOATING  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*Cau hinh USART*/
	UART_InitStructure.USART_BaudRate = baudrate;
	UART_InitStructure.USART_WordLength = USART_WordLength_8b;
	UART_InitStructure.USART_StopBits = USART_StopBits_1;
	UART_InitStructure.USART_Parity = USART_Parity_No;
	UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &UART_InitStructure);
	USART_ITConfig(USART3,USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART3_IRQn);
	/* Cho phep UART hoat dong */
	USART_Cmd(USART3, ENABLE);

}

 void UART_SendChar(USART_TypeDef *USARTx, char data){
    USARTx->DR = 0x00000000;
    USART_SendData(USARTx,data);
    //TxE = 1: Data is transferred to the shift register)
    //TxE = 0; Data is not transferred to the shift register
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
}
void UART_PutStr(USART_TypeDef *USARTx, char *Str){
    while(*Str){
        UART_SendChar(USARTx, *Str);
        Str++;
    }
}

 uint8_t USART_GetChar(USART_TypeDef* USARTx){
    uint8_t Data;
    while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
    Data = (uint8_t)USART_ReceiveData(USARTx);
    return Data;
}

void USART3_IRQHandler()// đọc dữ liệu từ UART gửi xuống và lưu vào mảng
{
  uint8_t chartoreceive = USART_GetChar(USART3);// gán biến bằng dữ liệu uart gửi xuống
  Rx_bufArr[rx_count] = chartoreceive;// lưu dữ liệu vào mảngb
  rx_count++;
  if(Rx_bufArr[rx_count-1] == '\n')
  {
    rx_count = 0;
  }
}

void USART1_IRQHandler()// đọc dữ liệu từ UART gửi xuống và lưu vào mảng
{
  uint8_t chartoreceive = USART_GetChar(USART1);// gán biến bằng dữ liệu uart gửi xuống
  uart1_rx[index] = chartoreceive;
  index++;
  if(uart1_rx[index-1] == '\n')
  {
      index = 0;
  }
}
ErrorStatus HSEStartUpStatus;
void SYSCLKConfig_STOP(void)
{
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {

#ifdef STM32F10X_CL
    /* Enable PLL2 */
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {
    }

#endif

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

#define LED_OFF()           GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define LED_ON()            GPIO_ResetBits(GPIOC, GPIO_Pin_13)
void delay(uint32_t ti)
{
    while(ti--)
    {
        __NOP();
    }
}
int main(void)
{
    //SysTick_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    Timer_Init();
    Gpio_Init();
    UART3_Config(9600);
    UART_PutStr(USART3, "1234\n");
    while(1)
    {
        delay(1500000);
        LED_OFF();
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
        LED_ON();
    }
}
