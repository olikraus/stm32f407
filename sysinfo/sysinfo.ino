/*
  sysinfo

  /.arduino15/packages/STMicroelectronics/hardware/stm32/2.2.0/system/Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h

  SystemCoreClockUpdate()
  
*/

#include "stm32f4xx_hal.h"

uint32_t prex[8] = { 0, 0, 0, 0, 2, 4, 8, 16 };

void configure160(void)
{
  RCC_OscInitTypeDef  RCC_OscInitStruct;
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
  }
  SystemCoreClockUpdate();

}

void sysinfo(void)
{
  Serial.print("SystemCoreClock = ");
  Serial.print(SystemCoreClock/1000000UL, DEC);
  Serial.println(" MHz");

  Serial.print("HPRE (AHB Prescaler Bits) = ");
  Serial.print((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos, BIN);
  Serial.println("");

  Serial.print("PRE1 (APB1 Prescaler Bits) = ");
  Serial.print((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos, BIN);
  Serial.print(" / ");
  Serial.print(SystemCoreClock/1000000UL / prex[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos], DEC);
  Serial.println(" MHz");

  Serial.print("PRE2 (APB2 Prescaler Bits) = ");
  Serial.print((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos, BIN);
  Serial.print(" / ");
  Serial.print(SystemCoreClock/1000000UL / prex[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos], DEC);
  Serial.println(" MHz");

  Serial.print("SysTick->LOAD=0x");
  Serial.print(SysTick->LOAD, HEX);
  Serial.print("=");
  Serial.print(SysTick->LOAD, DEC);
  Serial.println("");
  Serial.print("SysTick->CTRL=0x");
  Serial.print(SysTick->CTRL, HEX);
  Serial.println("");


  if ( RCC->APB2ENR & RCC_APB2ENR_SPI1EN )
    Serial.println("SPI1 @APB2 clock enabled");

  if ( RCC->APB2ENR & RCC_APB2ENR_TIM1EN )
    Serial.println("TIM1 @APB2 clock enabled");

  if ( RCC->APB1ENR & RCC_APB1ENR_TIM2EN )
    Serial.println("TIM2 @APB1 clock enabled");

  if ( RCC->APB1ENR & RCC_APB1ENR_TIM3EN )
    Serial.println("TIM3 @APB1 clock enabled");

  if ( RCC->APB1ENR & RCC_APB1ENR_TIM4EN )
    Serial.println("TIM4 @APB1 clock enabled");

  if ( RCC->APB1ENR & RCC_APB1ENR_TIM5EN )
    Serial.println("TIM5 @APB1 clock enabled");

  if ( RCC->APB1ENR & RCC_APB1ENR_TIM6EN )
    Serial.println("TIM6 @APB1 clock enabled");

  if ( RCC->APB1ENR & RCC_APB1ENR_TIM7EN )
    Serial.println("TIM7 @APB1 clock enabled");

  if ( RCC->APB2ENR & RCC_APB2ENR_TIM8EN )
    Serial.println("TIM8 @APB2 clock enabled");
    
  if ( RCC->APB2ENR & RCC_APB2ENR_TIM9EN )
    Serial.println("TIM9 @APB2 clock enabled");

  if ( RCC->APB2ENR & RCC_APB2ENR_TIM10EN )
    Serial.println("TIM10 @APB2 clock enabled");

  if ( RCC->APB2ENR & RCC_APB2ENR_TIM11EN )
    Serial.println("TIM11 @APB2 clock enabled");

}

// the setup function runs once when you press reset or power the board
void setup(void) {

  //configure160();
  
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PA1, OUTPUT);

}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(PA1, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(PA1, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  sysinfo();
}
