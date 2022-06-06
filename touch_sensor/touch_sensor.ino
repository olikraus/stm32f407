/*
  touch_sensor.ino

*/

//#include "stm32f4xx_hal.h"


/*================================================*/

/*
  Measure the number or processor clock cycles:
  uint32_t start;

  start = SysTick->VAL;
  ...
  getProcessorClockDelta(start); // return the number of processor cycles since start was recorded

  Limitation: The number of cycles between start and getProcessorClockDelta(start) must not exceed Systick->LOAD
  
*/
uint32_t getProcessorClockDelta(uint32_t start_value)
{
  uint32_t current_value = SysTick->VAL;
  /* SysTick->VAL is decremented, so the simple case is current_value < start_value */
  if ( current_value < start_value )
    return start_value-current_value;
  /* reload happend since start_value */
  return SysTick->LOAD - current_value + start_value;
}

/*================================================*/

/*
  pin: 0..15
  value: 0..1
  pupd: 0..2  (0=no pullup/pulldown, 1:pullup, 2:pulldown)
*/
void gpio_config_output(GPIO_TypeDef *gpio, unsigned pin, unsigned value, unsigned pupd)
{
  uint32_t b1 = (((uint32_t)1)<<pin);
  uint32_t m2 = ~(((uint32_t)3)<<(pin*2));         /* two bit mask */
  uint32_t m1 = ~b1;         /* one bit mask */
  uint32_t b01 = (((uint32_t)1)<<(pin*2)); /* 01 bit pattern */
  //uint32_t b10 = (((uint32_t)2)<<(pin*2)); /* 10 bit pattern */
  uint32_t b11 = (((uint32_t)3)<<(pin*2)); /* 11 bit pattern */
  
  gpio->MODER &= m2;
  gpio->MODER |= b01;             /* 01: output mode */
  gpio->OTYPER &= m1;           /* push pull */
  gpio->OSPEEDR &= m2;
  gpio->OSPEEDR |= b11;         /* 11: very fast */
  gpio->PUPDR &= m2;
  gpio->PUPDR |= ((uint32_t)pupd)<<(pin*2);             /* 10: pull down */
  
  if ( value != 0 )
    gpio->BSRR = b1;
  else
    gpio->BSRR = b1<<16;
}

void gpio_config_input(GPIO_TypeDef *gpio, unsigned pin, unsigned pupd)
{
  uint32_t m2 = ~(((uint32_t)3)<<(pin*2));         /* two bit mask */
 
  gpio->MODER &= m2;    /* 00: input */
  gpio->PUPDR &= m2;
  gpio->PUPDR |= ((uint32_t)pupd)<<(pin*2);             /* 10: pull down */
}

uint32_t getChangeToCount(GPIO_TypeDef *gpio, unsigned pin, unsigned value)
{
  uint32_t b1 = (((uint32_t)1)<<pin);
  uint32_t start, count;
  uint32_t v1 = 0;
  uint32_t max = 50000;
  if ( value != 0 )
    v1 = b1;
  start = SysTick->VAL;
  while( max > 0 )
  {
    if ( (gpio->IDR & b1) == v1 )
      break;
    max--;
  }
  count = getProcessorClockDelta(start);
  if ( max == 0 )
    return 0xffffffff;
  return count;
}

uint32_t getCapValue(GPIO_TypeDef *gpio, unsigned pin)
{
    /* configure the pin as output without any pullup/pulldown resistor */
    gpio_config_output(gpio, pin, 1, 2);
    /* ensure that the high output level is reached */ 
    delay(1);
    /* enable pull down */
    gpio_config_output(gpio, pin, 1, 2);
    /* change to input, keep the pulldown */
    gpio_config_input(gpio, pin, 2);
    /* count sys ticks until 0 is detected */
    return getChangeToCount(gpio, pin, 0);
}

/*================================================*/

#define SAMPLES 128

uint16_t iosample[SAMPLES];
uint16_t remove_me[SAMPLES];

uint32_t getChangeTo0(GPIO_TypeDef *gpio, unsigned pin)
{
  uint32_t b1 = (((uint32_t)1)<<pin);
  uint32_t m2 = ~(((uint32_t)3)<<(pin*2));         /* two bit mask */
  uint32_t m1 = ~b1;         /* one bit mask */
  uint32_t b01 = (((uint32_t)1)<<(pin*2)); /* 01 bit pattern */
  //uint32_t b10 = (((uint32_t)2)<<(pin*2)); /* 10 bit pattern */
  uint32_t b11 = (((uint32_t)3)<<(pin*2)); /* 11 bit pattern */
  uint32_t start;
  uint16_t *s = iosample;
  uint32_t i;
  
  gpio->MODER &= m2;
  gpio->MODER |= b01;             /* 01: output mode */
  gpio->OTYPER &= m1;           /* push pull */
  gpio->OSPEEDR &= m2;
  gpio->OSPEEDR |= b11;         /* 11: very fast */
  gpio->PUPDR &= m2;
  gpio->PUPDR |= ((uint32_t)2)<<(pin*2);             /* 10: pull down */
  
  gpio->BSRR = b1;      /* high output */
  //delay(10);     /* wait until high is stable */

  /* wait for some time */
  __NOP(); __NOP(); __NOP(); __NOP();  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();  __NOP(); __NOP(); __NOP(); __NOP();

  
  //start = SysTick->VAL; /* get start value */
  

#define GPIO_SAMPLE_LINE *s++ = gpio->IDR;

#define GPIO_SAMPLE_LINE2 GPIO_SAMPLE_LINE GPIO_SAMPLE_LINE
#define GPIO_SAMPLE_LINE4 GPIO_SAMPLE_LINE2 GPIO_SAMPLE_LINE2
#define GPIO_SAMPLE_LINE8 GPIO_SAMPLE_LINE4 GPIO_SAMPLE_LINE4
#define GPIO_SAMPLE_LINE16 GPIO_SAMPLE_LINE8 GPIO_SAMPLE_LINE8
#define GPIO_SAMPLE_LINE32 GPIO_SAMPLE_LINE16 GPIO_SAMPLE_LINE16
#define GPIO_SAMPLE_LINE64 GPIO_SAMPLE_LINE32 GPIO_SAMPLE_LINE32
#define GPIO_SAMPLE_LINE128 GPIO_SAMPLE_LINE64 GPIO_SAMPLE_LINE64

  /* generate all 128 GPIO read statements, disable interrupts during this GPIO read */
  __disable_irq();
  gpio->MODER &= m2;    /* change to input */
  GPIO_SAMPLE_LINE128
  __enable_irq();

  for( i = 0; i < SAMPLES; i++ )
  {
    if ( (iosample[i] & b1) == 0 )
      return i;
  }
  return i;
  //return getProcessorClockDelta(start);
}

/*================================================*/
/*

 Do a binary search in the global iosample array.
 Look for a 1 to 0 transition and return the position of the same
 mask: only one bit should be set here, which denotest the bit for which the 1 to 0 transition will be searched

 returns:
  0 if all bits are 0
  1..SAMPLES-1 if there the first initial bits are 1
  SAMPLES if all bits are 1

*/
uint32_t getTo0PosByBinarySearch(uint16_t mask)
{
  uint16_t l = 0;
  uint16_t r = SAMPLES-1;
  uint32_t mid;
  while(l < r) 
  {
    mid = l+(r-l)/2;
    if ( iosample[mid] & mask )
      l = mid+1;
    else
      r = mid-1;
  }
  if ( l >= SAMPLES ) return SAMPLES;
  if ( iosample[l] & mask ) return l+1;
  return l;
}

/*
 selectMask: consider only pins where the bit is set inside mask
*/
void getAllChangeTo0(GPIO_TypeDef *gpio, uint16_t selectMask, uint16_t changeTo0Cnt[16])
{

  uint16_t *s = iosample;
  uint32_t m01 = selectMask; 

  // interleave with zero bits
  // https://graphics.stanford.edu/~seander/bithacks.html#InterleaveBMN
  m01 = ( m01 | ( m01 << 8 )) & 0x00ff00ff;
  m01 = ( m01 | ( m01 << 4 )) & 0x0f0f0f0f;
  m01 = ( m01 | ( m01 << 2 )) & 0x33333333;
  m01 = ( m01 | ( m01 << 1 )) & 0x55555555;

  uint32_t m10 = m01<<1;
  uint32_t m11 = m01 | m10;

  gpio->MODER &= ~m11;            /* 00: clear modes of all related pins to zero */
  gpio->MODER |= m01;             /* 01: output mode */
  gpio->OTYPER &= ~m11;           /* 00: push pull */
  gpio->OSPEEDR |= m11;           /* 11: very fast */
  gpio->PUPDR &= ~m11;            /* 00: clear */
  gpio->PUPDR |= m10;             /* 10: pull down */
  /* it is assumed, that the alt function is 0000 */
  
  gpio->BSRR = selectMask;      /* high output */

  /* wait for some time to charge the touch sensor */
  __NOP(); __NOP(); __NOP(); __NOP();  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();  __NOP(); __NOP(); __NOP(); __NOP();

#define GPIO_ALLSAMPLE_LINE *s++ = gpio->IDR;

#define GPIO_ALLSAMPLE_LINE2 GPIO_ALLSAMPLE_LINE GPIO_ALLSAMPLE_LINE
#define GPIO_ALLSAMPLE_LINE4 GPIO_ALLSAMPLE_LINE2 GPIO_ALLSAMPLE_LINE2
#define GPIO_ALLSAMPLE_LINE8 GPIO_ALLSAMPLE_LINE4 GPIO_ALLSAMPLE_LINE4
#define GPIO_ALLSAMPLE_LINE16 GPIO_ALLSAMPLE_LINE8 GPIO_ALLSAMPLE_LINE8
#define GPIO_ALLSAMPLE_LINE32 GPIO_ALLSAMPLE_LINE16 GPIO_ALLSAMPLE_LINE16
#define GPIO_ALLSAMPLE_LINE64 GPIO_ALLSAMPLE_LINE32 GPIO_ALLSAMPLE_LINE32
#define GPIO_ALLSAMPLE_LINE128 GPIO_ALLSAMPLE_LINE64 GPIO_ALLSAMPLE_LINE64

  // generate all 128 GPIO read statements, disable interrupts during this GPIO read
  __disable_irq();
  gpio->MODER &= ~m11;    /* change to input */
  GPIO_SAMPLE_LINE128
  __enable_irq();
  // data is now in the iosample array
  
  for( uint16_t i = 0; i < 16; i++ )
  {
    if ( selectMask & (1<<i) )  
      changeTo0Cnt[i] = getTo0PosByBinarySearch(1<<i);
    else
      changeTo0Cnt[i] = 0;
  }
  
}

/*================================================*/


uint32_t getDMAChangeTo0(GPIO_TypeDef *gpio, unsigned pin)
{
  uint32_t b1 = (((uint32_t)1)<<pin);
  uint32_t m2 = ~(((uint32_t)3)<<(pin*2));         /* two bit mask */
  uint32_t m1 = ~b1;         /* one bit mask */
  uint32_t b01 = (((uint32_t)1)<<(pin*2)); /* 01 bit pattern */
  //uint32_t b10 = (((uint32_t)2)<<(pin*2)); /* 10 bit pattern */
  uint32_t b11 = (((uint32_t)3)<<(pin*2)); /* 11 bit pattern */
  uint32_t i;
  
  gpio->MODER &= m2;
  gpio->MODER |= b01;             /* 01: output mode */
  gpio->OTYPER &= m1;           /* push pull */
  gpio->OSPEEDR &= m2;
  gpio->OSPEEDR |= b11;         /* 11: very fast */
  gpio->PUPDR &= m2;
  gpio->PUPDR |= ((uint32_t)2)<<(pin*2);             /* 10: pull down */
  
  gpio->BSRR = b1;      /* high output */
  delay(1);     /* wait until high is stable */
  
  //start = SysTick->VAL; /* get start value */

  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;        /* Enable DMA2 */
  __NOP();
  __NOP();
  RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA2RST;
  __NOP();
  __NOP();
  RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA2RST;
  __NOP();
  __NOP();
  
  /* the pulldown should enforce a zero after some time */
  //do {
  //} while( (gpio->IDR & b1) != 0 ) ;
  for( i = 0; i < SAMPLES; i++ )
  {
    iosample[i] = 0;
  }
  
  // typedef struct
  // {
  //   __IO uint32_t CR;     /*!< DMA stream x configuration register      */
  //   __IO uint32_t NDTR;   /*!< DMA stream x number of data register     */
  //   __IO uint32_t PAR;    /*!< DMA stream x peripheral address register */
  //   __IO uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  //   __IO uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  //   __IO uint32_t FCR;    /*!< DMA stream x FIFO control register       */
  // } DMA_Stream_TypeDef;

  //DMA1->IFCR |= DMA_ISR_TCIF2;
  DMA2_Stream2->CR = 0;    /* disable DMA channel */  
  
  /* DIR:10 Memory-to-memory source: DMA_SxPAR dest:DMA_SxM0AR */
  DMA2_Stream2->CR |= 0
    | DMA_SxCR_DIR_1  /* memory to memory mode */
    | DMA_SxCR_MINC  /* memory increment */ 
    | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 /* 16 bit size */
    | DMA_SxCR_PL_0 | DMA_SxCR_PL_1               /* high prio */
    //| DMA_CCR_DIR               /* read from i/o */
    //| DMA_CCR_TEIE            /* transfer error interrupt */
    //| DMA_CCR_TCIE            /* transfer complete interrupt enable */
    //| DMA_CCR_CIRC            /* circular mode */
    ;

  DMA2_Stream2->PAR = (uint32_t) (&(gpio->IDR)); /* connect to input bits */
  DMA2_Stream2->M0AR = (uint32_t)(iosample);
  DMA2_Stream2->NDTR = SAMPLES;
  
  //NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
  //NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn,3);

  DMA2_Stream2->CR |= DMA_SxCR_EN;   /* enable DMA channel */  
  
  gpio->MODER &= m2;    /* change to input */
  
  delay(10);     /* wait for DMA*/
  
  //return DMA2_Stream2->NDTR;
  for( i = 0; i < SAMPLES; i++ )
  {
    if ( (iosample[i] & b1) == 0 )
      return i;
  }
  return i;
  //return getProcessorClockDelta(start);
}


/*================================================*/



/*================================================*/


// the setup function runs once when you press reset or power the board
void setup(void) {

  
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PA1, OUTPUT);
  pinMode(PB9, OUTPUT);
  pinMode(PB2, INPUT);

}

uint16_t changeTo0Cnt[16];

// the loop function runs over and over again forever
void loop() {
  digitalWrite(PA1, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(PA1, LOW);    // turn the LED off by making the voltage LOW
  delay(500);                       // wait for a second
  Serial.print("getCapValue=");
  Serial.print(getCapValue(GPIOB, 9), DEC);

  Serial.print(" getChangeTo0=");
  Serial.print(getChangeTo0(GPIOB, 9), DEC);

  Serial.print(" getDMAChangeTo0=");
  Serial.print(getDMAChangeTo0(GPIOB, 9), DEC);

  //getAllChangeTo0(GPIOB, ~(1<<2), changeTo0Cnt);
  getAllChangeTo0(GPIOB, (1<<9), changeTo0Cnt);
  Serial.print(" getAllChangeTo0=");
  Serial.print(changeTo0Cnt[9], DEC);

  //for( int i = 0; i< 24; i++ )
  //{
  //  Serial.print(" ");
  //  Serial.print((word)iosample[i], HEX);
  //}

  
  Serial.println("");
}
