/*
  touch_keys.ino

*/

/*================================================*/


/* global variables */

#define TOUCH_IO_SAMPLE_COUNT 128   /* do not change: this must match the generated sample statements */
uint16_t touch_io_sample_array[TOUCH_IO_SAMPLE_COUNT];


/*================================================*/


/*

 Do a binary search in the global iosample array.
 Look for a 1 to 0 transition and return the position of the 1-0 transition (the first 0 value)
 mask: only one bit should be set here, which must be the bit for which the 1 to 0 transition will be searched

 returns:
  0 if all bits are 0
  1..TOUCH_IO_SAMPLE_COUNT-1 for the initial number of 1 found
  TOUCH_IO_SAMPLE_COUNT if all bits are 1

*/
uint32_t getTo0PosByBinarySearch(uint16_t mask)
{
  uint16_t l = 0;
  uint16_t r = TOUCH_IO_SAMPLE_COUNT-1;
  uint32_t mid;
  while(l < r) 
  {
    mid = l+(r-l)/2;
    if ( touch_io_sample_array[mid] & mask )
      l = mid+1;
    else
      r = mid-1;
  }
  if ( l >= TOUCH_IO_SAMPLE_COUNT ) return TOUCH_IO_SAMPLE_COUNT;
  if ( touch_io_sample_array[l] & mask ) return l+1;
  return l;
}

/*
 selectMask: consider only pins where the bit is set inside mask
*/
void getTouchCapForPortPins(GPIO_TypeDef *gpio, uint16_t selectMask, uint16_t changeTo0Cnt[16])
{
  

  uint16_t *s = touch_io_sample_array;
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

  /* generate all 128 GPIO read statements, disable interrupts during this GPIO read */
  __disable_irq();
  gpio->MODER &= ~m11;    /* change to input */
  GPIO_ALLSAMPLE_LINE128
  __enable_irq();
  
  /* data is now in the iosample array */
  
  for( uint16_t i = 0; i < 16; i++ )
  {
    if ( selectMask & (1<<i) )  
      changeTo0Cnt[i] = getTo0PosByBinarySearch(1<<i);
    else
      changeTo0Cnt[i] = 0;
  }  
}

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

}

uint16_t port_touch_capacitance[16];

// the loop function runs over and over again forever
void loop() {
  digitalWrite(PA1, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(PA1, LOW);    // turn the LED off by making the voltage LOW
  delay(500);                       // wait for a second
  
  getTouchCapForPortPins(GPIOB, (1<<9), port_touch_capacitance);
  Serial.print(" port_touch_capacitance[9]=");
  Serial.print(port_touch_capacitance[9], DEC);

  Serial.println("");
}
