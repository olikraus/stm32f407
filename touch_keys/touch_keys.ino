/*

  touch_keys.ino
  
  
  Includes:
    - WS2812
    - Sensor key read
    - Sensor key mapping and learn mode

  Linux:
    stty -F /dev/ttyUSB0 sane 115200 && cat /dev/ttyUSB0
    or stty -F /dev/ttyUSB0 sane 115200 igncr  && cat /dev/ttyUSB0
    screen /dev/ttyUSB0  115200 (terminate with "C-a k" or "C-a \")
    minicom -D /dev/ttyUSB0  -b 115200 (terminate with "C-a x", change CR mode: "C-a u", disable HW control flow!)

  --> 9600

  Ports:
  A0,   User Button
  A1,   User LED
  A7, --> SPI1 MOSI, used for LED Matrix
  A9, A10, A11, A12, 
  A13  SWDIO
  A14  SWCLK
  PB12 seems to  have a resistor towards GND  --> Not used
  C14, C15 --> not on header
  
  Currently unused: B12 B15 D8 D10 E0 E1
  
  Ports not listed above are part of the sensor input

*/

/*================================================*/
/*

  SystemCoreClock: 168 MHz
  APB2: 84Mhz 

  SPI Clock BR = 100 --> Clock Divide by 32 --> 84 / 32 = 2.625 MHz --> 380ns

  per bit:
    0: 1000 --> 380ns pulse
    1: 1100 --> 760ns pulse
  
*/

struct _ws2812_spi
{
  volatile uint8_t *spi_data;
  volatile uint32_t byte_cnt;    // remaining bytes
  volatile uint32_t b;    // the current byte
  volatile uint32_t bit_cnt;
  volatile uint32_t post_data_wait;
  volatile uint32_t isr_cnt;
  volatile uint32_t spi_status_register;
  volatile uint32_t isr_ticks;
  volatile uint32_t max_isr_ticks;
  volatile int in_progress;
};
typedef struct _ws2812_spi ws2812_spi_t;

ws2812_spi_t ws2812_spi;


void ws2812_spi_init(void)
{
  ws2812_spi.isr_cnt = 0;
  
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  delay(1);
  SPI1->CR1 = 0;  /* disable SPI */
  SPI1->I2SCFGR = 0;  /* no I2S */
  SPI1->CR2 = 0; /* disable interrupts */

  /* PA7, AF5 --> SPI1 MOSI */

  GPIOA->MODER &= ~GPIO_MODER_MODER7;
  GPIOA->MODER |= GPIO_MODER_MODER7_1; /* alternate function mode */

  GPIOA->OTYPER &= ~GPIO_OTYPER_OT7;  /* push pull */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD7;  /* no pullup/pulldown */
  
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7;
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL7_2 | GPIO_AFRL_AFSEL7_0; /* configure AF5, bitmask 0101 */

  NVIC_SetPriority(SPI1_IRQn, 2);  /* 0: highes priority, 3: lowest priority */
  NVIC_EnableIRQ(SPI1_IRQn);    
}


/*
  This SPI handler will do an online conversion of the bit values to the WS2812B format 
  4 bits of data are converted into two bytes (16 bit) of data
  This procedure is time critical: 64 Sys-Clock-Cycle per SPI bit 64*16 = 1024 Clock Cycles
  ==> Upper Limit for this procedure are 1024 clock cycles
*/
extern "C" void __attribute__ ((interrupt)) SPI1_IRQHandler(void)
{
  //uint32_t start = SysTick->VAL;
  //uint32_t end;

  ws2812_spi.spi_status_register = SPI1->SR;
  ws2812_spi.isr_cnt++;

  if ( (ws2812_spi.spi_status_register & SPI_SR_TXE) == 0 ) 
    return;

  if ( ws2812_spi.byte_cnt > 0 || ws2812_spi.bit_cnt > 0 )
  {
    uint16_t d = 0x4444;
    uint16_t b;
    
    if ( ws2812_spi.bit_cnt == 0 )
    {
      ws2812_spi.b = *ws2812_spi.spi_data;
      ws2812_spi.spi_data++;
      ws2812_spi.byte_cnt--;
      ws2812_spi.bit_cnt = 2;
    }
    
    b = ws2812_spi.b;
    if ( b & 128 )
      d |= 0x2000;
    if ( b & 64 )
      d |= 0x0200;
    if ( b & 32 )
      d |= 0x0020;
    if ( b & 16 )
      d |= 0x0002;
    b <<= 4;
    ws2812_spi.b = b;
    
    SPI1->DR = d;
    ws2812_spi.bit_cnt--;
  }
  else
  {
    if ( ws2812_spi.post_data_wait > 0 )
    {
      /* wait for 50us, this are 125 SPI clocks (each is 0.4us) --> send 128 bits, 8x 16 Bit words */
      SPI1->DR = 0;
      ws2812_spi.post_data_wait--;
    }
    else
    {      
      /* ensure, that the SCK goes to low after the byte transfer... */
      
      /* disable interrupt, needs to be re-enabled whenever new data should be transmitted */
      SPI1->CR2 = 0;
      ws2812_spi.in_progress = 0;
    }
  } 
  /*
  end = SysTick->VAL;
  if ( start < end )
    start += SysTick->LOAD;
  start -= end;      // calculate the duration in ticks
  if ( ws2812_spi.max_isr_ticks < start )
    ws2812_spi.max_isr_ticks = start; // calculate maximum
  ws2812_spi.isr_ticks = start;
  */
}

void ws2812_spi_out(uint8_t *data, int cnt)
{

  /* wait until data is transmitted */
  if ( ws2812_spi.in_progress != 0 )
  {
    while( ws2812_spi.in_progress != 0 )
      ;
    delay(1);
  }

  SPI1->CR1 = 0;    /* disable SPI1 */
  
  SPI1->CR1 = 0
    | SPI_CR1_DFF   /* select 16 bit data format */
    | SPI_CR1_BR_2  /* 100: divide by 32 */
    | SPI_CR1_MSTR  /* master transmit */
    | SPI_CR1_SSM   /* SW Slave Management */
    | SPI_CR1_SSI   /* Internal Slave Select */
    | SPI_CR1_BIDIMODE /* select single line (Master: MOSI pin) bidirectional mode */
    | SPI_CR1_BIDIOE   /* select transmit mode*/
    ;
  
  SPI1->CR2 = 0
    | SPI_CR2_TXEIE /* buffer empty interrupt */
    ;
  
  ws2812_spi.spi_data = data;
  ws2812_spi.byte_cnt = cnt;
  ws2812_spi.bit_cnt = 0;
  ws2812_spi.post_data_wait = 9;    /* 8 would be sufficient, use 9 to be on the safe side */
  ws2812_spi.in_progress  = 1;


  SPI1->CR1 |= SPI_CR1_SPE;   /* enable SPI */
  
  //ws2812_spi.spi_status_register = SPI1->SR;

  /* load first byte, so that the TXRDY interrupt will be generated */
  /* this is just a zero byte and is ignored by the WS2812B */
  SPI1->DR = 0;  

}

/*================================================*/

#define LED_CNT 64
uint8_t LEDMatrixData[LED_CNT*3];

void initLEDMatrix(void)
{
  ws2812_spi_init();
}

void sendLEDMatrix(void)
{
  ws2812_spi_out(LEDMatrixData, LED_CNT*3);
}

void setRGB(uint8_t pos, uint8_t r, uint8_t g, uint8_t b)
{
  if ( pos < 64 )
  {
    LEDMatrixData[pos*3] = g;
    LEDMatrixData[pos*3+1] = r;
    LEDMatrixData[pos*3+2] = b;
  }
}

void clearRGBMatrix(void)
{
  for( uint8_t i = 0; i < 64; i++ )
  {
    setRGB(i, 0, 0, 0);
  }  
}

/* https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both */
void hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region, remainder, p, q, t;

    if (s == 0)
    {
        *r = v;
        *g = v;
        *b = v;
        return ;
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6; 

    p = (v * (uint16_t)(255 - s)) >> 8;
    q = (v * (uint16_t)(255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (uint16_t)(255 - ((s * (uint16_t)(255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            *r = v; *g = t; *b = p;
            break;
        case 1:
            *r = q; *g = v; *b = p;
            break;
        case 2:
            *r = p; *g = v; *b = t;
            break;
        case 3:
            *r = p; *g = q; *b = v;
            break;
        case 4:
            *r = t; *g = p; *b = v;
            break;
        default:
            *r = v; *g = p; *b = q;
            break;
    }
}

void setHSV(uint8_t pos, uint8_t h, uint8_t s, uint8_t v)
{
  hsv_to_rgb(h, s, v, LEDMatrixData+pos*3+1, LEDMatrixData+pos*3, LEDMatrixData+pos*3+2);
}


/*================================================*/

#define TOUCH_KEY_STATUS_RELEASED 0
#define TOUCH_KEY_STATUS_RP_DEBOUNCE1 1
#define TOUCH_KEY_STATUS_PRESSED 10
#define TOUCH_KEY_STATUS_PR_DEBOUNCE1 11

/* 
  the following value defines the sensitivity 
  higher values: less sensitive 
  lower values: more sensitive, but risk of faulty detects
*/
#define TOUCH_KEY_MIN_TH_DELTA_CAP 12

/* list of all GPIO lines, used as a sensor key */
struct touch_status_struct {
  GPIO_TypeDef *gpio; /* e.g. GPIOB */
  uint16_t pin;   /* pin number within that GPIO block (0..15) */
  uint16_t min_cap;   /* automatically calculated, typical values seem to be 18..19 */
  uint16_t threshold_cap;   /* if the cap value is below this value, then an untouched touch pad is assumed, use 0 for default */
  uint16_t status;
  uint32_t time;
};

/* helper struct, used to measure up to 16 GPIO lines. This struct will refer to "touch_status_struct" */
struct touch_measure_struct {
  GPIO_TypeDef *gpio; /* e.g. GPIOB */
  uint16_t mask;    /* each set pin means, that this GPIO should be considered */
  int16_t touch_status_index[16];    /* index into the touch status list, negative value means, that the pin is not used (0 in the mask) */
};



/*================================================*/

/* global variables */

/*
  the touch status list contains a list of all possible sensor inputs
  some of the ports can be commented if they are used for otherwise
*/
struct touch_status_struct touch_status_list[] =  {
  //{ GPIOA, 0, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  //{ GPIOA, 1, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},  
  { GPIOA, 2, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOA, 3, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOA, 4, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOA, 5, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOA, 6, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  //{ GPIOA, 7, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},  // MOSI for LED matrix
  { GPIOA, 8, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  //{ GPIOA, 9, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},    // for USART, not on header
  //{ GPIOA, 10, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},   // for USART, not on header
  
  //{ GPIOA, 11, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},   // for USART
  //{ GPIOA, 12, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},   // for USART
  //{ GPIOA, 13, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},  // SWDIO
  //{ GPIOA, 14, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},   // SWDCLK
  { GPIOA, 15, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},

  { GPIOB, 0, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 1, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  //{ GPIOB, 2, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 3, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  //{ GPIOB, 4, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  
  { GPIOB, 5, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 6, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 7, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 8, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 9, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 10, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 11, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 12, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 13, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 14, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 15, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  
  { GPIOC, 0, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 1, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 2, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 3, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 4, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 5, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 6, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 7, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 8, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 9, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 10, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 11, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 12, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 13, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0}, 
  //{ GPIOC, 14, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},  // not available on Header
  //{ GPIOC, 15, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0}, // not available on Header

  { GPIOD, 0, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 1, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 2, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 3, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 4, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 5, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 6, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 7, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 8, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 9, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 10, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 11, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 12, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 13, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 14, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 15, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},

  { GPIOE, 0, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 1, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 2, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 3, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 4, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 5, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 6, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 7, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 8, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 9, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 10, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 11, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 12, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 13, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 14, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 15, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  
};

#define TOUCH_KEY_CNT (sizeof(touch_status_list)/sizeof(struct touch_status_struct))
//#define TOUCH_KEY_CNT 5

/* maps a key (index into touch_status_list) to a RGB LED number */

//int16_t key_to_LED_map[TOUCH_KEY_CNT];  // assign all zeros, this will enforce a complete reset of the assignment

int16_t key_to_LED_map[TOUCH_KEY_CNT]={
/*0:*/36,51,21,25,-1,20,17,44,59,8,
/*10:*/56,58,4,48,7,49,19,-1,63,55,
/*20:*/-1,22,41,28,5,46,11,57,31,12,
/*30:*/27,14,24,38,2,62,32,37,40,42,
/*40:*/15,26,9,-1,16,-1,45,3,30,0,
/*50:*/53,-1,-1,52,39,10,47,6,50,1,
/*60:*/43,23,18,13,29,54,60};  

volatile int current_key = -1; // contains the current pressed key as global variable, assigned in signalKeyPressEvent() and signalKeyReleasedEvent() procedures.

#define TOUCH_MEASURE_CNT 5 /* GPIOA .. GPIOE */
struct touch_measure_struct touch_measure_list[TOUCH_MEASURE_CNT];


#define TOUCH_IO_SAMPLE_COUNT 128   /* do not change: this must match the generated sample statements */
uint16_t touch_io_sample_array[TOUCH_IO_SAMPLE_COUNT];


/*================================================*/
/*
  the following two procedures will establish the links between "touch_measure_list" and "touch_status_list"
*/


void fillTouchMeasure(struct touch_measure_struct *m, GPIO_TypeDef *gpio)
{
  int i;
  
  m->gpio = gpio;
  m->mask = 0;
  for( i = 0; i < 16; i++ )
    m->touch_status_index[i] = -1;
    
  for( i = 0; i < TOUCH_KEY_CNT; i++ )
  {
    if ( touch_status_list[i].gpio == gpio )
    {
      m->mask |= 1 << touch_status_list[i].pin;
      m->touch_status_index[touch_status_list[i].pin] = i;
    }
  }
}

void buildTouchMeasureList(void)
{
  /*
  int i;
  for( i = 0; i < TOUCH_KEY_CNT; i++ )
    key_to_LED_map[i] = -1;
  */

  fillTouchMeasure(touch_measure_list+0, GPIOA);
  fillTouchMeasure(touch_measure_list+1, GPIOB);
  fillTouchMeasure(touch_measure_list+2, GPIOC);
  fillTouchMeasure(touch_measure_list+3, GPIOD);
  fillTouchMeasure(touch_measure_list+4, GPIOE);  
}

/*
  get the key for a given led
  return -1 if the led can not be controlled
*/
int getKeyByLED(int led)
{
  int i;
  for( i = 0; i < TOUCH_KEY_CNT; i++ )
    if ( key_to_LED_map[i] == led )
      return i;
  return -1;
}

/*
  removes duplicate enties from the key to LED mapping table
*/
void fixKeyToLEDMap(void)
{
  int16_t i, j, cnt;
  int16_t led = -1;
  for( i = 0; i < TOUCH_KEY_CNT; i++ )
  {
    if ( key_to_LED_map[i] >= 0 )
    {
      led = -1;
      /* find duplicates */
      for( j = i+1; j < TOUCH_KEY_CNT; j++ )
      {
        if ( key_to_LED_map[i] == key_to_LED_map[j] )
        {
          led = key_to_LED_map[i];
          break;
        }
      }
      /* any duplicates found? */
      if ( led >= 0 )
      {
        /* erase duplicates */
        cnt = 0;
        for( j = 0; j < TOUCH_KEY_CNT; j++ )
        {
          if ( key_to_LED_map[j] == led )
          {
            key_to_LED_map[j] = -1;
            cnt++;
          }
        }
        Serial.print("fixKeyToLEDMap: Removed LED ");
        Serial.print(led, DEC);
        Serial.print(" cnt=");
        Serial.print(cnt, DEC);
        Serial.print("\n");
      }
    }
  }
}

/*
  returns the number of sensor keys assigned, should be 60 
*/
int16_t getAssignedKeyCount(void)
{
  int16_t i;
  int16_t cnt = 0;
  for( i = 0; i < TOUCH_KEY_CNT; i++ )
    if ( key_to_LED_map[i] >= 0 )
      cnt++;
  return cnt;
}

/*
    Output the current key to LED map status to serial port
*/
void showKeyMapStatus(void)
{
  int i;
  
  /* Print the key_to_LED_map: Can be copied to the code */
  
  Serial.print("int16_t key_to_LED_map[TOUCH_KEY_CNT]={");
  for( i = 0; i < TOUCH_KEY_CNT; i++ )
  {
    if ( i % 10 == 0 )
    {
      Serial.print("/*");
      Serial.print(i, DEC);
      Serial.print(":*/");
    }
    Serial.print(key_to_LED_map[i], DEC);
    if ( i+1 != TOUCH_KEY_CNT )
      Serial.print(",");
  }
  Serial.print("};\n");

  /* Print the total number of assigned keys */

  Serial.print("Assigned keys: ");
  Serial.print(getAssignedKeyCount(), DEC);
  Serial.print("\n");  
  
  /* print a list of GPIOs, which are not assigned */
  
  Serial.print("Unassigned GPIOs: ");
  for( i = 0; i < TOUCH_KEY_CNT; i++ )
  {
    if ( key_to_LED_map[i] < 0 )
    {
      if ( touch_status_list[i].gpio == GPIOA ) Serial.print("A");
      else if ( touch_status_list[i].gpio == GPIOB ) Serial.print("B");
      else if ( touch_status_list[i].gpio == GPIOC ) Serial.print("C");
      else if ( touch_status_list[i].gpio == GPIOD ) Serial.print("D");
      else if ( touch_status_list[i].gpio == GPIOE ) Serial.print("E");
      else Serial.print("?");
      Serial.print(touch_status_list[i].pin, DEC);
      Serial.print(" ");
    }
  }  
  Serial.print("\n");
}

/*================================================*/


/*

  void signalKeyPressEvent(int key, uint16_t cap)

  called by updateTouchStatus(), which is called by updateTouchKeys().

*/
void signalKeyPressEvent(int key, uint16_t cap)
{

  Serial.print("min cap=");
  Serial.print(touch_status_list[key].min_cap, DEC);
  Serial.print(" threshold=");
  Serial.print(touch_status_list[key].threshold_cap, DEC);
  Serial.print(" current cap=");
  Serial.print(cap, DEC);
  Serial.print(" Key ");
  Serial.print(key, DEC);
  Serial.print(" pressed\n");  

  setRGB(key_to_LED_map[key], 200, 0, 100);  
  current_key = key;
}


/*

  void signalKeyReleasedEvent(int key)

  called by updateTouchStatus(), which is called by updateTouchKeys().

*/
void signalKeyReleasedEvent(int key)
{
  Serial.print("Key ");
  Serial.print(key, DEC);
  Serial.print(" released\n");  

  //showKeyMapStatus();
  
  setRGB(key_to_LED_map[key], 0, 20, 0);  
  current_key = -1;
}

/*
  Update status of one key, based on the current measured capacitance value
  key is an index into "touch_status_list" array (0 .. TOUCH_KEY_CNT-1].
  Status is updated within "touch_status_list".
 
  updateTouchStatus() is called by updateTouchKeys().
*/
void updateTouchStatus(int key, uint16_t cap)
{
  struct touch_status_struct *s = touch_status_list+key;

  if ( s->min_cap == 0 )
  {
    s->min_cap = cap;
  }
  else if ( s->min_cap > cap )
  {
    s->min_cap = cap;
  }
  s->threshold_cap = s->min_cap + TOUCH_KEY_MIN_TH_DELTA_CAP;
  
  switch(s->status)
  {
    case TOUCH_KEY_STATUS_RELEASED:
      if ( cap >= s->threshold_cap )
        s->status = TOUCH_KEY_STATUS_RP_DEBOUNCE1;
      break;
    case TOUCH_KEY_STATUS_RP_DEBOUNCE1:
      if ( cap >= s->threshold_cap )
      {
        s->status = TOUCH_KEY_STATUS_PRESSED;
        s->time = millis();
        signalKeyPressEvent(key, cap);
      }
      else
        s->status = TOUCH_KEY_STATUS_RELEASED;
      break;
    case TOUCH_KEY_STATUS_PRESSED:
      if ( cap < s->threshold_cap )
        s->status = TOUCH_KEY_STATUS_PR_DEBOUNCE1;
      break;
    case TOUCH_KEY_STATUS_PR_DEBOUNCE1:
      if ( cap < s->threshold_cap )
      {
        s->status = TOUCH_KEY_STATUS_RELEASED;
        signalKeyReleasedEvent(key);
        s->time = 0;
      }
      else
        s->status = TOUCH_KEY_STATUS_PRESSED;      
      break;
    default:
      s->status = TOUCH_KEY_STATUS_RELEASED;
      break;
  }
}

/*================================================*/


/*

 Do a binary search in the global iosample array.
 Look for a 1 to 0 transition and return the position of the 1-0 transition (the first 0 value)
 mask: only one bit should be set here, 
  which must be the bit for which the 1 to 0 transition will be searched

 returns:
  0 if all bits are 0
  1..TOUCH_IO_SAMPLE_COUNT-1 for the initial number of 1 found
  TOUCH_IO_SAMPLE_COUNT if all bits are 1

  called by getTouchCapForPortPins()

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

  Measure the capacitance at the touch sensor keys. 
  Do this for the specified port. Selected pins of that port are measured in parallel.

  Args:
    gpio: GPIO port for which the measure should happen
    selectMask: consider only pins where the bit is set inside mask
    changeTo0Cnt: For each of the selected GPIO ports of selected GPIO block, this will contain the 1-0 transition time. 
      A key is detected if this 1-0 transition time is small.
      
  Called by
    updateTouchKeys()
  
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

/*
  void updateTouchKeys(void)
  
  Calculate the status of all touch senser keys.
  This will call signalKeyPressEvent() and signalKeyReleasedEvent() procedures.
  
*/
void updateTouchKeys(void)
{
  uint16_t changeTo0Cnt[16];
  int i, pin, key;
  
  for( i = 0; i < TOUCH_MEASURE_CNT; i++ )
  {
    getTouchCapForPortPins(touch_measure_list[i].gpio, touch_measure_list[i].mask, changeTo0Cnt);
    for( pin = 0; pin < 16; pin++ )
    {
      key = touch_measure_list[i].touch_status_index[pin];
      if ( changeTo0Cnt[pin] > 0 && key >= 0 )
      {
        updateTouchStatus(key, changeTo0Cnt[pin]);
      }
    }
  }
}

/*================================================*/
/* Learn mode: Graph */

/* graph element struct (actually it is the edge of the ikosidodecaeder */
struct ge_struct
{
  int16_t led;               // key can be derived with getKeyByLED(led)
  int16_t key;              // LED can be derived via key_to_LED_map[key]
  int16_t next[6];          // each ikosidodecaeder has six neighbours
};

struct ge_struct gel[60];

uint16_t getGELPosByKey(uint16_t key)
{
  uint16_t pos;
  for( pos = 0; pos < 60; pos++ )
    if ( gel[pos].key == key )
      return pos;
  return -1;
}

/*
  Learn algo:
    show a ge, let user choose the most right neighbour for each pentagon --> next[5]
    show a ge, let user choose the most right neighbour for each triangle --> next[2]
    For the next[5], next[0] will be the shown ge
    later:
      for( i = 0; i < 60; i++)
        gel[gel[i].next[5]].next[0] = i;

      next[4] = next[5].next[2]
      next[1] = next[0].next[3]

*/

void clearGEL(void)
{
  int i, j;
  int key_to_led_pos = 0;

  Serial.print("clear GEL\n");

  // assumes that getAssignedKeyCount() >= 60 !!!!!
  
  for( i = 0; i < 60; i++)
  {
    while(key_to_LED_map[key_to_led_pos] < 0 )
      key_to_led_pos++;
    gel[i].led = key_to_LED_map[key_to_led_pos];
    gel[i].key = key_to_led_pos;
    for( j = 0; j < 6; j++)
        gel[i].next[j] = -1;
    key_to_led_pos++;
  }

}

int learnPentagon()
{
  static int gel_pos = 0; 
  static int state = 0; 
  static uint32_t t = 0;
  static uint32_t wait_time_ms = 4000;
  int i;
  
  switch(state)
  {
    case 0:
      for( i = 0; i < 60; i++ )
      {
        if ( gel[i].next[5] < 0 )
        {
          Serial.print("pentagon learn mode: continue with learning\n");
          break;
        }
      }
      
      if ( i >= 60 )
      {
        state = 9;     // pentagon finished
        break;
      }
    
      // find a suitable edge, which could be checked
      while( gel[gel_pos].next[5] >= 0 )
      {
        gel_pos++;
        if ( gel_pos >= 60 )
          gel_pos = 0;
      }
      
      t = millis();
      setRGB(gel[gel_pos].led, 200, 200, 0);  
      Serial.print("press pentagon next right edge (led number=");
      Serial.print(gel[gel_pos].led, DEC);
      Serial.print(")\n");
      state = 1;
      break;
    case 1:  // wait for keypress or timeout
      if ( current_key >= 0 )
      {
        setRGB(gel[gel_pos].led, 100, 100, 200);
        gel[gel_pos].next[5] = getGELPosByKey(current_key);
        if ( gel_pos == gel[gel_pos].next[5] )
        {
          gel[gel_pos].next[5] = -1;            // illegal self assignment
          Serial.print("pentagon edge self asignment ignored\n");
        }
        else
        {
          Serial.print("pentagon edge ");
          Serial.print(gel_pos, DEC);
          Serial.print("(led= ");
          Serial.print(gel[gel_pos].led, DEC);
          Serial.print(") has next right edge ");
          Serial.print(gel[gel_pos].next[5], DEC);
          Serial.print("\n");          
          gel_pos = gel[gel_pos].next[5];
        }
        state = 2;
      }
      else if ( (millis() - t) > wait_time_ms )
      {
        setRGB(gel[gel_pos].led, 0, 0, 0);  
        gel_pos++;
        clearRGBMatrix();
        state = 0; 
      }
      break;
     case 2:    // wait for key release
        if ( current_key == -1 )
        {
          //setRGB(gel[gel_pos].led, 0, 0, 0);  
          clearRGBMatrix();
          state = 0;         
        }
        break;
      
    case 9: // finished
      Serial.print("pentagon learn mode done (9)\n");
      state = 0;
      return 1;
  }
  return 0;
}


/*================================================*/
/* Learn mode: Key to LED Mapping */

int learnKeyLEDMap()
{
  static uint8_t led_number = 0;
  static uint32_t t = 0;
  static int state = 0; 
  static uint32_t wait_time_ms = 4000;

  
  if ( getAssignedKeyCount() >= 60 )
  {
    Serial.print("Key LED learing done\n");
    return 1; // learn mode done
  }
    
  switch(state)
  {
    case 0: // find the next open key
      /* find the next unassigned led */
      while( getKeyByLED(led_number) >= 0 )
        led_number++;
      if ( led_number >= 64 )
        led_number = 0;
        
      t = millis();
      setRGB(led_number, 200, 200, 0);  
      Serial.print("press sensor for led number=");
      Serial.print(led_number, DEC);
      Serial.print("\n");
      state = 1;
      break;
    case 1:  // wait for keypress or timeout
      if ( current_key >= 0 )
      {
        key_to_LED_map[current_key] = led_number;
        Serial.print("New assignment: Key ");
        Serial.print(current_key, DEC);
        Serial.print(" assigned to LED ");
        Serial.print(led_number, DEC);
        Serial.print("\n");
        fixKeyToLEDMap();
        setRGB(led_number, 100, 100, 200);
        showKeyMapStatus();
        state = 2;
      }
      else if ( (millis() - t) > wait_time_ms )
      {
        setRGB(led_number, 0, 0, 0);  
        led_number++;
        clearRGBMatrix();
        state = 0; 
      }
      break;
     case 2:    // wait for key release
        if ( current_key == -1 )
        {
          setRGB(led_number, 0, 0, 0);  
          led_number++;
          state = 0;         
        }
        break;
  }
  return 0;
}



//uint16_t port_touch_capacitance[16];


#define MASTER_MODE_NONE 0
#define MASTER_MODE_LEARN_KEY_LED_MAP 1
#define MASTER_MODE_LEARN_PENTAGON 2

uint8_t master_mode = MASTER_MODE_NONE;

// the setup function runs once when you press reset or power the board
void setup(void) {
  //Serial.begin(9600);
  Serial.begin(115200);
  
  delay(1200);
  /*
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  */

  Serial.println("buildTouchMeasureList");
  buildTouchMeasureList();
  
  Serial.println("initLEDMatrix");
  initLEDMatrix();
    
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PA1, OUTPUT);

  pinMode(PA2, OUTPUT);
  pinMode(PA3, OUTPUT);
  pinMode(PA4, OUTPUT);
  pinMode(PA5, OUTPUT);
  pinMode(PA6, OUTPUT);
  
  // pinMode(PA7, OUTPUT); // MOSI for LED matrix
  pinMode(PA8, OUTPUT);
  //pinMode(PA9, OUTPUT); // not on header
  //pinMode(PA10, OUTPUT); // not on header
  
  //pinMode(PA11, OUTPUT); // breaks USART
  //pinMode(PA12, OUTPUT); // breaks USART
  //pinMode(PA13, OUTPUT); // not on header
  //pinMode(PA14, OUTPUT); // not on header
  pinMode(PA15, OUTPUT);

  pinMode(PB0, OUTPUT);
  pinMode(PB1, OUTPUT);
  //pinMode(PB2, OUTPUT);
  pinMode(PB3, OUTPUT);
  //pinMode(PB4, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB8, OUTPUT);
  pinMode(PB9, OUTPUT);
  pinMode(PB10, OUTPUT);
  pinMode(PB11, OUTPUT);
  pinMode(PB12, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB15, OUTPUT);

  pinMode(PC0, OUTPUT);
  pinMode(PC1, OUTPUT);
  pinMode(PC2, OUTPUT);
  pinMode(PC3, OUTPUT);
  pinMode(PC4, OUTPUT);
  pinMode(PC5, OUTPUT);
  pinMode(PC6, OUTPUT);
  pinMode(PC7, OUTPUT);
  pinMode(PC8, OUTPUT);
  pinMode(PC9, OUTPUT);
  pinMode(PC10, OUTPUT);
  pinMode(PC11, OUTPUT);
  pinMode(PC12, OUTPUT);
  pinMode(PC13, OUTPUT);
  //pinMode(PC14, OUTPUT);
  //pinMode(PC15, OUTPUT);

  pinMode(PD0, OUTPUT);
  pinMode(PD1, OUTPUT);
  pinMode(PD2, OUTPUT);
  pinMode(PD3, OUTPUT);
  pinMode(PD4, OUTPUT);
  pinMode(PD5, OUTPUT);
  pinMode(PD6, OUTPUT);
  pinMode(PD7, OUTPUT);
  pinMode(PD8, OUTPUT);
  pinMode(PD9, OUTPUT);
  pinMode(PD10, OUTPUT);
  pinMode(PD11, OUTPUT);
  pinMode(PD12, OUTPUT);
  pinMode(PD13, OUTPUT);
  pinMode(PD14, OUTPUT);
  pinMode(PD15, OUTPUT);

  pinMode(PE0, OUTPUT);
  pinMode(PE1, OUTPUT);
  pinMode(PE2, OUTPUT);
  pinMode(PE3, OUTPUT);
  pinMode(PE4, OUTPUT);
  pinMode(PE5, OUTPUT);
  pinMode(PE6, OUTPUT);
  pinMode(PE7, OUTPUT);
  pinMode(PE8, OUTPUT);
  pinMode(PE9, OUTPUT);
  pinMode(PE10, OUTPUT);
  pinMode(PE11, OUTPUT);
  pinMode(PE12, OUTPUT);
  pinMode(PE13, OUTPUT);
  pinMode(PE14, OUTPUT);
  pinMode(PE15, OUTPUT);

  fixKeyToLEDMap();

  Serial.println("Setup done");

  if (  getAssignedKeyCount() < 60 )
  {
    master_mode = MASTER_MODE_LEARN_KEY_LED_MAP;
    Serial.print("Key LED mapping mode started\n");
  }
  else
  {
    clearGEL();
    Serial.print("Pentagon learn mode started\n");
    master_mode = MASTER_MODE_LEARN_PENTAGON;
  }
}


// the loop function runs over and over again forever
void loop() 
{
  updateTouchKeys();
  sendLEDMatrix();

  switch(master_mode)
  {
    case MASTER_MODE_LEARN_KEY_LED_MAP:
      if ( learnKeyLEDMap() != 0 )
        master_mode = MASTER_MODE_NONE;
      break;
    case MASTER_MODE_LEARN_PENTAGON:
      if ( learnPentagon() != 0 )
        master_mode = MASTER_MODE_NONE;
      break;    
  }
  
  if ( (millis() & 0x01ff) < 0xff )
  {
    digitalWrite(PA1, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else
  {
    digitalWrite(PA1, LOW);    // turn the LED off by making the voltage LOW
  }
}
