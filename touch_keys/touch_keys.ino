/*

  touch_keys.ino
  
  
  Includes:
    - WS2812
    - Sensor key read
    - Sensor key mapping and learn mode
    
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

#define TOUCH_KEY_MIN_TH_DELTA_CAP 6

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

#define TOUCH_KEY_CNT 5
struct touch_status_struct touch_status_list[TOUCH_KEY_CNT] =  {
  { GPIOB, 5, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 7, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 6, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 8, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 9, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
};

/* maps a key (index into touch_status_list) to a RGB LED number */
int key_to_LED_map[TOUCH_KEY_CNT];

volatile int current_key = -1; // contains the current pressed key as global variable, assigned in signalKeyPressEvent() and signalKeyReleasedEvent() procedures.

#define TOUCH_MEASURE_CNT 5 /* GPIOA .. GPIOE */
struct touch_measure_struct touch_measure_list[TOUCH_MEASURE_CNT];


#define TOUCH_IO_SAMPLE_COUNT 128   /* do not change: this must match the generated sample statements */
uint16_t touch_io_sample_array[TOUCH_IO_SAMPLE_COUNT];


/*================================================*/
/*
  the following two procedures will
    - establish the links between "touch_measure_list" and "touch_status_list"
    - clear key_to_LED_map
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
  int i;
  for( i = 0; i < TOUCH_KEY_CNT; i++ )
    key_to_LED_map[i] = -1;

  fillTouchMeasure(touch_measure_list+0, GPIOA);
  fillTouchMeasure(touch_measure_list+1, GPIOB);
  fillTouchMeasure(touch_measure_list+2, GPIOC);
  fillTouchMeasure(touch_measure_list+3, GPIOD);
  fillTouchMeasure(touch_measure_list+4, GPIOE);  
}

/*================================================*/


void signalKeyPressEvent(int key)
{

  Serial.print("min cap=");
  Serial.print(touch_status_list[key].min_cap, DEC);
  Serial.print(" threshold=");
  Serial.print(touch_status_list[key].threshold_cap, DEC);
  Serial.print(" Key ");
  Serial.print(key, DEC);
  Serial.print(" pressed\n");  

  setRGB(key_to_LED_map[key], 200, 0, 100);  
  current_key = key;
}

void signalKeyReleasedEvent(int key)
{
  Serial.print("Key ");
  Serial.print(key, DEC);
  Serial.print(" released ");  

  for( int i = 0; i < TOUCH_KEY_CNT; i++ )
  {
    Serial.print(i, DEC);
    Serial.print(":");
    Serial.print(key_to_LED_map[i], DEC);
    Serial.print(" ");
  }

  Serial.print("\n");

  
  setRGB(key_to_LED_map[key], 10, 30, 10);  
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
        signalKeyPressEvent(key);
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

  Measure the capacitance at the touch sensor keys. Do this for the specified port. Selected pins of that port are measured in parallel.

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

// the setup function runs once when you press reset or power the board
void setup(void) {

  
  Serial.begin(9600);
  /*
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  */

  buildTouchMeasureList();
  initLEDMatrix();
    
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PA1, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB8, OUTPUT);
  pinMode(PB9, OUTPUT);


  key_to_LED_map[0] = 56; 
  key_to_LED_map[1] = 4;
  key_to_LED_map[2] = 58;
  key_to_LED_map[3] = 48; 
  key_to_LED_map[4] = 7;

}


int learn_key_led_map()
{
  static uint8_t led_number = 0;
  static uint32_t t = 0;
  static int state = 0; 
  if ( led_number >= 64 )
    return 1; // learn mode done
  switch(state)
  {
    case 0:
      t = millis();
      setRGB(led_number, 0, 200, 0);  
      state = 1;
      break;
    case 1:  // wait for keypress or timeout
      if ( current_key >= 0 )
      {
        key_to_LED_map[current_key] = led_number;
        state = 2;
        setRGB(led_number, 100, 100, 100);
      }
      else if ( (millis() - t) > 2000 )
      {
        setRGB(led_number, 0, 0, 0);  
        led_number++;
        state = 0; 
      }
      break;
     case 2:
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

//uint8_t master_mode = MASTER_MODE_LEARN_KEY_LED_MAP;
uint8_t master_mode = MASTER_MODE_NONE;

// the loop function runs over and over again forever
void loop() {
  updateTouchKeys();
  sendLEDMatrix();

  switch(master_mode)
  {
    case MASTER_MODE_LEARN_KEY_LED_MAP:
      if ( learn_key_led_map() != 0 )
        master_mode = MASTER_MODE_NONE;
      break;
  }
  
  //digitalWrite(PA1, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(100);                       // wait for a second
  //digitalWrite(PA1, LOW);    // turn the LED off by making the voltage LOW
  //delay(100);                       // wait for a second
  
  //getTouchCapForPortPins(GPIOB, (1<<9), port_touch_capacitance);
  //Serial.print(" port_touch_capacitance[9]=");
  //Serial.print(port_touch_capacitance[9], DEC);

  //Serial.println("");
}
