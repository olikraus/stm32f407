/*

  ikosi_learn.ino
  
  tool to train "key_to_LED_map" and "gel" arrays
  
  
  Includes:
    - WS2812
    - Sensor key read
    - Sensor key mapping and learn mode

  Linux:
    stty -F /dev/ttyUSB0 sane 115200 && cat /dev/ttyUSB0
    or stty -F /dev/ttyUSB0 sane 115200 igncr  && cat /dev/ttyUSB0
    screen /dev/ttyUSB0  115200 (terminate with "C-a k" or "C-a \")
    minicom -D /dev/ttyUSB0  -b 115200 (terminate with "C-a x", change CR mode: "C-a u", disable HW control flow!)


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

#include <stdarg.h>

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

void p(const char *fmt, ...)
{
  static char s[1024];
  va_list va;
  va_start(va, fmt);
  vsnprintf(s, 1024, fmt, va);
  va_end(va);
  Serial.print(s);
}

void pn(const char *fmt, ...)
{
  static char s[1024];
  va_list va;
  va_start(va, fmt);
  vsnprintf(s, 1024, fmt, va);
  va_end(va);
  Serial.print(s);
  Serial.print("\n");
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
  uint16_t arduino_pin;         /* arduino pin number */
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
  
  The index into touch_status_list is refered as "key" in this software.
*/
struct touch_status_struct touch_status_list[] =  {
  //{ GPIOA, 0, PA0, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  //{ GPIOA, 1, PA1, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},  
  { GPIOA, 2, PA2, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOA, 3, PA3, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOA, 4, PA4, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOA, 5, PA5, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOA, 6, PA6, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  //{ GPIOA, 7, PA7, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},  // MOSI for LED matrix
  { GPIOA, 8, PA8, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  //{ GPIOA, 9, PA9, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},    // for USART, not on header
  //{ GPIOA, 10, PA10, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},   // for USART, not on header
  
  //{ GPIOA, 11, PA11, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},   // for USART
  //{ GPIOA, 12, PA12, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},   // for USART
  //{ GPIOA, 13, PA13, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},  // SWDIO
  //{ GPIOA, 14, PA14, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},   // SWDCLK
  { GPIOA, 15, PA15, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},

  { GPIOB, 0, PB0, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 1, PB1, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  //{ GPIOB, 2, PB2, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 3, PB3, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  //{ GPIOB, 4, PB4, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  
  { GPIOB, 5, PB5, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 6, PB6, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 7, PB7, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 8, PB8, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 9, PB9, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 10, PB10, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 11, PB11, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 12, PB12, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 13, PB13, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 14, PB14, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOB, 15, PB15, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  
  { GPIOC, 0, PC0, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 1, PC1, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 2, PC2, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 3, PC3, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 4, PC4, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 5, PC5, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 6, PC6, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 7, PC7, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 8, PC8, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 9, PC9, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 10, PC10, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 11, PC11, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 12, PC12, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOC, 13, PC13, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0}, 
  //{ GPIOC, 14, PC14, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},  // not available on Header
  //{ GPIOC, 15, PC15, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0}, // not available on Header

  { GPIOD, 0, PD0 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 1, PD1 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 2, PD2 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 3, PD3 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 4, PD4 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 5, PD5 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 6, PD6 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 7, PD7 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 8, PD8 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 9, PD9 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 10, PD11 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 11, PD11 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 12, PD12 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 13, PD13 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 14, PD14 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOD, 15, PD15 , 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},

  { GPIOE, 0, PE0, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 1, PE1, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 2, PE2, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 3, PE3, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 4, PE4, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 5, PE5, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 6, PE6, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 7, PE7, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 8, PE8, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 9, PE9, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 10, PE10, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 11, PE11, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 12, PE12, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 13, PE13, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 14, PE14, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  { GPIOE, 15, PE15, 0, 0, TOUCH_KEY_STATUS_RELEASED, 0},
  
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

const char *getGPIONameByKey(int key)
{
  static char name[16];
  strcpy(name, "?");
  if ( touch_status_list[key].gpio == GPIOA ) strcpy(name, "A");
  if ( touch_status_list[key].gpio == GPIOB ) strcpy(name, "B");
  if ( touch_status_list[key].gpio == GPIOC ) strcpy(name, "C");
  if ( touch_status_list[key].gpio == GPIOD ) strcpy(name, "D");
  if ( touch_status_list[key].gpio == GPIOE ) strcpy(name, "E");
  sprintf(name+1, "%d", touch_status_list[key].pin);
  return name;
}

const char *getKeyInfoString(int key)
{
  static char s[32];    /* max 23 */
  strcpy(s, "[");
  strcat(s, "key=");
  sprintf(s+strlen(s), "%d", key);
  strcat(s, " io=");
  strcat(s, getGPIONameByKey(key));
  strcat(s, " led=");
  sprintf(s+strlen(s), "%d]", key_to_LED_map[key]);
  return s;
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
int fixKeyToLEDMap(void)
{
  int16_t i, j, cnt;
  int16_t led = -1;
  int removed_led_cnt = 0;
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
            removed_led_cnt++;
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
  return removed_led_cnt;
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
      Serial.print(", ");
      Serial.print(getGPIONameByKey(i));
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
      /*
      if ( touch_status_list[i].gpio == GPIOA ) Serial.print("A");
      else if ( touch_status_list[i].gpio == GPIOB ) Serial.print("B");
      else if ( touch_status_list[i].gpio == GPIOC ) Serial.print("C");
      else if ( touch_status_list[i].gpio == GPIOD ) Serial.print("D");
      else if ( touch_status_list[i].gpio == GPIOE ) Serial.print("E");
      else Serial.print("?");
      Serial.print(touch_status_list[i].pin, DEC);
      */
      Serial.print(getGPIONameByKey(i));
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
  Serial.print(" (");
  Serial.print(getGPIONameByKey(key));
  Serial.print(") pressed\n");  

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
  Serial.print(" (");
  Serial.print(getGPIONameByKey(key));
  Serial.print(") released\n");  

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
/* Graph definitions */



/* graph element struct (actually it is the edge of the ikosidodecaeder */
struct ge_struct
{
  int16_t led;               // key can be derived with getKeyByLED(led)
  int16_t key;              // LED can be derived via key_to_LED_map[key]
  int16_t pentagon;     // pentagon number, starts with 0
  int16_t triangle;        // triangle number, starts with 0
  int16_t next[6];          // each ikosidodecaeder has six neighbour edges
  /*
    if edge is assumed to be part of a pentagon then
      next[0] is previous clock wise edge of the pentagon
      next[5] is next clock wise edge of the pentagon
    if edge is assumed to be part of a triangle then
      next[3] is previous clock wise edge of the triangle
      next[2] is next clock wise edge of the triangle
      
  */
};

struct ge_struct gel[60] = {
{36,0 /* A2 */,0,0,{-1,-1,53,-1,-1,27}}
,{51,1 /* A3 */,0,1,{-1,-1,7,-1,-1,0}}
,{21,2 /* A4 */,1,2,{-1,-1,18,-1,-1,26}}
,{25,3 /* A5 */,1,3,{-1,-1,34,-1,-1,2}}
,{20,5 /* A8 */,2,4,{-1,-1,26,-1,-1,44}}
,{17,6 /* A15 */,3,5,{-1,-1,15,-1,-1,28}}
,{44,7 /* B0 */,0,6,{-1,-1,41,-1,-1,51}}
,{59,8 /* B1 */,4,1,{-1,-1,46,-1,-1,59}}
,{8,9 /* B3 */,5,7,{-1,-1,9,-1,-1,54}}
,{56,10 /* B5 */,6,7,{-1,-1,50,-1,-1,13}}
,{58,11 /* B6 */,6,8,{-1,-1,14,-1,-1,12}}
,{4,12 /* B7 */,6,9,{-1,-1,37,-1,-1,9}}
,{48,13 /* B8 */,6,10,{-1,-1,49,-1,-1,11}}
,{7,14 /* B9 */,6,11,{-1,-1,24,-1,-1,10}}
,{49,15 /* B10 */,4,8,{-1,-1,32,-1,-1,17}}
,{19,16 /* B11 */,7,5,{-1,-1,43,-1,-1,31}}
,{63,18 /* B13 */,8,12,{-1,-1,59,-1,-1,22}}
,{55,19 /* B14 */,4,11,{-1,-1,13,-1,-1,45}}
,{22,21 /* C0 */,9,2,{-1,-1,23,-1,-1,29}}
,{41,22 /* C1 */,10,13,{-1,-1,45,-1,-1,36}}
,{28,23 /* C2 */,2,14,{-1,-1,27,-1,-1,4}}
,{5,24 /* C3 */,3,15,{-1,-1,48,-1,-1,5}}
,{46,25 /* C4 */,8,6,{-1,-1,6,-1,-1,57}}
,{11,26 /* C5 */,3,2,{-1,-1,2,-1,-1,21}}
,{57,27 /* C6 */,5,11,{-1,-1,17,-1,-1,8}}
,{31,28 /* C7 */,2,16,{-1,-1,55,-1,-1,41}}
,{12,29 /* C8 */,1,4,{-1,-1,56,-1,-1,38}}
,{27,30 /* C9 */,0,14,{-1,-1,38,-1,-1,6}}
,{14,31 /* C10 */,3,17,{-1,-1,40,-1,-1,56}}
,{24,32 /* C11 */,9,3,{-1,-1,3,-1,-1,47}}
,{38,33 /* C12 */,1,0,{-1,-1,0,-1,-1,3}}
,{2,34 /* C13 */,7,15,{-1,-1,21,-1,-1,42}}
,{62,35 /* D0 */,8,8,{-1,-1,10,-1,-1,16}}
,{32,36 /* D1 */,11,10,{-1,-1,12,-1,-1,55}}
,{37,37 /* D2 */,10,3,{-1,-1,29,-1,-1,53}}
,{40,38 /* D3 */,5,18,{-1,-1,47,-1,-1,58}}
,{42,39 /* D4 */,10,18,{-1,-1,35,-1,-1,34}}
,{15,40 /* D5 */,11,9,{-1,-1,52,-1,-1,33}}
,{26,41 /* D6 */,1,14,{-1,-1,20,-1,-1,30}}
,{9,42 /* D7 */,9,19,{-1,-1,54,-1,-1,48}}
,{16,44 /* D9 */,11,17,{-1,-1,44,-1,-1,43}}
,{45,46 /* D11 */,2,6,{-1,-1,22,-1,-1,20}}
,{3,47 /* D12 */,7,19,{-1,-1,39,-1,-1,50}}
,{30,48 /* D13 */,11,5,{-1,-1,5,-1,-1,37}}
,{0,49 /* D14 */,2,17,{-1,-1,28,-1,-1,25}}
,{53,50 /* D15 */,4,13,{-1,-1,58,-1,-1,7}}
,{52,53 /* E2 */,10,1,{-1,-1,1,-1,-1,19}}
,{39,54 /* E3 */,9,18,{-1,-1,36,-1,-1,39}}
,{10,55 /* E4 */,9,15,{-1,-1,31,-1,-1,18}}
,{47,56 /* E5 */,8,10,{-1,-1,33,-1,-1,32}}
,{6,57 /* E6 */,7,7,{-1,-1,8,-1,-1,52}}
,{50,58 /* E7 */,0,12,{-1,-1,16,-1,-1,1}}
,{1,59 /* E8 */,7,9,{-1,-1,11,-1,-1,15}}
,{43,60 /* E9 */,10,0,{-1,-1,30,-1,-1,46}}
,{23,61 /* E10 */,5,19,{-1,-1,42,-1,-1,35}}
,{18,62 /* E11 */,11,16,{-1,-1,57,-1,-1,40}}
,{13,63 /* E12 */,3,4,{-1,-1,4,-1,-1,23}}
,{29,64 /* E13 */,8,16,{-1,-1,25,-1,-1,49}}
,{54,65 /* E14 */,5,13,{-1,-1,19,-1,-1,24}}
,{60,66 /* E15 */,4,12,{-1,-1,51,-1,-1,14}}
};



uint16_t getGELPosByKey(uint16_t key)
{
  uint16_t pos;
  for( pos = 0; pos < 60; pos++ )
    if ( gel[pos].key == key )
      return pos;
  return -1;
}

void printGEL(void)
{
  uint16_t pos, i;
  pn("struct ge_struct gel[60] = {");
  for( pos = 0; pos < 60; pos++ )
  {
    if ( pos != 0 )
      p(",");
    p("{%d,%d /* %s */,%d,%d,{", gel[pos].led, gel[pos].key, getGPIONameByKey(gel[pos].key), gel[pos].pentagon, gel[pos].triangle);
    for( i = 0; i < 6; i++ )
    {
      if ( i != 0)
        p(",");
      p("%d", gel[pos].next[i]);
    }
    pn("}}");
  }
  pn("};");
}



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
    gel[i].pentagon = -1;
    gel[i].triangle = -1;
    for( j = 0; j < 6; j++)
        gel[i].next[j] = -1;
    key_to_led_pos++;
  }

}

/*
  check for a next loop with element at gel_pos with the given index (ge.next[index]).
  index is 5 for pentagon
  return value:
    -1 if no loop exists
    >= 0 for any loop count (5 for pentagon)
*/
int checkNextLoop(int index, int gel_pos)
{
  int cnt = 1;
  int pos = gel_pos;
  for(;;)
  {
    if ( gel[pos].next[index] < 0 )
      return -1;
    if ( gel[pos].next[index] == gel_pos )
      return cnt;
    if ( cnt > 60 )
      return cnt;
    pos = gel[pos].next[index];
    cnt++;
  }
}

/*================================================*/
/* Common preocedures for Pentagon and Triangle Learn Mode */

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

/*
  calculate next 0, 1, 3 & 4
  this function must be called only if isGELPentagonCorrect() and isGELTriangleCorrect() return true
*/
void calculateOtherNextGELValues(void)
{
  int i, j;
  /* get previous pentagon edge */
  for( i = 0; i < 60; i++)
    if ( gel[i].next[5] >= 0 )
      gel[gel[i].next[5]].next[0] = i;
      
  /* get previous triangle edge */
  for( i = 0; i < 60; i++)
    if ( gel[i].next[2] >= 0 )
      gel[gel[i].next[2]].next[3] = i;

  for( i = 0; i < 60; i++)
    if ( gel[i].next[5] >= 0 )
      gel[i].next[4] = gel[gel[i].next[5]].next[2];
      
  for( i = 0; i < 60; i++)
    if ( gel[i].next[0] >= 0 )
      gel[i].next[1] = gel[gel[i].next[0]].next[3];  
}


/*
  clears an existing next loop  for the given next index, starting with gel_pos.
  the corresponding 
    int checkNextLoop(int index, int gel_pos)
  must return a positive value with the same arguments
*/
void clearNextLoop(int index, int gel_pos)
{
  int cnt = 0;
  int pos = gel_pos;
  if ( checkNextLoop(index, gel_pos) < 0 )
  {
    //pn("clearNextLoop: No loop found for pos=%d", gel_pos);
    return;
  }
  
  for(;;)
  {
    if ( gel[pos].next[index] < 0 )
    {
      gel[pos].pentagon=-1;
      break;
    }
    if ( gel[pos].next[index] == gel_pos )
    {
      gel[pos].next[index] = -1;
      gel[pos].pentagon = -1;
      break;
    }
    if ( cnt > 60 )
      break ;
    pos = gel[pos].next[index];
    gel[pos].next[index] = -1;
    cnt++;
  }
}

/*================================================*/
/* Learn mode: Pentagons */

void invalidatePentagon(uint16_t pentagon)
{
  uint16_t pos;
  for( pos = 0; pos < 60; pos++ )
  {
    if ( gel[pos].pentagon == pentagon )
    {
      gel[pos].pentagon = -1;
      gel[pos].next[5] = -1;
    }
  }
}


/*
  Calculate the "pentagon" value in ge struct (although it has been calculated before, it is recalculated here)
  This will also validate, whether all pentagons are correct
  If an illegal pentagon is detected, then the pentagon is deleted and needs to be relearned
  This is called after the learning process.
*/
int calculatePentagonNumbers(void)
{
  uint16_t pos, i, ipos, pentagon;
  for( pos = 0; pos < 60; pos++ )
  {
    gel[pos].pentagon = -1;  
  }
  pos = 0;
  for(pentagon = 0; pentagon <12; pentagon++)
  {
    /* search for an unassigned pentagon */
    for( pos = 0; pos < 60; pos++ )
    {
      if ( gel[pos].pentagon == -1 )
        break;
    }
    if ( pos >= 60 )
      return 1; /* all done */

    /* assign the pentagon number */
    p("Pentagon %d: ", pentagon);
    i = 0;
    ipos = pos;
    for(;;)
    {
      p("%d ", ipos);
      gel[ipos].pentagon = pentagon;
      ipos = gel[ipos].next[5];
      if ( ipos < 0 )
      {
        pn("incomplete");
        invalidatePentagon(pentagon);
        return 0;
      }
      i++;
      if ( ipos == pos )
        break;        // "i" should be 5 
      if ( gel[ipos].pentagon < 0 )
      {
        gel[ipos].pentagon = pentagon;
      }
      else
      {
        pn("other pentagon %d hit", gel[ipos].pentagon);
        invalidatePentagon(gel[ipos].pentagon);
        invalidatePentagon(pentagon);
        return 0;
      }
    }
    
    if ( i != 5 )
    {
      invalidatePentagon(pentagon);
      pn("invalid count %d", i);
      return 0;
    }

    pn("");
    /* pentagon is valid, continue */
  } /* for pentagon */
  
  /* this is reached if the outer loop is done */
  return 1;
}

/*
  check whether pentagon information in GEL is correct.
  Used during startup to decide whether to go to pentagon learn mode  
  this will just use next[5] and the pentagon number.
*/
int isGELPentagonCorrect(void)
{
  int pos;
  for( pos = 0; pos < 60; pos++ )
  {
    if ( gel[pos].pentagon < 0 )
      return 0;         /* pentagon number missing: not correct */
    if ( checkNextLoop(5, pos) != 5 )
      return 0;         /* loop is not a pentagon: not correct */
  }
  return 1;     /* pentagon information is correct in GEL */
}


/*
  mark a loop as pentagon loop
    checkNextLoop(5, int gel_pos) must return 5
*/
void markPentagonNextLoop(int gel_pos, uint16_t pentagon_number)
{
  int cnt = 0;
  int pos = gel_pos;
  
  /* check whether this is a pentagon */
  
  if ( checkNextLoop(5, gel_pos) != 5 )
    return;
    
  /* mark this pentagon with the given (unique) number */
    
  for(;;)
  {
    if ( gel[pos].next[5] < 0 )
      break; /* shouln't happen */
    if ( gel[pos].next[5] == gel_pos )
    {
      gel[pos].pentagon = pentagon_number;
      break;
    }
    if ( cnt > 60 )
      break;            /* shouldn't happen */       
    gel[pos].pentagon = pentagon_number;
    pos = gel[pos].next[5];
    cnt++;
  }
  
  /* clear any illegal next references to any member of that loop */
  
  for( pos = 0; pos < 60; pos++)
  {
    /* if the edge has a next and doesn't belong to the current pentagon */
    if ( gel[pos].next[5] > 0 && gel[pos].pentagon != pentagon_number )
    {
      /* however if that edge points to an edge of the current pentagon, then this is an error */
      if ( gel[gel[pos].next[5]].pentagon == pentagon_number )
      {
        gel[pos].next[5] = -1;
      }
    }
  }
}


uint16_t pentagon_mark_number = 0;
void checkAndMarkPentagons(void)
{
  int cnt;
  int pos;
  for( pos = 0; pos < 60; pos++)
  {
    if ( gel[pos].pentagon < 0 )
    {
      cnt  = checkNextLoop(5, pos);
      if ( cnt >= 0 )
      {
        if ( cnt == 5 )
        {
          pn("pentagon loop found at %d: Marked with %d", pos, pentagon_mark_number);
          markPentagonNextLoop(pos, pentagon_mark_number);
          pentagon_mark_number++;
        }
        else
        {
          pn("Pentagon illegal loop (cnt=%d) found at pos=%d: Will be cleared", cnt, pos);
          clearNextLoop(5, pos);
        }
      } /* cnt >= 0 */
    } /* pentagon < 0 */
  } /* for */
}

int learnPentagon()
{
  static int gel_pos = 0; 
  static int state = 0; 
  static uint32_t t = 0;
  static uint32_t wait_time_ms = 8000;
  int i;
  int missing_ge_cnt = 0;
  
  switch(state)
  {
    case 0:
      for( i = 0; i < 60; i++ )
      {
        if ( gel[i].next[5] < 0 )
          missing_ge_cnt++;
      }
      
      for( i = 0; i < 60; i++ )
      {
        if ( gel[i].next[5] < 0 )
        {
          pn("pentagon learn mode: select clockwise next pentagon edge (missing edge cnt=%d)", missing_ge_cnt);
          break;        /* break out of the for loop */
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
      pn("press pentagon next right (clockwise) edge %s", getKeyInfoString(gel[gel_pos].key));
      state = 1;
      break;
    case 1:  // wait for keypress or timeout
      if ( current_key >= 0 )
      {
        int16_t next_gel_pos;
        setRGB(gel[gel_pos].led, 100, 100, 200);
        next_gel_pos = getGELPosByKey(current_key);;
        if ( gel_pos == next_gel_pos )
        {
          // illegal self assignment
          pn("pentagon edge self asignment ignored %s", getKeyInfoString(current_key));
        }
        else if ( gel[next_gel_pos].pentagon > 0 )
        {
          pn("pentagon edge towards existing valid pentagon ignored %s", getKeyInfoString(current_key));
        }
        else
        {
          pn("pentagon edge %d %s 'next' changed to %d", gel_pos, getKeyInfoString(gel[gel_pos].key), next_gel_pos);
          gel[gel_pos].next[5] =  next_gel_pos;
          gel_pos = next_gel_pos;

          checkAndMarkPentagons();
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
      if ( calculatePentagonNumbers() == 0 )
      {
        printGEL();
        pn("pentagon learn mode with errors, redo required (9)");
        state = 0;
        break;
      }
      printGEL();
      pn("pentagon learn mode done (9)");
      state = 0;
      return 1;
  }
  return 0;
}

/*================================================*/
/* Learn mode: Triangles */

void invalidateTriangle(uint16_t triangle)
{
  uint16_t pos;
  for( pos = 0; pos < 60; pos++ )
  {
    if ( gel[pos].triangle == triangle )
    {
      gel[pos].triangle = -1;
      gel[pos].next[2] = -1;
    }
  }
}


/*
  Calculate the "triangle" value in ge struct (although it has been calculated before, it is recalculated here)
  This will also validate, whether all triangles are correct
  If an illegal triangle is detected, then the triangle is deleted and needs to be relearned
  This is called after the learning process.
*/
int calculateTriangleNumbers(void)
{
  uint16_t pos, i, ipos, triangle;
  for( pos = 0; pos < 60; pos++ )
  {
    gel[pos].triangle = -1;  
  }
  pos = 0;
  for(triangle = 0; triangle <20; triangle++)
  {
    /* search for an unassigned triangle */
    for( pos = 0; pos < 60; pos++ )
    {
      if ( gel[pos].triangle == -1 )
        break;
    }
    if ( pos >= 60 )
      return 1; /* all done */

    /* assign the triangle number */
    p("Triangle %d: ", triangle);
    i = 0;
    ipos = pos;
    for(;;)
    {
      p("%d ", ipos);
      gel[ipos].triangle = triangle;
      ipos = gel[ipos].next[2];
      if ( ipos < 0 )
      {
        pn("incomplete");
        invalidateTriangle(triangle);
        return 0;
      }
      i++;
      if ( ipos == pos )
        break;        // "i" should be 3 
      if ( gel[ipos].triangle < 0 )
      {
        gel[ipos].triangle = triangle;
      }
      else
      {
        pn("other triangle %d hit", gel[ipos].triangle);
        invalidateTriangle(gel[ipos].triangle);
        invalidateTriangle(triangle);
        return 0;
      }
    }
    
    if ( i != 3 )
    {
      invalidateTriangle(triangle);
      pn("invalid count %d", i);
      return 0;
    }

    pn("");
    /* triangle is valid, continue */
  } /* for triangle */
  
  /* this is reached if the outer loop is done */
  return 1;
}


/*
  check whether pentagon information in GEL is correct.
  Used during startup to decide whether to go to pentagon learn mode  
  this will just use next[5] and the pentagon number.
*/
int isGELTriangleCorrect(void)
{
  int pos;
  for( pos = 0; pos < 60; pos++ )
  {
    if ( gel[pos].triangle < 0 )
      return 0;         /* triangle number missing: not correct */
    if ( checkNextLoop(2, pos) != 3 )
      return 0;         /* loop is not a triangle: not correct */
  }
  return 1;     /* triangle information is correct in GEL */
}


/*
  mark a loop as triangle loop
    checkNextLoop(2, int gel_pos) must return 3
*/
void markTriangleNextLoop(int gel_pos, uint16_t triangle_number)
{
  int cnt = 0;
  int pos = gel_pos;
  
  /* check whether this is a triangle */
  
  if ( checkNextLoop(2, gel_pos) != 3 )
    return;
    
  /* mark this triangle with the given (unique) number */
    
  for(;;)
  {
    if ( gel[pos].next[2] < 0 )
      break; /* shouln't happen */
    if ( gel[pos].next[2] == gel_pos )
    {
      gel[pos].triangle = triangle_number;
      break;
    }
    if ( cnt > 60 )
      break;            /* shouldn't happen */       
    gel[pos].triangle = triangle_number;
    pos = gel[pos].next[2];
    cnt++;
  }
  
  /* clear any illegal next references to any member of that loop */
  
  for( pos = 0; pos < 60; pos++)
  {
    /* if the edge has a next and doesn't belong to the current triangle */
    if ( gel[pos].next[2] > 0 && gel[pos].triangle != triangle_number )
    {
      /* however if that edge points to an edge of the current triangle, then this is an error */
      if ( gel[gel[pos].next[2]].triangle == triangle_number )
      {
        gel[pos].next[2] = -1;
      }
    }
  }
}

uint16_t triangle_mark_number = 0;
void checkAndMarkTriangles(void)
{
  int cnt;
  int pos;
  for( pos = 0; pos < 60; pos++)
  {
    if ( gel[pos].triangle < 0 )
    {
      cnt  = checkNextLoop(2, pos);
      if ( cnt >= 0 )
      {
        if ( cnt == 3 )
        {
          pn("triangle loop found at %d: Marked with %d", pos, triangle_mark_number);
          markTriangleNextLoop(pos, triangle_mark_number);
          triangle_mark_number++;
        }
        else
        {
          pn("Triangle illegal loop (cnt=%d) found at pos=%d: Will be cleared", cnt, pos);
          clearNextLoop(2, pos);
        }
      } /* cnt >= 0 */
    } /* triangle < 0 */
  } /* for */
}

int learnTriangle()
{
  static int gel_pos = 0; 
  static int state = 0; 
  static uint32_t t = 0;
  static uint32_t wait_time_ms = 8000;
  int i;
  int missing_ge_cnt = 0;
  
  switch(state)
  {
    case 0:
      for( i = 0; i < 60; i++ )
      {
        if ( gel[i].next[2] < 0 )
          missing_ge_cnt++;
      }
      
      for( i = 0; i < 60; i++ )
      {
        if ( gel[i].next[2] < 0 )
        {
          pn("triangle learn mode: select clockwise next triangle edge (missing edge cnt=%d)", missing_ge_cnt);
          break;        /* break out of the for loop */
        }
      }
      
      if ( i >= 60 )
      {
        state = 9;     // pentagon finished
        break;
      }
      
      // find a suitable edge, which could be checked
      while( gel[gel_pos].next[2] >= 0 )
      {
        gel_pos++;
        if ( gel_pos >= 60 )
          gel_pos = 0;
      }
      
      t = millis();
      setRGB(gel[gel_pos].led, 200, 200, 0);  
      pn("press triangle next right (clockwise) edge %s", getKeyInfoString(gel[gel_pos].key));
      state = 1;
      break;
    case 1:  // wait for keypress or timeout
      if ( current_key >= 0 )
      {
        int16_t next_gel_pos;
        setRGB(gel[gel_pos].led, 100, 100, 200);
        next_gel_pos = getGELPosByKey(current_key);
        if ( gel_pos == next_gel_pos )
        {
          // illegal self assignment
          pn("triangle edge self asignment ignored %s", getKeyInfoString(current_key));
        }
        else if ( gel[next_gel_pos].triangle > 0 )
        {
          pn("triangle edge towards existing valid triangle ignored %s", getKeyInfoString(current_key));
        }
        else
        {
          pn("triangle edge %d %s 'next' changed to %d", gel_pos, getKeyInfoString(gel[gel_pos].key), next_gel_pos);
          gel[gel_pos].next[2] =  next_gel_pos;
          gel_pos = next_gel_pos;

          checkAndMarkTriangles();
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
      if ( calculateTriangleNumbers() == 0 )
      {
        printGEL();
        pn("triangle learn mode with errors, redo required (9)");
        state = 0;
        break;
      }
      printGEL();
      pn("triangle learn mode done (9)");
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




/*================================================*/

int masterModeGELShow(void)
{
  static int state = 0;

  switch(state)
  {
    case 0:
      state = 1;
      clearRGBMatrix();
      break;
    case 1:
      if ( current_key >= 0 )
      {
        pn("GEL Show %s", getKeyInfoString(current_key));
        uint16_t pos;
        uint16_t i;
        
        pos = getGELPosByKey(current_key);
        for( i = 0; i < 5; i++ )
        {
          if ( i == 1 )
            setRGB(gel[pos].led, 0,30, 30);  
          else
            setRGB(gel[pos].led, 0, 0, 30);  
          pos = gel[pos].next[0];
        }

        pos = getGELPosByKey(current_key);
        pos = gel[pos].next[3];
        for( i = 1; i < 3; i++ )
        {
          if ( i == 1 )
            setRGB(gel[pos].led, 0,30, 30);  
          else
            setRGB(gel[pos].led, 0, 0, 30);  
          pos = gel[pos].next[3];
        }

        pos = getGELPosByKey(current_key);
        pos = gel[pos].next[4];
        for( i = 1; i < 10; i++ )
        {
          if ( i == 1 )
            setRGB(gel[pos].led, 30,30, 0);  
          else
            setRGB(gel[pos].led, 30, 0, 0);  
          /* looping over the outer ring requires alternating use of next[1] and next[4] */
          if ( (i & 1) != 0 )
            pos = gel[pos].next[1];
          else
            pos = gel[pos].next[4];
        }
        
        //setRGB(gel[gel[pos].next[1]].led, 30, 0, 0);  
        //setRGB(gel[gel[pos].next[4]].led, 0, 30, 0);  

        state = 2;
      }
      break;
    case 2:
      if ( current_key < 0 )
      {
        clearRGBMatrix();
        state = 1;
      }
      break;
  }
  return 0;
}


/*================================================*/
/* hardware self test */

int selfTest(void)
{
  int i, j;
  int is_error = 0;
  char keyname[16];
  for( i = 0; i < TOUCH_KEY_CNT; i++ )
  {
    strcpy(keyname, getGPIONameByKey(i));

    pinMode(touch_status_list[i].arduino_pin, OUTPUT);
    
    digitalWrite(touch_status_list[i].arduino_pin, 0);
    for( j = 0; j < TOUCH_KEY_CNT; j++ )
    {   
      if ( j != i ) pinMode(touch_status_list[i].arduino_pin, INPUT_PULLUP);
    }
    for( j = 0; j < TOUCH_KEY_CNT; j++ )
    {   
      if ( j != i ) 
        if ( digitalRead(touch_status_list[i].arduino_pin) == 0 )
          pn("short circuit between key %d (%s) and %d (%s) found", i, keyname, j, getGPIONameByKey(j)), is_error=1;
    }
    
    digitalWrite(touch_status_list[i].arduino_pin, 1);
    for( j = 0; j < TOUCH_KEY_CNT; j++ )
    {   
      if ( j != i ) pinMode(touch_status_list[i].arduino_pin, INPUT_PULLDOWN);
    }
    for( j = 0; j < TOUCH_KEY_CNT; j++ )
    {   
      if ( j != i ) 
        if ( digitalRead(touch_status_list[i].arduino_pin) != 0 )
          pn("short circuit between key %d (%s) and %d (%s) found", i, keyname, j, getGPIONameByKey(j)), is_error=1;
    }
  }
  return is_error;
}

/*================================================*/
/* setup() and loop() */

#define MASTER_MODE_NONE 0
#define MASTER_MODE_LEARN_KEY_LED_MAP 1
#define MASTER_MODE_LEARN_PENTAGON 2
#define MASTER_MODE_LEARN_TRIANGLE 3
#define MASTER_MODE_GEL_SHOW 4

uint8_t master_mode = MASTER_MODE_NONE;

// the setup function runs once when you press reset or power the board

void setup(void) 
{
  //Serial.begin(9600);
  Serial.begin(115200);
  
  delay(1200);
  
  pn("buildTouchMeasureList");
  buildTouchMeasureList();
  
  pn("initLEDMatrix");
  initLEDMatrix();

  pn("self test");
  selfTest();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PA1, OUTPUT);
  for( int i = 0; i < TOUCH_KEY_CNT; i++ )
  {
    pinMode(touch_status_list[i].arduino_pin, OUTPUT);
  }

  fixKeyToLEDMap();

  pn("Setup done");

  if (  getAssignedKeyCount() < 60 )
  {
    master_mode = MASTER_MODE_LEARN_KEY_LED_MAP;
    pn("Key LED mapping mode started");
  }
  else if ( isGELPentagonCorrect() == 0 )
  {
    clearGEL();
    pn("Pentagon learn mode started");
    master_mode = MASTER_MODE_LEARN_PENTAGON;
  }
  else if ( isGELTriangleCorrect() == 0 )
  {
    pn("Triangle learn mode started");
    master_mode = MASTER_MODE_LEARN_TRIANGLE;
  }
  else
  {
    pn("GEL show mode started");
    master_mode = MASTER_MODE_GEL_SHOW;
    //pn("Default mode started");
  }

  calculateOtherNextGELValues();

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
        master_mode = MASTER_MODE_LEARN_PENTAGON;
      break;
    case MASTER_MODE_LEARN_PENTAGON:
      if ( learnPentagon() != 0 )
        master_mode = MASTER_MODE_LEARN_TRIANGLE;
      break;    
    case MASTER_MODE_LEARN_TRIANGLE:
      if ( learnTriangle() != 0 )
      {
        calculateOtherNextGELValues();
        master_mode = MASTER_MODE_NONE;
      }
      break;    
    case MASTER_MODE_GEL_SHOW:
      if ( masterModeGELShow() != 0 )
      {
        master_mode = MASTER_MODE_NONE;
      }
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
