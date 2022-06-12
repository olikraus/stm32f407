/*
  spi_ws2812b.ino
  
*/

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
    /*
    if ( b & 8 )
      d |= 0x2000;
    if ( b & 4 )
      d |= 0x0200;
    if ( b & 2 )
      d |= 0x0020;
    if ( b & 1 )
      d |= 0x0002;
    b >>= 4;
    */
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
      /* ensure, that the SCK goes to low after the byte transfer: */
      /* Set the EOT flag at the end of the transfer */
      
      /* not used */
      
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
  while( ws2812_spi.in_progress != 0 )
    ;


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

/*=======================================================================*/

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

/*=======================================================================*/


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PA1, OUTPUT);
  Serial.begin(9600);
  /*
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  */
  ws2812_spi_init();
}

/* format is GRB */


uint8_t a[64*3];

void setColByPos(uint8_t pos, uint8_t r, uint8_t g, uint8_t b)
{
  a[pos*3] = g;
  a[pos*3+1] = r;
  a[pos*3+2] = b;
}

void addColByPos(uint8_t pos, uint8_t r, uint8_t g, uint8_t b)
{
  a[pos*3] += g;
  a[pos*3+1] += r;
  a[pos*3+2] += b;
}

void setColByXY(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b )
{
  uint8_t pos = y*8;
  if ( y & 1 )
  {
    pos += 7-x;
  }
  else
  {
    pos += x;
  }
  setColByPos(pos, r, g, b);
}

void addColByXY(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b )
{
  uint8_t pos = y*8;
  if ( y & 1 )
  {
    pos += 7-x;
  }
  else
  {
    pos += x;
  }
  addColByPos(pos, r, g, b);
}

void clrAll(void)
{
  for( uint8_t i; i < 3*64; i++ )
    a[i] = 0;
}

uint8_t v = 0;
int8_t v1 = 0;
int8_t v2 = 0;

/*
 * radius=100 small
 */
void addBallByXY(uint8_t cx, uint8_t cy, uint8_t radius, uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t x, y;
  uint16_t dx, dy;
  uint16_t dint;
  for( y = 0; y < 8; y++ )
  {
    for( x = 0; x < 8; x++ )
    {
      dx = cx-x;
      dy = cy-y;
      dint = dx*dx+dy*dy;
      dint = dint*radius+64;
      addColByXY(x, y, (r*64)/dint, (g*64)/dint, (b*64)/dint );
    }
  }
}

void loop() {
  uint8_t r,g,b;
  Serial.print("> ");
  digitalWrite(PA1, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(10);                       // wait for a second
  digitalWrite(PA1, LOW);    // turn the LED off by making the voltage LOW
  delay(10);                       // wait for a second
  Serial.print(ws2812_spi.isr_cnt, DEC);

  Serial.print(" SR=");
  Serial.print(ws2812_spi.spi_status_register, HEX);
  
  Serial.println("");
  v++;
    
  v1++;
  if ( v1 >= 10 )
    v1 = -10;
    
  v2++;
  if ( v2 >= 7 )
    v2 = -7;
    
  clrAll();
  /*
  for( uint8_t y = 0; y < 8; y++ )
    for( uint8_t x = 0; x < 8; x++ )
    {
      hsv_to_rgb(y*16+v, 200, x/2+3, &r, &g, &b);
      setColByXY(x, y, r, g, b);
         
    }
  */
  hsv_to_rgb(v, 255, 100, &r, &g, &b);
  setColByXY(3, 3, r, g, b);
  ws2812_spi_out(a, 3*64);
}
