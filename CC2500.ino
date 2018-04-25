#include <krnl.h>
#include <SPI.h>

// Carrier frequency
#define FREQ 2408000000
// Ping packet generation interval
#define PING_INTERVAL_MS 1000
// Device address. Same address is used by
// both devices.
#define DEVICE_ADDR 0xBA
// Length of the rx/tx buffer. A buffer is maintained
// by each of the tasks.
#define RXTX_BUFLEN 128
// Size of the stack available to each task. Must
// be at least RXTX_BUFLEN.
#define STACK_SIZE (RXTX_BUFLEN + 128)
// Chip select for device 1
#define DEV1_CS 12
// GDO0 pin for device 1
#define DEV1_GDO0_PIN 7
// Chip select for device 2
#define DEV2_CS 11
// GDO0 pin for device 2
#define DEV2_GDO0_PIN 3

// Mutexes for shared resources
struct k_t *spi_mtx, *serial_mtx;
// SPI
SPISettings spi_settings(2000000, MSBFIRST, SPI_MODE0);

/*******************************
 * Convenient CC2500 functions *
 *******************************/
uint8_t cc2500_send_strobe(uint8_t addr, int cs_pin) {
  uint8_t status;
  k_wait(spi_mtx, 0);
  SPI.beginTransaction(spi_settings);
  digitalWrite(cs_pin, LOW);
  status = SPI.transfer(addr);
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();
  k_signal(spi_mtx);
  return status;
}

void cc2500_write_reg(uint8_t addr, uint8_t value, int cs_pin) {
  k_wait(spi_mtx, 0);
  SPI.beginTransaction(spi_settings);
  digitalWrite(cs_pin, LOW);
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();
  k_signal(spi_mtx);
}

void cc2500_burst_write_reg(uint8_t addr, uint8_t *buf, uint8_t len, int cs_pin) {
  k_wait(spi_mtx, 0);
  SPI.beginTransaction(spi_settings);
  digitalWrite(cs_pin, LOW);
  SPI.transfer((1<<6) | addr);
  for(int i = 0; i < len; i++) {
    SPI.transfer(*buf);
    buf++;
  }
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();
  k_signal(spi_mtx);
}

uint8_t cc2500_read_reg(uint8_t addr, int cs_pin) {
  uint8_t value;
  k_wait(spi_mtx, 0);
  SPI.beginTransaction(spi_settings);
  digitalWrite(cs_pin, LOW);
  SPI.transfer((1<<7) | addr);
  value = SPI.transfer(0);
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();
  k_signal(spi_mtx);
  return value;
}

void cc2500_burst_read_reg(uint8_t addr, uint8_t *buf, uint8_t len, int cs_pin) {
  k_wait(spi_mtx, 0);
  SPI.beginTransaction(spi_settings);
  digitalWrite(cs_pin, LOW);
  SPI.transfer((1<<7) | (1<<6) | addr);
  for(int i = 0; i < len; i++) {
    *buf = SPI.transfer(0);
    buf++;
  }
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();
  k_signal(spi_mtx);
}

/***********************
 * Task implementation *
 ***********************/

// Thread safe serial print
void print(int cs_pin, char *str) {
  k_wait(serial_mtx, 0);
  Serial.print(cs_pin);
  Serial.print(": ");
  Serial.println(str);
  k_signal(serial_mtx);
}


// This state does not directly reflect the CC2500 state
enum prog_state {
  STATE_UNDEF = -1,
  STATE_IDLE = 0,
  STATE_RX = 1
};

// This function is run by both CC2500 tasks
void task(int cs_pin, int gdo0_pin, void (*gdo0_interrupt_fun)(void), volatile uint8_t *interrupt_flag) {
  uint8_t buf[RXTX_BUFLEN];
  enum prog_state state;
  // Setup pins
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  pinMode(gdo0_pin, INPUT);

  // Initialize CS2500
  cc2500_initialize(cs_pin);
  
  // Attach interrupt
  *interrupt_flag = 0;
  attachInterrupt(digitalPinToInterrupt(gdo0_pin), gdo0_interrupt_fun, FALLING);

  // Default state
  state = STATE_IDLE;

  unsigned long last_tx = 0;
  while(1) {
    // Check if it is time for PING TX
    if(state == STATE_IDLE || state == STATE_RX) {
      if(millis()-last_tx > PING_INTERVAL_MS) {
        // Transmit!
        print(cs_pin, "TX PING");
        strcpy((char*)&buf[2], "PING");
        // Write length field: String length + '\0' + address field + 40 tail bytes
        buf[0] = 5+1+40;
        buf[1] = DEVICE_ADDR;
        // Add 40 tail bytes to make it look better on a spectrum
        for(int i = 0; i < 40; i++) {
          buf[7+i] = 0xAA;
        }
        tx_pkt(buf, buf[0]+1, cs_pin, interrupt_flag);
        // CS2500 enters IDLE state after transmission
        state = STATE_IDLE;
        last_tx = millis();
      }
    }
  
    switch(state) {
    case STATE_IDLE:
      // Enter RX mode
      cc2500_send_strobe(0x34, cs_pin);
      state = STATE_RX;
      break;
    case STATE_RX:
      if(*interrupt_flag) {
        // Packet received (but may be corrupt)
        *interrupt_flag = 0;
        handle_rx_pkt(buf, cs_pin, interrupt_flag);
        // CC2500 transitions into IDLE
        state = STATE_IDLE;
      }
      break;
    }
  }
}

void cc2500_initialize(int cs_pin) {
  // Reset
  cc2500_send_strobe(0x30, cs_pin);
  // Wait until CC2500 is ready
  while(cc2500_read_reg(0x3D, cs_pin) & (1<<7)) {
    delayMicroseconds(100);
  }
  // Let GDO0 go HIGH on pkt sent/received
  cc2500_write_reg(0x02, 0x06, cs_pin);
  
  // Enable auto calibration from IDLE -> RX/TX
  cc2500_write_reg(0x18, 0b00010100, cs_pin);

  // Set carrier frequency to FREQ
  //FREQ = (F_clk/2^16)*val
  // => val = (2^16/F_clk)*FREQ = (2^16/(26*10^6))*FREQ
  uint32_t val = (((uint64_t)FREQ) << 16)/26000000;
  // Store in array as big endian
  uint8_t val_arr[3] = {
    ((val>>16) & 0xFF),
    ((val>>8) & 0xFF),
    ((val) & 0xFF)
  };
  cc2500_burst_write_reg(0x0D, val_arr, 3, cs_pin);
 
  // Enable autoflush and address check
  cc2500_write_reg(0x07, 0x0D, cs_pin);
  // Set address
  cc2500_write_reg(0x09, DEVICE_ADDR, cs_pin);
  // Set 24 preamble bytes - too many, but make the spectrum look nice.
  cc2500_write_reg(0x13, 0x72, cs_pin);
  
  // Flush rx and tx registers
  cc2500_send_strobe(0x3A, cs_pin);
  cc2500_send_strobe(0x3B, cs_pin);
}

void handle_rx_pkt(uint8_t *buf, int cs_pin, volatile uint8_t *interrupt_flag) {
  // Read RXBYTES
  uint8_t rxbytes = cc2500_read_reg(0xFB, cs_pin);
  if(rxbytes & (1<<7)) {
    // Overflow
    print(cs_pin, "==>>> OVERFLOW <<<==");
    // Flush buffer
    cc2500_send_strobe(0x3A, cs_pin);
  } else if(rxbytes > 0) {
    // Received packet
    // Read RXFIFO
    cc2500_burst_read_reg(0x3F, buf, rxbytes, cs_pin);
    // Flush buffer
    cc2500_send_strobe(0x3A, cs_pin);
    // Read RSSI
    uint8_t rssi = cc2500_read_reg(0xF4, cs_pin);
    // Read LQI
    uint8_t lqi = cc2500_read_reg(0xF3, cs_pin);
    if(strncmp("PING", (char*)&buf[2], RXTX_BUFLEN) == 0) {
      print(cs_pin, "RX PING - TX PONG");
      
      // Reply. Same as PING.
      strcpy((char*)&buf[2], "PONG");
      buf[0] = 5+1+40;
      buf[1] = DEVICE_ADDR;
      for(int i = 0; i < 40; i++) {
        buf[7+i] = 0xAA;
      }
      tx_pkt(buf, buf[0]+1, cs_pin, interrupt_flag);
      
    } else if(strncmp("PONG", (char*)&buf[2], RXTX_BUFLEN) == 0) {
      // Received PONG
      print(cs_pin, "RX PONG");
    } else {
      // Unknown packet addressed to us(?)
      print(cs_pin, "RX UNKOWN");
    }
  }
}

void tx_pkt(uint8_t *buf, uint8_t len, int cs_pin, volatile uint8_t *interrupt_flag) {
  // Go to idle state
  cc2500_send_strobe(0x36, cs_pin);

  // Flush TX buffer
  cc2500_send_strobe(0x3B, cs_pin);

  // Transfer packet to FIFO
  cc2500_burst_write_reg(0x3F, buf, len, cs_pin);
  
  // Reset interrupt flag and transmit
  *interrupt_flag = 0;
  cc2500_send_strobe(0x35, cs_pin);
  
  // Wait until transmitted
  while(*interrupt_flag == 0);
  *interrupt_flag = 0;
}


/*************
 * GDO0 ISRs *
 *************/

volatile uint8_t gdo0_1_flag = 0;
void gdo0_1_interrupt() {
  gdo0_1_flag = 1;
}
volatile uint8_t gdo0_2_flag = 0;
void gdo0_2_interrupt() {
  gdo0_2_flag = 1;
}

/*****************
 * Task starters *
 *****************/

void task1(void) {
  // Provide arguments for device 1
  task(DEV1_CS, DEV1_GDO0_PIN, gdo0_1_interrupt, &gdo0_1_flag);
}
void task2(void) {
  // Start delayed
  delay(500);
  // Provide arguments for device 2
  task(DEV2_CS, DEV2_GDO0_PIN, gdo0_2_interrupt, &gdo0_2_flag);
}

/******************
 * Initialization *
 ******************/

// Allocate task stacks
char stack1[STACK_SIZE];
char stack2[STACK_SIZE];

void setup() {
  Serial.begin(9600);
  SPI.begin();

  // Initialize SPI CSs
  pinMode(DEV1_CS, OUTPUT);
  digitalWrite(DEV1_CS, HIGH);
  pinMode(DEV2_CS, OUTPUT);
  digitalWrite(DEV2_CS, HIGH);

  // LED
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Blink to indicate that we are starting
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);

  // Not needed, but makes it easier to attach serial terminal
  delay(5000);
  
  Serial.println("STARTING");

  // Initialize krnl
  k_init(2, 2, 0);

  // Create tasks and mutexes
  k_crt_task(task1, 1, stack1, STACK_SIZE);
  k_crt_task(task2, 1, stack2, STACK_SIZE);
  spi_mtx = k_crt_sem(1,1);
  serial_mtx = k_crt_sem(1,1);
  // Start krnl
  k_start(10); // 10ms tick
  
  Serial.println("Kernel failure");
}

void loop() {} // Not used

