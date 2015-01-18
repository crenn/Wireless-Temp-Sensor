// Sketch is for a Leaflab Maple or Similiar Compatible Board.
// This sketch gets temperature values and sends it back every 60 seconds
// Author - Trystan -Crenn- Jones
// Twitter - @crennsmind
// Dependancies - RTCClock library from https://github.com/bubulindo/MapleRTC
#include <RTClock.h>
#include <HardWire.h>
#include <scb.h>

//Switch to 1 if you do not want the main microcontroller to sleep and enable debug messages over USB
#define DEBUG 0

//RTC Variables and Function
RTClock rtc(RTCSEL_LSI);
volatile boolean startRX;
volatile boolean takeSample;
volatile unsigned long rtctime;

void eventtrigger(void) {
  static unsigned long lastsample = 0;
  static unsigned long lastradio = 0;
  unsigned long currtime;
  unsigned long timepassed;
  unsigned long nextevent;
  unsigned long timetilsample = 0;
  unsigned long timetilradio = 0;
  currtime = rtc.getTime();
  if (DEBUG) {
    SerialUSB.print("AET: ");
    SerialUSB.println(currtime);
  }
  timepassed = currtime - rtctime;
  lastsample += timepassed;
  lastradio += timepassed;
  if (lastsample >= COLLECT_SAM_S) {
    takeSample = true;
    lastsample -= COLLECT_SAM_S;
  }
  if (lastradio >= WAKEUP_nRF_S) {
    startRX = true;
    lastradio -= WAKEUP_nRF_S;
  }
  if (COLLECT_SAM_S >= lastsample)
    timetilsample = COLLECT_SAM_S - lastsample;
  if (timetilsample == 0) {
    timetilsample++;
  }
  if (WAKEUP_nRF_S >= lastradio)
    timetilradio = WAKEUP_nRF_S - lastradio;
  if (timetilradio == 0) {
    timetilradio++;
  }
  nextevent = (timetilsample <= timetilradio) ? timetilsample : timetilradio;
  if (DEBUG) {
    if ((timetilsample > 100) || (timetilradio > 100)) {
      SerialUSB.print("FUD: ");
      SerialUSB.print(timepassed);
      SerialUSB.print(", ");
      SerialUSB.print(nextevent);
      SerialUSB.print(", ");
      SerialUSB.print(lastsample);
      SerialUSB.print(", ");
      SerialUSB.print(timetilsample);
      SerialUSB.print(", ");
      SerialUSB.print(lastradio);
      SerialUSB.print(", ");
      SerialUSB.println(timetilradio);
    }
  }
  nextevent += currtime;
  if (nextevent == currtime) {
    nextevent++;
  }
  if (rtc.getTime() >= nextevent) {
    nextevent = rtc.getTime() + 1;
  }
  rtctime = currtime;
  rtc.setAlarmTime(nextevent);
  if (DEBUG) {
    SerialUSB.print("NEA: ");
    SerialUSB.println(nextevent);
  }
}

// Program variables and constants
// Timing in ms
#define COLLECT_SAM  10000
#define WAKEUP_nRF   60000
#define TIMEOUT_nRF  60000

// Timing is whole seconds
#define COLLECT_SAM_S (COLLECT_SAM/1000)
#define WAKEUP_nRF_S  (WAKEUP_nRF/1000)
#define TIMEOUT_nRF_S (TIMEOUT_nRF/1000)

// Temp
#define TEMP_SAMPLES (WAKEUP_nRF/COLLECT_SAM)
#define TEMP_BYTES   (TEMP_SAMPLES*2)
byte tempsamp[TEMP_BYTES];
byte samples;

//nRF24L01+ Constants, Variables and Functions
//nRF24L01+ Pins (Maple)
HardwareSPI spi(2);
#define MOSI 34
#define MISO 33
#define SCK 32
#define CS_PIN 31
#define CE_PIN 27
#define IRQ_PIN 28

// nRF24L01+ Commands
#define R_REGISTER           0x00  // Last 5 bits are Register Address
#define W_REGISTER           0x20  // Last 5 bits are Register Address
#define R_RX_PAYLOAD         0x61
#define W_TX_PAYLOAD         0xA0
#define FLUSH_TX             0xE1
#define FLUSH_RX             0xE2
#define REUSE_TX_PL          0xE3
#define R_RX_PL_WID          0x60
#define W_ACK_PAYLOAD        0xA8 // Last 3 bits are for Pipe number
#define W_TX_PAYLOAD_NOACK   0xB0
#define NOP                  0xFF

// nRF24L01+ Registers
#define CONFIG      0x00
#define EN_AA       0x01 //Enhanced ShockBurst
#define EN_RX_ADDR  0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

//nRF24L01+ Variables and Access functions
byte awake = 0;

void WriteRegister(byte reg, byte data) {
  digitalWrite(CS_PIN, LOW);
  spi.send(W_REGISTER|(reg&0x1F));
  spi.send(data);
  digitalWrite(CS_PIN, HIGH);
}

byte ReadRegister(byte reg) {
  byte temp;
  digitalWrite(CS_PIN, LOW);
  spi.send(R_REGISTER|(reg&0x1F));
  temp = spi.send(0x00);
  digitalWrite(CS_PIN, HIGH);
  return temp;
}

byte GetStatus(void) {
  byte temp;
  digitalWrite(CS_PIN, LOW);
  temp = spi.send(NOP);
  digitalWrite(CS_PIN, HIGH);
  return temp;
}

void initnRF24(void) {
  spi.begin(SPI_1_125MHZ, MSBFIRST, 0);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(CE_PIN, OUTPUT);
  digitalWrite(CE_PIN, LOW);
  pinMode(MISO, INPUT_PULLDOWN);
  pinMode(IRQ_PIN, INPUT_PULLUP);
  delay(150);
  WriteRegister(FEATURE, 0x06);
  WriteRegister(DYNPD, 0x03);
  WriteRegister(SETUP_RETR, 0x33);
  ModeRX();
  delay(100);
}

//Power Modes and easy use functions
void EnableRF(void) {
  digitalWrite(CE_PIN, HIGH);
}

void StandbyRF(void) {
  digitalWrite(CE_PIN, LOW); //Return to Idle
}

void SleepRF(void) {
  StandbyRF();
  WriteRegister(CONFIG, 0x09);
}

void ModeRX(void) {
  StandbyRF();
  WriteRegister(CONFIG, 0x0B);
}

void ModeTX(void) {
  StandbyRF();
  WriteRegister(CONFIG, 0x0A);
}

//nRF24L01+ Messages
void sayAwake(void) {
  digitalWrite(CS_PIN, LOW);
  spi.send(W_TX_PAYLOAD);
  spi.send('A');
  spi.send('W');
  digitalWrite(CS_PIN, HIGH);
}

void replyCalibration(void) {
  byte blah[8];
  byte i;
  getBMPValue(&blah[0], 4, 0xB2);
  getBMPValue(&blah[4], 4, 0xBC);
  digitalWrite(CS_PIN, LOW);
  spi.send(W_TX_PAYLOAD);
  spi.send('C');
  for(i = 0; i < 8; i++) {
    spi.send((char)blah[i]);
  }
  digitalWrite(CS_PIN, HIGH);
}

void replyTemps(void) {
  byte i;
  byte len = samples * 2;
  digitalWrite(CS_PIN, LOW);
  spi.send(W_TX_PAYLOAD);
  spi.send('T');
  for(i = 0; i < len; i++) {
    spi.send((char)tempsamp[i]);
  }
  digitalWrite(CS_PIN, HIGH);
  samples = 0;
}

//BMP085 Pressure Sensor - Used as a temporary measure until I have a breakout board for an SI7021 humidity and temperature sensor
HardWire Wire(1);
#define ADDR 0x77
#define ADDR_R 0x01

void getBMPValue(byte array[], byte len, byte startaddr) {
  int i=0;
  Wire.beginTransmission(ADDR);
  Wire.send(startaddr);
  Wire.endTransmission();
  Wire.requestFrom(ADDR, len);
  do {
    array[i]=Wire.receive();
  } while (i++ < len);
}

void getSample(void) {
  if (samples < TEMP_SAMPLES) {
    Wire.beginTransmission(ADDR);
    Wire.send(0xF4);
    Wire.send(0x2E);
    Wire.endTransmission();
    delay(10);
    getBMPValue(&tempsamp[samples*2], 2, 0xF6);
    samples++;
  }
}

void setup(void) {
  unsigned long whee;
  initnRF24();
  startRX = false;
  takeSample = false;
  // Initialize the built-in LED pin as an output:
  pinMode(BOARD_LED_PIN, OUTPUT);
  // Initialize the built-in button (labeled BUT) as an input:
  pinMode(BOARD_BUTTON_PIN, INPUT);
  digitalWrite(BOARD_LED_PIN, HIGH);
  delay(5000);
  rtctime = rtc.getTime();
  if (DEBUG) {
    SerialUSB.print("time is: ");
    SerialUSB.println(rtctime);
  }
  whee = rtctime+COLLECT_SAM_S;
  rtc.createAlarm(eventtrigger, whee);
  if (DEBUG) {
    SerialUSB.print("next alarm is: ");
    SerialUSB.println(whee);
    SerialUSB.print("curr time is now: ");
    SerialUSB.println(rtc.getTime());
  }
}

unsigned long tempsample = 0;
unsigned long tempradio = 0;

void loop(void) {
  byte status;
  byte pipe;
  byte count;
  byte buffer[32];
  byte i;
  unsigned long time = millis();
  digitalWrite(BOARD_LED_PIN, HIGH);
  if (takeSample) {
    tempsample = time;
    getSample();
    takeSample = false;
  }
  if (awake == 0) {
    if (startRX) {
      if (DEBUG) {
        SerialUSB.println("Saying I've woken up");
      }
      sayAwake();
      ModeTX();
      EnableRF();
      while (digitalRead(IRQ_PIN) == HIGH);
      status = GetStatus();
      if (status&0x10) {
        WriteRegister(STATUS, 0x10);
        digitalWrite(CS_PIN, LOW);
        spi.send(FLUSH_TX);
        digitalWrite(CS_PIN, HIGH);
        SleepRF();
        if (DEBUG) {
          SerialUSB.println("Oh darn... Going back to sleep");
        }
      }
      if (status&0x20) {
        WriteRegister(STATUS, 0x20);
        ModeRX();
        EnableRF();
        awake = 1;
        if (DEBUG) {
          SerialUSB.println("Sent Message! Got an ACK! Staying Awake and waiting");
        }
      }
      tempradio = time;
      startRX = false;
    }
  } else if (awake == 1) {
    if (digitalRead(IRQ_PIN) == LOW) {
      status = GetStatus();
      if (status&0x40) {
        WriteRegister(STATUS, 0x40);
        pipe = ((status&0x0E)>>1);
        if (pipe == 0) {
          digitalWrite(CS_PIN, LOW);
          spi.send(R_RX_PL_WID);
          count = spi.send(0x00);
          digitalWrite(CS_PIN, HIGH);
          if (count > 0) {
            digitalWrite(CS_PIN, LOW);
            spi.send(R_RX_PAYLOAD);
            for(i=0; i < count; i++) {
              buffer[i] = (char)spi.send(0x00);
            }
            digitalWrite(CS_PIN, HIGH);
          }
          if (count == 2) {
            if ((buffer[0] == 'R') && (buffer[1] == 'C')) {
              if (DEBUG) {
                SerialUSB.println("Got a request for Calibration");
              }
              replyCalibration();
              ModeTX();
              EnableRF();
              while (digitalRead(IRQ_PIN) == HIGH);
              status = GetStatus();
              if (status&0x10) {
                WriteRegister(STATUS, 0x10);
                digitalWrite(CS_PIN, LOW);
                spi.send(FLUSH_TX);
                digitalWrite(CS_PIN, HIGH);
                if (DEBUG) {
                  SerialUSB.println("Oh darn... Calibration values not sent");
                }
              }
              if (status&0x20) {
                if (DEBUG) {
                  SerialUSB.println("Sent the Calibration values!");
                }
                WriteRegister(STATUS, 0x20);
              }
            } else if ((buffer[0] == 'R') && (buffer[1] == 'T')) {
              if (DEBUG) {
                SerialUSB.println("Got a request for Temperature");
              }
              replyTemps();
              ModeTX();
              EnableRF();
              while (digitalRead(IRQ_PIN) == HIGH);
              status = GetStatus();
              if (status&0x10) {
                WriteRegister(STATUS, 0x10);
                digitalWrite(CS_PIN, LOW);
                spi.send(FLUSH_TX);
                digitalWrite(CS_PIN, HIGH);
                if (DEBUG) {
                  SerialUSB.println("Oh darn... Temperatures values not sent");
                }
              }
              if (status&0x20) {
                if (DEBUG) {
                  SerialUSB.println("Sent the Temperature values! Going back to sleep.");
                }
                WriteRegister(STATUS, 0x20);
                tempradio = time;
                SleepRF();
                awake = 0;
              }
            }
            if (awake == 1) {
              ModeRX();
              EnableRF();
            }
          }
        }
      }
    }
    if ((time - tempradio) >= TIMEOUT_nRF) {
      if (DEBUG) {
        SerialUSB.println("Stayed awake for the limit, going back to sleep.");
      }
      SleepRF();
      awake = 0;
      tempradio = time;
    }
  }
  if (!DEBUG) {
    if ((awake == 0) && (!takeSample) && (!startRX)) {
      digitalWrite(BOARD_LED_PIN, LOW);
      //Set power mode for stop mode and low power on the internal regulator
      PWR_BASE->CR &= ~(1 << PWR_CR_PDDS);
      PWR_BASE->CR |= (1 << PWR_CR_LPDS);
      SCB_BASE->SCR &= 0x00000006;
      SCB_BASE->SCR |= 0x00000004;
      //Wake on interrupt (RTC alarm)
      asm ("WFI");
    }
  }
}
