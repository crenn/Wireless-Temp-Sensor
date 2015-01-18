// Sketch is so a Leaflab Maple or Similiar Compatible Board.
// This sketch gets temperature values and sends it back every 5 seconds
// nRF24L01+ Pins (Maple Mini)
HardwareSPI spi(1);
#define MOSI 4
#define MISO 5
#define SCK 6
#define CS_PIN 7
#define CE_PIN 3
#define IRQ_PIN 10

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
  EnableRF(); //Waiting for a message
}

void ModeRX(void) {
  StandbyRF();
  WriteRegister(CONFIG, 0x0B);
}

void ModeTX(void) {
  StandbyRF();
  WriteRegister(CONFIG, 0x0A);
}

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

void requestCalibration(void) {
  byte status;
  SerialUSB.println("Requesting Calibration values");
  digitalWrite(CS_PIN, LOW);
  spi.send(W_TX_PAYLOAD);
  spi.send('R');
  spi.send('C');
  digitalWrite(CS_PIN, HIGH);
  ModeTX();
  EnableRF();
  while (digitalRead(IRQ_PIN) == HIGH);
  status = GetStatus();
  if (status&0x10) {
    WriteRegister(STATUS, 0x10);
    digitalWrite(CS_PIN, LOW);
    spi.send(FLUSH_TX);
    digitalWrite(CS_PIN, HIGH);
    SerialUSB.println("Oh darn... Calibration request not sent");
  }
  if (status&0x20) {
    SerialUSB.println("Sent the Calibration request!");
    WriteRegister(STATUS, 0x20);
  }
}

void requestTemps(void) {
  byte status;
  SerialUSB.println("Requesting Temperature values");
  digitalWrite(CS_PIN, LOW);
  spi.send(W_TX_PAYLOAD);
  spi.send('R');
  spi.send('T');
  digitalWrite(CS_PIN, HIGH);
  ModeTX();
  EnableRF();
  while (digitalRead(IRQ_PIN) == HIGH);
  status = GetStatus();
  if (status&0x10) {
    WriteRegister(STATUS, 0x10);
    digitalWrite(CS_PIN, LOW);
    spi.send(FLUSH_TX);
    digitalWrite(CS_PIN, HIGH);
    SerialUSB.println("Oh darn... Temperature request not sent");
  }
  if (status&0x20) {
    SerialUSB.println("Sent the Temperature request!");
    WriteRegister(STATUS, 0x20);
  }
}

void setup(void) {
  initnRF24();
  // Initialize the built-in LED pin as an output:
  pinMode(BOARD_LED_PIN, OUTPUT);
  // Initialize the built-in button (labeled BUT) as an input:
  pinMode(BOARD_BUTTON_PIN, INPUT);
}

byte calibration = 0;
unsigned short AC5=0;
unsigned short AC6=0;
short MC=0;
short MD=0;

byte awake = 0;
unsigned long tempradio = 0;
unsigned long tempmessage = 0;

void loop(void) {
  byte status;
  byte pipe;
  byte count;
  byte buffer[32];
  byte i;
  byte samples;
  long UT;
  long X1;
  long X2;
  long B5;
  long Temp;
  float ActTemp;
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
          if ((buffer[0] == 'A') && (buffer[1] == 'W')) {
            awake = 1;
            SerialUSB.println("Remote Sensor is awake!");
            tempradio = millis();
            tempmessage = tempradio - 250;
          }
        } else {
          if (buffer[0] == 'C') {
            AC5 = ((buffer[1] << 8) | buffer[2]);
            AC6 = ((buffer[3] << 8) | buffer[4]);
            MC = ((buffer[5] << 8) | buffer[6]);
            MD = ((buffer[7] << 8) | buffer[8]);
            calibration = 1;
          } else if (buffer[0] == 'T') {
            SerialUSB.println("Got Temps!");
            samples = (count - 1) >> 1;
            for (i = 0; i < samples; i++) {
              UT = ((buffer[(i*2)+1] << 8) | buffer[(i*2)+2]);
              X1 = ((UT - AC6) * AC5) >> 15;
              X2 = (MC << 11) / (X1 + MD);
              B5 = X1 + X2;
              Temp = (B5 + 8) >> 4;
              ActTemp = (float)Temp / 10;
              SerialUSB.print("True Temp is : ");
              SerialUSB.print(ActTemp);
              SerialUSB.println("oC");
            }
            awake = 0;
          }
        }
      }
    }
  }
  if (awake == 1) {
    if ((millis() - tempradio) > 5000) {
      SerialUSB.println("It's not likely to be awake now");
      awake = 0;
    } else {
      if ((millis() - tempmessage) > 500) {
        if (calibration == 0) {
          requestCalibration();
        } else {
          requestTemps();
        }
        ModeRX();
        EnableRF();
        tempmessage = millis();
      }
    }
  }
}
