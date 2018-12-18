#include <EEPROM.h>

#include <mcp_can.h>
#include <SPI.h>

#include <math.h>
#include "LED.h"
#include "ALS31300.h"

#define BITS_PER_BYTE 8
#define SERIAL_NUM 160

unsigned long last_update;
int device_address = 0x60;
const int UPDATE_PERIOD = 50;


SENSOR_DATA cal_data = {0};
SENSOR_DATA cur_data = {0};
ALS31300 HallSensor = ALS31300(device_address);

#define CAN0_INT 2    // Set INT to pin 2
MCP_CAN CAN0(10);     // Set CS to pin 10

typedef union
{
  int number;
  uint8_t bytes[2];
} INT_UNION;

void setup()
{

  // Initialize the I2C communication library
  Wire.begin();
  // 1 MHz i2c clock
  Wire.setClock(1000000);

  // Sending to a software serial port, so go slow for now
  Serial.begin(19200);
  Serial.println("Serial ready!");
  scanI2CBus();

  setSerialNumber();

  LED.init();
  LED.chase(3);
  HallSensor.init();

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  // Since we do not set NORMAL mode, we are in loopback mode by default.
  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input

  sendCANStatus();  // Let the master device know whether this sensor is working

}

// loop
//
// Every half second, read the ADCs of the ALS31300 and display
// the values and toggle the state of the LED.
//
void loop()
{
  const int BLINK_PERIOD = 500;
  static long last_blink = millis();
  if (millis() - last_blink > BLINK_PERIOD) {
    static bool is_on = false;
    if (HallSensor.initOK()) {
      LED.set(GREEN, is_on);
      LED.set(RED, LOW);
    }
    else {
      LED.blink(RED, is_on);
      LED.set(GREEN, LOW);
    }
    is_on = !is_on;
    last_blink = millis();
  }
  // only perform the reading of the ALS31300 and the
  // toggling the state of the LED every half second
  static long last_update = millis();
  if (millis() - last_update > UPDATE_PERIOD) {
    SENSOR_DATA data = HallSensor.readFullLoop();
    sendCANUpdate(data);
    last_update = millis();
  }
//  readCAN();

}

void readCAN() {
  if (!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
  {
    // CAN RX Variables
    long unsigned int rxId;
    unsigned char len;
    unsigned char rxBuf[8];
    // Serial Output String Buffer
    char msgString[128];

    CAN0.readMsgBuf(&rxId, &len, rxBuf);              // Read data: len = data length, buf = data byte(s)
    int last_digit = rxId % 10;
    if (rxId >= 100 && rxId <= 400 && last_digit <= 3) {
      if ((rxId & 0x80000000) == 0x80000000)            // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
      else {
        sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
      }


      Serial.print(msgString);

      if ((rxId & 0x40000000) == 0x40000000) {          // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      }
      else {
        for (byte i = 0; i < len; i++) {
          sprintf(msgString, " 0x%.2X", rxBuf[i]);
          Serial.print(msgString);
        }
      }
      Serial.println();
    }
  }
}

void sendCANStatus() {
  const int DATA_LEN = 3;
  byte out_packet[DATA_LEN] = {0xBE, 0xEF, 0};
  out_packet[2] = (uint8_t)HallSensor.initOK();
  byte sndStat = CAN0.sendMsgBuf(SERIAL_NUM, DATA_LEN, out_packet);
}

void sendCANUpdate(SENSOR_DATA data) {

  if (data.i2c_error) {
    const int DATA_LEN = 3;
    byte out_packet[DATA_LEN] = {0xFA, 0xDE, 0};
    out_packet[2] = data.i2c_error;
    byte sndStat = CAN0.sendMsgBuf(SERIAL_NUM, DATA_LEN, out_packet);
  }
  else {
    // send data:  ID = SERIAL_NUM, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    int raw_val[3] = {data.x, data.y, data.z};
    int group_id = (int) millis();

    for (int i = 0; i < 3; i++) {
      const int DATA_LEN = 5;
      byte out_packet[DATA_LEN] = {0};

      // group_id is the two least significant bytes of the current time
      // This is used to make sure the data from the three different packets are grouped
      // together properly by the receiving device
      out_packet[0] = (byte)(group_id >> (BITS_PER_BYTE * 1));  // Most significant byte
      out_packet[1] = (byte)(group_id >> (BITS_PER_BYTE * 0));  // Least significant byte

      out_packet[2] = i;  // Send three packets, one for each value

      INT_UNION this_val;
      this_val.number = raw_val[i];
      out_packet[3] = this_val.bytes[0];  // Most significant byte
      out_packet[4] = this_val.bytes[1];  // Least significant byte


      byte sndStat = CAN0.sendMsgBuf(SERIAL_NUM, DATA_LEN, out_packet);
      //    if (sndStat == CAN_OK) {
      //      Serial.print("Message ");
      //      Serial.print(i);
      //      Serial.println(" Sent Successfully!");
      //    }
      //    else {
      //      Serial.print("Error Sending Message");
      //      Serial.print(i);
      //      Serial.println("...");
      //    }
    }
  }
}

void setSerialNumber() {
  const int SER_NUM_IS_SET_MAGIC_NUM = 0xFB;
  if (EEPROM.read(0xA9) != SER_NUM_IS_SET_MAGIC_NUM) {
    EEPROM.write(0xAA, SERIAL_NUM);
    EEPROM.write(0xA9, SER_NUM_IS_SET_MAGIC_NUM);
  }
}


void scanI2CBus()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
}
