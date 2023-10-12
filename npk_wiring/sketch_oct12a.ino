/*
In the provided code, the Arduino Uno is interfacing with a JXCT NPK sensor using the RS485 communication protocol. Here's a breakdown of the connections between the Arduino Uno and the RS485 module:

1. *RS485 Module Connections:*
   - *RS485 DI (Data In) Signal to Pin 9:* The Data In signal from the RS485 module is connected to digital pin 9 on the Arduino Uno. This pin is used to receive data from the RS485 module.
   - *RS485 RO (Receive Out) Signal to Pin 8:* The Receive Out signal from the RS485 module is connected to digital pin 8 on the Arduino Uno. This pin is used for receiving data from the RS485 module.
   - *RS485 RE (Receiver Enable) Signal to Pin 7:* The Receiver Enable signal from the RS485 module is connected to digital pin 7 on the Arduino Uno. This pin is used to enable the receiver mode of the RS485 module.
   - *RS485 DE (Driver Enable) Signal to Pin 6:* The Driver Enable signal from the RS485 module is connected to digital pin 6 on the Arduino Uno. This pin is used to enable the driver mode of the RS485 module.
   - *RS485 VCC to 5V:* The VCC (power) pin of the RS485 module is connected to the 5V output of the Arduino Uno.
   - *RS485 GND to GND:* The Ground (GND) pin of the RS485 module is connected to the Ground (GND) pin of the Arduino Uno.

2. *MAX485 Direction Control Pins:*
   - *MAX485 DE (Driver Enable) Pin to Pin 6:* This pin is used to control the Driver Enable (DE) signal of the MAX485 chip. It is connected to digital pin 6 on the Arduino Uno.
   - *MAX485 RE (Receiver Enable) Pin to Pin 7:* This pin is used to control the Receiver Enable (RE) signal of the MAX485 chip. It is connected to digital pin 7 on the Arduino Uno.

These connections allow the Arduino Uno to communicate with the JXCT NPK sensor using the RS485 protocol.
 The AltSoftSerial library is used for serial communication, and the ModbusMaster library is utilized for Modbus communication with the sensor.
  The preTransmission and postTransmission functions are defined to control the direction of data transmission/reception on the RS485 bus.
*/
#include <ModbusMaster.h>
#include <AltSoftSerial.h>

#define NITROGEN_ADDR   0x1E
#define PHOSPHORUS_ADDR 0x1F
#define POTASSIUM_ADDR  0x20

#define MAX485_DE      6
#define MAX485_RE_NEG  7

AltSoftSerial swSerial;
ModbusMaster node;

// Put the MAX485 into transmit mode
void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

// Put the MAX485 into receive mode
void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup() {
  Serial.begin( 9600 );

  // configure the MAX485 RE & DE control signals and enable receive mode
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 9600 baud
  swSerial.begin(9600);

  // Modbus slave ID of NPK sensor is 1
  node.begin(1, swSerial);

  // Callbacks to allow us to set the RS485 Tx/Rx direction
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  uint8_t result;

  // NITROGEN
  result = node.readHoldingRegisters(NITROGEN_ADDR, 1);
  if (result == node.ku8MBSuccess)
  {
    Serial.print("   Nitrogen: ");
    Serial.print(node.getResponseBuffer(0x0));
    Serial.println(" mg/kg");
  } else {
    printModbusError( result );
  }

  // PHOSPHORUS
  result = node.readHoldingRegisters(PHOSPHORUS_ADDR, 1);
  if (result == node.ku8MBSuccess)
  {
    Serial.print("Phosphorous: ");
    Serial.print(node.getResponseBuffer(0x0));
    Serial.println(" mg/kg");
  } else {
    printModbusError( result );
  }

  // POTASSIUM
  result = node.readHoldingRegisters(POTASSIUM_ADDR, 1);
  if (result == node.ku8MBSuccess)
  {
    Serial.print("  Potassium: ");
    Serial.print(node.getResponseBuffer(0x0));
    Serial.println(" mg/kg");
  } else {
    printModbusError( result );
  }
  Serial.println();
  delay(2000);
}

// print out the error received from the Modbus library
void printModbusError( uint8_t errNum )
{
  switch ( errNum ) {
    case node.ku8MBSuccess:
      Serial.println(F("Success"));
      break;
    case node.ku8MBIllegalFunction:
      Serial.println(F("Illegal Function Exception"));
      break;
    case node.ku8MBIllegalDataAddress:
      Serial.println(F("Illegal Data Address Exception"));
      break;
    case node.ku8MBIllegalDataValue:
      Serial.println(F("Illegal Data Value Exception"));
      break;
    case node.ku8MBSlaveDeviceFailure:
      Serial.println(F("Slave Device Failure"));
      break;
    case node.ku8MBInvalidSlaveID:
      Serial.println(F("Invalid Slave ID"));
      break;
    case node.ku8MBInvalidFunction:
      Serial.println(F("Invalid Function"));
      break;
    case node.ku8MBResponseTimedOut:
      Serial.println(F("Response Timed Out"));
      break;
    case node.ku8MBInvalidCRC:
      Serial.println(F("Invalid CRC"));
      break;
    default:
      Serial.println(F("Unknown Error"));
      break;
  }
}