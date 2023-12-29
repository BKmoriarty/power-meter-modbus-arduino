/*******************************************************************************
   ET-ESP32(WROVER) RS485 V2
   Tools->Board:"ESP32 Wrover Module"
 *******************************************************************************
   I2C Interface & I2C Bus
   -> IO22                = I2C_SCL
   -> IO21                = I2C_SDA
   -> I2C RTC:DS3231      = I2C Address : 0x68:1100100(x)
   -> I2C EEPROM 24LC16   = I2C Address : 0x50:1010000(x)
   -> I2C ADC MCP3423     = I2C Address : 0x6D:1100101(x)
   -> I2C Sensor:BME280   = I2C Address : 0x76:1110110(x)
   -> I2C Sebsor:SHT31    = I2C Address : 0x44:1000100(x)/0x45:1010101(x)
   SPI Interface SD Card
   -> SD_CS               = IO4
   -> SPI_MISO            = IO19
   -> SPI_MOSI            = IO23
   -> SPI_SCK             = IO18
   UART2 RS485 Half Duplex Auto Direction
   -> IO26                = RX2
   -> IO27                = TX2
   User Switch
   -> IO36                = USER_SW
   RTC Interrupt
   -> IO39                = RTC_INT#
 *******************************************************************************/

//=================================================================================================
#include <Wire.h>
//=================================================================================================

//=================================================================================================
// Start of Default Hardware : ET-ESP32(WROVER) RS485 V2
//=================================================================================================
// Remap Pin USART -> C:\Users\Admin\Documents\Arduino\hardware\espressif\esp32\cores\esp32\HardwareSerial.cpp
//                    C:\Users\Admin\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.0\cores\esp32\HardwareSerial.cpp
//=================================================================================================
#include <HardwareSerial.h>
//=================================================================================================
#define SerialDebug           Serial                                                              // USB Serial(Serial0)
//=================================================================================================
#define SerialRS485_RX_PIN    26
#define SerialRS485_TX_PIN    27
#define SerialRS485           Serial2                                                             // Serial2(IO27=TXD,IO26=RXD)
//=================================================================================================
#define SerialLora_RX_PIN     14
#define SerialLora_TX_PIN     13
#define SerialLora            Serial1                                                             // Serial1(IO13=TXD,IO14=RXD)
//=================================================================================================
#define LORA_RES_PIN          33                                                                  // ESP32-WROVER :IO33(LoRa-RESET)
#define LORA_RES_PRESS        LOW
#define LORA_RES_RELEASE      HIGH
//=================================================================================================
#define I2C_SCL_PIN           22                                                                  // ESP32-WROVER : IO22(SCL1)
#define I2C_SDA_PIN           21                                                                  // ESP32-WROVER : IO21(SDA1)
//=================================================================================================
#define LED_PIN               2                                                                   // ESP-WROVER  : IO2
#define LedON                 1
#define LedOFF                0
//=================================================================================================
#define USER_SW_PIN           36                                                                  // ESP32-WROVER :IO36
#define SW_PRESS              LOW
#define SW_RELEASE            HIGH
//=================================================================================================
#define RTC_INT_PIN           39                                                                  // ESP32-WROVER :IO39
#define RTC_INT_ACTIVE        LOW
#define RTC_INT_DEACTIVE      HIGH
//=================================================================================================
// End of Default Hardware : ET-ESP32(WROVER) RS485 V2
//=================================================================================================

//=================================================================================================
#include "ModbusMaster.h"                                                                         // https://github.com/4-20ma/ModbusMaster
//=================================================================================================
ModbusMaster node1;                                                                    // instantiate ModbusMaster object
//=================================================================================================

void setup()
{
  //===============================================================================================
  // Start of Initial Default Hardware : ET-ESP32(WROVER) RS485 V2
  //===============================================================================================
  SerialDebug.begin(115200);
  while (!SerialDebug);
  //===============================================================================================
  // End of Initial Default Hardware : ET-ESP32(WROVER) RS485 V2
  //===============================================================================================

  //===============================================================================================
  SerialDebug.println();
  SerialDebug.println("ET-ESP32(WROVER)RS485 V2.....Ready");
  //===============================================================================================

  //===============================================================================================
  SerialRS485.begin(115200, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
  while (!SerialRS485);
  //===============================================================================================
  node1.begin(1, SerialRS485);                                                         // Soil Moisture = Modbus slave ID 3
  //===============================================================================================
}

unsigned long period = 500;
unsigned long last_time = 0;
int states = 1;
int errorCount;

void loop()
{

  if ( millis() - last_time > period) {

    last_time = millis();

    if (states == 1) {
      //      SerialDebug.println("Scaning node 1");
      if (node1.readHoldingRegisters(0x0000, 12) == node1.ku8MBSuccess)
      {

        SerialDebug.print("L1 PHASE VOLTAGE   = ");
        SerialDebug.print(int((node1.getResponseBuffer(0) << 16) + (node1.getResponseBuffer(1))) * 0.1);
        SerialDebug.println(" V");

        SerialDebug.print("L2 PHASE VOLTAGE   = ");
        SerialDebug.print(int((node1.getResponseBuffer(2) << 16) + (node1.getResponseBuffer(3))) * 0.1);
        SerialDebug.println(" V");

        SerialDebug.print("L3 PHASE VOLTAGE   = ");
        SerialDebug.print(int((node1.getResponseBuffer(4) << 16) + (node1.getResponseBuffer(5))) * 0.1);
        SerialDebug.println(" V");

        SerialDebug.print("L1 PHASE CURRENT   = ");
        SerialDebug.print(int((node1.getResponseBuffer(6) << 16) + (node1.getResponseBuffer(7))) * 0.001);
        SerialDebug.println(" A");

        SerialDebug.print("L1 PHASE CURRENT   = ");
        SerialDebug.print(int((node1.getResponseBuffer(8) << 16) + (node1.getResponseBuffer(9))) * 0.001);
        SerialDebug.println(" A");

        SerialDebug.print("L1 PHASE CURRENT   = ");
        SerialDebug.print(int((node1.getResponseBuffer(10) << 16) + (node1.getResponseBuffer(11))) * 0.001);
        SerialDebug.println(" A");

      } else {
        errorCount++;
        SerialDebug.print("====================Eror count:");
        SerialDebug.print(errorCount);
        SerialDebug.println(" ====================");
      }
    }

    if (states == 2) {
      //      SerialDebug.println("Scaning node 2");
      if (node1.readHoldingRegisters(0x0018, 2) == node1.ku8MBSuccess)
      {

        SerialDebug.print("Frequency = ");
        SerialDebug.print(int((node1.getResponseBuffer(0) << 16) + (node1.getResponseBuffer(1))) * 0.01);
        SerialDebug.println(" Hz");

      } else {
        errorCount++;
        SerialDebug.print("====================Eror count:");
        SerialDebug.print(errorCount);
        SerialDebug.println(" ====================");
      }
    }

    //    if (states == 3) {
    //      SerialDebug.println("Scaning node 3");
    //      if (node1.readHoldingRegisters(1206, 1) == node1.ku8MBSuccess)
    //      {
    //
    //        SerialDebug.print("Unknow    = ");
    //        SerialDebug.print(node1.getResponseBuffer(0));
    //        SerialDebug.println("");
    //
    //      } else {
    //        errorCount++;
    //        SerialDebug.print("====================Eror count:");
    //        SerialDebug.print(errorCount);
    //        SerialDebug.println(" ====================");
    //      }
    //    }

    states++;
    if (states > 2) states = 1;
    node1.clearResponseBuffer();
  }

}
