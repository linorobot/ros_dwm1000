
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000Device.h"
// #include "sender.h"

// Sender sender;
#define SENSOR_OFFSET 0.4
// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

void setup() {
  Serial.begin(115200);
  delay(1000);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.useRangeFilter(true);

  //start the hardware as tag
  DW1000Ranging.startAsTag("01:00:5B:D5:A9:9A:E2:9C",DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop() {
  DW1000Ranging.loop();
  //keep waiting for command from ROS node.
  // sender.loop();
}

void newRange() {
  //insert to the message
  // sender.newDevice(DW1000Ranging.getDistantDevice()->getShortAddress(),
  //                  DW1000Ranging.getDistantDevice()->getRange() - SENSOR_OFFSET);
  Serial.print('$');
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(',');
  Serial.println(DW1000Ranging.getDistantDevice()->getRange());

}

void newDevice(DW1000Device* device) {
  //add device from total no of devices around
  // sender.addDevice();
}

void inactiveDevice(DW1000Device* device) {
  //delete device from total no of devices around
  // sender.deleteDevice();

}
