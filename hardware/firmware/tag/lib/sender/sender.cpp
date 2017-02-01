//PROTOCOL: "$TOTAL_DEVICES_AROUND,ADDRESS_1,RANGE_1,ADDRESS_2,RANGE_2,ADDRESS_N,RANGE_N,TOTAL_DEVICES_SENT\r\n"

#include "sender.h"
#include "Arduino.h"

Sender::Sender()
{
  _totalDevices = 0;
  _isTriggered = false;
  _dataString = "";
  _devicesSent = 0;
}

void Sender::newDevice(byte address, float range)
{
  if (_isTriggered)
  {
    Serial.print(address);
    Serial.print(',');
    Serial.print(range);
    Serial.print(',');
    _devicesSent++;
  }
}

void Sender::addDevice()
{
  _totalDevices++;
}

void Sender::deleteDevice()
{
  _totalDevices--;
}

void Sender::_endPacket()
{
  Serial.println(_devicesSent);
  _isTriggered = false;
  _devicesSent = 0;

}

void Sender::_startPacket()
{
  Serial.print('$');
  Serial.print(_totalDevices);
  Serial.print(',');
  _isTriggered = true;
}

void Sender::loop()
{
  while (Serial.available())
  {
    char data = Serial.read();
    if (data == '+')
    {
      _startPacket();
    }
  }
  
  if (_isTriggered && _devicesSent == _totalDevices)
  {
    _endPacket();
  }
}

