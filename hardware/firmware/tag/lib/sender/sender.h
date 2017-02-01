#ifndef SENDER_H
#define SENDER_H

#include <Arduino.h>

class Sender{
  public:
    Sender();
    void newDevice(byte address, float range);
    void addDevice();
    void deleteDevice();
    void loop();
    
  private:
    int _totalDevices;
    bool _isTriggered;
    String _dataString;
    int _devicesSent;
    void _endPacket();
    void _startPacket();
    int _getChecksum();
};

#endif
