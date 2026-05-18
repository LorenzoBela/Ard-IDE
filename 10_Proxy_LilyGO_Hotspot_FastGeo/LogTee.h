#ifndef LOG_TEE_H
#define LOG_TEE_H

#include <Arduino.h>
#include <WiFi.h>

class SerialTee : public Stream {
public:
  explicit SerialTee(HardwareSerial &base) : _base(base) {}

  void begin(unsigned long baud) {
    _base.begin(baud);
  }

  void begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin) {
    _base.begin(baud, config, rxPin, txPin);
  }

  void end() {
    _base.end();
  }

  void setMirrorClient(WiFiClient *client) {
    _client = client;
  }

  size_t write(uint8_t c) override {
    _base.write(c);
    if (_client && _client->connected()) {
      _client->write(&c, 1);
    }
    return 1;
  }

  size_t write(const uint8_t *buffer, size_t size) override {
    _base.write(buffer, size);
    if (_client && _client->connected()) {
      _client->write(buffer, size);
    }
    return size;
  }

  int available() override {
    return _base.available();
  }

  int read() override {
    return _base.read();
  }

  int peek() override {
    return _base.peek();
  }

  void flush() override {
    _base.flush();
  }

  using Print::write;

private:
  HardwareSerial &_base;
  WiFiClient *_client = nullptr;
};

extern SerialTee LogSerial;

#endif // LOG_TEE_H
