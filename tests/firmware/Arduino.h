#pragma once

#include <cstdint>
#include <deque>
#include <string>

constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OUTPUT = 1;
constexpr int INPUT = 0;
constexpr int MSBFIRST = 1;

struct FakeSerial {
  std::deque<char> bytes;
  std::string output;
  void begin(unsigned long) {}
  int available() const { return bytes.size(); }
  int read() { char byte = bytes.front(); bytes.pop_front(); return byte; }
  void print(const char* text) { output += text; }
  void println(const char* text) { output += text; output += '\n'; }
  void println(char value) { output += value; output += '\n'; }
  void printf(const char*, ...) {}
};

extern FakeSerial Serial;
void pinMode(int, int);
void digitalWrite(int, int);
void shiftOut(int, int, int, uint8_t);
void delayMicroseconds(unsigned long);
unsigned long pulseIn(int, int, unsigned long);
unsigned long millis();
void delay(unsigned long);
long random(long, long);
bool digitalPinCanOutput(uint8_t);
bool digitalPinIsValid(uint8_t);
