// Общий протокол и пины проводной линии между двумя ESP32.
//
// СЕЙЧАС (без MAX3485), прямое соединение по UART:
//   sender  GPIO17 (TX2) -> receiver GPIO16 (RX2)
//   sender  GND          -> receiver GND        (общая земля обязательна!)
//   Оба уровня 3.3 В, так что платы можно соединять напрямую.
//
// ПОТОМ (с MAX3485 на каждой плате):
//   ESP32 TX2 (17) -> DI        ESP32 RX2 (16) <- RO
//   ESP32 GPIO4    -> DE и RE   (оба вывода соединены вместе)
//   VCC -> 3V3, GND -> GND
//   A <-> A, B <-> B между модулями (витая пара) + общий GND
//   120 Ом между A и B на обоих концах линии (терминаторы)
//   sender:   DE=HIGH (передатчик включён всегда)
//   receiver: DE=LOW  (приёмник включён всегда)
// Код менять не придётся: управление DE/RE уже есть, оно просто никуда не подключено.

#pragma once
#include <Arduino.h>

constexpr uint32_t LINK_BAUD = 115200;
constexpr int8_t PIN_LINK_RX = 16;  // UART2 RX
constexpr int8_t PIN_LINK_TX = 17;  // UART2 TX
constexpr int8_t PIN_RS485_DE = 4;  // DE+RE для MAX3485

constexpr uint8_t FRAME_START = 0xAA;

// Режим работы (поле Frame::mode). Переключается ОДНИМ нажатием кнопки на sender.
constexpr uint8_t MODE_NEUTRAL = 0;  // все моторы в нейтрали
constexpr uint8_t MODE_CONTROL = 1;  // моторы слушаются джойстиков

struct __attribute__((packed)) Frame {
  uint8_t start;  // всегда FRAME_START
  int16_t x;      // джойстик 1, ось X (вперёд/назад),  0..4095
  int16_t y;      // джойстик 1, ось Y (вверх/вниз),    0..4095
  int16_t x2;     // джойстик 2, ось X (поворот),       0..4095
  int16_t y2;     // джойстик 2, ось Y (резерв),        0..4095
  uint8_t mode;   // MODE_NEUTRAL / MODE_CONTROL (защёлка, а не «кнопка зажата»)
  uint8_t seq;    // счётчик кадров (переполняется 255 -> 0)
  uint8_t crc;    // CRC-8 (poly 0x07) по первым 11 байтам
};
static_assert(sizeof(Frame) == 12, "Frame must be 12 bytes");

inline uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  while (n--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

inline Frame makeFrame(int16_t x, int16_t y, int16_t x2, int16_t y2, uint8_t mode, uint8_t seq) {
  Frame f;
  f.start = FRAME_START;
  f.x = x;
  f.y = y;
  f.x2 = x2;
  f.y2 = y2;
  f.mode = mode;
  f.seq = seq;
  f.crc = crc8((const uint8_t *)&f, sizeof(Frame) - 1);
  return f;
}

// Скользящее окно: после любого сбоя или шума сам находит начало кадра по маркеру и CRC.
class FrameParser {
 public:
  bool feed(uint8_t b, Frame &out) {
    memmove(buf_, buf_ + 1, sizeof(Frame) - 1);
    buf_[sizeof(Frame) - 1] = b;
    if (buf_[0] != FRAME_START) return false;
    if (crc8(buf_, sizeof(Frame) - 1) != buf_[sizeof(Frame) - 1]) return false;
    memcpy(&out, buf_, sizeof(Frame));
    memset(buf_, 0, sizeof(buf_));
    return true;
  }

 private:
  uint8_t buf_[sizeof(Frame)] = {0};
};

// Простой периодический таймер: due() вернёт true раз в period мс.
class Periodic {
 public:
  explicit Periodic(uint32_t periodMs) : period_(periodMs) {}
  bool due() {
    uint32_t now = millis();
    if (now - last_ < period_) return false;
    last_ = now;
    return true;
  }

 private:
  uint32_t period_;
  uint32_t last_ = 0;
};