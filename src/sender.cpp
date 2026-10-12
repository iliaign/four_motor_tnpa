// ESP32 №1: читает джойстик и шлёт кадры по проводной линии (UART2, позже RS-485).
//
// Подключение джойстика:
//   GND -> GND
//   +5V -> 3V3   (питаем от 3.3 В, чтобы выходы осей не превышали 3.3 В для АЦП ESP32)
//   URX -> GPIO34
//   URY -> GPIO35
//   SW  -> GPIO25
// Линия связи: см. protocol.h
//
// Кнопка: ОДНО нажатие переключает режим (0 = нейтраль, 1 = управление моторами).
// Удержание ничего не делает: переключение только по фронту нажатия, с антидребезгом.

#include <Arduino.h>
#include "protocol.h"

constexpr uint8_t PIN_X = 34;
constexpr uint8_t PIN_Y = 35;
constexpr uint8_t PIN_SW = 25;

constexpr uint32_t SEND_PERIOD_MS = 20;   // 50 кадров в секунду
constexpr uint32_t PRINT_PERIOD_MS = 500;
constexpr uint32_t DEBOUNCE_MS = 30;

// ---------- Джойстик: две аналоговые оси ----------
class Joystick {
 public:
  Joystick(uint8_t pinX, uint8_t pinY) : pinX_(pinX), pinY_(pinY) {}

  void begin() { analogReadResolution(12); }

  int16_t readX() const { return (int16_t)analogRead(pinX_); }
  int16_t readY() const { return (int16_t)analogRead(pinY_); }

 private:
  uint8_t pinX_;
  uint8_t pinY_;
};

// ---------- Кнопка-переключатель (защёлка) ----------
class ModeButton {
 public:
  ModeButton(uint8_t pin, uint32_t debounceMs) : pin_(pin), debounceMs_(debounceMs) {}

  void begin() {
    pinMode(pin_, INPUT_PULLUP);
    // Если кнопка зажата при старте, это не считается нажатием.
    lastRaw_ = stable_ = isPressed();
    lastChange_ = millis();
  }

  // Вызывать как можно чаще (на каждом проходе loop). true = режим только что переключился.
  bool update() {
    bool raw = isPressed();
    uint32_t now = millis();

    if (raw != lastRaw_) {
      lastRaw_ = raw;
      lastChange_ = now;
    }

    if (raw != stable_ && now - lastChange_ >= debounceMs_) {
      stable_ = raw;
      if (stable_) {  // только момент нажатия; отпускание и удержание игнорируются
        mode_ = (mode_ == MODE_NEUTRAL) ? MODE_CONTROL : MODE_NEUTRAL;
        return true;
      }
    }
    return false;
  }

  uint8_t mode() const { return mode_; }

 private:
  bool isPressed() const { return digitalRead(pin_) == LOW; }

  uint8_t pin_;
  uint32_t debounceMs_;
  bool lastRaw_ = false;
  bool stable_ = false;
  uint32_t lastChange_ = 0;
  uint8_t mode_ = MODE_NEUTRAL;  // после включения всегда нейтраль
};

// ---------- Передатчик кадров ----------
class LinkTx {
 public:
  void begin() {
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, HIGH);  // для MAX3485: передатчик включён
    Serial2.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
  }

  void send(int16_t x, int16_t y, uint8_t mode) {
    Frame f = makeFrame(x, y, mode, seq_++);
    Serial2.write((const uint8_t *)&f, sizeof(f));
    sent_++;
  }

  uint32_t sent() const { return sent_; }

 private:
  uint8_t seq_ = 0;
  uint32_t sent_ = 0;
};

Joystick joystick(PIN_X, PIN_Y);
ModeButton modeButton(PIN_SW, DEBOUNCE_MS);
LinkTx link;
Periodic sendTimer(SEND_PERIOD_MS);
Periodic printTimer(PRINT_PERIOD_MS);

void setup() {
  Serial.begin(115200);  // USB, для отладки
  joystick.begin();
  modeButton.begin();
  link.begin();
  Serial.println("Sender ready, mode NEUTRAL");
}

void loop() {
  // Кнопку опрашиваем на каждом проходе, а не только раз в 20 мс.
  if (modeButton.update()) {
    Serial.println(modeButton.mode() == MODE_CONTROL ? "MODE -> CONTROL (1)" : "MODE -> NEUTRAL (0)");
  }

  if (!sendTimer.due()) return;

  int16_t x = joystick.readX();
  int16_t y = joystick.readY();
  link.send(x, y, modeButton.mode());

  if (printTimer.due()) {
    Serial.printf("sent=%lu x=%d y=%d mode=%d\n", (unsigned long)link.sent(), x, y, modeButton.mode());
  }
}