// ESP32 №2: принимает кадры и управляет 4 BLDC через РЕВЕРСИВНЫЕ ESC.
//
//   Моторы: 2 «горизонтальных» (M1 левый, M2 правый) и 2 «вертикальных» (M3, M4).
//   Каналы управления (из двух джойстиков):
//     SURGE (вперёд/назад)  <- джойстик 1, ось X
//     HEAVE (вверх/вниз)    <- джойстик 1, ось Y
//     YAW   (поворот)       <- джойстик 2, ось X
//   Микшерная матрица MIX превращает три канала в 4 команды -1000..+1000 ‰.
//   Центр (мёртвая зона) -> стоп (1500 мкс).
//
//   mode = 1 (кнопка нажата один раз) -> моторы слушаются джойстиков
//   mode = 0 (кнопка нажата ещё раз)  -> нейтраль
//   нет кадров > 500 мс -> нейтраль. После потери связи моторы включатся снова
//   только когда придёт mode = 0 и затем mode = 1 (то есть после нажатий кнопки).
//
// ESC: сигнал -> GPIO18, 19, 21, 22; GND -> общий GND.
// ПЕРВЫЕ ТЕСТЫ: без пропеллеров/нагрузки!

#include <Arduino.h>
#include "protocol.h"

// ---------- Настройки моторов ----------
constexpr uint8_t MOTOR_COUNT = 4;
constexpr uint8_t ESC_PINS[MOTOR_COUNT] = {18, 19, 21, 22};  // M1, M2, M3, M4

// Каналы управления (столбцы матрицы).
enum Channel : uint8_t { CH_SURGE = 0, CH_YAW = 1, CH_HEAVE = 2, CH_COUNT = 3 };

// Микшерная матрица: мотор[i] = (MIX[i][SURGE]*surge + MIX[i][YAW]*yaw + MIX[i][HEAVE]*heave) / 100.
// Каналы в диапазоне -1000..+1000 ‰. Коэффициенты в процентах, знак = направление вращения.
// Если мотор крутится «не в ту сторону», поменяй знак в его строке.
// Если стик вверх даёт погружение вместо всплытия, поменяй знак в столбце HEAVE у M3 и M4.
// Сумма модулей в строке > 100 даёт насыщение (обрезка до ±1000): при полном ходе вперёд
// и полном повороте один из горизонтальных моторов упрётся в 100 %.
//                                   SURGE  YAW  HEAVE
constexpr int8_t MIX[MOTOR_COUNT][CH_COUNT] = {
    {100, 100, 0},    // M1: горизонтальный (левый)
    {100, -100, 0},   // M2: горизонтальный (правый)
    {0, 0, 100},      // M3: вертикальный
    {0, 0, 100},      // M4: вертикальный
};

constexpr uint16_t ESC_NEUTRAL_US = 1500;  // стоп
constexpr uint16_t ESC_RANGE_US = 500;     // 1000..2000 -> 1500 ± 500
constexpr uint8_t MAX_PERCENT = 30;        // ограничение на время отладки (потом 100)

constexpr int JOY_MAX = 4095;
constexpr int JOY_CENTER = 2048;
constexpr int JOY_DEADZONE = 300;

constexpr uint32_t LINK_TIMEOUT_MS = 500;
constexpr uint32_t ARM_TIME_MS = 3000;
constexpr int RAMP_STEP = 5;                 // макс. изменение (‰) за шаг
constexpr uint32_t RAMP_PERIOD_MS = 10;
constexpr uint32_t PRINT_PERIOD_MS = 250;

// ---------- Один ESC: ШИМ 50 Гц, 16 бит (период 20000 мкс) ----------
class Esc {
 public:
  void begin(uint8_t pin, uint8_t channel) {
    channel_ = channel;
    ledcSetup(channel_, 50, 16);
    ledcAttachPin(pin, channel_);
    writeUs(ESC_NEUTRAL_US);
  }

  // -1000..+1000 ‰, минус = назад
  void writeThrottle(int permille) {
    permille = constrain(permille, -1000, 1000);
    writeUs((uint16_t)(ESC_NEUTRAL_US + (int32_t)ESC_RANGE_US * permille / 1000));
  }

 private:
  void writeUs(uint16_t us) { ledcWrite(channel_, (uint32_t)us * 65536UL / 20000UL); }

  uint8_t channel_ = 0;
};

// ---------- Микшер: оси двух джойстиков -> 3 канала -> 4 мотора ----------
class Mixer {
 public:
  void mix(const Frame &f, int out[MOTOR_COUNT]) const {
    int ch[CH_COUNT];
    ch[CH_SURGE] = axisToPermille(f.x);   // джойстик 1, X
    ch[CH_HEAVE] = axisToPermille(f.y);   // джойстик 1, Y
    ch[CH_YAW] = axisToPermille(f.x2);    // джойстик 2, X (f.y2 пока не используется)

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
      int sum = 0;
      for (uint8_t c = 0; c < CH_COUNT; c++) sum += (int)MIX[i][c] * ch[c];
      out[i] = constrain(sum / 100, -1000, 1000);
    }
  }

 private:
  // Сырое значение АЦП 0..4095 -> -1000..+1000 ‰ с мёртвой зоной вокруг центра.
  static int axisToPermille(int raw) {
    raw = constrain(raw, 0, JOY_MAX);
    int dev = raw - JOY_CENTER;
    int mag = abs(dev);
    if (mag <= JOY_DEADZONE) return 0;
    int maxDev = (dev > 0) ? (JOY_MAX - JOY_CENTER) : JOY_CENTER;
    int p = (int)map(mag, JOY_DEADZONE, maxDev, 0, 1000);
    p = constrain(p, 0, 1000);
    return (dev > 0) ? p : -p;
  }
};

// ---------- Приём кадров и состояние связи ----------
class LinkRx {
 public:
  void begin() {
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, LOW);  // приёмник включён
    Serial2.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
  }

  // Читает всё, что накопилось. Возвращает true, если состояние связи изменилось.
  bool poll() {
    while (Serial2.available()) {
      Frame f;
      if (parser_.feed((uint8_t)Serial2.read(), f)) {
        if (gotAny_) framesLost_ += (uint8_t)(f.seq - lastSeq_ - 1);
        lastSeq_ = f.seq;
        frame_ = f;
        framesOk_++;
        gotAny_ = true;
        lastRxTime_ = millis();
      }
    }
    bool nowOk = gotAny_ && (millis() - lastRxTime_ < LINK_TIMEOUT_MS);
    bool changed = (nowOk != linkOk_);
    linkOk_ = nowOk;
    return changed;
  }

  bool ok() const { return linkOk_; }
  const Frame &frame() const { return frame_; }
  uint32_t framesOk() const { return framesOk_; }
  uint32_t framesLost() const { return framesLost_; }

 private:
  FrameParser parser_;
  Frame frame_ = {FRAME_START, JOY_CENTER, JOY_CENTER, JOY_CENTER, JOY_CENTER, MODE_NEUTRAL, 0, 0};
  bool linkOk_ = false;
  bool gotAny_ = false;
  uint8_t lastSeq_ = 0;
  uint32_t lastRxTime_ = 0;
  uint32_t framesOk_ = 0;
  uint32_t framesLost_ = 0;
};

// ---------- Управление четырьмя моторами ----------
class MotorController {
 public:
  void begin() {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) esc_[i].begin(ESC_PINS[i], i);
  }

  void update(bool linkOk, const Frame &f) {
    updateEnable(linkOk, f);

    int target[MOTOR_COUNT] = {0};
    if (enabled_) {
      mixer_.mix(f, target);
      for (uint8_t i = 0; i < MOTOR_COUNT; i++) target[i] = target[i] * MAX_PERCENT / 100;
    }

    if (!enabled_) {
      // Выключение или потеря связи: мгновенный стоп без плавности.
      for (uint8_t i = 0; i < MOTOR_COUNT; i++) now_[i] = 0;
    } else if (rampTimer_.due()) {
      for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (now_[i] < target[i]) now_[i] = min(now_[i] + RAMP_STEP, target[i]);
        else if (now_[i] > target[i]) now_[i] = max(now_[i] - RAMP_STEP, target[i]);
      }
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) esc_[i].writeThrottle(now_[i]);
  }

  bool enabled() const { return enabled_; }
  int throttle(uint8_t i) const { return now_[i]; }  // ‰

 private:
  // mode=1 включает управление, mode=0 нейтраль. После потери связи (и после старта)
  // включение разрешено только после того, как придёт mode=0.
  void updateEnable(bool linkOk, const Frame &f) {
    bool want = false;
    if (!linkOk) {
      lockout_ = true;
    } else {
      if (f.mode == MODE_NEUTRAL) lockout_ = false;
      want = (f.mode == MODE_CONTROL) && !lockout_;
    }
    if (want != enabled_) {
      enabled_ = want;
      Serial.println(enabled_ ? "MOTORS: CONTROL" : "MOTORS: NEUTRAL");
    }
  }

  Esc esc_[MOTOR_COUNT];
  Mixer mixer_;
  Periodic rampTimer_{RAMP_PERIOD_MS};
  int now_[MOTOR_COUNT] = {0, 0, 0, 0};  // текущая тяга, ‰
  bool enabled_ = false;
  bool lockout_ = true;
};

LinkRx link;
MotorController motors;
Periodic printTimer(PRINT_PERIOD_MS);

void setup() {
  Serial.begin(115200);

  motors.begin();  // на все ESC уходит нейтраль 1500 мкс
  Serial.println("Arming ESC (neutral 1500us)...");
  delay(ARM_TIME_MS);

  link.begin();
  Serial.println("Receiver ready, motors NEUTRAL");
}

void loop() {
  if (link.poll()) Serial.println(link.ok() ? ">>> LINK OK" : ">>> LINK LOST");

  motors.update(link.ok(), link.frame());

  if (printTimer.due()) {
    const Frame &f = link.frame();
    Serial.printf("ok=%lu lost=%lu J1(%d,%d) J2(%d,%d) mode=%d motors=%s thr=[%.1f %.1f %.1f %.1f]%%\n",
                  (unsigned long)link.framesOk(), (unsigned long)link.framesLost(),
                  f.x, f.y, f.x2, f.y2, f.mode, motors.enabled() ? "CONTROL" : "NEUTRAL",
                  motors.throttle(0) / 10.0f, motors.throttle(1) / 10.0f,
                  motors.throttle(2) / 10.0f, motors.throttle(3) / 10.0f);
  }
}