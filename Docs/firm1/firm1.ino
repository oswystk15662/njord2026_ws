#include <HardwareSerial.h>
#include "thrust_duty_profile.hpp"

#define TIMER_12_BIT  12
#define BASE_FREQ     50
#define SERVO_PIN_1   16
#define SERVO_PIN_2   4
#define SERVO_PIN_3   18
#define SERVO_PIN_4   17

#define FET_SW        14

#define OLD_ESC_C     1500
#define OLD_ESC_MIN   1000
#define OLD_ESC_MAX   2000

#define TIMEOUT       5000

#define LED_G_PIN     25
#define LED_Y_PIN     26
#define LED_R_PIN     27
#define RELAY_PIN     2

const bool DEBUG = false;
const int  SERIAL_SPEED = 115200;

// パケットサイズ: float×4 + uint8×1 = 17バイト
#define PACKET_SIZE   17

unsigned long time_data = 0;
unsigned long now = 0;


void servoAnalogWrite(uint8_t pin, uint32_t value, uint32_t valueMax = 20000) {
  uint32_t duty = (uint32_t)((4095 * value) / valueMax);
  ledcWrite(pin, duty);
}

void servoCalibration() {
  servoAnalogWrite(SERVO_PIN_1, OLD_ESC_C);
  servoAnalogWrite(SERVO_PIN_2, OLD_ESC_C);
  servoAnalogWrite(SERVO_PIN_3, OLD_ESC_C);
  servoAnalogWrite(SERVO_PIN_4, OLD_ESC_C);
  delay(3000);
}

void servoSetCenter() {
  servoAnalogWrite(SERVO_PIN_1, OLD_ESC_C);
  servoAnalogWrite(SERVO_PIN_2, OLD_ESC_C);
  servoAnalogWrite(SERVO_PIN_3, OLD_ESC_C);
  servoAnalogWrite(SERVO_PIN_4, OLD_ESC_C);
}

void isTimeOut() {
  now = millis();
  if (now - time_data > TIMEOUT) {
    servoSetCenter();
  }
}

void setup() {
  ledcAttach(SERVO_PIN_1, BASE_FREQ, TIMER_12_BIT);
  ledcAttach(SERVO_PIN_2, BASE_FREQ, TIMER_12_BIT);
  ledcAttach(SERVO_PIN_3, BASE_FREQ, TIMER_12_BIT);
  ledcAttach(SERVO_PIN_4, BASE_FREQ, TIMER_12_BIT);

  pinMode(FET_SW,    OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_Y_PIN, OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(RELAY_PIN, INPUT_PULLUP);

  digitalWrite(FET_SW, HIGH);

  Serial.begin(SERIAL_SPEED);

  servoCalibration();
}

void loop() {
  if (Serial.available() >= PACKET_SIZE) {
    uint8_t buf[PACKET_SIZE];
    Serial.readBytes(buf, PACKET_SIZE);
    time_data = millis();

    // th1〜th4: 推力（N）をfloatとしてパース
    float th1, th2, th3, th4;
    memcpy(&th1, buf +  0, 4);
    memcpy(&th2, buf +  4, 4);
    memcpy(&th3, buf +  8, 4);
    memcpy(&th4, buf + 12, 4);

    // settingバイトのビット展開
    // bit3: 緊急停止, bit2: LED緑, bit1: LED黄, bit0: LED赤
    uint8_t setting = buf[16];
    bool em_stop = (setting >> 3) & 0x01;
    int  led_g   = (setting >> 2) & 0x01;
    int  led_y   = (setting >> 1) & 0x01;
    int  led_r   = (setting >> 0) & 0x01;

    if (em_stop) {
      servoSetCenter();
      digitalWrite(FET_SW, LOW);
    } else {
      digitalWrite(FET_SW, HIGH);

      // 推力（N）→PWMパルス幅（µs）変換
      uint16_t pwm1 = thrust_profile::forceN_to_on_time(th1);
      uint16_t pwm2 = thrust_profile::forceN_to_on_time(th2);
      uint16_t pwm3 = thrust_profile::forceN_to_on_time(th3);
      uint16_t pwm4 = thrust_profile::forceN_to_on_time(th4);

      // ピンマッピングは元コードと同じ
      servoAnalogWrite(SERVO_PIN_4, pwm1);
      servoAnalogWrite(SERVO_PIN_1, pwm2);
      servoAnalogWrite(SERVO_PIN_3, pwm3);
      servoAnalogWrite(SERVO_PIN_2, pwm4);
    }

    digitalWrite(LED_G_PIN, led_g);
    digitalWrite(LED_Y_PIN, led_y);
    digitalWrite(LED_R_PIN, led_r);

    // ESP→PC: 1バイト, bit0 = 緊急停止リレーの動作状態
    // INPUT_PULLUPのためLOW=リレー動作中
    uint8_t relay_state = (digitalRead(RELAY_PIN) == LOW) ? 0x01 : 0x00;
    Serial.write(relay_state);
  }

  isTimeOut();
}
