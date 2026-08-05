#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "telemetry_protocol.h"

#ifndef CELL_ID
#error "CELL_ID must be defined before including cell_node_impl.h"
#endif

#ifndef ADC_PIN
#define ADC_PIN D0
#endif
#ifndef R_TOP_OHM
#define R_TOP_OHM 100000.0F
#endif
#ifndef R_BOTTOM_OHM
#define R_BOTTOM_OHM 100000.0F
#endif
#ifndef CALIBRATION_GAIN
#define CALIBRATION_GAIN 1.0F
#endif
#ifndef CALIBRATION_OFFSET_V
#define CALIBRATION_OFFSET_V 0.0F
#endif
#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1
#endif
#ifndef CELL_TEMPERATURE_C
#define CELL_TEMPERATURE_C 25.0F
#endif

namespace {

constexpr uint32_t kSendIntervalMs = 250;
constexpr uint8_t kSamplesPerReading = 32;
constexpr uint16_t kAdcMaxCount = 4095;
constexpr uint8_t kBroadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint32_t sequence_number = 0;
uint32_t next_send_ms = 0;
volatile bool last_send_ok = false;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataSent(const wifi_tx_info_t *, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *, esp_now_send_status_t status) {
#endif
  last_send_ok = (status == ESP_NOW_SEND_SUCCESS);
}

void stopWithError(const char *message, esp_err_t error) {
  Serial.printf("FATAL: %s (error=%d)\n", message, static_cast<int>(error));
  while (true) {
    delay(1000);
  }
}

bms::CellTelemetry takeReading() {
  uint32_t raw_sum = 0;
  uint32_t mv_sum = 0;
  uint16_t raw_peak = 0;

  for (uint8_t i = 0; i < kSamplesPerReading; ++i) {
    const uint16_t raw = analogRead(ADC_PIN);
    const uint32_t mv = analogReadMilliVolts(ADC_PIN);
    raw_sum += raw;
    mv_sum += mv;
    raw_peak = max(raw_peak, raw);
    delay(2);
  }

  const uint16_t raw_average =
      static_cast<uint16_t>(raw_sum / kSamplesPerReading);
  const float adc_voltage =
      (static_cast<float>(mv_sum) / kSamplesPerReading) / 1000.0F;
  const float divider_ratio =
      (R_TOP_OHM + R_BOTTOM_OHM) / R_BOTTOM_OHM;
  const float cell_voltage =
      (adc_voltage * divider_ratio * CALIBRATION_GAIN) +
      CALIBRATION_OFFSET_V;

  bms::CellTelemetry packet{};
  packet.magic = bms::kPacketMagic;
  packet.protocol_version = bms::kProtocolVersion;
  packet.cell_id = CELL_ID;
  packet.flags = (raw_peak >= kAdcMaxCount - 8) ? bms::kAdcSaturated : 0;
  packet.sequence = sequence_number++;
  packet.uptime_ms = millis();
  packet.raw_adc = raw_average;
  packet.sample_count = kSamplesPerReading;
  packet.adc_voltage_v = adc_voltage;
  packet.cell_voltage_v = cell_voltage;
  // Placeholder until a LiPo-mounted temperature sensor is selected. Each
  // cell node sends its own value so the master can report their average.
  packet.temperature_c = CELL_TEMPERATURE_C;
  return packet;
}

void setupRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_err_t result = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (result != ESP_OK) stopWithError("cannot set Wi-Fi channel", result);

  result = esp_now_init();
  if (result != ESP_OK) stopWithError("ESP-NOW init failed", result);

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, kBroadcastAddress, sizeof(kBroadcastAddress));
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  result = esp_now_add_peer(&peer);
  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
    stopWithError("cannot add broadcast peer", result);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  static_assert(CELL_ID >= 1 && CELL_ID <= bms::kCellCount,
                "CELL_ID must be 1..4");

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
  setupRadio();

  // Stagger nodes that are powered on together to reduce RF collisions.
  next_send_ms = millis() + (static_cast<uint32_t>(CELL_ID) * 40U);

  Serial.printf("Cell %u ready, MAC=%s, channel=%u\n", CELL_ID,
                WiFi.macAddress().c_str(), ESPNOW_CHANNEL);
}

void loop() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - next_send_ms) < 0) return;
  next_send_ms = now + kSendIntervalMs;

  const bms::CellTelemetry packet = takeReading();
  const esp_err_t result =
      esp_now_send(kBroadcastAddress,
                   reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));

  Serial.printf("cell=%u seq=%lu raw=%u adc=%.4fV cell=%.4fV temp=%.1fC queued=%s last=%s%s\n",
                CELL_ID, static_cast<unsigned long>(packet.sequence),
                packet.raw_adc, packet.adc_voltage_v, packet.cell_voltage_v, packet.temperature_c,
                result == ESP_OK ? "yes" : "no", last_send_ok ? "ok" : "fail",
                (packet.flags & bms::kAdcSaturated) ? " SATURATED" : "");
}
