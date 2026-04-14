#include "UsManager.h"

UsManager::UsManager()
: pins_() {}

UsManager::UsManager(const Pins& pins)
: pins_(pins) {}

void UsManager::begin() {
  auto initPair = [](int trig, int echo) {
    pinMode(trig, OUTPUT);
    digitalWrite(trig, LOW);
    pinMode(echo, INPUT);
  };

  initPair(pins_.trigFront,       pins_.echoFront);
  initPair(pins_.trigLeft,        pins_.echoLeft);
  initPair(pins_.trigRight,       pins_.echoRight);
  initPair(pins_.trigBack,        pins_.echoBack);
  initPair(pins_.trigFrontAngled, pins_.echoFrontAngled);

  // Start background task once
  if (taskHandle_ != nullptr) return;

  // Low priority; run on core 0 (Arduino loop typically runs on core 1)
  xTaskCreatePinnedToCore(
    &UsManager::taskThunk_,
    "UsManagerTask",
    4096,
    this,
    1,
    &taskHandle_,
    0
  );
}

void UsManager::update() {
  // Measurements are handled by the background task.
  // This function exists so the main sketch can call us.update() each loop.
}

int UsManager::readFrontCm() const       { return lastFrontCm_; }
int UsManager::readLeftCm() const        { return lastLeftCm_; }
int UsManager::readRightCm() const       { return lastRightCm_; }
int UsManager::readBackCm() const        { return lastBackCm_; }
int UsManager::readFrontAngledCm() const { return lastFrontAngledCm_; }

int UsManager::readCmRaw_(int trigPin, int echoPin, uint32_t timeoutUs, int maxDistanceCm) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Blocking wait for echo HIGH, but only inside the background task
  const uint32_t duration = pulseIn(echoPin, HIGH, timeoutUs);
  if (duration == 0) return maxDistanceCm;

  const int distanceCm = (int)(duration / 58UL);
  if (distanceCm <= 0 || distanceCm > maxDistanceCm) return maxDistanceCm;
  return distanceCm;
}

void UsManager::taskThunk_(void* param) {
  UsManager* self = static_cast<UsManager*>(param);
  self->taskLoop_();
}

void UsManager::taskLoop_() {
  for (;;) {
    // Copy settings atomically
    uint32_t timeoutUs;
    int maxDist;
    uint32_t interDelayMs;
    uint32_t sweepDelayMs;
    portENTER_CRITICAL(&mux_);
    timeoutUs = timeoutUs_;
    maxDist = maxDistanceCm_;
    interDelayMs = interSensorDelayMs_;
    sweepDelayMs = sweepDelayMs_;
    portEXIT_CRITICAL(&mux_);

    // Measure sequentially to avoid cross-talk
    const int dFront = readCmRaw_(pins_.trigFront, pins_.echoFront, timeoutUs, maxDist);
    portENTER_CRITICAL(&mux_);
    lastFrontCm_ = dFront;
    portEXIT_CRITICAL(&mux_);
    if (interDelayMs) vTaskDelay(pdMS_TO_TICKS(interDelayMs));

    const int dLeft = readCmRaw_(pins_.trigLeft, pins_.echoLeft, timeoutUs, maxDist);
    portENTER_CRITICAL(&mux_);
    lastLeftCm_ = dLeft;
    portEXIT_CRITICAL(&mux_);
    if (interDelayMs) vTaskDelay(pdMS_TO_TICKS(interDelayMs));

    const int dRight = readCmRaw_(pins_.trigRight, pins_.echoRight, timeoutUs, maxDist);
    portENTER_CRITICAL(&mux_);
    lastRightCm_ = dRight;
    portEXIT_CRITICAL(&mux_);
    if (interDelayMs) vTaskDelay(pdMS_TO_TICKS(interDelayMs));

    const int dBack = readCmRaw_(pins_.trigBack, pins_.echoBack, timeoutUs, maxDist);
    portENTER_CRITICAL(&mux_);
    lastBackCm_ = dBack;
    portEXIT_CRITICAL(&mux_);
    if (interDelayMs) vTaskDelay(pdMS_TO_TICKS(interDelayMs));

    const int dAng = readCmRaw_(pins_.trigFrontAngled, pins_.echoFrontAngled, timeoutUs, maxDist);
    portENTER_CRITICAL(&mux_);
    lastFrontAngledCm_ = dAng;
    portEXIT_CRITICAL(&mux_);

    if (sweepDelayMs) vTaskDelay(pdMS_TO_TICKS(sweepDelayMs));
  }
}
