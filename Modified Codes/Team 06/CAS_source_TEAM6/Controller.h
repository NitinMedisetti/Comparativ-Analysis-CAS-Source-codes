// MAC adress of ESP32-Cam module:  D8:13:2A:C2:2E:2C
// MAC adress of ESP32-Dev module:  5C:01:3B:9C:31:C4

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_MCP23X17.h>
#include <Arduino.h>

// Pin definitions for Go2 Controller (MCP23017 pin numbers)
#define START_BTN  0   // GPA0
#define L2_BTN     1   // GPA1
#define A_BTN      2   // GPA2
#define ON_OFF     3   // GPA3 
#define RELAY      4   // GPA4 

//Constants ---
const int FULL_HIGH   = 4095;
const int FULL_LOW    = 0;
const int MIDDLE      = 2048;

const int MAX_PERCENT = 100;   // logical 100%
const int SPEED_LIMIT = 28;    // actual speed limit

const int delay_routine   = 2000;
const int percent_routine = SPEED_LIMIT;

//Static Objects / State
static Adafruit_MCP4728 dac;
static Adafruit_MCP23X17 expMod;
static uint8_t expanderAddress = 0; // discovered I2C address (0 if none)

// DAC channel values
static int val_Va = MIDDLE;
static int val_Vb = MIDDLE;
static int val_Vc = MIDDLE;
static int val_Vd = MIDDLE;

//Helper Functions ---

// Maps percentage to DAC value
static int mapPercentToHalfRange(int percent, bool highDirection) {
    percent = constrain(percent, 0, MAX_PERCENT);
    return highDirection ?
           map(percent, 0, MAX_PERCENT, MIDDLE, FULL_HIGH) :
           map(percent, 0, MAX_PERCENT, MIDDLE, FULL_LOW);
}

// Resets all DAC channels to middle (neutral) position
static void resetToMiddle() {
    val_Va = MIDDLE;
    val_Vb = MIDDLE;
    val_Vc = MIDDLE;
    val_Vd = MIDDLE;
}

// Updates the DAC with current channel values
static void updateDAC() {
    dac.setChannelValue(MCP4728_CHANNEL_A, val_Va);
    dac.setChannelValue(MCP4728_CHANNEL_B, val_Vb);
    dac.setChannelValue(MCP4728_CHANNEL_C, val_Vc);
    dac.setChannelValue(MCP4728_CHANNEL_D, val_Vd);
}

// Simulates pressing the RC receiver power switch
static void RConoffswitch() {
    expMod.digitalWrite(ON_OFF, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    expMod.digitalWrite(ON_OFF, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    expMod.digitalWrite(ON_OFF, LOW);
    vTaskDelay(pdMS_TO_TICKS(2000));
    expMod.digitalWrite(ON_OFF, HIGH);
    Serial.println("RC Receiver power switch \"pressed\".");
}

//Initialization Functions ---

// Scans for MCP23017 in the 0x20..0x27 range and set expander module address
static bool scanForExpander() {
    for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            expanderAddress = addr;
            Serial.print("Found MCP23017 at 0x");
            Serial.println(addr, HEX);
            return true;
        }
        delay(1);
    }
    expanderAddress = 0;
    return false;
}

// Initializes the MCP23017 expander module
bool initExpMod() {
    // require Wire.begin() in setup() before calling this
    if (!scanForExpander()) {
        Serial.println("MCP23017 not found on 0x20..0x27 range!");
        return false;
    }

    if (!expMod.begin_I2C(expanderAddress)) {
        Serial.println("expMod.begin_I2C() failed!");
        return false;
    }

    // Set button pins as OUTPUT
    expMod.pinMode(START_BTN, OUTPUT);
    expMod.pinMode(L2_BTN, OUTPUT);
    expMod.pinMode(A_BTN, OUTPUT);
    expMod.pinMode(ON_OFF, OUTPUT);

    // Release all buttons by default (HIGH = not pressed)
    expMod.digitalWrite(START_BTN, HIGH);
    expMod.digitalWrite(L2_BTN, HIGH);
    expMod.digitalWrite(A_BTN, HIGH);
    expMod.digitalWrite(ON_OFF, HIGH);

    Serial.println("MCP23017 initialized.");
    return true;
}

// Initializes the MCP4728 DAC
bool initDAC() {
    if (!dac.begin()) {
        Serial.println("MCP4728 not found!");
        return false;
    }
    updateDAC();
    Serial.println("MCP4728 initialized.");
    return true;
}

//Movement Functions ---

// Stops all movement by resetting to middle position
inline void stop() {
    resetToMiddle();
    updateDAC();
}

// Moves forward by specified percentage
inline void forward(int percent) {
    percent = constrain(percent, 0, SPEED_LIMIT);
    resetToMiddle();
    val_Va = mapPercentToHalfRange(percent, true);
    updateDAC();
}

// Moves backward at specified percentage
inline void backward(int percent) {
    percent = constrain(percent, 0, SPEED_LIMIT);
    resetToMiddle();
    val_Va = mapPercentToHalfRange(percent, false);
    updateDAC();
}

// Moves left by specified percentage
inline void stepLeft(int percent) {
    percent = constrain(percent, 0, SPEED_LIMIT);
    resetToMiddle();
    val_Vb = mapPercentToHalfRange(percent, true);
    updateDAC();
}

// Moves right by specified percentage
inline void stepRight(int percent) {
    percent = constrain(percent, 0, SPEED_LIMIT);
    resetToMiddle();
    val_Vb = mapPercentToHalfRange(percent, false);
    updateDAC();
}

// Rotates left by specified percentage
inline void rotateLeft(int percent) {
    percent = constrain(percent, 0, SPEED_LIMIT);
    resetToMiddle();
    val_Vc = mapPercentToHalfRange(percent, true);
    updateDAC();
}

// Rotates right by specified percentage
inline void rotateRight(int percent) {
    percent = constrain(percent, 0, SPEED_LIMIT);
    resetToMiddle();
    val_Vc = mapPercentToHalfRange(percent, false);
    updateDAC();
}

//Button Control Functions ---

// Simulates pressing the Start button
inline void clickStart() {
    expMod.digitalWrite(START_BTN, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
    expMod.digitalWrite(START_BTN, HIGH);
}

// Simulates toggling (lock -> lay down -> stand up) in this exact order
inline void lockLaydownStand() {
    expMod.digitalWrite(L2_BTN, LOW);
    expMod.digitalWrite(A_BTN, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
    expMod.digitalWrite(L2_BTN, HIGH);
    expMod.digitalWrite(A_BTN, HIGH);
}

//Initial Checkup Routine ---

inline void initialcheckuproutine() {
    Serial.println("Turning the RC receiver On");
    RConoffswitch();
    delay(delay_routine);

    Serial.println("Executing: forward(28)");
    forward(percent_routine);
    delay(delay_routine);

    Serial.println("Executing: backward(28)");
    backward(percent_routine);
    delay(delay_routine);

    Serial.println("Executing: stepLeft(28)");
    stepLeft(percent_routine);
    delay(delay_routine);

    Serial.println("Executing: stepRight(28)");
    stepRight(percent_routine);
    delay(delay_routine);

    Serial.println("Executing: rotateRight(28)");
    rotateRight(percent_routine);
    delay(delay_routine);

    Serial.println("Executing: rotateLeft(28)");
    rotateLeft(percent_routine);
    delay(delay_routine);

    Serial.println("Executing: stop()");
    stop();
    delay(delay_routine);

    Serial.println("Executing: lockLaydownStand()");
    lockLaydownStand();
    delay(delay_routine);

    Serial.println("Executing: lockLaydownStand()");
    lockLaydownStand();
    delay(delay_routine);

    Serial.println("Executing: lockLaydownStand()");
    lockLaydownStand();
    delay(delay_routine);

    Serial.println("Executing: clickStart()");
    clickStart();

    Serial.println("Initial checkup routine complete.");
}

#endif
