#pragma once
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

class IMU
{
public:
    IMU(uint8_t address = 0x28) : m_bno(55, address), m_isCalibrated(false) {}

    bool begin()
    {
        if (!m_bno.begin())
        {
            return false;
        }
        // Use NDOF mode for sensor fusion and magnetic heading
        m_bno.setMode(OPERATION_MODE_NDOF);
        delay(100);
        return true;
    }

    /**
     * @brief Reads orientation and applies correction for True North.
     * @param roll Output for roll angle (degrees).
     * @param pitch Output for pitch angle (degrees).
     * @param yaw Output for True North heading (degrees).
     * @param declinationDegrees Magnetic declination value (+East, -West).
     */
    void readOrientation(float &roll, float &pitch, float &yaw, float declinationDegrees)
    {
        sensors_event_t event;
        m_bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

        // BNO055 Euler angle mapping:
        // event.orientation.x = yaw (Magnetic Heading)
        float magneticYaw = event.orientation.x;

        // Apply True North Correction: True Yaw = Magnetic Yaw + Declination
        float trueYaw = magneticYaw + declinationDegrees;

        // Normalize the angle to keep it between 0 and 360 degrees
        while (trueYaw >= 360.0f) {
            trueYaw -= 360.0f;
        }
        while (trueYaw < 0.0f) {
            trueYaw += 360.0f;
        }

        yaw = trueYaw;
        roll = event.orientation.y;
        pitch = event.orientation.z;
    }

    /**
     * @brief Reads the Z-axis angular velocity (Yaw Rate) from the gyroscope.
     * @return Yaw Rate in radians/second.
     */
    float getYawRateRadS()
    {
        const float PI_OVER_180_F = 3.14159265358979323846f / 180.0f;
        imu::Vector<3> gyro = m_bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        return (float)gyro.z() * PI_OVER_180_F;
    }

    // --- NEW FUNCTION ADDED FOR GYRO SPRINT (Degrees Per Second) ---
    float readGyroRateZ()
    {
        // Get raw Gyroscope vector (x, y, z)
        imu::Vector<3> gyro = m_bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        // Return Z-axis (Spinning left/right) in Degrees
        return (float)gyro.z(); 
    }

    /**
     * @brief Check and update calibration status.
     * @return true if fully calibrated (system calibration = 3)
     */
    bool updateCalibration()
    {
        uint8_t sys, gyro, accel, mag;
        m_bno.getCalibration(&sys, &gyro, &accel, &mag);

        if (!m_isCalibrated)
        {
            Serial.printf("CALIBRATION: Sys=%d Gyro=%d Accel=%d Mag=%d\n",
                          sys, gyro, accel, mag);
        }

        m_isCalibrated = (sys == 3);
        return m_isCalibrated;
    }

    bool isCalibrated() const
    {
        return m_isCalibrated;
    }

private:
    Adafruit_BNO055 m_bno;
    bool m_isCalibrated;
};