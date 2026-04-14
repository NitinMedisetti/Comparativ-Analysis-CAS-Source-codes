#pragma once
#include <Arduino.h>

class Geolocation
{
public:
    Geolocation(double targetLat = 0.0, double targetLon = 0.0)
        : m_targetLat(targetLat),
          m_targetLon(targetLon),
          m_targetNear(false),
          m_originSet(false),
          m_refLat(targetLat),
          m_refLon(targetLon) {}

    // NEW: Explicitly set the Local Origin (0,0)
    void setOrigin(double lat, double lon)
    {
        m_refLat = lat;
        m_refLon = lon;
        m_originSet = true;
        Serial.printf("📍 Origin set to: %.6f, %.6f\n", lat, lon);
    }

    // Set or change target location dynamically
    void setTarget(double lat, double lon)
    {
        m_targetLat = lat;
        m_targetLon = lon;
        m_targetNear = false;
    }

    // Convert GPS coordinates to local N/E coordinates (meters) relative to origin
    void toLocalCoordinates(double lat, double lon, float &localN, float &localE)
    {
        if (!m_originSet)
        {
            localN = 0.0f;
            localE = 0.0f;
            return;
        }

        double dLat = lat - m_refLat;
        double dLon = lon - m_refLon;

        // Approximate meters per degree (valid for small areas)
        // 111319.9 meters per degree latitude
        localN = dLat * 111319.9;
        localE = dLon * 111319.9 * cos(m_refLat * DEG_TO_RAD);
    }

    // Euclidean distance in meters (more stable for local paths)
    float getLocalDistance(float n1, float e1, float n2, float e2)
    {
        return sqrt(pow(n2 - n1, 2) + pow(e2 - e1, 2));
    }

    // Haversine distance (Keep for legacy or long range)
    float distanceToTarget(double lat, double lon)
    {
        return haversineDistance(lat, lon, m_targetLat, m_targetLon);
    }

    // Existing helper
    void setOriginAtTarget()
    {
        setOrigin(m_targetLat, m_targetLon);
    }

    bool isNearTarget() const { return m_targetNear; }

    // Haversine formula
    float haversineDistance(double lat1, double lon1, double lat2, double lon2)
    {
        double dLat = (lat2 - lat1) * DEG_TO_RAD;
        double dLon = (lon2 - lon1) * DEG_TO_RAD;

        double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
                       sin(dLon / 2) * sin(dLon / 2);

        double c = 2 * atan2(sqrt(a), sqrt(1 - a));

        return EARTH_RADIUS * c;
    }

    // Check if close to target (Haversine based)
    bool updateProximity(double currentLat, double currentLon, float thresholdMeters = 10.0f)
    {
        float dist = haversineDistance(currentLat, currentLon, m_targetLat, m_targetLon);
        if (!m_targetNear && dist < thresholdMeters)
        {
            m_targetNear = true;
        }
        return m_targetNear;
    }

    // Getters
    double getTargetLat() const { return m_targetLat; }
    double getTargetLon() const { return m_targetLon; }

private:
    static constexpr float EARTH_RADIUS = 6371000.0f; // meters

    double m_targetLat;
    double m_targetLon;
    bool m_targetNear;
    bool m_originSet;
    double m_refLat;
    double m_refLon;
};