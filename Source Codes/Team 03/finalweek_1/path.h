String getPathCommand(float currX, float currY, float targetX, float targetY, float headingDeg)
{
    float dx = targetX - currX;
    float dy = targetY - currY;

    float distance = sqrt(dx*dx + dy*dy);

    if (distance < 3) {
        return "STOP";
    }

    // Step 1: atan2 gives angle in math coordinates (0° = East, CCW)
    float targetAngle = atan2(dy, dx) * 180.0 / PI;
    if (targetAngle < 0) targetAngle += 360;

    // Step 2: Convert to robot heading (0° = North, CW)
    targetAngle = fmod((450 - targetAngle), 360);

    // Step 3: shortest turning direction
    float diff = targetAngle - headingDeg;

    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;

    if (abs(diff) < 7)
        return "FWD";

    return (diff > 0) ? "ROT_R" : "ROT_L";
}
