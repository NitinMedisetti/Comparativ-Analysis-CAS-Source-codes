#ifndef WALL_ANGLE_CALCULATOR_H
#define WALL_ANGLE_CALCULATOR_H



#define FULL_SCAN_POINTS 360
#define MIN_POINTS_FOR_FIT 6
#define MAX_WALL_DISTANCE_MM 5000

typedef struct {
    LidarPoint points[FULL_SCAN_POINTS];
    uint16_t speed;
    bool isComplete;
} AggregatedLidarFrame;


/**
 * @brief Calculates signed wall angle relative to perpendicular
 *        Perfect perpendicular = 90°
 *        CCW rotation          > 90°
 *        CW rotation           < 90°
 */
 typedef struct {
    float angleDeg;     // 90 = perpendicular
    float distance_m;   // perpendicular distance to wall
    bool  valid;
} WallResult;

WallResult calculate_wall_angle_and_distance(AggregatedLidarFrame *frame,
                                             const char *wall)
{
    WallResult result = {0};
    result.valid = false;

    float wall_angle1 = 0.0f;
    float wall_angle2 = 0.0f;
    float ideal_angle_deg = 0.0f;

    /* ---------------------------------------------------------
     * 1. Wall sector & ideal radial direction
     * --------------------------------------------------------- */
    if (strcmp(wall, "front") == 0) {
        wall_angle1 = 330.0f; wall_angle2 = 30.0f;  ideal_angle_deg = 0.0f;
    } else if (strcmp(wall, "left") == 0) {
        wall_angle1 = 255.0f; wall_angle2 = 285.0f; ideal_angle_deg = 270.0f;
    } else if (strcmp(wall, "right") == 0) {
        wall_angle1 = 75.0f;  wall_angle2 = 105.0f; ideal_angle_deg = 90.0f;
    } else if (strcmp(wall, "back") == 0) {
        wall_angle1 = 165.0f; wall_angle2 = 195.0f; ideal_angle_deg = 180.0f;
    } else {
        return result;
    }

    bool wrap = (wall_angle1 > wall_angle2);

    float x[FULL_SCAN_POINTS];
    float y[FULL_SCAN_POINTS];
    int n = 0;

    float sum_x = 0.0f;
    float sum_y = 0.0f;

    /* ---------------------------------------------------------
     * 2. Filter + polar → Cartesian
     * --------------------------------------------------------- */
    for (int i = 0; i < FULL_SCAN_POINTS; i++) {
        const LidarPoint *p = &frame->points[i];

        float a = p->angle;
        bool in_range = wrap ?
            (a >= wall_angle1 || a <= wall_angle2) :
            (a >= wall_angle1 && a <= wall_angle2);

        if (!in_range || p->distance == 0 || p->distance > MAX_WALL_DISTANCE_MM)
            continue;

        float r = p->distance * 0.001f;
        float rad = a * (M_PI / 180.0f);

        x[n] = r * cosf(rad);
        y[n] = r * sinf(rad);

        sum_x += x[n];
        sum_y += y[n];
        n++;
    }

    if (n < MIN_POINTS_FOR_FIT)
        return result;

    /* ---------------------------------------------------------
     * 3. Least squares line fit (y = m x + b)
     * --------------------------------------------------------- */
    float mean_x = sum_x / n;
    float mean_y = sum_y / n;

    float sxx = 0.0f;
    float sxy = 0.0f;

    for (int i = 0; i < n; i++) {
        float xc = x[i] - mean_x;
        float yc = y[i] - mean_y;
        sxx += xc * xc;
        sxy += xc * yc;
    }

    if (fabsf(sxx) < 1e-6f)
        return result;

    float m = sxy / sxx;
    float b = mean_y - m * mean_x;

    /* ---------------------------------------------------------
     * 4. Wall normal (signed)
     * --------------------------------------------------------- */
    float dx = 1.0f;
    float dy = m;
    float mag = sqrtf(dx*dx + dy*dy);
    dx /= mag; dy /= mag;

    float nx = -dy;
    float ny =  dx;

    /* Ensure normal points toward robot */
    if ((mean_x * nx + mean_y * ny) > 0.0f) {
        nx = -nx;
        ny = -ny;
    }

    /* ---------------------------------------------------------
     * 5. Angle computation
     * --------------------------------------------------------- */
    float normal_angle = atan2f(ny, nx) * 180.0f / M_PI;
    if (normal_angle < 0.0f) normal_angle += 360.0f;

    float delta = normal_angle - ideal_angle_deg;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    float angle = 90.0f + delta;

    /* Constrain around perpendicular */
    while (angle > 180.0f) angle -= 180.0f;
    while (angle <   0.0f) angle += 180.0f;

    /* ---------------------------------------------------------
     * 6. Perpendicular distance to wall
     *    Line: m x - y + b = 0
     * --------------------------------------------------------- */
    float distance = fabsf(b) / sqrtf(m*m + 1.0f);

    result.angleDeg   = angle;
    result.distance_m = distance;
    result.valid      = true;

    return result;
}


#endif