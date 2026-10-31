#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    double wheel_diameter_mm[2];
    double counts_per_revolution;
    double encoder_revolutions_per_wheel_revolution[2];
    int direction[2];
    double rolling_angle_deg[2];
    double intersection_x_mm, intersection_y_mm;
} odometry_config_t;

/* Keep the integer anchor separate so large cumulative counts do not lose
   sub-count interpolation precision on conversion to double. */
typedef struct {
    int64_t counts;
    double fraction;
} odometry_position_t;

typedef struct {
    odometry_config_t config;
    double inverse[2][2], mm_per_count[2];
    odometry_position_t previous[2];
    double previous_yaw_rad;
    double x_mm, y_mm, yaw_rad;
    bool initialized;
} odometry_t;

/* Pure geometry, no HAL/RTOS. First update establishes zero at chassis centre.
   Subsequent updates must cover consecutive common sampling intervals. */
bool odometry_init(odometry_t *odometry, const odometry_config_t *config);
bool odometry_update(odometry_t *odometry, const odometry_position_t positions[2], double yaw_deg);
double odometry_yaw_deg(const odometry_t *odometry);
#endif
