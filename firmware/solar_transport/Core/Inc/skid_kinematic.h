/*
 * skid_kinematic.h
 *
 *  Created on: Apr 8, 2025
 *      Author: transporter
 */

/*
 * skid_kinematic.h
 *
 *  Created on: Apr 8, 2025
 *      Author: transporter
 */
#ifndef INC_SKID_KINEMATIC_H_
#define INC_SKID_KINEMATIC_H_
#include "main.h"

typedef struct {
    float wheel_radius;      // Wheel radius in meters
    float wheel_distance;    // Distance between left and right wheels in meters
    float max_velocity;      // Maximum linear velocity in m/s
    float max_angular_vel;   // Maximum angular velocity in rad/s
    float left_cmd;          // Left wheel command (-100 to 100)
    float right_cmd;         // Right wheel command (-100 to 100)

    // Robot pose
    float x;                 // X position in meters
    float y;                 // Y position in meters
    float theta;             // Orientation in radians

    // Timestamp for pose calculation
    uint32_t last_update_time; // Last update time in milliseconds
} SKID_KINEMATIC;

// Function prototypes
void SKID_init(SKID_KINEMATIC* skid,
               float wheel_radius, float wheel_distance,
               float max_velocity, float max_angular_vel);

// Convert linear and angular velocity to wheel commands
void SKID_inverse_kinematic(SKID_KINEMATIC* skid, float linear_vel, float angular_vel);

// Convert wheel commands to linear and angular velocity
void SKID_forward_kinematic(SKID_KINEMATIC* skid, float left_vel, float right_vel, float* linear_vel, float* angular_vel);

// Calculate robot pose from wheel angular velocities (omegas) at 1000Hz
void SKID_calculate_pose(SKID_KINEMATIC* skid, float omega_left_front, float omega_left_rear,
                         float omega_right_front, float omega_right_rear, uint32_t current_time);

// Reset robot pose to a specific position and orientation
void SKID_reset_pose(SKID_KINEMATIC* skid, float x, float y, float theta);

#endif /* INC_SKID_KINEMATIC_H_ */
