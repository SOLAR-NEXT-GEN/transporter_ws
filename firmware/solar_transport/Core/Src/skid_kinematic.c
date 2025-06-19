/*
 * skid_kinematic.c
 *
 *  Created on: Apr 8, 2025
 *      Author: transporter
 */

#include "skid_kinematic.h"
#include <math.h>

void SKID_init(SKID_KINEMATIC* skid,
               float wheel_radius, float wheel_distance,
               float max_velocity, float max_angular_vel) {
    skid->wheel_radius = wheel_radius;
    skid->wheel_distance = wheel_distance;
    skid->max_velocity = max_velocity;
    skid->max_angular_vel = max_angular_vel;
    skid->left_cmd = 0.0f;
    skid->right_cmd = 0.0f;

    // Initialize pose
    skid->x = 0.0f;
    skid->y = 0.0f;
    skid->theta = 0.0f;
    skid->last_update_time = 0;
}

void SKID_inverse_kinematic(SKID_KINEMATIC* skid, float linear_vel, float angular_vel) {
    // Clamp input velocities to maximum values
    if (linear_vel > skid->max_velocity) {
        linear_vel = skid->max_velocity;
    } else if (linear_vel < -skid->max_velocity) {
        linear_vel = -skid->max_velocity;
    }

    if (angular_vel > skid->max_angular_vel) {
        angular_vel = skid->max_angular_vel;
    } else if (angular_vel < -skid->max_angular_vel) {
        angular_vel = -skid->max_angular_vel;
    }

    // Calculate wheel velocities using the inverse kinematic model
    float left_wheel_vel = linear_vel - (angular_vel * skid->wheel_distance / 2.0f);
    float right_wheel_vel = linear_vel + (angular_vel * skid->wheel_distance / 2.0f);

    // Normalize to the range of -100 to 100 for motor commands
    float max_wheel_vel = skid->max_velocity + (skid->max_angular_vel * skid->wheel_distance / 2.0f);

    skid->left_cmd = (left_wheel_vel / max_wheel_vel);
    skid->right_cmd = (right_wheel_vel / max_wheel_vel);
}

void SKID_forward_kinematic(SKID_KINEMATIC* skid, float left_vel, float right_vel, float* linear_vel, float* angular_vel) {
    // Normalize input velocities (assuming they are in the range -100 to 100)
    float max_wheel_vel = skid->max_velocity + (skid->max_angular_vel * skid->wheel_distance / 2.0f);

    float left_wheel_vel = (left_vel / 100.0f) * max_wheel_vel;
    float right_wheel_vel = (right_vel / 100.0f) * max_wheel_vel;

    // Calculate linear and angular velocities using the forward kinematic model
    *linear_vel = (left_wheel_vel + right_wheel_vel) / 2.0f;
    *angular_vel = (right_wheel_vel - left_wheel_vel) / skid->wheel_distance;

    // Clamp output velocities to maximum values
    if (*linear_vel > skid->max_velocity) {
        *linear_vel = skid->max_velocity;
    } else if (*linear_vel < -skid->max_velocity) {
        *linear_vel = -skid->max_velocity;
    }

    if (*angular_vel > skid->max_angular_vel) {
        *angular_vel = skid->max_angular_vel;
    } else if (*angular_vel < -skid->max_angular_vel) {
        *angular_vel = -skid->max_angular_vel;
    }
}

void SKID_calculate_pose(SKID_KINEMATIC* skid, float omega_left_front, float omega_left_rear,
                         float omega_right_front, float omega_right_rear, uint32_t current_time) {
    // Skip if this is the first update
    if (skid->last_update_time == 0) {
        skid->last_update_time = current_time;
        return;
    }

    // Calculate time delta in seconds (1000Hz = 0.001 seconds per step)
    float dt = (float)(current_time - skid->last_update_time) / 1000.0f;

    // Ensure we're not processing too fast or if timer wrapped around
    if (dt <= 0 || dt > 0.1f) {
        skid->last_update_time = current_time;
        return;
    }

    // Average omega for each side (rad/s)
    float omega_left = (omega_left_front + omega_left_rear) / 2.0f;
    float omega_right = (omega_right_front + omega_right_rear) / 2.0f;

    // Calculate wheel linear velocities from angular velocities
    float v_left = omega_left * skid->wheel_radius;
    float v_right = omega_right * skid->wheel_radius;

    // Calculate robot velocities
    float v = (v_right + v_left) / 2.0f;                     // Linear velocity (m/s)
    float omega = (v_right - v_left) / skid->wheel_distance; // Angular velocity (rad/s)

    // Update pose using simple Euler integration
    // For higher accuracy, consider using Runge-Kutta or similar methods
    float delta_theta = omega * dt;

    // Use average orientation during the time step for better accuracy
    float theta_avg = skid->theta + delta_theta / 2.0f;

    // Update position based on linear velocity and orientation
    skid->x += v * cosf(theta_avg) * dt;
    skid->y += v * sinf(theta_avg) * dt;
    skid->theta += delta_theta;

    // Normalize theta to -π to π range
    if (skid->theta > M_PI)
        skid->theta -= 2.0f * M_PI;
    else if (skid->theta < -M_PI)
        skid->theta += 2.0f * M_PI;

    // Update last update time
    skid->last_update_time = current_time;
}

void SKID_reset_pose(SKID_KINEMATIC* skid, float x, float y, float theta) {
    skid->x = x;
    skid->y = y;
    skid->theta = theta;

    // Don't reset the timestamp to avoid large delta time in the next update
}
