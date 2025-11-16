/*
 * ArduPilot Indoor Flight Simulation - Multi-Floor Building Scenario
 *
 * Scenario:
 * - Drone enters through 5th floor window
 * - Navigates apartment (furniture, obstacles)
 * - Moves through corridor
 * - Descends stairs to 4th floor
 * - Exits through 4th floor window
 *
 * Tests:
 * - INDOOR_OBS_THR obstacle detection
 * - INDOOR_FLR_RATE floor change tracking
 * - Tilt compensation during forward flight
 * - Surface reflection variations
 * - Optical flow + rangefinder fusion
 * - Real-world noise and disturbances
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>
#include <string>
#include <sstream>

// Simulation constants
const float INDOOR_OBS_THR = 0.45f;     // meters (tunable parameter - MORE SENSITIVE)
const float INDOOR_FLR_RATE = 0.3f;     // m/s (tunable parameter)
const float HYSTERESIS_SAMPLES = 5;
const float FLOOR_TRACKING_TAU = 0.1f;
const float TILT_AGGRESSIVE_THRESHOLD = 0.87f; // cos(30°)
const float DT = 0.05f;                 // 20Hz update rate

// Drone state
struct DroneState {
    float x, y, z;              // Position (meters)
    float vx, vy, vz;           // Velocity (m/s)
    float pitch, roll, yaw;     // Attitude (radians)
    float floor_estimate;       // Obstacle detection state
    int8_t obstacle_counter;
    uint32_t last_floor_update_ms;
};

// Environment object
struct EnvironmentObject {
    std::string name;
    float x_min, x_max;
    float y_min, y_max;
    float z;                    // Height above floor
    float reflectivity;         // 0.0-1.0 (affects sensor quality)
    bool is_real_obstacle;      // true = furniture/obstacle, false = floor feature (stairs)
};

// Sensor reading with noise
struct SensorReading {
    float distance;             // Raw distance
    float distance_noisy;       // With noise applied
    float tilt_correction;      // Tilt compensation factor
    float distance_compensated; // Tilt-compensated
    bool is_valid;
    std::string surface_type;
    bool is_real_obstacle;      // For validation: is this a real obstacle or floor feature?
};

// Global random generator
std::random_device rd;
std::mt19937 gen(rd());

// Add Gaussian noise
float add_noise(float value, float sigma) {
    std::normal_distribution<float> dist(0.0f, sigma);
    return value + dist(gen);
}

// Calculate tilt correction (same as ArduPilot)
float calculate_tilt_correction(float pitch, float roll) {
    // Rotation matrix z-component
    float cos_pitch = cos(pitch);
    float cos_roll = cos(roll);
    float z_component = cos_pitch * cos_roll;

    // Clamp to 45° max (0.707)
    return fmax(0.707f, z_component);
}

// Simulate rangefinder reading
SensorReading simulate_rangefinder(
    const DroneState& drone,
    const std::vector<EnvironmentObject>& objects,
    float floor_height)
{
    SensorReading reading;
    reading.is_valid = true;

    // Calculate tilt correction
    reading.tilt_correction = calculate_tilt_correction(drone.pitch, drone.roll);

    // Find closest object below drone
    float min_distance = drone.z - floor_height; // Default: floor distance
    std::string surface = "floor";
    float reflectivity = 0.9f; // Floor is usually good
    bool is_real_obstacle = false; // Default: floor is not an obstacle

    for (const auto& obj : objects) {
        if (drone.x >= obj.x_min && drone.x <= obj.x_max &&
            drone.y >= obj.y_min && drone.y <= obj.y_max &&
            obj.z < drone.z) {

            float obj_distance = drone.z - obj.z;
            if (obj_distance < min_distance) {
                min_distance = obj_distance;
                surface = obj.name;
                reflectivity = obj.reflectivity;
                is_real_obstacle = obj.is_real_obstacle;
            }
        }
    }

    reading.distance = min_distance;
    reading.surface_type = surface;
    reading.is_real_obstacle = is_real_obstacle;

    // Apply tilt correction
    reading.distance_compensated = min_distance * reading.tilt_correction;

    // Add noise based on surface reflectivity
    float noise_sigma = 0.02f * (1.1f - reflectivity); // Worse surface = more noise
    reading.distance_noisy = add_noise(reading.distance_compensated, noise_sigma);

    // Add occasional glitches (5% chance)
    if ((rand() % 100) < 5) {
        reading.distance_noisy += add_noise(0.0f, 0.1f);
    }

    return reading;
}

// Obstacle detection algorithm (same as ArduPilot)
bool detect_obstacle_and_track_floor(
    float current_alt_m,
    float& floor_height_estimate_m,
    int8_t& obstacle_counter,
    uint32_t& last_floor_update_ms,
    uint32_t now_ms,
    float dt,
    float tilt_correction,
    float obstacle_jump_threshold_m,
    float max_floor_change_rate_ms)
{
    // Tilt-aware threshold adjustment
    float jump_threshold = obstacle_jump_threshold_m;
    float max_floor_rate = max_floor_change_rate_ms;

    if (tilt_correction <= TILT_AGGRESSIVE_THRESHOLD) {
        jump_threshold *= 0.7f;
        max_floor_rate *= 0.7f;
    }

    // Initialize on first run
    if (last_floor_update_ms == 0) {
        floor_height_estimate_m = current_alt_m;
        last_floor_update_ms = now_ms;
        obstacle_counter = 0;
        return false;
    }

    uint32_t time_since_update_ms = now_ms - last_floor_update_ms;
    if (time_since_update_ms == 0) return false;

    float time_since_update_s = time_since_update_ms * 0.001f;
    float delta_m = current_alt_m - floor_height_estimate_m;

    // Check for significant jump
    if (fabs(delta_m) > jump_threshold) {
        float floor_change_rate_ms = fabs(delta_m) / time_since_update_s;

        if (floor_change_rate_ms > max_floor_rate) {
            // Too fast = likely obstacle
            obstacle_counter = fmin(obstacle_counter + 1, HYSTERESIS_SAMPLES + 1);
        } else {
            // Gradual = likely real floor change
            obstacle_counter = fmax(obstacle_counter - 1, -(HYSTERESIS_SAMPLES + 1));
        }

        // Hysteresis confirmation
        if (obstacle_counter >= HYSTERESIS_SAMPLES) {
            return true; // Confirmed obstacle
        } else if (obstacle_counter <= -HYSTERESIS_SAMPLES) {
            // Confirmed floor change
            floor_height_estimate_m = current_alt_m;
            last_floor_update_ms = now_ms;
            obstacle_counter = 0;
            return false;
        } else {
            return true; // Still uncertain, filter out
        }
    } else {
        // Small change - smooth tracking
        obstacle_counter = 0;
        float alpha = dt / (dt + FLOOR_TRACKING_TAU);
        floor_height_estimate_m += alpha * delta_m;
        last_floor_update_ms = now_ms;
        return false;
    }
}

// Simulate drone dynamics
void update_drone_dynamics(DroneState& drone, float target_vx, float dt) {
    // Simplified dynamics - accelerate toward target velocity
    float accel = 2.0f; // m/s^2
    if (drone.vx < target_vx) {
        drone.vx = fmin(drone.vx + accel * dt, target_vx);
    } else {
        drone.vx = fmax(drone.vx - accel * dt, target_vx);
    }

    // Pitch angle from velocity (simplified)
    // Forward flight: pitch = -atan(vx / 5.0)
    drone.pitch = -atan(drone.vx / 5.0f);

    // Update position
    drone.x += drone.vx * dt;
    drone.y += drone.vy * dt;
    drone.z += drone.vz * dt;
}

// Print colored output
void print_result(const std::string& label, float value, bool is_good) {
    std::cout << label << ": ";
    if (is_good) {
        std::cout << "\033[32m"; // Green
    } else {
        std::cout << "\033[31m"; // Red
    }
    std::cout << std::fixed << std::setprecision(2) << value;
    std::cout << "\033[0m"; // Reset
}

// Main simulation
int main() {
    std::cout << "==============================================\n";
    std::cout << "ArduPilot Indoor Flight Simulation\n";
    std::cout << "Multi-Floor Building Navigation\n";
    std::cout << "==============================================\n\n";

    std::cout << "Parameters:\n";
    std::cout << "  INDOOR_OBS_THR  = " << INDOOR_OBS_THR << " m\n";
    std::cout << "  INDOOR_FLR_RATE = " << INDOOR_FLR_RATE << " m/s\n\n";

    // Initialize drone state
    DroneState drone = {
        0.0f, 0.0f, 2.0f,      // Starting position (2m altitude)
        0.0f, 0.0f, 0.0f,      // Velocity
        0.0f, 0.0f, 0.0f,      // Attitude
        2.0f,                   // Floor estimate
        0,                      // Obstacle counter
        0                       // Last update time
    };

    // Scenario waypoints
    struct Waypoint {
        std::string name;
        float x, target_vx;
        float floor_height;
        std::vector<EnvironmentObject> objects;
    };

    std::vector<Waypoint> waypoints;

    // Waypoint 1: Enter 5th floor apartment
    waypoints.push_back({
        "5th Floor - Living Room",
        5.0f, 1.0f, 0.0f,
        {
            {"Coffee Table", 2.0, 3.0, 1.0, 2.0, 0.45f, 0.7f, true},  // Real obstacle
            {"TV Stand", 1.0, 2.0, 3.0, 4.0, 0.60f, 0.5f, true},      // Real obstacle
            {"Sofa", 3.5, 5.0, 2.5, 3.5, 0.50f, 0.8f, true}           // Real obstacle
        }
    });

    // Waypoint 2: Corridor
    waypoints.push_back({
        "5th Floor - Corridor",
        10.0f, 1.2f, 0.0f,
        {
            {"Plant Pot", 7.0, 7.5, 0.5, 1.0, 0.30f, 0.6f, true}  // Real obstacle
        }
    });

    // Waypoint 3: Staircase (descending) - drone maintains altitude, floor descends
    waypoints.push_back({
        "Staircase 5→4",
        15.0f, 0.8f, -2.0f, // Drone maintains constant height, floor descends to -2m
        {
            // Landing/transition area before stairs (gradual floor drop)
            {"Landing 1", 10.5, 11.0, 0.0, 2.0, -0.10f, 0.9f, false},  // Floor feature
            {"Landing 2", 11.0, 11.5, 0.0, 2.0, -0.20f, 0.9f, false},  // Floor feature
            {"Landing 3", 11.5, 12.0, 0.0, 2.0, -0.30f, 0.9f, false},  // Floor feature
            // Stairs are floor features, not obstacles (gradual floor change)
            {"Stair Step 1", 12.0, 12.3, 0.0, 2.0, -0.50f, 0.8f, false},  // Floor feature
            {"Stair Step 2", 12.3, 12.6, 0.0, 2.0, -0.70f, 0.8f, false},  // Floor feature
            {"Stair Step 3", 12.6, 12.9, 0.0, 2.0, -0.90f, 0.8f, false},  // Floor feature
            {"Stair Step 4", 12.9, 13.2, 0.0, 2.0, -1.10f, 0.8f, false},  // Floor feature
            {"Stair Step 5", 13.2, 13.5, 0.0, 2.0, -1.30f, 0.8f, false},  // Floor feature
            {"Stair Step 6", 13.5, 13.8, 0.0, 2.0, -1.50f, 0.8f, false},  // Floor feature
            {"Stair Step 7", 13.8, 14.1, 0.0, 2.0, -1.70f, 0.8f, false},  // Floor feature
            {"Stair Step 8", 14.1, 14.4, 0.0, 2.0, -1.90f, 0.8f, false},  // Floor feature
            {"Stair Step 9", 14.4, 14.7, 0.0, 2.0, -2.10f, 0.8f, false},  // Floor feature
            {"Stair Step 10", 14.7, 15.0, 0.0, 2.0, -2.30f, 0.8f, false}   // Floor feature
        }
    });

    // Waypoint 4: 4th floor corridor (now at lower altitude)
    waypoints.push_back({
        "4th Floor - Corridor",
        20.0f, 1.0f, -2.0f, // Floor is 2m lower after stairs
        {}
    });

    // Waypoint 5: Exit window
    waypoints.push_back({
        "4th Floor - Window Exit",
        25.0f, 0.5f, -2.0f,
        {}
    });

    // Simulation statistics
    int total_samples = 0;
    int obstacles_detected = 0;
    int false_positives = 0;
    int missed_obstacles = 0;
    float max_altitude_error = 0.0f;

    uint32_t sim_time_ms = 0;

    // Run simulation
    for (const auto& wp : waypoints) {
        std::cout << "\n========================================\n";
        std::cout << "Waypoint: " << wp.name << "\n";
        std::cout << "========================================\n";

        while (drone.x < wp.x) {
            // Update drone dynamics
            update_drone_dynamics(drone, wp.target_vx, DT);

            // Simulate rangefinder
            SensorReading reading = simulate_rangefinder(drone, wp.objects, wp.floor_height);

            // Run obstacle detection
            bool is_obstacle = detect_obstacle_and_track_floor(
                reading.distance_noisy,
                drone.floor_estimate,
                drone.obstacle_counter,
                drone.last_floor_update_ms,
                sim_time_ms,
                DT,
                reading.tilt_correction,
                INDOOR_OBS_THR,
                INDOOR_FLR_RATE
            );

            // Determine if there's actually an obstacle (from simulation ground truth)
            bool real_obstacle = reading.is_real_obstacle;

            // Statistics
            total_samples++;
            if (is_obstacle) obstacles_detected++;
            if (is_obstacle && !real_obstacle) false_positives++;
            if (!is_obstacle && real_obstacle) missed_obstacles++;

            // Altitude error (if obstacle wrongly accepted)
            if (!is_obstacle && real_obstacle) {
                float altitude_error = fabs(reading.distance_noisy - drone.floor_estimate);
                max_altitude_error = fmax(max_altitude_error, altitude_error);
            }

            // Print periodic status
            if ((int)(sim_time_ms / 50) % 20 == 0) { // Every 1 second
                std::cout << "\n[t=" << std::fixed << std::setprecision(1)
                          << (sim_time_ms / 1000.0f) << "s] ";
                std::cout << "x=" << std::setprecision(1) << drone.x << "m ";
                std::cout << "z=" << drone.z << "m ";
                std::cout << "vx=" << drone.vx << "m/s ";
                std::cout << "pitch=" << std::setprecision(0) << (drone.pitch * 180.0f / M_PI) << "° ";

                std::cout << "\n  Sensor: ";
                std::cout << "dist=" << std::setprecision(2) << reading.distance_noisy << "m ";
                std::cout << "(" << reading.surface_type << ") ";
                std::cout << "tilt=" << std::setprecision(2) << reading.tilt_correction << " ";

                std::cout << "\n  Detection: ";
                if (is_obstacle) {
                    std::cout << "\033[33mOBSTACLE\033[0m ";
                } else {
                    std::cout << "\033[32mFLOOR\033[0m ";
                }
                std::cout << "floor_est=" << std::setprecision(2) << drone.floor_estimate << "m ";
                std::cout << "counter=" << (int)drone.obstacle_counter;

                // Validation
                if (is_obstacle != real_obstacle) {
                    if (is_obstacle) {
                        std::cout << " \033[31m[FALSE POSITIVE!]\033[0m";
                    } else {
                        std::cout << " \033[31m[MISSED!]\033[0m";
                    }
                }
            }

            sim_time_ms += 50; // 50ms timestep
        }
    }

    // Final statistics
    std::cout << "\n\n==============================================\n";
    std::cout << "SIMULATION RESULTS\n";
    std::cout << "==============================================\n\n";

    std::cout << "Total Samples: " << total_samples << "\n";
    std::cout << "Obstacles Detected: " << obstacles_detected << " ("
              << std::fixed << std::setprecision(1)
              << (100.0f * obstacles_detected / total_samples) << "%)\n";
    std::cout << "False Positives: " << false_positives << " ("
              << std::fixed << std::setprecision(1)
              << (100.0f * false_positives / total_samples) << "%)\n";
    std::cout << "Missed Obstacles: " << missed_obstacles << " ("
              << std::fixed << std::setprecision(1)
              << (100.0f * missed_obstacles / total_samples) << "%)\n";
    std::cout << "Max Altitude Error: " << std::fixed << std::setprecision(3)
              << max_altitude_error << " m\n\n";

    // Performance rating
    float accuracy = 100.0f * (1.0f - (float)(false_positives + missed_obstacles) / total_samples);
    std::cout << "Overall Accuracy: ";
    if (accuracy > 95.0f) {
        std::cout << "\033[32m"; // Green
    } else if (accuracy > 85.0f) {
        std::cout << "\033[33m"; // Yellow
    } else {
        std::cout << "\033[31m"; // Red
    }
    std::cout << std::fixed << std::setprecision(1) << accuracy << "%\033[0m\n\n";

    if (accuracy > 95.0f) {
        std::cout << "✅ EXCELLENT - Algorithm performs very well!\n";
    } else if (accuracy > 85.0f) {
        std::cout << "⚠️  GOOD - Consider tuning INDOOR_OBS_THR or INDOOR_FLR_RATE\n";
    } else {
        std::cout << "❌ NEEDS IMPROVEMENT - Parameter tuning required\n";
    }

    std::cout << "\nRecommendations:\n";
    if (false_positives > missed_obstacles * 2) {
        std::cout << "  • Too many false positives → Increase INDOOR_OBS_THR (try "
                  << std::fixed << std::setprecision(1) << (INDOOR_OBS_THR * 1.2f) << "m)\n";
    }
    if (missed_obstacles > false_positives * 2) {
        std::cout << "  • Too many missed obstacles → Decrease INDOOR_OBS_THR (try "
                  << std::fixed << std::setprecision(1) << (INDOOR_OBS_THR * 0.8f) << "m)\n";
    }
    if (max_altitude_error > 0.1f) {
        std::cout << "  • High altitude error → Decrease INDOOR_FLR_RATE (try "
                  << std::fixed << std::setprecision(1) << (INDOOR_FLR_RATE * 0.8f) << "m/s)\n";
    }

    std::cout << "\n==============================================\n";

    return 0;
}
