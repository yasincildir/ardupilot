/*
 * Comprehensive Indoor Flight Test Suite
 * Tests all challenging indoor scenarios for obstacle detection algorithm
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <random>
#include <string>
#include <algorithm>

// Algorithm parameters (same as ArduPilot)
const float INDOOR_OBS_THR = 0.8f;      // m (tunable parameter)
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
    bool is_real_obstacle;      // true = furniture/obstacle, false = floor feature
};

// Test scenario
struct TestScenario {
    std::string name;
    std::string description;
    float duration_s;
    float target_vx;
    float floor_height;
    std::vector<EnvironmentObject> objects;
    bool expect_obstacles;      // Should we detect obstacles?
    std::string challenge_type;
};

// Sensor reading with noise
struct SensorReading {
    float distance;             // Raw distance
    float distance_noisy;       // With noise applied
    float tilt_correction;      // Tilt compensation factor
    float distance_compensated; // Tilt-compensated
    bool is_valid;
    std::string surface_type;
    bool is_real_obstacle;      // For validation
};

// Test results for each scenario
struct ScenarioResults {
    std::string name;
    int total_samples;
    int obstacles_detected;
    int false_positives;
    int missed_obstacles;
    float max_altitude_error;
    float accuracy;
};

// Global random generator
std::random_device rd;
std::mt19937 gen(rd());

// Add Gaussian noise
float add_noise(float value, float sigma) {
    std::normal_distribution<float> dist(value, sigma);
    return dist(gen);
}

// Calculate tilt correction
float calculate_tilt_correction(float pitch, float roll) {
    float cos_pitch = cosf(pitch);
    float cos_roll = cosf(roll);
    return fmaxf(0.707f, cos_pitch * cos_roll); // Min 0.707 = cos(45°)
}

// Simulate rangefinder sensor
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
    float delta_m = current_alt_m - floor_height_estimate_m;
    float time_since_update_s = (now_ms - last_floor_update_ms) / 1000.0f;

    if (time_since_update_s < 0.001f) time_since_update_s = dt;

    // Tilt-aware threshold adjustment
    float jump_threshold = obstacle_jump_threshold_m;
    float max_floor_rate = max_floor_change_rate_ms;

    if (tilt_correction <= TILT_AGGRESSIVE_THRESHOLD) {
        jump_threshold *= 0.7f;
        max_floor_rate *= 0.7f;
    }

    // Rate-of-change check
    if (fabsf(delta_m) > jump_threshold) {
        float floor_change_rate_ms = fabsf(delta_m) / time_since_update_s;
        if (floor_change_rate_ms > max_floor_rate) {
            obstacle_counter = std::min((int)(obstacle_counter + 1), (int)(HYSTERESIS_SAMPLES + 1));
        } else {
            obstacle_counter = std::max((int)(obstacle_counter - 1), -(int)(HYSTERESIS_SAMPLES + 1));
        }

        if (obstacle_counter >= HYSTERESIS_SAMPLES) {
            return true; // Obstacle detected
        }
    } else {
        obstacle_counter = std::max((int)(obstacle_counter - 1), -(int)(HYSTERESIS_SAMPLES + 1));
    }

    // Smooth floor tracking
    const float alpha = dt / (dt + FLOOR_TRACKING_TAU);
    floor_height_estimate_m += alpha * delta_m;
    last_floor_update_ms = now_ms;

    return false; // No obstacle
}

// Run a single test scenario
ScenarioResults run_scenario(const TestScenario& scenario, DroneState& drone, uint32_t& sim_time_ms) {
    ScenarioResults results;
    results.name = scenario.name;
    results.total_samples = 0;
    results.obstacles_detected = 0;
    results.false_positives = 0;
    results.missed_obstacles = 0;
    results.max_altitude_error = 0.0f;

    std::cout << "\n========================================\n";
    std::cout << "TEST: " << scenario.name << "\n";
    std::cout << "========================================\n";
    std::cout << scenario.description << "\n";
    std::cout << "Challenge: " << scenario.challenge_type << "\n\n";

    float start_x = drone.x;
    float duration_ms = scenario.duration_s * 1000.0f;
    uint32_t start_time = sim_time_ms;

    while ((sim_time_ms - start_time) < duration_ms) {
        // Update velocity smoothly
        float vel_error = scenario.target_vx - drone.vx;
        drone.vx += vel_error * 0.1f; // Smooth acceleration

        // Update pitch from velocity (forward tilt)
        drone.pitch = -atan2f(drone.vx, 10.0f); // Approximate pitch angle

        // Update position
        drone.x += drone.vx * DT;

        // Simulate rangefinder
        SensorReading reading = simulate_rangefinder(drone, scenario.objects, scenario.floor_height);

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

        // Determine ground truth
        bool real_obstacle = reading.is_real_obstacle;

        // Statistics
        results.total_samples++;
        if (is_obstacle) results.obstacles_detected++;
        if (is_obstacle && !real_obstacle) results.false_positives++;
        if (!is_obstacle && real_obstacle) results.missed_obstacles++;

        // Altitude error
        if (!is_obstacle && real_obstacle) {
            float altitude_error = fabs(reading.distance_noisy - drone.floor_estimate);
            results.max_altitude_error = fmax(results.max_altitude_error, altitude_error);
        }

        // Print periodic status (every 0.5s)
        if ((int)(sim_time_ms / 50) % 10 == 0) {
            std::cout << "[t=" << std::fixed << std::setprecision(1)
                      << ((sim_time_ms - start_time) / 1000.0f) << "s] ";
            std::cout << "x=" << std::setprecision(1) << drone.x << "m ";
            std::cout << "z=" << std::setprecision(1) << drone.z << "m ";
            std::cout << "dist=" << std::setprecision(2) << reading.distance_noisy << "m ";
            std::cout << "floor_est=" << std::setprecision(2) << drone.floor_estimate << "m ";

            if (is_obstacle) {
                std::cout << "\033[33m[OBS]\033[0m ";
            } else {
                std::cout << "[FLR] ";
            }

            if (is_obstacle != real_obstacle) {
                if (is_obstacle) {
                    std::cout << "\033[31m[FALSE+]\033[0m";
                } else {
                    std::cout << "\033[31m[MISS]\033[0m";
                }
            }
            std::cout << "\n";
        }

        sim_time_ms += 50; // 50ms timestep
    }

    // Calculate accuracy
    results.accuracy = 100.0f * (1.0f - (float)(results.false_positives + results.missed_obstacles) / results.total_samples);

    return results;
}

// Main comprehensive test suite
int main() {
    std::cout << "==============================================\n";
    std::cout << "COMPREHENSIVE INDOOR FLIGHT TEST SUITE\n";
    std::cout << "==============================================\n\n";

    std::cout << "Algorithm Parameters:\n";
    std::cout << "  INDOOR_OBS_THR  = " << INDOOR_OBS_THR << " m\n";
    std::cout << "  INDOOR_FLR_RATE = " << INDOOR_FLR_RATE << " m/s\n";
    std::cout << "  HYSTERESIS      = " << HYSTERESIS_SAMPLES << " samples\n";
    std::cout << "  TRACKING_TAU    = " << FLOOR_TRACKING_TAU << " s\n\n";

    // Initialize drone
    DroneState drone = {0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0, 0};
    uint32_t sim_time_ms = 0;

    std::vector<TestScenario> scenarios;
    std::vector<ScenarioResults> all_results;

    // Test 1: Normal flat floor (baseline)
    scenarios.push_back({
        "1. Normal Flat Floor",
        "Baseline test with flat floor, no obstacles",
        3.0f, 1.0f, 0.0f, {}, false,
        "Baseline - No challenges"
    });

    // Test 2: Low ceiling (0.5m clearance)
    scenarios.push_back({
        "2. Very Low Ceiling",
        "Flying with only 0.5m ground clearance (minimum altitude)",
        3.0f, 0.8f, 0.0f, {}, false,
        "Low altitude operation"
    });
    drone.z = 0.5f; // Reduce altitude for this test
    drone.floor_estimate = 0.5f;

    // Test 3: High ceiling jump (2m to 5m)
    drone.z = 2.0f;
    drone.floor_estimate = 2.0f;
    scenarios.push_back({
        "3. High Ceiling (Sudden Jump)",
        "Floor suddenly drops from 0m to -3m (open hall)",
        3.0f, 0.8f, -3.0f, {}, false,
        "Sudden large floor drop"
    });

    // Test 4: Dense furniture field
    scenarios.push_back({
        "4. Dense Furniture Field",
        "Navigate through cluttered room with many obstacles",
        5.0f, 0.8f, 0.0f,
        {
            {"Table 1", 1.0, 2.0, 0.0, 2.0, 0.75f, 0.6f, true},
            {"Table 2", 3.0, 4.0, 0.0, 2.0, 0.75f, 0.6f, true},
            {"Chair 1", 2.2, 2.8, 0.5, 1.5, 0.45f, 0.7f, true},
            {"Chair 2", 4.2, 4.8, 0.5, 1.5, 0.45f, 0.7f, true},
            {"Shelf", 5.0, 6.0, 1.0, 2.0, 1.80f, 0.5f, true},
        },
        true, "Dense obstacles"
    });

    // Test 5: Reflective surfaces (glass, mirror)
    scenarios.push_back({
        "5. Reflective Surfaces",
        "Poor reflectivity surfaces (glass table, mirror, polished floor)",
        4.0f, 0.8f, 0.0f,
        {
            {"Glass Table", 1.0, 2.0, 0.0, 2.0, 0.60f, 0.2f, true},  // Very reflective
            {"Mirror", 3.0, 4.0, 0.0, 2.0, 1.50f, 0.1f, true},       // Extremely reflective
        },
        true, "Poor sensor quality (reflective)"
    });

    // Test 6: Ramp (gradual slope)
    scenarios.push_back({
        "6. Gradual Ramp",
        "Ascending ramp (5° slope, 0.5m rise over 5m)",
        6.0f, 0.8f, 0.0f,
        {
            {"Ramp Seg 1", 0.0, 1.0, 0.0, 2.0, 0.10f, 0.85f, false},
            {"Ramp Seg 2", 1.0, 2.0, 0.0, 2.0, 0.20f, 0.85f, false},
            {"Ramp Seg 3", 2.0, 3.0, 0.0, 2.0, 0.30f, 0.85f, false},
            {"Ramp Seg 4", 3.0, 4.0, 0.0, 2.0, 0.40f, 0.85f, false},
            {"Ramp Seg 5", 4.0, 5.0, 0.0, 2.0, 0.50f, 0.85f, false},
        },
        false, "Gradual slope (acceptable)"
    });

    // Test 7: Staircase descending
    scenarios.push_back({
        "7. Staircase Down",
        "Descending stairs (10 steps, 2m total drop)",
        6.0f, 0.6f, -2.0f,
        {
            {"Step 1", 0.5, 0.8, 0.0, 2.0, -0.20f, 0.8f, false},
            {"Step 2", 0.8, 1.1, 0.0, 2.0, -0.40f, 0.8f, false},
            {"Step 3", 1.1, 1.4, 0.0, 2.0, -0.60f, 0.8f, false},
            {"Step 4", 1.4, 1.7, 0.0, 2.0, -0.80f, 0.8f, false},
            {"Step 5", 1.7, 2.0, 0.0, 2.0, -1.00f, 0.8f, false},
            {"Step 6", 2.0, 2.3, 0.0, 2.0, -1.20f, 0.8f, false},
            {"Step 7", 2.3, 2.6, 0.0, 2.0, -1.40f, 0.8f, false},
            {"Step 8", 2.6, 2.9, 0.0, 2.0, -1.60f, 0.8f, false},
            {"Step 9", 2.9, 3.2, 0.0, 2.0, -1.80f, 0.8f, false},
            {"Step 10", 3.2, 3.5, 0.0, 2.0, -2.00f, 0.8f, false},
        },
        false, "Stairs (gradual floor change)"
    });

    // Test 8: Staircase ascending
    drone.z = 2.0f;
    drone.floor_estimate = 2.0f;
    scenarios.push_back({
        "8. Staircase Up",
        "Ascending stairs (10 steps, 2m total rise)",
        6.0f, 0.6f, 2.0f,
        {
            {"Step 1", 0.5, 0.8, 0.0, 2.0, 0.20f, 0.8f, false},
            {"Step 2", 0.8, 1.1, 0.0, 2.0, 0.40f, 0.8f, false},
            {"Step 3", 1.1, 1.4, 0.0, 2.0, 0.60f, 0.8f, false},
            {"Step 4", 1.4, 1.7, 0.0, 2.0, 0.80f, 0.8f, false},
            {"Step 5", 1.7, 2.0, 0.0, 2.0, 1.00f, 0.8f, false},
            {"Step 6", 2.0, 2.3, 0.0, 2.0, 1.20f, 0.8f, false},
            {"Step 7", 2.3, 2.6, 0.0, 2.0, 1.40f, 0.8f, false},
            {"Step 8", 2.6, 2.9, 0.0, 2.0, 1.60f, 0.8f, false},
            {"Step 9", 2.9, 3.2, 0.0, 2.0, 1.80f, 0.8f, false},
            {"Step 10", 3.2, 3.5, 0.0, 2.0, 2.00f, 0.8f, false},
        },
        false, "Stairs ascending"
    });

    // Test 9: Mixed heights (varying obstacles)
    scenarios.push_back({
        "9. Mixed Height Obstacles",
        "Various obstacle heights (low, medium, high)",
        5.0f, 0.8f, 0.0f,
        {
            {"Low Box", 1.0, 1.5, 0.0, 2.0, 0.30f, 0.7f, true},     // 30cm
            {"Medium Table", 2.0, 3.0, 0.0, 2.0, 0.75f, 0.6f, true}, // 75cm
            {"High Shelf", 4.0, 5.0, 0.0, 2.0, 1.60f, 0.5f, true},   // 1.6m
        },
        true, "Variable obstacle heights"
    });

    // Test 10: Narrow corridor
    scenarios.push_back({
        "10. Narrow Corridor",
        "Long narrow corridor with side obstacles",
        8.0f, 1.0f, 0.0f,
        {
            {"Wall Deco 1", 2.0, 2.3, 0.0, 0.5, 0.80f, 0.6f, true},
            {"Wall Deco 2", 4.0, 4.3, 1.5, 2.0, 0.80f, 0.6f, true},
            {"Wall Deco 3", 6.0, 6.3, 0.0, 0.5, 0.80f, 0.6f, true},
        },
        true, "Narrow space navigation"
    });

    // Test 11: Uneven floor (bumps and dips)
    scenarios.push_back({
        "11. Uneven Floor",
        "Floor with small bumps and dips (±10cm variation)",
        5.0f, 0.8f, 0.0f,
        {
            {"Bump 1", 1.0, 1.5, 0.0, 2.0, 0.10f, 0.9f, false},
            {"Dip 1", 2.0, 2.5, 0.0, 2.0, -0.08f, 0.9f, false},
            {"Bump 2", 3.0, 3.5, 0.0, 2.0, 0.12f, 0.9f, false},
            {"Dip 2", 4.0, 4.5, 0.0, 2.0, -0.10f, 0.9f, false},
        },
        false, "Small floor variations"
    });

    // Test 12: Fast flight
    scenarios.push_back({
        "12. High Speed Flight",
        "Fast forward flight (2 m/s) with obstacles",
        4.0f, 2.0f, 0.0f,
        {
            {"Obstacle 1", 2.0, 3.0, 0.0, 2.0, 0.70f, 0.6f, true},
            {"Obstacle 2", 5.0, 6.0, 0.0, 2.0, 0.70f, 0.6f, true},
        },
        true, "High speed (aggressive tilt)"
    });

    // Test 13: Under furniture (low clearance)
    drone.z = 1.0f;
    drone.floor_estimate = 1.0f;
    scenarios.push_back({
        "13. Under Table Flight",
        "Flying under low obstacles (table at 1.2m height)",
        4.0f, 0.6f, 0.0f,
        {
            {"Table Surface", 1.5, 3.5, 0.0, 2.0, 0.75f, 0.6f, true},
            // Drone at 1.0m, table at 0.75m = only 0.25m clearance to sensor
        },
        false, "Very low clearance operation"
    });

    // Test 14: Balcony/Platform (elevated floor)
    drone.z = 2.0f;
    drone.floor_estimate = 2.0f;
    scenarios.push_back({
        "14. Elevated Platform",
        "Flying onto raised platform (0.5m step up)",
        4.0f, 0.8f, 0.5f,
        {
            {"Platform Edge", 2.0, 5.0, 0.0, 2.0, 0.50f, 0.9f, false},
        },
        false, "Step up (floor rise)"
    });

    // Test 15: Doorway transition
    scenarios.push_back({
        "15. Doorway Transition",
        "Flying through doorway with threshold",
        4.0f, 0.8f, 0.0f,
        {
            {"Door Threshold", 2.0, 2.2, 0.0, 2.0, 0.05f, 0.8f, false},
        },
        false, "Small step/threshold"
    });

    // Test 16: Carpet to tile transition
    scenarios.push_back({
        "16. Surface Transitions",
        "Different floor materials (carpet, tile, wood)",
        5.0f, 0.8f, 0.0f,
        {
            {"Carpet", 0.0, 1.5, 0.0, 2.0, 0.03f, 0.7f, false},   // Thick carpet, poor reflection
            {"Tile", 2.5, 4.0, 0.0, 2.0, 0.00f, 0.95f, false},     // Smooth tile, good reflection
        },
        false, "Reflectivity changes"
    });

    // Test 17: Extreme noise scenario
    scenarios.push_back({
        "17. High Sensor Noise",
        "All surfaces with poor reflectivity (worst case)",
        4.0f, 0.8f, 0.0f,
        {
            {"Dark Carpet", 1.0, 5.0, 0.0, 2.0, 0.00f, 0.3f, false}, // Very poor reflectivity
        },
        false, "Maximum sensor noise"
    });

    // Test 18: Complex multi-floor scenario
    scenarios.push_back({
        "18. Multi-Floor Building",
        "Complete building: rooms → corridor → stairs → exit",
        15.0f, 0.8f, -2.0f,
        {
            // Room 1 (floor 0m)
            {"Sofa", 1.0, 2.5, 0.0, 2.0, 0.50f, 0.75f, true},
            {"TV", 2.0, 2.5, 1.0, 1.5, 0.60f, 0.4f, true},
            // Corridor
            {"Plant", 5.0, 5.5, 0.5, 1.0, 0.40f, 0.65f, true},
            // Transition to stairs
            {"Landing 1", 7.0, 7.5, 0.0, 2.0, -0.15f, 0.9f, false},
            {"Landing 2", 7.5, 8.0, 0.0, 2.0, -0.30f, 0.9f, false},
            // Stairs
            {"Step 1", 8.0, 8.3, 0.0, 2.0, -0.50f, 0.8f, false},
            {"Step 2", 8.3, 8.6, 0.0, 2.0, -0.70f, 0.8f, false},
            {"Step 3", 8.6, 8.9, 0.0, 2.0, -0.90f, 0.8f, false},
            {"Step 4", 8.9, 9.2, 0.0, 2.0, -1.10f, 0.8f, false},
            {"Step 5", 9.2, 9.5, 0.0, 2.0, -1.30f, 0.8f, false},
            {"Step 6", 9.5, 9.8, 0.0, 2.0, -1.50f, 0.8f, false},
            {"Step 7", 9.8, 10.1, 0.0, 2.0, -1.70f, 0.8f, false},
            {"Step 8", 10.1, 10.4, 0.0, 2.0, -1.90f, 0.8f, false},
            // Lower floor
            {"Desk", 12.0, 13.0, 0.0, 2.0, -1.25f, 0.6f, true},
        },
        true, "Real-world complex scenario"
    });

    // Run all scenarios
    for (auto& scenario : scenarios) {
        ScenarioResults results = run_scenario(scenario, drone, sim_time_ms);
        all_results.push_back(results);

        // Print scenario results
        std::cout << "\n--- RESULTS ---\n";
        std::cout << "Samples: " << results.total_samples << "\n";
        std::cout << "Obstacles Detected: " << results.obstacles_detected << "\n";
        std::cout << "False Positives: " << results.false_positives << "\n";
        std::cout << "Missed Obstacles: " << results.missed_obstacles << "\n";
        std::cout << "Accuracy: ";
        if (results.accuracy >= 95.0f) {
            std::cout << "\033[32m";
        } else if (results.accuracy >= 85.0f) {
            std::cout << "\033[33m";
        } else {
            std::cout << "\033[31m";
        }
        std::cout << std::fixed << std::setprecision(1) << results.accuracy << "%\033[0m\n";
    }

    // Final summary
    std::cout << "\n\n==============================================\n";
    std::cout << "COMPREHENSIVE TEST SUMMARY\n";
    std::cout << "==============================================\n\n";

    std::cout << std::setw(40) << std::left << "Test Scenario"
              << std::setw(10) << "Accuracy"
              << std::setw(10) << "False+"
              << std::setw(10) << "Missed"
              << "Status\n";
    std::cout << std::string(70, '-') << "\n";

    int passed = 0, warning = 0, failed = 0;
    float total_accuracy = 0.0f;

    for (const auto& result : all_results) {
        std::cout << std::setw(40) << std::left << result.name;

        // Color code accuracy
        if (result.accuracy >= 95.0f) {
            std::cout << "\033[32m";
            passed++;
        } else if (result.accuracy >= 85.0f) {
            std::cout << "\033[33m";
            warning++;
        } else {
            std::cout << "\033[31m";
            failed++;
        }

        std::cout << std::setw(10) << std::fixed << std::setprecision(1) << result.accuracy << "%";
        std::cout << "\033[0m";
        std::cout << std::setw(10) << result.false_positives;
        std::cout << std::setw(10) << result.missed_obstacles;

        if (result.accuracy >= 95.0f) {
            std::cout << "\033[32m✓ PASS\033[0m";
        } else if (result.accuracy >= 85.0f) {
            std::cout << "\033[33m⚠ WARN\033[0m";
        } else {
            std::cout << "\033[31m✗ FAIL\033[0m";
        }
        std::cout << "\n";

        total_accuracy += result.accuracy;
    }

    std::cout << std::string(70, '-') << "\n";
    std::cout << "Overall Average Accuracy: ";
    float avg_accuracy = total_accuracy / all_results.size();
    if (avg_accuracy >= 95.0f) {
        std::cout << "\033[32m";
    } else if (avg_accuracy >= 85.0f) {
        std::cout << "\033[33m";
    } else {
        std::cout << "\033[31m";
    }
    std::cout << std::fixed << std::setprecision(1) << avg_accuracy << "%\033[0m\n\n";

    std::cout << "Test Results:\n";
    std::cout << "  \033[32m✓ PASSED:  " << passed << "\033[0m\n";
    std::cout << "  \033[33m⚠ WARNING: " << warning << "\033[0m\n";
    std::cout << "  \033[31m✗ FAILED:  " << failed << "\033[0m\n\n";

    std::cout << "==============================================\n";

    return 0;
}
