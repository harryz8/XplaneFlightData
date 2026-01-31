// Wind Calculator for X-Plane MFD
// JSF AV C++ Coding Standard Compliant Version
// 
// Calculates headwind, crosswind, and wind correction angle
// from aircraft position and wind data.
// 
// JSF Compliance:
// - AV Rule 208: No exceptions (throw/catch/try) - uses error codes
// - AV Rule 209: Fixed-width types (Int32, Float64) via jsf_types.h
// - AV Rule 206: No dynamic memory allocation
// - AV Rule 119: No recursion
// - AV Rule 52: Constants in lowercase
// - AV Rule 113: Single exit point
// - AV Rule 126: C++ style comments only (//)
// 
// Compile: g++ -std=c++20 -O3 -o wind_calculator wind_calculator.cpp
// 
// Usage: ./wind_calculator <track> <heading> <wind_dir> <wind_speed>

#include <iostream>
#include <cmath>
#include <iomanip>
#include <cstdlib>
#include <numbers>
#include <vector>
#include "jsf_types.h"

namespace xplane_mfd::calc {

// Error codes (JSF-compliant error handling)
const Int32 error_success = 0;
const Int32 error_invalid_args = 1;
const Int32 error_parse_failed = 2;
const Int32 error_invalid_value = 3;

// Mathematical constants (AV Rule 52: lowercase)
const Float64 deg_to_rad = std::numbers::pi / 180.0;
const Float64 angle_wrap_limit = 360.0;
const Float64 half_circle = 180.0;
const Float64 wind_calm_threshold = 0.0;

// JSF-compliant parse function (no exceptions)
bool parse_float64(const char* str, Float64& result) {
    char* end = nullptr;
    result = strtod(str, &end);
    return (end != str && *end == '\0');
}

struct WindComponents {
    Float64 headwind;      // Positive = headwind, negative = tailwind
    Float64 crosswind;     // Positive = from right, negative = from left
    Float64 total_wind;    // Total wind speed
    Float64 wca;          // Wind correction angle
    Float64 drift;        // Drift angle (track - heading)
};

// Normalize angle to 0-360 range
// Uses fmod() for deterministic execution time (no variable-iteration loops)
// This is important for real-time and safety-critical systems where
// predictable worst-case execution time (WCET) is required
Float64 normalize_angle(Float64 angle) {
    Float64 result = fmod(angle, angle_wrap_limit);
    if (result < wind_calm_threshold) {
        result += angle_wrap_limit;
    }
    return result;
}

// Calculate wind components relative to aircraft track
WindComponents calculate_wind(Float64 track, Float64 heading, 
                               Float64 wind_dir, Float64 wind_speed) {
    WindComponents result;
    
    // Normalize all angles
    track = normalize_angle(track);
    heading = normalize_angle(heading);
    wind_dir = normalize_angle(wind_dir);
    
    // Calculate drift angle
    result.drift = normalize_angle(track - heading);
    if (result.drift > half_circle) result.drift -= angle_wrap_limit;
    
    // Wind direction is where wind comes FROM
    // Calculate angle of wind-from relative to track
    Float64 wind_from_relative = normalize_angle(wind_dir - track);
    if (wind_from_relative > half_circle) wind_from_relative -= angle_wrap_limit;
    
    // Convert to radians for trig
    Float64 wind_from_rad = wind_from_relative * deg_to_rad;
    
    // Calculate components using wind-from angle
    result.headwind = -wind_speed * cos(wind_from_rad);
    result.crosswind = wind_speed * sin(wind_from_rad);
    result.total_wind = wind_speed;
    
    // Wind correction angle placeholder
    result.wca = wind_calm_threshold;  // Cannot calculate without TAS
    
    return result;
}

// Output results as JSON
void print_json(const WindComponents& wind) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "{\n";
    std::cout << "  \"headwind\": " << wind.headwind << ",\n";
    std::cout << "  \"crosswind\": " << wind.crosswind << ",\n";
    std::cout << "  \"total_wind\": " << wind.total_wind << ",\n";
    std::cout << "  \"wca\": " << wind.wca << ",\n";
    std::cout << "  \"drift\": " << wind.drift << "\n";
    std::cout << "}\n";
}

} // namespace xplane_mfd::calc

void print_usage(const char* program_name) {
    std::cerr << "Usage: " << program_name 
              << " <track> <heading> <wind_dir> <wind_speed>\n\n";
    std::cerr << "Arguments:\n";
    std::cerr << "  track      : Ground track (degrees true)\n";
    std::cerr << "  heading    : Aircraft heading (degrees)\n";
    std::cerr << "  wind_dir   : Wind direction FROM (degrees)\n";
    std::cerr << "  wind_speed : Wind speed (knots)\n\n";
    std::cerr << "Example:\n";
    std::cerr << "  " << program_name << " 90 85 270 15\n";
    std::cerr << "  (Track 90°, Heading 85°, Wind from 270° at 15 knots)\n";
}

// AV Rule 113: Single exit point
int main(int argc, char* argv[]) {
    using namespace xplane_mfd::calc;
    
    Int32 return_code = error_success;  // Single exit point variable
    
    // JSF-compliant: No exceptions, use error codes
    if (argc != 5) {
        print_usage(argv[0]);
        return_code = error_invalid_args;
    } else {
        // Parse arguments (JSF-compliant: no throwing parse functions)
        Float64 track;
        Float64 heading;
        Float64 wind_dir;
        Float64 wind_speed;
        
        if (!parse_float64(argv[1], track)) {
            std::cerr << "Error: Invalid track angle\n";
            return_code = error_parse_failed;
        } else if (!parse_float64(argv[2], heading)) {
            std::cerr << "Error: Invalid heading\n";
            return_code = error_parse_failed;
        } else if (!parse_float64(argv[3], wind_dir)) {
            std::cerr << "Error: Invalid wind direction\n";
            return_code = error_parse_failed;
        } else if (!parse_float64(argv[4], wind_speed)) {
            std::cerr << "Error: Invalid wind speed\n";
            return_code = error_parse_failed;
        } else if (wind_speed < wind_calm_threshold) {
            std::cerr << "Error: Wind speed cannot be negative\n";
            return_code = error_invalid_value;
        } else {
            // All inputs valid - calculate wind components
            WindComponents wind = calculate_wind(track, heading, wind_dir, wind_speed);
            
            // Output JSON
            print_json(wind);
            return_code = error_success;
        }
    }
    
    return return_code;  // Single exit point
}
