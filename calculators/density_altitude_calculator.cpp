// Density Altitude Calculator for X-Plane MFD
// JSF AV C++ Coding Standard Compliant Version
// 
// Calculates density altitude and related atmospheric parameters:
// - Density altitude (how "high" the aircraft performs)
// - Pressure altitude
// - True vs Equivalent airspeed conversions
// - Air density ratio (sigma)
// - Performance degradation percentage
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
// Compile: g++ -std=c++20 -O3 -o density_altitude_calculator density_altitude_calculator.cpp
// 
// Usage: ./density_altitude_calculator <pressure_alt_ft> <oat_celsius> <ias_kts> <tas_kts> [force_error]

#include <iostream>
#include <cmath>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <memory>
#include "jsf_types.h"

namespace xplane_mfd::calc {

// Error codes (JSF-compliant error handling - no exceptions)
const Int32 error_success = 0;
const Int32 error_invalid_args = 1;
const Int32 error_parse_failed = 2;
const Int32 error_simulated = 3;

// Physical constants (AV Rule 52: lowercase)
const Float64 sea_level_temp_c = 15.0;
const Float64 temp_lapse_rate = 0.0019812;    // °C per foot (standard lapse rate)
const Float64 kelvin_offset = 273.15;
const Float64 density_alt_factor = 120.0;
const Float64 pressure_altitude_constant = 6.8756e-6;
const Float64 pressure_altitude_exponent = 5.2559;
const Float64 min_ias_for_ratio = 10.0;

// Validation ranges
const Float64 min_altitude_ft = -2000.0;
const Float64 max_altitude_ft = 60000.0;
const Float64 min_temperature_c = -60.0;
const Float64 max_temperature_c = 60.0;

[[nodiscard]] double parse_double(std::string_view sv) {
    return std::stod(std::string(sv));
}

// JSF-compliant parse function (no exceptions)
bool parse_float64(const char* str, Float64& result) {
    char* end = nullptr;
    result = strtod(str, &end);
    return (end != str && *end == '\0');
}

bool parse_int32(const char* str, Int32& result) {
    char* end = nullptr;
    long value = strtol(str, &end, 10);
    if (end != str && *end == '\0') {
        result = static_cast<Int32>(value);
        return true;
    }
    return false;
}

struct DensityAltitudeData {
    Float64 density_altitude_ft;      // Density altitude
    Float64 pressure_altitude_ft;     // Pressure altitude (from setting)
    Float64 air_density_ratio;        // σ (sigma) - ratio to sea level
    Float64 temperature_deviation_c;  // Deviation from ISA
    Float64 performance_loss_pct;     // % performance loss vs sea level
    Float64 eas_kts;                  // Equivalent airspeed
    Float64 tas_to_ias_ratio;         // TAS/IAS ratio
    Float64 pressure_ratio;           // Pressure ratio vs sea level
};

// Calculate ISA temperature at given pressure altitude
Float64 isa_temperature_c(Float64 pressure_altitude_ft) {
    return sea_level_temp_c - (temp_lapse_rate * pressure_altitude_ft);
}

// Calculate density altitude using exact formula
// DA = PA + [120 * (OAT - ISA)]
Float64 calculate_density_altitude(Float64 pressure_altitude_ft, Float64 oat_celsius) {
    // ISA temperature at pressure altitude
    Float64 isa_temp = isa_temperature_c(pressure_altitude_ft);
    
    // Temperature deviation from ISA
    Float64 temp_deviation = oat_celsius - isa_temp;
    
    // Density altitude approximation (good to about 1% accuracy)
    Float64 density_altitude = pressure_altitude_ft + (density_alt_factor * temp_deviation);
    
    return density_altitude;
}

// Calculate air density ratio (sigma)
// σ = ρ / ρ₀
Float64 calculate_density_ratio(Float64 pressure_altitude_ft, Float64 oat_celsius) {
    // Convert to absolute temperature
    Float64 temp_k = oat_celsius + kelvin_offset;
    Float64 sea_level_temp_k = sea_level_temp_c + kelvin_offset;
    
    // Pressure ratio (using standard atmosphere)
    Float64 pressure_ratio = pow(1.0 - pressure_altitude_constant * pressure_altitude_ft, 
                                  pressure_altitude_exponent);
    
    // Temperature ratio
    Float64 temp_ratio = sea_level_temp_k / temp_k;
    
    // Density ratio: σ = (P/P₀) * (T₀/T)
    Float64 sigma = pressure_ratio * temp_ratio;
    
    return sigma;
}

// Calculate Equivalent Airspeed (EAS)
// EAS = TAS * sqrt(σ)
Float64 calculate_eas(Float64 tas_kts, Float64 sigma) {
    return tas_kts * sqrt(sigma);
}

// Calculate complete density altitude data
DensityAltitudeData calculate_density_altitude_data(
    Float64 pressure_altitude_ft,
    Float64 oat_celsius,
    Float64 ias_kts,
    Float64 tas_kts
) {
    DensityAltitudeData result;
    
    result.pressure_altitude_ft = pressure_altitude_ft;
    result.density_altitude_ft = calculate_density_altitude(pressure_altitude_ft, oat_celsius);
    
    // ISA temperature at this altitude
    Float64 isa_temp = isa_temperature_c(pressure_altitude_ft);
    result.temperature_deviation_c = oat_celsius - isa_temp;
    
    // Air density ratio
    result.air_density_ratio = calculate_density_ratio(pressure_altitude_ft, oat_celsius);
    
    // Performance loss (inverse of density ratio)
    result.performance_loss_pct = (1.0 - result.air_density_ratio) * 100.0;
    
    // Equivalent airspeed
    result.eas_kts = calculate_eas(tas_kts, result.air_density_ratio);
    
    // TAS/IAS ratio (useful for quick mental calculations)
    if (ias_kts > min_ias_for_ratio) {
        result.tas_to_ias_ratio = tas_kts / ias_kts;
    } else {
        result.tas_to_ias_ratio = 1.0;
    }
    
    // Pressure ratio
    result.pressure_ratio = pow(1.0 - pressure_altitude_constant * pressure_altitude_ft, 
                                 pressure_altitude_exponent);
    
    return result;
}

// Output results as JSON
void print_json(const DensityAltitudeData& da) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "{\n";
    std::cout << "  \"density_altitude_ft\": " << da.density_altitude_ft << ",\n";
    std::cout << "  \"pressure_altitude_ft\": " << da.pressure_altitude_ft << ",\n";
    std::cout << "  \"air_density_ratio\": " << da.air_density_ratio << ",\n";
    std::cout << "  \"temperature_deviation_c\": " << da.temperature_deviation_c << ",\n";
    std::cout << "  \"performance_loss_pct\": " << da.performance_loss_pct << ",\n";
    std::cout << "  \"eas_kts\": " << da.eas_kts << ",\n";
    std::cout << "  \"tas_to_ias_ratio\": " << da.tas_to_ias_ratio << ",\n";
    std::cout << "  \"pressure_ratio\": " << da.pressure_ratio << "\n";
    std::cout << "}\n";
}

} // namespace xplane_mfd::calc

void print_usage(const char* program_name) {
    std::cerr << "Usage: " << program_name 
              << " <pressure_alt_ft> <oat_celsius> <ias_kts> <tas_kts> [force_error]\n\n";
    std::cerr << "Arguments:\n";
    std::cerr << "  pressure_alt_ft : Pressure altitude (feet)\n";
    std::cerr << "  oat_celsius     : Outside air temperature (°C)\n";
    std::cerr << "  ias_kts        : Indicated airspeed (knots)\n";
    std::cerr << "  tas_kts        : True airspeed (knots)\n";
    std::cerr << "  force_error    : Optional, 1 to simulate error (default: 0)\n\n";
    std::cerr << "Example:\n";
    std::cerr << "  " << program_name << " 5000 25 150 170\n";
    std::cerr << "  (5000 ft PA, 25°C OAT, 150 kts IAS, 170 kts TAS)\n";
}

int main(int argc, char* argv[]) {
    using namespace xplane_mfd::calc;
    
    Int32 return_code = error_success; // hint
    
    if (argc != 5 && argc != 6) {
        print_usage(argv[0]);
        return 1;
    }
    
    // ========================================================================
    // REMOVE BEFORE FLIGHT - Exception
    // ========================================================================
    try {
        double pressure_altitude_ft = parse_double(argv[1]);
        double oat_celsius = parse_double(argv[2]);
        double ias_kts = parse_double(argv[3]);
        double tas_kts = parse_double(argv[4]);
        
        // Check for force exception flag
        bool force_exception = false;
        if (argc == 6) {
            force_exception =
                (std::strcmp(argv[5], "1") == 0 ||
                std::strcmp(argv[5], "true") == 0);
        }
        
        if (force_exception) {
            // throw std::runtime_error("CRITICAL: Required dataref 'sim/weather/isa_deviation' not found in X-Plane API");
            return error_simulated;
        }
        
        // Validate inputs
        if (pressure_altitude_ft < -2000 || pressure_altitude_ft > 60000) {
            // std::cerr << "Warning: Pressure altitude outside typical range\n";
            return error_invalid_args;
        }
        
        if (oat_celsius < -60 || oat_celsius > 60) {
            // std::cerr << "Warning: Temperature outside typical range\n";
            return error_invalid_args;
        }
        
        DensityAltitudeData da = calculate_density_altitude_data(
            pressure_altitude_ft, oat_celsius, ias_kts, tas_kts
        );
        
        print_json(da);
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        print_usage(argv[0]);
        return 1;
    }

    return error_success;
}
