/**
 * Density Altitude Calculator for X-Plane MFD
 * 
 * Calculates density altitude and related atmospheric parameters:
 * - Density altitude (how "high" the aircraft performs)
 * - Pressure altitude
 * - True vs Equivalent airspeed conversions
 * - Air density ratio (sigma)
 * - Performance degradation percentage
 * 
 * Compile: g++ -std=c++20 -O3 -o density_altitude_calculator density_altitude_calculator.cpp
 * 
 * Usage: ./density_altitude_calculator <pressure_alt_ft> <oat_celsius> <ias_kts> <tas_kts>
 */
#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <string_view>

namespace xplane_mfd::calc {

// Use constexpr for compile-time evaluation
constexpr double SEA_LEVEL_TEMP_C = 15.0;
constexpr double TEMP_LAPSE_RATE = 0.0019812;    // °C per foot (standard lapse rate)

[[nodiscard]] double parse_double(std::string_view sv) {
    return std::stod(std::string(sv));
}

struct DensityAltitudeData {
    double density_altitude_ft;      // Density altitude
    double pressure_altitude_ft;     // Pressure altitude (from setting)
    double air_density_ratio;        // σ (sigma) - ratio to sea level
    double temperature_deviation_c;  // Deviation from ISA
    double performance_loss_pct;     // % performance loss vs sea level
    double eas_kts;                  // Equivalent airspeed
    double tas_to_ias_ratio;         // TAS/IAS ratio
    double pressure_ratio;           // Pressure ratio vs sea level
};

/**
 * Calculate ISA temperature at given pressure altitude
 */
[[nodiscard]] constexpr double isa_temperature_c(double pressure_altitude_ft) {
    return SEA_LEVEL_TEMP_C - (TEMP_LAPSE_RATE * pressure_altitude_ft);
}

/**
 * Calculate density altitude using exact formula
 * 
 * DA = PA + [120 * (OAT - ISA)]
 * where 120 is an approximation factor
 * 
 * More precisely:
 * ρ = P / (R * T)
 * DA = altitude where standard atmosphere has same density as current conditions
 */
[[nodiscard]] double calculate_density_altitude(double pressure_altitude_ft, double oat_celsius) {
    // ISA temperature at pressure altitude
    double isa_temp = isa_temperature_c(pressure_altitude_ft);
    
    // Temperature deviation from ISA
    double temp_deviation = oat_celsius - isa_temp;
    
    // Density altitude approximation (good to about 1% accuracy)
    // DA = PA + [120 * (OAT - ISA)]
    double density_altitude = pressure_altitude_ft + (120.0 * temp_deviation);
    
    return density_altitude;
}

/**
 * Calculate air density ratio (sigma)
 * σ = ρ / ρ₀
 */
[[nodiscard]] double calculate_density_ratio(double pressure_altitude_ft, double oat_celsius) {
    // Convert to absolute temperature
    double temp_k = oat_celsius + 273.15;
    double sea_level_temp_k = SEA_LEVEL_TEMP_C + 273.15;
    
    // Pressure ratio (using standard atmosphere)
    // P/P₀ = (1 - 6.8756e-6 * h)^5.2559
    double pressure_ratio = std::pow(1.0 - 6.8756e-6 * pressure_altitude_ft, 5.2559);
    
    // Temperature ratio
    double temp_ratio = sea_level_temp_k / temp_k;
    
    // Density ratio: σ = (P/P₀) * (T₀/T)
    double sigma = pressure_ratio * temp_ratio;
    
    return sigma;
}

/**
 * Calculate Equivalent Airspeed (EAS)
 * EAS = TAS * sqrt(σ)
 */
[[nodiscard]] constexpr double calculate_eas(double tas_kts, double sigma) {
    return tas_kts * std::sqrt(sigma);
}

/**
 * Calculate complete density altitude data
 */
[[nodiscard]] DensityAltitudeData calculate_density_altitude_data(
    double pressure_altitude_ft,
    double oat_celsius,
    double ias_kts,
    double tas_kts
) {
    DensityAltitudeData result;
    
    result.pressure_altitude_ft = pressure_altitude_ft;
    result.density_altitude_ft = calculate_density_altitude(pressure_altitude_ft, oat_celsius);
    
    // ISA temperature at this altitude
    double isa_temp = isa_temperature_c(pressure_altitude_ft);
    result.temperature_deviation_c = oat_celsius - isa_temp;
    
    // Air density ratio
    result.air_density_ratio = calculate_density_ratio(pressure_altitude_ft, oat_celsius);
    
    // Performance loss (inverse of density ratio)
    // If σ = 0.8, you have 80% of sea level air density = 20% performance loss
    result.performance_loss_pct = (1.0 - result.air_density_ratio) * 100.0;
    
    // Equivalent airspeed
    result.eas_kts = calculate_eas(tas_kts, result.air_density_ratio);
    
    // TAS/IAS ratio (useful for quick mental calculations)
    if (ias_kts > 10.0) {
        result.tas_to_ias_ratio = tas_kts / ias_kts;
    } else {
        result.tas_to_ias_ratio = 1.0;
    }
    
    // Pressure ratio
    result.pressure_ratio = std::pow(1.0 - 6.8756e-6 * pressure_altitude_ft, 5.2559);
    
    return result;
}

/**
 * Output results as JSON
 */
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
              << " <pressure_alt_ft> <oat_celsius> <ias_kts> <tas_kts> [force_exception]\n\n";
    std::cerr << "Arguments:\n";
    std::cerr << "  pressure_alt_ft : Pressure altitude (feet)\n";
    std::cerr << "  oat_celsius     : Outside air temperature (°C)\n";
    std::cerr << "  ias_kts        : Indicated airspeed (knots)\n";
    std::cerr << "  tas_kts        : True airspeed (knots)\n";
    std::cerr << "  force_exception : Optional, 1 to trigger exception (default: 0)\n\n";
    std::cerr << "Example:\n";
    std::cerr << "  " << program_name << " 5000 25 150 170\n";
    std::cerr << "  (5000 ft PA, 25°C OAT, 150 kts IAS, 170 kts TAS)\n";
}

int main(int argc, char* argv[]) {
    using namespace xplane_mfd::calc;
    
    const std::vector<std::string_view> args(argv + 1, argv + argc);
    
    if (args.size() != 4 && args.size() != 5) {
        print_usage(argv[0]);
        return 1;
    }
    
    // ========================================================================
    // REMOVE BEFORE FLIGHT - Exception
    // ========================================================================
    try {
        double pressure_altitude_ft = parse_double(args[0]);
        double oat_celsius = parse_double(args[1]);
        double ias_kts = parse_double(args[2]);
        double tas_kts = parse_double(args[3]);
        
        // Check for force exception flag
        bool force_exception = false;
        if (args.size() == 5) {
            force_exception = (args[4] == "1" || args[4] == "true");
        }
        
        if (force_exception) {
            throw std::runtime_error("CRITICAL: Required dataref 'sim/weather/isa_deviation' not found in X-Plane API");
        }
        
        // Validate inputs
        if (pressure_altitude_ft < -2000 || pressure_altitude_ft > 60000) {
            std::cerr << "Warning: Pressure altitude outside typical range\n";
        }
        
        if (oat_celsius < -60 || oat_celsius > 60) {
            std::cerr << "Warning: Temperature outside typical range\n";
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
}
